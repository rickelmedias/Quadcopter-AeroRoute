/**
 * @file make_synthetic_capture.c
 * @brief Gera uma captura sintética cujos STEP trazem as saídas VERDADEIRAS
 *        dos estimadores, rodando o mesmo código de estimação do firmware.
 *
 * Existe para que o contrato do replay possa ser verificado sem gastar uma
 * sessão de bancada. Se o replay não reproduz uma captura gerada pelas
 * mesmíssimas funções, certamente não reproduz o veículo, e o defeito está no
 * formato ou no replay, não no hardware - muito mais barato de achar aqui.
 *
 * A simulação percorre de propósito os caminhos difíceis, não só o feliz: um
 * trecho com a IMU offline, um salto no relógio do sensor que torna um frame
 * inutilizável, frames com a distância inválida e movimento rápido o bastante
 * para o gate do filtro rejeitar algumas correções. Um gerador que só
 * produzisse frames limpos deixaria passar um defeito do replay em qualquer
 * um desses caminhos.
 *
 * Uso:
 *
 *     gcc -std=c11 -IApp/Inc tools/make_synthetic_capture.c \
 *         App/Src/disp_estimator.c App/Src/flow_ekf.c App/Src/attitude.c \
 *         -lm -o make_synthetic && ./make_synthetic > synthetic.bin
 *
 * A captura sai na saída padrão; o resumo sai na saída de erro.
 */
#include "capture.h"
#include "displacement.h"
#include "flow_ekf.h"
#include "attitude.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/** Conversão de "cm/s a 1 m" para rad/s (igual a App/Src/displacement.c). */
#define FLOW_CM_S_AT_1M_TO_RAD_S 0.01F
/** Quantidade de frames simulados. */
#define FRAMES       400U
/** Intervalo entre frames simulados, em ms. */
#define FRAME_MS     20U
/** pi em float. */
#define PI_F         3.14159265358979323846F

/** Buffer onde a captura é montada. */
static uint8_t out[262144];
/** Bytes já usados em @c out. */
static size_t used;
/** Próximo número de sequência. */
static uint16_t seq;

/**
 * @brief Fecha um registro e o incorpora à captura.
 * @param w  Registro montado por begin().
 */
static void put(cap_writer_t *w) {
  const uint16_t n = cap_finish(w);
  if (n == 0U) { fprintf(stderr, "record did not fit\n"); return; }
  used += n;
}
/**
 * @brief Abre um registro no fim da captura, consumindo um número de
 *        sequência.
 * @param type  Tipo do registro.
 * @param t_ms  Tick simulado de recepção.
 * @return O montador do registro (com capacidade zero se o buffer acabou).
 */
static cap_writer_t begin(cap_type_t type, uint32_t t_ms) {
  cap_writer_t w;
  /* CAP_MAX_RECORD, e não o espaço restante: o montador guarda a capacidade
     num uint16_t, e um buffer maior que 64 KB seria truncado na conversão -
     o que daria em silêncio uma capacidade zero em vez de uma grande. */
  if (used + CAP_MAX_RECORD > sizeof out) {
    fprintf(stderr, "output buffer exhausted\n");
    cap_begin(&w, out, 0U, type, seq, t_ms);
    return w;
  }
  cap_begin(&w, out + used, (uint16_t)CAP_MAX_RECORD, type, seq++, t_ms);
  return w;
}

/** Estado do gerador pseudoaleatório. Determinístico, para que uma falha
    seja reproduzível e uma correção possa ser conferida. É um LCG simples:
    o objetivo é oscilação repetível, não qualidade estatística. */
static uint32_t rng_state = 12345U;

/**
 * @brief Oscilação pseudoaleatória determinística.
 * @param amplitude  Amplitude máxima.
 * @return Valor em torno de [-amplitude, amplitude].
 */
static float jitter(float amplitude) {
  rng_state = rng_state * 1103515245U + 12345U;
  return amplitude * ((float)((rng_state >> 16) & 0x7FFFU) / 16383.5F - 1.0F);
}

/**
 * @brief Simula ::FRAMES frames e escreve a captura na saída padrão.
 * @return 0.
 */
int main(void) {
  disp_config_t cfg = {
    .flow_scale_x = 0.944F, .flow_scale_y = 0.942F,
    .gyro_sign_x = 1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F,
  };
  const flow_ekf_config_t ekf_cfg = FLOW_EKF_DEFAULTS;

  cap_writer_t w = begin(CAP_TYPE_SESSION, 0);
  cap_u8(&w, CAP_FORMAT_VERSION);
  cap_str(&w, "synthetic", CAP_FIRMWARE_ID_MAX);
  cap_f32(&w, cfg.flow_scale_x); cap_f32(&w, cfg.flow_scale_y);
  cap_f32(&w, cfg.gyro_sign_x);  cap_f32(&w, cfg.gyro_sign_y);
  cap_f32(&w, cfg.deadband_rad_s);
  cap_f32(&w, DISP_FLOW_LSB_RAD_S);
  cap_u32(&w, DISP_MAX_FRAME_GAP_MS);
  cap_u32(&w, DISP_MAX_CORRECTION_AGE_MS);
  cap_f32(&w, ekf_cfg.accel_noise);
  cap_f32(&w, ekf_cfg.flow_noise_rad_s);
  cap_f32(&w, ekf_cfg.gate_sigma);
  put(&w);

  disp_state_t raw = {0}, fused = {0};
  flow_ekf_t ekf = {0};
  uint32_t last_sensor_ms = 0, clock_resyncs = 0;
  bool have_previous = false;
  uint32_t rot_count = 0, gyro_count = 0, accel_count = 0;
  float q[4] = {1.0F, 0.0F, 0.0F, 0.0F};
  float gyro[3] = {0}, lin_accel[3] = {0};
  uint32_t sensor_ms = 100000U;
  unsigned steps = 0, skipped = 0;

  for (uint32_t i = 0; i < FRAMES; ++i) {
    const uint32_t tick = 1000U + i * FRAME_MS;
    const float time_s = (float)i * 0.001F * (float)FRAME_MS;

    /* Um trecho com a IMU offline, para a predição seguir com aceleração zero
       e o FUSED rejeitar - um caminho que o caso feliz nunca alcança. */
    const bool imu_online = !(i >= 150U && i < 180U);

    /* Salto no relógio do sensor: o firmware descarta este frame e reancora. */
    if (i == 250U) sensor_ms += 5000U;
    else if (i > 0U) sensor_ms += FRAME_MS;

    /* Atitude: uma varredura lenta de yaw com um pouco de rolagem, mantida na
       esfera unitária para que a verificação do quatérnio no firmware a
       aceitasse. */
    const float yaw = 0.35F * sinf(0.7F * time_s);
    const float roll = 0.12F * sinf(1.9F * time_s);
    q[0] = cosf(yaw * 0.5F) * cosf(roll * 0.5F);
    q[1] = cosf(yaw * 0.5F) * sinf(roll * 0.5F);
    q[2] = 0.0F;
    q[3] = sinf(yaw * 0.5F) * cosf(roll * 0.5F);

    gyro[0] = 0.12F * 1.9F * cosf(1.9F * time_s) + jitter(0.01F);
    gyro[1] = 0.08F * cosf(1.3F * time_s) + jitter(0.01F);
    gyro[2] = 0.35F * 0.7F * cosf(0.7F * time_s);
    lin_accel[0] = 0.4F * cosf(0.9F * time_s) + jitter(0.05F);
    lin_accel[1] = 0.3F * sinf(1.1F * time_s) + jitter(0.05F);
    lin_accel[2] = jitter(0.05F);

    /* Translação mais a parte da rotação do corpo que o estimador precisa
       remover, depois quantizada nos passos de 20 contagens em que o MTF-01
       real informa. Uma rajada de movimento rápido perto do frame 300 aciona
       o gate de inovação. */
    const float boost = (i >= 300U && i < 320U) ? 6.0F : 1.0F;
    const float trans_x = boost * 0.25F * sinf(1.4F * time_s);
    const float trans_y = boost * 0.20F * cosf(1.1F * time_s);
    const float flow_x_rad = trans_x + cfg.gyro_sign_x * gyro[0] * -1.0F;
    const float flow_y_rad = trans_y + cfg.gyro_sign_y * gyro[1] * -1.0F;
    const int16_t flow_x = (int16_t)(20 * (int)lrintf(flow_x_rad * 100.0F / 20.0F));
    const int16_t flow_y = (int16_t)(20 * (int)lrintf(flow_y_rad * 100.0F / 20.0F));

    const uint32_t distance_mm = 800U + (uint32_t)(50.0F * (1.0F + sinf(0.5F * time_s)));
    const bool range_valid = !(i >= 60U && i < 70U); /* uma breve perda da distância */
    const uint8_t quality = (uint8_t)(range_valid ? 90 : 10);

    /* Primeiro o BNO, depois o MTF, depois o STEP que os consumiu - a ordem em
       que as tasks do firmware os produzem. */
    ++rot_count; ++gyro_count; ++accel_count;
    const uint64_t meas_us = (uint64_t)tick * 1000U;
    struct { cap_bno_report_t r; const float *v; uint32_t c; uint32_t off; } reports[3] = {
      { CAP_BNO_ROTATION, q, rot_count, 0U },
      { CAP_BNO_GYRO, gyro, gyro_count, 2500U },
      { CAP_BNO_ACCEL, lin_accel, accel_count, 5000U },
    };
    for (unsigned k = 0; k < 3U; ++k) {
      w = begin(CAP_TYPE_BNO, tick);
      cap_u8(&w, (uint8_t)reports[k].r);
      cap_u8(&w, 3U);
      cap_u8(&w, (uint8_t)(i & 0xFFU));
      for (unsigned c = 0; c < 4U; ++c)
        cap_f32(&w, (reports[k].r == CAP_BNO_ROTATION || c < 3U) ? reports[k].v[c] : 0.0F);
      cap_u32(&w, reports[k].c);
      cap_u64(&w, meas_us + reports[k].off);
      cap_u32(&w, 2500U);
      put(&w);
    }

    w = begin(CAP_TYPE_MTF, tick);
    cap_i16(&w, flow_x); cap_i16(&w, flow_y);
    cap_u32(&w, distance_mm);
    cap_u8(&w, quality); cap_u8(&w, 0U); cap_u8(&w, 120U); cap_u8(&w, 3U);
    cap_u8(&w, (uint8_t)((range_valid ? 1U : 0U) | 2U));
    cap_u32(&w, sensor_ms);
    cap_u32(&w, i + 1U);
    put(&w);

    /* Daqui para baixo, é o espelho exato de App/Src/displacement.c. */
    const uint32_t gap_ms = sensor_ms - last_sensor_ms;
    const bool gap_usable = have_previous && disp_frame_gap_usable(gap_ms);
    w = begin(CAP_TYPE_OBSERVATION, tick);
    cap_u32(&w, i + 1U); cap_u32(&w, last_sensor_ms);
    cap_u8(&w, (uint8_t)((have_previous ? 1U : 0U) | (gap_usable ? 2U : 0U) |
                         (range_valid ? 4U : 0U) | 8U));
    put(&w);
    if (!gap_usable) {
      ++clock_resyncs;
      ++skipped;
      w = begin(CAP_TYPE_DIAG, tick);
      cap_u32(&w, 0U); cap_u32(&w, 0U); cap_u32(&w, 0U); cap_u32(&w, clock_resyncs);
      cap_u32(&w, 12048U); cap_u32(&w, 14096U); cap_u32(&w, 624U);
      cap_str(&w, "Displace", 12U);
      put(&w);
    } else {
      disp_input_t in = {0};
      in.flow_x = (float)flow_x * FLOW_CM_S_AT_1M_TO_RAD_S;
      in.flow_y = (float)flow_y * FLOW_CM_S_AT_1M_TO_RAD_S;
      in.gyro_x = gyro[0];
      in.gyro_y = gyro[1];
      for (int c = 0; c < 4; ++c) in.q[c] = q[c];
      in.range_m = (float)distance_mm * 0.001F;
      in.dt_s = (float)gap_ms * 0.001F;
      in.quality = quality;
      in.range_valid = range_valid && true && imu_online;

      disp_raw_update(&raw, &in, &cfg);
      const bool fused_ok = disp_fused_update(&fused, &in, &cfg);

      float accel_world[2] = {0.0F, 0.0F};
      if (imu_online) attitude_rotate_to_world(in.q, lin_accel, accel_world);
      flow_ekf_predict(&ekf, accel_world[0], accel_world[1], in.dt_s, &ekf_cfg);
      bool ekf_ok = false;
      if (fused_ok && flow_ekf_update(&ekf, fused.vx, fused.vy,
                                      in.range_m, &ekf_cfg)) {
        ekf_ok = true;
      }

      w = begin(CAP_TYPE_STEP, tick);
      cap_u32(&w, i + 1U);
      cap_u32(&w, rot_count); cap_u32(&w, gyro_count); cap_u32(&w, accel_count);
      cap_f32(&w, in.dt_s);
      cap_u8(&w, (uint8_t)((fused_ok ? 1U : 0U) | (ekf_ok ? 2U : 0U) |
                           (imu_online ? 4U : 0U)));
      cap_f32(&w, raw.x); cap_f32(&w, raw.y);
      cap_f32(&w, fused.vx); cap_f32(&w, fused.vy);
      cap_f32(&w, ekf.axis[0].v); cap_f32(&w, ekf.axis[1].v);
      cap_f32(&w, ekf.axis[0].p); cap_f32(&w, ekf.axis[1].p);
      put(&w);
      ++steps;
    }
    last_sensor_ms = sensor_ms;
    have_previous = true;
  }

  fwrite(out, 1, used, stdout);
  fprintf(stderr,
          "%u frames -> %u steps, %u skipped, %zu bytes\n"
          "filter: %u updates, %u gated out, %u resets, converged=%d\n",
          FRAMES, steps, skipped, used,
          ekf.updates, ekf.rejected, ekf.resets, (int)ekf.converged);
  return 0;
}
