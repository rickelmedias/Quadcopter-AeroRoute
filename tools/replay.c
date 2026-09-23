/**
 * @file replay.c
 * @brief Reexecuta uma captura binária com o mesmo código de estimação do
 *        firmware e confere se reproduz o que o firmware realmente fez.
 *
 * É a porta que precisa ser vencida antes de qualquer mudança de algoritmo
 * valer discussão. Se o PC não reproduz a execução embarcada a partir de uma
 * gravação, um experimento no PC mostrando que "a nova regra de fluxo zero é
 * melhor" não diz nada sobre o veículo - só descreve outro programa.
 *
 * São conferidas três coisas, em ordem crescente de rigor:
 *
 *   1. PAREAMENTO. Cada STEP busca, num histórico limitado, as amostras pelos
 *      IDs gravados. Os registros OBSERVATION reconstroem a linha do tempo da
 *      task, independentemente da chegada dos sensores e da preempção.
 *   2. DECISÕES. Aceitar/rejeitar no FUSED e no gate do filtro precisam bater
 *      exatamente. São booleanos: "perto" não existe, e uma única troca
 *      significa que as duas execuções divergiram.
 *   3. NÚMEROS. Posições e velocidades precisam concordar dentro de uma
 *      tolerância, e o pior desvio visto é informado em vez de escondido, para
 *      que um afastamento lento não passe por sucesso.
 *
 * Os fontes dos estimadores são ligados diretamente - disp_estimator.c,
 * flow_ekf.c, attitude.c -, então isto exercita o código de produção, e não
 * uma descrição dele.
 *
 * A configuração vem do registro SESSION da captura, nunca dos padrões
 * compilados aqui: uma gravação feita com outra calibração precisa ser
 * reexecutada com aquela calibração.
 *
 * Cada captura é de um único boot com configuração fixa. Reinícios ou
 * mudanças de configuração em tempo de execução exigem outra sessão; este
 * programa não os trata.
 *
 * Uso:
 *
 *     gcc -std=c11 -IApp/Inc tools/replay.c App/Src/disp_estimator.c \
 *         App/Src/flow_ekf.c App/Src/attitude.c -lm -o replay
 *     ./replay capture.bin
 *
 * Código de saída: 0 = REPRODUCED; 1 = DIVERGED, INCOMPLETE ou nada
 * reexecutado; 2 = argumento ou arquivo inválido.
 */
#include "capture.h"
#include "displacement.h"
#include "flow_ekf.h"
#include "attitude.h"
#include "bno_history.h"
#include "range_filter.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Conversão de "cm/s a 1 m" para rad/s; precisa ser igual à de
    App/Src/displacement.c. */
#define FLOW_CM_S_AT_1M_TO_RAD_S 0.01F

/** Tolerância numérica da comparação. Velocidades chegam a poucos m/s e
    posições a poucos metros, então isto fica bem abaixo de qualquer coisa que
    mudaria uma conclusão, e bem acima do ruído do último bit ao reagrupar a
    mesma aritmética em outra FPU. */
#define TOLERANCE 1e-4F

/* ---- leitura ----------------------------------------------------------- */

/**
 * @brief Cursor de leitura sobre a carga de um registro.
 */
typedef struct {
  /** Bytes da carga. */
  const uint8_t *b;
  /** Tamanho da carga. */
  uint16_t len;
  /** Próximo byte a ler. */
  uint16_t pos;
  /** Uma leitura passou do fim; os valores lidos não valem. */
  bool bad;
} rd_t;

/** @brief Lê um u8; passar do fim marca @c bad e devolve 0. */
static uint8_t rd_u8(rd_t *r) {
  if (r->pos + 1U > r->len) { r->bad = true; return 0; }
  return r->b[r->pos++];
}
/** @brief Lê um u16 little-endian. */
static uint16_t rd_u16(rd_t *r) { uint16_t lo = rd_u8(r); return (uint16_t)(lo | (rd_u8(r) << 8)); }
/** @brief Lê um u32 little-endian. */
static uint32_t rd_u32(rd_t *r) { uint32_t lo = rd_u16(r); return lo | ((uint32_t)rd_u16(r) << 16); }
/** @brief Lê um u64 little-endian. */
static uint64_t rd_u64(rd_t *r) { uint64_t lo = rd_u32(r); return lo | ((uint64_t)rd_u32(r) << 32); }
/** @brief Lê um i16 little-endian. */
static int16_t rd_i16(rd_t *r) { return (int16_t)rd_u16(r); }
/** @brief Lê um f32 IEEE-754 little-endian. */
static float rd_f32(rd_t *r) { uint32_t b = rd_u32(r); float f; memcpy(&f, &b, sizeof f); return f; }

/**
 * @brief Lê uma string precedida do tamanho, truncando para caber no destino.
 * @param r    Cursor.
 * @param out  Destino (sempre terminado em zero).
 * @param cap  Tamanho do destino.
 */
static void rd_str(rd_t *r, char *out, uint8_t cap) {
  const uint8_t n = rd_u8(r);
  for (uint8_t i = 0; i < n; ++i) { const uint8_t c = rd_u8(r); if (i + 1U < cap) out[i] = (char)c; }
  out[n + 1U < cap ? n : (cap ? cap - 1U : 0U)] = '\0';
}

/**
 * @brief Diz se o tamanho combina com o tipo do registro.
 *
 * Espelha tools/decode_capture.py: o formato é conferido antes do checksum, e
 * toda rejeição avança um byte em vez de abandonar o resto do arquivo.
 *
 * @param type  Tipo do registro.
 * @param len   Tamanho da carga.
 * @return true se o tamanho é possível para esse tipo.
 */
static bool plausible(uint8_t type, uint8_t len) {
  switch (type) {
    case CAP_TYPE_MTF:     return len == 21U;
    case CAP_TYPE_BNO:     return len == 35U;
    case CAP_TYPE_STEP:    return len == 53U;
    case CAP_TYPE_OBSERVATION: return len == 9U;
    case CAP_TYPE_SESSION: return len >= 50U;
    case CAP_TYPE_DIAG:    return len >= 29U;
    default:               return false;
  }
}

/* ---- estado reexecutado ------------------------------------------------ */

/**
 * @brief Estado da reexecução: a última fotografia observada e os
 *        estimadores.
 *
 * Os campos de sensor espelham o que bno_get_sample()/mtf_get_sample()
 * entregam ao firmware: a amostra mais recente, sem alinhamento nem
 * interpolação.
 */
typedef struct {
  /** Fluxo X do frame observado, em cm/s a 1 m. */
  int16_t flow_x;
  /** Fluxo Y do frame observado, em cm/s a 1 m. */
  int16_t flow_y;
  /** Distância do frame observado, em mm. */
  uint32_t distance_mm;
  /** Relógio do sensor no frame observado, em ms. */
  uint32_t sensor_ms;
  /** Horário de recepção do frame MTF no STM32, em us. */
  uint32_t received_us;
  /** ID do frame observado. */
  uint32_t frames;
  /** Distância válida na observação. */
  bool range_valid;
  /** Fluxo válido na observação. */
  bool flow_valid;
  /** Qualidade da imagem do frame observado. */
  uint8_t quality;
  /** Algum frame já foi observado. */
  bool mtf_seen;

  /** Quatérnio usado no passo atual (w,x,y,z). */
  float q[4];
  /** Giro usado no passo atual, em rad/s. */
  float gyro[3];
  /** Aceleração linear usada no passo atual, em m/s^2. */
  float lin_accel[3];
  /** Não usado desde o formato 3 (as amostras vêm do histórico por ID). */
  uint32_t rot_count;
  /** Não usado desde o formato 3. */
  uint32_t gyro_count;
  /** Não usado desde o formato 3. */
  uint32_t accel_count;

  /** Estimador RAW (mesmo tipo do firmware). */
  disp_state_t raw;
  /** Estimador FUSED (mesmo tipo do firmware). */
  disp_state_t fused;
  /** Filtro de Kalman (mesmo tipo do firmware). */
  flow_ekf_t ekf;
  range_filter_t range_filter;
  /** Calibração lida da SESSION. */
  disp_config_t cfg;
  /** Parâmetros do filtro lidos da SESSION. */
  flow_ekf_config_t ekf_cfg;

  /** Relógio do sensor na observação anterior, em ms. */
  uint32_t last_sensor_ms;
  /** Já houve uma observação anterior. */
  bool have_previous;
} replay_t;

/** Tamanho de cada histórico de amostras. Uma amostra ausente ou já
    sobrescrita nunca é trocada por uma mais nova: nesse caso a captura não
    consegue provar a reprodução. */
#define HISTORY 8192U

/**
 * @brief Um frame do MTF guardado no histórico.
 */
typedef struct {
  /** Posição ocupada. */
  bool present;
  /** ID do frame (contador de frames). */
  uint32_t id;
  /** Relógio do sensor, em ms. */
  uint32_t sensor_ms;
  /** Horário do registro MTF, equivalente a mtf_sample_t.received_us. */
  uint32_t received_us;
  /** Distância, em mm. */
  uint32_t distance_mm;
  /** Fluxo X, em cm/s a 1 m. */
  int16_t fx;
  /** Fluxo Y, em cm/s a 1 m. */
  int16_t fy;
  /** Qualidade da imagem. */
  uint8_t quality;
  /** Distância válida segundo o parser. */
  bool range_valid;
  /** Fluxo válido segundo o parser. */
  bool flow_valid;
} mtf_history_t;

/**
 * @brief Um relatório do BNO guardado no histórico.
 */
typedef struct {
  /** Posição ocupada. */
  bool present;
  /** ID da amostra (contador do relatório). */
  uint32_t id;
  /** Os quatro valores gravados. */
  float v[4];
  /** Timestamp SH-2 na linha de tempo do STM32. */
  uint32_t time_us;
} captured_bno_history_t;

/** Histórico de frames do MTF, indexado por ID módulo ::HISTORY. */
static mtf_history_t mtf_history[HISTORY];
/** Históricos do BNO (rotação, giro, aceleração), indexados por ID. */
static captured_bno_history_t captured_bno[3][HISTORY];
/** Mesmo histórico temporal puro usado pelo firmware. */
static bno_history_t imu_history;
/** Uma observação com intervalo utilizável aguarda o seu STEP. */
static bool observation_pending;
/** ID do frame da observação pendente. */
static uint32_t observation_frames;
/** Intervalo da observação pendente, em ms. */
static uint32_t observation_gap;

/**
 * @brief Busca uma amostra do BNO pelo ID.
 *
 * @param kind  0 = rotação, 1 = giro, 2 = aceleração.
 * @param id    ID da amostra; 0 significa "nenhuma ainda" e devolve zeros.
 * @param out   Destino.
 * @param n     Quantos valores copiar.
 * @return false se a amostra não está no histórico.
 */
static bool select_bno(unsigned kind, uint32_t id, float *out, unsigned n) {
  if (!id) { for (unsigned i=0; i<n; ++i) out[i]=0; return true; }
  const captured_bno_history_t *h = &captured_bno[kind][id % HISTORY];
  if (!h->present || h->id != id) return false;
  for (unsigned i=0; i<n; ++i) out[i]=h->v[i];
  return true;
}

/**
 * @brief Contadores do resultado da reexecução.
 */
typedef struct {
  /** STEPs reexecutados. */
  unsigned steps;
  /** STEPs cujas amostras não puderam ser encontradas. */
  unsigned pairing_fail;
  /** STEPs com decisão diferente. */
  unsigned decision_fail;
  /** Valores fora da tolerância ou não finitos. */
  unsigned numeric_fail;
  /** Maior desvio visto. */
  float worst;
  /** Nome do campo com o maior desvio. */
  const char *worst_field;
  /** Passos sem cobertura temporal suficiente no histórico do BNO. */
  unsigned alignment_fail;
} tally_t;

/**
 * @brief Compara um valor reexecutado com o gravado e atualiza os contadores.
 *
 * Valores não finitos em qualquer lado contam como falha. Só as 10 primeiras
 * falhas de tolerância são impressas.
 *
 * @param t      Contadores.
 * @param field  Nome do campo, para a mensagem.
 * @param got    Valor reexecutado.
 * @param want   Valor gravado.
 */
static void note(tally_t *t, const char *field, float got, float want) {
  if (!isfinite(got) || !isfinite(want)) {
    ++t->numeric_fail;
    printf("    %s: non-finite value in replay or recording\n", field);
    return;
  }
  const float d = fabsf(got - want);
  if (d > t->worst) { t->worst = d; t->worst_field = field; }
  if (d > TOLERANCE) {
    ++t->numeric_fail;
    if (t->numeric_fail <= 10U)
      printf("    %-10s replay %+.6f  recorded %+.6f  diff %.3e\n",
             field, (double)got, (double)want, (double)d);
  }
}

/**
 * @brief Reexecuta um STEP e confere pareamento, decisões e números.
 *
 * @param s    Estado da reexecução.
 * @param r    Cursor sobre a carga do STEP.
 * @param t    Contadores.
 * @param seq  Número de sequência do registro, para as mensagens.
 */
static void do_step(replay_t *s, rd_t *r, tally_t *t, uint16_t seq,
                    uint32_t step_ms) {
  const uint32_t used_frames = rd_u32(r);
  const uint32_t used_rot = rd_u32(r), used_gyro = rd_u32(r), used_accel = rd_u32(r);
  const float rec_dt = rd_f32(r);
  const uint8_t flags = rd_u8(r);
  const bool rec_fused_ok = (flags & 1U) != 0U;
  const bool rec_ekf_ok = (flags & 2U) != 0U;
  const bool imu_online = (flags & 4U) != 0U;
  const float rec_raw_px = rd_f32(r), rec_raw_py = rd_f32(r);
  const float rec_fvx = rd_f32(r), rec_fvy = rd_f32(r);
  const float rec_evx = rd_f32(r), rec_evy = rd_f32(r);
  const float rec_epx = rd_f32(r), rec_epy = rd_f32(r);
  if (r->bad) return;

  ++t->steps;

  /* 1. Pareamento. Qualquer outra coisa invalida a comparação que vem a
        seguir, então isso é dito em voz alta e a execução continua, em vez de
        comparar em silêncio o par errado. */
  if (!observation_pending || observation_frames != used_frames ||
      !select_bno(0, used_rot, s->q, 4) ||
      !select_bno(1, used_gyro, s->gyro, 3) ||
      !select_bno(2, used_accel, s->lin_accel, 3)) {
    ++t->pairing_fail;
    printf("  seq %u: pairing differs - observed mtf=%u pending=%d; "
           "requested mtf=%u bno=%u/%u/%u (missing/evicted ID or missing observation)\n",
           seq, observation_frames, observation_pending,
           used_frames, used_rot, used_gyro, used_accel);
    return;
  }
  observation_pending = false;

  /* O STEP identifica as amostras disponíveis, mas o valor consumido agora é
     a associação temporal calculada com o mesmo histórico do firmware. */
  bno_aligned_sample_t aligned;
  (void)step_ms;
  const bool imu_aligned = imu_online && bno_history_align(
      &imu_history, s->received_us-DISP_MTF_DELAY_US,
      observation_gap*1000U, &aligned);
  if (!imu_aligned) ++t->alignment_fail;
  if (imu_aligned) {
    memcpy(s->q, aligned.q, sizeof aligned.q);
    memcpy(s->gyro, aligned.gyro, sizeof aligned.gyro);
    memcpy(s->lin_accel, aligned.accel, sizeof aligned.accel);
  } else {
    memset(s->q, 0, sizeof s->q);
    memset(s->gyro, 0, sizeof s->gyro);
    memset(s->lin_accel, 0, sizeof s->lin_accel);
  }

  /* Mesma montagem de App/Src/displacement.c, campo a campo. */
  const uint32_t gap_ms = observation_gap;
  disp_input_t in = {0};
  in.flow_x = (float)s->flow_x * FLOW_CM_S_AT_1M_TO_RAD_S;
  in.flow_y = (float)s->flow_y * FLOW_CM_S_AT_1M_TO_RAD_S;
  in.gyro_x = s->gyro[0];
  in.gyro_y = s->gyro[1];
  in.gyro_z = s->gyro[2];
  for (int i = 0; i < 4; ++i) in.q[i] = s->q[i];
  in.dt_s = (float)gap_ms * 0.001F;
  float filtered_range=0.0F;
  const bool range_filtered=range_filter_update(&s->range_filter,
      (float)s->distance_mm*0.001F,in.dt_s,s->range_valid,&filtered_range);
  in.range_m=range_filtered?filtered_range:(float)s->distance_mm*0.001F;
  in.quality = s->quality;
  in.range_valid = range_filtered && s->flow_valid && imu_aligned;

  note(t, "dt_s", in.dt_s, rec_dt);

  disp_raw_update(&s->raw, &in, &s->cfg);
  const bool fused_ok = disp_fused_update(&s->fused, &in, &s->cfg);

  float accel_local[2] = {0.0F, 0.0F};
  if (imu_aligned && s->fused.yaw_origin_set)
    attitude_rotate_to_local(in.q, s->fused.yaw_origin,
                             s->lin_accel, accel_local);
  flow_ekf_predict(&s->ekf, accel_local[0], accel_local[1], in.dt_s, &s->ekf_cfg);
  bool ekf_ok = false;
  if (fused_ok && flow_ekf_update_observation(
      &s->ekf, s->fused.vx, s->fused.vy, in.range_m, in.quality,
      s->fused.quant_half_width[0], s->fused.quant_half_width[1],
      &s->ekf_cfg)) {
    ekf_ok = true;
  }

  /* 2. Decisões. */
  if (fused_ok != rec_fused_ok || ekf_ok != rec_ekf_ok) {
    ++t->decision_fail;
    printf("  seq %u: decision differs - replay fused=%d ekf=%d, "
           "firmware fused=%d ekf=%d\n", seq, fused_ok, ekf_ok,
           rec_fused_ok, rec_ekf_ok);
  }

  /* 3. Números. */
  note(t, "raw.x", s->raw.x, rec_raw_px);
  note(t, "raw.y", s->raw.y, rec_raw_py);
  note(t, "fused.vx", s->fused.vx, rec_fvx);
  note(t, "fused.vy", s->fused.vy, rec_fvy);
  note(t, "ekf.vx", s->ekf.axis[0].v, rec_evx);
  note(t, "ekf.vy", s->ekf.axis[1].v, rec_evy);
  note(t, "ekf.px", s->ekf.axis[0].p, rec_epx);
  note(t, "ekf.py", s->ekf.axis[1].p, rec_epy);
}

/**
 * @brief Lê a captura, reexecuta todos os STEPs e imprime o resumo.
 *
 * @param argc  Deve ser 2.
 * @param argv  argv[1] = caminho da captura.
 * @return 0 se reproduziu; 1 se divergiu, ficou incompleto ou não havia
 *         STEP; 2 para uso ou arquivo inválido.
 */
int main(int argc, char **argv) {
  if (argc != 2) { fprintf(stderr, "usage: %s capture.bin\n", argv[0]); return 2; }
  FILE *f = fopen(argv[1], "rb");
  if (!f) { perror(argv[1]); return 2; }
  fseek(f, 0, SEEK_END);
  long size = ftell(f);
  if (size <= 0) { fprintf(stderr, "empty capture\n"); fclose(f); return 2; }
  rewind(f);
  uint8_t *blob = malloc((size_t)size);
  if (!blob || fread(blob, 1, (size_t)size, f) != (size_t)size) {
    fprintf(stderr, "read failed\n"); fclose(f); free(blob); return 2;
  }
  fclose(f);

  replay_t s = {0};
  s.q[0] = 1.0F;
  tally_t t = {0};
  bool have_session = false;
  unsigned seq_gaps = 0, skipped = 0;
  bool losses_reported = false;
  unsigned observations = 0;
  long last_seq = -1;
  const char *stop_reason = NULL;
  long stop_offset = 0;
  unsigned stop_seq = 0;
  uint32_t stop_ms = 0;
  unsigned verified_prefix_steps = 0, last_verified_seq = 0;
  /* Interrompe a leitura no registro atual (posição p, sequência seq),
     guardando o motivo, e vai direto para o resumo. */
#define STOP_REPLAY(reason) do { \
    stop_reason = (reason); stop_offset = p; stop_seq = seq; \
    stop_ms = (uint32_t)blob[p+5] | (uint32_t)blob[p+6]<<8 | \
              (uint32_t)blob[p+7]<<16 | (uint32_t)blob[p+8]<<24; \
    goto replay_summary; \
  } while (0)

  for (long p = 0; p + (long)CAP_OVERHEAD <= size; ) {
    if (blob[p] != CAP_SYNC) { ++p; ++skipped; continue; }
    const uint8_t type = blob[p + 1], len = blob[p + 2];
    const long rec = (long)len + (long)CAP_OVERHEAD;
    if (!plausible(type, len) || p + rec > size ||
        cap_crc8(blob + p, (uint16_t)(rec - 1)) != blob[p + rec - 1]) {
      ++p; ++skipped; continue;
    }
    const uint16_t seq = (uint16_t)(blob[p + 3] | (blob[p + 4] << 8));
    if (last_seq >= 0 && (uint16_t)(seq - (uint16_t)last_seq) != 1U) ++seq_gaps;
    last_seq = seq;
    const uint32_t record_ms = (uint32_t)blob[p+5] |
        (uint32_t)blob[p+6]<<8 | (uint32_t)blob[p+7]<<16 |
        (uint32_t)blob[p+8]<<24;

    rd_t r = { blob + p + CAP_HEADER_BYTES, len, 0, false };
    switch (type) {
      case CAP_TYPE_SESSION: {
        if (have_session) STOP_REPLAY("multiple sessions: split capture first");
        const uint8_t version = rd_u8(&r);
        if (version != CAP_FORMAT_VERSION) {
          fprintf(stderr, "capture is format version %u, this replay reads %u\n",
                  version, (unsigned)CAP_FORMAT_VERSION);
          STOP_REPLAY("unsupported capture version");
        }
        char fw[CAP_FIRMWARE_ID_MAX + 1];
        rd_str(&r, fw, sizeof fw);
        s.cfg.flow_scale_x = rd_f32(&r);
        s.cfg.flow_scale_y = rd_f32(&r);
        s.cfg.gyro_sign_x = rd_f32(&r);
        s.cfg.gyro_sign_y = rd_f32(&r);
        s.cfg.deadband_rad_s = rd_f32(&r);
        const float flow_lsb = rd_f32(&r);
        const uint32_t max_gap = rd_u32(&r), max_age = rd_u32(&r);
        if (!isfinite(flow_lsb) || flow_lsb != DISP_FLOW_LSB_RAD_S ||
            max_gap != DISP_MAX_FRAME_GAP_MS || max_age != DISP_MAX_CORRECTION_AGE_MS) {
          STOP_REPLAY("SESSION constants differ from replay build");
        }
        s.ekf_cfg.accel_noise = rd_f32(&r);
        s.ekf_cfg.flow_noise_rad_s = rd_f32(&r);
        s.ekf_cfg.gate_sigma = rd_f32(&r);
        if (r.bad || r.pos != r.len || !isfinite(s.cfg.flow_scale_x) ||
            !isfinite(s.cfg.flow_scale_y) || !isfinite(s.cfg.gyro_sign_x) ||
            !isfinite(s.cfg.gyro_sign_y) || !isfinite(s.cfg.deadband_rad_s) ||
            !isfinite(s.ekf_cfg.accel_noise) || !isfinite(s.ekf_cfg.flow_noise_rad_s) ||
            !isfinite(s.ekf_cfg.gate_sigma) || s.cfg.flow_scale_x <= 0 ||
            s.cfg.flow_scale_y <= 0 || s.ekf_cfg.flow_noise_rad_s <= 0 ||
            s.ekf_cfg.accel_noise < 0 || s.ekf_cfg.gate_sigma <= 0) {
          STOP_REPLAY("invalid SESSION configuration");
        }
        have_session = true;
        printf("firmware %s\n", fw);
        printf("scale %.4f/%.4f  gyro sign %+.0f/%+.0f  deadband %.4f\n",
               (double)s.cfg.flow_scale_x, (double)s.cfg.flow_scale_y,
               (double)s.cfg.gyro_sign_x, (double)s.cfg.gyro_sign_y,
               (double)s.cfg.deadband_rad_s);
        printf("ekf accel_noise %.3f  flow_noise %.3f  gate %.1f sigma\n\n",
               (double)s.ekf_cfg.accel_noise, (double)s.ekf_cfg.flow_noise_rad_s,
               (double)s.ekf_cfg.gate_sigma);
        break;
      }
      case CAP_TYPE_MTF: {
        const int16_t fx = rd_i16(&r), fy = rd_i16(&r);
        const uint32_t dist = rd_u32(&r);
        const uint8_t qual = rd_u8(&r);
        (void)rd_u8(&r); (void)rd_u8(&r); (void)rd_u8(&r);
        const uint8_t flags = rd_u8(&r);
        const uint32_t sensor_ms = rd_u32(&r);
        const uint32_t frames = rd_u32(&r);
        if (r.bad) break;
        /* A aquisição é independente das observações da task. Só guarda, sem
           avançar o carimbo de frame anterior do estimador. */
        mtf_history[frames % HISTORY] = (mtf_history_t){
          .present=true, .id=frames, .sensor_ms=sensor_ms,
          .received_us=record_ms*1000U, .distance_mm=dist,
          .fx=fx, .fy=fy, .quality=qual,
          .range_valid=(flags & 1U)!=0, .flow_valid=(flags & 2U)!=0};
        break;
      }
      case CAP_TYPE_BNO: {
        const uint8_t report = rd_u8(&r);
        (void)rd_u8(&r); (void)rd_u8(&r);
        float v[4]; for (int i = 0; i < 4; ++i) v[i] = rd_f32(&r);
        const uint32_t count = rd_u32(&r);
        const uint64_t sensor_us = rd_u64(&r); (void)rd_u32(&r);
        if (report < 1 || report > 3) STOP_REPLAY("invalid BNO kind");
        captured_bno_history_t *h = &captured_bno[report-1][count % HISTORY];
        h->present=true; h->id=count;
        h->time_us=(uint32_t)sensor_us;
        for (unsigned i=0;i<4;++i) h->v[i]=v[i];
        if (report == 1U) bno_history_push_rotation(&imu_history,(uint32_t)sensor_us,count,v);
        else if (report == 2U) bno_history_push_gyro(&imu_history,(uint32_t)sensor_us,count,v);
        else bno_history_push_accel(&imu_history,(uint32_t)sensor_us,count,v);
        break;
      }
      case CAP_TYPE_OBSERVATION: {
        const uint32_t id = rd_u32(&r), prev_ms = rd_u32(&r);
        const uint8_t flags = rd_u8(&r);
        const mtf_history_t *h = &mtf_history[id % HISTORY];
        if (!have_session || !h->present || h->id != id || observation_pending ||
            ((flags & 1U)!=0) != s.have_previous || prev_ms != s.last_sensor_ms) {
          STOP_REPLAY("missing sample or inconsistent observation timeline");
        }
        observation_gap = h->sensor_ms - prev_ms;
        const bool usable = s.have_previous && disp_frame_gap_usable(observation_gap);
        if (usable != ((flags & 2U)!=0)) STOP_REPLAY("gap decision differs");
        s.flow_x=h->fx; s.flow_y=h->fy; s.distance_mm=h->distance_mm;
        s.quality=h->quality; s.frames=id; s.sensor_ms=h->sensor_ms;
        s.received_us=h->received_us;
        s.range_valid=(flags & 4U)!=0; s.flow_valid=(flags & 8U)!=0;
        s.last_sensor_ms=h->sensor_ms; s.have_previous=true; s.mtf_seen=true;
        observation_frames=id; observation_pending=usable;
        ++observations;
        break;
      }
      case CAP_TYPE_DIAG: {
        const uint32_t full=rd_u32(&r), oversize=rd_u32(&r), contention=rd_u32(&r);
        losses_reported |= full != 0 || oversize != 0 || contention != 0;
        break;
      }
      case CAP_TYPE_STEP:
        if (!have_session) {
          fprintf(stderr, "STEP before SESSION: the configuration behind these "
                          "numbers is unknown, refusing to guess\n");
          STOP_REPLAY("STEP before SESSION");
        }
        do_step(&s, &r, &t, seq, record_ms);
        if (!t.pairing_fail && !t.decision_fail && !t.numeric_fail &&
            !seq_gaps && !skipped && !losses_reported) {
          ++verified_prefix_steps;
          last_verified_seq = seq;
        }
        break;
      default: break;
    }
    p += rec;
  }
replay_summary:
  free(blob);
  if (stop_reason) printf("\nSTOPPED: offset=%ld seq=%u t_ms=%u reason=%s\n",
                          stop_offset, stop_seq, stop_ms, stop_reason);
  printf("verified prefix steps: %u  last verified STEP seq: %u\n",
         verified_prefix_steps, last_verified_seq);

  printf("\nsteps replayed: %u\n", t.steps);
  printf("sequence gaps: %u   bytes skipped: %u\n", seq_gaps, skipped);
  if (t.steps == 0U) {
    printf("\nNOTHING REPLAYED - no STEP records in this capture.\n");
    return 1;
  }
  printf("pairing mismatches:  %u\n", t.pairing_fail);
  printf("decision mismatches: %u\n", t.decision_fail);
  printf("BNO alignment failures: %u\n", t.alignment_fail);
  printf("values outside %.0e: %u\n", (double)TOLERANCE, t.numeric_fail);
  printf("worst deviation: %.3e on %s\n", (double)t.worst,
         t.worst_field ? t.worst_field : "-");
  printf("replayed final position: %.1f,%.1f mm\n",
         (double)(s.ekf.axis[0].p*1000.0F),
         (double)(s.ekf.axis[1].p*1000.0F));

  const bool ok = t.pairing_fail == 0U && t.decision_fail == 0U && t.numeric_fail == 0U &&
                  !observation_pending && observations != 0 && !stop_reason;
  /* Uma captura com buracos não consegue provar a reprodução: o replay nunca
     viu as amostras que o firmware usou, então um resultado limpo no resto
     não diz nada sobre a execução inteira. */
  if (stop_reason || observation_pending || seq_gaps > 0U || skipped > 0U || losses_reported) {
    printf("\nINCOMPLETE - gaps=%u skipped_bytes=%u loss_reported_by_DIAG=%d.\n",
           seq_gaps, skipped, losses_reported);
    if (observation_pending) printf("Recording ended/stopped with an observation awaiting STEP.\n");
    return 1;
  }
  printf("\n%s\n", ok ? "REPRODUCED - same samples, same decisions, same numbers."
                      : "DIVERGED - see above.");
  return ok ? 0 : 1;
}
