/**
 * @file displacement.c
 * @brief Task de deslocamento: lê os dois sensores e alimenta os três
 *        estimadores (contratos em displacement.h).
 */
#include "displacement.h"
#include "sensor_log.h"
#include "sensors.h"
#include "capture.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/** Período da task, em ms. O MTF-01 transmite a ~100 Hz; a task consulta com
    o dobro dessa frequência. */
#define POLL_PERIOD_MS   5U

/** Converte a unidade do MTF-01 para rad/s. O sensor informa o fluxo já
    normalizado em "cm/s a 1 m". A 1 m, uma translação de v m/s produz
    exatamente v rad/s de fluxo angular, então dividir por 100 leva a unidade
    do fabricante direto para rad/s. */
#define FLOW_CM_S_AT_1M_TO_RAD_S 0.01F

/** Estimador RAW; alterado só pela task (e por displacement_reset()). */
static disp_state_t raw_state;
/** Estimador FUSED; alterado só pela task (e por displacement_reset()). */
static disp_state_t fused_state;
/** Filtro de Kalman; alterado só pela task (e por displacement_reset()). */
static flow_ekf_t ekf_state;
/** Parâmetros do filtro em uso. */
static const flow_ekf_config_t *ekf_cfg = &FLOW_EKF_DEFAULTS;
/** Relatório publicado; protegido por seção crítica. */
static displacement_report_t report = { .q = {1.0F, 0.0F, 0.0F, 0.0F} };
/** Calibração em uso; protegida por seção crítica. */
static disp_config_t cfg = {
  /* Medido com flow_calibrator.h depois de corrigir sinal e pareamento: 0,944
     (X) e 0,942 (Y), convergidos e estáveis com 3239/2215 amostras. Bem
     próximos de 1,0 - o ~1,27 que um ensaio de empurrar a placa sugeriu antes
     era principalmente o ruído da distância medida por pessoa, que a
     referência do giroscópio existe para evitar, e não erro real de escala.
     Confie neste número, não naquele. */
  .flow_scale_x = 0.944F,
  .flow_scale_y = 0.942F,
  /* Os dois eram -1,0, o palpite inicial de bancada de antes de haver um jeito
     rigoroso de conferir. Testes isolados por eixo (ver disp_estimator.c)
     mostraram que os eixos discordam de sinal nesta placa: X invertido, Y
     não. */
  .gyro_sign_x = 1.0F,
  .gyro_sign_y = -1.0F,
  .deadband_rad_s = 0.0F,
};
/** Acumuladores da calibração ao vivo; protegidos por seção crítica. */
static flow_calibrator_t calibrator;
/** Calibração ao vivo em andamento. */
static volatile bool calibrating = false;
/** Frames descartados por intervalo de relógio do sensor inutilizável. */
static uint32_t clock_resyncs;
/** Tick do último frame realmente integrado. */
static uint32_t last_integrated_ms;
/** Tick da última correção que o filtro ACEITOU. */
static uint32_t last_correction_ms;
/** Falso até o filtro aceitar a primeira correção. */
static bool have_correction;

/* ---- task ------------------------------------------------------------- */

/**
 * @brief Recalcula a parte do relatório que depende do tempo (idades, saúde e
 *        validade).
 *
 * Quem chama já deve estar dentro da seção crítica: estado e validade
 * precisam ficar visíveis no mesmo passo atômico, senão um leitor pode pegar
 * uma posição nova com a validade da passada anterior - a janela estreita que
 * uma versão antiga deixava ao liberar a trava entre os dois.
 *
 * @param[in] now  Tick atual, em ms.
 */
static void apply_staleness_locked(uint32_t now) {
  /* Antes do primeiro frame / primeira correção aceita, a idade é o tempo
     desde o boot: nada ancorou a estimativa ainda, e isso não pode parecer
     "recente". */
  const uint32_t frame_age = last_integrated_ms ? (now - last_integrated_ms) : now;
  const uint32_t corr_age = have_correction ? (now - last_correction_ms) : now;
  const disp_health_t health = disp_health(ekf_state.converged, corr_age);
  report.age_ms = frame_age;
  report.correction_age_ms = corr_age;
  report.clock_resyncs = clock_resyncs;
  report.health = health;
  report.valid = (health == DISP_HEALTH_OK);
}

/**
 * @brief Publica o relatório completo depois de um frame integrado.
 *
 * @param[in] in  Frame que acabou de ser integrado.
 */
static void publish(const disp_input_t *in) {
  euler_t e;
  attitude_to_euler(in->q, &e);
  const uint32_t now = HAL_GetTick(); /* fora da trava, para ela durar pouco */
  taskENTER_CRITICAL();
  report.raw = raw_state;
  report.fused = fused_state;
  report.ekf = ekf_state;
  report.attitude = e;
  report.q[0] = in->q[0]; report.q[1] = in->q[1];
  report.q[2] = in->q[2]; report.q[3] = in->q[3];
  report.range_m = in->range_m;
  /* O telêmetro mede no próprio eixo, então um veículo inclinado lê a mais.
     Multiplicar por cos(inclinação) transforma essa distância inclinada em
     altura real. */
  report.height_m = in->range_m * attitude_cos_tilt(in->q);
  report.dt_s = in->dt_s;
  report.v_lsb_m_s = DISP_FLOW_LSB_RAD_S * in->range_m;
  report.quality = in->quality;
  report.cal_running = calibrating;
  for (int i = 0; i < 2; ++i) {
    report.cal_result_valid[i] = flow_cal_result(&calibrator, i, &report.cal_scalar[i],
                                                  &report.cal_residual[i]);
    report.cal_raw_valid[i] = flow_cal_raw_scalar(&calibrator, i, &report.cal_scalar_raw[i]);
    report.cal_samples[i] = calibrator.axis[i].samples;
  }
  apply_staleness_locked(now);
  taskEXIT_CRITICAL();
}

/**
 * @brief Atualiza as idades sem frame novo, para que elas continuem avançando
 *        enquanto a estimativa está parada.
 *
 * Roda em toda passada que não publicou, inclusive quando chegou um frame
 * com intervalo rejeitado - uma versão antiga pulava esse caso e congelava a
 * idade justamente quando o enlace com o sensor estava com problema.
 */
static void refresh_staleness(void) {
  const uint32_t now = HAL_GetTick();
  taskENTER_CRITICAL();
  apply_staleness_locked(now);
  report.ekf = ekf_state;
  taskEXIT_CRITICAL();
}

/**
 * @brief Entrega um frame à calibração ao vivo, se ela estiver rodando.
 *
 * Fica separada dos estimadores: usa o mesmo fluxo bruto e o mesmo giro, mas
 * a comparação que faz (fluxo bruto contra o que o giroscópio diz que um
 * sensor com escala perfeita leria) não tem relação com translação, distância
 * ou rotação pelo yaw - ver flow_calibrator.h. O pareamento de eixos e o sinal
 * são exatamente os de trans_x/trans_y em disp_fused_update (mesmo eixo, não
 * cruzado), então a manobra que zera a deriva do FUSED é a mesma que calibra
 * bem aqui.
 *
 * @param[in] in      Frame (só é usado se @c range_valid for verdadeiro).
 * @param[in] gyro_z  Velocidade de guinada, em rad/s, usada como filtro.
 */
static void calibrate_sample(const disp_input_t *in, float gyro_z) {
  taskENTER_CRITICAL();
  bool running = calibrating;
  float sign_x = cfg.gyro_sign_x, sign_y = cfg.gyro_sign_y;
  taskEXIT_CRITICAL();
  if (!running || !in->range_valid) return;

  const float expected_x = -sign_x * in->gyro_x;
  const float expected_y = -sign_y * in->gyro_y;
  taskENTER_CRITICAL();
  flow_cal_add_sample(&calibrator, 0, in->flow_x, expected_x, gyro_z);
  flow_cal_add_sample(&calibrator, 1, in->flow_y, expected_y, gyro_z);
  taskEXIT_CRITICAL();
}

/**
 * @brief Task de deslocamento.
 *
 * A cada 5 ms lê as fotografias dos dois sensores. Para cada frame novo do
 * MTF-01, confere o intervalo, monta a entrada, roda RAW, FUSED e EKF, grava
 * a observação e o passo na captura e publica o relatório. No build Capture,
 * só integra durante a sessão de captura e a partir de amostras novas.
 *
 * @param[in] argument  Não usado.
 */
static void displacement_task(void *argument) {
  (void)argument;
  /* Explícito, embora flow_ekf_predict/update agora se autoinicializem: isto
     documenta onde começa a vida do filtro. O risco é uma struct zerada vinda
     do armazenamento estático, em que P11 = 0 zera o primeiro ganho de Kalman
     e ao mesmo tempo satisfaz o teste de convergência. O ganho se recupera
     sozinho conforme a predição aumenta a covariância - o defeito real era a
     certeza inicial sem fundamento e um "convergiu" levantado antes de
     qualquer medida. */
  flow_ekf_reset(&ekf_state);
#if DISP_CALIBRATE_FLOW
  /* Inicia a calibração referenciada no giroscópio já no boot, em vez de
     esperar uma chamada a displacement_calibrate_start() - ainda não há canal
     de comandos em tempo de execução, então esta chave de compilação é o
     interruptor. */
  displacement_calibrate_start();
#endif
  uint32_t last_frames = 0;
  uint32_t last_sensor_ms = 0;
  bool have_previous = false;
  uint32_t last_diag_ms = 0;
#if CAPTURE_ENABLED
  bool capture_seen = false, capture_primed = false;
  uint32_t base_rotation = 0, base_gyro = 0, base_accel = 0, base_mtf = 0;
#endif

  for (;;) {
    bno_sample_t b;
    mtf_sample_t m;
    bno_get_sample(&b);
    mtf_get_sample(&m);
#if CAPTURE_ENABLED
    /* Não integrar movimento anterior à gravação. Começar do mesmo estado
       zerado do replay e esperar que todas as amostras referenciadas sejam
       desta sessão. */
    capture_diag_poll(&last_diag_ms, clock_resyncs);
    if (!capture_is_active()) {
      taskENTER_CRITICAL();
      report.valid = false;
      taskEXIT_CRITICAL();
      vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
      continue;
    }
    if (!capture_seen) {
      capture_seen = true;
      base_rotation = b.rotation.count; base_gyro = b.angular.count;
      base_accel = b.acceleration.count; base_mtf = m.frames;
    }
    if (!capture_primed) {
      capture_primed = b.online && b.rotation.count != base_rotation &&
        b.angular.count != base_gyro && b.acceleration.count != base_accel &&
        m.seen && m.frames != base_mtf;
      if (!capture_primed) { vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS)); continue; }
    }
#endif

    /* Integra por frame, não por consulta: o MTF-01 carimba cada frame com o
       próprio relógio, então usar o contador de frames como gatilho e esse
       relógio para o dt mantém a integral correta mesmo com a consulta
       oscilando. */
    bool published = false;
    if (m.seen && m.frames != last_frames) {
      uint32_t sensor_ms = m.time_ms;
      const uint32_t gap_ms = sensor_ms - last_sensor_ms;
      /* Reancora em vez de integrar através de um reinício do relógio do
         sensor ou de um intervalo longo demais para saber o que aconteceu
         nele. Todos os consumidores pulam este frame, inclusive a predição do
         filtro - predizer com um dt absurdo é o que faz a estimativa
         disparar. */
      const bool gap_usable = have_previous && disp_frame_gap_usable(gap_ms);
      capture_observation(m.frames, last_sensor_ms, have_previous, gap_usable,
                          m.range_valid, m.flow_valid);
      if (!gap_usable) { ++clock_resyncs; capture_diag(clock_resyncs); }
      if (gap_usable) {
        disp_input_t in = {0};
        in.flow_x = (float)m.flow_x * FLOW_CM_S_AT_1M_TO_RAD_S;
        in.flow_y = (float)m.flow_y * FLOW_CM_S_AT_1M_TO_RAD_S;
        in.gyro_x = b.gyro[0];
        in.gyro_y = b.gyro[1];
        in.q[0] = b.q[0]; in.q[1] = b.q[1]; in.q[2] = b.q[2]; in.q[3] = b.q[3];
        in.range_m = (float)m.distance_mm * 0.001F;
        in.dt_s = (float)gap_ms * 0.001F;
        in.quality = m.quality;
        /* O caminho FUSED precisa de uma atitude confiável, então uma IMU
           offline desqualifica o frame mesmo que o fluxo pareça bom. */
        in.range_valid = m.range_valid && m.flow_valid && b.online;

        taskENTER_CRITICAL();
        disp_config_t snapshot = cfg;
        taskEXIT_CRITICAL();

        calibrate_sample(&in, b.gyro[2]);

        disp_raw_update(&raw_state, &in, &snapshot);
        const bool fused_ok = disp_fused_update(&fused_state, &in, &snapshot);

        /* O filtro prediz com a aceleração da IMU girada para o mundo e
           corrige com a mesma velocidade que o FUSED acabou de calcular. A
           predição acontece em todo frame, mesmo quando a medida não serve:
           seguir por inércia com incerteza crescente é a resposta honesta, e
           é o que impede uma rejeição de enviesar o resultado. */
        float accel_world[2] = {0.0F, 0.0F};
        if (b.online) attitude_rotate_to_world(in.q, b.lin_accel, accel_world);
        flow_ekf_predict(&ekf_state, accel_world[0], accel_world[1], in.dt_s, ekf_cfg);
        /* Só uma correção aceita reancora a estimativa. Um frame que o FUSED
           rejeitou (IMU offline, distância ruim, qualidade baixa) ou que o
           gate do filtro descartou deixa o estado seguindo por inércia, e não
           pode renovar a idade em que um controlador confia. */
        bool ekf_ok = false;
        if (fused_ok && flow_ekf_update(&ekf_state, fused_state.vx, fused_state.vy,
                                        in.range_m, ekf_cfg)) {
          last_correction_ms = HAL_GetTick();
          have_correction = true;
          ekf_ok = true;
        }
        /* Registra exatamente quais amostras este passo usou, para o replay
           provar que reproduziu o mesmo pareamento em vez de supor a
           associação que o firmware fez. */
        capture_step(m.frames, b.rotation.count, b.angular.count,
                     b.acceleration.count, in.dt_s, fused_ok, ekf_ok,
                     b.online, raw_state.x, raw_state.y,
                     fused_state.vx, fused_state.vy,
                     ekf_state.axis[0].v, ekf_state.axis[1].v,
                     ekf_state.axis[0].p, ekf_state.axis[1].p);
        last_integrated_ms = HAL_GetTick();
        publish(&in);
        published = true;
      }
      last_frames = m.frames;
      last_sensor_ms = sensor_ms;
      have_previous = true;
    }
    /* Toda passada que não publicou ainda precisa avançar o relógio, senão o
       relatório congela no momento em que o MTF-01 se cala e um consumidor
       não distingue um veículo parado de um sensor parado. */
    if (!published) refresh_staleness();
    capture_diag_poll(&last_diag_ms, clock_resyncs);
    vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
  }
}

/* ---- API pública ------------------------------------------------------ */

/** @brief Cópia segura do relatório (ver displacement.h). */
void displacement_get(displacement_report_t *out) {
  taskENTER_CRITICAL(); *out = report; taskEXIT_CRITICAL();
}

/** @brief Zera os estimadores e reinicia o filtro (ver displacement.h). */
void displacement_reset(void) {
  taskENTER_CRITICAL();
  disp_state_t zero = {0};
  raw_state = fused_state = zero;
  report.raw = report.fused = zero;
  flow_ekf_reset(&ekf_state);
  report.ekf = ekf_state;
  taskEXIT_CRITICAL();
}

/** @brief Define a escala do fluxo por eixo (ver displacement.h). */
void displacement_set_flow_scale(float scale_x, float scale_y) {
  if (!isfinite(scale_x) || scale_x <= 0.0F) return;
  if (!isfinite(scale_y) || scale_y <= 0.0F) return;
  taskENTER_CRITICAL(); cfg.flow_scale_x = scale_x; cfg.flow_scale_y = scale_y; taskEXIT_CRITICAL();
}
/** @brief Escala do eixo X em uso (ver displacement.h). */
float displacement_get_flow_scale_x(void) { return cfg.flow_scale_x; }
/** @brief Escala do eixo Y em uso (ver displacement.h). */
float displacement_get_flow_scale_y(void) { return cfg.flow_scale_y; }

/** @brief Inicia a calibração ao vivo (ver displacement.h). */
void displacement_calibrate_start(void) {
  taskENTER_CRITICAL();
  flow_cal_reset(&calibrator);
  calibrating = true;
  taskEXIT_CRITICAL();
}
/** @brief Interrompe a calibração ao vivo (ver displacement.h). */
void displacement_calibrate_stop(void) {
  taskENTER_CRITICAL(); calibrating = false; taskEXIT_CRITICAL();
}
/** @brief Informa se a calibração ao vivo está rodando (ver displacement.h). */
bool displacement_calibrate_is_running(void) { return calibrating; }

/** @brief Aplica o resultado da calibração, se os dois eixos forem válidos (ver displacement.h). */
bool displacement_calibrate_apply(void) {
  float sx, sy;
  taskENTER_CRITICAL();
  bool ok = flow_cal_result(&calibrator, 0, &sx, NULL) &&
            flow_cal_result(&calibrator, 1, &sy, NULL);
  taskEXIT_CRITICAL();
  if (!ok) return false;
  displacement_set_flow_scale(sx, sy);
  return true;
}

/** @brief Define o sinal do giroscópio por eixo (ver displacement.h). */
void displacement_set_gyro_sign(float sign_x, float sign_y) {
  taskENTER_CRITICAL();
  cfg.gyro_sign_x = (sign_x >= 0.0F) ? 1.0F : -1.0F;
  cfg.gyro_sign_y = (sign_y >= 0.0F) ? 1.0F : -1.0F;
  taskEXIT_CRITICAL();
}
/** @brief Sinal do giroscópio no eixo X (ver displacement.h). */
float displacement_get_gyro_sign_x(void) { return cfg.gyro_sign_x; }
/** @brief Sinal do giroscópio no eixo Y (ver displacement.h). */
float displacement_get_gyro_sign_y(void) { return cfg.gyro_sign_y; }

/** @brief Define a zona morta opcional (ver displacement.h). */
void displacement_set_deadband(float rad_s) {
  if (!isfinite(rad_s) || rad_s < 0.0F) return;
  taskENTER_CRITICAL(); cfg.deadband_rad_s = rad_s; taskEXIT_CRITICAL();
}
/** @brief Parâmetros do filtro em uso (ver displacement.h). */
const flow_ekf_config_t *displacement_get_ekf_config(void) {
  taskENTER_CRITICAL();
  const flow_ekf_config_t *c = ekf_cfg;
  taskEXIT_CRITICAL();
  return c;
}

/** @brief Zona morta em uso (ver displacement.h). */
float displacement_get_deadband(void) { return cfg.deadband_rad_s; }

/** @brief Troca os parâmetros do filtro (ver displacement.h). */
void displacement_set_ekf_config(const flow_ekf_config_t *cfg) {
  if (!cfg) return;
  taskENTER_CRITICAL(); ekf_cfg = cfg; taskEXIT_CRITICAL();
}

/** @brief Cria a task de deslocamento (ver displacement.h). */
bool displacement_start(void) {
  return xTaskCreate(displacement_task, "Displace", 512, NULL, tskIDLE_PRIORITY+2, NULL) == pdPASS;
}
