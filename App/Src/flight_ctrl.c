/**
 * @file flight_ctrl.c
 * @brief Malha de controle LQG (contrato em flight_ctrl.h).
 */
#include "flight_ctrl.h"
#include "flight_config.h"
#include "lqg.h"
#include "bno08x.h"
#include "displacement.h"
#include "motor_test.h"
#include "sensor_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Sem relatório do BNO por este tempo, o voo é cortado, em ms. */
#define BNO_TIMEOUT_MS 20U

typedef enum { MODE_IDLE = 0, MODE_DRY = 1, MODE_FLY = 2 } flight_mode_t;

static lqg_t filter;
static float x_ref[LQG_NX];
static volatile flight_mode_t mode;
/** Pedido de início vindo do comando (task de motores) para esta task. */
static volatile flight_mode_t start_request;
static uint32_t steps;
static uint32_t last_disp_samples;
static uint32_t bno_timeouts;

/** @brief Converte a rotação do modelo em aceleração DShot (flight_config.h). */
static uint16_t omega_to_permille(float omega) {
  const float span = MOTOR_OMEGA_AT_MAX_RAD_S - MOTOR_OMEGA_AT_MIN_RAD_S;
  float pm = 1000.0F * (omega - MOTOR_OMEGA_AT_MIN_RAD_S) / span;
  if (!(pm >= (float)FLY_MIN_PERMILLE)) pm = (float)FLY_MIN_PERMILLE;
  if (pm > 1000.0F) pm = 1000.0F;
  return (uint16_t)pm;
}

/** @brief Referência = estimativa atual: segurar onde está, sem velocidade. */
static void hold_here(void) {
  memset(x_ref, 0, sizeof(x_ref));
  x_ref[LQG_X_X] = filter.x[LQG_X_X];
  x_ref[LQG_X_Y] = filter.x[LQG_X_Y];
  x_ref[LQG_X_Z] = filter.x[LQG_X_Z];
  x_ref[LQG_X_PSI] = filter.x[LQG_X_PSI];
}

static void say(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void say(const char *fmt, ...) {
  char text[61];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);
  motor_say(text);
}

/** @brief Chamada pelo teste de motores a cada parada. */
void flight_on_stop(void) {
  mode = MODE_IDLE;
  start_request = MODE_IDLE;
}

/** @brief Comandos do controle (chamada na task de motores). */
bool flight_command(int argc, char **argv) {
  const char *c = argv[0];
  if (!strcmp(c, "dry")) {
    start_request = MODE_DRY;
    say("dry: LQG ligado, motores parados");
    return true;
  }
  if (!strcmp(c, "fly")) {
#if LQG_FLIGHT_ENABLED
    if (!motor_control_engage()) { say("fly recusado: motores ou porta"); return true; }
    start_request = MODE_FLY;
    say("FLY: LQG nos motores. Enter vazio para parar");
#else
    say("fly desligado: LQG_FLIGHT_ENABLED=0");
#endif
    return true;
  }
  if (!strcmp(c, "lqg")) {
    say("modo=%u passos=%lu bno_timeout=%lu", (unsigned)mode, (unsigned long)steps,
        (unsigned long)bno_timeouts);
    say("rej z=%lu vx=%lu vy=%lu", (unsigned long)filter.rejected[LQG_Y_Z],
        (unsigned long)filter.rejected[LQG_Y_VX], (unsigned long)filter.rejected[LQG_Y_VY]);
    return true;
  }
  if (!strcmp(c, "ref") && argc >= 3) {
    if (!strcmp(argv[1], "z")) {
      float h = strtof(argv[2], NULL);
      if (!(h >= 0.0F && h <= REF_Z_MAX_M)) { say("ref z fora de 0..%d cm", (int)(REF_Z_MAX_M * 100)); return true; }
      x_ref[LQG_X_Z] = -h;  /* NED: z para baixo */
      say("ref z=%d cm", (int)(h * 100.0F));
      return true;
    }
    if (!strcmp(argv[1], "xy") && argc >= 4) {
      x_ref[LQG_X_X] = strtof(argv[2], NULL);
      x_ref[LQG_X_Y] = strtof(argv[3], NULL);
      say("ref xy=%d,%d cm", (int)(x_ref[LQG_X_X] * 100.0F), (int)(x_ref[LQG_X_Y] * 100.0F));
      return true;
    }
    if (!strcmp(argv[1], "yaw")) {
      x_ref[LQG_X_PSI] += strtof(argv[2], NULL) * 0.0174533F;
      say("ref yaw alterada");
      return true;
    }
  }
  return false;
}

/** @brief Uma iteração da malha: predição, correções, controle e saída. */
static void step(const bno_sample_t *b) {
  lqg_predict(&filter);

  if (b->euler_valid) {
    lqg_update(&filter, LQG_Y_PHI, SIGN_ROLL * b->euler.angle.roll, true);
    lqg_update(&filter, LQG_Y_THETA, SIGN_PITCH * b->euler.angle.pitch, true);
    lqg_update(&filter, LQG_Y_PSI, SIGN_YAW * b->euler.angle.yaw, true);
  }
  lqg_update(&filter, LQG_Y_P, SIGN_P * b->gyro[0], false);
  lqg_update(&filter, LQG_Y_Q, SIGN_Q * b->gyro[1], false);
  lqg_update(&filter, LQG_Y_R, SIGN_R * b->gyro[2], false);

  /* Altura e velocidades só quando o deslocamento integrou um frame novo. */
  static displacement_report_t d;
  displacement_get(&d);
  if (d.fused.samples != last_disp_samples) {
    last_disp_samples = d.fused.samples;
    if (d.valid) {
      lqg_update(&filter, LQG_Y_Z, -d.height_m, false);
      lqg_update(&filter, LQG_Y_VX, SIGN_VX * d.fused.vx, false);
      lqg_update(&filter, LQG_Y_VY, SIGN_VY * d.fused.vy, false);
    }
  }

  float u[LQG_NU];
  if (mode == MODE_IDLE) {
    /* Sem controle: a predição usa a entrada de equilíbrio (motores parados
       não seguem o modelo, mas o Kalman é corrigido pelas medidas). */
    for (unsigned j = 0; j < LQG_NU; ++j) u[j] = filter.u[j] = LQG_U0[j];
  } else {
    lqg_control(&filter, x_ref, u);
    if (fabsf(filter.x[LQG_X_PHI]) > FAILSAFE_TILT_RAD ||
        fabsf(filter.x[LQG_X_THETA]) > FAILSAFE_TILT_RAD) {
      motor_request_stop("inclinacao excessiva");
    }
    uint16_t pm[4] = {0, 0, 0, 0};
    if (mode == MODE_FLY) {
      for (unsigned j = 0; j < 4U; ++j) pm[j] = omega_to_permille(u[j]);
    } else {
      /* Ensaio a seco: os motores não recebem u, então a predição seguinte
         não pode supor que receberam. */
      for (unsigned j = 0; j < LQG_NU; ++j) filter.u[j] = LQG_U0[j];
    }
    motor_control_output(pm);
  }
  if (++steps % LOG_EST_DIVIDER == 0U) log_est((uint8_t)mode, filter.x, u);
}

/** @brief Task do controle, acordada pelo BNO a cada relatório de rotação. */
static void flight_task(void *argument) {
  (void)argument;
  bno_set_listener(xTaskGetCurrentTaskHandle());
  lqg_init(&filter, NULL);
  for (;;) {
    const bool fresh = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BNO_TIMEOUT_MS)) != 0;
    if (start_request != MODE_IDLE) {
      hold_here();
      mode = start_request;
      start_request = MODE_IDLE;
    }
    if (mode == MODE_FLY && !motor_control_active()) mode = MODE_IDLE;
    if (!fresh) {
      ++bno_timeouts;
      if (mode != MODE_IDLE) motor_request_stop("BNO sem dado");
      continue;
    }
    bno_sample_t b;
    bno_get_sample(&b);
    if (!b.online && mode != MODE_IDLE) { motor_request_stop("BNO offline"); continue; }
    step(&b);
  }
}

static StaticTask_t flight_tcb;
static StackType_t flight_stack[768];

bool flight_ctrl_start(void) {
  return xTaskCreateStatic(flight_task, "Flight", sizeof(flight_stack) / sizeof(flight_stack[0]),
                           NULL, tskIDLE_PRIORITY + 4, flight_stack, &flight_tcb) != NULL;
}
