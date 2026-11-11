/**
 * @file motors.c
 * @brief Task dos motores (contrato em motors.h).
 *
 * As regras (dono, rampa, limites, failsafes, comandos especiais) estão em
 * actuators/motor_arbiter.c, que é puro e testado no PC. Aqui ficam só a
 * seção crítica em volta dele, a task de 1 kHz, o DShot e o log.
 */
#include "motors.h"

#include "dshot.h"
#include "main.h"
#include "sensor_log.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

/** Período da task e dos quadros DShot, em ms (1 kHz). */
#define PERIOD_MS 1U
/** Uma linha MOT a cada N quadros (100 Hz). */
#define LOG_EVERY 10U

/** Estado da saída; protegido por seção crítica curta. */
static motor_arb_t arb;
/** Pedido de parada; motivo em texto fixo. Escrito por qualquer task. */
static const char *volatile stop_request;

bool motors_arm(motors_owner_t who) {
  bool ok;
  taskENTER_CRITICAL();
  ok = (stop_request == NULL) && motor_arb_arm(&arb, who, HAL_GetTick());
  taskEXIT_CRITICAL();
  return ok;
}

void motors_stop(const char *why) {
  stop_request = (why != NULL) ? why : "stop";
}

motors_owner_t motors_owner(void) {
  return arb.owner;
}

void motors_set_flight_throttle(const float throttle[MOTORS_COUNT]) {
  taskENTER_CRITICAL();
  if (stop_request == NULL) {
    motor_arb_set_flight(&arb, throttle, HAL_GetTick());
  }
  taskEXIT_CRITICAL();
}

bool motors_bench_set(uint8_t mask, uint16_t permille) {
  bool ok;
  taskENTER_CRITICAL();
  ok = motor_arb_bench_set(&arb, mask, permille);
  taskEXIT_CRITICAL();
  return ok;
}

void motors_bench_rate(uint32_t permille_per_s) {
  taskENTER_CRITICAL();
  motor_arb_bench_rate(&arb, permille_per_s);
  taskEXIT_CRITICAL();
}

bool motors_bench_sweep(uint8_t mask, uint16_t max_permille, uint16_t step_permille,
                        uint32_t hold_ms) {
  bool ok;
  taskENTER_CRITICAL();
  ok = motor_arb_bench_sweep(&arb, mask, max_permille, step_permille, hold_ms, HAL_GetTick());
  taskEXIT_CRITICAL();
  return ok;
}

bool motors_beep(uint8_t mask) {
  bool ok;
  taskENTER_CRITICAL();
  ok = motor_arb_beep(&arb, mask);
  taskEXIT_CRITICAL();
  return ok;
}

void motors_bench_keepalive(void) {
  taskENTER_CRITICAL();
  motor_arb_bench_keepalive(&arb, HAL_GetTick());
  taskEXIT_CRITICAL();
}

uint16_t motors_bench_cap(uint16_t permille) {
  uint16_t cap;
  taskENTER_CRITICAL();
  cap = motor_arb_bench_cap(&arb, permille);
  taskEXIT_CRITICAL();
  return cap;
}

bool motors_set_direction(uint8_t mask, uint8_t reversed, bool save) {
  bool ok;
  taskENTER_CRITICAL();
  ok = motor_arb_set_direction(&arb, mask, reversed, save);
  taskEXIT_CRITICAL();
  return ok;
}

void motors_get_status(motors_status_t *out) {
  taskENTER_CRITICAL();
  out->owner = arb.owner;
  for (uint32_t i = 0U; i < MOTORS_COUNT; ++i) {
    out->permille[i] = arb.sent_permille[i];
    out->dshot[i] = arb.sent_dshot[i];
    out->bench_target[i] = arb.target[i];
  }
  out->bench_rate = arb.rate;
  out->sweeping = arb.sweep.active;
  out->bench_cap = arb.cap;
  out->reversed = arb.reversed;
  out->reversed_known = arb.reversed_known;
  out->commands_pending = (uint8_t)arb.queue_len;
  taskEXIT_CRITICAL();
}

/** @brief Registra no log o avanço da varredura (fora da seção crítica). */
static void report_sweep(void) {
  taskENTER_CRITICAL();
  const int32_t note = arb.sweep_note;
  arb.sweep_note = -2;
  taskEXIT_CRITICAL();
  if (note == -2) {
    return;
  }
  char text[32];
  if (note < 0) {
    (void)snprintf(text, sizeof(text), "sweep terminou");
  } else {
    (void)snprintf(text, sizeof(text), "sweep degrau %ld%%", (long)(note / 10));
  }
  log_text(text);
}

/** @brief Task dos motores: atende paradas e envia um quadro por ms. */
static void motors_task(void *argument) {
  (void)argument;
  uint32_t frames = 0U;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    taskENTER_CRITICAL();
    const char *why = stop_request;
    stop_request = NULL;
    const motor_arb_frame_t f = motor_arb_frame(&arb, HAL_GetTick(), why);
    const bool armed = arb.owner != MOTORS_OWNER_NONE;
    uint16_t permille[MOTORS_COUNT];
    for (uint32_t i = 0U; i < MOTORS_COUNT; ++i) {
      permille[i] = arb.sent_permille[i];
    }
    taskEXIT_CRITICAL();

    (void)dshot_write(f.value, f.telemetry);
    if (f.stopped != NULL) {
      char text[61];
      (void)snprintf(text, sizeof(text), "MOTORES PARADOS: %s", f.stopped);
      log_text(text);
    }
    report_sweep();

    if (++frames >= LOG_EVERY) {
      frames = 0U;
      log_motor_t m;
      m.armed = armed;
      for (uint32_t i = 0U; i < MOTORS_COUNT; ++i) {
        m.throttle_permille[i] = permille[i];
        m.dshot[i] = f.value[i];
      }
      log_motor(&m);
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));
  }
}

/** Controle da task dos motores (alocação estática). */
static StaticTask_t motors_tcb;
/** Pilha da task dos motores, em words. */
static StackType_t motors_stack[512];

bool motors_start(void) {
  motor_arb_init(&arb, HAL_GetTick());
  dshot_init();
  return xTaskCreateStatic(motors_task, "Motors", sizeof(motors_stack) / sizeof(motors_stack[0]),
                           NULL, tskIDLE_PRIORITY + 5, motors_stack, &motors_tcb) != NULL;
}
