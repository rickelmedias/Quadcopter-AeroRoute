/**
 * @file motors.c
 * @brief Task dos motores e arbitragem bancada/voo (contrato em motors.h).
 *
 * O estado compartilhado (dono, alvos e pedidos) é protegido por seção
 * crítica curta. Só a task dos motores mexe na rampa e no DShot.
 */
#include "motors.h"

#include "dshot.h"
#include "main.h"
#include "sensor_log.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

/** Período da task e dos quadros DShot, em ms (1 kHz). */
#define PERIOD_MS 1U
/** Sem atualização do controle de voo por este tempo, corta e exige novo armamento. */
#define FLIGHT_COMMAND_TIMEOUT_MS 100U
/** Uma linha MOT a cada N quadros (100 Hz). */
#define LOG_EVERY 10U
/** Rampa padrão da bancada, em décimos de por cento por segundo (20 %/s). */
#define DEFAULT_RATE 200U
/** Quadros com o comando de beep (o ESC precisa recebê-lo repetido). */
#define BEEP_FRAMES 10U
/** Faixa de aceleração do DShot (48..2047). */
#define DSHOT_SPAN (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)

/** @brief Varredura automática da bancada. */
typedef struct {
  bool active;        /**< Varredura em andamento. */
  uint8_t mask;       /**< Bit 0 = motor 1. */
  uint16_t max;       /**< Décimos de por cento. */
  uint16_t step;      /**< Décimos de por cento. */
  uint32_t hold_ms;   /**< Tempo em cada degrau. */
  uint16_t level;     /**< Degrau atual. */
  bool down;          /**< Já está descendo. */
  uint32_t next_ms;   /**< Próxima troca de degrau. */
} sweep_t;

/* ---- Estado compartilhado (seção crítica) ------------------------------ */

/** Dono atual. */
static motors_owner_t owner;
/** Alvo de cada motor na bancada, em décimos de por cento. */
static uint16_t target[MOTORS_COUNT];
/** Rampa da bancada, em décimos de por cento por segundo. */
static uint32_t rate = DEFAULT_RATE;
/** Varredura da bancada. */
static sweep_t sweep;
/** Aceleração pedida pelo voo (0..1). */
static float flight_throttle[MOTORS_COUNT];
/** Tick da última atualização válida do controle de voo. */
static uint32_t flight_command_ms;
/** Pedido de parada; motivo em texto fixo. */
static const char *volatile stop_request;
/** Motores do beep (máscara). */
static uint8_t beep_mask;
/** Quadros de beep restantes. */
static uint32_t beep_left;
/** Aviso da varredura para o log: degrau novo (>= 0), fim (-1) ou nada (-2). */
static int32_t sweep_note = -2;

/* ---- Só a task dos motores --------------------------------------------- */

/** Valor atual de cada motor na bancada, em milésimos de décimo de por cento. */
static uint32_t current[MOTORS_COUNT];
/** Último valor DShot enviado, para status. */
static uint16_t sent_dshot[MOTORS_COUNT];
/** Aceleração do último quadro, em décimos de por cento. */
static uint16_t sent_permille[MOTORS_COUNT];

/** @brief Diz se nenhum motor está girando nem tem alvo. Chamar na seção crítica. */
static bool all_zero_locked(void) {
  for (unsigned i = 0U; i < MOTORS_COUNT; ++i) {
    if (target[i] != 0U || current[i] != 0U) return false;
  }
  return true;
}

/** @brief Converte a fração 0..1 do voo no valor DShot com resolução total. */
static uint16_t dshot_from_fraction(float fraction) {
  if (!(fraction > 0.0F)) return 0U;  /* também pega NaN */
  if (fraction > 1.0F) fraction = 1.0F;
  return (uint16_t)(DSHOT_THROTTLE_MIN + (uint16_t)(fraction * (float)DSHOT_SPAN + 0.5F));
}

bool motors_arm(motors_owner_t who) {
  if (who != MOTORS_OWNER_BENCH && who != MOTORS_OWNER_FLIGHT) return false;
  bool ok;
  taskENTER_CRITICAL();
  ok = owner == MOTORS_OWNER_NONE && all_zero_locked() && stop_request == NULL;
  if (ok) {
    owner = who;
    sweep.active = false;
    flight_command_ms = HAL_GetTick();
    for (unsigned i = 0U; i < MOTORS_COUNT; ++i) flight_throttle[i] = 0.0F;
  }
  taskEXIT_CRITICAL();
  return ok;
}

void motors_stop(const char *why) { stop_request = why ? why : "stop"; }

motors_owner_t motors_owner(void) { return owner; }

void motors_set_flight_throttle(const float throttle[MOTORS_COUNT]) {
  taskENTER_CRITICAL();
  if (owner == MOTORS_OWNER_FLIGHT && stop_request == NULL) {
    for (unsigned i = 0U; i < MOTORS_COUNT; ++i) flight_throttle[i] = throttle[i];
    flight_command_ms = HAL_GetTick();
  }
  taskEXIT_CRITICAL();
}

bool motors_bench_set(uint8_t mask, uint16_t permille) {
  if (permille > MOTORS_BENCH_MAX_PERMILLE) permille = MOTORS_BENCH_MAX_PERMILLE;
  bool ok;
  taskENTER_CRITICAL();
  ok = owner == MOTORS_OWNER_BENCH;
  if (ok) {
    sweep.active = false;
    for (unsigned i = 0U; i < MOTORS_COUNT; ++i) {
      if (mask & (1U << i)) target[i] = permille;
    }
  }
  taskEXIT_CRITICAL();
  return ok;
}

void motors_bench_rate(uint32_t permille_per_s) {
  if (permille_per_s < 10U) permille_per_s = 10U;
  if (permille_per_s > 1000U) permille_per_s = 1000U;
  taskENTER_CRITICAL();
  rate = permille_per_s;
  taskEXIT_CRITICAL();
}

bool motors_bench_sweep(uint8_t mask, uint16_t max_permille, uint16_t step_permille,
                        uint32_t hold_ms) {
  if (max_permille > MOTORS_BENCH_MAX_PERMILLE) max_permille = MOTORS_BENCH_MAX_PERMILLE;
  if (step_permille == 0U) step_permille = 100U;
  bool ok;
  taskENTER_CRITICAL();
  ok = owner == MOTORS_OWNER_BENCH;
  if (ok) {
    sweep.mask = mask;
    sweep.max = max_permille;
    sweep.step = step_permille;
    sweep.hold_ms = hold_ms;
    sweep.level = 0U;
    sweep.down = false;
    sweep.next_ms = HAL_GetTick();
    sweep.active = true;
    for (unsigned i = 0U; i < MOTORS_COUNT; ++i) target[i] = 0U;
  }
  taskEXIT_CRITICAL();
  return ok;
}

bool motors_beep(uint8_t mask) {
  bool ok;
  taskENTER_CRITICAL();
  ok = owner == MOTORS_OWNER_NONE && all_zero_locked();
  if (ok) {
    beep_mask = mask;
    beep_left = BEEP_FRAMES;
  }
  taskEXIT_CRITICAL();
  return ok;
}

void motors_get_status(motors_status_t *out) {
  taskENTER_CRITICAL();
  out->owner = owner;
  for (unsigned i = 0U; i < MOTORS_COUNT; ++i) {
    out->permille[i] = sent_permille[i];
    out->dshot[i] = sent_dshot[i];
    out->bench_target[i] = target[i];
  }
  out->bench_rate = rate;
  out->sweeping = sweep.active;
  taskEXIT_CRITICAL();
}

/** @brief Zera tudo e desarma. Só a task dos motores chama. */
static void stop_now(const char *why) {
  taskENTER_CRITICAL();
  owner = MOTORS_OWNER_NONE;
  sweep.active = false;
  for (unsigned i = 0U; i < MOTORS_COUNT; ++i) {
    target[i] = 0U;
    current[i] = 0U;
    flight_throttle[i] = 0.0F;
  }
  taskEXIT_CRITICAL();
  char text[61];
  snprintf(text, sizeof(text), "MOTORES PARADOS: %s", why);
  log_text(text);
}

/**
 * @brief Avança a varredura: um degrau a cada hold_ms, sobe e depois desce.
 *        Chamar na seção crítica.
 */
static void sweep_step_locked(uint32_t now) {
  if (!sweep.active || (int32_t)(now - sweep.next_ms) < 0) return;
  if (!sweep.down) {
    if (sweep.level >= sweep.max) {
      sweep.down = true;
    } else {
      const uint32_t next = (uint32_t)sweep.level + sweep.step;
      sweep.level = (uint16_t)(next > sweep.max ? sweep.max : next);
    }
  }
  if (sweep.down) {
    if (sweep.level == 0U) {
      sweep.active = false;
      sweep_note = -1;
      return;
    }
    sweep.level = (uint16_t)(sweep.level > sweep.step ? sweep.level - sweep.step : 0);
  }
  for (unsigned i = 0U; i < MOTORS_COUNT; ++i) {
    if (sweep.mask & (1U << i)) target[i] = sweep.level;
  }
  sweep_note = sweep.level;
  /* O tempo de cada degrau conta a partir do fim da rampa até ele. */
  sweep.next_ms = now + sweep.hold_ms + (uint32_t)sweep.step * 1000U / rate;
}

/**
 * @brief Calcula o quadro deste milissegundo.
 * @param[out] value      Valor DShot de cada motor.
 * @param[out] telemetry  Bit de telemetria (exigido pelos comandos especiais).
 */
static void compute_frame(uint16_t value[MOTORS_COUNT], bool *telemetry) {
  *telemetry = false;
  taskENTER_CRITICAL();
  const motors_owner_t who = owner;
  if (who == MOTORS_OWNER_FLIGHT) {
    for (unsigned i = 0U; i < MOTORS_COUNT; ++i) {
      value[i] = dshot_from_fraction(flight_throttle[i]);
      const float pm = flight_throttle[i] * 1000.0F;
      sent_permille[i] = (uint16_t)(pm < 0.0F ? 0.0F : (pm > 1000.0F ? 1000.0F : pm));
      current[i] = (uint32_t)sent_permille[i] * 1000U;
    }
  } else if (who == MOTORS_OWNER_BENCH) {
    sweep_step_locked(HAL_GetTick());
    for (unsigned i = 0U; i < MOTORS_COUNT; ++i) {
      const uint32_t goal = (uint32_t)target[i] * 1000U;
      if (current[i] < goal) {
        current[i] = (goal - current[i] > rate) ? current[i] + rate : goal;
      } else if (current[i] > goal) {
        current[i] = (current[i] - goal > rate) ? current[i] - rate : goal;
      }
      sent_permille[i] = (uint16_t)(current[i] / 1000U);
      value[i] = dshot_from_permille(sent_permille[i]);
    }
  } else {
    for (unsigned i = 0U; i < MOTORS_COUNT; ++i) {
      sent_permille[i] = 0U;
      value[i] = 0U;
      if (beep_left != 0U && (beep_mask & (1U << i))) value[i] = DSHOT_CMD_BEEP1;
    }
    if (beep_left != 0U) {
      --beep_left;
      *telemetry = true;
    }
  }
  for (unsigned i = 0U; i < MOTORS_COUNT; ++i) sent_dshot[i] = value[i];
  taskEXIT_CRITICAL();
}

/** @brief Registra no log o avanço da varredura (fora da seção crítica). */
static void report_sweep(void) {
  taskENTER_CRITICAL();
  const int32_t note = sweep_note;
  sweep_note = -2;
  taskEXIT_CRITICAL();
  if (note == -2) return;
  char text[32];
  if (note < 0) {
    snprintf(text, sizeof(text), "sweep terminou");
  } else {
    snprintf(text, sizeof(text), "sweep degrau %ld%%", (long)(note / 10));
  }
  log_text(text);
}

/** @brief Task dos motores: atende paradas e envia um quadro por ms. */
static void motors_task(void *argument) {
  (void)argument;
  uint32_t frames = 0U;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const char *why = stop_request;
    if (why != NULL) {
      stop_request = NULL;
      stop_now(why);
    } else if (owner == MOTORS_OWNER_FLIGHT &&
               HAL_GetTick() - flight_command_ms >= FLIGHT_COMMAND_TIMEOUT_MS) {
      stop_now("controle de voo sem atualizacao por 100 ms");
    }
    uint16_t value[MOTORS_COUNT];
    bool telemetry;
    compute_frame(value, &telemetry);
    (void)dshot_write(value, telemetry);
    report_sweep();

    if (++frames >= LOG_EVERY) {
      frames = 0U;
      log_motor_t m;
      m.armed = owner != MOTORS_OWNER_NONE;
      for (unsigned i = 0U; i < MOTORS_COUNT; ++i) {
        m.throttle_permille[i] = sent_permille[i];
        m.dshot[i] = value[i];
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
  dshot_init();
  return xTaskCreateStatic(motors_task, "Motors", sizeof(motors_stack) / sizeof(motors_stack[0]),
                           NULL, tskIDLE_PRIORITY + 5, motors_stack, &motors_tcb) != NULL;
}
