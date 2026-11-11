/**
 * @file motor_arbiter.c
 * @brief Lógica pura da saída dos motores (contrato em actuators/motor_arbiter.h).
 */
#include "actuators/motor_arbiter.h"

#include "dshot.h"

#include <stddef.h>

/** Máscara com todos os motores. */
#define ALL_MOTORS ((uint8_t)((1U << MOTOR_ARB_COUNT) - 1U))
/** Faixa de aceleração do DShot (48..2047). */
#define DSHOT_SPAN (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)

/** @brief Diz se o motor @p i está na máscara. */
static bool in_mask(uint8_t mask, uint32_t i) {
  return (((uint32_t)mask >> i) & 1U) != 0U;
}

/** @brief Nenhum motor girando nem com alvo. */
static bool all_zero(const motor_arb_t *a) {
  bool zero = true;
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    if ((a->target[i] != 0U) || (a->current[i] != 0U)) {
      zero = false;
    }
  }
  return zero;
}

/** @brief Desarmado, parado e sem comando especial pendente. */
static bool idle(const motor_arb_t *a) {
  return (a->owner == MOTOR_ARB_OWNER_NONE) && all_zero(a);
}

/** @brief Converte a fração 0..1 do voo no valor DShot com resolução total. */
static uint16_t dshot_from_fraction(float fraction) {
  uint16_t value = 0U;
  if (fraction > 0.0F) { /* também descarta NaN */
    const float f = (fraction > 1.0F) ? 1.0F : fraction;
    const uint32_t span = DSHOT_SPAN;
    const float steps = (f * (float)span) + 0.5F;
    const uint32_t whole = (uint32_t)steps;
    value = (uint16_t)(DSHOT_THROTTLE_MIN + whole);
  }
  return value;
}

/** @brief Põe um comando no fim da fila. */
static void enqueue(motor_arb_t *a, const motor_arb_cmd_t *cmd) {
  const uint32_t slot = (a->queue_head + a->queue_len) % MOTOR_ARB_QUEUE_LEN;
  a->queue[slot] = *cmd;
  a->queue_len++;
}

void motor_arb_init(motor_arb_t *a, uint32_t now_ms) {
  *a = (motor_arb_t){0};
  a->owner = MOTOR_ARB_OWNER_NONE;
  a->rate = MOTOR_ARB_DEFAULT_RATE;
  a->cap = MOTOR_ARB_BENCH_HARD_MAX;
  a->flight_ms = now_ms;
  a->bench_alive_ms = now_ms;
  a->sweep_note = -2;
}

bool motor_arb_arm(motor_arb_t *a, motor_arb_owner_t owner, uint32_t now_ms) {
  bool ok = false;
  if (((owner == MOTOR_ARB_OWNER_BENCH) || (owner == MOTOR_ARB_OWNER_FLIGHT)) && idle(a) &&
      (a->queue_len == 0U)) {
    a->owner = owner;
    a->sweep.active = false;
    a->flight_ms = now_ms;
    a->bench_alive_ms = now_ms;
    for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
      a->flight_throttle[i] = 0.0F;
    }
    ok = true;
  }
  return ok;
}

void motor_arb_disarm(motor_arb_t *a) {
  a->owner = MOTOR_ARB_OWNER_NONE;
  a->sweep.active = false;
  a->queue_head = 0U;
  a->queue_len = 0U;
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    a->target[i] = 0U;
    a->current[i] = 0U;
    a->flight_throttle[i] = 0.0F;
  }
}

void motor_arb_set_flight(motor_arb_t *a, const float throttle[MOTOR_ARB_COUNT], uint32_t now_ms) {
  if (a->owner == MOTOR_ARB_OWNER_FLIGHT) {
    for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
      a->flight_throttle[i] = throttle[i];
    }
    a->flight_ms = now_ms;
  }
}

void motor_arb_bench_keepalive(motor_arb_t *a, uint32_t now_ms) {
  a->bench_alive_ms = now_ms;
}

bool motor_arb_bench_set(motor_arb_t *a, uint8_t mask, uint16_t permille) {
  bool ok = false;
  if (a->owner == MOTOR_ARB_OWNER_BENCH) {
    const uint16_t value = (permille > a->cap) ? a->cap : permille;
    a->sweep.active = false;
    for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
      if (in_mask(mask, i)) {
        a->target[i] = value;
      }
    }
    ok = true;
  }
  return ok;
}

void motor_arb_bench_rate(motor_arb_t *a, uint32_t permille_per_s) {
  uint32_t r = permille_per_s;
  if (r < 10U) {
    r = 10U;
  }
  if (r > 1000U) {
    r = 1000U;
  }
  a->rate = r;
}

uint16_t motor_arb_bench_cap(motor_arb_t *a, uint16_t permille) {
  a->cap = (permille > MOTOR_ARB_BENCH_HARD_MAX) ? (uint16_t)MOTOR_ARB_BENCH_HARD_MAX : permille;
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    if (a->target[i] > a->cap) {
      a->target[i] = a->cap;
    }
  }
  if (a->sweep.max > a->cap) {
    a->sweep.max = a->cap;
  }
  if (a->sweep.level > a->cap) {
    a->sweep.level = a->cap;
  }
  return a->cap;
}

bool motor_arb_bench_sweep(motor_arb_t *a, uint8_t mask, uint16_t max_permille,
                           uint16_t step_permille, uint32_t hold_ms, uint32_t now_ms) {
  bool ok = false;
  if (a->owner == MOTOR_ARB_OWNER_BENCH) {
    a->sweep.mask = mask;
    a->sweep.max = (max_permille > a->cap) ? a->cap : max_permille;
    a->sweep.step = (step_permille == 0U) ? 100U : step_permille;
    a->sweep.hold_ms = hold_ms;
    a->sweep.level = 0U;
    a->sweep.down = false;
    a->sweep.next_ms = now_ms;
    a->sweep.active = true;
    for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
      a->target[i] = 0U;
    }
    ok = true;
  }
  return ok;
}

bool motor_arb_beep(motor_arb_t *a, uint8_t mask) {
  const uint8_t m = mask & ALL_MOTORS;
  bool ok = false;
  if ((m != 0U) && idle(a) && (a->queue_len < MOTOR_ARB_QUEUE_LEN)) {
    motor_arb_cmd_t cmd = {.frames = MOTOR_ARB_CMD_FRAMES, .gap = 0U};
    for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
      cmd.value[i] = in_mask(m, i) ? (uint16_t)DSHOT_CMD_BEEP1 : 0U;
    }
    enqueue(a, &cmd);
    ok = true;
  }
  return ok;
}

bool motor_arb_set_direction(motor_arb_t *a, uint8_t mask, uint8_t reversed, bool save) {
  const uint8_t m = mask & ALL_MOTORS;
  const uint32_t needed = save ? 2U : 1U;
  bool ok = false;
  if ((m != 0U) && idle(a) && ((a->queue_len + needed) <= MOTOR_ARB_QUEUE_LEN)) {
    motor_arb_cmd_t dir = {.frames = MOTOR_ARB_CMD_FRAMES, .gap = MOTOR_ARB_DIR_GAP_FRAMES};
    motor_arb_cmd_t store = {.frames = MOTOR_ARB_CMD_FRAMES, .gap = MOTOR_ARB_SAVE_GAP_FRAMES};
    for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
      dir.value[i] = 0U;
      store.value[i] = 0U;
      if (in_mask(m, i)) {
        dir.value[i] = in_mask(reversed, i) ? (uint16_t)MOTOR_ARB_CMD_DIR_REVERSED
                                            : (uint16_t)MOTOR_ARB_CMD_DIR_NORMAL;
        store.value[i] = MOTOR_ARB_CMD_SAVE;
      }
    }
    enqueue(a, &dir);
    if (save) {
      enqueue(a, &store);
    }
    a->reversed = (uint8_t)((a->reversed & (uint8_t)~m) | (reversed & m));
    a->reversed_known |= m;
    ok = true;
  }
  return ok;
}

/** @brief Avança a varredura: um degrau a cada hold_ms, sobe e depois desce. */
static void sweep_step(motor_arb_t *a, uint32_t now_ms) {
  motor_arb_sweep_t *s = &a->sweep;
  /* Diferença modular: "já chegou" vale também na volta do contador de ms. */
  const uint32_t since = now_ms - s->next_ms;
  if (s->active && (since < 0x80000000U)) {
    if (!s->down) {
      if (s->level >= s->max) {
        s->down = true;
      } else {
        const uint32_t next = (uint32_t)s->level + s->step;
        s->level = (next > s->max) ? s->max : (uint16_t)next;
      }
    }
    if (s->down && (s->level == 0U)) {
      s->active = false;
      a->sweep_note = -1;
    } else {
      if (s->down) {
        const uint32_t lower = (uint32_t)s->level - s->step;
        s->level = (s->level > s->step) ? (uint16_t)lower : 0U;
      }
      for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
        if (in_mask(s->mask, i)) {
          a->target[i] = s->level;
        }
      }
      a->sweep_note = (int32_t)s->level;
      /* O tempo de cada degrau conta a partir do fim da rampa até ele. */
      s->next_ms = now_ms + s->hold_ms + (((uint32_t)s->step * 1000U) / a->rate);
    }
  }
}

/** @brief Quadro desarmado: zeros ou o comando especial da vez. */
static void idle_frame(motor_arb_t *a, motor_arb_frame_t *f) {
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    f->value[i] = 0U;
    a->sent_permille[i] = 0U;
  }
  if (a->queue_len != 0U) {
    motor_arb_cmd_t *cmd = &a->queue[a->queue_head];
    if (cmd->frames > 0U) {
      for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
        f->value[i] = cmd->value[i];
      }
      f->telemetry = true;
      cmd->frames--;
    } else if (cmd->gap > 0U) {
      cmd->gap--;
    } else {
      /* item já vazio: só sai da fila */
    }
    if ((cmd->frames == 0U) && (cmd->gap == 0U)) {
      a->queue_head = (a->queue_head + 1U) % MOTOR_ARB_QUEUE_LEN;
      a->queue_len--;
    }
  }
}

/** @brief Quadro da bancada: rampa até o alvo. */
static void bench_frame(motor_arb_t *a, uint32_t now_ms, motor_arb_frame_t *f) {
  sweep_step(a, now_ms);
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    const uint32_t goal = (uint32_t)a->target[i] * 1000U;
    if (a->current[i] < goal) {
      a->current[i] = ((goal - a->current[i]) > a->rate) ? (a->current[i] + a->rate) : goal;
    } else if (a->current[i] > goal) {
      a->current[i] = ((a->current[i] - goal) > a->rate) ? (a->current[i] - a->rate) : goal;
    } else {
      /* já no alvo */
    }
    a->sent_permille[i] = (uint16_t)(a->current[i] / 1000U);
    f->value[i] = dshot_from_permille(a->sent_permille[i]);
  }
}

/** @brief Quadro do voo: fração pedida pelo controle. */
static void flight_frame(motor_arb_t *a, motor_arb_frame_t *f) {
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    f->value[i] = dshot_from_fraction(a->flight_throttle[i]);
    const float pm = a->flight_throttle[i] * 1000.0F;
    float clamped = pm;
    if (!(pm > 0.0F)) {
      clamped = 0.0F;
    } else if (pm > 1000.0F) {
      clamped = 1000.0F;
    } else {
      /* dentro da faixa */
    }
    a->sent_permille[i] = (uint16_t)clamped;
    a->current[i] = (uint32_t)a->sent_permille[i] * 1000U;
  }
}

motor_arb_frame_t motor_arb_frame(motor_arb_t *a, uint32_t now_ms, const char *stop) {
  motor_arb_frame_t f = {.telemetry = false, .stopped = NULL};
  if (stop != NULL) {
    motor_arb_disarm(a);
    f.stopped = stop;
  } else if ((a->owner == MOTOR_ARB_OWNER_FLIGHT) &&
             ((now_ms - a->flight_ms) >= MOTOR_ARB_FLIGHT_TIMEOUT_MS)) {
    motor_arb_disarm(a);
    f.stopped = "controle de voo sem atualizacao por 100 ms";
  } else if ((a->owner == MOTOR_ARB_OWNER_BENCH) &&
             ((now_ms - a->bench_alive_ms) >= MOTOR_ARB_BENCH_KEEPALIVE_MS)) {
    motor_arb_disarm(a);
    f.stopped = "bancada sem keepalive por 300 ms";
  } else {
    /* segue armado ou desarmado */
  }

  if (a->owner == MOTOR_ARB_OWNER_FLIGHT) {
    flight_frame(a, &f);
  } else if (a->owner == MOTOR_ARB_OWNER_BENCH) {
    bench_frame(a, now_ms, &f);
  } else {
    idle_frame(a, &f);
  }
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    a->sent_dshot[i] = f.value[i];
  }
  return f;
}
