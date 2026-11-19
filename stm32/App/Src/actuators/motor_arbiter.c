/**
 * @file motor_arbiter.c
 * @brief Lógica pura da saída dos motores (contrato em actuators/motor_arbiter.h).
 */
#include "actuators/motor_arbiter.h"

#include "dshot.h"

#include <stddef.h>

/** Máscara com todos os motores. */
#define ALL_MOTORS ((uint8_t)((1U << MOTOR_ARB_COUNT) - 1U))
/** @brief Diz se o motor @p i está na máscara. */
static bool in_mask(uint8_t mask, uint32_t i) {
  return (((uint32_t)mask >> i) & 1U) != 0U;
}

/** @brief Nenhum motor girando nem com alvo. */
static bool all_zero(const motor_arb_t *a) {
  bool zero = true;
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    if ((a->target[i] != 0U) || (a->current[i] != 0U) || (a->sent_dshot[i] != 0U)) {
      zero = false;
    }
  }
  return zero;
}

/** @brief Desarmado, com alvos/rampa em zero e último quadro lógico em zero. */
static bool idle(const motor_arb_t *a) {
  return (a->owner == MOTOR_ARB_OWNER_NONE) && all_zero(a);
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
  a->cap = MOTOR_ARB_BENCH_DEFAULT_CAP;
  a->ctrl_cap = MOTOR_ARB_CONTROL_DEFAULT_CAP;
  a->bench_alive_ms = now_ms;
  a->ctrl_alive_ms = now_ms;
  a->sweep_note = -2;
}

bool motor_arb_arm(motor_arb_t *a, motor_arb_owner_t owner, uint32_t now_ms) {
  bool ok = false;
  if (((owner == MOTOR_ARB_OWNER_BENCH) || (owner == MOTOR_ARB_OWNER_CONTROL)) && idle(a) &&
      (a->queue_len == 0U)) {
    a->owner = owner;
    a->sweep.active = false;
    a->bench_alive_ms = now_ms;
    a->ctrl_alive_ms = now_ms;
    for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
      a->target[i] = 0U;
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
  }
}

void motor_arb_bench_keepalive(motor_arb_t *a, uint32_t now_ms) {
  a->bench_alive_ms = now_ms;
}

bool motor_arb_control_set(motor_arb_t *a, const uint16_t permille[MOTOR_ARB_COUNT],
                           uint32_t now_ms) {
  bool ok = false;
  if (a->owner == MOTOR_ARB_OWNER_CONTROL) {
    for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
      a->target[i] = (permille[i] > a->ctrl_cap) ? a->ctrl_cap : permille[i];
    }
    a->ctrl_alive_ms = now_ms;
    ok = true;
  }
  return ok;
}

uint16_t motor_arb_control_cap(motor_arb_t *a, uint16_t permille) {
  a->ctrl_cap = (permille > MOTOR_ARB_BENCH_HARD_MAX) ? (uint16_t)MOTOR_ARB_BENCH_HARD_MAX
                                                      : permille;
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    if (a->target[i] > a->ctrl_cap) {
      a->target[i] = a->ctrl_cap;
    }
  }
  return a->ctrl_cap;
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
    if ((a->owner == MOTOR_ARB_OWNER_BENCH) && (a->current[i] > ((uint32_t)a->cap * 1000U))) {
      a->current[i] = (uint32_t)a->cap * 1000U;
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
  if ((m != 0U) && idle(a) && (a->queue_len == 0U) && (needed <= MOTOR_ARB_QUEUE_LEN)) {
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
    /* Pedido não é confirmação: o ESC pode ignorar o comando. */
    a->reversed_known &= (uint8_t)~m;
    a->direction_transmitted &= (uint8_t)~m;
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

/** @brief Quadro do controle: o alvo vale direto, sem rampa. */
static void control_frame(motor_arb_t *a, motor_arb_frame_t *f) {
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    a->current[i] = (uint32_t)a->target[i] * 1000U;
    a->sent_permille[i] = a->target[i];
    f->value[i] = dshot_from_permille(a->sent_permille[i]);
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
      if (cmd->frames == 0U) {
        for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
          if ((cmd->value[i] == MOTOR_ARB_CMD_DIR_NORMAL) ||
              (cmd->value[i] == MOTOR_ARB_CMD_DIR_REVERSED)) {
            a->direction_transmitted |= (uint8_t)(1U << i);
          }
        }
      }
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

motor_arb_frame_t motor_arb_frame(motor_arb_t *a, uint32_t now_ms, const char *stop) {
  motor_arb_frame_t f = {.telemetry = false, .stopped = NULL};
  if (stop != NULL) {
    motor_arb_disarm(a);
    f.stopped = stop;
  } else if ((a->owner != MOTOR_ARB_OWNER_NONE) &&
             ((now_ms - a->bench_alive_ms) >= MOTOR_ARB_BENCH_KEEPALIVE_MS)) {
    motor_arb_disarm(a);
    f.stopped = "bancada sem keepalive por 300 ms";
  } else if ((a->owner == MOTOR_ARB_OWNER_CONTROL) &&
             ((now_ms - a->ctrl_alive_ms) >= MOTOR_ARB_CONTROL_TIMEOUT_MS)) {
    motor_arb_disarm(a);
    f.stopped = "controle sem atualizacao por 100 ms";
  } else {
    /* segue armado ou desarmado */
  }

  if (a->owner == MOTOR_ARB_OWNER_BENCH) {
    bench_frame(a, now_ms, &f);
  } else if (a->owner == MOTOR_ARB_OWNER_CONTROL) {
    control_frame(a, &f);
  } else {
    idle_frame(a, &f);
  }
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    a->sent_dshot[i] = f.value[i];
  }
  return f;
}

motor_arb_frame_t motor_arb_submit(motor_arb_t *a, uint32_t now_ms, const char *stop,
                                   motor_arb_send_fn send) {
  motor_arb_t candidate = *a;
  motor_arb_frame_t f = {0};
  f = motor_arb_frame(&candidate, now_ms, stop);
  f.accepted = send(f.value, f.telemetry);
  if (f.accepted) {
    *a = candidate;
  } else {
    /* Não avançar fila nem publicar quadro que o DMA não aceitou.
     * Falha de saída invalida a sessão: o próximo quadro tenta enviar zero.
     * Preservar sent_* permite mostrar o último quadro aceito, não zero fictício. */
    motor_arb_disarm(a);
    f.stopped = "falha de envio DShot; exige novo armamento";
    a->tx_failures++;
  }
  return f;
}
