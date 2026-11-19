/**
 * @file flight_command.c
 * @brief Executor dos trechos do DroneLink sobre a task de controle
 *        (contrato em dronelink/flight_command_api.h).
 *
 * Tudo aqui roda na task do DroneLink. Um trecho vira um alvo do controle
 * (::control_goto, ::control_takeoff_to) e termina quando o drone fica 1 s no
 * alvo: referência parada, posição dentro da tolerância (5 cm e 0,05 rad por
 * padrão), velocidades abaixo de 5 cm/s e 0,1 rad/s. Falha por tempo esgotado
 * (o drone fica em HOLD), por corte dos motores ou por pouso no meio.
 *
 * Num voo armado pela Pi, a renovação do prazo de 300 ms do árbitro de motores
 * vem daqui (não há IHM na USB). A segurança da Pi é a supervisão de enlace e
 * de autorização do protocolo: HOLD e depois pouso, nunca corte no ar.
 */
#include "dronelink/flight_command_api.h"

#include "app/control_task.h"
#include "motors.h"
#include "power.h"
#include "power_cal.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <string.h>

/** Tempo no alvo para concluir um trecho, em ms. */
#define STABLE_MS          1000U
/** Tolerâncias padrão quando o trecho não traz as suas, em m e rad. */
#define POS_TOL_DEFAULT_M  0.05F
#define YAW_TOL_DEFAULT    0.05F
/** Velocidades máximas para considerar o drone parado, em m/s e rad/s. */
#define STILL_SPEED_M_S    0.05F
#define STILL_RATE_RAD_S   0.1F
/** Espera máxima para a decolagem ser aceita (chão medido), em ms. */
#define TAKEOFF_START_MS   2000U
/** Textos guardados para a Pi. */
#define TEXT_SLOTS         4U
#define TEXT_LEN           50U

/** @brief Trecho em execução. */
typedef struct {
  bool active;
  uint8_t action;
  /** Alvo x, y (m, local), altura (m, para cima) e guinada (rad). */
  float goal[4];
  float pos_tol;
  float yaw_tol;
  uint32_t start_ms;
  uint32_t timeout_ms;
  /** Desde quando o drone está no alvo (0 = não está). */
  uint32_t stable_since;
  /** Decolagem aceita pela task de controle. */
  bool takeoff_started;
} leg_t;

static leg_t leg;
/** Resultado final pendente para ::flight_poll_leg_result. */
static bool result_ready;
static flight_request_status_t result_status;
static flight_reason_t result_reason;
/** O voo atual foi armado pela Pi. */
static bool by_pi;
/** Último aviso do controle já repassado. */
static uint32_t last_note_seq;
/** Fila circular de textos para a Pi. */
static char texts[TEXT_SLOTS][TEXT_LEN];
static bool texts_critical[TEXT_SLOTS];
static uint32_t text_head;
static uint32_t text_count;

void flight_push_text(const char *text, bool critical) {
  const uint32_t slot = (text_head + text_count) % TEXT_SLOTS;
  (void)strncpy(texts[slot], text, TEXT_LEN - 1U);
  texts[slot][TEXT_LEN - 1U] = '\0';
  texts_critical[slot] = critical;
  if (text_count < TEXT_SLOTS) {
    ++text_count;
  } else {
    text_head = (text_head + 1U) % TEXT_SLOTS; /* sobrescreve o mais antigo */
  }
}

bool flight_take_text(char *out, size_t size, bool *critical) {
  if ((text_count == 0U) || (out == NULL) || (size == 0U)) {
    return false;
  }
  (void)strncpy(out, texts[text_head], size - 1U);
  out[size - 1U] = '\0';
  *critical = texts_critical[text_head];
  text_head = (text_head + 1U) % TEXT_SLOTS;
  --text_count;
  return true;
}

static flight_request_result_t result(flight_request_status_t status, flight_reason_t reason) {
  const flight_request_result_t r = {status, reason};
  return r;
}

/** @brief Encerra o trecho ativo e deixa o resultado para o protocolo. */
static void finish(flight_request_status_t status, flight_reason_t reason) {
  if (leg.active) {
    leg.active = false;
    result_ready = true;
    result_status = status;
    result_reason = reason;
  }
}

static bool flying(const control_snapshot_t *s) {
  return s->airborne && ((s->state == LQG_STATE_CLIMB) || (s->state == LQG_STATE_HOVER));
}

static float wrap_pi(float a) {
  return atan2f(sinf(a), cosf(a));
}

/** @brief Motivo DroneLink de um corte do controle (LQG_FAIL_*). */
static flight_reason_t reason_of_fail(uint8_t fail) {
  flight_reason_t r = FLIGHT_REASON_BAD_STATE;
  if ((fail == LQG_FAIL_IMU) || (fail == LQG_FAIL_RANGE) || (fail == LQG_FAIL_TILT) ||
      (fail == LQG_FAIL_ESTIMATOR)) {
    r = FLIGHT_REASON_SENSOR_UNHEALTHY;
  }
  return r;
}

static bool cut_fail(uint8_t fail) {
  return (fail != LQG_FAIL_NONE) && (fail != LQG_FAIL_LANDED);
}

flight_request_result_t flight_request_arm(bool arm) {
  control_snapshot_t s;
  control_get_snapshot(&s);
  if (!arm) {
    motors_stop("desarmado pela Pi");
    finish(FLIGHT_REQUEST_FAILED, FLIGHT_REASON_CANCELLED);
    by_pi = false;
    return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
  }
  if ((s.state != LQG_STATE_IDLE) || (motors_owner() != MOTORS_OWNER_NONE)) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BAD_STATE);
  }
  power_sample_t p;
  power_cal_t cal;
  power_get(&p);
  power_get_cal(&cal);
  if (!power_cal_arm_allowed(&cal, &p)) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BATTERY_LOW);
  }
  motors_bench_keepalive(); /* sem IHM na USB, o prazo do árbitro vem daqui */
  if (!control_arm()) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_SENSOR_UNHEALTHY);
  }
  by_pi = true;
  return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
}

/** @brief Começa um trecho; devolve "em andamento". */
static flight_request_result_t start(uint8_t action, const float goal[4],
                                     const flight_leg_t *l) {
  (void)memset(&leg, 0, sizeof(leg));
  leg.active = true;
  leg.action = action;
  for (uint32_t i = 0U; i < 4U; ++i) {
    leg.goal[i] = goal[i];
  }
  leg.pos_tol = (l->position_tolerance > 0.0F) ? l->position_tolerance : POS_TOL_DEFAULT_M;
  leg.yaw_tol = (l->yaw_tolerance > 0.0F) ? l->yaw_tolerance : YAW_TOL_DEFAULT;
  leg.start_ms = HAL_GetTick();
  leg.timeout_ms = l->timeout_ms;
  result_ready = false;
  return result(FLIGHT_REQUEST_IN_PROGRESS, FLIGHT_REASON_NONE);
}

flight_request_result_t flight_request_leg(const flight_leg_t *l) {
  control_snapshot_t s;
  control_get_snapshot(&s);
  if (leg.active) {
    return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
  }
  if (l->action == FLIGHT_ACTION_TAKEOFF_RELATIVE) {
    const float height = -l->z; /* z para baixo: acima do chão é negativo */
    /* Logo depois do armar a task de controle pode ainda não ter dado o passo que
       arma o controle: vale a saída já armada para ele. */
    const bool armed_on_ground =
        (s.state == LQG_STATE_ARMED) ||
        ((s.state == LQG_STATE_IDLE) && (motors_owner() == MOTORS_OWNER_CONTROL));
    if (!armed_on_ground) {
      return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BAD_STATE);
    }
    if ((height < LQG_PARAM_HEIGHT_MIN_M) || (height > LQG_PARAM_HEIGHT_MAX_M)) {
      return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_OUT_OF_BOUNDS);
    }
    const float goal[4] = {0.0F, 0.0F, height, 0.0F};
    return start(l->action, goal, l);
  }
  if (!flying(&s)) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BAD_STATE);
  }
  /* Parte da referência atual (em HOLD ela é onde o drone está parado). */
  const lqg_nav_t *n = &s.nav;
  float goal[4] = {n->ref[0], n->ref[1], n->ref[2], n->ref[3]};
  const float c = cosf(n->ref[3]);
  const float sn = sinf(n->ref[3]);
  if (l->action == FLIGHT_ACTION_ROTATE_RELATIVE) {
    goal[3] = n->ref[3] + l->yaw;
  } else if (l->action == FLIGHT_ACTION_TRANSLATE_RELATIVE) {
    if (l->frame == FLIGHT_FRAME_BODY_FRD) {
      goal[0] += (c * l->x) - (sn * l->y);
      goal[1] += (sn * l->x) + (c * l->y);
    } else {
      goal[0] += l->x;
      goal[1] += l->y;
    }
    goal[2] -= l->z;
  } else if (l->action == FLIGHT_ACTION_GOTO_LOCAL) {
    goal[0] = l->x;
    goal[1] = l->y;
    goal[2] = (l->z != 0.0F) ? -l->z : n->ref[2];
    goal[3] = n->ref[3] + wrap_pi(l->yaw - n->ref[3]); /* menor giro até a guinada pedida */
  } else {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_NOT_IMPLEMENTED);
  }
  const bool moves_xy = (fabsf(goal[0] - n->ref[0]) > 1e-4F) || (fabsf(goal[1] - n->ref[1]) > 1e-4F);
  if (moves_xy && ((s.flags & LQG_TLM_XY_ACTIVE) == 0U)) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_SENSOR_UNHEALTHY);
  }
  if (!control_goto(goal[0], goal[1], goal[2], goal[3], l->speed)) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_OUT_OF_BOUNDS);
  }
  return start(l->action, goal, l);
}

flight_request_result_t flight_request_cancel(void) {
  if (leg.active) {
    (void)control_hold();
    leg.active = false; /* o protocolo publica o cancelamento */
  }
  return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
}

flight_request_result_t flight_request_hold(flight_reason_t reason) {
  if (!control_hold()) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BAD_STATE);
  }
  finish(FLIGHT_REQUEST_FAILED, (reason == FLIGHT_REASON_NONE) ? FLIGHT_REASON_CANCELLED : reason);
  return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
}

flight_request_result_t flight_request_land(flight_reason_t reason) {
  control_snapshot_t s;
  control_get_snapshot(&s);
  const flight_reason_t why = (reason == FLIGHT_REASON_NONE) ? FLIGHT_REASON_CANCELLED : reason;
  if (control_land()) {
    finish(FLIGHT_REQUEST_FAILED, why);
    return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
  }
  if (s.state == LQG_STATE_ARMED) {
    motors_stop("pouso pedido no chao: desarmado");
    finish(FLIGHT_REQUEST_FAILED, why);
    return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
  }
  return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BAD_STATE);
}

/** @brief Estado DroneLink a partir da fase do controle e do trecho ativo. */
static uint8_t flight_state(const control_snapshot_t *s) {
  uint8_t st = FLIGHT_STATE_STANDBY;
  switch (s->state) {
  case LQG_STATE_IDLE:
    st = cut_fail(s->fail) ? FLIGHT_STATE_FAILSAFE : FLIGHT_STATE_STANDBY;
    break;
  case LQG_STATE_ARMED:
    st = leg.active ? FLIGHT_STATE_TAKEOFF : FLIGHT_STATE_ARMED;
    break;
  case LQG_STATE_SPOOL:
    st = FLIGHT_STATE_TAKEOFF;
    break;
  case LQG_STATE_CLIMB:
  case LQG_STATE_HOVER:
    if (!leg.active) {
      st = FLIGHT_STATE_HOLD;
    } else if (leg.action == FLIGHT_ACTION_TAKEOFF_RELATIVE) {
      st = FLIGHT_STATE_TAKEOFF;
    } else {
      st = FLIGHT_STATE_EXECUTING_LEG;
    }
    break;
  case LQG_STATE_LAND:
  default:
    st = FLIGHT_STATE_LAND;
    break;
  }
  return st;
}

void flight_get_snapshot(flight_snapshot_t *snapshot) {
  if (snapshot == NULL) {
    return;
  }
  control_snapshot_t s;
  control_get_snapshot(&s);
  (void)memset(snapshot, 0, sizeof(*snapshot));
  snapshot->state = flight_state(&s);
  snapshot->armed = s.armed;
  snapshot->airborne = s.airborne || (s.state == LQG_STATE_SPOOL);
  snapshot->commanded_by_pi = by_pi && s.armed;
  snapshot->attitude_valid = (s.flags & LQG_TLM_ATTITUDE) != 0U;
  snapshot->altitude_valid = (s.flags & LQG_TLM_RANGE) != 0U;
  snapshot->position_valid = (s.flags & LQG_TLM_XY_ACTIVE) != 0U;
  snapshot->x = s.nav.pos[0];
  snapshot->y = s.nav.pos[1];
  snapshot->z = -s.nav.pos[2];
  snapshot->vx = s.nav.vel[0];
  snapshot->vy = s.nav.vel[1];
  snapshot->vz = -s.nav.vel[2];
  snapshot->attitude.roll = s.phi;
  snapshot->attitude.pitch = s.theta;
  snapshot->attitude.yaw = s.nav.psi;
  snapshot->roll_speed = s.p;
  snapshot->pitch_speed = s.q;
  snapshot->yaw_speed = s.nav.r;
}

bool flight_actuation_is_available(void) {
  return FLT_SIM == 0;
}

bool flight_poll_leg_result(flight_request_status_t *status, flight_reason_t *reason) {
  if (!result_ready) {
    return false;
  }
  result_ready = false;
  *status = result_status;
  *reason = result_reason;
  return true;
}

/** @brief O drone está parado no alvo do trecho. */
static bool at_goal(const control_snapshot_t *s) {
  const lqg_nav_t *n = &s->nav;
  const bool xy_known = (s->flags & LQG_TLM_XY_ACTIVE) != 0U;
  const float ex = xy_known ? (n->pos[0] - leg.goal[0]) : 0.0F;
  const float ey = xy_known ? (n->pos[1] - leg.goal[1]) : 0.0F;
  const float ez = n->pos[2] - leg.goal[2];
  const float dist = sqrtf((ex * ex) + (ey * ey) + (ez * ez));
  const float speed = sqrtf((n->vel[0] * n->vel[0]) + (n->vel[1] * n->vel[1]) +
                            (n->vel[2] * n->vel[2]));
  return (s->state == LQG_STATE_HOVER) && n->ref_at_goal && (dist <= leg.pos_tol) &&
         (fabsf(wrap_pi(n->psi - leg.goal[3])) <= leg.yaw_tol) && (speed <= STILL_SPEED_M_S) &&
         (fabsf(n->r) <= STILL_RATE_RAD_S);
}

void flight_command_tick(uint32_t now_ms) {
  control_snapshot_t s;
  control_get_snapshot(&s);
  const bool armed_now = motors_owner() == MOTORS_OWNER_CONTROL;
  if (by_pi && armed_now) {
    motors_bench_keepalive();
  } else if (!armed_now) {
    by_pi = false;
  } else {
    /* voo da IHM: o prazo vem do HEARTBEAT da USB */
  }
  if (s.note_seq != last_note_seq) {
    last_note_seq = s.note_seq;
    if (s.note != NULL) {
      flight_push_text(s.note, s.note_critical);
    }
  }
  if (!leg.active) {
    return;
  }
  const uint32_t elapsed = now_ms - leg.start_ms;
  if (!armed_now || ((s.state == LQG_STATE_IDLE) && (elapsed > 100U))) {
    finish(FLIGHT_REQUEST_FAILED, reason_of_fail(s.fail));
  } else if (s.state == LQG_STATE_LAND) {
    finish(FLIGHT_REQUEST_FAILED, FLIGHT_REASON_CANCELLED);
  } else if (elapsed >= leg.timeout_ms) {
    (void)control_hold();
    finish(FLIGHT_REQUEST_FAILED, FLIGHT_REASON_TIMEOUT);
  } else if ((leg.action == FLIGHT_ACTION_TAKEOFF_RELATIVE) && !leg.takeoff_started) {
    leg.takeoff_started = control_takeoff_to(leg.goal[2]);
    if (!leg.takeoff_started && (elapsed >= TAKEOFF_START_MS)) {
      finish(FLIGHT_REQUEST_FAILED, FLIGHT_REASON_SENSOR_UNHEALTHY);
    }
  } else if (at_goal(&s)) {
    if (leg.stable_since == 0U) {
      leg.stable_since = (now_ms != 0U) ? now_ms : 1U;
    } else if ((now_ms - leg.stable_since) >= STABLE_MS) {
      finish(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
    } else {
      /* contando o tempo no alvo */
    }
  } else {
    leg.stable_since = 0U;
  }
}
