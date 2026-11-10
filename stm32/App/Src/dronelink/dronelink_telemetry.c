/**
 * @file dronelink_telemetry.c
 * @brief Telemetria do controle para a Pi (contrato em dronelink/dronelink_telemetry.h).
 */
#include "dronelink/dronelink_telemetry.h"

#include <math.h>
#include <string.h>

#include "control/ctrl_params.h"
#include "control/flight_ctrl.h"
#include "dronelink/dronelink_config.h"
#include "dronelink/dronelink_mavlink.h"

/** Mensagens do voo enviadas por chamada de dronelink_telemetry_tick(). */
#define TEXTS_PER_TICK 2U
/** Período de ATTITUDE e LOCAL_POSITION_NED, em ms (5 Hz). */
#define STATE_PERIOD_MS 200U

/** Último envio da atitude/posição. */
static uint32_t last_state_ms;
/** Último envio do DRONE_CTRL_DEBUG. */
static uint32_t last_debug_ms;
/** Passo do último DRONE_CTRL_DEBUG (para não repetir o mesmo passo). */
static uint32_t last_debug_step;

/**
 * @brief z do LOCAL_POSITION_NED relativo ao chão (0 no chão, -h a h metros).
 *
 * O z do Kalman é a leitura do MTF-01, que no chão já vale a altura do
 * sensor (cerca de -8 cm). No ar, desconta o chão registrado ao armar; no chão,
 * desarmado, o chão é a própria medida (altura 0). Assim a Pi recebe a mesma
 * altura que o operador pede ("takeoff 30" = 30 cm) e que o console mostra.
 *
 * @param[in] t  Telemetria do passo.
 * @return z NED relativo ao chão, em m.
 */
static float z_over_ground(const flight_telemetry_t *t) {
  const float z = t->x_hat[LQG_X_Z];
  if (t->state == FLIGHT_STATE_STANDBY || t->state == FLIGHT_STATE_BOOT) {
    return z - t->y[LQG_Y_Z];
  }
  return isfinite(t->z_ground) ? z - t->z_ground : z;
}

/** @brief ATTITUDE e LOCAL_POSITION_NED com a estimativa do Kalman. */
static void send_state(const flight_telemetry_t *t) {
  mavlink_message_t m;
  mavlink_msg_attitude_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &m, t->time_ms,
                            t->x_hat[LQG_X_PHI], t->x_hat[LQG_X_THETA], t->x_hat[LQG_X_PSI],
                            t->x_hat[LQG_X_P], t->x_hat[LQG_X_Q], t->x_hat[LQG_X_R]);
  dronelink_send(&m, DRONELINK_TX_NORMAL);
  mavlink_msg_local_position_ned_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &m,
                                      t->time_ms, t->x_hat[LQG_X_X], t->x_hat[LQG_X_Y],
                                      z_over_ground(t), t->x_hat[LQG_X_VX],
                                      t->x_hat[LQG_X_VY], t->x_hat[LQG_X_VZ]);
  dronelink_send(&m, DRONELINK_TX_NORMAL);
}

/** @brief DRONE_CTRL_DEBUG com o passo completo do LQG. */
static void send_debug(const flight_telemetry_t *t) {
  uint16_t throttle[LQG_NU];
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    const float v = t->throttle[i] * 10000.0F;
    throttle[i] = (uint16_t)(v < 0.0F ? 0.0F : (v > 10000.0F ? 10000.0F : v));
  }
  mavlink_message_t m;
  mavlink_msg_drone_ctrl_debug_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &m,
                                    t->time_ms, t->step, t->y, t->x_hat, t->du, t->ref,
                                    throttle, t->state, t->meas_flags, t->flags);
  dronelink_send(&m, DRONELINK_TX_NORMAL);
}

void dronelink_telemetry_init(void) {
  last_state_ms = 0U;
  last_debug_ms = 0U;
  last_debug_step = 0U;
}

/**
 * @brief Envia à Pi, como STATUSTEXT, as mensagens do voo ainda não entregues.
 *
 * No máximo ::TEXTS_PER_TICK por chamada, para nunca encher a fila normal de
 * uma vez. FAILSAFE vai como crítico; o resto, como informação.
 */
static void send_flight_texts(void) {
  char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1U];
  for (unsigned n = 0U; n < TEXTS_PER_TICK && flight_ctrl_take_text(text, sizeof(text)); ++n) {
    const uint8_t severity = strncmp(text, "FAILSAFE", 8) == 0 ? MAV_SEVERITY_CRITICAL
                                                                : MAV_SEVERITY_INFO;
    mavlink_message_t m;
    mavlink_msg_statustext_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &m, severity,
                                text, 0U, 0U);
    /* O motivo de um FAILSAFE não pode se perder na fila normal, que a
       telemetria de 50 Hz quase enche; ele é raro e vai na fila das respostas. */
    dronelink_send(&m, severity == MAV_SEVERITY_CRITICAL ? DRONELINK_TX_CRITICAL
                                                          : DRONELINK_TX_NORMAL);
  }
}

void dronelink_telemetry_tick(uint32_t now_ms) {
  send_flight_texts();
  const bool state_due = now_ms - last_state_ms >= STATE_PERIOD_MS;
  ctrl_params_t p;
  ctrl_params_snapshot(&p);
  const float rate = p.debug_rate_hz;
  const bool debug_on = rate > 0.0F;
  const uint32_t period = debug_on ? (uint32_t)(1000.0F / rate) : 0U;
  const bool debug_due = debug_on && now_ms - last_debug_ms >= period;
  if (!state_due && !debug_due) return;

  static flight_telemetry_t t;
  flight_ctrl_get_telemetry(&t);
  if (state_due) {
    last_state_ms = now_ms;
    send_state(&t);
  }
  if (debug_due && t.step != last_debug_step) {
    last_debug_ms = now_ms;
    last_debug_step = t.step;
    send_debug(&t);
  }
}
