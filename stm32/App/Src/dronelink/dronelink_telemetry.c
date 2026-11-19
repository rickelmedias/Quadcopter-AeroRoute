/**
 * @file dronelink_telemetry.c
 * @brief Telemetria do controle para a Pi (contrato em dronelink/dronelink_telemetry.h).
 */
#include "dronelink/dronelink_telemetry.h"

#include "app/control_task.h"
#include "control/lqg_config.h"
#include "dronelink/dronelink_config.h"
#include "dronelink/dronelink_mavlink.h"
#include "dronelink/flight_command_api.h"

#include <math.h>
#include <string.h>

/** Mensagens do voo enviadas por chamada de dronelink_telemetry_tick(). */
#define TEXTS_PER_TICK 2U

/** Último envio da atitude/posição e do DRONE_CTRL_DEBUG. */
static uint32_t last_state_ms;
static uint32_t last_debug_ms;
/** Passo do último DRONE_CTRL_DEBUG (para não repetir o mesmo passo). */
static uint32_t last_debug_step;

/** @brief ATTITUDE e LOCAL_POSITION_NED com a estimativa do EKF (z relativo ao chão do armamento). */
static void send_state(uint32_t now_ms) {
  flight_snapshot_t f;
  flight_get_snapshot(&f);
  mavlink_message_t m;
  mavlink_msg_attitude_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &m, now_ms,
                            f.attitude.roll, f.attitude.pitch, f.attitude.yaw, f.roll_speed,
                            f.pitch_speed, f.yaw_speed);
  dronelink_send(&m, DRONELINK_TX_NORMAL);
  mavlink_msg_local_position_ned_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &m, now_ms,
                                      f.x, f.y, f.z, f.vx, f.vy, f.vz);
  dronelink_send(&m, DRONELINK_TX_NORMAL);
}

/** @brief Bits LQG_TLM_* -> DRONE_MEAS_FLAG (atitude e giro, distância, fluxo). */
static uint8_t meas_flags(uint8_t tlm) {
  uint8_t f = 0U;
  f |= ((tlm & LQG_TLM_ATTITUDE) != 0U) ? (uint8_t)(DRONE_MEAS_FLAG_ATTITUDE | DRONE_MEAS_FLAG_GYRO) : 0U;
  f |= ((tlm & LQG_TLM_RANGE) != 0U) ? (uint8_t)DRONE_MEAS_FLAG_RANGE : 0U;
  f |= ((tlm & LQG_TLM_FLOW) != 0U) ? (uint8_t)DRONE_MEAS_FLAG_FLOW : 0U;
  return f;
}

/** @brief DRONE_CTRL_DEBUG com o passo do controle (y, x̂, Δω aplicado, referência, motores). */
static void send_debug(const control_snapshot_t *s) {
  float du[4];
  uint16_t throttle[4];
  static const uint8_t motor_of_input[4] = LQG_MOTOR_OF_INPUT;
  for (uint32_t i = 0U; i < 4U; ++i) {
    const float pm = s->permille[motor_of_input[i]];
    du[i] = (pm - s->base) * LQG_W_PER_PERMILLE;
    const float v = s->permille[i] * 10.0F; /* ‰ -> 0..10000 */
    throttle[i] = (uint16_t)((v < 0.0F) ? 0.0F : ((v > 10000.0F) ? 10000.0F : v));
  }
  const float ref[4] = {s->nav.ref[0], s->nav.ref[1], -s->nav.ref[2], s->nav.ref[3]};
  flight_snapshot_t f;
  flight_get_snapshot(&f);
  mavlink_message_t m;
  mavlink_msg_drone_ctrl_debug_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &m, s->t_ms,
                                    s->step, s->y, s->x_hat, du, ref, throttle, f.state,
                                    meas_flags(s->flags),
                                    (uint8_t)((((s->flags & LQG_TLM_SATURATED) != 0U) ? DRONE_CTRL_FLAG_SATURATED : 0U) |
                                              (s->armed ? DRONE_CTRL_FLAG_CONTROL_ACTIVE : 0U) |
                                              (s->simulated ? DRONE_CTRL_FLAG_DRY_RUN : 0U)));
  dronelink_send(&m, DRONELINK_TX_NORMAL);
}

void dronelink_telemetry_init(void) {
  last_state_ms = 0U;
  last_debug_ms = 0U;
  last_debug_step = 0U;
}

/**
 * @brief Envia à Pi, como STATUSTEXT, os avisos do voo ainda não entregues.
 *
 * No máximo ::TEXTS_PER_TICK por chamada. Cortes vão como crítico e na fila das
 * respostas (a telemetria quase enche a normal); o resto, como informação.
 */
static void send_flight_texts(void) {
  char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1U];
  bool critical = false;
  for (unsigned n = 0U; (n < TEXTS_PER_TICK) && flight_take_text(text, sizeof(text), &critical);
       ++n) {
    mavlink_message_t m;
    mavlink_msg_statustext_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &m,
                                critical ? MAV_SEVERITY_CRITICAL : MAV_SEVERITY_INFO, text, 0U,
                                0U);
    dronelink_send(&m, critical ? DRONELINK_TX_CRITICAL : DRONELINK_TX_NORMAL);
  }
}

void dronelink_telemetry_tick(uint32_t now_ms) {
  send_flight_texts();
  if ((now_ms - last_state_ms) >= DRONELINK_STATE_PERIOD_MS) {
    last_state_ms = now_ms;
    send_state(now_ms);
  }
  if ((now_ms - last_debug_ms) >= DRONELINK_DEBUG_PERIOD_MS) {
    control_snapshot_t s;
    control_get_snapshot(&s);
    if (s.step != last_debug_step) {
      last_debug_ms = now_ms;
      last_debug_step = s.step;
      send_debug(&s);
    }
  }
}
