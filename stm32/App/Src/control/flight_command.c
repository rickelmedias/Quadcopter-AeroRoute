/**
 * @file flight_command.c
 * @brief Implementa o contrato do DroneLink (dronelink/flight_command_api.h)
 *        com o controle de voo (control/flight_ctrl.h).
 *
 * Tudo o que chega por aqui veio da Raspberry Pi, então o voo é armado com
 * ::FLIGHT_ORIGIN_PI: a supervisão de enlace e de autorização do DroneLink
 * vale para ele.
 */
#include "dronelink/flight_command_api.h"

#include "control/ctrl_params.h"
#include "control/flight_ctrl.h"
#include "control/meas.h"
#include "flight_config.h"

#include <string.h>

flight_request_result_t flight_request_arm(bool arm) {
  if (arm) {
    return flight_ctrl_arm(FLIGHT_ORIGIN_PI);
  }
  flight_ctrl_disarm("desarmado pela Pi");
  const flight_request_result_t ok = {FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE};
  return ok;
}

flight_request_result_t flight_request_leg(const flight_leg_t *leg) {
  return flight_ctrl_leg(leg);
}

flight_request_result_t flight_request_cancel(void) {
  return flight_ctrl_cancel();
}

flight_request_result_t flight_request_hold(flight_reason_t reason) {
  return flight_ctrl_hold(reason);
}

flight_request_result_t flight_request_land(flight_reason_t reason) {
  return flight_ctrl_land(reason);
}

void flight_get_snapshot(flight_snapshot_t *snapshot) {
  if (snapshot == NULL) {
    return;
  }
  flight_telemetry_t t;
  flight_ctrl_get_telemetry(&t);
  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->state = t.state;
  snapshot->armed = flight_ctrl_is_armed();
  snapshot->airborne = t.state == FLIGHT_STATE_TAKEOFF || t.state == FLIGHT_STATE_HOLD ||
                       t.state == FLIGHT_STATE_EXECUTING_LEG || t.state == FLIGHT_STATE_LAND;
  snapshot->commanded_by_pi = snapshot->armed && flight_ctrl_origin() == FLIGHT_ORIGIN_PI;
  snapshot->attitude_valid = (t.meas_flags & MEAS_ATTITUDE) != 0U;
  snapshot->altitude_valid = (t.meas_flags & MEAS_RANGE) != 0U;
  /* x e y só são integrados a partir do fluxo: valem enquanto ele chega. */
  snapshot->position_valid = snapshot->armed && (t.meas_flags & MEAS_FLOW) != 0U;
  snapshot->x = t.x_hat[LQG_X_X];
  snapshot->y = t.x_hat[LQG_X_Y];
  snapshot->z = t.x_hat[LQG_X_Z];
  snapshot->vx = t.x_hat[LQG_X_VX];
  snapshot->vy = t.x_hat[LQG_X_VY];
  snapshot->vz = t.x_hat[LQG_X_VZ];
  snapshot->attitude.roll = t.x_hat[LQG_X_PHI];
  snapshot->attitude.pitch = t.x_hat[LQG_X_THETA];
  snapshot->attitude.yaw = t.x_hat[LQG_X_PSI];
  memcpy(snapshot->q, t.q, sizeof(snapshot->q));
  snapshot->roll_speed = t.x_hat[LQG_X_P];
  snapshot->pitch_speed = t.x_hat[LQG_X_Q];
  snapshot->yaw_speed = t.x_hat[LQG_X_R];
}

bool flight_actuation_is_available(void) {
  ctrl_params_t p;
  ctrl_params_snapshot(&p);
  return LQG_FLIGHT_ENABLED && p.dry_run < 0.5F;
}

bool flight_poll_leg_result(flight_request_status_t *status, flight_reason_t *reason) {
  flight_leg_event_t event;
  if (!flight_ctrl_take_leg_event(&event)) {
    return false;
  }
  *status = event.status;
  *reason = event.reason;
  return true;
}
