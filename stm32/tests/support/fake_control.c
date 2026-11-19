/**
 * @file fake_control.c
 * @brief Versões síncronas de app/control_task.h, motors.h e power.h para os testes
 *        do DroneLink: rodam o lqg_ctrl e o lqg_sim reais, 10 ms por fake_step().
 */
#include "support/fake_control.h"

#include "dronelink/flight_command_api.h"

#include <string.h>

lqg_ctrl_t ctrl;
lqg_sim_t sim;
uint32_t now_ms = 1000U;
motors_owner_t owner = MOTORS_OWNER_NONE;
uint32_t keepalives;
bool fake_runs_executor = true;
control_snapshot_t snap;
static uint8_t last_fail;

uint32_t HAL_GetTick(void) {
  return now_ms;
}

/* ---- motores e bateria ---------------------------------------------------- */
motors_owner_t motors_owner(void) {
  return owner;
}
void motors_stop(const char *why) {
  (void)why;
  owner = MOTORS_OWNER_NONE;
}
void motors_bench_keepalive(void) {
  ++keepalives;
}
void power_get(power_sample_t *out) {
  memset(out, 0, sizeof(*out));
}
void power_get_cal(power_cal_t *out) {
  power_cal_default(out);
}

/* ---- task de controle, síncrona ------------------------------------------- */
bool control_arm(void) {
  if (owner != MOTORS_OWNER_NONE) {
    return false;
  }
  owner = MOTORS_OWNER_CONTROL;
  return true;
}
bool control_takeoff_to(float height_m) {
  const bool ok = lqg_ctrl_takeoff(&ctrl);
  if (ok && (height_m > 0.0F)) {
    ctrl.h_goal = height_m;
  }
  return ok;
}
bool control_takeoff(void) {
  return control_takeoff_to(0.0F);
}
bool control_land(void) {
  return lqg_ctrl_land(&ctrl);
}
bool control_goto(float x, float y, float h, float psi, float speed) {
  return lqg_ctrl_goto(&ctrl, x, y, h, psi, speed);
}
bool control_hold(void) {
  return lqg_ctrl_hold(&ctrl);
}
void control_get_snapshot(control_snapshot_t *out) {
  *out = snap;
}

void fake_step(void) {
  if ((owner != MOTORS_OWNER_CONTROL) && (lqg_ctrl_state(&ctrl) != LQG_STATE_IDLE)) {
    lqg_ctrl_abort(&ctrl);
  } else if ((owner == MOTORS_OWNER_CONTROL) && (lqg_ctrl_state(&ctrl) == LQG_STATE_IDLE)) {
    (void)lqg_ctrl_arm(&ctrl);
    lqg_sim_init(&sim, 560.0F);
    last_fail = LQG_FAIL_NONE;
  }
  lqg_sensors_t in;
  lqg_out_t out;
  lqg_sim_sensors(&sim, &in);
  lqg_ctrl_step(&ctrl, &in, &out);
  if (out.fail != LQG_FAIL_NONE) {
    last_fail = out.fail;
  }
  if (out.stop != NULL) {
    owner = MOTORS_OWNER_NONE;
  } else if ((owner == MOTORS_OWNER_CONTROL) && (out.state >= LQG_STATE_SPOOL)) {
    lqg_sim_step(&sim, out.permille);
  }
  const uint32_t note_seq = snap.note_seq;
  const uint32_t steps = snap.step;
  memset(&snap, 0, sizeof(snap));
  snap.note_seq = note_seq;
  snap.step = steps + 1U;
  snap.t_ms = now_ms;
  snap.state = lqg_ctrl_state(&ctrl);
  snap.armed = owner == MOTORS_OWNER_CONTROL;
  snap.airborne = ctrl.airborne;
  snap.flags = out.flags;
  snap.fail = last_fail;
  snap.note = out.stop != NULL ? out.stop : out.note;
  snap.note_seq += (snap.note != NULL) ? 1U : 0U;
  lqg_ctrl_nav(&ctrl, &snap.nav);
  now_ms += 10U;
  if (fake_runs_executor) {
    flight_command_tick(now_ms);
  }
}

