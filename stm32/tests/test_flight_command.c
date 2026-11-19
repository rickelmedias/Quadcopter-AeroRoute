/**
 * @file test_flight_command.c
 * @brief Executor de trechos do DroneLink sobre o controle real e a planta do FLT_SIM.
 *
 * As funções de app/control_task.h, motors.h e power.h são trocadas por versões
 * síncronas (support/fake_control.c) que rodam o lqg_ctrl e o lqg_sim, 10 ms por passo.
 * Prova a sequência armar -> decolar -> girar -> andar -> ir a ponto -> pousar,
 * as recusas e o fim por tempo esgotado; não testa o enlace serial.
 */
#include "dronelink/flight_command_api.h"
#include "support/fake_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static flight_leg_t make_leg(uint8_t action, uint8_t frame, float x, float y, float z, float yaw) {
  flight_leg_t l;
  memset(&l, 0, sizeof(l));
  l.action = action;
  l.frame = frame;
  l.x = x;
  l.y = y;
  l.z = z;
  l.yaw = yaw;
  l.timeout_ms = 60000U;
  return l;
}

/** @brief Roda até o trecho terminar; devolve a situação final. */
static flight_request_status_t run_leg(uint32_t max_steps, flight_reason_t *reason) {
  flight_request_status_t st = FLIGHT_REQUEST_FAILED;
  for (uint32_t i = 0U; i < max_steps; ++i) {
    fake_step();
    if (flight_poll_leg_result(&st, reason)) {
      return st;
    }
  }
  assert(!"trecho não terminou");
  return st;
}

static uint8_t state_now(void) {
  flight_snapshot_t f;
  flight_get_snapshot(&f);
  return f.state;
}

int main(void) {
  lqg_ctrl_init(&ctrl);
  for (uint32_t i = 0U; i < 5U; ++i) {
    fake_step();
  }
  assert(state_now() == FLIGHT_STATE_STANDBY);
  flight_reason_t why = FLIGHT_REASON_NONE;

  /* Girar no chão: estado errado. */
  flight_leg_t l = make_leg(FLIGHT_ACTION_ROTATE_RELATIVE, FLIGHT_FRAME_BODY_FRD, 0, 0, 0, 0.5F);
  flight_request_result_t r = flight_request_leg(&l);
  assert(r.status == FLIGHT_REQUEST_DENIED && r.reason == FLIGHT_REASON_BAD_STATE);

  /* Armar e decolar a 30 cm logo em seguida (o chão ainda está sendo medido). */
  assert(flight_request_arm(true).status == FLIGHT_REQUEST_ACCEPTED);
  assert(owner == MOTORS_OWNER_CONTROL);
  l = make_leg(FLIGHT_ACTION_TAKEOFF_RELATIVE, FLIGHT_FRAME_LOCAL_FRD, 0, 0, -0.30F, 0);
  assert(flight_request_leg(&l).status == FLIGHT_REQUEST_IN_PROGRESS);
  l = make_leg(FLIGHT_ACTION_TAKEOFF_RELATIVE, FLIGHT_FRAME_LOCAL_FRD, 0, 0, -0.30F, 0);
  r = flight_request_leg(&l);
  assert(r.status == FLIGHT_REQUEST_TEMPORARILY_REJECTED && r.reason == FLIGHT_REASON_BUSY);
  fake_step();
  assert(state_now() == FLIGHT_STATE_TAKEOFF);
  const uint32_t before = keepalives;
  assert(run_leg(4000U, &why) == FLIGHT_REQUEST_ACCEPTED);
  assert(keepalives > before); /* voo da Pi: o prazo do árbitro vem do executor */
  assert(state_now() == FLIGHT_STATE_HOLD);
  assert(fabsf(-(float)sim.x[2] - 0.30F) < 0.06F);
  /* A origem XY do EKF é onde o fluxo passou a valer (acima de 10 cm), não onde armou. */
  const float ox = (float)sim.x[0] - snap.nav.pos[0];
  const float oy = (float)sim.x[1] - snap.nav.pos[1];

  /* Girar 90 graus para a direita. */
  l = make_leg(FLIGHT_ACTION_ROTATE_RELATIVE, FLIGHT_FRAME_BODY_FRD, 0, 0, 0, 1.5707963F);
  assert(flight_request_leg(&l).status == FLIGHT_REQUEST_IN_PROGRESS);
  assert(state_now() == FLIGHT_STATE_EXECUTING_LEG);
  assert(run_leg(4000U, &why) == FLIGHT_REQUEST_ACCEPTED);
  assert(fabsf((float)sim.x[5] - 1.5707963F) < 0.08F);

  /* 30 cm para a frente do nariz (agora é +y local). */
  l = make_leg(FLIGHT_ACTION_TRANSLATE_RELATIVE, FLIGHT_FRAME_BODY_FRD, 0.30F, 0, 0, 0);
  assert(flight_request_leg(&l).status == FLIGHT_REQUEST_IN_PROGRESS);
  assert(run_leg(4000U, &why) == FLIGHT_REQUEST_ACCEPTED);
  assert(fabsf((float)sim.x[1] - oy - 0.30F) < 0.06F && fabsf((float)sim.x[0] - ox) < 0.06F);

  /* Ir ao ponto local (0, 0) a 40 cm, de frente para o norte. */
  l = make_leg(FLIGHT_ACTION_GOTO_LOCAL, FLIGHT_FRAME_LOCAL_FRD, 0, 0, -0.40F, 0);
  assert(flight_request_leg(&l).status == FLIGHT_REQUEST_IN_PROGRESS);
  assert(run_leg(6000U, &why) == FLIGHT_REQUEST_ACCEPTED);
  assert(fabsf((float)sim.x[0] - ox) < 0.06F && fabsf((float)sim.x[1] - oy) < 0.06F);
  assert(fabsf(-(float)sim.x[2] - 0.40F) < 0.06F && fabsf((float)sim.x[5]) < 0.08F);

  /* Fora dos limites e tempo esgotado. */
  l = make_leg(FLIGHT_ACTION_TRANSLATE_RELATIVE, FLIGHT_FRAME_BODY_FRD, 0, 0, -1.0F, 0);
  r = flight_request_leg(&l);
  assert(r.status == FLIGHT_REQUEST_DENIED && r.reason == FLIGHT_REASON_OUT_OF_BOUNDS);
  l = make_leg(FLIGHT_ACTION_TRANSLATE_RELATIVE, FLIGHT_FRAME_BODY_FRD, 0.5F, 0, 0, 0);
  l.timeout_ms = 300U;
  assert(flight_request_leg(&l).status == FLIGHT_REQUEST_IN_PROGRESS);
  assert(run_leg(100U, &why) == FLIGHT_REQUEST_FAILED && why == FLIGHT_REASON_TIMEOUT);
  for (uint32_t i = 0U; i < 500U; ++i) {
    fake_step();
  }
  assert(state_now() == FLIGHT_STATE_HOLD);

  /* Pousar: desarma sozinho e o voo deixa de ser da Pi. */
  assert(flight_request_land(FLIGHT_REASON_NONE).status == FLIGHT_REQUEST_ACCEPTED);
  for (uint32_t i = 0U; (i < 4000U) && (owner != MOTORS_OWNER_NONE); ++i) {
    fake_step();
  }
  fake_step();
  assert(owner == MOTORS_OWNER_NONE && state_now() == FLIGHT_STATE_STANDBY);
  flight_snapshot_t f;
  flight_get_snapshot(&f);
  assert(!f.commanded_by_pi);
  puts("test_flight_command ok");
  return 0;
}
