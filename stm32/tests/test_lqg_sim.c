/**
 * @file test_lqg_sim.c
 * @brief Planta do FLT_SIM: o controle decola, paira, vai a um alvo e pousa sobre ela.
 */
#include "control/lqg_ctrl.h"
#include "control/lqg_sim.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static lqg_out_t tick(lqg_ctrl_t *c, lqg_sim_t *s) {
  lqg_sensors_t in;
  lqg_out_t out;
  lqg_sim_sensors(s, &in);
  lqg_ctrl_step(c, &in, &out);
  if ((out.stop == NULL) && (out.state >= LQG_STATE_SPOOL)) {
    lqg_sim_step(s, out.permille);
  }
  return out;
}

int main(void) {
  lqg_ctrl_t c;
  lqg_sim_t s;
  lqg_ctrl_init(&c);
  lqg_sim_init(&s, 560.0F);
  assert(lqg_ctrl_arm(&c));
  for (uint32_t i = 0U; i < 60U; ++i) {
    assert(tick(&c, &s).state == LQG_STATE_ARMED);
  }
  assert(lqg_ctrl_takeoff(&c));
  lqg_out_t o = {0};
  for (uint32_t i = 0U; i < 2500U; ++i) {
    o = tick(&c, &s);
    assert(o.stop == NULL);
  }
  assert(o.state == LQG_STATE_HOVER);
  assert(fabsf(-(float)s.x[2] - 0.30F) < 0.03F);
  assert((o.flags & 0x40U) != 0U); /* XY no controle */
  assert(lqg_ctrl_goto(&c, 0.3F, 0.3F, 0.4F, 0.0F, 0.1F));
  for (uint32_t i = 0U; i < 2500U; ++i) {
    o = tick(&c, &s);
    assert(o.stop == NULL);
  }
  assert(fabsf((float)s.x[0] - 0.3F) < 0.05F && fabsf((float)s.x[1] - 0.3F) < 0.05F);
  assert(lqg_ctrl_land(&c));
  for (uint32_t i = 0U; (i < 3000U) && (o.stop == NULL); ++i) {
    o = tick(&c, &s);
  }
  assert(o.fail == LQG_FAIL_LANDED);
  puts("test_lqg_sim ok");
  return 0;
}
