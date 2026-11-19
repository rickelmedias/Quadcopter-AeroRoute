/** @file test_boot_values.c @brief Os valores de boot (LQG_BOOT_*) passam nos limites do controle. */
#include "control/lqg_config.h"
#include "control/lqg_ctrl.h"

#include <assert.h>
#include <stdio.h>

int main(void) {
  const lqg_params_t p = LQG_BOOT_PARAMS;
  const lqg_gains_t g = LQG_BOOT_GAINS;
  lqg_ctrl_t c;
  lqg_ctrl_init(&c);
  assert(lqg_ctrl_set_params(&c, &p) && lqg_ctrl_set_gains(&c, &g));
  c.ekf_on = true;
  /* Com altura 10×/5× o ajuste do hover vale inteiro (limite ~131 ‰/(m·s)). */
  assert(lqg_ctrl_trim_ki(&c) == p.trim_ki);
  puts("test_boot_values ok");
  return 0;
}
