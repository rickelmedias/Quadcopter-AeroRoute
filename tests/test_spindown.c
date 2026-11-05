/**
 * @file test_spindown.c
 * @brief Descida suave da aceleração no fim do pouso.
 */
#include "control/spindown.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void) {
  assert(spindown_factor(0U, 1500U) == 1.0F);
  assert(fabsf(spindown_factor(750U, 1500U) - 0.5F) < 1e-6F);
  assert(spindown_factor(1500U, 1500U) == 0.0F);
  assert(spindown_factor(9999U, 1500U) == 0.0F);
  assert(spindown_factor(0U, 0U) == 0.0F);  /* sem rampa: corta */

  /* 52% com marcha lenta de 5%: desce em linha reta até 5%. */
  assert(spindown_throttle(0.52F, 0.05F, 1.0F) == 0.52F);
  assert(fabsf(spindown_throttle(0.52F, 0.05F, 0.5F) - 0.285F) < 1e-6F);
  assert(spindown_throttle(0.52F, 0.05F, 0.0F) == 0.05F);
  float last = 0.52F;
  for (unsigned ms = 0U; ms <= 1500U; ms += 10U) {  /* monotônica, sem degrau */
    const float thr = spindown_throttle(0.52F, 0.05F, spindown_factor(ms, 1500U));
    assert(thr <= last + 1e-6F && thr >= 0.05F);
    assert(last - thr < 0.01F);
    last = thr;
  }
  assert(spindown_throttle(0.03F, 0.05F, 0.5F) == 0.03F);  /* já abaixo da marcha lenta */
  printf("test_spindown: ok\n");
  return 0;
}
