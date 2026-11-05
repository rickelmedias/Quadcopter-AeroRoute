/**
 * @file test_throttle.c
 * @brief Rampas de partida e de pouso e dessaturação da aceleração.
 */
#include "control/throttle.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

/** @brief Rampa de pouso: de 52% até a marcha lenta, sem degrau. */
static void test_ramp_down(void) {
  assert(throttle_ramp_down(0U, 1500U) == 1.0F);
  assert(fabsf(throttle_ramp_down(750U, 1500U) - 0.5F) < 1e-6F);
  assert(throttle_ramp_down(1500U, 1500U) == 0.0F);
  assert(throttle_ramp_down(9999U, 1500U) == 0.0F);
  assert(throttle_ramp_down(0U, 0U) == 0.0F);
  assert(fabsf(throttle_blend(0.52F, 0.05F, 0.5F) - 0.285F) < 1e-6F);
  float last = 0.52F;
  for (unsigned ms = 0U; ms <= 1500U; ms += 10U) {
    const float thr = throttle_blend(0.52F, 0.05F, throttle_ramp_down(ms, 1500U));
    assert(thr <= last + 1e-6F && thr >= 0.05F && last - thr < 0.01F);
    last = thr;
  }
  assert(throttle_blend(0.03F, 0.05F, 0.5F) == 0.03F);
}

/** @brief Rampa de partida: da marcha lenta até o hover, sem degrau. */
static void test_ramp_up(void) {
  assert(throttle_ramp_up(0U, 2000U) == 0.0F);
  assert(throttle_ramp_up(2000U, 2000U) == 1.0F);
  assert(throttle_ramp_up(0U, 0U) == 1.0F);
  float last = 0.05F;
  for (unsigned ms = 0U; ms <= 2000U; ms += 10U) {
    const float thr = throttle_blend(0.48F, 0.05F, throttle_ramp_up(ms, 2000U));
    assert(thr >= last - 1e-6F && thr - last < 0.005F);
    last = thr;
  }
  assert(fabsf(last - 0.48F) < 1e-6F);
}

/** @brief Dessaturação preserva a diferença entre os motores. */
static void test_desaturate(void) {
  /* Caso do ensaio: traseira pedindo 66%, frente 45%, teto 60%. */
  float t[4] = {0.66F, 0.45F, 0.66F, 0.45F};
  assert(throttle_desaturate(t, 4U, 0.03F, 0.60F));
  assert(fabsf(t[0] - 0.60F) < 1e-6F && fabsf(t[1] - 0.39F) < 1e-6F);
  assert(fabsf((t[0] - t[1]) - 0.21F) < 1e-6F);   /* a diferença ficou */

  float ok[4] = {0.40F, 0.42F, 0.44F, 0.41F};
  assert(!throttle_desaturate(ok, 4U, 0.03F, 0.60F));
  assert(ok[2] == 0.44F);

  float low[4] = {0.01F, 0.10F, 0.10F, 0.10F};      /* abaixo do mínimo: sobe todos */
  assert(throttle_desaturate(low, 4U, 0.03F, 0.60F));
  assert(fabsf(low[0] - 0.03F) < 1e-6F && fabsf(low[1] - 0.12F) < 1e-6F);

  float wide[4] = {0.90F, 0.0F, 0.50F, 0.50F};      /* não cabe: teto manda, piso limita */
  assert(throttle_desaturate(wide, 4U, 0.03F, 0.60F));
  assert(wide[0] == 0.60F && wide[1] == 0.03F && fabsf(wide[2] - 0.20F) < 1e-6F);

  float bad[4] = {NAN, 0.40F, 0.40F, 0.40F};
  assert(throttle_desaturate(bad, 4U, 0.03F, 0.60F) && bad[0] == 0.03F);
}

int main(void) {
  test_ramp_down();
  test_ramp_up();
  test_desaturate();
  printf("test_throttle: ok\n");
  return 0;
}
