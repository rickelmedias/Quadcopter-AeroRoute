/**
 * @file test_power_cal.c
 * @brief Conversão de potência e bloqueio de armamento por tensão.
 */
#include "power_cal.h"

#include <assert.h>
#include <math.h>

int main(void) {
  power_cal_t cal;
  power_cal_default(&cal);
  power_sample_t s = {1.5F, 0.6F, 0.0F, 0.0F, true, true, false, false};

  power_cal_apply(&cal, &s);
  assert(!s.vbat_calibrated && !s.current_calibrated && isnan(s.vbat_v) && isnan(s.current_a));
  assert(power_cal_arm_allowed(&cal, &s)); /* sem limite configurado */

  cal.vbat_gain = 11.0F;
  cal.current_a_per_v = 50.0F;
  cal.current_offset_v = 0.5F;
  cal.vbat_valid = cal.current_valid = true;
  power_cal_apply(&cal, &s);
  assert(s.vbat_calibrated && fabsf(s.vbat_v - 16.5F) < 1e-5F);
  assert(s.current_calibrated && fabsf(s.current_a - 5.0F) < 1e-5F);

  cal.vbat_min_v = 14.0F;
  assert(power_cal_arm_allowed(&cal, &s));
  cal.vbat_min_v = 17.0F;
  assert(!power_cal_arm_allowed(&cal, &s));

  /* Limite configurado e leitura inválida: não arma. */
  cal.vbat_min_v = 10.0F;
  s.vbat_read = false;
  power_cal_apply(&cal, &s);
  assert(!s.vbat_calibrated && !power_cal_arm_allowed(&cal, &s));
  return 0;
}
