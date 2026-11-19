/**
 * @file power_cal.c
 * @brief Conversão das leituras de potência (contrato em power_cal.h).
 */
#include "power_cal.h"

#include <math.h>

void power_cal_default(power_cal_t *config) {
  config->vbat_gain = 0.0F;
  config->current_a_per_v = 0.0F;
  config->current_offset_v = 0.0F;
  config->vbat_min_v = 0.0F;
  config->vbat_valid = false;
  config->current_valid = false;
}

void power_cal_apply(const power_cal_t *config, power_sample_t *s) {
  s->vbat_calibrated = s->vbat_read && config->vbat_valid && (config->vbat_gain > 0.0F);
  s->current_calibrated = s->current_read && config->current_valid && (config->current_a_per_v > 0.0F);
  s->vbat_v = s->vbat_calibrated ? (s->vbat_pin_v * config->vbat_gain) : NAN;
  s->current_a =
      s->current_calibrated ? ((s->current_pin_v - config->current_offset_v) * config->current_a_per_v) : NAN;
}

bool power_cal_arm_allowed(const power_cal_t *config, const power_sample_t *s) {
  bool ok = true;
  if (config->vbat_min_v > 0.0F) {
    ok = s->vbat_calibrated && (s->vbat_v >= config->vbat_min_v);
  }
  return ok;
}
