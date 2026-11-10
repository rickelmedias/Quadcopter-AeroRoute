/**
 * @file throttle.c
 * @brief Rampas e dessaturação da aceleração (contrato em control/throttle.h).
 */
#include "control/throttle.h"

#include <math.h>

float throttle_ramp_down(uint32_t elapsed_ms, uint32_t total_ms) {
  if (total_ms == 0U || elapsed_ms >= total_ms) return 0.0F;
  return 1.0F - (float)elapsed_ms / (float)total_ms;
}

float throttle_ramp_up(uint32_t elapsed_ms, uint32_t total_ms) {
  return 1.0F - throttle_ramp_down(elapsed_ms, total_ms);
}

float throttle_blend(float throttle, float idle, float factor) {
  if (!(throttle > idle)) return throttle;
  if (factor <= 0.0F) return idle;
  if (factor >= 1.0F) return throttle;
  return idle + (throttle - idle) * factor;
}

void throttle_blend_common(float *throttle, unsigned count, float idle, float factor) {
  if (count == 0U) return;
  float mean = 0.0F;
  for (unsigned i = 0U; i < count; ++i) mean += throttle[i];
  mean /= (float)count;
  if (!(mean > idle) || factor >= 1.0F) return;
  const float f = factor > 0.0F ? factor : 0.0F;
  const float shift = (idle + (mean - idle) * f) - mean;
  for (unsigned i = 0U; i < count; ++i) throttle[i] += shift;
}

bool throttle_desaturate(float *throttle, unsigned count, float min, float max) {
  if (count == 0U) return false;
  bool changed = false;
  float hi = -INFINITY, lo = INFINITY;
  for (unsigned i = 0U; i < count; ++i) {
    if (!isfinite(throttle[i])) {
      throttle[i] = min;
      changed = true;
    }
    hi = fmaxf(hi, throttle[i]);
    lo = fminf(lo, throttle[i]);
  }
  /* O teto tem prioridade: um motor acima do máximo desloca todos para baixo. */
  float shift = 0.0F;
  if (hi > max) {
    shift = max - hi;
  } else if (lo < min) {
    shift = fminf(min - lo, max - hi);
  }
  for (unsigned i = 0U; i < count; ++i) {
    float v = throttle[i] + shift;
    if (v < min) v = min;
    if (v > max) v = max;
    if (v != throttle[i]) changed = true;
    throttle[i] = v;
  }
  return changed;
}
