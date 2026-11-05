/**
 * @file spindown.c
 * @brief Descida suave da aceleração no pouso (contrato em control/spindown.h).
 */
#include "control/spindown.h"

float spindown_factor(uint32_t elapsed_ms, uint32_t total_ms) {
  if (total_ms == 0U || elapsed_ms >= total_ms) return 0.0F;
  return 1.0F - (float)elapsed_ms / (float)total_ms;
}

float spindown_throttle(float throttle, float idle, float factor) {
  if (!(throttle > idle)) return throttle;
  if (factor <= 0.0F) return idle;
  if (factor >= 1.0F) return throttle;
  return idle + (throttle - idle) * factor;
}
