/**
 * @file lqi.c
 * @brief Integradores do LQI (contrato em control/lqi.h).
 */
#include "control/lqi.h"

#include <math.h>
#include <string.h>

/** @brief Maior |ξ| permitido no integrador @p k para a correção caber em @p limit. */
static float xi_limit(const lqi_gains_t *g, unsigned k, float limit_rad_s) {
  float largest = 0.0F;
  for (unsigned i = 0U; i < LQG_NU; ++i) largest = fmaxf(largest, fabsf((float)g->Ki[i][k]));
  return largest > 0.0F ? limit_rad_s / largest : 0.0F;
}

void lqi_reset(lqi_state_t *s) { memset(s, 0, sizeof(*s)); }

void lqi_apply_gains(const lqi_gains_t *g, lqg_gains_t *gains) {
  memcpy(gains->Kd, g->Kx, sizeof(gains->Kd));
  memcpy(gains->Kfd, g->Kfd, sizeof(gains->Kfd));
}

void lqi_update(lqi_state_t *s, const lqi_gains_t *g, bool enabled, float dt,
                const float error[LQI_NI], float limit_rad_s) {
  if (!enabled) return;
  for (unsigned k = 0U; k < LQI_NI; ++k) {
    if (!isfinite(error[k])) continue;
    const float limit = xi_limit(g, k, limit_rad_s);
    float v = s->xi[k] + error[k] * dt;
    if (v > limit) v = limit;
    if (v < -limit) v = -limit;
    s->xi[k] = v;
  }
}

void lqi_output(const lqi_state_t *s, const lqi_gains_t *g, float out[LQG_NU]) {
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    float u = 0.0F;
    for (unsigned k = 0U; k < LQI_NI; ++k) u -= (float)g->Ki[i][k] * s->xi[k];
    out[i] = u;
  }
}

void lqi_reduce_hover(lqi_state_t *s, const lqi_gains_t *g, float delta_rad_s,
                      float limit_rad_s) {
  /* Correção média por unidade de ξ_z: -Ki_z (positivo, sobe os quatro). */
  float per_xi = 0.0F;
  for (unsigned i = 0U; i < LQG_NU; ++i) per_xi -= 0.25F * (float)g->Ki[i][LQI_I_Z];
  if (!(per_xi > 0.0F) || !(delta_rad_s > 0.0F)) return;
  const float floor_xi = -xi_limit(g, LQI_I_Z, limit_rad_s);
  s->xi[LQI_I_Z] = fmaxf(s->xi[LQI_I_Z] - delta_rad_s / per_xi, floor_xi);
}

bool lqi_at_limit(const lqi_state_t *s, const lqi_gains_t *g, float limit_rad_s) {
  for (unsigned k = 0U; k < LQI_NI; ++k) {
    const float limit = xi_limit(g, k, limit_rad_s);
    if (limit > 0.0F && fabsf(s->xi[k]) >= 0.999F * limit) return true;
  }
  return false;
}
