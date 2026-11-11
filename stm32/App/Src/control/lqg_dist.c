/**
 * @file lqg_dist.c
 * @brief LQG ajustado e estimador de perturbação (contrato em control/lqg_dist.h).
 */
#include "control/lqg_dist.h"

#include <math.h>
#include <string.h>

void lqg_dist_apply_gains(lqg_gains_t *gains) {
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned j = 0U; j < LQG_NX; ++j) gains->Kd[i][j] = LQG_AJ_KD[i][j];
    for (unsigned j = 0U; j < LQG_NR; ++j) gains->Kfd[i][j] = LQG_AJ_KFD[i][j];
  }
}

void lqg_dist_apply_kalman(lqg_gains_t *gains, bool fused) {
  const lqg_real_t(*M)[LQG_NY] = fused ? LQG_AJ_M_FUSED : LQG_AJ_M;
  const lqg_real_t(*L)[LQG_NY] = fused ? LQG_AJ_L_FUSED : LQG_AJ_L;
  for (unsigned i = 0U; i < LQG_NX; ++i) {
    for (unsigned j = 0U; j < LQG_NY; ++j) {
      gains->M[i][j] = M[i][j];
      gains->L[i][j] = L[i][j];
    }
  }
}

void lqg_dist_kalman_input(const lqg_real_t du[LQG_NU], lqg_real_t out[LQG_NU]) {
  const float *pattern = LQG_DIST_CHANNELS[LQG_DIST_PSI].pattern;
  lqg_real_t yaw = 0.0;
  for (unsigned j = 0U; j < LQG_NU; ++j) yaw += (lqg_real_t)pattern[j] * du[j];
  yaw /= (lqg_real_t)LQG_NU;
  lqg_real_t collective = 0.0;
  for (unsigned j = 0U; j < LQG_NU; ++j) collective += du[j];
  collective /= (lqg_real_t)LQG_NU;
  const lqg_real_t remove_yaw = yaw * (1.0 - (lqg_real_t)LQG_AJ_YAW_SCALE);
  const lqg_real_t remove_z = collective * (1.0 - (lqg_real_t)LQG_AJ_Z_SCALE);
  for (unsigned j = 0U; j < LQG_NU; ++j) {
    out[j] = du[j] - remove_yaw * (lqg_real_t)pattern[j] - remove_z;
  }
}

bool lqg_dist_limit_yaw(const lqg_real_t du[LQG_NU], lqg_real_t out[LQG_NU], float limit) {
  for (unsigned j = 0U; j < LQG_NU; ++j) out[j] = du[j];
  if (!(limit > 0.0F)) return false;
  const float *pattern = LQG_DIST_CHANNELS[LQG_DIST_PSI].pattern;
  lqg_real_t yaw = 0.0;
  for (unsigned j = 0U; j < LQG_NU; ++j) yaw += (lqg_real_t)pattern[j] * du[j];
  yaw /= (lqg_real_t)LQG_NU;
  const lqg_real_t lim = (lqg_real_t)limit;
  if (yaw <= lim && yaw >= -lim) return false;
  const lqg_real_t excess = yaw > lim ? yaw - lim : yaw + lim;
  for (unsigned j = 0U; j < LQG_NU; ++j) out[j] = du[j] - excess * (lqg_real_t)pattern[j];
  return true;
}

void lqg_dist_reset(lqg_dist_t *e) { memset(e, 0, sizeof(*e)); }

void lqg_dist_correct(lqg_dist_t *e, const lqg_real_t y[LQG_NY], bool range_ok) {
  for (unsigned c = 0U; c < LQG_DIST_NC; ++c) {
    const lqg_dist_channel_t *ch = &LQG_DIST_CHANNELS[c];
    e->d_before[c] = e->s[c][2];
    if (c == LQG_DIST_Z && !range_ok) continue;
    float meas[2];
    bool finite = true;
    for (unsigned k = 0U; k < ch->ny; ++k) {
      meas[k] = (float)y[ch->y[k]];
      finite = finite && isfinite(meas[k]);
    }
    if (!finite) continue;
    if (!e->started[c]) {
      /* Primeira medida: posição medida, parado, sem perturbação. */
      e->s[c][0] = meas[0];
      e->s[c][1] = ch->ny > 1U ? meas[1] : 0.0F;
      e->s[c][2] = 0.0F;
      e->started[c] = true;
      continue;
    }
    float innovation[2];
    for (unsigned k = 0U; k < ch->ny; ++k) innovation[k] = meas[k] - e->s[c][k];
    for (unsigned i = 0U; i < 3U; ++i) {
      for (unsigned k = 0U; k < ch->ny; ++k) e->s[c][i] += ch->M[i][k] * innovation[k];
    }
  }
}

void lqg_dist_predict(lqg_dist_t *e, const float u[LQG_NU], bool freeze, bool freeze_z,
                      float limit) {
  for (unsigned c = 0U; c < LQG_DIST_NC; ++c) {
    if (!e->started[c]) continue;
    const lqg_dist_channel_t *ch = &LQG_DIST_CHANNELS[c];
    float U = 0.0F;
    for (unsigned j = 0U; j < LQG_NU; ++j) U += ch->pattern[j] * u[j];
    if (!isfinite(U)) continue;
    float next[3];
    for (unsigned i = 0U; i < 3U; ++i) {
      next[i] = ch->B[i] * U;
      for (unsigned j = 0U; j < 3U; ++j) next[i] += ch->A[i][j] * e->s[c][j];
    }
    memcpy(e->s[c], next, sizeof(next));
    /* Congelado, d̂ volta ao valor de antes da correção deste passo: posição
       e velocidade continuam acompanhando. */
    if (freeze || (freeze_z && c == LQG_DIST_Z)) e->s[c][2] = e->d_before[c];
    e->s[c][2] = fmaxf(fminf(e->s[c][2], limit), -limit);
  }
}

void lqg_dist_output(const lqg_dist_t *e, float out[LQG_NU]) {
  for (unsigned j = 0U; j < LQG_NU; ++j) {
    out[j] = 0.0F;
    for (unsigned c = 0U; c < LQG_DIST_NC; ++c) {
      out[j] += LQG_DIST_CHANNELS[c].pattern[j] * e->s[c][2];
    }
  }
}

float lqg_dist_get(const lqg_dist_t *e, lqg_dist_index_t c) {
  return (unsigned)c < LQG_DIST_NC ? e->s[c][2] : 0.0F;
}

void lqg_dist_set(lqg_dist_t *e, lqg_dist_index_t c, float value) {
  if ((unsigned)c < LQG_DIST_NC && isfinite(value)) {
    e->s[c][2] = value;
    e->d_before[c] = value;
  }
}
