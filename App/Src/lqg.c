/**
 * @file lqg.c
 * @brief LQG genérico discreto (contrato em lqg.h).
 */
#include "lqg.h"
#include <math.h>
#include <string.h>

void lqg_init(lqg_t *f, const float x0[LQG_NX]) {
  memset(f, 0, sizeof(*f));
  for (unsigned i = 0; i < LQG_NX; ++i) {
    f->x[i] = x0 ? x0[i] : 0.0F;
    f->P[i][i] = LQG_P0[i];
  }
  for (unsigned j = 0; j < LQG_NU; ++j) f->u[j] = LQG_U0[j];
}

void lqg_predict(lqg_t *f) {
  float du[LQG_NU];
  for (unsigned j = 0; j < LQG_NU; ++j) du[j] = f->u[j] - LQG_U0[j];

  float x[LQG_NX];
  for (unsigned i = 0; i < LQG_NX; ++i) {
    float s = 0.0F;
    for (unsigned k = 0; k < LQG_NX; ++k) s += LQG_AD[i][k] * f->x[k];
    for (unsigned j = 0; j < LQG_NU; ++j) s += LQG_BD[i][j] * du[j];
    x[i] = s;
  }
  memcpy(f->x, x, sizeof(x));

  /* P = Ad P Ad' + Qw, em dois produtos: T = Ad P e depois T Ad'. */
  static float T[LQG_NX][LQG_NX];
  for (unsigned i = 0; i < LQG_NX; ++i) {
    for (unsigned k = 0; k < LQG_NX; ++k) {
      float s = 0.0F;
      for (unsigned m = 0; m < LQG_NX; ++m) s += LQG_AD[i][m] * f->P[m][k];
      T[i][k] = s;
    }
  }
  for (unsigned i = 0; i < LQG_NX; ++i) {
    for (unsigned k = i; k < LQG_NX; ++k) {
      float s = 0.0F;
      for (unsigned m = 0; m < LQG_NX; ++m) s += T[i][m] * LQG_AD[k][m];
      if (i == k) s += LQG_QW[i];
      f->P[i][k] = f->P[k][i] = s;  /* mantém a simetria */
    }
  }
  /* Limita a variância dos estados não observáveis (ver lqg.h). */
  for (unsigned i = 0; i < LQG_NX; ++i) {
    if (f->P[i][i] > LQG_P_MAX[i]) {
      const float scale = sqrtf(LQG_P_MAX[i] / f->P[i][i]);
      for (unsigned k = 0; k < LQG_NX; ++k) { f->P[i][k] *= scale; f->P[k][i] *= scale; }
    }
  }
}

bool lqg_update(lqg_t *f, unsigned i, float y, bool wrap) {
  if (i >= LQG_NY || !isfinite(y)) return false;
  const float *c = LQG_C[i];

  float pc[LQG_NX];  /* P C_i' */
  float yhat = 0.0F;
  for (unsigned r = 0; r < LQG_NX; ++r) {
    float s = 0.0F;
    for (unsigned k = 0; k < LQG_NX; ++k) s += f->P[r][k] * c[k];
    pc[r] = s;
    yhat += c[r] * f->x[r];
  }
  float s = LQG_RV[i];
  for (unsigned r = 0; r < LQG_NX; ++r) s += c[r] * pc[r];
  if (!(s > 0.0F)) return false;

  float innovation = y - yhat;
  if (wrap) innovation = remainderf(innovation, 6.2831853F);
  const float gate = LQG_GATE_SIGMA[i];
  if (gate > 0.0F && innovation * innovation > gate * gate * s) { ++f->rejected[i]; return false; }

  const float inv = 1.0F / s;
  for (unsigned r = 0; r < LQG_NX; ++r) f->x[r] += pc[r] * inv * innovation;
  /* P = P - (P C') (P C')' / s */
  for (unsigned r = 0; r < LQG_NX; ++r) {
    for (unsigned k = r; k < LQG_NX; ++k) {
      const float v = f->P[r][k] - pc[r] * pc[k] * inv;
      f->P[r][k] = f->P[k][r] = v;
    }
  }
  return true;
}

void lqg_control(lqg_t *f, const float x_ref[LQG_NX], float u[LQG_NU]) {
  float e[LQG_NX];
  for (unsigned i = 0; i < LQG_NX; ++i) e[i] = f->x[i] - x_ref[i];
  for (unsigned i = 0; i < LQG_NX; ++i) {
    if (LQG_ANGLE_STATE[i]) e[i] = remainderf(e[i], 6.2831853F);
  }
  for (unsigned j = 0; j < LQG_NU; ++j) {
    float v = LQG_U0[j];
    for (unsigned i = 0; i < LQG_NX; ++i) v -= LQG_K[j][i] * e[i];
    if (!isfinite(v)) v = LQG_U_MIN[j];
    if (v < LQG_U_MIN[j]) v = LQG_U_MIN[j];
    if (v > LQG_U_MAX[j]) v = LQG_U_MAX[j];
    u[j] = v;
    f->u[j] = v;
  }
}
