/**
 * @file lqi_design.c
 * @brief Projeto do LQI (contrato em lqi_design.h).
 */
#include "lqi_design.h"

#include "control/lqg_ert_model.h"
#include "lqr_design.h"

#include <string.h>

/** Estados do subsistema de atitude e altura, na ordem do modelo. */
static const unsigned SUBSYSTEM[] = {LQG_X_Z, LQG_X_PHI, LQG_X_THETA, LQG_X_PSI,
                                     LQG_X_VZ, LQG_X_P, LQG_X_Q, LQG_X_R};
/** Quantidade de estados do subsistema. */
#define NS (sizeof(SUBSYSTEM) / sizeof(SUBSYSTEM[0]))
/** Estados do sistema aumentado. */
#define NA (NS + LQI_NI)
/** Posição de z, φ e θ dentro do subsistema (o que cada integrador integra). */
static const unsigned INTEGRATED[LQI_NI] = {0U, 1U, 2U};

void lqi_default_weights(lqi_weights_t *w) {
  static const double q_tcc[LQG_NX] = {5, 5, 500, 500, 500, 50, 5, 5, 5, 10, 10, 5};
  memcpy(w->q, q_tcc, sizeof(w->q));
  for (unsigned i = 0U; i < LQG_NU; ++i) w->r[i] = 0.1;
  w->q_int[LQI_I_Z] = 1000.0;
  w->q_int[LQI_I_PHI] = 10000.0;
  w->q_int[LQI_I_THETA] = 10000.0;
}

int lqi_design(const lqi_weights_t *w, lqi_gains_t *out) {
  double Aa[NA * NA], Ba[NA * LQG_NU], q[NA], K[LQG_NU * NA];
  memset(Aa, 0, sizeof(Aa));
  memset(Ba, 0, sizeof(Ba));
  for (unsigned i = 0U; i < NS; ++i) {
    for (unsigned j = 0U; j < NS; ++j) Aa[i * NA + j] = LQG_ERT_A[SUBSYSTEM[i]][SUBSYSTEM[j]];
    for (unsigned j = 0U; j < LQG_NU; ++j) Ba[i * LQG_NU + j] = LQG_ERT_B[SUBSYSTEM[i]][j];
    q[i] = w->q[SUBSYSTEM[i]];
  }
  for (unsigned k = 0U; k < LQI_NI; ++k) {
    Aa[(NS + k) * NA + NS + k] = 1.0;
    Aa[(NS + k) * NA + INTEGRATED[k]] = LQG_TS_S;
    q[NS + k] = w->q_int[k];
  }
  const int iterations = lqr_dlqr((int)NA, (int)LQG_NU, Aa, Ba, q, w->r, K);
  if (iterations < 0) return -1;

  memset(out, 0, sizeof(*out));
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned j = 0U; j < NS; ++j) out->Kx[i][SUBSYSTEM[j]] = K[i * NA + j];
    for (unsigned k = 0U; k < LQI_NI; ++k) out->Ki[i][k] = K[i * NA + NS + k];
    /* A referência entra como desvio de estado: du = -Kx (x̂ - x_ref). */
    out->Kfd[i][LQG_R_Z] = out->Kx[i][LQG_X_Z];
    out->Kfd[i][LQG_R_PSI] = out->Kx[i][LQG_X_PSI];
  }
  return iterations;
}
