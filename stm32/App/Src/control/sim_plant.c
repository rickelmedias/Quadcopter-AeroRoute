/**
 * @file sim_plant.c
 * @brief Drone simulado (contrato em control/sim_plant.h).
 */
#include "control/sim_plant.h"

#include "control/lqg_ert_model.h"

#include <string.h>

void sim_plant_reset(sim_plant_t *s, float ground_z, float psi) {
  memset(s, 0, sizeof(*s));
  s->ground_z = ground_z;
  s->x[LQG_X_Z] = (lqg_real_t)ground_z;
  s->x[LQG_X_PSI] = (lqg_real_t)psi;
}

void sim_plant_step(sim_plant_t *s, const lqg_real_t du[LQG_NU]) {
  lqg_real_t next[LQG_NX];
  for (unsigned i = 0U; i < LQG_NX; ++i) {
    lqg_real_t sum = 0.0;
    for (unsigned j = 0U; j < LQG_NX; ++j) sum += LQG_ERT_A[i][j] * s->x[j];
    for (unsigned j = 0U; j < LQG_NU; ++j) sum += LQG_ERT_B[i][j] * du[j];
    next[i] = sum;
  }
  memcpy(s->x, next, sizeof(next));
  /* No chão: não afunda e não acelera para baixo (z NED positivo é descer). */
  if (s->x[LQG_X_Z] > (lqg_real_t)s->ground_z) {
    s->x[LQG_X_Z] = (lqg_real_t)s->ground_z;
    if (s->x[LQG_X_VZ] > 0.0) s->x[LQG_X_VZ] = 0.0;
  }
}

void sim_plant_measure(const sim_plant_t *s, lqg_real_t y[LQG_NY]) {
  for (unsigned i = 0U; i < LQG_NY; ++i) {
    lqg_real_t sum = 0.0;
    for (unsigned j = 0U; j < LQG_NX; ++j) sum += LQG_ERT_C[i][j] * s->x[j];
    y[i] = sum;
  }
}
