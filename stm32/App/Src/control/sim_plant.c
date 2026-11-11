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
    for (unsigned j = 0U; j < LQG_NX; ++j)
      sum += LQG_ERT_A[i][j] * s->x[j];
    for (unsigned j = 0U; j < LQG_NU; ++j)
      sum += LQG_ERT_B[i][j] * du[j];
    next[i] = sum;
  }
  /* No chão: não afunda e não acelera para baixo (z NED positivo é descer). */
  if (next[LQG_X_Z] >= (lqg_real_t)s->ground_z) {
    next[LQG_X_Z] = (lqg_real_t)s->ground_z;
    if (next[LQG_X_VZ] > 0.0)
      next[LQG_X_VZ] = 0.0;
    /* Apoiado nas pernas, também não inclina, não gira e não escorrega. Sem
       isso, o desequilíbrio fixo (CTL_FF_*), que o drone simulado recebe
       inteiro mesmo com os motores em marcha lenta, o tombava na partida. */
    static const unsigned kept[] = {LQG_X_X, LQG_X_Y, LQG_X_PSI};
    static const unsigned held[] = {LQG_X_PHI, LQG_X_THETA, LQG_X_VX, LQG_X_VY,
                                    LQG_X_P,   LQG_X_Q,     LQG_X_R};
    for (unsigned k = 0U; k < sizeof(kept) / sizeof(kept[0]); ++k) {
      next[kept[k]] = s->x[kept[k]];
    }
    for (unsigned k = 0U; k < sizeof(held) / sizeof(held[0]); ++k)
      next[held[k]] = 0.0;
  }
  memcpy(s->x, next, sizeof(next));
}

void sim_plant_measure(const sim_plant_t *s, lqg_real_t y[LQG_NY]) {
  for (unsigned i = 0U; i < LQG_NY; ++i) {
    lqg_real_t sum = 0.0;
    for (unsigned j = 0U; j < LQG_NX; ++j)
      sum += LQG_ERT_C[i][j] * s->x[j];
    y[i] = sum;
  }
}
