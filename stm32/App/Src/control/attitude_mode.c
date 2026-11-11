/**
 * @file attitude_mode.c
 * @brief Modo só atitude e altura (contrato em control/attitude_mode.h).
 */
#include "control/attitude_mode.h"

#include "control/lqg_ert_model.h"

#include <math.h>

/** Estados horizontais que saem da lei de controle. */
static const unsigned HORIZONTAL_STATES[] = {LQG_X_X, LQG_X_Y, LQG_X_VX, LQG_X_VY};
/** Referências horizontais que saem do pré-compensador. */
static const unsigned HORIZONTAL_REFS[] = {LQG_R_X, LQG_R_Y};
/** Medidas horizontais que repetem a predição. */
static const unsigned HORIZONTAL_MEASUREMENTS[] = {LQG_Y_VX, LQG_Y_VY};

void attitude_mode_mask_gains(lqg_gains_t *gains) {
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned k = 0U; k < sizeof(HORIZONTAL_STATES) / sizeof(HORIZONTAL_STATES[0]); ++k) {
      gains->Kd[i][HORIZONTAL_STATES[k]] = 0.0;
    }
    for (unsigned k = 0U; k < sizeof(HORIZONTAL_REFS) / sizeof(HORIZONTAL_REFS[0]); ++k) {
      gains->Kfd[i][HORIZONTAL_REFS[k]] = 0.0;
    }
  }
}

void attitude_mode_repeat_prediction(const lqg_ert_t *s, lqg_real_t y[LQG_NY]) {
  for (unsigned k = 0U; k < sizeof(HORIZONTAL_MEASUREMENTS) / sizeof(HORIZONTAL_MEASUREMENTS[0]);
       ++k) {
    const unsigned row = HORIZONTAL_MEASUREMENTS[k];
    lqg_real_t predicted = 0.0;
    if (!s->ic_load) {
      for (unsigned j = 0U; j < LQG_NX; ++j)
        predicted += LQG_ERT_C[row][j] * s->x_pred[j];
    }
    y[row] = predicted;
  }
}

void attitude_mode_rotate_pair(lqg_real_t *a, lqg_real_t *b, float angle) {
  const lqg_real_t c = (lqg_real_t)cosf(angle), sn = (lqg_real_t)sinf(angle);
  const lqg_real_t na = *a * c - *b * sn;
  *b = *a * sn + *b * c;
  *a = na;
}

void attitude_mode_rotate_horizontal(lqg_real_t x[LQG_NX], float angle) {
  attitude_mode_rotate_pair(&x[LQG_X_X], &x[LQG_X_Y], angle);
  attitude_mode_rotate_pair(&x[LQG_X_VX], &x[LQG_X_VY], angle);
}
