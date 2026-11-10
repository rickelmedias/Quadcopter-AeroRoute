/**
 * @file lqg_ert.c
 * @brief Kalman + LQR do TCC (contrato em control/lqg_ert.h).
 *
 * Cada soma abaixo acumula os termos na mesma ordem do `STM32_step()` gerado
 * pelo Simulink. Isso não é estética: em ponto flutuante a ordem das somas
 * muda os últimos bits, e é ela que permite ao teste exigir igualdade exata
 * com o código de referência.
 */
#include "control/lqg_ert.h"

#include "control/lqg_ert_model.h"

#include <string.h>

void lqg_ert_init(lqg_ert_t *s) {
  memset(s, 0, sizeof(*s));
  s->ic_load = true;
}

/**
 * @brief Inovação e = y - C x̂[k|k-1] (blocos Add1 e Sum do Simulink).
 *
 * @param[in]  x_pred  x̂[k|k-1].
 * @param[in]  y       Medidas.
 * @param[out] e       Inovação.
 */
static void innovation(const lqg_real_t x_pred[LQG_NX], const lqg_real_t y[LQG_NY],
                       lqg_real_t e[LQG_NY]) {
  for (unsigned i = 0U; i < LQG_NY; ++i) {
    lqg_real_t cx = 0.0;
    for (unsigned j = 0U; j < LQG_NX; ++j) {
      cx += LQG_ERT_C[i][j] * x_pred[j];
    }
    e[i] = y[i] - cx;
  }
}

void lqg_ert_output(lqg_ert_t *s, const lqg_gains_t *g,
                    const lqg_real_t y[LQG_NY], const lqg_real_t r[LQG_NR],
                    lqg_real_t du[LQG_NU]) {
  /* Delay MemoryX: no primeiro passo a estimativa parte de zero. */
  if (s->ic_load) {
    memset(s->x_pred, 0, sizeof(s->x_pred));
  }

  innovation(s->x_pred, y, s->innovation);

  /* Estimador corrente: x̂[k|k] = x̂[k|k-1] + M e. */
  for (unsigned i = 0U; i < LQG_NX; ++i) {
    lqg_real_t me = 0.0;
    for (unsigned j = 0U; j < LQG_NY; ++j) {
      me += g->M[i][j] * s->innovation[j];
    }
    s->x_filt[i] = me + s->x_pred[i];
  }

  /* du = Kfd r - Kd x̂[k|k] (blocos Gain7, Gain5 e Sum1). */
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    lqg_real_t kx = 0.0;
    for (unsigned j = 0U; j < LQG_NX; ++j) {
      kx += g->Kd[i][j] * s->x_filt[j];
    }
    lqg_real_t kr = g->Kfd[i][0] * r[0];
    for (unsigned j = 1U; j < LQG_NR; ++j) {
      kr += g->Kfd[i][j] * r[j];
    }
    du[i] = kr - kx;
  }
}

void lqg_ert_update(lqg_ert_t *s, const lqg_gains_t *g,
                    const lqg_real_t du_applied[LQG_NU]) {
  s->ic_load = false;

  lqg_real_t next[LQG_NX];
  for (unsigned i = 0U; i < LQG_NX; ++i) {
    lqg_real_t le = 0.0;
    for (unsigned j = 0U; j < LQG_NY; ++j) {
      le += g->L[i][j] * s->innovation[j];
    }
    lqg_real_t bu = LQG_ERT_B[i][0] * du_applied[0];
    for (unsigned j = 1U; j < LQG_NU; ++j) {
      bu += LQG_ERT_B[i][j] * du_applied[j];
    }
    lqg_real_t ax = 0.0;
    for (unsigned j = 0U; j < LQG_NX; ++j) {
      ax += LQG_ERT_A[i][j] * s->x_pred[j];
    }
    /* Mesma associação do bloco Sum '<S23>/Add': (B u + A x) + L e. */
    next[i] = (bu + ax) + le;
  }
  memcpy(s->x_pred, next, sizeof(next));
}

void lqg_ert_step(lqg_ert_t *s, const lqg_gains_t *g,
                  const lqg_real_t y[LQG_NY], const lqg_real_t r[LQG_NR],
                  lqg_real_t du[LQG_NU]) {
  lqg_ert_output(s, g, y, r, du);
  lqg_ert_update(s, g, du);
}
