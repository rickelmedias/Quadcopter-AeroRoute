/**
 * @file test_attitude_mode.c
 * @brief Modo "só atitude e altura": os termos horizontais saem da lei de controle.
 */
#include "control/attitude_mode.h"
#include "control/lqg_ert.h"
#include "control/lqg_ert_model.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @brief Zera exatamente as colunas horizontais e deixa o resto intacto. */
static void test_mask_zeroes_only_horizontal_terms(void) {
  lqg_gains_t g = LQG_ERT_DEFAULT_GAINS;
  attitude_mode_mask_gains(&g);
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned j = 0U; j < LQG_NX; ++j) {
      const int horizontal = j == LQG_X_X || j == LQG_X_Y || j == LQG_X_VX || j == LQG_X_VY;
      if (horizontal) {
        assert(g.Kd[i][j] == 0.0);
      } else {
        assert(g.Kd[i][j] == LQG_ERT_DEFAULT_GAINS.Kd[i][j]);
      }
    }
    assert(g.Kfd[i][LQG_R_X] == 0.0 && g.Kfd[i][LQG_R_Y] == 0.0);
    assert(g.Kfd[i][LQG_R_Z] == LQG_ERT_DEFAULT_GAINS.Kfd[i][LQG_R_Z]);
    assert(g.Kfd[i][LQG_R_PSI] == LQG_ERT_DEFAULT_GAINS.Kfd[i][LQG_R_PSI]);
  }
  assert(memcmp(g.L, LQG_ERT_DEFAULT_GAINS.L, sizeof(g.L)) == 0);
  assert(memcmp(g.M, LQG_ERT_DEFAULT_GAINS.M, sizeof(g.M)) == 0);
}

/** @brief A saída não muda com x, y, vx, vy; muda com φ. */
static void test_output_ignores_horizontal_states(void) {
  lqg_gains_t g = LQG_ERT_DEFAULT_GAINS;
  attitude_mode_mask_gains(&g);
  lqg_ert_t a, b;
  lqg_ert_init(&a);
  lqg_ert_init(&b);
  lqg_real_t y[LQG_NY] = {0};
  lqg_real_t r[LQG_NR] = {0};
  lqg_real_t du_a[LQG_NU], du_b[LQG_NU];
  lqg_ert_output(&a, &g, y, r, du_a);
  lqg_ert_update(&a, &g, du_a);
  lqg_ert_output(&b, &g, y, r, du_b);
  lqg_ert_update(&b, &g, du_b);
  /* Mesmo estado, mas b "acredita" que está longe e rápido na horizontal. */
  b.x_pred[LQG_X_X] = 3.0;
  b.x_pred[LQG_X_VY] = -2.0;
  lqg_real_t ya[LQG_NY] = {0}, yb[LQG_NY] = {0};
  attitude_mode_repeat_prediction(&a, ya);
  attitude_mode_repeat_prediction(&b, yb);
  lqg_ert_output(&a, &g, ya, r, du_a);
  lqg_ert_output(&b, &g, yb, r, du_b);
  for (unsigned i = 0U; i < LQG_NU; ++i) assert(fabs(du_a[i] - du_b[i]) < 1e-9);

  /* Já a rolagem medida mexe na saída. */
  lqg_ert_t c;
  lqg_ert_init(&c);
  lqg_real_t yc[LQG_NY] = {0};
  yc[LQG_Y_PHI] = 0.1;
  lqg_real_t du_c[LQG_NU];
  lqg_ert_output(&c, &g, yc, r, du_c);
  double diff = 0.0;
  for (unsigned i = 0U; i < LQG_NU; ++i) diff += fabs(du_c[i]);
  assert(diff > 1e-3);
}

/** @brief As medidas horizontais repetem a predição e a inovação delas fica zero. */
static void test_repeat_prediction_gives_zero_innovation(void) {
  lqg_gains_t g = LQG_ERT_DEFAULT_GAINS;
  lqg_ert_t s;
  lqg_ert_init(&s);
  lqg_real_t y[LQG_NY] = {0};
  lqg_real_t r[LQG_NR] = {0};
  lqg_real_t du[LQG_NU];
  y[LQG_Y_PHI] = 0.05;
  lqg_ert_output(&s, &g, y, r, du);
  lqg_ert_update(&s, &g, du);
  for (int k = 0; k < 50; ++k) {
    y[LQG_Y_VX] = 99.0;  /* fluxo absurdo: tem de ser ignorado */
    y[LQG_Y_VY] = -99.0;
    attitude_mode_repeat_prediction(&s, y);
    lqg_ert_output(&s, &g, y, r, du);
    assert(fabs(s.innovation[LQG_Y_VX]) < 1e-12 && fabs(s.innovation[LQG_Y_VY]) < 1e-12);
    lqg_ert_update(&s, &g, du);
  }
}

/** @brief No primeiro passo (estimativa zerada) a medida vira 0. */
static void test_first_step_is_zero(void) {
  lqg_ert_t s;
  lqg_ert_init(&s);
  lqg_real_t y[LQG_NY];
  for (unsigned i = 0U; i < LQG_NY; ++i) y[i] = 7.0;
  attitude_mode_repeat_prediction(&s, y);
  assert(y[LQG_Y_VX] == 0.0 && y[LQG_Y_VY] == 0.0 && y[LQG_Y_Z] == 7.0);
}

/** Ruído uniforme em [-1, 1]. */
static double noise(void) { return (rand() / (double)RAND_MAX - 0.5) * 2.0; }

/**
 * @brief Malha fechada na planta exportada pelo Simulink: sobe 10 cm em rampa
 *        de 0,05 m/s, com ruído nos sensores e um fluxo óptico absurdo que
 *        precisa ser ignorado.
 *
 * Roll, pitch e guinada iniciais fora de zero precisam voltar a zero, e a
 * altura precisa seguir a rampa. A posição horizontal NÃO é controlada neste
 * modo: o drone desliza, e o teste só confere que isso não desestabiliza a
 * atitude.
 */
static void test_closed_loop_attitude_and_height(void) {
  srand(1);
  lqg_gains_t g = LQG_ERT_DEFAULT_GAINS;
  attitude_mode_mask_gains(&g);
  lqg_ert_t f;
  lqg_ert_init(&f);
  double x[LQG_NX] = {0};
  x[LQG_X_Z] = -0.065;
  x[LQG_X_PHI] = 0.05;
  x[LQG_X_THETA] = -0.04;
  x[LQG_X_PSI] = 0.1;
  const double z0 = -0.065;
  double zref = z0;
  static const unsigned idx[LQG_NY] = {LQG_X_Z, LQG_X_PHI, LQG_X_THETA, LQG_X_PSI, LQG_X_VX,
                                       LQG_X_VY, LQG_X_P, LQG_X_Q, LQG_X_R};
  double max_late_angle = 0.0;
  for (int k = 0; k < 1500; ++k) {
    if (k >= 100) zref = fmax(zref - 0.05 * LQG_TS_S, z0 - 0.10);
    lqg_real_t y[LQG_NY], du[LQG_NU];
    const lqg_real_t r[LQG_NR] = {0.0, 0.0, zref, 0.0};
    for (unsigned i = 0U; i < LQG_NY; ++i) y[i] = x[idx[i]];
    y[LQG_Y_Z] += 0.005 * noise();
    y[LQG_Y_PHI] += 0.003 * noise();
    y[LQG_Y_THETA] += 0.003 * noise();
    y[LQG_Y_P] += 0.02 * noise();
    y[LQG_Y_Q] += 0.02 * noise();
    y[LQG_Y_R] += 0.02 * noise();
    y[LQG_Y_VX] = 99.0;
    y[LQG_Y_VY] = -99.0;
    attitude_mode_repeat_prediction(&f, y);
    lqg_ert_output(&f, &g, y, r, du);
    lqg_ert_update(&f, &g, du);
    double next[LQG_NX];
    for (unsigned i = 0U; i < LQG_NX; ++i) {
      double sum = 0.0;
      for (unsigned j = 0U; j < LQG_NX; ++j) sum += LQG_ERT_A[i][j] * x[j];
      for (unsigned j = 0U; j < LQG_NU; ++j) sum += LQG_ERT_B[i][j] * du[j];
      next[i] = sum;
    }
    memcpy(x, next, sizeof(x));
    if (k > 300) {
      max_late_angle = fmax(max_late_angle, fabs(x[LQG_X_PHI]));
      max_late_angle = fmax(max_late_angle, fabs(x[LQG_X_THETA]));
      max_late_angle = fmax(max_late_angle, fabs(x[LQG_X_PSI]));
    }
  }
  assert(max_late_angle < 0.01);                 /* menos de 0,6 grau depois de 3 s */
  assert(fabs(x[LQG_X_Z] - (z0 - 0.10)) < 0.005); /* 10 cm, com erro < 5 mm */
}

int main(void) {
  test_mask_zeroes_only_horizontal_terms();
  test_output_ignores_horizontal_states();
  test_repeat_prediction_gives_zero_innovation();
  test_first_step_is_zero();
  test_closed_loop_attitude_and_height();
  printf("test_attitude_mode: ok\n");
  return 0;
}
