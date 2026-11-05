/**
 * @file test_lqg_dist.c
 * @brief LQG ajustado e estimador de perturbação: unidades e voo simulado.
 */
#include "control/attitude_mode.h"
#include "control/lqg_dist.h"
#include "control/lqg_ert.h"
#include "control/lqg_ert_model.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @brief A primeira medida vira a posição; congelado, d̂ não muda; a saída segue o padrão. */
static void test_units(void) {
  lqg_dist_t e;
  lqg_dist_reset(&e);
  lqg_real_t y[LQG_NY] = {0};
  y[LQG_Y_Z] = -0.08;
  y[LQG_Y_THETA] = 0.01;
  lqg_dist_correct(&e, y, false);
  assert(!e.started[LQG_DIST_Z] && e.started[LQG_DIST_THETA]);
  lqg_dist_correct(&e, y, true);
  assert(e.started[LQG_DIST_Z] && fabsf(e.s[LQG_DIST_Z][0] + 0.08F) < 1e-6F);

  lqg_dist_set(&e, LQG_DIST_Z, 10.0F);
  lqg_dist_set(&e, LQG_DIST_THETA, -5.0F);
  float out[LQG_NU];
  lqg_dist_output(&e, out);
  /* +10 nos quatro; −5 na frente (u1, u2) e +5 atrás. */
  assert(fabsf(out[0] - 5.0F) < 1e-6F && fabsf(out[1] - 5.0F) < 1e-6F);
  assert(fabsf(out[2] - 15.0F) < 1e-6F && fabsf(out[3] - 15.0F) < 1e-6F);

  const float u[LQG_NU] = {50.0F, 50.0F, 50.0F, 50.0F};
  y[LQG_Y_Z] = -0.5;  /* bem abaixo do previsto: d̂ de altura mudaria */
  lqg_dist_correct(&e, y, true);
  lqg_dist_set(&e, LQG_DIST_Z, 10.0F);
  lqg_dist_predict(&e, u, true, false, 250.0F);
  assert(lqg_dist_get(&e, LQG_DIST_Z) == 10.0F);
  lqg_dist_predict(&e, u, false, true, 250.0F);
  assert(lqg_dist_get(&e, LQG_DIST_Z) == 10.0F);
  lqg_dist_set(&e, LQG_DIST_Z, 1000.0F);
  lqg_dist_predict(&e, u, false, false, 250.0F);
  assert(lqg_dist_get(&e, LQG_DIST_Z) <= 250.0F);
}

/**
 * @brief Ensaio de 05/11: na rampa de partida, os motores ficam bem abaixo do
 *        hover e o chão segura o drone. Congelado, d̂ não pode aprender isso
 *        como "empuxo sobrando" (a correção também não mexe nele).
 */
static void test_ground_does_not_teach(void) {
  lqg_dist_t e;
  lqg_dist_reset(&e);
  lqg_real_t y[LQG_NY] = {0};
  y[LQG_Y_Z] = -0.08;
  const float below_hover[LQG_NU] = {-400.0F, -400.0F, -400.0F, -400.0F};
  for (int k = 0; k < 300; ++k) {
    lqg_dist_correct(&e, y, true);
    lqg_dist_predict(&e, below_hover, true, false, 250.0F);
  }
  assert(lqg_dist_get(&e, LQG_DIST_Z) == 0.0F);
  /* Sem congelar, é exatamente o erro do ensaio: d̂ vai muito para baixo. */
  for (int k = 0; k < 300; ++k) {
    lqg_dist_correct(&e, y, true);
    lqg_dist_predict(&e, below_hover, false, false, 250.0F);
  }
  assert(lqg_dist_get(&e, LQG_DIST_Z) < -100.0F);
}

/** @brief Ajustado: mesmo projeto do TCC, atitude e altura mais firmes. */
static void test_adjusted_gains(void) {
  lqg_gains_t g = LQG_ERT_DEFAULT_GAINS;
  lqg_dist_apply_gains(&g);
  assert(fabs(g.Kd[0][LQG_X_THETA]) > 2.5 * fabs(LQG_ERT_DEFAULT_GAINS.Kd[0][LQG_X_THETA]));
  assert(fabs(g.Kd[0][LQG_X_Z]) > 2.5 * fabs(LQG_ERT_DEFAULT_GAINS.Kd[0][LQG_X_Z]));
  /* O Kalman continua o do Simulink. */
  assert(memcmp(g.L, LQG_ERT_DEFAULT_GAINS.L, sizeof(g.L)) == 0);
  assert(memcmp(g.M, LQG_ERT_DEFAULT_GAINS.M, sizeof(g.M)) == 0);
  /* z e ψ: a planta é um integrador, então Kfd é a própria coluna de Kd. */
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    assert(fabs(g.Kfd[i][LQG_R_Z] - g.Kd[i][LQG_X_Z]) < 1e-6 * fabs(g.Kd[i][LQG_X_Z]));
  }
}

/** Ruído uniforme em [-1, 1]. */
static double noise(void) { return (rand() / (double)RAND_MAX - 0.5) * 2.0; }

/** @brief Resultado de um voo simulado. */
typedef struct {
  double height_error;  /**< Erro médio de altura em 50–60 s, em m. */
  double theta, phi;    /**< Ângulos no fim, em graus. */
  double rebound;       /**< Maior |θ| depois do empurrão, em graus. */
  double max_height;    /**< Maior altura, em m. */
} result_t;

/**
 * @brief Voo simulado como no firmware: Kalman do TCC, LQR (TCC ou ajustado),
 *        estimador opcional fora do Kalman, motor com atraso @p tau, hover
 *        errado em @p dh rad/s, desequilíbrio @p dth/@p dph, chão, subida a
 *        20 cm em rampa e um empurrão de 1,5 s em 60 s.
 */
static result_t simulate(bool adjusted, bool estimator, double dh, double dth, double dph,
                         double tau) {
  srand(9);
  lqg_gains_t eff = LQG_ERT_DEFAULT_GAINS;
  if (adjusted) lqg_dist_apply_gains(&eff);
  attitude_mode_mask_gains(&eff);
  lqg_ert_t f;
  lqg_ert_init(&f);
  lqg_dist_t e;
  lqg_dist_reset(&e);
  bool released = false;
  static const unsigned idx[LQG_NY] = {LQG_X_Z, LQG_X_PHI, LQG_X_THETA, LQG_X_PSI, LQG_X_VX,
                                       LQG_X_VY, LQG_X_P, LQG_X_Q, LQG_X_R};
  static const double theta_sign[4] = {1, 1, -1, -1}, phi_sign[4] = {1, -1, -1, 1};
  const double ground = -0.08;
  double x[LQG_NX] = {0}, u_motor[4] = {0};
  x[LQG_X_Z] = ground;
  double zref = ground;
  result_t res = {0};
  int samples = 0;
  for (int k = 0; k < 9000; ++k) {
    if (k >= 100) zref = fmax(zref - 0.05 * LQG_TS_S, ground - 0.20);
    lqg_real_t y[LQG_NY], du[LQG_NU];
    const lqg_real_t r[LQG_NR] = {0.0, 0.0, zref, 0.0};
    for (unsigned i = 0U; i < LQG_NY; ++i) y[i] = x[idx[i]];
    y[LQG_Y_Z] += 0.004 * noise();
    y[LQG_Y_PHI] += 0.002 * noise();
    y[LQG_Y_THETA] += 0.002 * noise();
    y[LQG_Y_P] += 0.02 * noise();
    y[LQG_Y_Q] += 0.02 * noise();
    y[LQG_Y_R] += 0.02 * noise();
    attitude_mode_repeat_prediction(&f, y);
    lqg_ert_output(&f, &eff, y, r, du);
    float comp[LQG_NU] = {0.0F, 0.0F, 0.0F, 0.0F};
    if (estimator) {
      lqg_dist_correct(&e, y, true);
      lqg_dist_output(&e, comp);
    }
    lqg_ert_update(&f, &eff, du);
    if (estimator) {
      /* Como no firmware (update_dist): congelado até se soltar do chão; com
         o alvo acima, o hover sobe 10 rad/s por segundo até lá. */
      float u[LQG_NU];
      for (unsigned j = 0U; j < LQG_NU; ++j) u[j] = (float)du[j] + comp[j];
      if (k >= 100 && ground - y[LQG_Y_Z] >= 0.01) released = true;
      lqg_dist_predict(&e, u, k < 100 || !released, false, 250.0F);
      if (k >= 100 && !released && y[LQG_Y_Z] - zref > 0.02) {
        lqg_dist_set(&e, LQG_DIST_Z, lqg_dist_get(&e, LQG_DIST_Z) + 10.0F * (float)LQG_TS_S);
      }
    }
    const double push = (k >= 6000 && k < 6150) ? 30.0 : 0.0;
    const double alpha = tau > 0.0 ? 0.01 / (tau + 0.01) : 1.0;
    double next[LQG_NX];
    for (unsigned j = 0U; j < LQG_NU; ++j) u_motor[j] += alpha * ((du[j] + comp[j]) - u_motor[j]);
    for (unsigned i = 0U; i < LQG_NX; ++i) {
      double sum = 0.0;
      for (unsigned j = 0U; j < LQG_NX; ++j) sum += LQG_ERT_A[i][j] * x[j];
      for (unsigned j = 0U; j < LQG_NU; ++j) {
        sum += LQG_ERT_B[i][j] *
               (u_motor[j] - dh - (dth + push) * theta_sign[j] - dph * phi_sign[j]);
      }
      next[i] = sum;
    }
    memcpy(x, next, sizeof(x));
    if (x[LQG_X_Z] > ground) {
      x[LQG_X_Z] = ground;
      if (x[LQG_X_VZ] > 0.0) x[LQG_X_VZ] = 0.0;
    }
    res.max_height = fmax(res.max_height, ground - x[LQG_X_Z]);
    if (k >= 6150) res.rebound = fmax(res.rebound, fabs(x[LQG_X_THETA]) * 57.29578);
    if (k >= 5000 && k < 6000) {
      res.height_error += (ground - x[LQG_X_Z]) - 0.20;
      ++samples;
    }
  }
  res.height_error /= samples;
  res.theta = x[LQG_X_THETA] * 57.29578;
  res.phi = x[LQG_X_PHI] * 57.29578;
  printf("  %-10s hover %+4.0f deseq %+4.0f/%+4.0f atraso %3.0f ms: erro de altura %+5.1f cm, "
         "theta %+5.1f, phi %+5.1f graus, rebote %4.1f, max %4.1f cm\n",
         estimator ? "melhorado" : (adjusted ? "ajustado" : "original"), dh, dth, dph,
         tau * 1000.0, res.height_error * 100.0, res.theta, res.phi, res.rebound,
         res.max_height * 100.0);
  return res;
}

int main(void) {
  test_units();
  test_ground_does_not_teach();
  test_adjusted_gains();

  /* Original do TCC com um desequilíbrio residual de 15 rad/s: inclina ~17°. */
  result_t r = simulate(false, false, 30.0, 15.0, 10.0, 0.04);
  assert(fabs(r.theta) > 10.0);
  /* Ajustado: atitude muito melhor, mas sem estimar o hover a altura fica longe. */
  r = simulate(true, false, 30.0, 15.0, 10.0, 0.04);
  assert(fabs(r.theta) < 8.0 && fabs(r.height_error) > 0.10);
  /* Melhorado: zera altura, φ e θ, com hover errado para os dois lados, sem a
     compensação fixa (105 rad/s) e com o motor atrasando até 120 ms. */
  const double cases[][4] = {{30, 15, 10, 0.04}, {-30, -15, -10, 0.08}, {30, 105, 10, 0.04},
                             {30, 15, 10, 0.12}, {-30, 15, 10, 0.12}};
  for (unsigned c = 0U; c < sizeof(cases) / sizeof(cases[0]); ++c) {
    r = simulate(true, true, cases[c][0], cases[c][1], cases[c][2], cases[c][3]);
    assert(fabs(r.height_error) < 0.005);
    assert(fabs(r.theta) < 0.3 && fabs(r.phi) < 0.3);
    assert(r.rebound < 20.0);
    /* Com o CTL_W0 30 rad/s acima do hover o drone passa do alvo de 20 cm
       (~36 cm) antes de o estimador achar o excesso: CTL_W0 perto do hover
       real importa. */
    assert(r.max_height < 0.40);
  }
  printf("test_lqg_dist: ok\n");
  return 0;
}
