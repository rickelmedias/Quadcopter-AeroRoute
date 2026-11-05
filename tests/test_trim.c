/**
 * @file test_trim.c
 * @brief Ajuste lento do hover e do equilíbrio (control/trim.h).
 *
 * Confere os sinais contra a matriz B do Simulink e simula o drone saindo do
 * chão com hover errado e peso mal distribuído, com o LQR em modo atitude +
 * altura: o ajuste precisa achar a perturbação e levar φ, θ a zero e a altura
 * ao alvo.
 */
#include "control/attitude_mode.h"
#include "control/lqg_ert.h"
#include "control/lqg_ert_model.h"
#include "control/trim.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Configuração usada no firmware (flight_config.h). */
static const trim_config_t CFG = {.ki_z = 20.0F, .ki_att = 80.0F, .ground_rate = 10.0F,
                                  .settled_vz = 0.05F, .max_z = 250.0F, .max_att = 250.0F,
                                  .att_error_max = 0.08F};

/** @brief Os sinais do trim batem com as colunas da matriz B (efeito sobre θ e φ). */
static void test_signs_match_b_matrix(void) {
  trim_state_t t = {0.0F, 1.0F, 0.0F};
  float v[TRIM_INPUTS];
  trim_values(&t, v);
  for (unsigned i = 0U; i < TRIM_INPUTS; ++i) assert((v[i] > 0.0F) == (LQG_ERT_B[LQG_X_THETA][i] > 0.0));
  t = (trim_state_t){0.0F, 0.0F, 1.0F};
  trim_values(&t, v);
  for (unsigned i = 0U; i < TRIM_INPUTS; ++i) assert((v[i] > 0.0F) == (LQG_ERT_B[LQG_X_PHI][i] > 0.0));
  t = (trim_state_t){5.0F, 0.0F, 0.0F};
  trim_values(&t, v);
  for (unsigned i = 0U; i < TRIM_INPUTS; ++i) assert(v[i] == 5.0F);
}

/** @brief Limites, congelamento e rampa de partida. */
static void test_limits_and_freeze(void) {
  trim_state_t t;
  trim_reset(&t);
  for (int k = 0; k < 40000; ++k) trim_update(&t, &CFG, true, 0.01F, true, 0.0F, 0.1F, 0.0F, 0.0F);
  assert(t.z == CFG.max_z);                              /* rampa de partida, no limite */
  trim_reset(&t);
  for (int k = 0; k < 20000; ++k) trim_update(&t, &CFG, true, 0.01F, false, 0.0F, 0.0F, 0.3F, 0.3F);
  assert(t.theta == -CFG.max_att && t.phi == -CFG.max_att);
  trim_reset(&t);
  trim_update(&t, &CFG, false, 0.01F, true, 0.0F, 0.1F, 0.3F, 0.3F);  /* desabilitado */
  assert(t.z == 0.0F && t.theta == 0.0F && t.phi == 0.0F);
  trim_update(&t, &CFG, true, 0.01F, false, 0.0F, NAN, 0.1F, 0.1F);   /* entrada inválida */
  assert(t.z == 0.0F);
  /* No ar, subindo rápido: o hover não integra. */
  trim_update(&t, &CFG, true, 0.01F, false, -0.3F, 0.1F, 0.0F, 0.0F);
  assert(t.z == 0.0F);
  trim_update(&t, &CFG, true, 0.01F, false, 0.0F, 0.1F, 0.0F, 0.0F);
  assert(t.z > 0.0F);

  /* Arfagem no limite: o hover não sobe (nem no chão), mas pode descer. */
  trim_reset(&t);
  t.theta = -CFG.max_att;
  trim_update(&t, &CFG, true, 0.01F, true, 0.0F, 0.1F, 0.0F, 0.2F);
  assert(t.z == 0.0F);
  trim_update(&t, &CFG, true, 0.01F, false, 0.0F, -0.1F, 0.0F, 0.2F);
  assert(t.z < 0.0F);

  /* No ar, um ângulo grande integra como se fosse o limite; no chão, inteiro. */
  trim_state_t air, ground;
  trim_reset(&air);
  trim_reset(&ground);
  trim_update(&air, &CFG, true, 1.0F, false, 0.0F, 0.0F, 0.0F, 0.3F);
  trim_update(&ground, &CFG, true, 1.0F, true, 0.0F, 0.0F, 0.0F, 0.3F);
  assert(fabsf(air.theta + CFG.ki_att * 0.08F) < 1e-4F);
  assert(fabsf(ground.theta + CFG.ki_att * 0.3F) < 1e-4F);
}

/** Ruído uniforme em [-1, 1]. */
static double noise(void) { return (rand() / (double)RAND_MAX - 0.5) * 2.0; }

/**
 * @brief Decolagem simulada com hover errado de @p dh rad/s e desequilíbrio de
 *        @p dth (arfagem) e @p dph (rolagem) rad/s, com chão.
 *
 * O Kalman recebe só o du (o trim é o novo ponto de hover).
 */
static void simulate_push(double dh, double dth, double dph, double push, double *rebound_deg);

/** @brief Decolagem sem empurrão (ver simulate_push). */
static void simulate(double dh, double dth, double dph) {
  double rebound;
  simulate_push(dh, dth, dph, 0.0, &rebound);
}

/**
 * @brief Decolagem e, depois de assentado, um empurrão de @p push rad/s na
 *        arfagem por 1,5 s. @p rebound_deg recebe a maior arfagem do lado
 *        oposto depois do empurrão.
 */
static void simulate_push(double dh, double dth, double dph, double push, double *rebound_deg) {
  srand(7);
  lqg_gains_t g = LQG_ERT_DEFAULT_GAINS;
  attitude_mode_mask_gains(&g);
  lqg_ert_t f;
  lqg_ert_init(&f);
  trim_state_t tr;
  trim_reset(&tr);
  static const unsigned idx[LQG_NY] = {LQG_X_Z, LQG_X_PHI, LQG_X_THETA, LQG_X_PSI, LQG_X_VX,
                                       LQG_X_VY, LQG_X_P, LQG_X_Q, LQG_X_R};
  static const double theta_sign[4] = {1, 1, -1, -1}, phi_sign[4] = {1, -1, -1, 1};
  const double ground = -0.08;
  double x[LQG_NX] = {0};
  x[LQG_X_Z] = ground;
  double zref = ground, max_height = 0.0;
  int released = 0;
  *rebound_deg = 0.0;
  for (int k = 0; k < 6000; ++k) {
    if (k >= 100) zref = fmax(zref - 0.05 * LQG_TS_S, ground - 0.15);
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
    lqg_ert_output(&f, &g, y, r, du);
    float tv[TRIM_INPUTS];
    trim_values(&tr, tv);
    lqg_ert_update(&f, &g, du);
    if (ground - f.x_filt[LQG_X_Z] >= 0.010) released = 1;
    const double extra = (k >= 4000 && k < 4150) ? push : 0.0;
    trim_update(&tr, &CFG, k >= 100, 0.01F, !released, (float)f.x_filt[LQG_X_VZ],
                (float)(f.x_filt[LQG_X_Z] - zref), (float)f.x_filt[LQG_X_PHI],
                (float)f.x_filt[LQG_X_THETA]);
    double next[LQG_NX];
    for (unsigned i = 0U; i < LQG_NX; ++i) {
      double sum = 0.0;
      for (unsigned j = 0U; j < LQG_NX; ++j) sum += LQG_ERT_A[i][j] * x[j];
      for (unsigned j = 0U; j < LQG_NU; ++j) {
        const double input = du[j] + tv[j] - dh - (dth + extra) * theta_sign[j] - dph * phi_sign[j];
        sum += LQG_ERT_B[i][j] * input;
      }
      next[i] = sum;
    }
    memcpy(x, next, sizeof(x));
    if (x[LQG_X_Z] > ground) {  /* chão */
      x[LQG_X_Z] = ground;
      if (x[LQG_X_VZ] > 0.0) x[LQG_X_VZ] = 0.0;
    }
    max_height = fmax(max_height, ground - x[LQG_X_Z]);
    if (k >= 4150) *rebound_deg = fmax(*rebound_deg, x[LQG_X_THETA] * 57.29578);
  }
  assert(fabs(x[LQG_X_THETA]) < 0.01 && fabs(x[LQG_X_PHI]) < 0.01);    /* < 0,6 grau */
  assert(fabs((ground - x[LQG_X_Z]) - 0.15) < 0.01);                    /* 15 cm, erro < 1 cm */
  assert(fabs(tr.z - dh) < 5.0 && fabs(tr.theta - dth) < 3.0 && fabs(tr.phi - dph) < 3.0);
  assert(max_height < 0.30);
  printf("  hover %+.0f, desequilibrio %+.0f/%+.0f: altura %.3f m, trim z=%.1f th=%.1f phi=%.1f\n",
         dh, dth, dph, ground - x[LQG_X_Z], tr.z, tr.theta, tr.phi);                                            /* abaixo do corte de pouso */
}

int main(void) {
  test_signs_match_b_matrix();
  test_limits_and_freeze();
  simulate(40.0, 15.0, 5.0);
  simulate(80.0, 25.0, 10.0);
  simulate(120.0, 30.0, -20.0);
  simulate(170.0, 125.0, 0.0);  /* desequilíbrio medido no ensaio de 04/11 */
  double rebound;
  simulate_push(170.0, 125.0, 0.0, 60.0, &rebound);  /* empurrão de 1,5 s */
  printf("  rebote depois do empurrao: %.1f graus\n", rebound);
  assert(rebound < 15.0);
  printf("test_trim: ok\n");
  return 0;
}
