/**
 * @file test_lqi.c
 * @brief LQI: projeto (dlqr em C), tabela gerada, integradores e voo simulado.
 */
#include "control/attitude_mode.h"
#include "control/lqg_ert.h"
#include "control/lqg_ert_model.h"
#include "control/lqi.h"
#include "control/lqi_tuning.h"
#include "lqi_design.h"
#include "lqr_design.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @brief O dlqr em C reproduz o Kd exportado pelo Simulink com os pesos do TCC. */
static void test_dlqr_reproduces_simulink(void) {
  double A[LQG_NX * LQG_NX], B[LQG_NX * LQG_NU], K[LQG_NU * LQG_NX];
  for (unsigned i = 0U; i < LQG_NX; ++i) {
    for (unsigned j = 0U; j < LQG_NX; ++j) A[i * LQG_NX + j] = LQG_ERT_A[i][j];
    for (unsigned j = 0U; j < LQG_NU; ++j) B[i * LQG_NU + j] = LQG_ERT_B[i][j];
  }
  lqi_weights_t w;
  lqi_tcc_weights(&w);
  assert(lqr_dlqr((int)LQG_NX, (int)LQG_NU, A, B, w.q, w.r, K) > 0);
  double worst = 0.0, scale = 0.0;
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned j = 0U; j < LQG_NX; ++j) {
      worst = fmax(worst, fabs(K[i * LQG_NX + j] - LQG_ERT_DEFAULT_GAINS.Kd[i][j]));
      scale = fmax(scale, fabs(LQG_ERT_DEFAULT_GAINS.Kd[i][j]));
    }
  }
  printf("  dlqr vs Simulink: erro relativo %.1e\n", worst / scale);
  assert(worst / scale < 1e-9);
}

/** @brief A tabela do firmware é a que o projeto gera agora. */
static void test_table_matches_design(void) {
  lqi_weights_t w;
  lqi_default_weights(&w);
  lqi_gains_t g;
  assert(lqi_design(&w, &g) > 0);
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned j = 0U; j < LQG_NX; ++j) assert(fabs(g.Kx[i][j] - LQI_GAINS.Kx[i][j]) < 1e-9);
    for (unsigned j = 0U; j < LQI_NI; ++j) assert(fabs(g.Ki[i][j] - LQI_GAINS.Ki[i][j]) < 1e-9);
    /* A posição horizontal fica fora do LQI. */
    assert(LQI_GAINS.Kx[i][LQG_X_X] == 0.0 && LQI_GAINS.Kx[i][LQG_X_VY] == 0.0);
    assert(LQI_GAINS.Kfd[i][LQG_R_X] == 0.0 && LQI_GAINS.Kfd[i][LQG_R_Y] == 0.0);
  }
}

/** @brief Sinais: abaixo do alvo sobe os quatro; nariz para cima sobe a traseira. */
static void test_integrator_signs_and_limits(void) {
  lqi_state_t s;
  lqi_reset(&s);
  const float below[LQI_NI] = {0.1F, 0.0F, 0.0F};   /* z NED acima da referência = abaixo do alvo */
  lqi_update(&s, &LQI_GAINS, true, 1.0F, below, 250.0F);
  float u[LQG_NU];
  lqi_output(&s, &LQI_GAINS, u);
  for (unsigned i = 0U; i < LQG_NU; ++i) assert(u[i] > 0.0F);

  lqi_reset(&s);
  const float nose_up[LQI_NI] = {0.0F, 0.0F, 0.1F};
  lqi_update(&s, &LQI_GAINS, true, 1.0F, nose_up, 250.0F);
  lqi_output(&s, &LQI_GAINS, u);
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    /* Para baixar o nariz, o empuxo cai onde B dá θ positivo (frente). */
    assert((u[i] < 0.0F) == (LQG_ERT_B[LQG_X_THETA][i] > 0.0));
  }

  /* Limite: a correção de um integrador num motor não passa do limite. */
  lqi_reset(&s);
  for (int k = 0; k < 10000; ++k) lqi_update(&s, &LQI_GAINS, true, 0.01F, nose_up, 100.0F);
  lqi_output(&s, &LQI_GAINS, u);
  for (unsigned i = 0U; i < LQG_NU; ++i) assert(fabsf(u[i]) <= 100.0F + 1e-3F);
  assert(lqi_at_limit(&s, &LQI_GAINS, 100.0F));

  /* Congelado e entrada inválida não mexem. */
  const lqi_state_t before = s;
  lqi_update(&s, &LQI_GAINS, false, 0.01F, below, 100.0F);
  const float bad[LQI_NI] = {NAN, NAN, NAN};
  lqi_update(&s, &LQI_GAINS, true, 0.01F, bad, 100.0F);
  assert(memcmp(&before, &s, sizeof(s)) == 0);
}

/** @brief O desconto do pouso tira exatamente o pedido do hover e respeita o limite. */
static void test_reduce_hover(void) {
  lqi_state_t s;
  lqi_reset(&s);
  const float below[LQI_NI] = {0.1F, 0.02F, -0.03F};
  lqi_update(&s, &LQI_GAINS, true, 5.0F, below, 250.0F);
  float u[LQG_NU], v[LQG_NU];
  lqi_output(&s, &LQI_GAINS, u);
  lqi_reduce_hover(&s, &LQI_GAINS, 10.0F, 250.0F);
  lqi_output(&s, &LQI_GAINS, v);
  /* Cai 10 rad/s em cada motor; φ e θ não mudam. */
  for (unsigned i = 0U; i < LQG_NU; ++i) assert(fabsf((u[i] - v[i]) - 10.0F) < 1e-3F);
  assert(s.xi[LQI_I_PHI] != 0.0F && s.xi[LQI_I_THETA] != 0.0F);

  /* Não passa do limite negativo, e delta <= 0 não mexe. */
  lqi_reset(&s);
  for (int k = 0; k < 1000; ++k) lqi_reduce_hover(&s, &LQI_GAINS, 10.0F, 100.0F);
  lqi_output(&s, &LQI_GAINS, u);
  for (unsigned i = 0U; i < LQG_NU; ++i) assert(fabsf(u[i] + 100.0F) < 1e-2F);
  const lqi_state_t before = s;
  lqi_reduce_hover(&s, &LQI_GAINS, 0.0F, 100.0F);
  lqi_reduce_hover(&s, &LQI_GAINS, NAN, 100.0F);
  assert(memcmp(&before, &s, sizeof(s)) == 0);
}

/** @brief Prepara no lqi_tuning todos os valores de @p g; pula o de índice @p skip. */
static void stage_all(const lqi_gains_t *g, int skip) {
  lqi_tuning_begin();
  int k = 0;
  for (unsigned r = 0U; r < LQG_NU; ++r) {
    for (unsigned c = 0U; c < LQG_NX; ++c, ++k) {
      if (k != skip) assert(lqi_tuning_stage(LQI_TUNING_KX, r, c, (float)g->Kx[r][c]) == LQI_TUNING_OK);
    }
  }
  for (unsigned r = 0U; r < LQG_NU; ++r) {
    for (unsigned c = 0U; c < LQG_NR; ++c, ++k) {
      if (k != skip) assert(lqi_tuning_stage(LQI_TUNING_KFD, r, c, (float)g->Kfd[r][c]) == LQI_TUNING_OK);
    }
  }
  for (unsigned r = 0U; r < LQG_NU; ++r) {
    for (unsigned c = 0U; c < LQI_NI; ++c, ++k) {
      if (k != skip) assert(lqi_tuning_stage(LQI_TUNING_KI, r, c, (float)g->Ki[r][c]) == LQI_TUNING_OK);
    }
  }
  assert(k == (int)LQI_TUNING_VALUES);
}

/** @brief Envio de ganhos do LQI: só troca com tudo, finito e CRC certo. */
static void test_tuning(void) {
  lqi_tuning_init();
  assert(lqi_tuning_is_default());
  const uint32_t default_crc = lqi_tuning_active_crc();
  assert(default_crc == lqi_tuning_crc(&LQI_GAINS));
  printf("lqi_default_crc=%08X\n", (unsigned)default_crc);

  /* Pesos diferentes: o dobro em φ e θ. */
  lqi_weights_t w;
  lqi_default_weights(&w);
  w.q[LQG_X_PHI] *= 2.0;
  w.q[LQG_X_THETA] *= 2.0;
  lqi_gains_t g;
  assert(lqi_design(&w, &g) > 0);
  const uint32_t crc = lqi_tuning_crc(&g);
  uint32_t computed = 0U;

  stage_all(&g, 17);  /* falta um */
  assert(lqi_tuning_commit(crc, &computed) == LQI_TUNING_INCOMPLETE);
  stage_all(&g, -1);
  assert(lqi_tuning_commit(crc ^ 1U, &computed) == LQI_TUNING_CRC_MISMATCH && computed == crc);
  assert(lqi_tuning_is_default());
  assert(lqi_tuning_stage(LQI_TUNING_KI, 4U, 0U, 1.0F) == LQI_TUNING_BAD_INDEX);
  assert(lqi_tuning_stage(LQI_TUNING_KI, 0U, 3U, 1.0F) == LQI_TUNING_BAD_INDEX);
  assert(lqi_tuning_stage(LQI_TUNING_COUNT, 0U, 0U, 1.0F) == LQI_TUNING_BAD_INDEX);

  const uint32_t before = lqi_tuning_generation();
  assert(lqi_tuning_commit(crc, &computed) == LQI_TUNING_OK);
  assert(!lqi_tuning_is_default() && lqi_tuning_active_crc() == crc);
  assert(lqi_tuning_generation() == before + 1U);
  lqi_gains_t active;
  lqi_tuning_copy_active(&active, NULL);
  assert(fabs(active.Kx[0][LQG_X_PHI] - g.Kx[0][LQG_X_PHI]) < 1e-4 * fabs(g.Kx[0][LQG_X_PHI]));
  assert(fabs(active.Kx[0][LQG_X_PHI]) > fabs(LQI_GAINS.Kx[0][LQG_X_PHI]));
  /* O commit consome o preparo. */
  assert(lqi_tuning_commit(crc, NULL) == LQI_TUNING_INCOMPLETE);

  stage_all(&g, -1);
  assert(lqi_tuning_stage(LQI_TUNING_KX, 0U, 0U, NAN) == LQI_TUNING_OK);
  assert(lqi_tuning_commit(crc, NULL) == LQI_TUNING_NOT_FINITE);

  lqi_tuning_restore_default();
  assert(lqi_tuning_is_default() && lqi_tuning_active_crc() == default_crc);
}

/** Ruído uniforme em [-1, 1]. */
static double noise(void) { return (rand() / (double)RAND_MAX - 0.5) * 2.0; }

/**
 * @brief Voo simulado como no firmware: Kalman do Simulink recebendo só a parte
 *        do LQR, integradores alimentados pelas medidas, motores com atraso de
 *        primeira ordem @p tau, hover errado em @p dh rad/s, desequilíbrio de
 *        @p dth (arfagem) e @p dph (rolagem), chão, e um empurrão de 1,5 s.
 */
static void simulate(double dh, double dth, double dph, double tau, double push) {
  srand(9);
  lqg_gains_t eff = LQG_ERT_DEFAULT_GAINS;
  lqi_apply_gains(&LQI_GAINS, &eff);
  attitude_mode_mask_gains(&eff);
  lqg_ert_t f;
  lqg_ert_init(&f);
  lqi_state_t integ;
  lqi_reset(&integ);
  static const unsigned idx[LQG_NY] = {LQG_X_Z, LQG_X_PHI, LQG_X_THETA, LQG_X_PSI, LQG_X_VX,
                                       LQG_X_VY, LQG_X_P, LQG_X_Q, LQG_X_R};
  static const double theta_sign[4] = {1, 1, -1, -1}, phi_sign[4] = {1, -1, -1, 1};
  const double ground = -0.08;
  double x[LQG_NX] = {0}, u_motor[4] = {0};
  x[LQG_X_Z] = ground;
  double zref = ground, max_height = 0.0, rebound = 0.0;
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
    /* Anti-windup como no firmware: no ar, ângulo integrado limitado a 0,08 rad. */
    const float limit = (ground - x[LQG_X_Z] > 0.03) ? 0.08F : INFINITY;
    const float error[LQI_NI] = {(float)(y[LQG_Y_Z] - zref),
                                 fmaxf(fminf((float)y[LQG_Y_PHI], limit), -limit),
                                 fmaxf(fminf((float)y[LQG_Y_THETA], limit), -limit)};
    attitude_mode_repeat_prediction(&f, y);
    lqg_ert_output(&f, &eff, y, r, du);
    float ui[LQG_NU];
    lqi_output(&integ, &LQI_GAINS, ui);
    lqg_ert_update(&f, &eff, du);
    lqi_update(&integ, &LQI_GAINS, k >= 100, 0.01F, error, 250.0F);
    const double extra = (k >= 4000 && k < 4150) ? push : 0.0;
    const double alpha = tau > 0.0 ? 0.01 / (tau + 0.01) : 1.0;
    double next[LQG_NX];
    for (unsigned j = 0U; j < LQG_NU; ++j) u_motor[j] += alpha * ((du[j] + ui[j]) - u_motor[j]);
    for (unsigned i = 0U; i < LQG_NX; ++i) {
      double sum = 0.0;
      for (unsigned j = 0U; j < LQG_NX; ++j) sum += LQG_ERT_A[i][j] * x[j];
      for (unsigned j = 0U; j < LQG_NU; ++j) {
        sum += LQG_ERT_B[i][j] * (u_motor[j] - dh - (dth + extra) * theta_sign[j] - dph * phi_sign[j]);
      }
      next[i] = sum;
    }
    memcpy(x, next, sizeof(x));
    if (x[LQG_X_Z] > ground) {
      x[LQG_X_Z] = ground;
      if (x[LQG_X_VZ] > 0.0) x[LQG_X_VZ] = 0.0;
    }
    max_height = fmax(max_height, ground - x[LQG_X_Z]);
    if (k >= 4150) rebound = fmax(rebound, fabs(x[LQG_X_THETA]) * 57.29578);
  }
  const double height = ground - x[LQG_X_Z];
  printf("  hover %+4.0f, deseq. %+4.0f/%+4.0f, atraso %3.0f ms: altura %.3f m (max %.3f), "
         "theta %+.2f, phi %+.2f graus, rebote do empurrao %.1f graus\n",
         dh, dth, dph, tau * 1000.0, height, max_height, x[LQG_X_THETA] * 57.29578,
         x[LQG_X_PHI] * 57.29578, rebound);
  assert(fabs(height - 0.15) < 0.01);
  assert(fabs(x[LQG_X_THETA]) < 0.005 && fabs(x[LQG_X_PHI]) < 0.005);  /* < 0,3 grau */
  assert(max_height < 0.30);
  assert(rebound < 30.0);
}

int main(void) {
  test_dlqr_reproduces_simulink();
  test_table_matches_design();
  test_integrator_signs_and_limits();
  test_reduce_hover();
  test_tuning();
  simulate(30.0, 125.0, 10.0, 0.0, 30.0);
  simulate(30.0, 125.0, 10.0, 0.04, 30.0);
  simulate(30.0, 125.0, 10.0, 0.08, 30.0);
  simulate(170.0, 125.0, 0.0, 0.04, 30.0);
  simulate(0.0, 0.0, 0.0, 0.04, 30.0);
  printf("test_lqi: ok\n");
  return 0;
}
