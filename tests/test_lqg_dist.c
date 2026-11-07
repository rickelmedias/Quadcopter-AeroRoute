/**
 * @file test_lqg_dist.c
 * @brief LQG ajustado e estimador de perturbação: unidades e voo simulado.
 */
#include "control/attitude_mode.h"
#include "control/lqg_dist.h"
#include "control/lqg_ert.h"
#include "control/lqg_ert_model.h"
#include "control/lqg_tuning.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief Entrada da planta simulada: o B do TCC com a guinada na autoridade
 *        real (a do drone, ~3e-4 da do TCC), como o projeto do ajustado.
 */
static void plant_input(const double u[LQG_NU], double out[LQG_NU]) {
  lqg_real_t in[LQG_NU], o[LQG_NU];
  for (unsigned j = 0U; j < LQG_NU; ++j) in[j] = (lqg_real_t)u[j];
  lqg_dist_kalman_input(in, o);
  for (unsigned j = 0U; j < LQG_NU; ++j) out[j] = (double)o[j];
}

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
  if (adjusted) {
    lqg_dist_apply_gains(&eff);
    lqg_dist_apply_kalman(&eff);
  }
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
    if (adjusted) {
      lqg_real_t du_model[LQG_NU];
      lqg_dist_kalman_input(du, du_model);
      lqg_ert_update(&f, &eff, du_model);
    } else {
      lqg_ert_update(&f, &eff, du);
    }
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
    /* O hover errado e os desequilíbrios são erros de comando (rad/s nos
       motores): passam pela planta real junto com ele. */
    double u_cmd[LQG_NU], u_plant[LQG_NU];
    for (unsigned j = 0U; j < LQG_NU; ++j) {
      u_cmd[j] = u_motor[j] - dh - (dth + push) * theta_sign[j] - dph * phi_sign[j];
    }
    plant_input(u_cmd, u_plant);
    for (unsigned i = 0U; i < LQG_NX; ++i) {
      double sum = 0.0;
      for (unsigned j = 0U; j < LQG_NX; ++j) sum += LQG_ERT_A[i][j] * x[j];
      for (unsigned j = 0U; j < LQG_NU; ++j) sum += LQG_ERT_B[i][j] * u_plant[j];
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

/** @brief Envio de Kd/Kfd: padrão = gerado; commit só completo e com CRC certo. */
static void test_tuning(void) {
  lqg_tuning_init();
  assert(lqg_tuning_is_default());
  lqg_aj_gains_t g;
  uint32_t gen0;
  lqg_tuning_copy_active(&g, &gen0);
  assert(g.Kd[0][2] == LQG_AJ_KD[0][2] && g.Kfd[3][3] == LQG_AJ_KFD[3][3]);
  const uint32_t crc0 = lqg_tuning_crc(&g);
  assert(lqg_tuning_active_crc() == crc0);

  lqg_aj_gains_t novo = g;
  novo.Kd[1][4] += 1.0F;
  novo.Kfd[2][1] -= 0.5F;
  lqg_tuning_begin();
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned j = 0U; j < LQG_NX; ++j) {
      assert(lqg_tuning_stage(LQG_TUNING_KD, i, j, (float)novo.Kd[i][j]) == LQG_TUNING_OK);
    }
    for (unsigned j = 0U; j < LQG_NR; ++j) {
      if (i == 3U && j == 3U) continue;  /* falta um valor */
      assert(lqg_tuning_stage(LQG_TUNING_KFD, i, j, (float)novo.Kfd[i][j]) == LQG_TUNING_OK);
    }
  }
  uint32_t calc;
  const uint32_t crc1 = lqg_tuning_crc(&novo);
  assert(lqg_tuning_commit(crc1, &calc) == LQG_TUNING_INCOMPLETE);
  assert(lqg_tuning_stage(LQG_TUNING_KFD, 3U, 3U, (float)novo.Kfd[3][3]) == LQG_TUNING_OK);
  assert(lqg_tuning_stage(LQG_TUNING_KD, 4U, 0U, 0.0F) == LQG_TUNING_BAD_INDEX);
  assert(lqg_tuning_commit(crc1 ^ 1U, &calc) == LQG_TUNING_CRC_MISMATCH && calc == crc1);
  assert(lqg_tuning_commit(crc1, &calc) == LQG_TUNING_OK);
  assert(!lqg_tuning_is_default() && lqg_tuning_active_crc() == crc1);
  uint32_t gen1;
  lqg_aj_gains_t usado;
  lqg_tuning_copy_active(&usado, &gen1);
  assert(gen1 != gen0 && (float)usado.Kd[1][4] == (float)novo.Kd[1][4]);

  lqg_gains_t gains;
  memset(&gains, 0, sizeof(gains));
  lqg_tuning_apply(&usado, &gains);
  assert(gains.Kd[1][4] == usado.Kd[1][4] && gains.Kfd[2][1] == usado.Kfd[2][1]);

  lqg_tuning_restore_default();
  assert(lqg_tuning_is_default() && lqg_tuning_active_crc() == crc0);
}

/**
 * @brief Segura o ponto (0, 0) com o drone virado ψ, como o voo com CTL_XY.
 *
 * A verdade evolui no referencial da proa (o modelo linear vale ali para
 * qualquer ψ constante); o fluxo chega no local, girado de ψ. Com @p rotate
 * o LQG roda na proa (attitude_mode_rotate_*), como o firmware.
 * @return Distância final ao ponto, em m (NAN se divergiu).
 */
static double simulate_xy(float psi, bool rotate) {
  lqg_gains_t eff = LQG_ERT_DEFAULT_GAINS;
  lqg_dist_apply_gains(&eff);
  lqg_dist_apply_kalman(&eff);
  lqg_ert_t f;
  lqg_ert_init(&f);
  static const unsigned idx[LQG_NY] = {LQG_X_Z, LQG_X_PHI, LQG_X_THETA, LQG_X_PSI, LQG_X_VX,
                                       LQG_X_VY, LQG_X_P, LQG_X_Q, LQG_X_R};
  double x[LQG_NX] = {0}, u_motor[4] = {0};
  x[LQG_X_Z] = -0.3;
  x[LQG_X_PSI] = psi;
  /* Começa no ponto, empurrado a 0,3 m/s para o norte e 0,2 m/s para oeste
     (a posição não é medida: o Kalman a integra do fluxo). */
  lqg_real_t vn = 0.3, ve = -0.2;
  attitude_mode_rotate_pair(&vn, &ve, -psi);
  x[LQG_X_VX] = vn;
  x[LQG_X_VY] = ve;
  for (int k = 0; k < 2000; ++k) {
    lqg_real_t y[LQG_NY], du[LQG_NU];
    lqg_real_t r[LQG_NR] = {0.0, 0.0, -0.3, psi};
    for (unsigned i = 0U; i < LQG_NY; ++i) y[i] = x[idx[i]];
    attitude_mode_rotate_pair(&y[LQG_Y_VX], &y[LQG_Y_VY], psi);  /* fluxo no local */
    if (rotate) {
      attitude_mode_rotate_pair(&y[LQG_Y_VX], &y[LQG_Y_VY], -psi);
      attitude_mode_rotate_pair(&r[LQG_R_X], &r[LQG_R_Y], -psi);
      if (!f.ic_load) attitude_mode_rotate_horizontal(f.x_pred, -psi);
    }
    lqg_ert_output(&f, &eff, y, r, du);
    lqg_real_t du_model[LQG_NU];
    lqg_dist_kalman_input(du, du_model);
    lqg_ert_update(&f, &eff, du_model);
    if (rotate) {
      attitude_mode_rotate_horizontal(f.x_pred, psi);
      attitude_mode_rotate_horizontal(f.x_filt, psi);
    }
    double next[LQG_NX];
    for (unsigned j = 0U; j < LQG_NU; ++j) u_motor[j] += 0.2 * (du[j] - u_motor[j]);
    double u_plant[LQG_NU];
    plant_input(u_motor, u_plant);
    for (unsigned i = 0U; i < LQG_NX; ++i) {
      double sum = 0.0;
      for (unsigned j = 0U; j < LQG_NX; ++j) sum += LQG_ERT_A[i][j] * x[j];
      for (unsigned j = 0U; j < LQG_NU; ++j) sum += LQG_ERT_B[i][j] * u_plant[j];
      next[i] = sum;
    }
    memcpy(x, next, sizeof(x));
    if (fabs(x[LQG_X_X]) > 5.0 || fabs(x[LQG_X_PHI]) > 1.0 || fabs(x[LQG_X_THETA]) > 1.0) return NAN;
  }
  return hypot(x[LQG_X_X], x[LQG_X_Y]);
}

/** @brief Com o LQG na proa o ponto é seguro em qualquer ψ; sem, só perto de 0. */
static void test_xy_heading_frame(void) {
  const float psis[] = {0.0F, 1.0F, -1.68F, 2.6F, -3.0F};
  for (unsigned i = 0U; i < sizeof(psis) / sizeof(psis[0]); ++i) {
    const double d = simulate_xy(psis[i], true);
    printf("  XY na proa, psi %+5.0f graus: %.1f cm do ponto apos 20 s\n", psis[i] * 57.29578,
           d * 100.0);
    assert(isfinite(d) && d < 0.01);
  }
  /* Sem girar, com o drone de costas o controle de posição empurra para fora. */
  const double d = simulate_xy(2.6F, false);
  printf("  XY sem girar, psi +149 graus: %s\n", isfinite(d) ? "nao diverge" : "diverge");
  assert(!(isfinite(d) && d < 0.05));
}

/** @brief Kalman do ajustado: só y e vy mudam, e vy passa a confiar no fluxo como vx. */
static void test_adjusted_kalman(void) {
  for (unsigned i = 0U; i < LQG_NX; ++i) {
    for (unsigned j = 0U; j < LQG_NY; ++j) {
      if (i == LQG_X_Y || i == LQG_X_VY) continue;
      assert(fabs(LQG_AJ_M[i][j] - LQG_ERT_DEFAULT_GAINS.M[i][j]) < 1e-7);  /* φ<-vy: 4e-9 */
    }
  }
  const double vx = LQG_AJ_M[LQG_X_VX][LQG_Y_VX], vy = LQG_AJ_M[LQG_X_VY][LQG_Y_VY];
  assert(fabs(vx - vy) < 1e-6);
  assert(LQG_ERT_DEFAULT_GAINS.M[LQG_X_VY][LQG_Y_VY] > 1.8 * vy);
}

/**
 * @brief Guinada com torque constante (hélice/motor diferente), LQG melhorado.
 *
 * Planta: o modelo do TCC com a autoridade de guinada real multiplicada por
 * @p authority (0,5 a 2: a real não é conhecida com precisão) e o torque como
 * @p torque rad/s por motor no padrão de guinada. Começa 30° fora do alvo.
 * @return Maior |ψ| nos últimos 5 s de 30, em graus (NAN se divergiu).
 */
static double simulate_yaw(double authority, double torque, double tau) {
  srand(5);
  lqg_gains_t eff = LQG_ERT_DEFAULT_GAINS;
  lqg_dist_apply_gains(&eff);
  lqg_dist_apply_kalman(&eff);
  lqg_ert_t f;
  lqg_ert_init(&f);
  lqg_dist_t e;
  lqg_dist_reset(&e);
  static const unsigned idx[LQG_NY] = {LQG_X_Z, LQG_X_PHI, LQG_X_THETA, LQG_X_PSI, LQG_X_VX,
                                       LQG_X_VY, LQG_X_P, LQG_X_Q, LQG_X_R};
  const float *pat = LQG_DIST_CHANNELS[LQG_DIST_PSI].pattern;
  double x[LQG_NX] = {0}, u_motor[4] = {0};
  x[LQG_X_Z] = -0.3;
  x[LQG_X_PSI] = 0.52;
  double worst = 0.0;
  const double alpha = 0.01 / (tau + 0.01);
  for (int k = 0; k < 3000; ++k) {
    lqg_real_t y[LQG_NY], du[LQG_NU];
    const lqg_real_t r[LQG_NR] = {0.0, 0.0, -0.3, 0.0};
    for (unsigned i = 0U; i < LQG_NY; ++i) y[i] = x[idx[i]];
    y[LQG_Y_PSI] += 0.002 * noise();
    y[LQG_Y_R] += 0.02 * noise();
    attitude_mode_repeat_prediction(&f, y);
    lqg_ert_output(&f, &eff, y, r, du);
    float comp[LQG_NU];
    lqg_dist_correct(&e, y, true);
    lqg_dist_output(&e, comp);
    lqg_real_t du_model[LQG_NU];
    lqg_dist_kalman_input(du, du_model);
    lqg_ert_update(&f, &eff, du_model);
    float u[LQG_NU];
    for (unsigned j = 0U; j < LQG_NU; ++j) u[j] = (float)du[j] + comp[j];
    lqg_dist_predict(&e, u, false, false, 400.0F);
    for (unsigned j = 0U; j < LQG_NU; ++j) u_motor[j] += alpha * (u[j] - u_motor[j]);
    /* Componente de guinada na autoridade real (× authority), com o torque. */
    double yaw = 0.0;
    for (unsigned j = 0U; j < LQG_NU; ++j) yaw += pat[j] * u_motor[j];
    yaw = yaw / LQG_NU - torque;
    const double keep = LQG_AJ_YAW_SCALE * authority;
    /* u_plant vai no B do TCC: a componente de guinada já vem multiplicada por keep. */
    double u_plant[LQG_NU];
    for (unsigned j = 0U; j < LQG_NU; ++j) {
      u_plant[j] = u_motor[j] - pat[j] * (yaw + torque) + pat[j] * yaw * keep;
    }
    double next[LQG_NX];
    for (unsigned i = 0U; i < LQG_NX; ++i) {
      double sum = 0.0;
      for (unsigned j = 0U; j < LQG_NX; ++j) sum += LQG_ERT_A[i][j] * x[j];
      for (unsigned j = 0U; j < LQG_NU; ++j) sum += LQG_ERT_B[i][j] * u_plant[j];
      next[i] = sum;
    }
    memcpy(x, next, sizeof(x));
    if (!(fabs(x[LQG_X_PSI]) < 20.0)) return NAN;
    if (k >= 2500) worst = fmax(worst, fabs(x[LQG_X_PSI]) * 57.29578);
  }
  return worst;
}

/** @brief O estimador de guinada zera um torque constante mesmo com a autoridade errada. */
static void test_yaw_disturbance(void) {
  const double authorities[] = {0.5, 1.0, 2.0};
  for (unsigned a = 0U; a < 3U; ++a) {
    for (int t = 0; t < 2; ++t) {
      const double tau = t ? 0.12 : 0.04;
      const double w = simulate_yaw(authorities[a], 40.0, tau);
      printf("  guinada: autoridade x%.1f, atraso %3.0f ms, torque 40: %.2f graus nos ultimos 5 s\n",
             authorities[a], tau * 1000.0, w);
      assert(isfinite(w) && w < 2.0);
    }
  }
}

int main(void) {
  test_tuning();
  test_yaw_disturbance();
  test_adjusted_kalman();
  test_xy_heading_frame();
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
