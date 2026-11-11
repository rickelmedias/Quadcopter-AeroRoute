/**
 * @file test_sim_plant.c
 * @brief Drone simulado + LQG do TCC: decola, estabiliza, gira e anda.
 *
 * Reproduz o que o voo faz com FLT_SIM = 1 (referência em rampa e critério de
 * "estável" de flight_ctrl.c) e confere que cada etapa termina num tempo
 * razoável, no lugar certo.
 */
#include "control/lqg_ert.h"
#include "control/lqg_ert_model.h"
#include "control/sim_plant.h"
#include "flight_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** Passo, em s. */
#define DT      0.01
/** Tolerâncias padrão de flight_ctrl.c. */
#define POS_TOL 0.05
#define YAW_TOL 0.05

static sim_plant_t sim;
static lqg_ert_t lqg;
static double ref[LQG_NR];

/** @brief Aproxima @p cur de @p target em no máximo @p step. */
static double approach(double cur, double target, double step) {
  const double d = target - cur;
  return d > step ? cur + step : d < -step ? cur - step : target;
}

/**
 * @brief Persegue o alvo em rampa até ficar estável (como check_goal).
 * @return Tempo até estável, em s (negativo se não estabilizou em 30 s).
 */
static double run_until_stable(const double target[LQG_NR]) {
  double settle = -1.0;
  for (int k = 0; k < 3000; ++k) {
    ref[LQG_R_X] = approach(ref[LQG_R_X], target[LQG_R_X], 0.3 * DT);
    ref[LQG_R_Y] = approach(ref[LQG_R_Y], target[LQG_R_Y], 0.3 * DT);
    ref[LQG_R_Z] = approach(ref[LQG_R_Z], target[LQG_R_Z], 0.3 * DT);
    ref[LQG_R_PSI] = approach(ref[LQG_R_PSI], target[LQG_R_PSI], 0.5 * DT);
    lqg_real_t y[LQG_NY], r[LQG_NR], du[LQG_NU];
    sim_plant_measure(&sim, y);
    for (unsigned i = 0U; i < LQG_NR; ++i)
      r[i] = ref[i];
    lqg_ert_output(&lqg, &LQG_ERT_DEFAULT_GAINS, y, r, du);
    lqg_ert_update(&lqg, &LQG_ERT_DEFAULT_GAINS, du);
    sim_plant_step(&sim, du);
    const lqg_real_t *x = lqg.x_filt;
    const bool ramp = memcmp(ref, target, sizeof(ref)) == 0;
    const bool there = fabs(x[LQG_X_X] - target[LQG_R_X]) < POS_TOL &&
                       fabs(x[LQG_X_Y] - target[LQG_R_Y]) < POS_TOL &&
                       fabs(x[LQG_X_Z] - target[LQG_R_Z]) < POS_TOL &&
                       fabs(x[LQG_X_PSI] - target[LQG_R_PSI]) < YAW_TOL;
    const bool still = fabs(x[LQG_X_VX]) < FLIGHT_SETTLE_SPEED_M_S &&
                       fabs(x[LQG_X_VY]) < FLIGHT_SETTLE_SPEED_M_S &&
                       fabs(x[LQG_X_VZ]) < FLIGHT_SETTLE_SPEED_M_S &&
                       fabs(x[LQG_X_P]) < FLIGHT_SETTLE_RATE_RAD_S &&
                       fabs(x[LQG_X_Q]) < FLIGHT_SETTLE_RATE_RAD_S &&
                       fabs(x[LQG_X_R]) < FLIGHT_SETTLE_RATE_RAD_S;
    if (!(ramp && there && still)) {
      settle = -1.0;
      continue;
    }
    if (settle < 0.0)
      settle = k * DT;
    if (k * DT - settle >= FLIGHT_SETTLE_MS / 1000.0)
      return k * DT;
  }
  return -1.0;
}

int main(void) {
  sim_plant_reset(&sim, SIM_PLANT_GROUND_Z, 0.0F);
  lqg_ert_init(&lqg);
  memset(ref, 0, sizeof(ref));
  ref[LQG_R_Z] = SIM_PLANT_GROUND_Z;

  /* Parado no chão com du = 0: não sai do lugar. */
  for (int k = 0; k < 100; ++k) {
    const lqg_real_t zero[LQG_NU] = {0};
    sim_plant_step(&sim, zero);
  }
  assert(fabs(sim.x[LQG_X_Z] - SIM_PLANT_GROUND_Z) < 1e-9);

  /* Na partida (motores abaixo do hover) com o desequilíbrio fixo do drone
     real (CTL_FF_TH/PHI/PSI): apoiado no chão, não tomba nem gira. */
  const lqg_real_t unbalanced[LQG_NU] = {-900.0 - 140.0, -900.0 + 40.0, -900.0 + 60.0,
                                         -900.0 + 40.0};
  for (int k = 0; k < 200; ++k)
    sim_plant_step(&sim, unbalanced);
  assert(fabs(sim.x[LQG_X_Z] - SIM_PLANT_GROUND_Z) < 1e-9);
  for (unsigned i = LQG_X_PHI; i <= LQG_X_R; ++i) {
    if (i != LQG_X_Z && i != LQG_X_PSI && i != LQG_X_VZ)
      assert(sim.x[i] == 0.0);
  }
  assert(sim.x[LQG_X_PSI] == 0.0 && sim.x[LQG_X_X] == 0.0 && sim.x[LQG_X_Y] == 0.0);
  /* No ar, o mesmo desequilíbrio inclina: o chão só segura enquanto apoia. */
  sim_plant_t air = sim;
  air.x[LQG_X_Z] = SIM_PLANT_GROUND_Z - 0.3;
  const lqg_real_t tilt[LQG_NU] = {-140.0, 40.0, 60.0, 40.0};
  for (int k = 0; k < 20; ++k)
    sim_plant_step(&air, tilt);
  assert(fabs(air.x[LQG_X_PHI]) + fabs(air.x[LQG_X_THETA]) > 1e-3);

  /* Decolar 0,5 m. */
  double target[LQG_NR] = {0.0, 0.0, SIM_PLANT_GROUND_Z - 0.5, 0.0};
  double t = run_until_stable(target);
  printf("decolar 0,5 m: estavel em %.1f s\n", t);
  assert(t > 0.0 && t < 15.0);

  /* Girar 90° (horário visto de cima). */
  target[LQG_R_PSI] += 1.5707963267948966;
  t = run_until_stable(target);
  printf("girar 90 graus: estavel em %.1f s\n", t);
  assert(t > 0.0 && t < 15.0);

  /* Frente 1 m: com ψ = 90° (nariz para leste, NED) é y + 1. */
  const double psi = target[LQG_R_PSI];
  target[LQG_R_X] += 1.0 * cos(psi);
  target[LQG_R_Y] += 1.0 * sin(psi);
  t = run_until_stable(target);
  printf("frente 1 m: estavel em %.1f s, drone em x=%.2f y=%.2f\n", t, sim.x[LQG_X_X],
         sim.x[LQG_X_Y]);
  assert(t > 0.0 && t < 20.0);
  assert(fabs(sim.x[LQG_X_X]) < 0.06 && fabs(sim.x[LQG_X_Y] - 1.0) < 0.06);

  /* Descer até o chão: o chão segura. */
  target[LQG_R_Z] = SIM_PLANT_GROUND_Z + 0.2;
  for (int k = 0; k < 1000; ++k) {
    lqg_real_t y[LQG_NY], r[LQG_NR], du[LQG_NU];
    ref[LQG_R_Z] = approach(ref[LQG_R_Z], target[LQG_R_Z], 0.3 * DT);
    sim_plant_measure(&sim, y);
    for (unsigned i = 0U; i < LQG_NR; ++i)
      r[i] = ref[i];
    lqg_ert_output(&lqg, &LQG_ERT_DEFAULT_GAINS, y, r, du);
    lqg_ert_update(&lqg, &LQG_ERT_DEFAULT_GAINS, du);
    sim_plant_step(&sim, du);
    assert(sim.x[LQG_X_Z] <= SIM_PLANT_GROUND_Z + 1e-9);
  }
  printf("test_sim_plant: ok\n");
  return 0;
}
