/**
 * @file test_meas.c
 * @brief Conversão das leituras dos sensores no vetor de medidas do modelo.
 */
#include "control/meas.h"
#include "control/ctrl_params.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** Verdadeiro se @p a e @p b diferem no máximo @p tol. */
#define NEAR(a, b, tol) (fabs((double)(a) - (double)(b)) <= (tol))

/** @brief Leitura com o drone nivelado, parado, a @p range m do chão. */
static meas_input_t level(float range) {
  meas_input_t in;
  memset(&in, 0, sizeof(in));
  in.q[0] = 1.0F;
  in.attitude_valid = in.gyro_valid = in.range_valid = in.flow_valid = true;
  in.range_m = range;
  return in;
}

/** @brief Parâmetros padrão, com o zero de nível do BNO em 0 (BNO sem montagem invertida). */
static ctrl_params_t defaults(void) {
  ctrl_params_init();
  ctrl_params_t p;
  ctrl_params_snapshot(&p);
  p.zero_roll_deg = 0.0F;
  p.zero_pitch_deg = 0.0F;
  return p;
}

/** @brief Altura com o drone nivelado e inclinado (cosseno da inclinação). */
static void test_height(void) {
  ctrl_params_t p = defaults();
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];
  meas_input_t in = level(0.5F);
  assert(meas_update(&s, &in, &p, y) == (MEAS_ATTITUDE | MEAS_GYRO | MEAS_RANGE | MEAS_FLOW));
  assert(NEAR(y[LQG_Y_Z], -0.5, 1e-6));

  /* Rolagem de 30 graus: o telêmetro mede a mais, a altura é range*cos. */
  in.roll = 0.5235988F;
  in.range_m = 0.6F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_Z], -0.6 * cos(0.5235988), 1e-5));
}

/** @brief BNO montado de cabeça para baixo: roll/pitch são medidos a partir do zero de nível. */
static void test_inverted_mount_zero(void) {
  ctrl_params_init();
  ctrl_params_t p;
  ctrl_params_snapshot(&p);  /* zero padrão: roll -174,5 e pitch -2,9 graus */
  const float deg = 0.017453293F;
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];

  /* Nivelado: as leituras cruas do zero viram atitude 0 e altura positiva. */
  meas_input_t in = level(0.5F);
  in.roll = -174.5F * deg;
  in.pitch = -2.9F * deg;
  in.q[0] = 0.0F;  /* quatérnio invertido: o cosseno dele seria -1 */
  in.q[1] = 1.0F;
  assert(meas_update(&s, &in, &p, y) & MEAS_RANGE);
  assert(NEAR(y[LQG_Y_PHI], 0.0, 1e-5) && NEAR(y[LQG_Y_THETA], 0.0, 1e-5));
  assert(NEAR(y[LQG_Y_Z], -0.5, 1e-5));

  /* 5 graus a mais de roll e atravessando +-180: continua sendo 5 graus de erro. */
  in.roll = -169.5F * deg;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_THETA], p.sgn_pitch * 5.0F * deg, 1e-5));
  in.roll = 179.5F * deg;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_THETA], p.sgn_pitch * -6.0F * deg, 1e-5));
}

/**
 * @brief Sinais medidos na bancada (BNO invertido): roll do sensor sobe com a
 *        frente para cima (θ+), pitch do sensor desce com a direita para
 *        cima (φ-), guinada do sensor sobe girando à esquerda (ψ-).
 */
static void test_bench_measured_signs(void) {
  ctrl_params_init();
  ctrl_params_t p;
  ctrl_params_snapshot(&p);
  const float deg = 0.017453293F;
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];
  meas_input_t in = level(0.5F);
  in.roll = (-174.5F + 4.0F) * deg;    /* roll do sensor +4: frente para cima */
  in.pitch = (-2.9F - 3.0F) * deg;     /* pitch do sensor -3: lado direito para cima */
  in.yaw = 0.0F;
  (void)meas_update(&s, &in, &p, y);
  assert(y[LQG_Y_THETA] > 0.0 && NEAR(y[LQG_Y_THETA], 4.0 * deg, 1e-5));
  assert(y[LQG_Y_PHI] < 0.0 && NEAR(y[LQG_Y_PHI], -3.0 * deg, 1e-5));
  in.yaw = 10.0F * deg;                /* girou à esquerda: guinada do sensor sobe */
  (void)meas_update(&s, &in, &p, y);
  assert(y[LQG_Y_PSI] < 0.0 && NEAR(y[LQG_Y_PSI], -10.0 * deg, 1e-5));
}

/** @brief Armar mantém a guinada zerada na energização; só as medidas são esquecidas. */
static void test_rearm_keeps_yaw_origin(void) {
  ctrl_params_t p = defaults();
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];
  meas_input_t in = level(0.5F);
  in.yaw = 1.0F;                       /* energização: este é o zero */
  (void)meas_update(&s, &in, &p, y);
  in.yaw = 1.5F;
  (void)meas_update(&s, &in, &p, y);
  meas_rearm(&s);
  assert(s.yaw_ready && NEAR(s.y[LQG_Y_Z], 0.0, 1e-9));
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_PSI], p.sgn_yaw * 0.5, 1e-6));
}

/** @brief Sinais BNO -> NED e guinada contínua, relativa ao início. */
static void test_attitude_signs_and_yaw_unwrap(void) {
  ctrl_params_t p = defaults();
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];
  meas_input_t in = level(0.5F);
  in.roll = 0.1F;
  in.pitch = 0.2F;
  in.yaw = 3.0F;  /* origem */
  in.gyro[0] = 0.01F;
  in.gyro[1] = 0.02F;
  in.gyro[2] = 0.03F;
  (void)meas_update(&s, &in, &p, y);
  /* O BNO está girado 90 graus: φ vem do pitch dele e θ do roll. */
  assert(NEAR(y[LQG_Y_PHI], p.sgn_roll * 0.2F, 1e-7));
  assert(NEAR(y[LQG_Y_THETA], p.sgn_pitch * 0.1F, 1e-7));
  assert(NEAR(y[LQG_Y_PSI], 0.0, 1e-7));
  assert(NEAR(y[LQG_Y_P], p.sgn_p * 0.02F, 1e-7));  /* giro y */
  assert(NEAR(y[LQG_Y_Q], p.sgn_q * 0.01F, 1e-7));  /* giro x */
  assert(NEAR(y[LQG_Y_R], p.sgn_r * 0.03F, 1e-7));

  /* Passar por +-pi não pode virar um salto de 2*pi. */
  in.yaw = 3.1F;
  (void)meas_update(&s, &in, &p, y);
  in.yaw = -3.1F;  /* = 3.1832 */
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_PSI], p.sgn_yaw * (2.0 * 3.14159265 - 3.1 - 3.0), 1e-4));
  in.yaw = -2.0F;  /* mais uma volta parcial */
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_PSI], p.sgn_yaw * (2.0 * 3.14159265 - 2.0 - 3.0), 1e-4));

  /* Depois do reset, a guinada atual vira o zero. */
  meas_reset(&s);
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_PSI], 0.0, 1e-7));
}

/** @brief Fluxo -> velocidade, com escala, compensação do giro e guinada. */
static void test_flow_velocity(void) {
  ctrl_params_t p = defaults();
  p.flow_scale_x = p.flow_scale_y = 1.0F;
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];

  /* Translação pura: 0,2 rad/s a 1 m = 0,2 m/s. */
  meas_input_t in = level(1.0F);
  in.flow_x = 0.2F;
  in.flow_y = -0.1F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], p.sgn_vx * 0.2, 1e-6));
  assert(NEAR(y[LQG_Y_VY], p.sgn_vy * -0.1, 1e-6));

  /* Rotação pura: o fluxo que o giro explica é removido (X com sinal oposto
     e Y com o mesmo sinal, como medido na bancada). */
  in.flow_x = -0.5F;
  in.gyro[0] = 0.5F;
  in.flow_y = 0.3F;
  in.gyro[1] = 0.3F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.0, 1e-6) && NEAR(y[LQG_Y_VY], 0.0, 1e-6));

  /* Com a guinada a 90 graus, a velocidade do corpo gira para o local. */
  meas_reset(&s);
  in = level(1.0F);
  in.yaw = 0.0F;
  (void)meas_update(&s, &in, &p, y);
  in.yaw = 1.5707963F;
  in.flow_x = 0.2F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.0, 1e-6));
  assert(NEAR(fabs(y[LQG_Y_VY]), 0.2, 1e-6));
}

/** @brief Sensor sem leitura válida repete a última medida e zera o bit. */
static void test_invalid_readings_hold_last_value(void) {
  ctrl_params_t p = defaults();
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];
  meas_input_t in = level(0.4F);
  in.flow_x = 0.1F;
  (void)meas_update(&s, &in, &p, y);
  const lqg_real_t z = y[LQG_Y_Z], vx = y[LQG_Y_VX];

  in.range_valid = false;
  in.range_m = 7.0F;
  in.flow_x = 3.0F;
  const uint8_t flags = meas_update(&s, &in, &p, y);
  /* Sem altura também não há velocidade (o fluxo precisa da distância). */
  assert((flags & (MEAS_RANGE | MEAS_FLOW)) == 0U);
  assert(y[LQG_Y_Z] == z && y[LQG_Y_VX] == vx);

  in = level(NAN);
  assert((meas_update(&s, &in, &p, y) & MEAS_RANGE) == 0U);
  in = level(0.4F);
  in.attitude_valid = false;
  assert((meas_update(&s, &in, &p, y) & (MEAS_ATTITUDE | MEAS_RANGE)) == 0U);
}

int main(void) {
  test_height();
  test_inverted_mount_zero();
  test_bench_measured_signs();
  test_rearm_keeps_yaw_origin();
  test_attitude_signs_and_yaw_unwrap();
  test_flow_velocity();
  test_invalid_readings_hold_last_value();
  printf("test_meas: ok\n");
  return 0;
}
