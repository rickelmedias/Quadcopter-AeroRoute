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
  in.pred_vx = in.pred_vy = NAN;
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
  ctrl_params_snapshot(&p); /* zero padrão: roll -174,5 e pitch -2,9 graus */
  const float deg = 0.017453293F;
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];

  /* Nivelado: as leituras cruas do zero viram atitude 0 e altura positiva. */
  meas_input_t in = level(0.5F);
  in.roll = -174.5F * deg;
  in.pitch = -2.9F * deg;
  in.q[0] = 0.0F; /* quatérnio invertido: o cosseno dele seria -1 */
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
  in.roll = (-174.5F + 4.0F) * deg; /* roll do sensor +4: frente para cima */
  in.pitch = (-2.9F - 3.0F) * deg;  /* pitch do sensor -3: lado direito para cima */
  in.yaw = 0.0F;
  (void)meas_update(&s, &in, &p, y);
  assert(y[LQG_Y_THETA] > 0.0 && NEAR(y[LQG_Y_THETA], 4.0 * deg, 1e-5));
  assert(y[LQG_Y_PHI] < 0.0 && NEAR(y[LQG_Y_PHI], -3.0 * deg, 1e-5));
  in.yaw = 10.0F * deg; /* girou à esquerda: guinada do sensor sobe */
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
  in.yaw = 1.0F; /* energização: este é o zero */
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
  in.yaw = 3.0F; /* origem */
  in.gyro[0] = 0.01F;
  in.gyro[1] = 0.02F;
  in.gyro[2] = 0.03F;
  (void)meas_update(&s, &in, &p, y);
  /* O BNO está girado 90 graus: φ vem do pitch dele e θ do roll. */
  assert(NEAR(y[LQG_Y_PHI], p.sgn_roll * 0.2F, 1e-7));
  assert(NEAR(y[LQG_Y_THETA], p.sgn_pitch * 0.1F, 1e-7));
  assert(NEAR(y[LQG_Y_PSI], 0.0, 1e-7));
  assert(NEAR(y[LQG_Y_P], p.sgn_p * 0.02F, 1e-7)); /* giro y */
  assert(NEAR(y[LQG_Y_Q], p.sgn_q * 0.01F, 1e-7)); /* giro x */
  assert(NEAR(y[LQG_Y_R], p.sgn_r * 0.03F, 1e-7));

  /* Passar por +-pi não pode virar um salto de 2*pi. */
  in.yaw = 3.1F;
  (void)meas_update(&s, &in, &p, y);
  in.yaw = -3.1F; /* = 3.1832 */
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_PSI], p.sgn_yaw * (2.0 * 3.14159265 - 3.1 - 3.0), 1e-4));
  in.yaw = -2.0F; /* mais uma volta parcial */
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_PSI], p.sgn_yaw * (2.0 * 3.14159265 - 2.0 - 3.0), 1e-4));

  /* Depois do reset, a guinada atual vira o zero. */
  meas_reset(&s);
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_PSI], 0.0, 1e-7));
}

/** @brief FLOW_GATE: qualidade com histerese, inclinação e fluxo absurdo. */
static void test_flow_gate(void) {
  ctrl_params_t p = defaults();
  p.flow_scale_x = p.flow_scale_y = 1.0F;
  p.flow_gate = 1.0F;
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];
  meas_input_t in = level(1.0F);
  in.flow_x = 0.2F;
  in.flow_quality = 20U; /* abaixo do limiar de ligar (30): ainda recusado */
  assert((meas_update(&s, &in, &p, y) & MEAS_FLOW) == 0U);
  in.flow_quality = 40U;
  assert((meas_update(&s, &in, &p, y) & MEAS_FLOW) != 0U);
  in.flow_quality = 20U; /* histerese: acima de 15 continua ligado */
  assert((meas_update(&s, &in, &p, y) & MEAS_FLOW) != 0U);
  in.flow_quality = 10U; /* abaixo de 15 desliga */
  assert((meas_update(&s, &in, &p, y) & MEAS_FLOW) == 0U);
  in.flow_quality = 50U;
  in.flow_x = 5.0F; /* acima de 4,2 rad/s: corrompido */
  assert((meas_update(&s, &in, &p, y) & MEAS_FLOW) == 0U);
  in.flow_x = 0.2F;
  in.roll = 0.7F; /* inclinado ~40°: a distância não é a altura */
  assert((meas_update(&s, &in, &p, y) & MEAS_FLOW) == 0U);
  p.flow_gate = 0.0F; /* desligado: aceita tudo, como antes */
  assert((meas_update(&s, &in, &p, y) & MEAS_FLOW) != 0U);
}

/**
 * @brief Pareamento cruzado: rotação pura (sem translação) não gera velocidade.
 *
 * Como no voo de 08/11: flow_x responde ao giro y do BNO (-1,45), flow_y ao giro
 * x (+1,65). Com o pareamento direto antigo sobra velocidade falsa; com o
 * cruzado, ela some, nas duas orientações do sensor.
 */
static void test_flow_gyro_cross(void) {
  ctrl_params_t p = defaults();
  p.flow_scale_x = p.flow_scale_y = 1.0F;
  p.flow_yaw_x = p.flow_yaw_y = 0.0F;
  p.sgn_vx = p.sgn_vy = p.sgn_yaw = 1.0F;
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];
  const float gx = 0.3F, gy = -0.4F, range = 0.12F;
  meas_input_t in = level(range);
  in.gyro[0] = gx;
  in.gyro[1] = gy;
  in.flow_x = -1.45F * gy;
  in.flow_y = +1.65F * gx;
  /* Pareamento direto de antes: sobra velocidade falsa. */
  p.flow_gyro_x = 1.0F;
  p.flow_gyro_y = -1.0F;
  p.flow_gyro_xy = p.flow_gyro_yx = 0.0F;
  p.flow_swap = 1.0F;
  (void)meas_update(&s, &in, &p, y);
  assert(fabs((double)y[LQG_Y_VX]) > 0.05 || fabs((double)y[LQG_Y_VY]) > 0.05);
  /* Cruzado: rotação pura vira velocidade nula, com ou sem FLOW_SWAP. */
  p.flow_gyro_x = p.flow_gyro_y = 0.0F;
  p.flow_gyro_xy = 1.45F;
  p.flow_gyro_yx = -1.65F;
  for (int swap = 0; swap < 2; ++swap) {
    p.flow_swap = (float)swap;
    (void)meas_update(&s, &in, &p, y);
    assert(NEAR(y[LQG_Y_VX], 0.0, 1e-5) && NEAR(y[LQG_Y_VY], 0.0, 1e-5));
  }
  /* Translação continua passando: 0,1 rad/s a mais no canal Y (frente com swap). */
  p.flow_swap = 1.0F;
  in.flow_y += 0.1F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.1 * range, 1e-5) && NEAR(y[LQG_Y_VY], 0.0, 1e-5));
}

/**
 * @brief EKF de navegação: com FLOW_EKF o vx do LQG sai bem menos ruidoso que o
 *        fluxo cru; desligado, nada muda. Só quadro novo do MTF corrige.
 */
static void test_flow_ekf(void) {
  ctrl_params_t p = defaults();
  p.flow_scale_x = p.flow_scale_y = 1.0F;
  p.flow_yaw_x = p.flow_yaw_y = 0.0F;
  p.flow_gyro_x = p.flow_gyro_y = p.flow_gyro_xy = p.flow_gyro_yx = 0.0F;
  p.sgn_vx = p.sgn_vy = p.sgn_yaw = 1.0F;
  p.flow_swap = 0.0F;
  lqg_real_t y[LQG_NY];
  double var[2] = {0.0, 0.0};
  for (int fuse = 0; fuse < 2; ++fuse) {
    p.flow_ekf = (float)fuse;
    meas_state_t s;
    meas_reset(&s);
    unsigned long seed = 7UL;
    double acc = 0.0;
    int n = 0;
    for (int k = 0; k < 1500; ++k) {
      meas_input_t in = level(0.15F);
      in.accel_valid = true;
      in.mtf_frame = (uint32_t)k + 1U;
      in.mtf_time_ms = 10U * (uint32_t)k;
      seed = seed * 1103515245UL + 12345UL;
      const float jitter = ((float)((seed >> 8) & 0xFFFFU) / 65535.0F - 0.5F) * 2.0F;
      in.flow_x = 0.6F + jitter; /* 0,6 rad/s a 15 cm = 9 cm/s, com ±1 rad/s de ruído */
      (void)meas_update(&s, &in, &p, y);
      if (k > 500) {
        acc += ((double)y[LQG_Y_VX] - 0.09) * ((double)y[LQG_Y_VX] - 0.09);
        ++n;
      }
    }
    var[fuse] = acc / n;
    assert(fuse == 0 || (s.diag.flags & MEAS_DIAG_XY_TO_LQG) != 0U);
  }
  assert(var[1] < var[0] / 9.0); /* desvio pelo menos 3 vezes menor */

  /* O mesmo quadro repetido não corrige o EKF de novo. */
  meas_state_t s;
  meas_reset(&s);
  meas_input_t in = level(0.15F);
  in.accel_valid = true;
  in.mtf_frame = 5U;
  (void)meas_update(&s, &in, &p, y);
  const uint32_t updates = s.nav.flow_updates + (s.nav.h_started ? 1U : 0U);
  (void)meas_update(&s, &in, &p, y);
  assert(s.nav.flow_updates + (s.nav.h_started ? 1U : 0U) == updates);
  assert((s.diag.flags & MEAS_DIAG_NEW_FRAME) == 0U);

  /* Aceleração para a frente (eixo y do BNO com o sinal de p) faz a
     velocidade x do EKF crescer sem fluxo. */
  in.flow_valid = false;
  in.lin_accel[1] = p.sgn_p * 1.0F;
  for (int k = 0; k < 20; ++k)
    (void)meas_update(&s, &in, &p, y);
  assert(s.nav.h[0].x[1] > 0.15F && fabsf(s.nav.h[1].x[1]) < 1e-4F);
}

/** @brief Z_EKF: com a distância filtrada, um salto isolado do telêmetro não chega ao LQG. */
static void test_z_ekf(void) {
  ctrl_params_t p = defaults();
  lqg_real_t y[LQG_NY];
  for (int use = 0; use < 2; ++use) {
    p.z_ekf = (float)use;
    meas_state_t s;
    meas_reset(&s);
    for (int k = 0; k < 200; ++k) {
      meas_input_t in = level(0.15F);
      in.accel_valid = true;
      in.mtf_frame = (uint32_t)k + 1U;
      in.mtf_time_ms = 10U * (uint32_t)k;
      if (k == 150)
        in.range_m = 0.60F; /* reflexo isolado */
      (void)meas_update(&s, &in, &p, y);
      if (k == 150) {
        if (use) {
          assert(NEAR(y[LQG_Y_Z], -0.15, 0.01));
          assert((s.diag.flags & MEAS_DIAG_RANGE_REJECTED) != 0U);
        } else {
          assert(NEAR(y[LQG_Y_Z], -0.60, 1e-6));
        }
      }
    }
    assert(use == 0 || (s.diag.flags & MEAS_DIAG_Z_TO_LQG) != 0U);
  }
}

/** @brief FLOW_BIAS e FLOW_CENS: viés subtraído e zero tratado como faixa. */
static void test_flow_bias_and_censoring(void) {
  ctrl_params_t p = defaults();
  p.flow_scale_x = p.flow_scale_y = 1.0F;
  p.flow_yaw_x = p.flow_yaw_y = 0.0F;
  p.flow_gyro_x = p.flow_gyro_y = 0.0F;
  p.sgn_vx = p.sgn_vy = p.sgn_yaw = 1.0F;
  p.flow_swap = 0.0F;
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];
  meas_input_t in = level(1.0F);
  in.flow_x = 0.4F;
  p.flow_bias_x = 0.1F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.3, 1e-6)); /* viés de 0,1 rad/s a 1 m = 0,1 m/s tirado */
  p.flow_bias_x = 0.0F;

  /* Censura: contagem zero. A predição dentro de ±0,1 m/s (a 1 m) é aceita. */
  p.flow_cens = 1.0F;
  in.flow_x = 0.0F;
  in.flow_y = 0.0F;
  in.pred_vx = 0.06F;
  in.pred_vy = -0.04F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.06, 1e-5) && NEAR(y[LQG_Y_VY], -0.04, 1e-5));
  /* Fora da faixa, é limitada à borda (meio degrau = 0,1 m/s a 1 m). */
  in.pred_vx = 0.5F;
  in.pred_vy = -0.5F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.1, 1e-5) && NEAR(y[LQG_Y_VY], -0.1, 1e-5));
  /* Leitura não zero: a medida vale, a predição é ignorada. */
  in.flow_x = 0.4F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.4, 1e-6));
  /* Sem predição (NaN) ou com FLOW_CENS desligado: zero é zero, como antes. */
  in.flow_x = 0.0F;
  in.pred_vx = NAN;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.0, 1e-6));
  in.pred_vx = 0.05F;
  p.flow_cens = 0.0F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.0, 1e-6));
}

/** @brief Fluxo -> velocidade, com escala, compensação do giro e guinada. */
static void test_flow_velocity(void) {
  ctrl_params_t p = defaults();
  p.flow_scale_x = p.flow_scale_y = 1.0F;
  p.flow_yaw_x = p.flow_yaw_y = 0.0F;
  /* Mapeamento original e pareamento direto (os padrões mudaram em 08/11). */
  p.flow_swap = 0.0F;
  p.flow_gyro_x = 1.0F;
  p.flow_gyro_y = -1.0F;
  p.flow_gyro_xy = p.flow_gyro_yx = 0.0F;
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

  /* FLOW_SWAP: o canal Y vira a frente e o X a esquerda (direita = -X). */
  p.flow_swap = 1.0F;
  in.flow_x = 0.2F;
  in.flow_y = -0.1F;
  in.gyro[0] = in.gyro[1] = 0.0F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], p.sgn_vx * -0.1, 1e-6));
  assert(NEAR(y[LQG_Y_VY], p.sgn_vy * -0.2, 1e-6));
  p.flow_swap = 0.0F;

  /* Rotação pura: o fluxo que o giro explica é removido (X com sinal oposto
     e Y com o mesmo sinal, como medido na bancada). */
  in.flow_x = -0.5F;
  in.gyro[0] = 0.5F;
  in.flow_y = 0.3F;
  in.gyro[1] = 0.3F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.0, 1e-6) && NEAR(y[LQG_Y_VY], 0.0, 1e-6));

  /* Guinada pura com o MTF fora do centro: a velocidade r × distância sai. */
  p.flow_yaw_y = 0.09F;
  in.flow_x = 0.0F;
  in.flow_y = 0.0F;
  in.gyro[0] = in.gyro[1] = 0.0F;
  in.gyro[2] = 1.0F;
  const float r = p.sgn_r * 1.0F;
  in.flow_y = 0.09F * r / (p.sgn_vy * 1.0F); /* o que o sensor vê girando, a 1 m */
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.0, 1e-5) && NEAR(y[LQG_Y_VY], 0.0, 1e-5));
  in.gyro[2] = 0.0F;
  in.flow_y = 0.0F;
  p.flow_yaw_y = 0.0F;

  /* Com a guinada a 90 graus, a velocidade do corpo gira para o local. O
     sensor girou 90° à esquerda: em NED o nariz aponta para oeste (ψ = −90°),
     então andar para a frente é y negativo. */
  meas_reset(&s);
  in = level(1.0F);
  in.yaw = 0.0F;
  (void)meas_update(&s, &in, &p, y);
  in.yaw = 1.5707963F;
  in.flow_x = 0.2F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.0, 1e-6));
  assert(NEAR(y[LQG_Y_PSI], -1.5707963, 1e-5));
  assert(NEAR(y[LQG_Y_VY], -0.2, 1e-6));
  /* Andar para a direita do corpo, com o nariz a oeste, é ir para o norte. */
  in.flow_x = 0.0F;
  in.flow_y = 0.2F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_VX], 0.2, 1e-6) && NEAR(y[LQG_Y_VY], 0.0, 1e-6));
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

/**
 * @brief SNS_LVL_PHI/TH somam a correção fina do nível a φ e θ, sem mexer na
 *        altura (o cosseno usa a inclinação do sensor).
 */
static void test_level_trim(void) {
  ctrl_params_t p = defaults();
  meas_state_t s;
  meas_reset(&s);
  lqg_real_t y[LQG_NY];
  meas_input_t in = level(0.4F);
  (void)meas_update(&s, &in, &p, y);
  const double phi0 = y[LQG_Y_PHI], theta0 = y[LQG_Y_THETA], z0 = y[LQG_Y_Z];
  p.lvl_phi_deg = 0.5F;
  p.lvl_theta_deg = -1.0F;
  (void)meas_update(&s, &in, &p, y);
  assert(NEAR(y[LQG_Y_PHI] - phi0, 0.5 * 0.017453293, 1e-6));
  assert(NEAR(y[LQG_Y_THETA] - theta0, -1.0 * 0.017453293, 1e-6));
  assert(NEAR(y[LQG_Y_Z], z0, 1e-9));
}

int main(void) {
  test_height();
  test_inverted_mount_zero();
  test_bench_measured_signs();
  test_rearm_keeps_yaw_origin();
  test_attitude_signs_and_yaw_unwrap();
  test_flow_velocity();
  test_flow_gate();
  test_flow_bias_and_censoring();
  test_flow_gyro_cross();
  test_flow_ekf();
  test_z_ekf();
  test_invalid_readings_hold_last_value();
  test_level_trim();
  printf("test_meas: ok\n");
  return 0;
}
