/**
 * @file test_ekf3.c
 * @brief Quatérnios e EKF3 contra uma trajetória 3D não linear (verdade gerada aqui).
 *
 * A verdade é analítica: posição, inclinação e guinada variam no tempo, o giro e a
 * aceleração do corpo saem dela, e distância e fluxo são calculados com a geometria
 * exata (matriz de rotação, braço do sensor). Um Jacobiano errado em ekf3.c faz o
 * filtro divergir aqui. Não prova nada sobre o drone real.
 */
#include "control/ekf3.h"
#include "control/lqg_config.h"
#include "control/quat.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DT        0.01F
#define GROUND_M  0.05F
#define PI_F      3.14159265F
#define SENSOR_X  0.09F
#define SENSOR_Y  0.02F

static uint32_t lcg_state = 12345U;

/** @brief Ruído uniforme em [-1, 1], determinístico. */
static float noise(void) {
  lcg_state = (lcg_state * 1664525U) + 1013904223U;
  return ((float)(lcg_state >> 8) / 8388608.0F) - 1.0F;
}

static void test_quat_basics(void) {
  /* 90° de guinada leva o eixo x do corpo para o y local (leste). */
  float q[4];
  quat_from_euler(0.0F, 0.0F, PI_F / 2.0F, q);
  const float x_body[3] = {1.0F, 0.0F, 0.0F};
  float v[3];
  quat_rotate(q, x_body, v);
  assert((fabsf(v[0]) < 1e-6F) && (fabsf(v[1] - 1.0F) < 1e-6F) && (fabsf(v[2]) < 1e-6F));
  float back[3];
  quat_rotate_inv(q, v, back);
  assert(fabsf(back[0] - 1.0F) < 1e-6F);

  /* Euler <-> quatérnio e vetor de rotação, ida e volta. */
  for (int i = 0; i < 200; ++i) {
    const float phi = 0.9F * noise();
    const float theta = 1.2F * noise();
    const float psi = 3.0F * noise();
    quat_from_euler(phi, theta, psi, q);
    float p2;
    float t2;
    float s2;
    quat_to_euler(q, &p2, &t2, &s2);
    assert((fabsf(p2 - phi) < 1e-4F) && (fabsf(t2 - theta) < 1e-4F) && (fabsf(s2 - psi) < 1e-4F));
    float rv[3];
    float q2[4];
    quat_to_rotvec(q, rv);
    quat_from_rotvec(rv, q2);
    const float dot = (q[0] * q2[0]) + (q[1] * q2[1]) + (q[2] * q2[2]) + (q[3] * q2[3]);
    assert(fabsf(fabsf(dot) - 1.0F) < 1e-5F);
  }

  /* q e -q são a mesma rotação: o vetor de rotação fica no ramo de menor ângulo. */
  const float qn[4] = {-0.9F, 0.1F, 0.2F, 0.3F};
  float qnn[4];
  (void)memcpy(qnn, qn, sizeof(qn));
  quat_normalize(qnn);
  float rv[3];
  quat_to_rotvec(qnn, rv);
  assert(sqrtf((rv[0] * rv[0]) + (rv[1] * rv[1]) + (rv[2] * rv[2])) < PI_F);

  /* Quatérnio inválido vira identidade. */
  float bad[4] = {NAN, 0.0F, 0.0F, 0.0F};
  quat_normalize(bad);
  assert(bad[0] == 1.0F);
  float zero[4] = {0.0F, 0.0F, 0.0F, 0.0F};
  quat_normalize(zero);
  assert(zero[0] == 1.0F);
  puts("quat ok");
}

/** @brief Verdade no instante t (s). */
typedef struct {
  float q[4];
  float p[3];
  float v[3];
  float a[3];
} truth_t;

static void truth_at(float t, truth_t *s) {
  const float phi = 0.15F * sinf(0.7F * t);
  const float theta = 0.10F * sinf((0.5F * t) + 1.0F) - (0.10F * sinf(1.0F));
  const float psi = 0.25F * t;
  quat_from_euler(phi, theta, psi, s->q);
  s->p[0] = 0.30F * (1.0F - cosf(0.4F * t));
  s->p[1] = 0.20F * (1.0F - cosf(0.3F * t));
  s->p[2] = -(0.30F + (0.05F * (1.0F - cosf(0.5F * t))));
  s->v[0] = 0.30F * 0.4F * sinf(0.4F * t);
  s->v[1] = 0.20F * 0.3F * sinf(0.3F * t);
  s->v[2] = -0.05F * 0.5F * sinf(0.5F * t);
  s->a[0] = 0.30F * 0.4F * 0.4F * cosf(0.4F * t);
  s->a[1] = 0.20F * 0.3F * 0.3F * cosf(0.3F * t);
  s->a[2] = -0.05F * 0.5F * 0.5F * cosf(0.5F * t);
}

typedef struct {
  float gyro_bias_z; /**< somado ao giro, rad/s */
  float flow_spike;  /**< espúrio somado ao fluxo a cada 37 quadros, m/s */
  bool accel_ok;
  float accel_bias_x; /**< somado à aceleração, m/s² */
  bool no_flow;       /**< sem fluxo (só distância e atitude) */
  float att_r_scale;  /**< multiplicador do ruído da atitude; 0 vale 1 */
} cfg_t;

/** @brief Roda @p seconds de voo e devolve o erro final (atitude em rad, velocidade e altura). */
static void run(const cfg_t *cfg, float seconds, ekf3_t *e, float *att_err, float *vel_err,
                float *h_err, float *max_att_err) {
  truth_t k0;
  truth_at(0.0F, &k0);
  ekf3_seed(e, k0.q, k0.p[2], NULL, NULL);
  *max_att_err = 0.0F;
  const uint32_t steps = (uint32_t)(seconds / DT);
  for (uint32_t k = 0U; k < steps; ++k) {
    const float t = (float)k * DT;
    truth_t a;
    truth_t b;
    truth_t mid;
    truth_at(t, &a);
    truth_at(t + DT, &b);
    truth_at(t + (0.5F * DT), &mid);
    ekf3_meas_t m;
    (void)memset(&m, 0, sizeof(m));
    /* Giro do intervalo (k, k+1): vetor de rotação de q_k^-1 q_(k+1) / dt. */
    float qi[4];
    float dq[4];
    float rv[3];
    quat_conj(a.q, qi);
    quat_mul(qi, b.q, dq);
    quat_to_rotvec(dq, rv);
    m.gyro_valid = true;
    for (int i = 0; i < 3; ++i) {
      m.gyro[i] = (rv[i] / DT) + (0.003F * noise());
    }
    m.gyro[2] += cfg->gyro_bias_z;
    /* Aceleração cinemática no corpo (sem gravidade), no ponto médio do intervalo. */
    m.att_r_scale = cfg->att_r_scale;
    m.accel_valid = cfg->accel_ok;
    float qm[4];
    (void)memcpy(qm, mid.q, sizeof(qm));
    float ab[3];
    quat_rotate_inv(qm, mid.a, ab);
    for (int i = 0; i < 3; ++i) {
      m.accel[i] = ab[i] + (0.05F * noise());
    }
    m.accel[0] += cfg->accel_bias_x;
    /* Atitude medida com ruído de ~2 mrad. */
    m.attitude_valid = true;
    const float en[3] = {0.002F * noise(), 0.002F * noise(), 0.003F * noise()};
    float dn[4];
    quat_from_rotvec(en, dn);
    quat_mul(a.q, dn, m.q_meas);
    /* Distância e fluxo a ~33 Hz, geometria exata. */
    float r[3][3];
    quat_to_dcm(a.q, r);
    if ((k % 3U) == 0U) {
      const float lever_z = (SENSOR_X * r[2][0]) + (SENSOR_Y * r[2][1]);
      m.range_valid = true;
      m.range_m = ((GROUND_M - a.p[2] - lever_z) / r[2][2]) + (0.003F * noise());
      m.ground_m = GROUND_M;
      m.range_pos_x = SENSOR_X;
      m.range_pos_y = SENSOR_Y;
      float vb[3];
      quat_rotate_inv(a.q, a.v, vb);
      m.flow_valid = !cfg->no_flow;
      m.flow_fwd = vb[0] + (0.02F * noise());
      m.flow_right = vb[1] + (0.02F * noise());
      m.flow_quality = 100U;
      if ((cfg->flow_spike != 0.0F) && ((k % 37U) == 0U)) {
        m.flow_fwd += cfg->flow_spike;
        m.flow_right -= cfg->flow_spike;
      }
    }
    ekf3_update(e, &m);
    float err[3];
    float qe[4];
    quat_mul(qi, e->q, qe); /* q_k^-1 q̂ */
    quat_to_rotvec(qe, err);
    const float ang = sqrtf((err[0] * err[0]) + (err[1] * err[1]) + (err[2] * err[2]));
    if ((k > 300U) && (ang > *max_att_err)) {
      *max_att_err = ang;
    }
    ekf3_predict(e, &m);
    for (uint32_t i = 0U; i < EKF3_NX; ++i) {
      for (uint32_t j = 0U; j < EKF3_NX; ++j) {
        assert(isfinite(e->P[i][j]) != 0);
        assert(fabsf(e->P[i][j] - e->P[j][i]) <= (1e-6F * (1.0F + fabsf(e->P[i][j]))));
      }
      assert(e->P[i][i] > 0.0F);
    }
  }
  /* Erros contra a verdade no instante final (o filtro acabou de predizer até t + dt). */
  truth_t f;
  truth_at((float)steps * DT, &f);
  float qi[4];
  float qe[4];
  float err[3];
  quat_conj(f.q, qi);
  quat_mul(qi, e->q, qe);
  quat_to_rotvec(qe, err);
  *att_err = sqrtf((err[0] * err[0]) + (err[1] * err[1]) + (err[2] * err[2]));
  *vel_err = sqrtf(((e->v[0] - f.v[0]) * (e->v[0] - f.v[0])) +
                   ((e->v[1] - f.v[1]) * (e->v[1] - f.v[1])) +
                   ((e->v[2] - f.v[2]) * (e->v[2] - f.v[2])));
  *h_err = fabsf(e->p[2] - f.p[2]);
}

static void test_tracks_truth(void) {
  ekf3_t e;
  float att;
  float vel;
  float h;
  float max_att;
  const cfg_t cfg = {.accel_ok = true};
  run(&cfg, 30.0F, &e, &att, &vel, &h, &max_att);
  printf("trajetoria: atitude %.4f rad (max %.4f), velocidade %.4f m/s, altura %.4f m\n", att,
         max_att, vel, h);
  assert((att < 0.01F) && (max_att < 0.02F));
  assert(vel < 0.06F);
  assert(h < 0.02F);
  assert(e.rejected_total < 30U);
  assert(e.variance_skips == 0U);
}

static void test_ba_xy_frozen_without_flow(void) {
  ekf3_t e;
  float att;
  float vel;
  float h;
  float max_att;
  /* Viés de aceleração real de 0,3 m/s² em x. Com fluxo o filtro o aprende; sem fluxo
     o estado fica congelado (não observável) em vez de absorver erro de outra origem. */
  const cfg_t with_flow = {.accel_ok = true, .accel_bias_x = 0.3F};
  run(&with_flow, 40.0F, &e, &att, &vel, &h, &max_att);
  printf("viés de aceleração x com fluxo: %.3f (real 0.300)\n", e.ba[0]);
  assert(fabsf(e.ba[0] - 0.3F) < 0.1F);
  const cfg_t no_flow = {.accel_ok = true, .accel_bias_x = 0.3F, .no_flow = true};
  run(&no_flow, 40.0F, &e, &att, &vel, &h, &max_att);
  printf("viés de aceleração x sem fluxo: %.4f (congelado)\n", e.ba[0]);
  assert(fabsf(e.ba[0]) < 1e-3F);
  assert(e.ba_xy_off && (e.P[EKF3_DBA][EKF3_DBA] < 1e-9F));
}

static void test_attitude_r_scale(void) {
  /* Atitude inicial errada em 0,2 rad: com o ruído normal o filtro converge; com a
     medida muito desconfiada (precisão baixa do BNO) segue o giro e demora. */
  float qt[4];
  float qw[4];
  quat_from_euler(0.0F, 0.0F, 0.0F, qt);
  quat_from_euler(0.2F, 0.0F, 0.0F, qw);
  float err[2];
  const float scale[2] = {1.0F, 1.0e5F};
  for (int k = 0; k < 2; ++k) {
    ekf3_t e;
    ekf3_seed(&e, qw, -0.3F, NULL, NULL);
    ekf3_meas_t m;
    (void)memset(&m, 0, sizeof(m));
    m.attitude_valid = true;
    (void)memcpy(m.q_meas, qt, sizeof(qt));
    m.att_r_scale = scale[k];
    for (int i = 0; i < 100; ++i) {
      ekf3_update(&e, &m);
      ekf3_predict(&e, &m);
    }
    float qi[4];
    float d[4];
    float rv[3];
    quat_conj(qt, qi);
    quat_mul(qi, e.q, d);
    quat_to_rotvec(d, rv);
    err[k] = sqrtf((rv[0] * rv[0]) + (rv[1] * rv[1]) + (rv[2] * rv[2]));
  }
  printf("ruído da atitude x1: erro %.4f rad; x1e5: %.4f rad\n", err[0], err[1]);
  assert((err[0] < 0.01F) && (err[1] > 0.15F));
}

static void test_gyro_bias_observable(void) {
  ekf3_t e;
  float att;
  float vel;
  float h;
  float max_att;
  const cfg_t cfg = {.gyro_bias_z = 0.03F, .accel_ok = true};
  run(&cfg, 30.0F, &e, &att, &vel, &h, &max_att);
  printf("viés de giro z: estimado %.4f (real 0.0300), atitude %.4f rad\n", e.bg[2], att);
  assert(fabsf(e.bg[2] - 0.03F) < 0.01F);
  assert((att < 0.015F) && (vel < 0.08F));
}

static void test_flow_spikes_rejected(void) {
  ekf3_t e;
  float att;
  float vel;
  float h;
  float max_att;
  const cfg_t cfg = {.flow_spike = 4.5F, .accel_ok = true};
  run(&cfg, 30.0F, &e, &att, &vel, &h, &max_att);
  printf("espúrios de fluxo: velocidade %.4f m/s, rejeitadas %u\n", vel,
         (unsigned)e.rejected_total);
  assert(vel < 0.15F);
  assert(e.rejected_total > 10U);
}

static void test_no_accel(void) {
  /* Sem aceleração o filtro segue estável: a velocidade vem do fluxo. */
  ekf3_t e;
  float att;
  float vel;
  float h;
  float max_att;
  const cfg_t cfg = {.accel_ok = false};
  run(&cfg, 30.0F, &e, &att, &vel, &h, &max_att);
  printf("sem aceleração: atitude %.4f rad, velocidade %.4f m/s, altura %.4f m\n", att, vel, h);
  assert((att < 0.02F) && (vel < 0.2F) && (h < 0.05F));
}

/**
 * @brief Aceleração vertical falsa (−20 m/s² por 0,45 s, como no log 20261114_184336) com a distância
 *        válida e o drone parado a 8 cm: a altura tem de seguir a distância, sem virar metros.
 */
static void test_false_vertical_acceleration(void) {
  ekf3_t e;
  float q[4];
  quat_from_euler(0.0F, 0.0F, 0.0F, q);
  const float hover_m = 0.08F;
  ekf3_seed(&e, q, -hover_m, NULL, NULL);
  float worst_h = 0.0F;
  float worst_w = 0.0F;
  for (uint32_t k = 0U; k < 400U; ++k) {
    ekf3_meas_t m;
    (void)memset(&m, 0, sizeof(m));
    m.gyro_valid = true;
    m.attitude_valid = true;
    (void)memcpy(m.q_meas, q, sizeof(q));
    m.att_r_scale = 1.0F;
    m.accel_valid = true;
    for (int i = 0; i < 3; ++i) {
      m.accel[i] = 0.05F * noise();
    }
    if ((k >= 100U) && (k < 145U)) {
      m.accel[2] = -20.0F;
    }
    if ((k % 3U) == 0U) {
      m.range_valid = true;
      m.range_m = GROUND_M + hover_m + (0.003F * noise());
      m.ground_m = GROUND_M;
    }
    ekf3_update(&e, &m);
    ekf3_predict(&e, &m);
    if (k >= 100U) {
      worst_h = fmaxf(worst_h, fabsf(-e.p[2] - hover_m));
      worst_w = fmaxf(worst_w, fabsf(e.v[2]));
    }
  }
  printf("aceleração vertical falsa: erro de altura máx %.3f m, |w| máx %.2f m/s, rejeitadas %u\n",
         worst_h, worst_w, (unsigned)e.rejected_total);
  assert(worst_h < 0.10F);
  assert(worst_w < 1.5F);
  assert(fabsf(-e.p[2] - hover_m) < 0.03F);
}

static void test_reset_xy_and_estimate(void) {
  ekf3_t e;
  float q[4];
  quat_from_euler(0.1F, -0.05F, 1.2F, q);
  ekf3_seed(&e, q, -0.3F, NULL, NULL);
  e.p[0] = 5.0F;
  e.p[1] = -2.0F;
  e.v[0] = 0.4F;
  ekf3_reset_xy(&e);
  float x[EKF3_NEST];
  ekf3_estimate(&e, x);
  assert((x[EKF3_PX] == 0.0F) && (x[EKF3_PY] == 0.0F) && (fabsf(x[EKF3_PZ] + 0.3F) < 1e-6F));
  assert((fabsf(x[EKF3_PHI] - 0.1F) < 1e-5F) && (fabsf(x[EKF3_THETA] + 0.05F) < 1e-5F) &&
         (fabsf(x[EKF3_PSI] - 1.2F) < 1e-5F));
  assert(x[EKF3_U] == 0.4F);
  float tilt[2];
  float yaw;
  ekf3_attitude(&e, tilt, &yaw);
  assert((fabsf(yaw - 1.2F) < 1e-5F) && (fabsf(tilt[0] - 0.1F) < 0.01F) &&
         (fabsf(tilt[1] + 0.05F) < 0.01F));
  puts("estimate ok");
}

int main(void) {
  test_quat_basics();
  test_reset_xy_and_estimate();
  test_tracks_truth();
  test_ba_xy_frozen_without_flow();
  test_attitude_r_scale();
  test_gyro_bias_observable();
  test_flow_spikes_rejected();
  test_no_accel();
  test_false_vertical_acceleration();
  puts("test_ekf3 ok");
  return 0;
}
