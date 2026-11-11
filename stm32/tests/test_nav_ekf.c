/**
 * @file test_nav_ekf.c
 * @brief EKF de navegação (control/nav_ekf.h) e filtro da distância
 *        (control/range_filter.h).
 */
#include "control/nav_ekf.h"
#include "control/range_filter.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

/** Passo do controle, em s. */
#define DT 0.01F

/** Ruídos padrão do firmware (flight_config.h). */
static const nav_ekf_config_t CFG = {1.0F, 1.0F, 0.02F, 0.8F, 0.006F, 3.0F};

/** @brief Gerador determinístico (LCG): os testes não dependem de sorte. */
static unsigned long rng = 12345UL;

/** @brief Ruído aproximadamente gaussiano de média 0 e desvio @p sigma. */
static float noise(float sigma) {
  float acc = 0.0F;
  for (int i = 0; i < 12; ++i) {
    rng = rng * 1103515245UL + 12345UL;
    acc += (float)((rng >> 8) & 0xFFFFU) / 65535.0F;
  }
  return (acc - 6.0F) * sigma;
}

/** @brief Covariâncias finitas, simétricas e com diagonal não negativa. */
static void assert_sane(const nav_ekf_axis_t *a) {
  for (int i = 0; i < 3; ++i) {
    assert(isfinite(a->x[i]) && a->P[i][i] >= 0.0F);
    for (int j = 0; j < 3; ++j)
      assert(fabsf(a->P[i][j] - a->P[j][i]) < 1e-6F);
  }
}

/** @brief Parado com acelerômetro enviesado: o viés é aprendido e a posição não deriva. */
static void test_bias_is_learned_and_position_holds(void) {
  nav_ekf_t f;
  nav_ekf_reset(&f);
  const float range = 0.15F, bias = 0.2F;
  for (int k = 0; k < 6000; ++k) { /* 60 s */
    nav_ekf_predict(&f, bias + noise(0.3F), -bias + noise(0.3F), 0.0F, DT, &CFG);
    (void)nav_ekf_update_flow(&f, noise(0.1F), noise(0.1F), range, 0.0F, 0.0F, &CFG);
  }
  assert_sane(&f.h[0]);
  assert(fabsf(f.h[0].x[2] - bias) < 0.05F && fabsf(f.h[1].x[2] + bias) < 0.05F);
  assert(fabsf(f.h[0].x[1]) < 0.02F);
  /* Sem o viés no estado, 0,2 m/s² por 60 s seriam metros de deriva. */
  assert(fabsf(f.h[0].x[0]) < 0.3F && fabsf(f.h[1].x[0]) < 0.3F);
  assert(nav_ekf_xy_usable(&f));
}

/** @brief Velocidade constante: segue a velocidade e integra a posição. */
static void test_tracks_constant_velocity(void) {
  nav_ekf_t f;
  nav_ekf_reset(&f);
  const float v = 0.3F;
  for (int k = 0; k < 2000; ++k) { /* 20 s */
    nav_ekf_predict(&f, noise(0.3F), 0.0F, 0.0F, DT, &CFG);
    (void)nav_ekf_update_flow(&f, v + noise(0.1F), noise(0.1F), 0.15F, 0.0F, 0.0F, &CFG);
  }
  assert(fabsf(f.h[0].x[1] - v) < 0.03F);
  assert(fabsf(f.h[0].x[0] - v * 20.0F) < 0.3F);
}

/** @brief A predição usa a IMU: 2 m/s² por 1 s, sem fluxo, dá 2 m/s e 1 m. */
static void test_prediction_uses_imu(void) {
  nav_ekf_t f;
  nav_ekf_reset(&f);
  (void)nav_ekf_update_flow(&f, 0.0F, 0.0F, 0.15F, 0.0F, 0.0F, &CFG);
  for (int k = 0; k < 100; ++k)
    nav_ekf_predict(&f, 2.0F, 0.0F, 0.0F, DT, &CFG);
  assert(fabsf(f.h[0].x[1] - 2.0F) < 1e-3F && fabsf(f.h[0].x[0] - 1.0F) < 0.02F);
  assert(!nav_ekf_xy_usable(&f)); /* sem correção há 100 passos */
}

/** @brief O gate recusa o par inteiro, mesmo com só um eixo fora; persistência reancora. */
static void test_gate_on_the_pair_and_recovery(void) {
  nav_ekf_t f;
  nav_ekf_reset(&f);
  for (int k = 0; k < 500; ++k) {
    nav_ekf_predict(&f, 0.0F, 0.0F, 0.0F, DT, &CFG);
    (void)nav_ekf_update_flow(&f, noise(0.05F), noise(0.05F), 0.15F, 0.0F, 0.0F, &CFG);
  }
  nav_ekf_predict(&f, 0.0F, 0.0F, 0.0F, DT, &CFG);
  const float vy_before = f.h[1].x[1];
  assert(!nav_ekf_update_flow(&f, 2.0F, 0.0F, 0.15F, 0.0F, 0.0F, &CFG));
  assert(f.flow_rejected == 1U && f.flow_ratio > 1.0F);
  assert(fabsf(f.h[1].x[1] - vy_before) < 1e-6F); /* o y também não foi fundido */
  for (unsigned k = 0U; k < NAV_EKF_MAX_REJECT + 2U; ++k) {
    nav_ekf_predict(&f, 0.0F, 0.0F, 0.0F, DT, &CFG);
    (void)nav_ekf_update_flow(&f, 1.0F, 0.0F, 0.15F, 0.0F, 0.0F, &CFG);
  }
  assert(f.flow_resets >= 1U && fabsf(f.h[0].x[1] - 1.0F) < 0.05F);
}

/** @brief Uma leitura zero do MTF (célula de quantização) pesa menos que uma leitura cheia. */
static void test_quantization_widens_the_measurement(void) {
  nav_ekf_t a, b;
  nav_ekf_reset(&a);
  nav_ekf_reset(&b);
  (void)nav_ekf_update_flow(&a, 0.0F, 0.0F, 0.15F, 0.0F, 0.0F, &CFG);
  (void)nav_ekf_update_flow(&b, 0.0F, 0.0F, 0.15F, 0.0F, 0.0F, &CFG);
  for (int k = 0; k < 50; ++k) {
    nav_ekf_predict(&a, 0.0F, 0.0F, 0.0F, DT, &CFG);
    nav_ekf_predict(&b, 0.0F, 0.0F, 0.0F, DT, &CFG);
  }
  (void)nav_ekf_update_flow(&a, 0.05F, 0.0F, 0.15F, 0.0F, 0.0F, &CFG);
  (void)nav_ekf_update_flow(&b, 0.05F, 0.0F, 0.15F, 0.2F, 0.0F, &CFG);
  assert(fabsf(b.h[0].x[1]) < fabsf(a.h[0].x[1]));
}

/** @brief z: segue uma subida com a distância e a aceleração, e recusa um salto isolado. */
static void test_vertical_channel(void) {
  nav_ekf_t f;
  nav_ekf_reset(&f);
  float z = -0.08F, vz = 0.0F;
  for (int k = 0; k < 1000; ++k) {
    const float a = (k < 200) ? -0.5F : (k < 400 ? 0.5F : 0.0F); /* sobe e para */
    vz += a * DT;
    z += vz * DT;
    nav_ekf_predict(&f, 0.0F, 0.0F, a + noise(0.3F), DT, &CFG);
    (void)nav_ekf_update_z(&f, z + noise(0.004F), &CFG);
  }
  assert(fabsf(f.z.x[0] - z) < 0.01F && fabsf(f.z.x[1] - vz) < 0.03F);
  const float z_before = f.z.x[0];
  nav_ekf_predict(&f, 0.0F, 0.0F, 0.0F, DT, &CFG);
  assert(!nav_ekf_update_z(&f, z - 0.3F, &CFG)); /* salto de 30 cm num passo */
  assert(f.z_rejected == 1U && fabsf(f.z.x[0] - z_before) < 1e-3F);
  assert(nav_ekf_z_usable(&f));
}

/** @brief Ao armar, posição e velocidade recomeçam e os vieses ficam. */
static void test_rearm_keeps_biases(void) {
  nav_ekf_t f;
  nav_ekf_reset(&f);
  f.h[0].x[0] = 1.0F;
  f.h[0].x[2] = 0.1F;
  f.z.x[2] = -0.2F;
  f.h_started = f.z_started = true;
  nav_ekf_rearm(&f);
  assert(f.h[0].x[0] == 0.0F && f.h[0].x[2] == 0.1F && f.z.x[2] == -0.2F);
  assert(!f.h_started && !f.z_started);
}

/** @brief Zerar a posição mantém velocidade e viés. */
static void test_zero_position(void) {
  nav_ekf_t f;
  nav_ekf_reset(&f);
  f.h[0].x[0] = 0.4F;
  f.h[0].x[1] = 0.1F;
  f.h[0].x[2] = 0.02F;
  f.h[0].P[0][0] = 0.5F;
  f.h[0].P[0][1] = f.h[0].P[1][0] = 0.1F;
  nav_ekf_zero_position(&f);
  assert(f.h[0].x[0] == 0.0F && f.h[0].x[1] == 0.1F && f.h[0].x[2] == 0.02F);
  assert(f.h[0].P[0][0] == 0.0F && f.h[0].P[0][1] == 0.0F && f.h[0].P[1][0] == 0.0F);
}

/** @brief ZUPT: no chão o filtro inicia e fica com velocidade zero; no ar segue pela inércia. */
static void test_zupt_then_inertial(void) {
  nav_ekf_t f;
  nav_ekf_reset(&f);
  assert(!nav_ekf_xy_inertial(&f));
  for (int k = 0; k < 200; ++k) {
    nav_ekf_predict(&f, 0.05F + noise(0.2F), noise(0.2F), 0.0F, DT, &CFG);
    nav_ekf_zupt(&f);
  }
  assert(nav_ekf_xy_inertial(&f) && nav_ekf_xy_usable(&f) && fabsf(f.h[0].x[1]) < 0.02F);
  for (int k = 0; k < 50; ++k)
    nav_ekf_predict(&f, 1.0F, 0.0F, 0.0F, DT, &CFG); /* 0,5 s a 1 m/s² */
  assert(nav_ekf_xy_inertial(&f) && fabsf(f.h[0].x[1] - 0.5F) < 0.1F);
  for (unsigned k = 0U; k <= NAV_EKF_INERTIAL_STEPS; ++k) {
    nav_ekf_predict(&f, 0.0F, 0.0F, 0.0F, DT, &CFG);
  }
  assert(!nav_ekf_xy_inertial(&f)); /* sem correção há mais de 1 s */
}

/** @brief Distância: salto isolado recusado, patamar novo reancorado, mínimo respeitado. */
static void test_range_filter(void) {
  range_filter_t r;
  range_filter_reset(&r);
  float out = 0.0F;
  assert(range_filter_update(&r, 0.12F, DT, true, 0.02F, &out) && out == 0.12F);
  assert(!range_filter_update(&r, 0.60F, DT, true, 0.02F, &out)); /* salto isolado */
  assert(range_filter_update(&r, 0.121F, DT, true, 0.02F, &out));
  for (int k = 0; k < 4; ++k)
    assert(!range_filter_update(&r, 0.40F, DT, true, 0.02F, &out));
  assert(range_filter_update(&r, 0.40F, DT, true, 0.02F, &out) && fabsf(out - 0.40F) < 1e-6F);
  assert(r.reanchors == 1U);
  assert(!range_filter_update(&r, 0.01F, DT, true, 0.02F, &out));  /* abaixo do mínimo */
  assert(!range_filter_update(&r, 0.40F, DT, false, 0.02F, &out)); /* inválida */
}

int main(void) {
  test_bias_is_learned_and_position_holds();
  test_tracks_constant_velocity();
  test_prediction_uses_imu();
  test_gate_on_the_pair_and_recovery();
  test_quantization_widens_the_measurement();
  test_vertical_channel();
  test_rearm_keeps_biases();
  test_zero_position();
  test_zupt_then_inertial();
  test_range_filter();
  printf("test_nav_ekf: ok\n");
  return 0;
}
