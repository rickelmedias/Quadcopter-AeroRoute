/**
 * @file test_lqg.c
 * @brief Testes de host do LQG genérico com os ganhos gerados.
 *
 * A planta é o próprio modelo discreto (Ad, Bd) com ruído de medição, e o MTF
 * só é medido a cada 4 passos, como no firmware. Não prova nada sobre o voo;
 * prova que o filtro, o LQR e a montagem das matrizes fecham uma malha estável
 * e que a regra de inovação e os limites funcionam.
 */
#include "lqg.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/** Ruído gaussiano aproximado (soma de uniformes), desvio padrão 1. */
static float noise(void) {
  float s = 0.0F;
  for (int i = 0; i < 12; ++i) s += (float)rand() / (float)RAND_MAX;
  return s - 6.0F;
}

static const unsigned y_state[LQG_NY] = {2, 6, 7, 3, 4, 5, 9, 10, 11};

/** @brief Avança a planta verdadeira um período. */
static void plant(float x[LQG_NX], const float u[LQG_NU]) {
  float next[LQG_NX];
  for (unsigned i = 0; i < LQG_NX; ++i) {
    float s = 0.0F;
    for (unsigned k = 0; k < LQG_NX; ++k) s += LQG_AD[i][k] * x[k];
    for (unsigned j = 0; j < LQG_NU; ++j) s += LQG_BD[i][j] * (u[j] - LQG_U0[j]);
    next[i] = s;
  }
  for (unsigned i = 0; i < LQG_NX; ++i) x[i] = next[i];
}

/** @brief Um desvio inicial de atitude e altura precisa voltar ao equilíbrio. */
static void test_regulation(void) {
  lqg_t f;
  lqg_init(&f, NULL);
  float x[LQG_NX] = {0};
  x[LQG_X_PHI] = 0.15F; x[LQG_X_THETA] = -0.10F; x[LQG_X_Z] = 0.3F; x[LQG_X_PSI] = 0.2F;
  const float ref[LQG_NX] = {0};
  float u[LQG_NU];
  for (int k = 0; k < 4000; ++k) {  /* 10 s a 400 Hz */
    lqg_predict(&f);
    for (unsigned i = 0; i < LQG_NY; ++i) {
      if (i <= LQG_Y_VY && (k % 4)) continue;  /* MTF a 100 Hz */
      const float y = x[y_state[i]] + sqrtf(LQG_RV[i]) * noise();
      lqg_update(&f, i, y, i >= LQG_Y_PHI && i <= LQG_Y_PSI);
    }
    lqg_control(&f, ref, u);
    for (unsigned j = 0; j < LQG_NU; ++j) {
      assert(u[j] >= LQG_U_MIN[j] && u[j] <= LQG_U_MAX[j]);
    }
    plant(x, u);
  }
  printf("  depois de 10 s: phi=%.4f theta=%.4f psi=%.4f z=%.4f\n",
         x[LQG_X_PHI], x[LQG_X_THETA], x[LQG_X_PSI], x[LQG_X_Z]);
  assert(fabsf(x[LQG_X_PHI]) < 0.01F && fabsf(x[LQG_X_THETA]) < 0.01F);
  assert(fabsf(x[LQG_X_PSI]) < 0.01F && fabsf(x[LQG_X_Z]) < 0.02F);
  for (unsigned i = 0; i < LQG_NX; ++i) {
    assert(isfinite(f.x[i]));
    assert(f.P[i][i] >= 0.0F && f.P[i][i] <= LQG_P_MAX[i] * 1.0001F);
  }
}

/** @brief Um salto absurdo do MTF precisa ser rejeitado pelo teste de inovação. */
static void test_gate(void) {
  lqg_t f;
  lqg_init(&f, NULL);
  for (int k = 0; k < 400; ++k) {
    lqg_predict(&f);
    lqg_update(&f, LQG_Y_Z, 0.0F, false);
  }
  assert(!lqg_update(&f, LQG_Y_Z, 5.0F, false));
  assert(f.rejected[LQG_Y_Z] == 1U);
  assert(fabsf(f.x[LQG_X_Z]) < 0.01F);
}

/** @brief A inovação de ângulo precisa dar a volta em +-pi. */
static void test_wrap(void) {
  lqg_t f;
  float x0[LQG_NX] = {0};
  x0[LQG_X_PSI] = 3.1F;
  lqg_init(&f, x0);
  assert(lqg_update(&f, LQG_Y_PSI, -3.1F, true));
  /* -3,1 é 0,083 rad à frente de 3,1; o estado não pode atravessar o zero. */
  assert(fabsf(f.x[LQG_X_PSI]) > 3.0F);
}

int main(void) {
  srand(1);
  test_regulation();
  test_gate();
  test_wrap();
  printf("test_lqg: ok\n");
  return 0;
}
