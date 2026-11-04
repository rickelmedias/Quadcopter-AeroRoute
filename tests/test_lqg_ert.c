/**
 * @file test_lqg_ert.c
 * @brief Equivalência entre control/lqg_ert.c e o código gerado pelo Simulink.
 *
 * O código de referência (reference/STM32_ert_rtw/STM32.c e STM32_data.c) é
 * compilado e ligado junto com o módulo do firmware. Os dois recebem as
 * mesmas medidas e referências por milhares de passos:
 *
 * - em malha aberta, com entradas aleatórias grandes;
 * - em malha fechada, com uma planta linear (o próprio A, B) e ruído de
 *   medida, que é o regime real de uso.
 *
 * Com lqg_real_t em double, a saída e o estado interno precisam ser iguais
 * bit a bit (comparação com ==). Compilado com -DLQG_ERT_SINGLE_PRECISION, o
 * teste mede o erro do float (~2e-4 relativo em malha fechada) e exige que
 * ele fique abaixo de 1e-3.
 */
#include "control/lqg_ert.h"
#include "control/lqg_ert_model.h"
#include "STM32.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Gerador pseudoaleatório determinístico (xorshift), para o teste repetir. */
static unsigned long long rng_state = 0x9E3779B97F4A7C15ULL;

/** @brief Número uniforme em [-1, 1]. */
static double uniform(void) {
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 7;
  rng_state ^= rng_state << 17;
  return (double)(rng_state >> 11) / (double)(1ULL << 53) * 2.0 - 1.0;
}

#if defined(LQG_ERT_SINGLE_PRECISION)
/** Maior erro relativo observado (só no modo float). */
static double worst_relative = 0.0;
#endif

/**
 * @brief Compara a saída e o estado do firmware com os da referência.
 * @param[in] s   Filtro do firmware.
 * @param[in] du  Saída do firmware.
 */
static void compare(const lqg_ert_t *s, const lqg_real_t du[LQG_NU]) {
#if defined(LQG_ERT_SINGLE_PRECISION)
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    const double ref = STM32_Y.Out1[i];
    const double err = fabs((double)du[i] - ref) / (1.0 + fabs(ref));
    if (err > worst_relative) worst_relative = err;
  }
  for (unsigned i = 0U; i < LQG_NX; ++i) {
    const double ref = STM32_DW.MemoryX_DSTATE[i];
    const double err = fabs((double)s->x_pred[i] - ref) / (1.0 + fabs(ref));
    if (err > worst_relative) worst_relative = err;
  }
#else
  for (unsigned i = 0U; i < LQG_NU; ++i) assert(du[i] == STM32_Y.Out1[i]);
  for (unsigned i = 0U; i < LQG_NX; ++i) assert(s->x_pred[i] == STM32_DW.MemoryX_DSTATE[i]);
#endif
}

/** @brief Malha aberta: medidas e referências aleatórias e grandes. */
static void test_open_loop(void) {
  lqg_ert_t s;
  lqg_ert_init(&s);
  STM32_initialize();
  memset(&STM32_DW, 0, sizeof(STM32_DW));
  STM32_DW.icLoad = true;

  for (int k = 0; k < 20000; ++k) {
    lqg_real_t y[LQG_NY], r[LQG_NR], du[LQG_NU];
    for (unsigned i = 0U; i < LQG_NY; ++i) {
      STM32_U.y[i] = uniform() * 2.0;
      y[i] = (lqg_real_t)STM32_U.y[i];
    }
    for (unsigned i = 0U; i < LQG_NR; ++i) {
      STM32_U.In1[i] = uniform();
      r[i] = (lqg_real_t)STM32_U.In1[i];
    }
#if defined(LQG_ERT_SINGLE_PRECISION)
    /* Mesma entrada vista pelos dois: a referência recebe o valor já
       arredondado para float. */
    for (unsigned i = 0U; i < LQG_NY; ++i) STM32_U.y[i] = y[i];
    for (unsigned i = 0U; i < LQG_NR; ++i) STM32_U.In1[i] = r[i];
#endif
    STM32_step();
    lqg_ert_step(&s, &LQG_ERT_DEFAULT_GAINS, y, r, du);
    compare(&s, du);
  }
  printf("  malha aberta: 20000 passos\n");
}

/**
 * @brief Malha fechada: o LQG controla uma planta x+ = A x + B u, medida com
 *        ruído, enquanto a referência anda em degraus.
 *
 * Também confere que o hover converge para a referência (o modelo e os
 * ganhos estão coerentes, e não só iguais à referência).
 */
static void test_closed_loop(void) {
  lqg_ert_t s;
  lqg_ert_init(&s);
  memset(&STM32_DW, 0, sizeof(STM32_DW));
  STM32_DW.icLoad = true;

  double x[LQG_NX] = {0};
  x[LQG_X_PHI] = 0.1;    /* começa inclinado */
  x[LQG_X_Z] = -0.05;    /* e a 5 cm do equilíbrio */
  double target[LQG_NR] = {0.0, 0.0, -0.5, 0.0};

  for (int k = 0; k < 6000; ++k) {  /* 60 s */
    if (k == 2000) { target[LQG_R_X] = 0.3; target[LQG_R_PSI] = 0.2; }
    if (k == 4000) { target[LQG_R_Z] = -0.8; target[LQG_R_Y] = -0.2; }

    lqg_real_t y[LQG_NY], r[LQG_NR], du[LQG_NU];
    for (unsigned i = 0U; i < LQG_NY; ++i) {
      double c = 0.0;
      for (unsigned j = 0U; j < LQG_NX; ++j) c += LQG_ERT_C[i][j] * x[j];
      y[i] = (lqg_real_t)(c + 1e-3 * uniform());
      STM32_U.y[i] = y[i];
    }
    for (unsigned i = 0U; i < LQG_NR; ++i) {
      r[i] = (lqg_real_t)target[i];
      STM32_U.In1[i] = r[i];
    }
    STM32_step();
    lqg_ert_step(&s, &LQG_ERT_DEFAULT_GAINS, y, r, du);
    compare(&s, du);

    double next[LQG_NX];
    for (unsigned i = 0U; i < LQG_NX; ++i) {
      double v = 0.0;
      for (unsigned j = 0U; j < LQG_NX; ++j) v += LQG_ERT_A[i][j] * x[j];
      for (unsigned j = 0U; j < LQG_NU; ++j) v += LQG_ERT_B[i][j] * STM32_Y.Out1[j];
      next[i] = v;
    }
    memcpy(x, next, sizeof(x));
  }
  /* Depois de 20 s no último degrau, o drone está na referência. */
  assert(fabs(x[LQG_X_X] - 0.3) < 0.02);
  assert(fabs(x[LQG_X_Y] + 0.2) < 0.02);
  assert(fabs(x[LQG_X_Z] + 0.8) < 0.01);
  assert(fabs(x[LQG_X_PSI] - 0.2) < 0.01);
  assert(fabs(x[LQG_X_PHI]) < 0.01 && fabs(x[LQG_X_THETA]) < 0.01);
  printf("  malha fechada: 6000 passos, converge para x=%.3f y=%.3f z=%.3f psi=%.3f\n",
         x[LQG_X_X], x[LQG_X_Y], x[LQG_X_Z], x[LQG_X_PSI]);
}

/** @brief Separar saída e predição com o mesmo du é o mesmo passo. */
static void test_split_step_matches_step(void) {
  lqg_ert_t a, b;
  lqg_ert_init(&a);
  lqg_ert_init(&b);
  for (int k = 0; k < 500; ++k) {
    lqg_real_t y[LQG_NY], r[LQG_NR], du_a[LQG_NU], du_b[LQG_NU];
    for (unsigned i = 0U; i < LQG_NY; ++i) y[i] = (lqg_real_t)uniform();
    for (unsigned i = 0U; i < LQG_NR; ++i) r[i] = (lqg_real_t)uniform();
    lqg_ert_step(&a, &LQG_ERT_DEFAULT_GAINS, y, r, du_a);
    lqg_ert_output(&b, &LQG_ERT_DEFAULT_GAINS, y, r, du_b);
    lqg_ert_update(&b, &LQG_ERT_DEFAULT_GAINS, du_b);
    assert(memcmp(du_a, du_b, sizeof(du_a)) == 0);
    assert(memcmp(a.x_pred, b.x_pred, sizeof(a.x_pred)) == 0);
  }
}

/** @brief Antes do primeiro passo a estimativa é zero, mesmo com lixo na memória. */
static void test_first_step_starts_from_zero(void) {
  lqg_ert_t s;
  memset(&s, 0x5A, sizeof(s));
  lqg_ert_init(&s);
  for (unsigned i = 0U; i < LQG_NX; ++i) s.x_pred[i] = 123.0F;  /* "lixo" */
  s.ic_load = true;
  const lqg_real_t y[LQG_NY] = {0};
  const lqg_real_t r[LQG_NR] = {0};
  lqg_real_t du[LQG_NU];
  lqg_ert_output(&s, &LQG_ERT_DEFAULT_GAINS, y, r, du);
  for (unsigned i = 0U; i < LQG_NU; ++i) assert(du[i] == 0.0F);
}

int main(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  test_first_step_starts_from_zero();
  test_split_step_matches_step();
  test_open_loop();
#if defined(LQG_ERT_SINGLE_PRECISION)
  /* Em malha aberta, x e y (integradores quase puros, polo 0,99999) acumulam
     o arredondamento do float por 20000 passos: é o pior caso possível. */
  printf("  float, malha aberta: maior erro relativo = %.3g\n", worst_relative);
  assert(worst_relative < 1e-2);
  worst_relative = 0.0;
#endif
  test_closed_loop();
#if defined(LQG_ERT_SINGLE_PRECISION)
  printf("  float, malha fechada: maior erro relativo = %.3g\n", worst_relative);
  assert(worst_relative < 1e-3);
  printf("test_lqg_ert (float): ok\n");
#else
  printf("test_lqg_ert (double, idêntico bit a bit ao Simulink): ok\n");
#endif
  return 0;
}
