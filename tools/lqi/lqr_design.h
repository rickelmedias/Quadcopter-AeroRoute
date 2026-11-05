/**
 * @file lqr_design.h
 * @brief Projeto do LQR discreto (o `dlqr` do MATLAB) e do LQI em C puro.
 *
 * Roda no PC, não no STM32: gera as tabelas de App/Src/control/lqi_gains.c e é
 * usado pelos testes. Com os pesos do TCC, ::lqr_dlqr reproduz o Kd exportado
 * pelo Simulink (tests/test_lqi.c confere).
 *
 * Matrizes em ordem de linha, como vetores `double` (a[i * cols + j]).
 */
#ifndef TOOLS_LQR_DESIGN_H
#define TOOLS_LQR_DESIGN_H

/** Maior dimensão de estado suportada (12 do modelo + 3 integradores). */
#define LQR_MAX_N 16
/** Maior número de entradas suportado. */
#define LQR_MAX_M 4

/**
 * @brief Inverte uma matriz quadrada n×n por Gauss-Jordan com pivoteamento.
 *
 * @param[in]  n    Dimensão (até ::LQR_MAX_N).
 * @param[in]  a    Matriz.
 * @param[out] out  Inversa.
 * @return 0 se deu certo, -1 se a matriz for singular.
 */
int lqr_inverse(int n, const double *a, double *out);

/**
 * @brief K = dlqr(A, B, Q, R), pela equação de Riccati discreta.
 *
 * Resolve a equação pelo algoritmo de duplicação (SDA), que converge em
 * dezenas de passos mesmo com modos de integrador.
 *
 * @param[in]  n   Estados.
 * @param[in]  m   Entradas.
 * @param[in]  A   n×n.
 * @param[in]  B   n×m.
 * @param[in]  q   Diagonal de Q (n valores).
 * @param[in]  r   Diagonal de R (m valores).
 * @param[out] K   m×n, com u = -K x.
 * @return Iterações usadas, ou -1 se não convergiu.
 */
int lqr_dlqr(int n, int m, const double *A, const double *B, const double *q, const double *r,
             double *K);

#endif
