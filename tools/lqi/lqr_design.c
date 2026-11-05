/**
 * @file lqr_design.c
 * @brief dlqr em C puro (contrato em lqr_design.h).
 */
#include "lqr_design.h"

#include <math.h>
#include <string.h>

/** Iterações máximas do algoritmo de duplicação (converge em dezenas). */
#define MAX_ITERATIONS 200
/** Variação relativa de P que encerra a iteração. */
#define TOLERANCE 1e-13

int lqr_inverse(int n, const double *a, double *out) {
  double m[LQR_MAX_N][2 * LQR_MAX_N];
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      m[i][j] = a[i * n + j];
      m[i][j + n] = (i == j) ? 1.0 : 0.0;
    }
  }
  for (int c = 0; c < n; ++c) {
    int p = c;
    for (int r = c + 1; r < n; ++r) {
      if (fabs(m[r][c]) > fabs(m[p][c])) p = r;
    }
    if (fabs(m[p][c]) < 1e-300) return -1;
    for (int j = 0; j < 2 * n; ++j) {
      const double t = m[c][j];
      m[c][j] = m[p][j];
      m[p][j] = t;
    }
    const double d = m[c][c];
    for (int j = 0; j < 2 * n; ++j) m[c][j] /= d;
    for (int r = 0; r < n; ++r) {
      if (r == c) continue;
      const double f = m[r][c];
      if (f == 0.0) continue;
      for (int j = 0; j < 2 * n; ++j) m[r][j] -= f * m[c][j];
    }
  }
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) out[i * n + j] = m[i][j + n];
  }
  return 0;
}

/** @brief out = a (n×k) · b (k×p). */
static void mul(int n, int k, int p, const double *a, const double *b, double *out) {
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < p; ++j) {
      double s = 0.0;
      for (int t = 0; t < k; ++t) s += a[i * k + t] * b[t * p + j];
      out[i * p + j] = s;
    }
  }
}

/** @brief out = a' (a é n×p, out é p×n). */
static void transpose(int n, int p, const double *a, double *out) {
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < p; ++j) out[j * n + i] = a[i * p + j];
  }
}

int lqr_dlqr(int n, int m, const double *A, const double *B, const double *q, const double *r,
             double *K) {
  /* Algoritmo de duplicação (SDA) para a equação de Riccati discreta
     X = A'XA - A'XB (R + B'XB)^-1 B'XA + Q. Converge quadraticamente, mesmo
     com modos de integrador (autovalor 1), onde a iteração simples demora
     centenas de milhares de passos:
       W = I + G H
       A <- A W^-1 A,  G <- G + A W^-1 G A',  H <- H + A' H W^-1 A
     com A0 = A, G0 = B R^-1 B', H0 = Q; H converge para X. */
  static double Ak[LQR_MAX_N * LQR_MAX_N], G[LQR_MAX_N * LQR_MAX_N], H[LQR_MAX_N * LQR_MAX_N];
  static double W[LQR_MAX_N * LQR_MAX_N], Wi[LQR_MAX_N * LQR_MAX_N], T1[LQR_MAX_N * LQR_MAX_N];
  static double T2[LQR_MAX_N * LQR_MAX_N], At[LQR_MAX_N * LQR_MAX_N], Hn[LQR_MAX_N * LQR_MAX_N];
  static double Gn[LQR_MAX_N * LQR_MAX_N], An[LQR_MAX_N * LQR_MAX_N];
  double Bt[LQR_MAX_M * LQR_MAX_N], BtX[LQR_MAX_M * LQR_MAX_N], S[LQR_MAX_M * LQR_MAX_M];
  double Si[LQR_MAX_M * LQR_MAX_M], BtXA[LQR_MAX_M * LQR_MAX_N];
  memcpy(Ak, A, sizeof(double) * (size_t)(n * n));
  transpose(n, m, B, Bt);
  memset(G, 0, sizeof(G));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      double s = 0.0;
      for (int t = 0; t < m; ++t) s += B[i * m + t] * Bt[t * n + j] / r[t];
      G[i * n + j] = s;
    }
  }
  memset(H, 0, sizeof(H));
  for (int i = 0; i < n; ++i) H[i * n + i] = q[i];

  int it;
  for (it = 0; it < MAX_ITERATIONS; ++it) {
    mul(n, n, n, G, H, W);
    for (int i = 0; i < n; ++i) W[i * n + i] += 1.0;
    if (lqr_inverse(n, W, Wi) != 0) return -1;
    transpose(n, n, Ak, At);
    mul(n, n, n, Wi, Ak, T1);              /* W^-1 A */
    mul(n, n, n, Ak, T1, An);              /* A W^-1 A */
    mul(n, n, n, Wi, G, T2);               /* W^-1 G */
    mul(n, n, n, Ak, T2, T1);              /* A W^-1 G */
    mul(n, n, n, T1, At, Gn);              /* A W^-1 G A' */
    mul(n, n, n, H, Wi, T2);               /* H W^-1 */
    mul(n, n, n, T2, Ak, T1);              /* H W^-1 A */
    mul(n, n, n, At, T1, Hn);              /* A' H W^-1 A */
    double change = 0.0, scale = 0.0;
    for (int i = 0; i < n * n; ++i) {
      Gn[i] += G[i];
      Hn[i] += H[i];
      change = fmax(change, fabs(Hn[i] - H[i]));
      scale = fmax(scale, fabs(Hn[i]));
    }
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        H[i * n + j] = 0.5 * (Hn[i * n + j] + Hn[j * n + i]);
        G[i * n + j] = 0.5 * (Gn[i * n + j] + Gn[j * n + i]);
      }
    }
    memcpy(Ak, An, sizeof(double) * (size_t)(n * n));
    if (!isfinite(change)) return -1;
    if (change <= TOLERANCE * (scale > 0.0 ? scale : 1.0)) break;
  }
  if (it == MAX_ITERATIONS) return -1;

  /* K = (R + B'XB)^-1 B'XA */
  mul(m, n, n, Bt, H, BtX);
  mul(m, n, m, BtX, B, S);
  for (int i = 0; i < m; ++i) S[i * m + i] += r[i];
  if (lqr_inverse(m, S, Si) != 0) return -1;
  mul(m, n, n, BtX, A, BtXA);
  mul(m, m, n, Si, BtXA, K);
  return it + 1;
}
