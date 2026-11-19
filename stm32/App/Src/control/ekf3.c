/**
 * @file ekf3.c
 * @brief EKF de erro de estado com quatérnio (contrato em control/ekf3.h).
 */
#include "control/ekf3.h"

#include "control/lqg_config.h"
#include "control/quat.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* ---- Ruídos (ajustáveis; ponto de partida do EKF de Euler e da bancada de 2026-11-11) ---- */

/** Incerteza inicial (desvio padrão): atitude, velocidade, posição, viés de giro, viés de aceleração. */
#define SIGMA0_ATT  0.02F
#define SIGMA0_VEL  0.10F
#define SIGMA0_POS  0.01F
#define SIGMA0_BG   0.005F
#define SIGMA0_BA   0.05F
/**
 * Ruído do IMU (desvio padrão, por raiz de segundo): giro e aceleração linear. Inflados de
 * propósito: com as hélices a vibração e o filtro interno do BNO atrasam a aceleração. O giro
 * em 0,1 equivale ao Q_ANG de 1e-6 por passo do EKF de Euler (segue o quatérnio do BNO).
 */
#define SIG_GYRO    0.10F
#define SIG_ACCEL   0.50F
/** Passeio aleatório dos vieses (por raiz de segundo): giro em rad/s, aceleração em m/s². */
#define SIG_BG_WALK 0.0005F
#define SIG_BA_WALK 0.01F
/** Aceleração desconhecida sem o IMU (desvio padrão, m/s²): a velocidade deixa de ser confiável. */
#define SIG_ACCEL_LOST 3.0F
/** Ruído das medidas de atitude (variância, rad²). O BNO entrega saída já fusionada: inflado. */
#define R_ATT_XY    1e-5F
#define R_ATT_Z     3e-5F
#define R_RANGE     1e-4F
/** Ruído do fluxo em rad/s (1 sigma) com boa qualidade e piso da variância de velocidade. */
#define FLOW_SIGMA_RAD_S 0.05F
#define R_FLOW_FLOOR     1e-3F
/** Qualidade de referência do fluxo (R cresce com (ref/q)^2 abaixo dela). */
#define FLOW_Q_REF       100.0F
#define FLOW_Q_MIN       20.0F
/** Teste de inovação: qui-quadrado de 1 grau (9 = 3 sigma; 16 = 4 sigma). */
#define GATE_RANGE       16.0F
#define GATE_FLOW        9.0F
/** Rejeições seguidas depois das quais a medida volta com R x10 (evita travar fora). */
#define REJECT_LIMIT     25U
/**
 * Idem para a distância: chega a ~20 Hz e é a única referência de altura, então 25 quadros eram
 * 1,25 s de altura estimada solta (log 20261114_184336: de 11 cm a 2,2 m com o MTF lendo 8 cm).
 */
#define REJECT_LIMIT_RANGE 4U
/**
 * Aceleração linear que a previsão aceita por eixo, em m/s². Acima disso ela é cortada neste
 * valor e tratada como desconhecida (SIG_ACCEL_LOST): P cresce e a distância corrige a altura em
 * vez de ser rejeitada. O BNO deu -10 a -25 m/s² por 0,45 s na queda e no impacto
 * (log 20261114_184336) e a velocidade integrada virou -7 m/s.
 */
#define ACCEL_MAX        3.0F
/** Menor cos(phi)cos(theta) em que a distância ainda vira altura. */
#define MIN_TILT_COS     0.5F
/** Altura mínima para o fluxo valer como velocidade, em m. */
#define FLOW_MIN_RANGE_M 0.04F
/**
 * Limite físico dos vieses estimados: giro calibrado do BNO em rad/s, aceleração linear em m/s².
 * Um salto de atitude sem giro correspondente não pode virar viés absurdo (e rotação falsa).
 */
#define BG_MAX           0.05F
#define BA_MAX           1.0F
/** Piso da variância na diagonal de P. */
#define P_FLOOR          1e-12F
/** Tetos da variância (diagonal de P): atitude em rad², viés de giro em (rad/s)², viés de aceleração em (m/s²)². */
#define P_MAX_ATT        1.0F
#define P_MAX_BG         0.01F
#define P_MAX_BA         4.0F
/** Passos sem fluxo a partir dos quais o viés de aceleração em x, y congela (0,5 s). */
#define FLOW_FRESH_STEPS 50U

#define DT LQG_TS_S

/** @brief Matriz antissimétrica [v]x. */
static void skew(const float v[3], float m[3][3]) {
  m[0][0] = 0.0F;
  m[0][1] = -v[2];
  m[0][2] = v[1];
  m[1][0] = v[2];
  m[1][1] = 0.0F;
  m[1][2] = -v[0];
  m[2][0] = -v[1];
  m[2][1] = v[0];
  m[2][2] = 0.0F;
}

void ekf3_seed(ekf3_t *e, const float q[4], float pz, const float bg[3], const float ba[3]) {
  (void)memset(e, 0, sizeof(*e));
  if (q != NULL) {
    (void)memcpy(e->q, q, sizeof(e->q));
    quat_normalize(e->q);
  } else {
    quat_identity(e->q);
  }
  e->p[2] = pz;
  e->flow_age = FLOW_FRESH_STEPS + 1U;
  for (uint32_t i = 0U; i < 3U; ++i) {
    e->bg[i] = (bg != NULL) ? bg[i] : 0.0F;
    e->ba[i] = (ba != NULL) ? ba[i] : 0.0F;
    e->P[EKF3_DTH + i][EKF3_DTH + i] = SIGMA0_ATT * SIGMA0_ATT;
    e->P[EKF3_DV + i][EKF3_DV + i] = SIGMA0_VEL * SIGMA0_VEL;
    e->P[EKF3_DP + i][EKF3_DP + i] = SIGMA0_POS * SIGMA0_POS;
    e->P[EKF3_DBG + i][EKF3_DBG + i] = SIGMA0_BG * SIGMA0_BG;
    e->P[EKF3_DBA + i][EKF3_DBA + i] = SIGMA0_BA * SIGMA0_BA;
  }
}

void ekf3_reset_xy(ekf3_t *e) {
  for (uint32_t k = 0U; k < 2U; ++k) {
    const uint32_t i = EKF3_DP + k;
    e->p[k] = 0.0F;
    for (uint32_t j = 0U; j < EKF3_NX; ++j) {
      e->P[i][j] = 0.0F;
      e->P[j][i] = 0.0F;
    }
    e->P[i][i] = SIGMA0_POS * SIGMA0_POS;
  }
}

static float clampf(float v, float lim) {
  return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

/** @brief Entrega o erro estimado @p dx ao estado nominal; o erro volta a zero. */
static void inject(ekf3_t *e, const float dx[EKF3_NX]) {
  float dq[4];
  float q[4];
  quat_from_rotvec(&dx[EKF3_DTH], dq);
  quat_mul(e->q, dq, q);
  quat_normalize(q);
  (void)memcpy(e->q, q, sizeof(q));
  for (uint32_t i = 0U; i < 3U; ++i) {
    e->v[i] += dx[EKF3_DV + i];
    e->p[i] += dx[EKF3_DP + i];
    e->bg[i] = clampf(e->bg[i] + dx[EKF3_DBG + i], BG_MAX);
    e->ba[i] = clampf(e->ba[i] + dx[EKF3_DBA + i], BA_MAX);
  }
}

/**
 * @brief Atualização escalar: y - h(x) = @p innov, com h' = @p h (no estado de erro) e variância @p r.
 * @param[in]     gate     Limite do teste de inovação; <= 0 desliga.
 * @param[in,out] rejects  Rejeições seguidas desta medida.
 */
static void scalar_update(ekf3_t *e, const float h[EKF3_NX], float innov, float r, float gate,
                          uint32_t *rejects, uint32_t reject_limit) {
  float ph[EKF3_NX];
  float hph = 0.0F;
  for (uint32_t i = 0U; i < EKF3_NX; ++i) {
    float s = 0.0F;
    for (uint32_t j = 0U; j < EKF3_NX; ++j) {
      s += e->P[i][j] * h[j];
    }
    ph[i] = s;
    hph += h[i] * s;
  }
  float rr = r;
  float s = hph + rr;
  bool accept = true;
  if ((gate > 0.0F) && ((innov * innov) > (gate * s))) {
    ++(*rejects);
    ++e->rejected_total;
    if (*rejects >= reject_limit) {
      /* Fora há tempo demais: aceita com pouca confiança para reancorar. */
      rr = r * 10.0F;
      s = hph + rr;
      *rejects = 0U;
    } else {
      accept = false;
    }
  } else {
    *rejects = 0U;
  }
  if (!accept || (s <= 1e-12F)) {
    return;
  }
  /* Como no EKF3 (FinishFusion): se a variância de algum estado ficaria negativa, pula. */
  for (uint32_t i = 0U; i < EKF3_NX; ++i) {
    if ((ph[i] * ph[i]) > (e->P[i][i] * s * 1.001F)) {
      ++e->variance_skips;
      return;
    }
  }
  float dx[EKF3_NX];
  for (uint32_t i = 0U; i < EKF3_NX; ++i) {
    dx[i] = (ph[i] / s) * innov;
  }
  /* P -= K s K': só o triângulo superior é calculado e espelhado, P fica simétrica. */
  for (uint32_t i = 0U; i < EKF3_NX; ++i) {
    for (uint32_t j = i; j < EKF3_NX; ++j) {
      const float v = e->P[i][j] - ((ph[i] * ph[j]) / s);
      e->P[i][j] = v;
      e->P[j][i] = v;
    }
    if (e->P[i][i] < P_FLOOR) {
      e->P[i][i] = P_FLOOR;
    }
  }
  inject(e, dx);
}

/** @brief Resíduo de atitude no corpo: vetor de rotação de q̂⁻¹ ⊗ q_meas. */
static void attitude_residual(const ekf3_t *e, const float q_meas[4], float rv[3]) {
  float qi[4];
  float qe[4];
  quat_conj(e->q, qi);
  quat_mul(qi, q_meas, qe);
  quat_to_rotvec(qe, rv);
}

/** @brief Atualização do fluxo com a velocidade do corpo prevista (R^T v), um canal (0 frente, 1 direita). */
static void flow_channel(ekf3_t *e, uint32_t ch, float meas, float r, uint32_t *rejects) {
  float rm[3][3];
  quat_to_dcm(e->q, rm);
  float vb[3];
  quat_rotate_inv(e->q, e->v, vb);
  float h[EKF3_NX] = {0.0F};
  for (uint32_t i = 0U; i < 3U; ++i) {
    h[EKF3_DV + i] = rm[i][ch];
  }
  float sk[3][3];
  skew(vb, sk);
  for (uint32_t i = 0U; i < 3U; ++i) {
    h[EKF3_DTH + i] = sk[ch][i];
  }
  scalar_update(e, h, meas - vb[ch], r, GATE_FLOW, rejects, REJECT_LIMIT);
}

void ekf3_update(ekf3_t *e, const ekf3_meas_t *m) {
  uint32_t no_gate = 0U; /* medidas diretas não têm teste de inovação */
  if (m->gyro_valid) {
    for (uint32_t i = 0U; i < 3U; ++i) {
      e->rate[i] = m->gyro[i] - e->bg[i];
    }
  }
  if (m->attitude_valid) {
    for (uint32_t k = 0U; k < 3U; ++k) {
      float rv[3];
      attitude_residual(e, m->q_meas, rv);
      float h[EKF3_NX] = {0.0F};
      h[EKF3_DTH + k] = 1.0F;
      const float scale = (m->att_r_scale > 1.0F) ? m->att_r_scale : 1.0F;
      scalar_update(e, h, rv[k], scale * ((k < 2U) ? R_ATT_XY : R_ATT_Z), 0.0F, &no_gate,
                    REJECT_LIMIT);
    }
  }
  float rm[3][3];
  quat_to_dcm(e->q, rm);
  const float tilt = rm[2][2];
  if (m->range_valid && (tilt > MIN_TILT_COS)) {
    /* Distância no eixo do sensor = (chão + altura + braço) / (cos phi cos theta), com o braço
       = -(R b)_z: quanto o sensor fora do centro sobe com a inclinação. */
    const float lever_z = (m->range_pos_x * rm[2][0]) + (m->range_pos_y * rm[2][1]);
    const float num = m->ground_m - e->p[2] - lever_z;
    /* d(R31, R32, R33)/dδθ, com R' = R (I + [δθ]x). */
    const float d31[3] = {0.0F, -rm[2][2], rm[2][1]};
    const float d32[3] = {rm[2][2], 0.0F, -rm[2][0]};
    const float d33[3] = {-rm[2][1], rm[2][0], 0.0F};
    float h[EKF3_NX] = {0.0F};
    h[EKF3_DP + 2U] = -1.0F / tilt;
    for (uint32_t i = 0U; i < 3U; ++i) {
      h[EKF3_DTH + i] = (-((m->range_pos_x * d31[i]) + (m->range_pos_y * d32[i])) / tilt) -
                        ((num * d33[i]) / (tilt * tilt));
    }
    scalar_update(e, h, m->range_m - (num / tilt), R_RANGE, GATE_RANGE, &e->rejected[0],
                  REJECT_LIMIT_RANGE);
  }
  if (m->flow_valid && m->range_valid && (m->range_m > FLOW_MIN_RANGE_M)) {
    const float q = (m->flow_quality < FLOW_Q_MIN) ? FLOW_Q_MIN : (float)m->flow_quality;
    const float ratio = (q < FLOW_Q_REF) ? (FLOW_Q_REF / q) : 1.0F;
    const float sigma = FLOW_SIGMA_RAD_S * ratio * m->range_m;
    const float r = (sigma * sigma) + R_FLOW_FLOOR;
    e->flow_age = 0U;
    flow_channel(e, 0U, m->flow_fwd, r, &e->rejected[1]);
    /* A primeira atualização mudou x e P: a previsão do segundo canal é refeita lá dentro. */
    flow_channel(e, 1U, m->flow_right, r, &e->rejected[2]);
  }
}

void ekf3_predict(ekf3_t *e, const ekf3_meas_t *m) {
  float w[3];
  float ab[3] = {0.0F, 0.0F, 0.0F};
  bool accel_implausible = false;
  for (uint32_t i = 0U; i < 3U; ++i) {
    w[i] = m->gyro_valid ? (m->gyro[i] - e->bg[i]) : e->rate[i];
    if (m->accel_valid) {
      const float raw = m->accel[i] - e->ba[i];
      accel_implausible = accel_implausible || (fabsf(raw) > ACCEL_MAX);
      ab[i] = clampf(raw, ACCEL_MAX);
    }
  }
  float rm[3][3];
  quat_to_dcm(e->q, rm);
  float an[3];
  quat_rotate(e->q, ab, an);

  /* Jacobiana F = I + dt A do estado de erro (atitude no corpo). */
  float F[EKF3_NX][EKF3_NX];
  (void)memset(F, 0, sizeof(F));
  for (uint32_t i = 0U; i < EKF3_NX; ++i) {
    F[i][i] = 1.0F;
  }
  float sw[3][3];
  float sa[3][3];
  skew(w, sw);
  skew(ab, sa);
  for (uint32_t i = 0U; i < 3U; ++i) {
    for (uint32_t j = 0U; j < 3U; ++j) {
      float ra = 0.0F; /* R [a]x */
      for (uint32_t k = 0U; k < 3U; ++k) {
        ra += rm[i][k] * sa[k][j];
      }
      F[EKF3_DTH + i][EKF3_DTH + j] -= DT * sw[i][j];
      F[EKF3_DV + i][EKF3_DTH + j] = -DT * ra;
      F[EKF3_DV + i][EKF3_DBA + j] = -DT * rm[i][j];
    }
    F[EKF3_DTH + i][EKF3_DBG + i] = -DT;
    F[EKF3_DP + i][EKF3_DV + i] = DT;
  }

  /* Estado nominal. */
  float dq[4];
  float wdt[3] = {w[0] * DT, w[1] * DT, w[2] * DT};
  float qn[4];
  quat_from_rotvec(wdt, dq);
  quat_mul(e->q, dq, qn);
  quat_normalize(qn);
  for (uint32_t i = 0U; i < 3U; ++i) {
    e->p[i] += (e->v[i] * DT) + (0.5F * an[i] * DT * DT);
    e->v[i] += an[i] * DT;
    e->rate[i] = w[i];
  }
  (void)memcpy(e->q, qn, sizeof(qn));

  /* Viés de aceleração em x, y: sem fluxo recente não é observável e fica congelado. */
  const bool ba_off = e->flow_age > FLOW_FRESH_STEPS;
  if (e->flow_age < UINT32_MAX) {
    ++e->flow_age;
  }

  /* P = F P F' + Q */
  float fp[EKF3_NX][EKF3_NX];
  for (uint32_t i = 0U; i < EKF3_NX; ++i) {
    for (uint32_t j = 0U; j < EKF3_NX; ++j) {
      float v = 0.0F;
      for (uint32_t k = 0U; k < EKF3_NX; ++k) {
        v += F[i][k] * e->P[k][j];
      }
      fp[i][j] = v;
    }
  }
  const float sa_acc = (m->accel_valid && !accel_implausible) ? SIG_ACCEL : SIG_ACCEL_LOST;
  const float q_th = SIG_GYRO * SIG_GYRO * DT * DT;
  const float q_v = sa_acc * sa_acc * DT * DT;
  const float q_bg = SIG_BG_WALK * SIG_BG_WALK * DT;
  const float q_ba = SIG_BA_WALK * SIG_BA_WALK * DT;
  for (uint32_t i = 0U; i < EKF3_NX; ++i) {
    for (uint32_t j = i; j < EKF3_NX; ++j) {
      float v = 0.0F;
      for (uint32_t k = 0U; k < EKF3_NX; ++k) {
        v += fp[i][k] * F[j][k];
      }
      if (i == j) {
        v += (i < EKF3_DV) ? q_th : ((i < EKF3_DP) ? q_v : ((i < EKF3_DBG) ? 0.0F : ((i < EKF3_DBA) ? q_bg : q_ba)));
      }
      e->P[i][j] = v;
      e->P[j][i] = v;
    }
    if (e->P[i][i] < P_FLOOR) {
      e->P[i][i] = P_FLOOR;
    }
  }
  /* Tetos de variância (EKF3: ConstrainVariances). */
  for (uint32_t i = 0U; i < 3U; ++i) {
    e->P[EKF3_DTH + i][EKF3_DTH + i] = fminf(e->P[EKF3_DTH + i][EKF3_DTH + i], P_MAX_ATT);
    e->P[EKF3_DBG + i][EKF3_DBG + i] = fminf(e->P[EKF3_DBG + i][EKF3_DBG + i], P_MAX_BG);
    e->P[EKF3_DBA + i][EKF3_DBA + i] = fminf(e->P[EKF3_DBA + i][EKF3_DBA + i], P_MAX_BA);
  }
  for (uint32_t k = 0U; k < 2U; ++k) {
    const uint32_t i = EKF3_DBA + k;
    if (ba_off) {
      for (uint32_t j = 0U; j < EKF3_NX; ++j) {
        e->P[i][j] = 0.0F;
        e->P[j][i] = 0.0F;
      }
      e->P[i][i] = P_FLOOR;
    } else if (e->ba_xy_off) {
      e->P[i][i] = SIGMA0_BA * SIGMA0_BA; /* volta a aprender, com a incerteza inicial */
    } else {
      /* já aprendendo */
    }
  }
  e->ba_xy_off = ba_off;
}

void ekf3_estimate(const ekf3_t *e, float x[EKF3_NEST]) {
  x[EKF3_PX] = e->p[0];
  x[EKF3_PY] = e->p[1];
  x[EKF3_PZ] = e->p[2];
  quat_to_euler(e->q, &x[EKF3_PHI], &x[EKF3_THETA], &x[EKF3_PSI]);
  x[EKF3_U] = e->v[0];
  x[EKF3_V] = e->v[1];
  x[EKF3_W] = e->v[2];
  x[EKF3_P] = e->rate[0];
  x[EKF3_Q] = e->rate[1];
  x[EKF3_R] = e->rate[2];
}

void ekf3_attitude(const ekf3_t *e, float tilt[2], float *yaw) {
  const float psi = quat_yaw(e->q);
  float qy[4];
  float qyi[4];
  float qt[4];
  float rv[3];
  quat_from_yaw(psi, qy);
  quat_conj(qy, qyi);
  quat_mul(qyi, e->q, qt);
  quat_to_rotvec(qt, rv);
  tilt[0] = rv[0];
  tilt[1] = rv[1];
  *yaw = psi;
}
