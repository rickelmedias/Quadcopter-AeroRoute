/**
 * @file nav_ekf.c
 * @brief EKF de navegação x, y, z com fluxo, telêmetro e acelerômetro
 *        (contrato em control/nav_ekf.h).
 */
#include "control/nav_ekf.h"

#include <math.h>
#include <string.h>

/** Incerteza inicial da posição, em m². A posição começa na origem, por definição. */
#define INITIAL_POS_VAR 0.0F
/** Incerteza inicial da velocidade, em (m/s)². */
#define INITIAL_VEL_VAR 1.0F
/** Incerteza inicial do viés, em (m/s²)². */
#define INITIAL_BIAS_VAR 0.25F

/** @brief Zera estado e covariância de um eixo. */
static void axis_reset(nav_ekf_axis_t *a, float bias) {
  memset(a, 0, sizeof(*a));
  a->x[2] = bias;
  a->P[0][0] = INITIAL_POS_VAR;
  a->P[1][1] = INITIAL_VEL_VAR;
  a->P[2][2] = INITIAL_BIAS_VAR;
}

void nav_ekf_reset(nav_ekf_t *f) {
  memset(f, 0, sizeof(*f));
  axis_reset(&f->h[0], 0.0F);
  axis_reset(&f->h[1], 0.0F);
  axis_reset(&f->z, 0.0F);
  range_filter_reset(&f->range);
}

void nav_ekf_rearm(nav_ekf_t *f) {
  const float bx = f->h[0].x[2], by = f->h[1].x[2], bz = f->z.x[2];
  const float vbx = f->h[0].P[2][2], vby = f->h[1].P[2][2], vbz = f->z.P[2][2];
  nav_ekf_reset(f);
  axis_reset(&f->h[0], bx);
  axis_reset(&f->h[1], by);
  axis_reset(&f->z, bz);
  f->h[0].P[2][2] = vbx;
  f->h[1].P[2][2] = vby;
  f->z.P[2][2] = vbz;
}

/**
 * @brief Predição de um eixo.
 *
 * F = [1 dt -dt²/2; 0 1 -dt; 0 0 1]; o ruído da aceleração entra por
 * G = [dt²/2; dt; 0] e o do viés como passeio aleatório.
 */
static void predict_axis(nav_ekf_axis_t *a, float accel, float dt, float accel_noise,
                         float bias_noise) {
  const float h = 0.5F * dt * dt;
  const float u = accel - a->x[2];
  a->x[0] += a->x[1] * dt + u * h;
  a->x[1] += u * dt;
  const float F[3][3] = {{1.0F, dt, -h}, {0.0F, 1.0F, -dt}, {0.0F, 0.0F, 1.0F}};
  float FP[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      FP[i][j] = F[i][0] * a->P[0][j] + F[i][1] * a->P[1][j] + F[i][2] * a->P[2][j];
    }
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      a->P[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1] + FP[i][2] * F[j][2];
    }
  }
  const float G[3] = {h, dt, 0.0F};
  const float qa = accel_noise * accel_noise;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) a->P[i][j] += qa * G[i] * G[j];
  }
  a->P[2][2] += bias_noise * bias_noise * dt;
}

/** @brief Variância da inovação de uma medida do estado @p k (0 = posição, 1 = velocidade). */
static float innov_var(const nav_ekf_axis_t *a, int k, float R) {
  return a->P[k][k] + R;
}

/** @brief Fusão escalar da medida @p z do estado @p k, com variância @p R. */
static void fuse(nav_ekf_axis_t *a, int k, float z, float R) {
  const float S = a->P[k][k] + R;
  if (!(S > 0.0F)) return;
  const float innov = z - a->x[k];
  float K[3];
  for (int i = 0; i < 3; ++i) K[i] = a->P[i][k] / S;
  for (int i = 0; i < 3; ++i) a->x[i] += K[i] * innov;
  float row[3];
  for (int j = 0; j < 3; ++j) row[j] = a->P[k][j];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) a->P[i][j] -= K[i] * row[j];
  }
  /* Mantém simétrica e com diagonal não negativa apesar do arredondamento. */
  for (int i = 0; i < 3; ++i) {
    if (a->P[i][i] < 0.0F) a->P[i][i] = 0.0F;
    for (int j = i + 1; j < 3; ++j) {
      const float m = 0.5F * (a->P[i][j] + a->P[j][i]);
      a->P[i][j] = a->P[j][i] = m;
    }
  }
}

/** @brief Adota a medida do estado @p k: valor medido e incerteza da medida, sem correlação. */
static void adopt(nav_ekf_axis_t *a, int k, float z, float R) {
  a->x[k] = z;
  for (int j = 0; j < 3; ++j) a->P[k][j] = a->P[j][k] = 0.0F;
  a->P[k][k] = R;
}

/** @brief Conta passos sem correção, saturando. */
static uint16_t age_step(uint16_t age) {
  return age < UINT16_MAX ? (uint16_t)(age + 1U) : age;
}

void nav_ekf_zero_position(nav_ekf_t *f) {
  for (int i = 0; i < 2; ++i) {
    f->h[i].x[0] = 0.0F;
    for (int j = 0; j < 3; ++j) f->h[i].P[0][j] = f->h[i].P[j][0] = 0.0F;
  }
}

void nav_ekf_predict(nav_ekf_t *f, float ax, float ay, float a_down, float dt,
                     const nav_ekf_config_t *cfg) {
  f->flow_age = age_step(f->flow_age);
  f->z_age = age_step(f->z_age);
  if (!(dt > 0.0F) || !isfinite(dt)) return;
  if (isfinite(ax) && isfinite(ay)) {
    predict_axis(&f->h[0], ax, dt, cfg->accel_noise, cfg->bias_noise);
    predict_axis(&f->h[1], ay, dt, cfg->accel_noise, cfg->bias_noise);
  }
  if (isfinite(a_down)) predict_axis(&f->z, a_down, dt, cfg->accel_noise_z, cfg->bias_noise);
}

bool nav_ekf_update_flow(nav_ekf_t *f, float vx, float vy, float range_m, float half_x,
                         float half_y, const nav_ekf_config_t *cfg) {
  if (!isfinite(vx) || !isfinite(vy) || !isfinite(range_m) || range_m <= 0.0F) return false;
  /* O mesmo erro angular vira um erro de velocidade maior quanto mais alto. A
     quantização soma variância uniforme (h²/3), sem zona morta: tratar a
     célula toda como inovação zero deixava o viés integrar livremente. */
  const float sigma = cfg->flow_noise_rad_s * range_m;
  const float z[2] = {vx, vy};
  const float half[2] = {fmaxf(half_x, 0.0F), fmaxf(half_y, 0.0F)};
  float R[2];
  for (int i = 0; i < 2; ++i) R[i] = sigma * sigma + half[i] * half[i] / 3.0F;
  if (!f->h_started) {
    for (int i = 0; i < 2; ++i) adopt(&f->h[i], 1, z[i], R[i]);
    f->h_started = true;
    f->flow_age = 0U;
    return true;
  }
  /* Gate no par: a medida é um vetor só. */
  float worst = 0.0F;
  for (int i = 0; i < 2; ++i) {
    const float S = innov_var(&f->h[i], 1, R[i]);
    f->flow_innov[i] = z[i] - f->h[i].x[1];
    const float ratio = S > 0.0F ? f->flow_innov[i] * f->flow_innov[i] / S : INFINITY;
    if (ratio > worst) worst = ratio;
  }
  const float g2 = cfg->gate * cfg->gate;
  f->flow_ratio = g2 > 0.0F ? worst / g2 : 0.0F;
  if (g2 > 0.0F && worst > g2) {
    ++f->flow_rejected;
    if (++f->flow_run < NAV_EKF_MAX_REJECT) return false;
    /* Discordância persistente: acreditar no sensor, não em nós mesmos. */
    for (int i = 0; i < 2; ++i) adopt(&f->h[i], 1, z[i], R[i]);
    f->flow_run = 0U;
    ++f->flow_resets;
    f->flow_age = 0U;
    return true;
  }
  f->flow_run = 0U;
  for (int i = 0; i < 2; ++i) fuse(&f->h[i], 1, z[i], R[i]);
  ++f->flow_updates;
  f->flow_age = 0U;
  return true;
}

bool nav_ekf_update_z(nav_ekf_t *f, float z_meas, const nav_ekf_config_t *cfg) {
  if (!isfinite(z_meas)) return false;
  const float R = cfg->range_noise * cfg->range_noise;
  if (!f->z_started) {
    adopt(&f->z, 0, z_meas, R);
    f->z_started = true;
    f->z_age = 0U;
    return true;
  }
  const float S = innov_var(&f->z, 0, R);
  f->z_innov = z_meas - f->z.x[0];
  const float ratio = S > 0.0F ? f->z_innov * f->z_innov / S : INFINITY;
  const float g2 = cfg->gate * cfg->gate;
  f->z_ratio = g2 > 0.0F ? ratio / g2 : 0.0F;
  if (g2 > 0.0F && ratio > g2) {
    ++f->z_rejected;
    if (++f->z_run < NAV_EKF_MAX_REJECT) return false;
    adopt(&f->z, 0, z_meas, R);
    f->z_run = 0U;
    ++f->z_resets;
    f->z_age = 0U;
    return true;
  }
  f->z_run = 0U;
  fuse(&f->z, 0, z_meas, R);
  ++f->z_updates;
  f->z_age = 0U;
  return true;
}

void nav_ekf_zupt(nav_ekf_t *f) {
  const float R = NAV_EKF_ZUPT_SIGMA * NAV_EKF_ZUPT_SIGMA;
  for (int i = 0; i < 2; ++i) {
    if (!f->h_started) {
      adopt(&f->h[i], 1, 0.0F, R);
    } else {
      fuse(&f->h[i], 1, 0.0F, R);
    }
  }
  f->h_started = true;
  f->flow_age = 0U;
}

bool nav_ekf_xy_inertial(const nav_ekf_t *f) {
  return f->h_started && f->flow_age <= NAV_EKF_INERTIAL_STEPS;
}

bool nav_ekf_xy_usable(const nav_ekf_t *f) {
  return f->h_started && f->flow_age <= NAV_EKF_FLOW_STALE_STEPS &&
         f->h[0].P[1][1] < NAV_EKF_CONVERGED_VEL_VAR && f->h[1].P[1][1] < NAV_EKF_CONVERGED_VEL_VAR;
}

bool nav_ekf_z_usable(const nav_ekf_t *f) {
  return f->z_started && f->z_age <= NAV_EKF_RANGE_STALE_STEPS;
}
