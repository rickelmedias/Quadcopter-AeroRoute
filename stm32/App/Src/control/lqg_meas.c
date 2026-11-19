/**
 * @file lqg_meas.c
 * @brief Leituras dos sensores -> medidas do modelo (contrato em control/lqg_meas.h).
 */
#include "control/lqg_meas.h"

#include "attitude.h"
#include "control/lqg_config.h"
#include "control/quat.h"

#include <math.h>
#include <string.h>

/** Graus para radianos. */
#define DEG_TO_RAD 0.017453293F

_Static_assert(LQG_FLOW_GYRO_AVG_STEPS <= LQG_GYRO_HISTORY, "historico de giro curto");

static const float Q_WORLD[4] = LQG_Q_WORLD;
static const float Q_MOUNT[4] = LQG_Q_MOUNT;

/** @brief Inclinação do corpo (sem guinada) de @p q: conj(guinada) ⊗ q. */
static void tilt_of(const float q[4], float tilt[4]) {
  float qy[4];
  float qyi[4];
  quat_from_yaw(quat_yaw(q), qy);
  quat_conj(qy, qyi);
  quat_mul(qyi, q, tilt);
}

/** @brief q_bno (sensor -> mundo ENU do BNO) em corpo -> NED, sem nível nem guinada de referência. */
static void body_from_bno(const float qs[4], float qb[4]) {
  float t[4];
  quat_mul(Q_WORLD, qs, t);
  quat_mul(t, Q_MOUNT, qb);
}

void lqg_meas_reset(lqg_meas_t *m) {
  (void)memset(m, 0, sizeof(*m));
}

/** @brief Constantes do caminho em quatérnio, na primeira vez (a struct pode ter sido zerada por fora). */
static void init_q(lqg_meas_t *m) {
  quat_identity(m->q_yaw0_inv);
  quat_identity(m->q_level_inv);
  /* A pose de nível constante da submissão: mesma conversão, inclinação descontada à direita. */
  float qs[4];
  float qb[4];
  float tilt[4];
  const float deg = 0.017453293F;
  quat_from_euler(LQG_ZERO_ROLL_DEG * deg, LQG_ZERO_PITCH_DEG * deg, 0.0F, qs);
  body_from_bno(qs, qb);
  tilt_of(qb, tilt);
  quat_conj(tilt, m->q_zero_inv);
  m->q_init = true;
}

void lqg_meas_set_level_q(lqg_meas_t *m, const float rv[3]) {
  if (!m->q_init) {
    init_q(m);
  }
  float q[4];
  quat_from_rotvec(rv, q);
  quat_conj(q, m->q_level_inv);
}

/**
 * @brief Quatérnio do BNO -> corpo relativo ao armamento (guinada) e ao nível, direto, sem Euler.
 * @return false com quatérnio fora da esfera unitária ou não finito.
 */
static bool attitude_from_bno(lqg_meas_t *m, const float qs_in[4], lqg_meas_out_t *out) {
  float qs[4] = {qs_in[0], qs_in[1], qs_in[2], qs_in[3]};
  const float n2 = (qs[0] * qs[0]) + (qs[1] * qs[1]) + (qs[2] * qs[2]) + (qs[3] * qs[3]);
  if ((isfinite(n2) == 0) || (n2 < 0.64F) || (n2 > 1.44F)) {
    return false;
  }
  quat_normalize(qs);
  if (!m->q_init) {
    init_q(m);
  }
  float qb[4];
  float qc[4];
  body_from_bno(qs, qb);
  quat_mul(qb, m->q_zero_inv, qc);
  if (!m->q_ready) {
    float y[4];
    quat_from_yaw(quat_yaw(qc), y);
    quat_conj(y, m->q_yaw0_inv);
    m->q_ready = true;
  }
  float rel[4];
  quat_mul(m->q_yaw0_inv, qc, rel);
  float tilt[4];
  tilt_of(rel, tilt);
  quat_to_rotvec(tilt, out->tilt_raw);
  quat_mul(rel, m->q_level_inv, out->q);
  quat_normalize(out->q);
  out->q_native = true;
  return true;
}

void lqg_meas_set_ground(lqg_meas_t *m, float ground_m) {
  m->ground_m = ground_m;
}

/** @brief Guinada desde a primeira leitura, sem o salto de ±pi. */
static float unwrap_yaw(lqg_meas_t *m, float yaw) {
  if (!m->yaw_ready) {
    m->yaw_last = yaw;
    m->yaw_unwrapped = 0.0F;
    m->yaw_ready = true;
  }
  m->yaw_unwrapped += attitude_wrap_pi(yaw - m->yaw_last);
  m->yaw_last = yaw;
  return m->yaw_unwrapped;
}

void lqg_meas_set_level(lqg_meas_t *m, float phi, float theta) {
  m->level_phi = phi;
  m->level_theta = theta;
}

/** @brief Guarda o giro x, y deste passo e devolve a média dos últimos ::LQG_FLOW_GYRO_AVG_STEPS. */
static void gyro_average(lqg_meas_t *m, const float gyro[3], float avg[2]) {
  const uint32_t slot = m->gyro_n % LQG_GYRO_HISTORY;
  m->gyro_hist[slot][0] = gyro[0];
  m->gyro_hist[slot][1] = gyro[1];
  ++m->gyro_n;
  const uint32_t n = (m->gyro_n < LQG_FLOW_GYRO_AVG_STEPS) ? m->gyro_n : LQG_FLOW_GYRO_AVG_STEPS;
  avg[0] = 0.0F;
  avg[1] = 0.0F;
  for (uint32_t k = 0U; k < n; ++k) {
    const uint32_t i = (m->gyro_n - 1U - k) % LQG_GYRO_HISTORY;
    avg[0] += m->gyro_hist[i][0];
    avg[1] += m->gyro_hist[i][1];
  }
  avg[0] /= (float)n;
  avg[1] /= (float)n;
}

void lqg_meas_update(lqg_meas_t *m, const lqg_sensors_t *in, lqg_meas_out_t *out) {
  (void)memset(out, 0, sizeof(*out));
  euler_t rel = {0.0F, 0.0F, 0.0F};

  if (in->attitude_valid && (isfinite(in->roll) != 0) && (isfinite(in->pitch) != 0) &&
      (isfinite(in->yaw) != 0)) {
    /* BNO invertido: roll e pitch entram relativos ao zero de nível. */
    const euler_t zero = {LQG_ZERO_ROLL_DEG * DEG_TO_RAD, LQG_ZERO_PITCH_DEG * DEG_TO_RAD, 0.0F};
    const euler_t raw = {in->roll, in->pitch, in->yaw};
    attitude_euler_relative(&zero, &raw, &rel);
    /* BNO girado 90 graus: o pitch do sensor é φ e o roll do sensor é θ. */
    out->y[LQG_Y_PHI] = (LQG_SGN_ROLL * rel.pitch) - m->level_phi;
    out->y[LQG_Y_THETA] = (LQG_SGN_PITCH * rel.roll) - m->level_theta;
    out->y[LQG_Y_PSI] = LQG_SGN_YAW * unwrap_yaw(m, in->yaw);
    out->flags |= LQG_MEAS_ATTITUDE;
    /* O EKF3 usa o quatérnio do BNO direto; sem ele, o monta dos ângulos acima. */
    if (!in->q_valid || !attitude_from_bno(m, in->q, out)) {
      quat_from_euler(out->y[LQG_Y_PHI], out->y[LQG_Y_THETA], out->y[LQG_Y_PSI], out->q);
    }
  } else {
    quat_identity(out->q);
  }
  out->att_accuracy = out->q_native ? ((in->att_accuracy < 3U) ? in->att_accuracy : 3U) : 3U;

  if (in->accel_valid && (isfinite(in->accel[0]) != 0) && (isfinite(in->accel[1]) != 0) &&
      (isfinite(in->accel[2]) != 0)) {
    /* Mesmo tripé do giroscópio: a aceleração segue o mapa de p, q, r. */
    out->accel_body[0] = LQG_SGN_P * in->accel[1];
    out->accel_body[1] = LQG_SGN_Q * in->accel[0];
    out->accel_body[2] = LQG_SGN_R * in->accel[2];
    out->flags |= LQG_MEAS_ACCEL;
  }

  float gyro_avg[2] = {0.0F, 0.0F};
  if (in->gyro_valid && (isfinite(in->gyro[0]) != 0) && (isfinite(in->gyro[1]) != 0) &&
      (isfinite(in->gyro[2]) != 0)) {
    out->y[LQG_Y_P] = LQG_SGN_P * in->gyro[1];
    out->y[LQG_Y_Q] = LQG_SGN_Q * in->gyro[0];
    out->y[LQG_Y_R] = LQG_SGN_R * in->gyro[2];
    out->flags |= LQG_MEAS_GYRO;
    gyro_average(m, in->gyro, gyro_avg);
  }

  /* Quadro novo do MTF-01: a malha e o sensor rodam a 100 Hz sem sincronia, então
     um quadro pode aparecer em dois passos. O filtro só deve usá-lo uma vez. */
  if ((in->mtf_frames == 0U) || (in->mtf_frames != m->last_frames)) {
    out->flags |= LQG_MEAS_FRESH;
    m->last_frames = in->mtf_frames;
  }

  /* Inclinado, o telêmetro lê a mais: altura = distância * cos(roll) * cos(pitch),
     com roll e pitch já relativos ao zero. O quatérnio não serve aqui, o cosseno
     dele seria ~ -1 com o BNO invertido. O sensor fora do centro também sobe ou
     desce com a inclinação: a altura do centro desconta esse braço. */
  if (((out->flags & LQG_MEAS_ATTITUDE) != 0U) && in->range_valid && (isfinite(in->range_m) != 0) &&
      (in->range_m > 0.0F)) {
    const float phi = out->y[LQG_Y_PHI];
    const float theta = out->y[LQG_Y_THETA];
    const float lever =
        (LQG_RANGE_POS_X_M * sinf(theta)) - (LQG_RANGE_POS_Y_M * sinf(phi) * cosf(theta));
    out->range_corrected_m = (in->range_m * cosf(rel.roll) * cosf(rel.pitch)) - lever;
    out->y[LQG_Y_Z] = -(out->range_corrected_m - m->ground_m);
    out->flags |= LQG_MEAS_RANGE;
    const float h = out->range_corrected_m - m->ground_m;
    if (h >= LQG_FLOW_MIN_HEIGHT_M) {
      m->flow_height_ok = true;
    } else if (h < LQG_FLOW_OFF_HEIGHT_M) {
      m->flow_height_ok = false;
    } else {
      /* histerese: mantém */
    }
  } else {
    m->flow_height_ok = false;
  }
  if (m->flow_height_ok) {
    out->flags |= LQG_MEAS_FLOW_HEIGHT;
  }

  /* Fluxo -> velocidade. A rotação do corpo também varre a imagem: somar o giro
     (ganho medido na bancada) deixa só a translação. O giro é a média dos últimos
     passos, mais perto do intervalo que o quadro do MTF integrou. Os eixos do corpo
     (frente, direita) giram para o referencial local pela guinada do modelo. Só
     vale acima de ::LQG_FLOW_MIN_HEIGHT_M e uma vez por quadro. */
  if (in->flow_valid && (in->flow_quality >= LQG_FLOW_MIN_QUALITY) &&
      ((out->flags & LQG_MEAS_RANGE) != 0U) && ((out->flags & LQG_MEAS_GYRO) != 0U) &&
      ((out->flags & LQG_MEAS_FRESH) != 0U) && m->flow_height_ok &&
      (isfinite(in->flow_x) != 0) && (isfinite(in->flow_y) != 0)) {
    const float trans_x = (LQG_FLOW_SCALE_X * in->flow_x) + (LQG_FLOW_GYRO_X * gyro_avg[0]);
    const float trans_y = (LQG_FLOW_SCALE_Y * in->flow_y) + (LQG_FLOW_GYRO_Y * gyro_avg[1]);
    const float r = out->y[LQG_Y_R];
    /* Ensaio de translação: Y positivo = frente; X negativo = direita.
       Aplicar escala e compensação nos canais do sensor antes de girá-los. */
    const float body_x = (LQG_SGN_VX * trans_y * in->range_m) - (LQG_FLOW_YAW_X * r);
    const float body_y = (-LQG_SGN_VY * trans_x * in->range_m) - (LQG_FLOW_YAW_Y * r);
    out->flow_body[0] = body_x;
    out->flow_body[1] = body_y;
    float vx = 0.0F;
    float vy = 0.0F;
    attitude_yaw_rotate(LQG_SGN_YAW * m->yaw_unwrapped, body_x, body_y, &vx, &vy);
    out->y[LQG_Y_U] = vx;
    out->y[LQG_Y_V] = vy;
    out->flags |= LQG_MEAS_FLOW;
  }
}
