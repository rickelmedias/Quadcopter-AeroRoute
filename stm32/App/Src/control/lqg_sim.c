/**
 * @file lqg_sim.c
 * @brief Planta simulada do modo FLT_SIM (contrato em control/lqg_sim.h).
 */
#include "control/lqg_sim.h"

#include "control/lqg_config.h"
#include "control/lqg_ctrl.h"
#include "control/lqg_gains.h"
#include "control/quat.h"
#include "lqg_model/STM32.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265F

static const uint8_t MOTOR_OF_INPUT[4] = LQG_MOTOR_OF_INPUT;

static float wrap_pi(float a) {
  float r = a;
  while (r > PI_F) {
    r -= 2.0F * PI_F;
  }
  while (r < -PI_F) {
    r += 2.0F * PI_F;
  }
  return r;
}

void lqg_sim_init(lqg_sim_t *s, float hover_permille) {
  (void)memset(s, 0, sizeof(*s));
  s->hover_permille = hover_permille;
}

void lqg_sim_sensors(lqg_sim_t *s, lqg_sensors_t *in) {
  const float phi = (float)s->x[3];
  const float theta = (float)s->x[4];
  const float psi = (float)s->x[5];
  const float deg = PI_F / 180.0F;
  (void)memset(in, 0, sizeof(*in));
  in->pitch = (phi / LQG_SGN_ROLL) + (LQG_ZERO_PITCH_DEG * deg);
  in->roll = wrap_pi((theta / LQG_SGN_PITCH) + (LQG_ZERO_ROLL_DEG * deg));
  in->yaw = wrap_pi(psi / LQG_SGN_YAW);
  quat_from_euler(in->roll, in->pitch, in->yaw, in->q);
  in->q_valid = true;
  in->att_accuracy = 3U;
  in->gyro[1] = (float)s->x[9] / LQG_SGN_P;
  in->gyro[0] = (float)s->x[10] / LQG_SGN_Q;
  in->gyro[2] = (float)s->x[11] / LQG_SGN_R;
  in->attitude_valid = true;
  in->gyro_valid = true;
  /* Aceleração linear (sem gravidade) no corpo, nos eixos do sensor (mapa inverso do giro). */
  float q[4];
  quat_from_euler(phi, theta, psi, q);
  const float an[3] = {(float)s->acc[0], (float)s->acc[1], (float)s->acc[2]};
  float ab[3];
  quat_rotate_inv(q, an, ab);
  in->accel[1] = ab[0] / LQG_SGN_P;
  in->accel[0] = ab[1] / LQG_SGN_Q;
  in->accel[2] = ab[2] / LQG_SGN_R;
  in->accel_valid = true;
  const float h = -(float)s->x[2];
  const float lever =
      (LQG_RANGE_POS_X_M * sinf(theta)) - (LQG_RANGE_POS_Y_M * sinf(phi) * cosf(theta));
  in->range_m = (LQG_SIM_GROUND_RANGE_M + h + lever) / (cosf(phi) * cosf(theta));
  in->range_valid = true;
  /* O fluxo mede a velocidade no corpo: gira a velocidade local por -psi e desfaz
     o mapa de lqg_meas (escala, compensação do giro e sinais). */
  /* O sensor fora do centro também anda quando o drone gira (r × braço). */
  const float r = (float)s->x[11];
  const float u =
      (cosf(psi) * (float)s->x[6]) + (sinf(psi) * (float)s->x[7]) + (LQG_FLOW_YAW_X * r);
  const float v =
      (-sinf(psi) * (float)s->x[6]) + (cosf(psi) * (float)s->x[7]) + (LQG_FLOW_YAW_Y * r);
  in->flow_x =
      ((-v / (LQG_SGN_VY * in->range_m)) - (LQG_FLOW_GYRO_X * in->gyro[0])) / LQG_FLOW_SCALE_X;
  in->flow_y =
      ((u / (LQG_SGN_VX * in->range_m)) - (LQG_FLOW_GYRO_Y * in->gyro[1])) / LQG_FLOW_SCALE_Y;
  in->flow_valid = true;
  in->flow_quality = 100U;
  ++s->frames;
  in->mtf_frames = s->frames;
}

void lqg_sim_step(lqg_sim_t *s, const float permille[4]) {
  double du[4];
  /* CG fora do centro: cada motor precisa de hover + o trim da tabela, proporcional ao empuxo
     (a média comandada sobre o hover), como o controlador compensa em lqg_ctrl.c. */
  const float mean = 0.25F * (permille[0] + permille[1] + permille[2] + permille[3]);
  const float scale =
      (LQG_CG_TRIM_ENABLE != 0) ? lqg_cg_trim_scale(mean) : 0.0F;
  for (uint32_t i = 0U; i < 4U; ++i) {
    const float need = s->hover_permille + (LQG_HOVER_TRIM_PERMILLE[i] * scale);
    du[i] = (double)((permille[MOTOR_OF_INPUT[i]] - need) * LQG_W_PER_PERMILLE);
  }
  double next[12];
  for (uint32_t r = 0U; r < 12U; ++r) {
    double v = 0.0;
    for (uint32_t c = 0U; c < 12U; ++c) {
      v += STM32_ConstP.A_Value[(12U * c) + r] * s->x[c];
    }
    for (uint32_t k = 0U; k < 4U; ++k) {
      v += STM32_ConstP.B_Value[r + (12U * k)] * du[k];
    }
    next[r] = v;
  }
  /* A gravidade inclinada acelera na direção da inclinação girada por psi (o modelo
     linear supõe psi = 0). */
  const double ts = (double)LQG_TS_S;
  const double g = 9.81;
  const double c = cos(s->x[5]);
  const double sn = sin(s->x[5]);
  const double a_n = -g * ((s->x[4] * c) + (s->x[3] * sn));
  const double a_e = g * ((s->x[3] * c) - (s->x[4] * sn));
  next[6] = s->x[6] + (ts * a_n);
  next[7] = s->x[7] + (ts * a_e);
  next[0] = s->x[0] + (ts * s->x[6]) + (0.5 * ts * ts * a_n);
  next[1] = s->x[1] + (ts * s->x[7]) + (0.5 * ts * ts * a_e);
  for (uint32_t i = 0U; i < 3U; ++i) {
    s->acc[i] = (next[6U + i] - s->x[6U + i]) / ts;
  }
  (void)memcpy(s->x, next, sizeof(next));
  if (s->x[2] >= 0.0) { /* chão: z NED não passa de zero */
    s->x[2] = 0.0;
    if (s->x[8] > 0.0) {
      s->x[8] = 0.0;
    }
    if (!s->lifted) {
      for (uint32_t i = 3U; i < 12U; ++i) {
        if (i != 8U) {
          s->x[i] = 0.0; /* apoiado: sem rotação nem translação */
        }
      }
    }
  } else {
    s->lifted = true;
  }
  if (!s->lifted) {
    s->acc[0] = 0.0;
    s->acc[1] = 0.0;
    s->acc[2] = 0.0;
  }
}
