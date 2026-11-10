/**
 * @file meas.c
 * @brief Leituras dos sensores -> medidas do modelo (contrato em control/meas.h).
 */
#include "control/meas.h"

#include "attitude.h"

#include <math.h>
#include <string.h>

void meas_reset(meas_state_t *s) {
  memset(s, 0, sizeof(*s));
}

void meas_rearm(meas_state_t *s) {
  memset(s->y, 0, sizeof(s->y));
}

/** Graus para radianos. */
#define DEG_TO_RAD 0.017453293F

/** @brief Guinada relativa à origem, sem o salto de ±π. */
static float unwrap_yaw(meas_state_t *s, float yaw) {
  if (!s->yaw_ready) {
    s->yaw_origin = yaw;
    s->yaw_last = yaw;
    s->yaw_unwrapped = 0.0F;
    s->yaw_ready = true;
  }
  s->yaw_unwrapped += attitude_wrap_pi(yaw - s->yaw_last);
  s->yaw_last = yaw;
  return s->yaw_unwrapped;
}

uint8_t meas_update(meas_state_t *s, const meas_input_t *in, const ctrl_params_t *p,
                    lqg_real_t y[LQG_NY]) {
  uint8_t flags = 0U;
  euler_t rel = {0.0F, 0.0F, 0.0F};

  if (in->attitude_valid && isfinite(in->roll) && isfinite(in->pitch) &&
      isfinite(in->yaw)) {
    /* O BNO está montado invertido (roll cru ~ -175 graus nivelado): roll e
       pitch entram no modelo relativos ao zero de nível. */
    const euler_t zero = {p->zero_roll_deg * DEG_TO_RAD, p->zero_pitch_deg * DEG_TO_RAD, 0.0F};
    const euler_t raw = {in->roll, in->pitch, in->yaw};
    attitude_euler_relative(&zero, &raw, &rel);
    /* O BNO está girado 90 graus em torno da vertical em relação ao drone:
       o roll dele é a arfagem θ do modelo e o pitch dele é a rolagem φ. */
    s->y[LQG_Y_PHI] = p->sgn_roll * rel.pitch;
    s->y[LQG_Y_THETA] = p->sgn_pitch * rel.roll;
    s->y[LQG_Y_PSI] = p->sgn_yaw * unwrap_yaw(s, in->yaw);
    flags |= MEAS_ATTITUDE;
  }

  if (in->gyro_valid && isfinite(in->gyro[0]) && isfinite(in->gyro[1]) &&
      isfinite(in->gyro[2])) {
    /* Mesmo mapa de eixos dos ângulos: p vem do giro em torno do eixo y do
       BNO e q do giro em torno do x. */
    s->y[LQG_Y_P] = p->sgn_p * in->gyro[1];
    s->y[LQG_Y_Q] = p->sgn_q * in->gyro[0];
    s->y[LQG_Y_R] = p->sgn_r * in->gyro[2];
    flags |= MEAS_GYRO;
  }

  /* O telêmetro mede no próprio eixo; inclinado, ele lê a mais. A altura
     real é a distância vezes o cosseno da inclinação (termo 3,3 da rotação
     em ZYX: cos(roll) * cos(pitch), já relativos ao zero). O quatérnio não
     serve aqui: com o BNO invertido o cosseno dele seria ~ -1. */
  const bool tilt_known = (flags & MEAS_ATTITUDE) != 0U;
  if (in->range_valid && tilt_known && isfinite(in->range_m) && in->range_m > 0.0F) {
    s->y[LQG_Y_Z] = -in->range_m * cosf(rel.roll) * cosf(rel.pitch);
    flags |= MEAS_RANGE;
  }

  /* Fluxo óptico -> velocidade. A rotação do corpo também varre a imagem;
     somar o giro (com o ganho e o sinal medidos na bancada) deixa só a parte
     que a translação produziu. Taxa angular vezes distância é velocidade
     linear nos eixos do corpo; a guinada leva para o referencial local. */
  if (in->flow_valid && (flags & MEAS_RANGE) != 0U && (flags & MEAS_GYRO) != 0U &&
      isfinite(in->flow_x) && isfinite(in->flow_y)) {
    const float trans_x = p->flow_scale_x * in->flow_x + p->flow_gyro_x * in->gyro[0];
    const float trans_y = p->flow_scale_y * in->flow_y + p->flow_gyro_y * in->gyro[1];
    float vx, vy;
    attitude_yaw_rotate(s->yaw_unwrapped, trans_x * in->range_m, trans_y * in->range_m,
                        &vx, &vy);
    s->y[LQG_Y_VX] = p->sgn_vx * vx;
    s->y[LQG_Y_VY] = p->sgn_vy * vy;
    flags |= MEAS_FLOW;
  }

  for (unsigned i = 0U; i < LQG_NY; ++i) {
    y[i] = (lqg_real_t)s->y[i];
  }
  return flags;
}
