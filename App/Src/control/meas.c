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

  if (in->attitude_valid && isfinite(in->roll) && isfinite(in->pitch) &&
      isfinite(in->yaw)) {
    s->y[LQG_Y_PHI] = p->sgn_roll * in->roll;
    s->y[LQG_Y_THETA] = p->sgn_pitch * in->pitch;
    s->y[LQG_Y_PSI] = p->sgn_yaw * unwrap_yaw(s, in->yaw);
    flags |= MEAS_ATTITUDE;
  }

  if (in->gyro_valid && isfinite(in->gyro[0]) && isfinite(in->gyro[1]) &&
      isfinite(in->gyro[2])) {
    s->y[LQG_Y_P] = p->sgn_p * in->gyro[0];
    s->y[LQG_Y_Q] = p->sgn_q * in->gyro[1];
    s->y[LQG_Y_R] = p->sgn_r * in->gyro[2];
    flags |= MEAS_GYRO;
  }

  /* O telêmetro mede no próprio eixo; inclinado, ele lê a mais. A altura
     real é a distância vezes o cosseno da inclinação. */
  const bool tilt_known = (flags & MEAS_ATTITUDE) != 0U;
  if (in->range_valid && tilt_known && isfinite(in->range_m) && in->range_m > 0.0F) {
    s->y[LQG_Y_Z] = -in->range_m * attitude_cos_tilt(in->q);
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
