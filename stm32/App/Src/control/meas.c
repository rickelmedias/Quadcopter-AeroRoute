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
  nav_ekf_reset(&s->nav);
}

void meas_rearm(meas_state_t *s) {
  memset(s->y, 0, sizeof(s->y));
  nav_ekf_rearm(&s->nav);
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

/** Degrau do fluxo do MTF-01, em rad/s (cm/s a 1 m em múltiplos de 20). */
#define FLOW_LSB_RAD_S 0.2F
/** Qualidade que liga o fluxo (ArduPilot: QUALITY_HIGH). */
#define FLOW_QUALITY_HIGH 30U
/** Qualidade abaixo da qual o fluxo desliga (histerese). */
#define FLOW_QUALITY_LOW 15U
/** Cosseno mínimo da inclinação (~37°); além disso a distância deixa de ser a altura. */
#define FLOW_MIN_COS_TILT 0.80F
/** Fluxo máximo aceito, em rad/s (mesmo teto do ArduPilot). */
#define FLOW_MAX_RAD_S 4.2F

/** @brief Limita @p v a [@p lo, @p hi]. */
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/**
 * @brief Filtro de plausibilidade do fluxo (FLOW_GATE), com histerese na qualidade.
 * @return true se o fluxo deste passo pode ser usado (sempre true com FLOW_GATE = 0).
 */
static bool flow_gate_ok(meas_state_t *s, const meas_input_t *in, const ctrl_params_t *p,
                         euler_t rel) {
  if (p->flow_gate < 0.5F) return true;
  if (s->flow_quality_ok) {
    if (in->flow_quality <= FLOW_QUALITY_LOW) s->flow_quality_ok = false;
  } else if (in->flow_quality >= FLOW_QUALITY_HIGH) {
    s->flow_quality_ok = true;
  }
  const float cos_tilt = cosf(rel.roll) * cosf(rel.pitch);
  return s->flow_quality_ok && cos_tilt >= FLOW_MIN_COS_TILT &&
         fabsf(in->flow_x) <= FLOW_MAX_RAD_S && fabsf(in->flow_y) <= FLOW_MAX_RAD_S;
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
    s->y[LQG_Y_PHI] = p->sgn_roll * rel.pitch + p->lvl_phi_deg * DEG_TO_RAD;
    s->y[LQG_Y_THETA] = p->sgn_pitch * rel.roll + p->lvl_theta_deg * DEG_TO_RAD;
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
  const float cos_tilt = tilt_known ? cosf(rel.roll) * cosf(rel.pitch) : 1.0F;
  const bool range_ok = in->range_valid && tilt_known && isfinite(in->range_m) &&
                        in->range_m > 0.0F;
  if (range_ok) {
    s->y[LQG_Y_Z] = -in->range_m * cos_tilt;
    flags |= MEAS_RANGE;
  }

  /* EKF de navegação (control/nav_ekf.h): roda sempre, como diagnóstico; o
     LQG só recebe o que ele estima com FLOW_EKF (vx, vy) e Z_EKF (z). A
     predição usa a aceleração linear do BNO a cada passo. O BNO está girado em
     relação ao corpo, e o mesmo mapa dos giros vale para a aceleração: frente
     = sinal de p × eixo y, direita = sinal de q × eixo x, para baixo = sinal
     de r × eixo z (conferido com o voo capturado em 08/11). */
  const nav_ekf_config_t ncfg = {p->fekf_acc_noise,   p->fekf_accz_noise, p->fekf_bias_noise,
                                 p->fekf_flow_noise,  p->fekf_range_noise, p->fekf_gate};
  const float ang = p->sgn_yaw * s->yaw_unwrapped;
  s->diag.flags = 0U;
  if (in->accel_valid && tilt_known && (flags & MEAS_GYRO) != 0U) {
    float ax, ay;
    attitude_yaw_rotate(ang, p->sgn_p * in->lin_accel[1], p->sgn_q * in->lin_accel[0], &ax, &ay);
    s->diag.accel[0] = ax;
    s->diag.accel[1] = ay;
    s->diag.accel[2] = p->sgn_r * in->lin_accel[2];
    nav_ekf_predict(&s->nav, ax, ay, s->diag.accel[2], (float)LQG_TS_S, &ncfg);
  }
  /* No chão a velocidade é zero (ZUPT): o EKF chega ao ar já iniciado e sem viés de velocidade. */
  if (in->on_ground && p->flow_ekf >= 0.5F) nav_ekf_zupt(&s->nav);
  /* As correções só usam quadro novo do MTF (100 Hz, sem relação fixa com o
     passo de 10 ms): repetir um quadro contaria a mesma medida duas vezes. */
  const bool new_frame = in->mtf_frame != s->last_mtf_frame;
  float frame_dt = (float)LQG_TS_S;
  if (new_frame) {
    const uint32_t gap = in->mtf_time_ms - s->last_mtf_time_ms;
    if (s->last_mtf_frame != 0U && gap > 0U && gap < 250U) frame_dt = (float)gap * 0.001F;
    s->last_mtf_frame = in->mtf_frame;
    s->last_mtf_time_ms = in->mtf_time_ms;
    s->diag.flags |= MEAS_DIAG_NEW_FRAME;
    s->diag.range_raw = in->range_m;
    s->diag.quality = in->flow_quality;
    if (tilt_known && isfinite(in->range_m)) {
      float filtered;
      if (range_filter_update(&s->nav.range, in->range_m, frame_dt, in->range_valid,
                              NAV_MIN_RANGE_M, &filtered)) {
        s->diag.range_filt = filtered;
        if (!nav_ekf_update_z(&s->nav, -filtered * cos_tilt, &ncfg)) {
          s->diag.flags |= MEAS_DIAG_Z_REJECTED;
        }
      } else {
        s->diag.flags |= MEAS_DIAG_RANGE_REJECTED;
      }
    }
  }
  if (nav_ekf_z_usable(&s->nav)) s->diag.flags |= MEAS_DIAG_Z_USABLE;
  if (p->z_ekf >= 0.5F && range_ok && nav_ekf_z_usable(&s->nav)) {
    s->y[LQG_Y_Z] = s->nav.z.x[0];
    s->diag.flags |= MEAS_DIAG_Z_TO_LQG;
  }

  /* Fluxo óptico -> velocidade. A rotação do corpo também varre a imagem;
     somar o giro (com o ganho e o sinal medidos na bancada) deixa só a parte
     que a translação produziu. Taxa angular vezes distância é velocidade
     linear nos eixos do corpo; a guinada leva para o referencial local. */
  /* Com FLOW_EKF, perto do piso (MTF a menos de NAV_FLOW_MIN_RANGE_M) o fluxo é ruído: vale a
     inércia do EKF (mais abaixo), não o fluxo. */
  const bool flow_range_ok = p->flow_ekf < 0.5F || in->range_m >= NAV_FLOW_MIN_RANGE_M;
  if (in->flow_valid && flow_range_ok && (flags & MEAS_RANGE) != 0U && (flags & MEAS_GYRO) != 0U &&
      isfinite(in->flow_x) && isfinite(in->flow_y) && flow_gate_ok(s, in, p, rel)) {
    /* O MTF-01 só reporta degraus de FLOW_LSB_RAD_S; contagem zero quer dizer
       "dentro de meio degrau", não "parado". */
    const float zero_x = in->flow_x;
    const float zero_y = in->flow_y;
    const float flow_x = in->flow_x - p->flow_bias_x;
    const float flow_y = in->flow_y - p->flow_bias_y;
    /* Compensação do giro: o ganho "direto" (X com o giro x, Y com o y) e o
       cruzado (X com o giro y, Y com o x). O voo de 08/11 mostrou o cruzado. */
    const float trans_x = p->flow_scale_x * flow_x + p->flow_gyro_x * in->gyro[0] +
                          p->flow_gyro_xy * in->gyro[1];
    const float trans_y = p->flow_scale_y * flow_y + p->flow_gyro_y * in->gyro[1] +
                          p->flow_gyro_yx * in->gyro[0];
    /* Os sinais levam o fluxo aos eixos do corpo do modelo (frente, direita),
       medidos com a guinada em zero; a rotação para o local usa ψ do modelo
       (horário visto de cima, = sgn_yaw × guinada do sensor). Girar pela
       guinada do sensor, como antes de 06/11, espelhava o y longe do zero. */
    /* Com o MTF fora do centro, a guinada soma r × distância (FLOW_YAW_X/Y). */
    const float r = p->sgn_r * in->gyro[2];
    /* Com FLOW_SWAP o MTF está girado em relação ao corpo: o canal Y é a
       frente e o X aponta para a esquerda (direita = -X). */
    const bool swap = p->flow_swap >= 0.5F;
    const float fwd = swap ? trans_y : trans_x;
    const float right = swap ? -trans_x : trans_y;
    float body_x = p->sgn_vx * fwd * in->range_m - p->flow_yaw_x * r;
    float body_y = p->sgn_vy * right * in->range_m - p->flow_yaw_y * r;
    const float sc_f = fabsf(swap ? p->flow_scale_y : p->flow_scale_x);
    const float sc_r = fabsf(swap ? p->flow_scale_x : p->flow_scale_y);
    const bool zero_f = fabsf(swap ? zero_y : zero_x) < 0.5F * FLOW_LSB_RAD_S;
    const bool zero_r = fabsf(swap ? zero_x : zero_y) < 0.5F * FLOW_LSB_RAD_S;
    const float hw_f = zero_f ? 0.5F * FLOW_LSB_RAD_S * sc_f * in->range_m : 0.0F;
    const float hw_r = zero_r ? 0.5F * FLOW_LSB_RAD_S * sc_r * in->range_m : 0.0F;
    s->diag.flow_raw[0] = in->flow_x;
    s->diag.flow_raw[1] = in->flow_y;
    s->diag.flow_body[0] = body_x;
    s->diag.flow_body[1] = body_y;
    if (new_frame) {
      /* O EKF recebe a medida crua (sem censura), girada para o local; a
         célula de quantização vai junto, como variância. */
      float vx_raw, vy_raw;
      attitude_yaw_rotate(ang, body_x, body_y, &vx_raw, &vy_raw);
      const float c = fabsf(cosf(ang)), sn = fabsf(sinf(ang));
      if (nav_ekf_update_flow(&s->nav, vx_raw, vy_raw, in->range_m, c * hw_f + sn * hw_r,
                              sn * hw_f + c * hw_r, &ncfg)) {
        s->diag.flags |= MEAS_DIAG_FLOW_FUSED;
      } else {
        s->diag.flags |= MEAS_DIAG_FLOW_REJECTED;
      }
    }
    if (p->flow_cens >= 0.5F && isfinite(in->pred_vx) && isfinite(in->pred_vy)) {
      /* Correção censurada: onde o canal leu zero, a velocidade verdadeira só
         está na faixa de meio degrau em torno do que o giro e o viés explicam.
         A predição dentro dela não gera inovação; fora, é puxada para a borda. */
      float pbx, pby;
      attitude_yaw_rotate(-ang, in->pred_vx, in->pred_vy, &pbx, &pby);
      const float t0_x = p->flow_scale_x * (-p->flow_bias_x) + p->flow_gyro_x * in->gyro[0] +
                         p->flow_gyro_xy * in->gyro[1];
      const float t0_y = p->flow_scale_y * (-p->flow_bias_y) + p->flow_gyro_y * in->gyro[1] +
                         p->flow_gyro_yx * in->gyro[0];
      const float f0 = swap ? t0_y : t0_x;
      const float r0 = swap ? -t0_x : t0_y;
      if (zero_f) {
        const float cc = p->sgn_vx * f0 * in->range_m - p->flow_yaw_x * r;
        body_x = clampf(pbx, cc - hw_f, cc + hw_f);
      }
      if (zero_r) {
        const float cc = p->sgn_vy * r0 * in->range_m - p->flow_yaw_y * r;
        body_y = clampf(pby, cc - hw_r, cc + hw_r);
      }
    }
    float vx, vy;
    attitude_yaw_rotate(ang, body_x, body_y, &vx, &vy);
    if (p->flow_ekf >= 0.5F && nav_ekf_xy_usable(&s->nav)) {
      /* FLOW_EKF: o LQG recebe a velocidade do EKF (fluxo + acelerômetro). */
      vx = s->nav.h[0].x[1];
      vy = s->nav.h[1].x[1];
      s->diag.flags |= MEAS_DIAG_XY_TO_LQG;
    }
    s->y[LQG_Y_VX] = vx;
    s->y[LQG_Y_VY] = vy;
    flags |= MEAS_FLOW;
  }
  if (!(flags & MEAS_FLOW) && p->flow_ekf >= 0.5F && nav_ekf_xy_inertial(&s->nav) &&
      (flags & MEAS_ATTITUDE) != 0U) {
    /* Sem fluxo (MTF muito perto do piso na subida, ou quadro inválido): a velocidade
       vem do EKF, que segue pela aceleração do BNO por até NAV_EKF_INERTIAL_STEPS. */
    s->y[LQG_Y_VX] = s->nav.h[0].x[1];
    s->y[LQG_Y_VY] = s->nav.h[1].x[1];
    s->diag.flags |= MEAS_DIAG_XY_TO_LQG;
    flags |= MEAS_FLOW;
  }
  if (nav_ekf_xy_usable(&s->nav)) s->diag.flags |= MEAS_DIAG_XY_USABLE;

  for (unsigned i = 0U; i < LQG_NY; ++i) {
    y[i] = (lqg_real_t)s->y[i];
  }
  return flags;
}
