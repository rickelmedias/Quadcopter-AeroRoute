/**
 * @file lqg_ctrl.c
 * @brief Controle LQR + Kalman do modo com cabo (contrato em control/lqg_ctrl.h).
 */
#include "control/lqg_ctrl.h"

#include "attitude.h"
#include "control/lqg_config.h"
#include "control/lqg_gains.h"
#include "control/quat.h"
#include "lqg_model/STM32.h"

#include <math.h>
#include <string.h>

/** Estados do modelo: [x y z phi theta psi u v w p q r]. */
#define X_Z 2U

/** Estado do modelo (índice) que cada medida y observa (matriz C do modelo). */
static const uint8_t Y_STATE[LQG_NY] = {2U, 3U, 4U, 5U, 6U, 7U, 9U, 10U, 11U};
/** Bit de ::lqg_meas_out_t::flags que valida cada medida. */
static const uint8_t Y_FLAG[LQG_NY] = {LQG_MEAS_RANGE,    LQG_MEAS_ATTITUDE, LQG_MEAS_ATTITUDE,
                                       LQG_MEAS_ATTITUDE, LQG_MEAS_FLOW,     LQG_MEAS_FLOW,
                                       LQG_MEAS_GYRO,     LQG_MEAS_GYRO,     LQG_MEAS_GYRO};
/** Motor físico de cada entrada do modelo. */
static const uint8_t MOTOR_OF_INPUT[4] = LQG_MOTOR_OF_INPUT;

/** Passos seguidos abaixo de ::LQG_TOUCHDOWN_M para valer como "tocou o chão". */
#define TOUCHDOWN_STEPS 10U
/** Referência de altura considerada no alvo, em m. */
#define REF_TOL_M       0.005F
/** Referência de guinada considerada no alvo, em rad. */
#define REF_TOL_RAD     0.005F
/** Máximo de leituras do chão acumuladas (limita a soma). */
#define GROUND_MAX      1000U
/** Maior velocidade de referência XY aceita, em m/s. */
#define XY_SPEED_MAX_M_S 0.5F

static float clampf(float v, float lo, float hi) {
  float r = v;
  if (r < lo) {
    r = lo;
  }
  if (r > hi) {
    r = hi;
  }
  return r;
}

static bool in_range(float v, float lo, float hi) {
  return (isfinite(v) != 0) && (v >= lo) && (v <= hi);
}

lqg_params_t lqg_params_default(void) {
  const lqg_params_t p = {0.30F, 550.0F, 950.0F, 0.10F, 0.10F, 40.0F, 60.0F, 0.05F, 0.3F, true, true};
  return p;
}

bool lqg_params_valid(const lqg_params_t *p) {
  return in_range(p->height_m, LQG_PARAM_HEIGHT_MIN_M, LQG_PARAM_HEIGHT_MAX_M) &&
         in_range(p->hover_permille, LQG_PARAM_HOVER_MIN, LQG_PARAM_HOVER_MAX) &&
         in_range(p->cap_permille, LQG_PARAM_CAP_MIN, LQG_PARAM_CAP_MAX) &&
         in_range(p->climb_m_s, LQG_PARAM_CLIMB_MIN_M_S, LQG_PARAM_CLIMB_MAX_M_S) &&
         in_range(p->land_m_s, LQG_PARAM_CLIMB_MIN_M_S, LQG_PARAM_CLIMB_MAX_M_S) &&
         in_range(p->spool_permille_s, LQG_PARAM_SPOOL_MIN, LQG_PARAM_SPOOL_MAX) &&
         in_range(p->trim_ki, 0.0F, LQG_PARAM_TRIM_KI_MAX) &&
         in_range(p->ki_xy, 0.0F, LQG_PARAM_KI_XY_MAX) &&
         in_range(p->ki_yaw, 0.0F, LQG_PARAM_KI_YAW_MAX);
}

void lqg_ctrl_init(lqg_ctrl_t *c) {
  (void)memset(c, 0, sizeof(*c));
  c->prm = lqg_params_default();
  c->gains = lqg_gains_default();
  c->state = LQG_STATE_IDLE;
  c->vbat_factor = 1.0F;
  if (LQG_CG_TRIM_ENABLE != 0) {
    (void)lqg_ctrl_set_cg_trim(c, LQG_HOVER_TRIM_PERMILLE);
  }
  STM32_initialize();
}

float lqg_cg_trim_scale(float base) {
  const float ratio = clampf(base / LQG_CG_TRIM_REF_PERMILLE, 0.0F, LQG_CG_TRIM_RATIO_MAX);
  const float r2 = ratio * ratio;
  return r2 * r2;
}

bool lqg_ctrl_set_cg_trim(lqg_ctrl_t *c, const float trim[4]) {
  bool ok = true;
  for (uint32_t i = 0U; i < 4U; ++i) {
    ok = ok && in_range(trim[i], -LQG_CG_TRIM_MAX_PERMILLE, LQG_CG_TRIM_MAX_PERMILLE);
  }
  if (ok) {
    (void)memcpy(c->cg_trim, trim, sizeof(c->cg_trim));
  }
  return ok;
}

lqg_gains_t lqg_gains_default(void) {
  const lqg_gains_t g = {1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F};
  return g;
}

bool lqg_gains_valid(const lqg_gains_t *g) {
  return in_range(g->z, 0.0F, LQG_GAIN_MAX) && in_range(g->w, 0.0F, LQG_GAIN_MAX) &&
         in_range(g->att, LQG_GAIN_ATT_MIN, LQG_GAIN_ATT_MAX) &&
         in_range(g->rate, LQG_GAIN_ATT_MIN, LQG_GAIN_ATT_MAX) &&
         in_range(g->yaw, 0.0F, LQG_GAIN_OTHER_MAX) &&
         in_range(g->yaw_rate, 0.0F, LQG_GAIN_OTHER_MAX) &&
         in_range(g->xy, 0.0F, LQG_GAIN_OTHER_MAX) && in_range(g->vel, 0.0F, LQG_GAIN_OTHER_MAX);
}

bool lqg_ctrl_set_gains(lqg_ctrl_t *c, const lqg_gains_t *g) {
  const bool ok = lqg_gains_valid(g);
  if (ok) {
    c->gains = *g;
  }
  return ok;
}

bool lqg_ctrl_set_params(lqg_ctrl_t *c, const lqg_params_t *p) {
  const bool ok = lqg_params_valid(p);
  if (ok) {
    /* A altura do parâmetro só move o alvo quando muda: reenviar os mesmos parâmetros
       (a IHM reenvia a cada sessão) não desfaz um alvo dado pela Pi. */
    if (p->height_m != c->prm.height_m) {
      c->h_goal = p->height_m;
    }
    c->prm = *p;
  }
  return ok;
}

bool lqg_ctrl_arm(lqg_ctrl_t *c) {
  const bool ok = c->state == LQG_STATE_IDLE;
  if (ok) {
    const lqg_params_t keep = c->prm;
    const lqg_gains_t keep_gains = c->gains;
    float keep_trim[4];
    (void)memcpy(keep_trim, c->cg_trim, sizeof(keep_trim));
    (void)memset(c, 0, sizeof(*c));
    c->prm = keep;
    c->gains = keep_gains;
    (void)memcpy(c->cg_trim, keep_trim, sizeof(keep_trim));
    c->vbat_factor = 1.0F;
    /* Estados do filtro zerados no primeiro passo (icLoad do modelo gerado). */
    STM32_initialize();
    c->ekf_on = c->prm.use_ekf;
    c->state = LQG_STATE_ARMED;
  }
  return ok;
}

bool lqg_ctrl_takeoff(lqg_ctrl_t *c) {
  const uint8_t need = LQG_MEAS_ATTITUDE | LQG_MEAS_GYRO | LQG_MEAS_RANGE;
  const bool ok = (c->state == LQG_STATE_ARMED) && (c->ground_n >= LQG_GROUND_SAMPLES) &&
                  ((c->last_flags & need) == need);
  if (ok) {
    c->state = LQG_STATE_SPOOL;
    c->base = LQG_IDLE_PERMILLE;
    c->trim = 0.0F;
    c->land_bias = 0.0F;
    c->ix = 0.0F;
    c->iy = 0.0F;
    c->ipsi = 0.0F;
    c->h_ref = 0.0F;
    c->h_est = 0.0F;
    c->h_floor = 0.0F;
    c->h_goal = c->prm.height_m;
    c->x_ref = 0.0F;
    c->y_ref = 0.0F;
    c->psi_ref = 0.0F;
    c->x_goal = 0.0F;
    c->y_goal = 0.0F;
    c->psi_goal = 0.0F;
    c->xy_speed = LQG_XY_SPEED_M_S;
    c->airborne = false;
    c->spindown = false;
    c->xy_anchored = false;
    c->xy_active = false;
    c->lift_n = 0U;
    c->touch_n = 0U;
    c->low_batt_n = 0U;
  }
  return ok;
}

/** @brief Entra no pouso (pedido, fluxo perdido, bateria ou altura acima do limite). */
static void enter_land(lqg_ctrl_t *c) {
  c->state = LQG_STATE_LAND;
  c->touch_n = 0U;
}

bool lqg_ctrl_land(lqg_ctrl_t *c) {
  const bool ok = (c->state == LQG_STATE_SPOOL) || (c->state == LQG_STATE_CLIMB) ||
                  (c->state == LQG_STATE_HOVER);
  if (ok) {
    enter_land(c);
  }
  return ok;
}

void lqg_ctrl_abort(lqg_ctrl_t *c) {
  c->state = LQG_STATE_IDLE;
  c->airborne = false;
  c->spindown = false;
}

lqg_state_t lqg_ctrl_state(const lqg_ctrl_t *c) {
  return c->state;
}

/** @brief Teto de estabilidade do ganho do ajuste do hover, em ‰/(m·s). */
static float trim_ki_limit(const lqg_ctrl_t *c) {
  const float scale = c->ekf_on ? (c->gains.z * c->gains.w) : 1.0F;
  return LQG_TRIM_STABILITY_FRACTION * LQG_TRIM_KI_LIMIT_1X * scale;
}

float lqg_ctrl_trim_ki(const lqg_ctrl_t *c) {
  const float limit = trim_ki_limit(c);
  return (c->prm.trim_ki < limit) ? c->prm.trim_ki : limit;
}

static bool flying(const lqg_ctrl_t *c) {
  return c->airborne && !c->spindown &&
         ((c->state == LQG_STATE_CLIMB) || (c->state == LQG_STATE_HOVER));
}

bool lqg_ctrl_goto(lqg_ctrl_t *c, float x, float y, float height_m, float psi, float speed) {
  const bool ok = flying(c) && in_range(x, -LQG_XY_LIMIT_M, LQG_XY_LIMIT_M) &&
                  in_range(y, -LQG_XY_LIMIT_M, LQG_XY_LIMIT_M) &&
                  in_range(height_m, LQG_PARAM_HEIGHT_MIN_M, LQG_PARAM_HEIGHT_MAX_M) &&
                  (isfinite(psi) != 0) && in_range(speed, 0.0F, XY_SPEED_MAX_M_S);
  if (ok) {
    c->x_goal = x;
    c->y_goal = y;
    c->h_goal = height_m;
    c->psi_goal = psi;
    c->xy_speed = (speed > 0.0F) ? speed : LQG_XY_SPEED_M_S;
    c->state = LQG_STATE_CLIMB;
  }
  return ok;
}

bool lqg_ctrl_hold(lqg_ctrl_t *c) {
  const bool ok = flying(c);
  if (ok) {
    c->x_goal = c->x_ref;
    c->y_goal = c->y_ref;
    c->h_goal = c->h_ref;
    c->psi_goal = c->psi_ref;
  }
  return ok;
}

static bool ref_at_goal(const lqg_ctrl_t *c) {
  return (fabsf(c->h_ref - c->h_goal) <= REF_TOL_M) && (fabsf(c->x_ref - c->x_goal) <= REF_TOL_M) &&
         (fabsf(c->y_ref - c->y_goal) <= REF_TOL_M) &&
         (fabsf(c->psi_ref - c->psi_goal) <= REF_TOL_RAD);
}

void lqg_ctrl_nav(const lqg_ctrl_t *c, lqg_nav_t *nav) {
  nav->pos[0] = c->est[0];
  nav->pos[1] = c->est[1];
  nav->pos[2] = c->h_est;
  nav->vel[0] = c->est[6];
  nav->vel[1] = c->est[7];
  nav->vel[2] = -c->est[8];
  nav->psi = c->est[5];
  nav->r = c->est[11];
  nav->ref[0] = c->x_ref;
  nav->ref[1] = c->y_ref;
  nav->ref[2] = c->h_ref;
  nav->ref[3] = c->psi_ref;
  nav->goal[0] = c->x_goal;
  nav->goal[1] = c->y_goal;
  nav->goal[2] = c->h_goal;
  nav->goal[3] = c->psi_goal;
  nav->ref_at_goal = ref_at_goal(c);
}

/** @brief Desarma por falha: motores em zero, texto fixo para o log. */
static void fail(lqg_ctrl_t *c, lqg_out_t *out, uint8_t code, const char *why) {
  c->state = LQG_STATE_IDLE;
  c->airborne = false;
  c->spindown = false;
  out->stop = why;
  out->fail = code;
  for (uint32_t i = 0U; i < 4U; ++i) {
    out->permille[i] = 0.0F;
  }
}

/** @brief Copia o que a IHM mostra: medidas, estimativa e ajustes. */
static void telemetry(const lqg_ctrl_t *c, const lqg_meas_out_t *m, lqg_out_t *out) {
  for (uint32_t i = 0U; i < LQG_NY; ++i) {
    out->y[i] = m->y[i];
  }
  out->pos[0] = c->est[0];
  out->pos[1] = c->est[1];
  out->pos[2] = c->est[2];
  out->vel[0] = c->est[6];
  out->vel[1] = c->est[7];
  out->vel[2] = c->est[8];
  out->h_ref = c->h_ref;
  out->trim = c->trim;
  out->base = c->base;
  out->ref_xy[0] = c->x_ref;
  out->ref_xy[1] = c->y_ref;
  out->ref_psi = c->psi_ref;
  out->vbat_factor = c->vbat_factor;
  out->state = c->state;
  out->flags |= ((m->flags & LQG_MEAS_RANGE) != 0U) ? LQG_TLM_RANGE : 0U;
  out->flags |= ((m->flags & LQG_MEAS_FLOW) != 0U) ? LQG_TLM_FLOW : 0U;
  out->flags |= (((m->flags & LQG_MEAS_ATTITUDE) != 0U) && ((m->flags & LQG_MEAS_GYRO) != 0U))
                    ? LQG_TLM_ATTITUDE
                    : 0U;
  out->flags |= c->airborne ? LQG_TLM_FILTER_FREE : 0U;
  out->flags |= c->sat ? LQG_TLM_SATURATED : 0U;
  out->flags |= c->xy_active ? LQG_TLM_XY_ACTIVE : 0U;
  out->flags |= (c->cells != 0U) ? LQG_TLM_VBAT : 0U;
}

/** @brief Prende o filtro nas medidas: no chão ele não deve integrar nada. */
static void seed_filter(const double y[LQG_NY]) {
  double *x = STM32_DW.MemoryX_DSTATE;
  for (uint32_t i = 0U; i < 12U; ++i) {
    x[i] = 0.0;
  }
  for (uint32_t i = 0U; i < LQG_NY; ++i) {
    /* u e v ficam em zero no chão; as demais medidas entram. */
    if ((i != LQG_Y_U) && (i != LQG_Y_V)) {
      x[Y_STATE[i]] = y[i];
    }
  }
  STM32_DW.icLoad = false;
}

/** @brief Referência anda até o alvo na velocidade pedida (m ou rad, por segundo). */
static float slew(float value, float target, float rate_per_s) {
  const float step = rate_per_s * LQG_TS_S;
  float r = value;
  if (r < (target - step)) {
    r += step;
  } else if (r > (target + step)) {
    r -= step;
  } else {
    r = target;
  }
  return r;
}

/** @brief Referência XY anda em linha reta até o alvo, na velocidade pedida. */
static void slew_xy(lqg_ctrl_t *c) {
  const float dx = c->x_goal - c->x_ref;
  const float dy = c->y_goal - c->y_ref;
  const float dist = sqrtf((dx * dx) + (dy * dy));
  const float step = c->xy_speed * LQG_TS_S;
  if (dist <= step) {
    c->x_ref = c->x_goal;
    c->y_ref = c->y_goal;
  } else {
    c->x_ref += (dx / dist) * step;
    c->y_ref += (dy / dist) * step;
  }
}

/**
 * @brief Tensão da bateria: filtro, células (contadas no chão) e fator da saída.
 *
 * Sem leitura calibrada desde o armamento o fator fica 1. Se a leitura some em voo,
 * o último fator continua (voltar a 1 seria um degrau no empuxo).
 */
static void battery_step(lqg_ctrl_t *c, const lqg_sensors_t *in) {
  if (in->vbat_valid && (isfinite(in->vbat_v) != 0) && (in->vbat_v > 3.0F)) {
    if (c->vbat_filt <= 0.0F) {
      c->vbat_filt = in->vbat_v;
    } else {
      c->vbat_filt += (in->vbat_v - c->vbat_filt) * (LQG_TS_S / LQG_VBAT_TAU_S);
    }
    if (c->state == LQG_STATE_ARMED) {
      const float n = floorf((c->vbat_filt / LQG_VBAT_REF_V_PER_CELL) + 0.5F);
      c->cells = (uint32_t)clampf(n, 1.0F, 6.0F);
    }
    if (c->cells != 0U) {
      const float ref = (float)c->cells * LQG_VBAT_REF_V_PER_CELL;
      c->vbat_factor = clampf(ref / c->vbat_filt, LQG_VBAT_FACTOR_MIN, LQG_VBAT_FACTOR_MAX);
    }
  }
}

/**
 * @brief Zero de nível: média de φ e θ crus enquanto armado no chão.
 *
 * A distância corrigida do chão usa φ e θ, então um nível novo muda pouco o chão
 * medido (cos de ângulos pequenos); os dois são médias das mesmas amostras.
 */
static void level_step(lqg_ctrl_t *c, const lqg_meas_out_t *m) {
  if (((m->flags & LQG_MEAS_ATTITUDE) == 0U) || (c->level_n >= GROUND_MAX)) {
    return;
  }
  if (m->q_native) {
    /* Caminho em quatérnio: média do vetor de rotação da inclinação no corpo. */
    for (uint32_t i = 0U; i < 3U; ++i) {
      c->level_rv_sum[i] += m->tilt_raw[i];
    }
  }
  c->level_sum[0] += m->y[LQG_Y_PHI] + c->meas.level_phi;
  c->level_sum[1] += m->y[LQG_Y_THETA] + c->meas.level_theta;
  c->level_n++;
  const float phi = c->level_sum[0] / (float)c->level_n;
  const float theta = c->level_sum[1] / (float)c->level_n;
  const bool level_ground = (fabsf(phi) <= LQG_LEVEL_MAX_RAD) && (fabsf(theta) <= LQG_LEVEL_MAX_RAD);
  lqg_meas_set_level(&c->meas, level_ground ? phi : 0.0F, level_ground ? theta : 0.0F);
  if (m->q_native) {
    float rv[3];
    bool ok = true;
    for (uint32_t i = 0U; i < 2U; ++i) {
      rv[i] = c->level_rv_sum[i] / (float)c->level_n;
      ok = ok && (fabsf(rv[i]) <= LQG_LEVEL_MAX_RAD);
    }
    rv[2] = 0.0F;
    if (!ok) {
      rv[0] = 0.0F;
      rv[1] = 0.0F;
    }
    lqg_meas_set_level_q(&c->meas, rv);
  }
}

/** @brief Checagens do passo; true se desarmou. */
static bool supervise(lqg_ctrl_t *c, const lqg_meas_out_t *m, lqg_out_t *out) {
  bool stopped = false;
  const uint8_t att = LQG_MEAS_ATTITUDE | LQG_MEAS_GYRO;
  c->imu_bad = ((m->flags & att) == att) ? 0U : (c->imu_bad + 1U);
  c->range_bad = ((m->flags & LQG_MEAS_RANGE) != 0U) ? 0U : (c->range_bad + 1U);
  /* Fluxo só é cobrado acima da altura em que ele vale: na decolagem o drone passa
     ~2 s abaixo de 10 cm sem fluxo, e isso não é perda. */
  if (c->airborne && ((m->flags & LQG_MEAS_FLOW_HEIGHT) != 0U)) {
    c->flow_bad = ((m->flags & LQG_MEAS_FLOW) != 0U) ? 0U : (c->flow_bad + 1U);
  } else {
    c->flow_bad = 0U;
  }

  if (c->imu_bad >= LQG_IMU_BAD_STEPS) {
    fail(c, out, LQG_FAIL_IMU, "BNO sem dado: motores cortados");
    stopped = true;
  } else if ((c->state != LQG_STATE_ARMED) && ((fabsf(m->y[LQG_Y_PHI]) > LQG_TILT_MAX_RAD) ||
                                               (fabsf(m->y[LQG_Y_THETA]) > LQG_TILT_MAX_RAD))) {
    fail(c, out, LQG_FAIL_TILT, "inclinacao excessiva: motores cortados");
    stopped = true;
  } else if ((c->state != LQG_STATE_ARMED) && (c->range_bad >= LQG_RANGE_BAD_STEPS)) {
    fail(c, out, LQG_FAIL_RANGE, "MTF sem distancia: motores cortados");
    stopped = true;
  } else {
    /* sensores em ordem */
  }
  return stopped;
}

/**
 * @brief Quanto tempo a rampa continuou depois de o empuxo igualar o peso, em s.
 *
 * Com a rampa a @p rate ‰/s, a aceleração líquida cresce a a·rate (a =
 * ::LQG_ACCEL_PER_PERMILLE) e a altura a a·rate·t³/6: subir @p h leva
 * t = cbrt(6·h/(a·rate)), mais os @p steps da confirmação.
 */
static float liftoff_lag_s(float h, uint32_t steps, float rate) {
  const float t =
      cbrtf((6.0F * h) / (LQG_ACCEL_PER_PERMILLE * rate)) + ((float)steps * LQG_TS_S);
  return (t < LQG_LIFTOFF_LAG_MAX_S) ? t : LQG_LIFTOFF_LAG_MAX_S;
}

/**
 * @brief Fase de subida da aceleração: devolve true se desarmou.
 *
 * Rampa rápida até perto do hover, devagar depois. Com a altura acima de
 * ::LQG_LIFTOFF_M por ::LQG_LIFTOFF_STEPS o drone conta como no ar: a base volta
 * ao ponto em que o empuxo igualou o peso (a rampa seguiu subindo durante a
 * detecção) e a referência de altura parte da altura medida. Daí em diante o
 * controle de altura e o integrador tiram o drone do efeito solo.
 *
 * Histórico: com a confirmação em 8 cm (e paradas da rampa a 3 cm) o drone real
 * ficou quicando a 2–7 cm por 4 s, sem controle de altura, enquanto a rampa
 * empurrava sozinha (ensaio de 12/11, 17:36). Com 3 cm por 50 ms a decolagem
 * falsa antiga não volta: a base agora é a estimativa do hover, não 0,45 s de
 * rampa abaixo dela, o integrador não tem janela e a correção de altura é 10×.
 */
static bool spool_step(lqg_ctrl_t *c, float h_meas, bool range_ok, lqg_out_t *out) {
  bool stopped = false;
  const bool fast = c->base < (c->prm.hover_permille - LQG_SPOOL_FAST_MARGIN_PERMILLE);
  const float rate = fast ? LQG_SPOOL_FAST_PERMILLE_S : c->prm.spool_permille_s;
  c->base += rate * LQG_TS_S;
  if (range_ok) {
    c->h_floor += (h_meas - c->h_floor) * (LQG_TS_S / LQG_LIFTOFF_FLOOR_TAU_S);
  }
  c->lift_n =
      (range_ok && (h_meas > (c->h_floor + LQG_LIFTOFF_M))) ? (c->lift_n + 1U) : 0U;
  if (c->lift_n >= LQG_LIFTOFF_STEPS) {
    const float hover = c->base - (rate * liftoff_lag_s(LQG_LIFTOFF_M, LQG_LIFTOFF_STEPS, rate));
    c->trim = clampf(hover - c->prm.hover_permille, -LQG_TRIM_MAX_PERMILLE, LQG_TRIM_MAX_PERMILLE);
    c->h_ref = (h_meas > 0.0F) ? h_meas : 0.0F;
    c->airborne = true;
    c->state = LQG_STATE_CLIMB;
    out->note = "descolou";
  } else if ((c->base * c->vbat_factor) >=
             clampf(c->prm.cap_permille - 10.0F, 0.0F,
                    (c->prm.hover_permille + LQG_TRIM_MAX_PERMILLE) * c->vbat_factor)) {
    fail(c, out, LQG_FAIL_NO_LIFTOFF, "nao descolou: motores no limite, cortados");
    stopped = true;
  } else {
    /* segue subindo */
  }
  return stopped;
}

/**
 * @brief Integrador de altura (ajuste do hover) com o erro do filtro.
 *
 * Sem janela de erro: com o alvo a 40 cm e o drone a 4 cm o erro passa de 30 cm,
 * e a janela antiga congelava o integrador justamente quando ele era necessário.
 * Anti-windup por direção: com motor no teto não sobe, com motor no piso não desce.
 */
static void trim_step(lqg_ctrl_t *c, lqg_out_t *out) {
  const float err = c->h_ref - c->h_est;
  /* Perto do chão (efeito solo, sem fluxo) integra mais rápido, nunca acima do teto de estabilidade. */
  const float boost = (c->h_est < LQG_TRIM_BOOST_BELOW_M) ? LQG_TRIM_BOOST : 1.0F;
  const float ki = fminf(lqg_ctrl_trim_ki(c) * boost, trim_ki_limit(c));
  const bool blocked = ((err > 0.0F) && c->sat_hi) || ((err < 0.0F) && c->sat_lo);
  if (c->prm.autotrim && (ki > 0.0F) && !blocked) {
    c->trim = clampf(c->trim + (ki * err * LQG_TS_S), -LQG_TRIM_MAX_PERMILLE, LQG_TRIM_MAX_PERMILLE);
    out->flags |= LQG_TLM_TRIM;
  }
}

/** @brief Pouso: referência desce; no toque, a aceleração cai até desarmar. */
static bool land_step(lqg_ctrl_t *c, float h_meas, bool range_ok, lqg_out_t *out) {
  bool stopped = false;
  if (!c->spindown) {
    c->h_ref = slew(c->h_ref, 0.0F, c->prm.land_m_s);
    /* O LQR de altura é mole: sem isto um hover mal ajustado não desce. */
    c->land_bias = clampf(c->land_bias + (LQG_LAND_BIAS_PERMILLE_S * LQG_TS_S), 0.0F,
                          LQG_LAND_BIAS_MAX_PERMILLE);
    /* Desce no lugar: o alvo horizontal e a guinada ficam onde estão. */
    c->x_goal = c->x_ref;
    c->y_goal = c->y_ref;
    c->psi_goal = c->psi_ref;
    c->touch_n = (range_ok && (h_meas <= LQG_TOUCHDOWN_M)) ? (c->touch_n + 1U) : 0U;
    if (!c->airborne || (c->touch_n >= TOUCHDOWN_STEPS)) {
      c->spindown = true;
      c->airborne = false;
      out->note = "toque: reduzindo os motores";
    }
  } else {
    c->base -= LQG_SPINDOWN_PERMILLE_S * LQG_TS_S;
    if (c->base <= LQG_IDLE_PERMILLE) {
      fail(c, out, LQG_FAIL_LANDED, "pouso concluido");
      stopped = true;
    }
  }
  return stopped;
}

/** @brief Bateria fraca ou base sem folga por 2 s em voo: pousa. */
static bool battery_low(lqg_ctrl_t *c) {
  const bool weak = (c->cells != 0U) &&
                    (c->vbat_filt < ((float)c->cells * LQG_VBAT_LAND_V_PER_CELL));
  const bool no_margin = (c->base * c->vbat_factor) > LQG_BASE_LAND_PERMILLE;
  c->low_batt_n = (weak || no_margin) ? (c->low_batt_n + 1U) : 0U;
  return c->low_batt_n >= LQG_LOW_BATT_STEPS;
}

/** @brief Referência, ajuste do hover e fase de um passo em voo. */
static bool flight_logic(lqg_ctrl_t *c, float h_meas, bool range_ok, lqg_out_t *out) {
  bool stopped = false;
  if (c->state == LQG_STATE_SPOOL) {
    stopped = spool_step(c, h_meas, range_ok, out);
  } else if (c->state == LQG_STATE_LAND) {
    stopped = land_step(c, h_meas, range_ok, out);
  } else {
    /* CLIMB ou HOVER: as referências andam até o alvo. */
    const float h_rate = (c->h_goal >= c->h_ref) ? c->prm.climb_m_s : c->prm.land_m_s;
    c->h_ref = slew(c->h_ref, c->h_goal, h_rate);
    slew_xy(c);
    c->psi_ref = slew(c->psi_ref, c->psi_goal, LQG_YAW_RATE_RAD_S);
    c->state = ref_at_goal(c) ? LQG_STATE_HOVER : LQG_STATE_CLIMB;
    trim_step(c, out);
    if (range_ok && (h_meas > (c->h_goal + LQG_OVERSHOOT_M))) {
      enter_land(c);
      out->note = "altura acima do limite: pousando";
    } else if (c->flow_bad >= LQG_FLOW_BAD_STEPS) {
      enter_land(c);
      out->note = "fluxo optico perdido: pousando";
    } else if (battery_low(c)) {
      enter_land(c);
      out->note = "bateria fraca ou motores sem folga: pousando";
    } else {
      /* segue no alvo */
    }
  }
  if (c->airborne && !c->spindown && (c->state != LQG_STATE_SPOOL)) {
    c->base = c->prm.hover_permille + c->trim - c->land_bias;
  }
  return stopped;
}

/**
 * @brief Δω do modelo -> aceleração dos motores físicos, com os limites.
 *
 * Prioridade da atitude: se um motor passa do teto, os quatro descem juntos pelo
 * excesso (a diferença entre eles, que segura a atitude, fica; perde-se altura).
 * Se um passa do piso, sobem juntos sem passar do teto. No chão, a diferença
 * entre motores fica em ±::LQG_GROUND_AUTHORITY_PERMILLE. Tudo é escalado pela
 * tensão da bateria no fim.
 */
static void mix(lqg_ctrl_t *c, const double du[4], lqg_out_t *out, double applied[4]) {
  const float lo = (c->spindown || (c->state == LQG_STATE_SPOOL)) ? 0.0F : LQG_MIN_PERMILLE;
  const float hi = c->prm.cap_permille;
  const float k = c->vbat_factor;
  float want[4];
  float mx = -1e9F;
  float mn = 1e9F;
  /* Só na rampa rápida (ainda longe de descolar) ou no fim do pouso a diferença entre
     motores é limitada. Da rampa lenta em diante o drone pode estar leve e quicando
     abaixo da confirmação de 8 cm: limitar ali deixou o drone sem como corrigir a
     atitude e ele virou (ensaios de 12/11, 17:23). */
  const bool grounded = c->spindown ||
                        ((c->state == LQG_STATE_SPOOL) &&
                         (c->base < (c->prm.hover_permille - LQG_SPOOL_FAST_MARGIN_PERMILLE)));
  float dd[4];
  float dmean = 0.0F;
  for (uint32_t i = 0U; i < 4U; ++i) {
    dd[i] = (float)du[i] / LQG_W_PER_PERMILLE;
    if (grounded) {
      dd[i] = clampf(dd[i], -LQG_GROUND_AUTHORITY_PERMILLE, LQG_GROUND_AUTHORITY_PERMILLE);
    }
    dmean += 0.25F * dd[i];
  }
  /* Trim do CG: cresce com (aceleração média comandada / 550)^4, a que o drone de fato recebe
     (base mais a parte coletiva do Δω da altura), e não só com a base. */
  const float trim_scale = lqg_cg_trim_scale(c->base + dmean);
  float cg[4];
  for (uint32_t i = 0U; i < 4U; ++i) {
    cg[i] = c->cg_trim[i] * trim_scale;
  }
  for (uint32_t i = 0U; i < 4U; ++i) {
    const float d = dd[i];
    want[i] = (c->base + cg[i] + d) * k;
    mx = (want[i] > mx) ? want[i] : mx;
    mn = (want[i] < mn) ? want[i] : mn;
  }
  float shift = 0.0F;
  c->sat_hi = false;
  c->sat_lo = false;
  if (mx > hi) {
    shift = hi - mx;
    c->sat_hi = true;
  } else if (mn < lo) {
    shift = clampf(lo - mn, 0.0F, hi - mx);
    c->sat_lo = true;
  } else {
    /* dentro dos limites */
  }
  for (uint32_t i = 0U; i < 4U; ++i) {
    const float pm = clampf(want[i] + shift, lo, hi);
    if (pm < (want[i] + shift)) {
      c->sat_hi = true;
    } else if (pm > (want[i] + shift)) {
      c->sat_lo = true;
    } else {
      /* sem corte individual */
    }
    out->permille[MOTOR_OF_INPUT[i]] = pm;
    applied[i] = (double)(((pm / k) - c->base - cg[i]) * LQG_W_PER_PERMILLE);
  }
  c->sat = c->sat_hi || c->sat_lo;
}

/** @brief Se um motor saturou, a predição do Kalman usa o Δω realmente aplicado. */
static void correct_prediction(const double du[4], const double applied[4]) {
  for (uint32_t r = 0U; r < 12U; ++r) {
    double delta = 0.0;
    for (uint32_t k = 0U; k < 4U; ++k) {
      delta += STM32_ConstP.B_Value[r + (12U * k)] * (applied[k] - du[k]);
    }
    STM32_DW.MemoryX_DSTATE[r] += delta;
  }
}

/** @brief Passo com o Kalman estacionário do modelo gerado. */
static void step_generated(lqg_ctrl_t *c, const lqg_meas_out_t *mp, lqg_out_t *out) {
  const lqg_meas_out_t m = *mp;
  /* Medidas inválidas entram como a própria predição (inovação zero). */
  double y_use[LQG_NY];
  for (uint32_t i = 0U; i < LQG_NY; ++i) {
    y_use[i] = ((m.flags & Y_FLAG[i]) != 0U) ? (double)m.y[i] : STM32_DW.MemoryX_DSTATE[Y_STATE[i]];
    STM32_U.y[i] = y_use[i];
  }
  STM32_U.In1[0] = (double)c->x_ref;
  STM32_U.In1[1] = (double)c->y_ref;
  STM32_U.In1[2] = -(double)c->h_ref; /* NED: para cima é negativo */
  STM32_U.In1[3] = (double)c->psi_ref;
  STM32_step();

  double du[4];
  double applied[4];
  for (uint32_t i = 0U; i < 4U; ++i) {
    du[i] = STM32_Y.Out1[i];
  }
  mix(c, du, out, applied);
  if (c->airborne) {
    correct_prediction(du, applied);
  } else {
    seed_filter(y_use);
  }
  for (uint32_t i = 0U; i < 12U; ++i) {
    c->est[i] = (float)STM32_DW.MemoryX_DSTATE[i];
  }
  c->h_est = -c->est[X_Z];
}

/** @brief Integradores de posição XY e guinada: só no ar, em subida ou hover, sem saturar. */
static void integrate(lqg_ctrl_t *c, const float x[EKF3_NEST]) {
  const bool active = flying(c) && !c->sat;
  if (active && c->xy_active) {
    c->ix = clampf(c->ix + (c->prm.ki_xy * (x[EKF3_PX] - c->x_ref) * LQG_TS_S), -LQG_INT_XY_LIMIT_M,
                   LQG_INT_XY_LIMIT_M);
    c->iy = clampf(c->iy + (c->prm.ki_xy * (x[EKF3_PY] - c->y_ref) * LQG_TS_S), -LQG_INT_XY_LIMIT_M,
                   LQG_INT_XY_LIMIT_M);
  }
  if (active) {
    const float yaw_err =
        clampf(c->psi_unw - c->psi_ref, -LQG_YAW_ERR_MAX_RAD, LQG_YAW_ERR_MAX_RAD);
    c->ipsi = clampf(c->ipsi + (c->prm.ki_yaw * yaw_err * LQG_TS_S),
                     -LQG_INT_YAW_LIMIT_RAD, LQG_INT_YAW_LIMIT_RAD);
  }
}

/**
 * @brief Δω = −Kd·G·(x̂ − x_ref) − integradores, no referencial da guinada.
 *
 * O modelo foi linearizado com psi = 0: o erro de posição e a velocidade
 * (locais) giram para o referencial do corpo alinhado ao norte do armamento
 * antes de entrar nos ganhos gerados, e os integradores usam as mesmas colunas
 * de Kd. A forma de erro equivale a Kfd·r do modelo gerado (no Kd gerado a
 * coluna z é igual à de Kfd) e continua correta quando G escala a coluna z.
 * G = multiplicadores da IHM por coluna (::lqg_gains_t); XY é relativo à
 * atitude. Sem origem XY (fluxo ainda abaixo da altura útil) posição e
 * velocidade horizontais ficam fora: só atitude, altura e guinada.
 *
 * Atitude: o erro de inclinação é o vetor de rotação do quatérnio estimado em
 * relação à própria guinada (a referência de φ, θ é nível), no lugar dos
 * ângulos de Euler; a guinada vem do quatérnio e é desenrolada em
 * ::lqg_ctrl_t::psi_unw.
 */
static void ekf_control(const lqg_ctrl_t *c, double du[4]) {
  float x[EKF3_NEST];
  float tilt[2];
  float yaw_q;
  ekf3_estimate(&c->ekf, x);
  ekf3_attitude(&c->ekf, tilt, &yaw_q);
  const lqg_gains_t *g = &c->gains;
  const float cs = cosf(c->psi_unw);
  const float sn = sinf(c->psi_unw);
  const float ex = x[EKF3_PX] - c->x_ref;
  const float ey = x[EKF3_PY] - c->y_ref;
  float xc[12];
  for (uint32_t i = 0U; i < 12U; ++i) {
    xc[i] = x[i];
  }
  xc[EKF3_PX] = (cs * ex) + (sn * ey);
  xc[EKF3_PY] = (-sn * ex) + (cs * ey);
  xc[EKF3_U] = (cs * x[EKF3_U]) + (sn * x[EKF3_V]);
  xc[EKF3_V] = (-sn * x[EKF3_U]) + (cs * x[EKF3_V]);
  xc[EKF3_PZ] = x[EKF3_PZ] + c->h_ref; /* z para baixo: referência é −h_ref */
  xc[EKF3_PHI] = tilt[0];
  xc[EKF3_THETA] = tilt[1];
  xc[EKF3_PSI] = clampf(c->psi_unw - c->psi_ref, -LQG_YAW_ERR_MAX_RAD, LQG_YAW_ERR_MAX_RAD);
  const float xy_on = c->xy_active ? 1.0F : 0.0F;
  const float gxy = g->xy * g->att * xy_on;
  const float gvel = g->vel * g->att * xy_on;
  const float scale[12] = {gxy,    gxy,    g->z,   g->att,  g->att,  g->yaw,
                           gvel,   gvel,   g->w,   g->rate, g->rate, g->yaw_rate};
  const float ixh = (cs * c->ix) + (sn * c->iy);
  const float iyh = (-sn * c->ix) + (cs * c->iy);
  const double *kd = LQG_KD;
  for (uint32_t i = 0U; i < 4U; ++i) {
    double v = 0.0;
    for (uint32_t k = 0U; k < 12U; ++k) {
      v -= kd[(k * 4U) + i] * (double)(scale[k] * xc[k]);
    }
    v -= kd[(EKF3_PX * 4U) + i] * (double)(gxy * ixh);
    v -= kd[(EKF3_PY * 4U) + i] * (double)(gxy * iyh);
    v -= kd[(EKF3_PSI * 4U) + i] * (double)(g->yaw * c->ipsi);
    du[i] = v;
  }
}

static const float ATT_R_SCALE[4] = LQG_ATT_R_SCALE;

/* Kd e Kfd vêm de tools/lqr_design.py (control/lqg_gains.h); o gerado pelo Simulink segue só no Kalman. */
_Static_assert(sizeof(LQG_KD) == (48U * sizeof(double)), "Kd tem 4x12 ganhos");
_Static_assert(sizeof(LQG_KFD) == (16U * sizeof(double)), "Kfd tem 4x4 ganhos");

/** @brief Soma giro e aceleração com o drone parado e armado (viés do IMU no chão). */
static void rest_step(lqg_ctrl_t *c, const lqg_meas_out_t *m) {
  const uint8_t need = LQG_MEAS_GYRO | LQG_MEAS_ACCEL;
  if (((m->flags & need) != need) || (c->rest_n >= GROUND_MAX)) {
    return;
  }
  c->rest_sum[0] += m->y[LQG_Y_P];
  c->rest_sum[1] += m->y[LQG_Y_Q];
  c->rest_sum[2] += m->y[LQG_Y_R];
  for (uint32_t i = 0U; i < 3U; ++i) {
    c->rest_sum[3U + i] += m->accel_body[i];
  }
  c->rest_n++;
}

/** @brief Viés inicial do IMU: média do repouso; sem amostras ou fora do plausível, zero. */
static void rest_bias(const lqg_ctrl_t *c, float bg[3], float ba[3]) {
  for (uint32_t i = 0U; i < 3U; ++i) {
    bg[i] = 0.0F;
    ba[i] = 0.0F;
  }
  if (c->rest_n < LQG_REST_MIN_SAMPLES) {
    return;
  }
  const float n = (float)c->rest_n;
  const bool ok = (fabsf(c->rest_sum[0] / n) < LQG_REST_GYRO_MAX_RAD_S) &&
                  (fabsf(c->rest_sum[1] / n) < LQG_REST_GYRO_MAX_RAD_S) &&
                  (fabsf(c->rest_sum[2] / n) < LQG_REST_GYRO_MAX_RAD_S) &&
                  (fabsf(c->rest_sum[3] / n) < LQG_REST_ACCEL_MAX_M_S2) &&
                  (fabsf(c->rest_sum[4] / n) < LQG_REST_ACCEL_MAX_M_S2) &&
                  (fabsf(c->rest_sum[5] / n) < LQG_REST_ACCEL_MAX_M_S2);
  if (ok) {
    for (uint32_t i = 0U; i < 3U; ++i) {
      bg[i] = c->rest_sum[i] / n;
      ba[i] = c->rest_sum[3U + i] / n;
    }
  }
}

/** @brief Passo com o EKF3: corrige, controla, limita, prediz com o IMU do passo. */
static void step_ekf(lqg_ctrl_t *c, const lqg_meas_out_t *m, const lqg_sensors_t *in,
                     lqg_out_t *out) {
  ekf3_t *e = &c->ekf;
  const bool att = (m->flags & LQG_MEAS_ATTITUDE) != 0U;
  const bool gyro = (m->flags & LQG_MEAS_GYRO) != 0U;
  const bool range = (m->flags & LQG_MEAS_RANGE) != 0U;
  const bool flow = (m->flags & LQG_MEAS_FLOW) != 0U;
  ekf3_meas_t z;
  (void)memset(&z, 0, sizeof(z));
  z.gyro_valid = gyro;
  z.gyro[0] = m->y[LQG_Y_P];
  z.gyro[1] = m->y[LQG_Y_Q];
  z.gyro[2] = m->y[LQG_Y_R];
  z.accel_valid = (m->flags & LQG_MEAS_ACCEL) != 0U;
  for (uint32_t i = 0U; i < 3U; ++i) {
    z.accel[i] = m->accel_body[i];
  }
  if (!c->airborne) {
    /* No chão o filtro é preso nas medidas: nada deve integrar. */
    float bg[3];
    float ba[3];
    rest_bias(c, bg, ba);
    ekf3_seed(e, att ? m->q : e->q, range ? m->y[LQG_Y_Z] : e->p[2], bg, ba);
    ekf3_meas_t rates;
    (void)memset(&rates, 0, sizeof(rates));
    rates.gyro_valid = gyro;
    (void)memcpy(rates.gyro, z.gyro, sizeof(rates.gyro));
    ekf3_update(e, &rates);
    if (att) {
      c->psi_unw = m->y[LQG_Y_PSI];
    }
    c->psi_last = quat_yaw(e->q);
  } else {
    z.attitude_valid = att;
    (void)memcpy(z.q_meas, m->q, sizeof(z.q_meas));
    z.att_r_scale = ATT_R_SCALE[m->att_accuracy & 3U];
    /* Distância e fluxo entram uma vez por quadro do MTF-01. */
    z.range_valid = range && ((m->flags & LQG_MEAS_FRESH) != 0U);
    z.range_m = in->range_m;
    z.ground_m = c->meas.ground_m;
    z.range_pos_x = LQG_RANGE_POS_X_M;
    z.range_pos_y = LQG_RANGE_POS_Y_M;
    z.flow_valid = flow;
    z.flow_fwd = m->flow_body[0];
    z.flow_right = m->flow_body[1];
    z.flow_quality = in->flow_quality;
    ekf3_update(e, &z);
    const float yaw_q = quat_yaw(e->q);
    c->psi_unw += attitude_wrap_pi(yaw_q - c->psi_last);
    c->psi_last = yaw_q;
    if (flow && !c->xy_anchored) {
      /* Primeiro fluxo confiável em voo: a posição dead-reckoned até aqui não vale
         nada; a origem horizontal passa a ser este ponto. */
      ekf3_reset_xy(e);
      c->ix = 0.0F;
      c->iy = 0.0F;
      c->x_ref = 0.0F;
      c->y_ref = 0.0F;
      c->x_goal = 0.0F;
      c->y_goal = 0.0F;
      c->xy_anchored = true;
    }
  }
  c->xy_active = c->airborne && c->xy_anchored && ((m->flags & LQG_MEAS_FLOW_HEIGHT) != 0U);
  ekf3_estimate(e, c->est);
  c->est[EKF3_PSI] = c->psi_unw;
  c->h_est = -c->est[EKF3_PZ];

  double du[4];
  ekf_control(c, du);
  bool finite = isfinite(c->h_est) != 0;
  for (uint32_t i = 0U; i < 4U; ++i) {
    finite = finite && (isfinite(du[i]) != 0);
  }
  if (!finite || (fabsf(c->h_est) > 5.0F)) {
    fail(c, out, LQG_FAIL_ESTIMATOR, "estimador invalido: motores cortados");
    return;
  }
  double applied[4];
  mix(c, du, out, applied);
  integrate(c, c->est);
  if (c->airborne) {
    ekf3_predict(e, &z);
  }
}

void lqg_ctrl_step(lqg_ctrl_t *c, const lqg_sensors_t *in, lqg_out_t *out) {
  lqg_meas_out_t m;
  (void)memset(out, 0, sizeof(*out));
  lqg_meas_update(&c->meas, in, &m);
  c->last_flags = m.flags;

  if (c->state == LQG_STATE_IDLE) {
    telemetry(c, &m, out);
    return;
  }
  battery_step(c, in);
  const bool range_ok = (m.flags & LQG_MEAS_RANGE) != 0U;

  if (c->state == LQG_STATE_ARMED) {
    /* No chão: só mede a distância de referência, motores em zero. */
    (void)supervise(c, &m, out);
    if (range_ok && (c->ground_n < GROUND_MAX) && (c->state == LQG_STATE_ARMED)) {
      c->ground_sum += m.range_corrected_m;
      c->ground_n++;
      lqg_meas_set_ground(&c->meas, c->ground_sum / (float)c->ground_n);
    }
    level_step(c, &m);
    rest_step(c, &m);
    telemetry(c, &m, out);
    return;
  }

  /* Altura medida acima do chão do armamento, em m (para cima). */
  const float h_meas = range_ok ? (m.range_corrected_m - c->meas.ground_m) : c->h_est;
  if (supervise(c, &m, out) || flight_logic(c, h_meas, range_ok, out)) {
    telemetry(c, &m, out);
    return;
  }

  if (c->ekf_on) {
    step_ekf(c, &m, in, out);
  } else {
    step_generated(c, &m, out);
  }
  telemetry(c, &m, out);
}
