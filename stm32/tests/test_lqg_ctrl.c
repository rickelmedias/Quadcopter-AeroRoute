/**
 * @file test_lqg_ctrl.c
 * @brief Controle LQR + Kalman em malha fechada contra a planta linear do modelo gerado.
 *
 * A planta é x+ = A x + B Δω com as próprias matrizes do modelo (STM32_ConstP),
 * mais o chão (a altura não fica negativa). Os sensores são gerados com o mapa
 * inverso de control/lqg_config.h. Não prova nada sobre o drone real: prova que
 * a sequência armar/decolar/pairar/pousar, o mapa de motores, os limites e as
 * proteções funcionam sobre o modelo para o qual o controle foi projetado.
 */
#include "control/lqg_config.h"
#include "control/lqg_ctrl.h"
#include "control/lqg_gains.h"
#include "control/quat.h"
#include "../App/Src/control/lqg_model/STM32.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define GROUND_RANGE_M 0.05F
#define YAW0           1.0F
#define PI_F           3.14159265F

static const uint8_t MOTOR_OF_INPUT[4] = LQG_MOTOR_OF_INPUT;

typedef struct {
  double x[12];
  float true_hover; /**< ‰ em que a planta de fato paira (diferente do parâmetro). */
  /** Efeito solo: o hover cai deste tanto (‰) rente ao chão, decaindo com a altura. */
  float ground_effect;
  /** Tensão da bateria (0 = sem medida) e a tensão em que true_hover vale. */
  float vbat, vbat_ref;
  /** A tensão existe na planta mas não chega ao controle (PA1 sem divisor). */
  bool vbat_hidden;
  /** Distância falsa somada ao MTF (inclinação, ruído), em m. */
  float range_bias;
  bool lifted;
  /** Falhas injetadas. */
  bool imu_dead;
  /** Sem o quatérnio do BNO: a atitude do EKF3 vem dos ângulos de Euler (caminho de reserva). */
  bool euler_only;
  bool range_dead;
  bool flow_dead;
  /** Distúrbios: nível do sensor errado (rad), torque de guinada (rad/s por passo), espúrios do fluxo. */
  float sens_bias_phi;
  float sens_bias_theta;
  double torque_r;
  /** Aceleração cinemática local (NED) do último passo, em m/s². */
  double acc[3];
  uint32_t spike_period;
  uint32_t steps;
} sim_t;

static float wrap_pi(float a) {
  while (a > PI_F) {
    a -= 2.0F * PI_F;
  }
  while (a < -PI_F) {
    a += 2.0F * PI_F;
  }
  return a;
}

/** @brief Sensores que o drone simulado daria no estado atual. */
static void make_sensors(const sim_t *s, lqg_sensors_t *in) {
  const float phi = (float)s->x[3];
  const float theta = (float)s->x[4];
  const float psi = (float)s->x[5];
  const float deg = PI_F / 180.0F;
  memset(in, 0, sizeof(*in));
  in->pitch = ((phi + s->sens_bias_phi) / LQG_SGN_ROLL) + (LQG_ZERO_PITCH_DEG * deg);
  in->roll = wrap_pi(((theta + s->sens_bias_theta) / LQG_SGN_PITCH) + (LQG_ZERO_ROLL_DEG * deg));
  in->yaw = wrap_pi(YAW0 + (psi / LQG_SGN_YAW));
  quat_from_euler(in->roll, in->pitch, in->yaw, in->q);
  in->q_valid = !s->euler_only;
  in->att_accuracy = 3U;
  in->gyro[1] = (float)s->x[9] / LQG_SGN_P;
  in->gyro[0] = (float)s->x[10] / LQG_SGN_Q;
  in->gyro[2] = (float)s->x[11] / LQG_SGN_R;
  in->attitude_valid = !s->imu_dead;
  in->gyro_valid = !s->imu_dead;
  /* Aceleração linear (sem gravidade) no corpo, pelo mapa inverso do giro. */
  {
    float q[4];
    quat_from_euler(phi, theta, psi, q);
    const float an[3] = {(float)s->acc[0], (float)s->acc[1], (float)s->acc[2]};
    float ab[3];
    quat_rotate_inv(q, an, ab);
    in->accel[1] = ab[0] / LQG_SGN_P;
    in->accel[0] = ab[1] / LQG_SGN_Q;
    in->accel[2] = ab[2] / LQG_SGN_R;
    in->accel_valid = !s->imu_dead;
  }
  const float h = -(float)s->x[2];
  const float lever = (LQG_RANGE_POS_X_M * sinf(theta)) - (LQG_RANGE_POS_Y_M * sinf(phi) * cosf(theta));
  in->range_m = (GROUND_RANGE_M + h + lever) / (cosf(phi) * cosf(theta)) + s->range_bias;
  in->mtf_frames = s->steps + 1U;
  in->vbat_v = s->vbat;
  in->vbat_valid = (s->vbat > 0.0F) && !s->vbat_hidden;
  in->range_valid = !s->range_dead;
  /* O fluxo mede a velocidade no corpo: gira a velocidade local por -psi. */
  /* O sensor fora do centro também anda quando o drone gira (r × braço). */
  const float r = (float)s->x[11];
  const float u = ((cosf(psi) * (float)s->x[6]) + (sinf(psi) * (float)s->x[7])) + (LQG_FLOW_YAW_X * r);
  const float v = ((-sinf(psi) * (float)s->x[6]) + (cosf(psi) * (float)s->x[7])) + (LQG_FLOW_YAW_Y * r);
  in->flow_x =
      ((-v / (LQG_SGN_VY * in->range_m)) - (LQG_FLOW_GYRO_X * in->gyro[0])) / LQG_FLOW_SCALE_X;
  in->flow_y =
      ((u / (LQG_SGN_VX * in->range_m)) - (LQG_FLOW_GYRO_Y * in->gyro[1])) / LQG_FLOW_SCALE_Y;
  if ((s->spike_period != 0U) && ((s->steps % s->spike_period) == 0U)) {
    in->flow_x += 15.0F; /* ~ +4,5 m/s a 0,3 m: espúrio do MTF */
    in->flow_y -= 15.0F;
  }
  in->flow_valid = !s->flow_dead;
  in->flow_quality = 100U;
}

/** @brief Avança a planta um passo com os motores físicos @p pm (‰). */
/** Trim de hover por motor (u1..u4, ‰) que a planta exige: o CG real do drone simulado. */
static float g_plant_trim[4];

static void plant_step(sim_t *s, const float pm[4]) {
  double du[4];
  const float mean = 0.25F * (pm[0] + pm[1] + pm[2] + pm[3]);
  const float h = -(float)s->x[2];
  /* Hover efetivo: menor rente ao chão (efeito solo, decai em ~5 cm). */
  const float hover = s->true_hover - (s->ground_effect * expf(-h / 0.05F));
  /* ESC em malha aberta: o mesmo ‰ rende proporcionalmente à tensão. */
  const float volts = ((s->vbat > 0.0F) && (s->vbat_ref > 0.0F)) ? (s->vbat / s->vbat_ref) : 1.0F;
  for (uint32_t i = 0U; i < 4U; ++i) {
    const float need = hover + (g_plant_trim[i] * lqg_cg_trim_scale(mean * volts));
    du[i] = (double)(((pm[MOTOR_OF_INPUT[i]] * volts) - need) * LQG_W_PER_PERMILLE);
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
  /* Verdade física: a gravidade inclinada acelera na direção da inclinação girada por psi
     (o modelo linear do Simulink supõe psi = 0). Escrito aqui sem reaproveitar o EKF. */
  const double ts = 0.01;
  const double g = 9.81;
  const double c = cos(s->x[5]);
  const double sn = sin(s->x[5]);
  const double a_n = -g * ((s->x[4] * c) + (s->x[3] * sn));
  const double a_e = g * ((s->x[3] * c) - (s->x[4] * sn));
  next[6] = s->x[6] + (ts * a_n);
  next[7] = s->x[7] + (ts * a_e);
  next[0] = s->x[0] + (ts * s->x[6]) + (0.5 * ts * ts * a_n);
  next[1] = s->x[1] + (ts * s->x[7]) + (0.5 * ts * ts * a_e);
  next[11] += s->torque_r;
  for (uint32_t i = 0U; i < 3U; ++i) {
    s->acc[i] = (next[6U + i] - s->x[6U + i]) / ts;
  }
  memcpy(s->x, next, sizeof(next));
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
    memset(s->acc, 0, sizeof(s->acc));
  }
}

/** @brief Uma volta da malha: sensores -> controle -> planta. */
static lqg_out_t tick(lqg_ctrl_t *c, sim_t *s) {
  lqg_sensors_t in;
  lqg_out_t out;
  make_sensors(s, &in);
  ++s->steps;
  lqg_ctrl_step(c, &in, &out);
  if ((out.stop == NULL) && (out.state >= LQG_STATE_SPOOL)) {
    plant_step(s, out.permille);
  }
  return out;
}

static float height(const sim_t *s) {
  return -(float)s->x[2];
}

/** @brief Arma, mede o chão e decola. */
static void arm_and_takeoff(lqg_ctrl_t *c, sim_t *s) {
  assert(lqg_ctrl_arm(c));
  for (uint32_t i = 0U; i < (LQG_GROUND_SAMPLES + 5U); ++i) {
    const lqg_out_t o = tick(c, s);
    assert(o.state == LQG_STATE_ARMED && o.stop == NULL);
    for (uint32_t m = 0U; m < 4U; ++m) {
      assert(o.permille[m] == 0.0F);
    }
  }
  assert(lqg_ctrl_takeoff(c));
}

static float max4(const float v[4]) {
  float m = v[0];
  for (uint32_t i = 1U; i < 4U; ++i) {
    m = (v[i] > m) ? v[i] : m;
  }
  return m;
}

static void test_params(void) {
  lqg_ctrl_t c;
  lqg_ctrl_init(&c);
  lqg_params_t p = lqg_params_default();
  assert(lqg_params_valid(&p));
  p.height_m = 0.0F;
  assert(!lqg_ctrl_set_params(&c, &p));
  p = lqg_params_default();
  p.hover_permille = NAN;
  assert(!lqg_params_valid(&p));
  p = lqg_params_default();
  p.cap_permille = 1001.0F;
  assert(!lqg_params_valid(&p));
  p = lqg_params_default();
  p.height_m = 0.5F;
  assert(lqg_ctrl_set_params(&c, &p) && (c.prm.height_m == 0.5F));
  assert(!lqg_ctrl_takeoff(&c)); /* desarmado */
  assert(!lqg_ctrl_land(&c));
}

/** @brief Sem o quatérnio do BNO (só Euler): o caminho de reserva também decola, paira e fica no lugar. */
static void test_euler_fallback(void) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  s.true_hover = c.prm.hover_permille;
  s.euler_only = true;
  arm_and_takeoff(&c, &s);
  lqg_out_t o = {0};
  for (uint32_t i = 0U; i < 3000U; ++i) {
    o = tick(&c, &s);
    assert(o.stop == NULL);
  }
  assert(o.state == LQG_STATE_HOVER);
  assert(fabsf(height(&s) - 0.30F) < 0.03F);
  assert(!c.meas.q_ready); /* o caminho nativo nunca foi usado */
}

static void test_takeoff_hover_land(void) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  s.true_hover = c.prm.hover_permille; /* parâmetro certo */
  arm_and_takeoff(&c, &s);

  lqg_out_t o = {0};
  float peak = 0.0F;
  bool saw_note = false;
  for (uint32_t i = 0U; i < 3000U; ++i) { /* 30 s */
    o = tick(&c, &s);
    assert(o.stop == NULL);
    assert(max4(o.permille) <= c.prm.cap_permille + 0.01F);
    saw_note = saw_note || (o.note != NULL);
    peak = (height(&s) > peak) ? height(&s) : peak;
  }
  assert(saw_note);
  assert(o.state == LQG_STATE_HOVER);
  assert(fabsf(height(&s) - 0.30F) < 0.03F);
  assert(peak < 0.30F + 0.10F); /* sem overshoot grande */
  assert((o.flags & LQG_TLM_FILTER_FREE) != 0U);
  assert(fabsf(o.pos[2] + height(&s)) < 0.03F); /* Kalman concorda (z para baixo) */

  /* Pouso: desce, corta e desarma sozinho. */
  assert(lqg_ctrl_land(&c));
  bool done = false;
  for (uint32_t i = 0U; (i < 3000U) && !done; ++i) {
    o = tick(&c, &s);
    if (o.stop != NULL) {
      done = true;
      assert(o.fail == LQG_FAIL_LANDED);
      assert(strcmp(o.stop, "pouso concluido") == 0);
      assert(o.state == LQG_STATE_IDLE);
      assert(height(&s) < 0.08F);
    }
  }
  assert(done);
}

/** @brief Ganhos de altura da bancada (P 10×, D 5×) e o resto 1×. */
static lqg_gains_t bench_gains(void) {
  lqg_gains_t g = lqg_gains_default();
  g.z = 10.0F;
  g.w = 5.0F;
  return g;
}

static void test_hover_error_is_trimmed(void) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  s.true_hover = 640.0F; /* 90 ‰ acima do hover configurado (550) */
  arm_and_takeoff(&c, &s);
  lqg_out_t o = {0};
  for (uint32_t i = 0U; i < 6000U; ++i) { /* 60 s */
    o = tick(&c, &s);
    assert(o.stop == NULL);
  }
  assert(o.state == LQG_STATE_HOVER);
  /* A base partiu do hover medido na decolagem, não do digitado. */
  assert(fabsf((c.prm.hover_permille + o.trim) - 640.0F) < 12.0F);
  assert(fabsf(height(&s) - 0.30F) < 0.04F);

  /* Hover que muda no ar (bateria cedendo): o integrador acha o novo valor; sem ele,
     sobra erro de altura (LQR puro). */
  float err[2];
  for (uint32_t k = 0U; k < 2U; ++k) {
    memset(&s, 0, sizeof(s));
    lqg_ctrl_init(&c);
    lqg_params_t p = lqg_params_default();
    p.autotrim = (k == 0U);
    assert(lqg_ctrl_set_params(&c, &p));
    const lqg_gains_t g = bench_gains();
    assert(lqg_ctrl_set_gains(&c, &g));
    s.true_hover = 550.0F;
    arm_and_takeoff(&c, &s);
    for (uint32_t i = 0U; i < 6000U; ++i) {
      if (i == 1500U) {
        s.true_hover = 590.0F;
      }
      assert(tick(&c, &s).stop == NULL);
    }
    err[k] = fabsf(height(&s) - 0.30F);
  }
  assert(err[0] < 0.02F);
  assert(err[1] > 0.10F); /* 40 ‰ / 200 ‰/m = 20 cm: justifica o integrador */
  /* O ganho efetivo respeita o limite de estabilidade com os ganhos 1×. */
  lqg_ctrl_init(&c);
  assert(fabsf(lqg_ctrl_trim_ki(&c) - (LQG_TRIM_STABILITY_FRACTION * LQG_TRIM_KI_LIMIT_1X)) < 0.01F);
}

/**
 * @brief Efeito solo: descola com 60 ‰ a menos e o hover sobe com a altura. Sem a
 *        janela de 30 cm do integrador antigo o drone chega ao alvo de 40 cm.
 */
static void test_ground_effect_climb(void) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  lqg_params_t p = lqg_params_default();
  p.height_m = 0.40F;
  assert(lqg_ctrl_set_params(&c, &p));
  const lqg_gains_t g = bench_gains();
  assert(lqg_ctrl_set_gains(&c, &g));
  s.true_hover = 650.0F;
  s.ground_effect = 60.0F;
  arm_and_takeoff(&c, &s);
  lqg_out_t o = {0};
  for (uint32_t i = 0U; i < 3000U; ++i) {
    o = tick(&c, &s);
    assert(o.stop == NULL);
  }
  assert(fabsf(height(&s) - 0.40F) < 0.03F);
  assert(fabsf((c.prm.hover_permille + o.trim) - 650.0F) < 10.0F);
}

/** @brief Leitura falsa de 4 cm no chão: conta como decolado e mesmo assim chega aos 30 cm. */
static void test_false_liftoff_hint(void) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  /* Com altura 1× uma decolagem falsa deixa o drone no chão (2 ‰ por cm de erro);
     os ganhos de boot (LQG_BOOT_GAINS) é que precisam tirá-lo dali. */
  const lqg_gains_t g = LQG_BOOT_GAINS;
  assert(lqg_ctrl_set_gains(&c, &g));
  s.true_hover = c.prm.hover_permille;
  arm_and_takeoff(&c, &s);
  bool lifted_note = false;
  lqg_out_t o = {0};
  for (uint32_t i = 0U; i < 1500U; ++i) {
    /* Entre 3,5 s e 5 s a distância lê 4 cm a mais (inclinação apoiada). */
    s.range_bias = ((i > 350U) && (i < 500U) && !s.lifted) ? 0.04F : 0.0F;
    o = tick(&c, &s);
    assert(o.stop == NULL);
    if ((o.note != NULL) && (strcmp(o.note, "descolou") == 0)) {
      lifted_note = true; /* pode ser o falso: o que importa é chegar ao alvo */
    }
  }
  assert(lifted_note && (fabsf(height(&s) - 0.30F) < 0.05F));
}

/**
 * @brief Com os motores girando o MTF lê 2,5 cm a mais (12/11, 18:02): isso não é decolagem.
 *        A decolagem só vale quando o drone de fato sai do chão, e a base é o hover real.
 */
static void test_liftoff_ignores_reading_offset(void) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  const lqg_gains_t g = LQG_BOOT_GAINS;
  assert(lqg_ctrl_set_gains(&c, &g));
  s.true_hover = 600.0F; /* sai do chão só com 60 % */
  arm_and_takeoff(&c, &s);
  bool early = false;
  lqg_out_t o = {0};
  for (uint32_t i = 0U; i < 2500U; ++i) {
    s.range_bias = (c.base > 100.0F) ? 0.025F : 0.0F; /* o offset aparece com o motor girando */
    o = tick(&c, &s);
    assert(o.stop == NULL);
    if ((o.note != NULL) && (strcmp(o.note, "descolou") == 0)) {
      early = !s.lifted;
    }
  }
  assert(!early);
  assert(fabsf(height(&s) - 0.30F) < 0.05F);
  assert(fabsf((c.prm.hover_permille + o.trim) - 600.0F) < 15.0F);
}

/** @brief Erro de guinada de 170° não pede mais que o erro limitado (±29°). */
static void test_yaw_error_clamped(void) {
  float diff[2];
  const float psi0[2] = {0.6F, 3.0F}; /* 34° e 172° */
  for (uint32_t k = 0U; k < 2U; ++k) {
    lqg_ctrl_t c;
    sim_t s;
    memset(&s, 0, sizeof(s));
    lqg_ctrl_init(&c);
    s.true_hover = c.prm.hover_permille;
    arm_and_takeoff(&c, &s);
    for (uint32_t i = 0U; i < 1500U; ++i) {
      (void)tick(&c, &s);
    }
    s.x[5] = psi0[k]; /* o drone aparece girado */
    lqg_sensors_t in;
    lqg_out_t o;
    make_sensors(&s, &in);
    for (uint32_t i = 0U; i < 30U; ++i) { /* o EKF precisa de alguns passos para ver o giro */
      lqg_ctrl_step(&c, &in, &o);
    }
    diff[k] = (o.permille[0] + o.permille[3]) - (o.permille[1] + o.permille[2]);
  }
  assert(fabsf(diff[1]) < (1.15F * fabsf(diff[0])) + 1.0F);
}

/** @brief BNO montado 1,5° fora de nível desde o chão: o zero do armamento tira o erro. */
static void test_level_at_arm(void) {
  float drift[2];
  for (uint32_t k = 0U; k < 2U; ++k) {
    lqg_ctrl_t c;
    sim_t s;
    memset(&s, 0, sizeof(s));
    lqg_ctrl_init(&c);
    lqg_params_t p = lqg_params_default();
    p.ki_xy = 0.0F; /* sem integrador: o erro de nível viraria deriva */
    assert(lqg_ctrl_set_params(&c, &p));
    s.true_hover = p.hover_permille;
    /* k = 0: 1,5° (dentro do limite, corrigido); k = 1: 6° (chão torto, não corrige). */
    s.sens_bias_theta = (k == 0U) ? 0.026F : 0.105F;
    arm_and_takeoff(&c, &s);
    if (k == 0U) {
      assert(fabsf(c.meas.level_theta - 0.026F) < 0.002F);
    } else {
      assert(c.meas.level_theta == 0.0F);
    }
    for (uint32_t i = 0U; i < 1500U; ++i) {
      (void)tick(&c, &s);
    }
    drift[k] = sqrtf((float)((s.x[0] * s.x[0]) + (s.x[1] * s.x[1])));
  }
  assert(drift[0] < 0.10F);
  assert(drift[1] > (3.0F * drift[0]));
}

/** @brief Bateria cai de 12,4 V para 11,0 V em voo: a saída compensa e a altura fica. */
static void test_battery_compensation(void) {
  float err[2];
  for (uint32_t k = 0U; k < 2U; ++k) {
    lqg_ctrl_t c;
    sim_t s;
    memset(&s, 0, sizeof(s));
    lqg_ctrl_init(&c);
    lqg_params_t p = lqg_params_default();
    p.autotrim = false; /* só a compensação */
    assert(lqg_ctrl_set_params(&c, &p));
    const lqg_gains_t g = bench_gains();
    assert(lqg_ctrl_set_gains(&c, &g));
    s.true_hover = 550.0F;
    s.vbat_ref = 12.4F;
    s.vbat = 12.4F;
    s.vbat_hidden = (k == 1U);
    arm_and_takeoff(&c, &s);
    lqg_out_t o = {0};
    float before = 0.0F;
    for (uint32_t i = 0U; i < 4000U; ++i) {
      if (i == 1500U) {
        before = height(&s);
      }
      if (i > 1500U) {
        s.vbat = 12.4F - (1.4F * fminf(1.0F, (float)(i - 1500U) / 1500.0F));
      }
      o = tick(&c, &s);
      assert(o.stop == NULL);
    }
    err[k] = fabsf(height(&s) - before); /* o quanto a queda de tensão moveu a altura */
    if (k == 0U) {
      assert((c.cells == 3U) && ((o.flags & LQG_TLM_VBAT) != 0U));
      assert(fabsf(o.vbat_factor - ((3.0F * LQG_VBAT_REF_V_PER_CELL) / 11.0F)) < 0.02F);
    } else {
      assert(o.vbat_factor == 1.0F);
    }
  }
  assert(err[0] < 0.02F);
  assert(err[1] > 0.10F);
}

/** @brief Motor no teto: os quatro descem juntos e a diferença da atitude fica. */
static void test_mixer_priority(void) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  lqg_gains_t g = lqg_gains_default();
  g.att = 5.0F; /* atitude da bancada: ~22 ‰ por motor a 0,15 rad */
  assert(lqg_ctrl_set_gains(&c, &g));
  s.true_hover = c.prm.hover_permille;
  arm_and_takeoff(&c, &s);
  for (uint32_t i = 0U; i < 2000U; ++i) {
    assert(tick(&c, &s).stop == NULL);
  }
  /* Mesmo passo duas vezes (cópias do controlador): sem teto e com o teto 1 ‰
     acima do hover, com uma rolagem que pede bem mais que isso de diferença. */
  s.x[3] = 0.15;
  lqg_sensors_t in;
  make_sensors(&s, &in);
  lqg_ctrl_t free_c = c;
  lqg_out_t free_o;
  lqg_ctrl_step(&free_c, &in, &free_o);
  lqg_params_t p = c.prm;
  p.cap_permille = 551.0F;
  assert(lqg_ctrl_set_params(&c, &p));
  lqg_out_t o;
  lqg_ctrl_step(&c, &in, &o);
  const float free_diff = (free_o.permille[0] + free_o.permille[1]) - (free_o.permille[2] + free_o.permille[3]);
  const float diff = (o.permille[0] + o.permille[1]) - (o.permille[2] + o.permille[3]);
  assert(max4(free_o.permille) > (p.cap_permille + 3.0F));
  assert(fabsf(max4(o.permille) - p.cap_permille) < 0.01F);
  assert((o.flags & LQG_TLM_SATURATED) != 0U);
  assert(fabsf(diff - free_diff) < 0.1F); /* a diferença da atitude ficou inteira */
}

/** @brief Fluxo só acima de 10 cm: decolagem sem fluxo não pousa; XY só lá em cima. */
static void test_flow_height_gate(void) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  s.true_hover = c.prm.hover_permille;
  arm_and_takeoff(&c, &s);
  bool flow_low = false;
  bool xy_low = false;
  lqg_out_t o = {0};
  for (uint32_t i = 0U; i < 3000U; ++i) {
    o = tick(&c, &s);
    assert(o.stop == NULL && o.state != LQG_STATE_LAND);
    if (height(&s) < (LQG_FLOW_OFF_HEIGHT_M - 0.01F)) {
      flow_low = flow_low || ((o.flags & LQG_TLM_FLOW) != 0U);
      xy_low = xy_low || ((o.flags & LQG_TLM_XY_ACTIVE) != 0U);
    }
  }
  assert(!flow_low && !xy_low);
  assert((o.flags & LQG_TLM_XY_ACTIVE) != 0U);
}

/** @brief Alvo da Pi: posição, altura e guinada; recusa fora de voo e fora dos limites. */
static void test_goto(void) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  s.true_hover = c.prm.hover_permille;
  assert(!lqg_ctrl_goto(&c, 0.1F, 0.0F, 0.3F, 0.0F, 0.1F)); /* desarmado */
  arm_and_takeoff(&c, &s);
  for (uint32_t i = 0U; i < 1500U; ++i) {
    assert(tick(&c, &s).stop == NULL);
  }
  assert(!lqg_ctrl_goto(&c, 5.0F, 0.0F, 0.3F, 0.0F, 0.1F));
  assert(!lqg_ctrl_goto(&c, 0.0F, 0.0F, 0.05F, 0.0F, 0.1F));
  assert(!lqg_ctrl_goto(&c, 0.0F, NAN, 0.3F, 0.0F, 0.1F));
  assert(lqg_ctrl_goto(&c, 0.40F, -0.20F, 0.40F, 0.5F, 0.10F));
  lqg_nav_t nav;
  lqg_out_t o = {0};
  for (uint32_t i = 0U; i < 3000U; ++i) {
    o = tick(&c, &s);
    assert(o.stop == NULL);
  }
  lqg_ctrl_nav(&c, &nav);
  assert(nav.ref_at_goal && (o.state == LQG_STATE_HOVER));
  assert(fabsf((float)s.x[0] - 0.40F) < 0.05F);
  assert(fabsf((float)s.x[1] + 0.20F) < 0.05F);
  assert(fabsf(height(&s) - 0.40F) < 0.03F);
  assert(fabsf((float)s.x[5] - 0.5F) < 0.05F);
  assert(fabsf(nav.pos[0] - (float)s.x[0]) < 0.03F);
  /* HOLD: o alvo vira a referência atual. */
  assert(lqg_ctrl_hold(&c));
  lqg_ctrl_nav(&c, &nav);
  assert(nav.goal[0] == nav.ref[0] && nav.goal[2] == nav.ref[2]);
}

static void test_height_change_live(void) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  s.true_hover = c.prm.hover_permille;
  arm_and_takeoff(&c, &s);
  for (uint32_t i = 0U; i < 2500U; ++i) {
    (void)tick(&c, &s);
  }
  lqg_params_t p = c.prm;
  p.height_m = 0.50F;
  assert(lqg_ctrl_set_params(&c, &p));
  lqg_out_t o = {0};
  for (uint32_t i = 0U; i < 2500U; ++i) {
    o = tick(&c, &s);
    assert(o.stop == NULL);
  }
  assert(fabsf(height(&s) - 0.50F) < 0.04F);
}

/** @brief φ positivo (lado direito para baixo) aumenta os motores da direita (M1, M2). */
static void test_motor_mapping(void) {
  lqg_ctrl_t c;
  sim_t s;
  const float no_trim[4] = {0.0F, 0.0F, 0.0F, 0.0F}; /* só o mapa de eixos: sem o CG */
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  assert(lqg_ctrl_set_cg_trim(&c, no_trim));
  s.true_hover = c.prm.hover_permille;
  arm_and_takeoff(&c, &s);
  s.x[3] = 0.05; /* φ: apoiado ainda; só os sensores veem a inclinação */
  lqg_sensors_t in;
  lqg_out_t o;
  make_sensors(&s, &in);
  for (uint32_t i = 0U; i < 3U; ++i) {
    lqg_ctrl_step(&c, &in, &o);
  }
  assert((o.permille[0] + o.permille[1]) > (o.permille[2] + o.permille[3]) + 1.0F);
  assert(fabsf(o.permille[0] - o.permille[1]) < 0.5F); /* sem arfagem */
  assert(fabsf(o.y[LQG_Y_PHI] - 0.05F) < 1e-4F);

  /* θ positivo (nariz para cima) aumenta os motores de trás (M1, M3). */
  memset(&s.x, 0, sizeof(s.x));
  s.x[4] = 0.05;
  make_sensors(&s, &in);
  for (uint32_t i = 0U; i < 3U; ++i) {
    lqg_ctrl_step(&c, &in, &o);
  }
  assert(fabsf(o.y[LQG_Y_THETA] - 0.05F) < 1e-4F);
  /* Nariz para cima: tira dos motores da frente (M2, M4) e põe nos de trás (M1, M3). */
  assert((o.permille[0] + o.permille[2]) > (o.permille[1] + o.permille[3]) + 1.0F);
  assert(fabsf(o.permille[0] - o.permille[2]) < 0.5F); /* sem rolagem */
}

static void test_failsafes(void) {
  lqg_ctrl_t c;
  sim_t s;

  /* Sem decolar: a aceleração chega ao limite e corta. */
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  s.true_hover = 990.0F;
  arm_and_takeoff(&c, &s);
  lqg_out_t o = {0};
  for (uint32_t i = 0U; (i < 4000U) && (o.stop == NULL); ++i) {
    o = tick(&c, &s);
  }
  assert((o.stop != NULL) && (o.fail == LQG_FAIL_NO_LIFTOFF) && (o.state == LQG_STATE_IDLE));
  assert(max4(o.permille) == 0.0F);

  /* BNO some em voo: corta em ~100 ms. */
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  s.true_hover = c.prm.hover_permille;
  arm_and_takeoff(&c, &s);
  for (uint32_t i = 0U; i < 1500U; ++i) {
    (void)tick(&c, &s);
  }
  s.imu_dead = true;
  uint32_t n = 0U;
  do {
    o = tick(&c, &s);
    ++n;
  } while ((o.stop == NULL) && (n < 100U));
  assert((o.fail == LQG_FAIL_IMU) && (n <= LQG_IMU_BAD_STEPS));

  /* Distância some em voo: corta em ~250 ms. */
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  s.true_hover = c.prm.hover_permille;
  arm_and_takeoff(&c, &s);
  for (uint32_t i = 0U; i < 1500U; ++i) {
    (void)tick(&c, &s);
  }
  s.range_dead = true;
  n = 0U;
  do {
    o = tick(&c, &s);
    ++n;
  } while ((o.stop == NULL) && (n < 100U));
  assert((o.fail == LQG_FAIL_RANGE) && (n <= LQG_RANGE_BAD_STEPS));

  /* Inclinação excessiva em voo corta na hora. */
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  s.true_hover = c.prm.hover_permille;
  arm_and_takeoff(&c, &s);
  for (uint32_t i = 0U; i < 1500U; ++i) {
    (void)tick(&c, &s);
  }
  s.x[3] = 0.8; /* ~46 graus */
  o = tick(&c, &s);
  assert((o.stop != NULL) && (o.fail == LQG_FAIL_TILT));

  /* Fluxo perdido por 2 s no ar: pousa (não corta). */
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  s.true_hover = c.prm.hover_permille;
  arm_and_takeoff(&c, &s);
  for (uint32_t i = 0U; i < 1500U; ++i) {
    (void)tick(&c, &s);
  }
  s.flow_dead = true;
  bool landing = false;
  for (uint32_t i = 0U; i < (LQG_FLOW_BAD_STEPS + 20U); ++i) {
    o = tick(&c, &s);
    assert(o.stop == NULL);
    landing = landing || (o.state == LQG_STATE_LAND);
  }
  assert(landing);
}

static void test_arm_rules(void) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  assert(lqg_ctrl_arm(&c));
  assert(!lqg_ctrl_arm(&c));
  assert(!lqg_ctrl_takeoff(&c)); /* sem o chão medido */
  for (uint32_t i = 0U; i < (LQG_GROUND_SAMPLES - 1U); ++i) {
    (void)tick(&c, &s);
  }
  assert(!lqg_ctrl_takeoff(&c));
  (void)tick(&c, &s);
  s.range_dead = true;
  (void)tick(&c, &s); /* última leitura sem distância */
  assert(!lqg_ctrl_takeoff(&c));
  s.range_dead = false;
  (void)tick(&c, &s);
  assert(lqg_ctrl_takeoff(&c));
  assert(!lqg_ctrl_takeoff(&c));
  /* Pouso antes de descolar só reduz os motores e desarma. */
  assert(lqg_ctrl_land(&c));
  lqg_out_t o = {0};
  for (uint32_t i = 0U; (i < 3000U) && (o.stop == NULL); ++i) {
    o = tick(&c, &s);
  }
  assert((o.stop != NULL) && (o.fail == LQG_FAIL_LANDED));
}

/** @brief Resultado de um voo de @p secs com distúrbios. */
typedef struct {
  bool stopped;
  float x, y, psi, h;
  float max_xy;
} scenario_t;

static scenario_t run_scenario(lqg_params_t p, float bias, double torque, uint32_t spike,
                               double secs) {
  lqg_ctrl_t c;
  sim_t s;
  memset(&s, 0, sizeof(s));
  lqg_ctrl_init(&c);
  assert(lqg_ctrl_set_params(&c, &p));
  s.true_hover = p.hover_permille;
  arm_and_takeoff(&c, &s);
  s.sens_bias_phi = bias;
  s.sens_bias_theta = bias;
  s.torque_r = torque;
  s.spike_period = spike;
  scenario_t r = {false, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  for (uint32_t i = 0U; (i < (uint32_t)(secs * 100.0)) && !r.stopped; ++i) {
    const lqg_out_t o = tick(&c, &s);
    r.stopped = o.stop != NULL;
    const float xy = fmaxf(fabsf((float)s.x[0]), fabsf((float)s.x[1]));
    r.max_xy = (xy > r.max_xy) ? xy : r.max_xy;
  }
  r.x = (float)s.x[0];
  r.y = (float)s.x[1];
  r.psi = (float)s.x[5];
  r.h = height(&s);
  return r;
}

/** @brief Espúrios do MTF-01 (~4,5 m/s a cada 0,37 s): o EKF rejeita, o Kalman gerado deriva. */
static void test_flow_spikes(void) {
  lqg_params_t p = lqg_params_default();
  scenario_t ekf = run_scenario(p, 0.0F, 0.0, 37U, 60.0);
  assert(!ekf.stopped && (ekf.max_xy < 0.1F) && (fabsf(ekf.h - 0.30F) < 0.03F));
  p.use_ekf = false;
  scenario_t kf = run_scenario(p, 0.0F, 0.0, 37U, 60.0);
  assert(kf.stopped || (kf.max_xy > 1.0F));
}

/**
 * @brief Torque de guinada constante que leva ψ além de 90°: o Kalman gerado
 *        (linearizado em ψ = 0) empurra para o lado errado e se perde; o EKF segura.
 */
static void test_heading_frame(void) {
  lqg_params_t p = lqg_params_default();
  p.ki_xy = 0.0F;
  p.ki_yaw = 0.0F;
  scenario_t ekf = run_scenario(p, 0.0F, 0.10, 0U, 120.0);
  assert(!ekf.stopped && (fabsf(ekf.psi) > 1.6F) && (ekf.max_xy < 0.1F));
  p.use_ekf = false;
  scenario_t kf = run_scenario(p, 0.0F, 0.10, 0U, 120.0);
  assert(kf.stopped || (kf.max_xy > 1.0F));
}

/** @brief Integradores: nível errado vira deriva de velocidade; torque vira erro de guinada. */
static void test_integrators(void) {
  lqg_params_t p = lqg_params_default();
  p.ki_xy = 0.0F;
  p.ki_yaw = 0.0F;
  scenario_t none = run_scenario(p, 0.03F, 0.0, 0U, 120.0);
  p = lqg_params_default();
  p.ki_yaw = 0.0F;
  scenario_t xy = run_scenario(p, 0.03F, 0.0, 0U, 120.0);
  assert(!none.stopped && !xy.stopped);
  assert(fabsf(xy.x) < (0.5F * fabsf(none.x)));

  p = lqg_params_default();
  p.ki_xy = 0.0F;
  p.ki_yaw = 0.0F;
  scenario_t yaw0 = run_scenario(p, 0.0F, 0.05, 0U, 120.0);
  p = lqg_params_default();
  p.ki_xy = 0.0F;
  scenario_t yaw = run_scenario(p, 0.0F, 0.05, 0U, 120.0);
  assert(fabsf(yaw0.psi) > 0.5F);
  assert(fabsf(yaw.psi) < 0.1F);
}

/** @brief Multiplicador da altura: hover que sobe 0,5 % no ar, sem trim: o erro cai ~10x. */
static void test_gain_multipliers(void) {
  float final[2];
  const float zg[2] = {1.0F, 10.0F};
  for (uint32_t k = 0U; k < 2U; ++k) {
    lqg_ctrl_t c;
    sim_t s;
    memset(&s, 0, sizeof(s));
    lqg_ctrl_init(&c);
    lqg_params_t p = lqg_params_default();
    p.autotrim = false;
    assert(lqg_ctrl_set_params(&c, &p));
    lqg_gains_t g = lqg_gains_default();
    g.z = zg[k];
    g.w = 5.0F * (zg[k] / 10.0F) + ((k == 0U) ? 0.5F : 0.0F);
    assert(lqg_ctrl_set_gains(&c, &g));
    s.true_hover = p.hover_permille;
    arm_and_takeoff(&c, &s);
    float before = 0.0F;
    for (uint32_t i = 0U; i < 6000U; ++i) {
      if (i == 2500U) {
        before = height(&s);
        s.true_hover += 5.0F; /* a base partiu do hover medido; o erro vem depois */
      }
      assert(tick(&c, &s).stop == NULL);
    }
    final[k] = fabsf(height(&s) - before);
  }
  assert(final[0] > 0.15F); /* 5 ‰ / 20 ‰/m = 25 cm */
  assert(final[1] < (0.2F * final[0]));
  /* Limites. */
  lqg_gains_t g = lqg_gains_default();
  g.att = 0.1F;
  assert(!lqg_gains_valid(&g));
  g = lqg_gains_default();
  g.z = 25.0F;
  assert(!lqg_gains_valid(&g));
  g = lqg_gains_default();
  g.yaw = NAN;
  assert(!lqg_gains_valid(&g));
  g = lqg_gains_default();
  g.xy = 0.0F;
  assert(lqg_gains_valid(&g));
}

/**
 * @brief CG deslocado (trim de hover por motor): a planta exige +30/+30/-30/-30 ‰ em u1..u4 (frente
 *        mais pesada). Sem o trim no controlador o drone inclina e deriva; com ele paira nivelado.
 */
static void test_cg_trim(void) {
  const float need[4] = {30.0F, 30.0F, -30.0F, -30.0F};
  const float none[4] = {0.0F, 0.0F, 0.0F, 0.0F};
  const float bad[4] = {0.0F, NAN, 0.0F, 0.0F};
  const float huge[4] = {0.0F, 0.0F, 0.0F, LQG_CG_TRIM_MAX_PERMILLE + 1.0F};
  float pitch[2];
  float drift[2];
  bool stopped[2];
  float saved[4];
  memcpy(saved, g_plant_trim, sizeof(saved));
  memcpy(g_plant_trim, need, sizeof(need));
  for (uint32_t with = 0U; with < 2U; ++with) {
    lqg_ctrl_t c;
    sim_t s;
    memset(&s, 0, sizeof(s));
    lqg_ctrl_init(&c);
    assert(lqg_ctrl_set_cg_trim(&c, (with != 0U) ? need : none));
    s.true_hover = lqg_params_default().hover_permille;
    arm_and_takeoff(&c, &s);
    float worst = 0.0F;
    stopped[with] = false;
    for (uint32_t i = 0U; (i < 3000U) && !stopped[with]; ++i) {
      const lqg_out_t o = tick(&c, &s);
      stopped[with] = o.stop != NULL;
      worst = fmaxf(worst, fmaxf(fabsf((float)s.x[0]), fabsf((float)s.x[1])));
    }
    pitch[with] = (float)s.x[4];
    drift[with] = worst;
  }
  /* sem trim: inclina e deriva (ou o limite de inclinação o corta); com ele: nivelado e parado */
  assert(stopped[0] || (fabsf(pitch[0]) > 0.01F));
  assert(!stopped[1]);
  assert(fabsf(pitch[1]) < 0.003F);
  assert(drift[1] < 0.1F);
  /* valores inválidos são recusados e não trocam o trim */
  lqg_ctrl_t c;
  lqg_ctrl_init(&c);
  assert(lqg_ctrl_set_cg_trim(&c, need));
  assert(!lqg_ctrl_set_cg_trim(&c, bad));
  assert(!lqg_ctrl_set_cg_trim(&c, huge));
  assert(c.cg_trim[0] == 30.0F && c.cg_trim[3] == -30.0F);
  /* o trim sobrevive ao armar e a soma do trim da tabela é ~0 (não muda a base) */
  assert(lqg_ctrl_arm(&c));
  assert(c.cg_trim[1] == 30.0F);
  float sum = 0.0F;
  for (uint32_t i = 0U; i < 4U; ++i) {
    sum += LQG_HOVER_TRIM_PERMILLE[i];
  }
  assert(fabsf(sum) < 1.0F);
  memcpy(g_plant_trim, saved, sizeof(saved));
}

static void test_cg_trim_scale(void) {
  assert(lqg_cg_trim_scale(0.0F) == 0.0F);
  assert(fabsf(lqg_cg_trim_scale(LQG_CG_TRIM_REF_PERMILLE) - 1.0F) < 1e-6F);
  /* 52 % em vez de 55 %: o trim fica menor; 62 %: maior; teto da razão em 1,25 */
  assert(lqg_cg_trim_scale(520.0F) < 0.82F);
  assert((lqg_cg_trim_scale(620.0F) > 1.55F) && (lqg_cg_trim_scale(620.0F) < 1.70F));
  const float cap = LQG_CG_TRIM_RATIO_MAX * LQG_CG_TRIM_RATIO_MAX;
  assert(fabsf(lqg_cg_trim_scale(2000.0F) - (cap * cap)) < 1e-4F);
}

int main(void) {
  test_cg_trim_scale();
  memcpy(g_plant_trim, LQG_HOVER_TRIM_PERMILLE, sizeof(g_plant_trim));
  test_cg_trim();
  test_params();
  test_arm_rules();
  test_motor_mapping();
  test_takeoff_hover_land();
  test_euler_fallback();
  test_height_change_live();
  test_hover_error_is_trimmed();
  test_ground_effect_climb();
  test_false_liftoff_hint();
  test_level_at_arm();
  test_liftoff_ignores_reading_offset();
  test_yaw_error_clamped();
  test_battery_compensation();
  test_mixer_priority();
  test_flow_height_gate();
  test_goto();
  test_failsafes();
  test_flow_spikes();
  test_heading_frame();
  test_integrators();
  test_gain_multipliers();
  puts("test_lqg_ctrl ok");
  return 0;
}
