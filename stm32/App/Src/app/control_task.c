/**
 * @file control_task.c
 * @brief Task do controle a 100 Hz (contrato em app/control_task.h).
 */
#include "app/control_task.h"

#include "bno08x.h"
#include "control/lqg_config.h"
#include "control/lqg_sim.h"
#include "link/bench_proto.h"
#include "motors.h"
#include "mtf01.h"
#include "power.h"
#include "telemetry.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

/** Período do controle, em ms (Ts = 0,01 s do modelo gerado). */
#define PERIOD_MS 10U
/** Uma mensagem CTRL a cada N passos (20 Hz). */
#define TLM_EVERY 5U

/** Pedidos deixados por outras tasks. */
#define REQ_TAKEOFF 1U
#define REQ_LAND    2U
#define REQ_GOTO    3U
#define REQ_HOLD    4U

/** Aceleração em que a planta do FLT_SIM paira, em ‰ (o parâmetro de hover é 550 por padrão). */
#define SIM_HOVER_PERMILLE 560.0F

/** Estado do controle; só a task escreve. */
static lqg_ctrl_t ctrl;

/** Pedidos e parâmetros pendentes; protegidos por seção crítica. */
static volatile uint8_t request;
static lqg_params_t pending_params;
static volatile bool params_pending;
static lqg_gains_t pending_gains;
static volatile bool gains_pending;
/** Alvo pendente de REQ_GOTO: x, y (m), altura (m), guinada (rad), velocidade (m/s). */
static float pending_goto[5];
/** Altura da decolagem pedida pela Pi, em m; 0 = a do parâmetro. */
static float pending_takeoff_m;
/** Fase e prontidão publicadas pela task para validar pedidos. */
static volatile lqg_state_t published_state;
static volatile bool published_ready;
/** Fotografia do último passo para os comandos da Pi; protegida por seção crítica. */
static control_snapshot_t published_snapshot;

#if FLT_SIM
/** Planta simulada: substitui os sensores; os motores reais recebem zero. */
static lqg_sim_t sim;
#endif

/** @brief Copia as leituras dos drivers para a entrada do controle (SI). */
static void read_sensors(lqg_sensors_t *in) {
#if FLT_SIM
  lqg_sim_sensors(&sim, in);
#else
  static bno_sample_t bno;
  static mtf_sample_t mtf;
  bno_get_sample(&bno);
  mtf_get_sample(&mtf);
  in->roll = bno.euler.angle.roll;
  in->pitch = bno.euler.angle.pitch;
  in->yaw = bno.euler.angle.yaw;
  for (uint32_t i = 0U; i < 3U; ++i) {
    in->gyro[i] = bno.gyro[i];
  }
  for (uint32_t i = 0U; i < 3U; ++i) {
    in->accel[i] = bno.lin_accel[i];
  }
  for (uint32_t i = 0U; i < 4U; ++i) {
    in->q[i] = bno.q[i];
  }
  in->q_valid = bno.online && bno.rotation.seen;
  in->att_accuracy = bno.rotation.accuracy;
  in->attitude_valid = bno.online && bno.euler_valid;
  in->gyro_valid = bno.online;
  in->accel_valid = bno.online && bno.acceleration.seen;
  in->range_m = (float)mtf.distance_mm * 0.001F;
  in->range_valid = mtf.range_valid;
  /* O MTF-01 informa o fluxo em cm/s a 1 m: dividido por 100 é rad/s. */
  in->flow_x = (float)mtf.flow_x * 0.01F;
  in->flow_y = (float)mtf.flow_y * 0.01F;
  in->flow_valid = mtf.flow_valid;
  in->flow_quality = mtf.quality;
  in->mtf_frames = mtf.frames;
  power_sample_t p;
  power_get(&p);
  in->vbat_v = p.vbat_v;
  in->vbat_valid = p.vbat_read && p.vbat_calibrated;
#endif
}

bool control_arm(void) {
  static bno_sample_t bno;
  static mtf_sample_t mtf;
  bno_get_sample(&bno);
  mtf_get_sample(&mtf);
  bool ok = true;
  if (FLT_SIM != 0) {
    ok = motors_arm(MOTORS_OWNER_CONTROL);
  } else if (!(bno.online && bno.euler_valid)) {
    telemetry_text(BL_TEXT_WARN, "ARM controle recusado: BNO sem dado");
    ok = false;
  } else if (!mtf.range_valid) {
    telemetry_text(BL_TEXT_WARN, "ARM controle recusado: MTF sem distancia");
    ok = false;
  } else {
    ok = motors_arm(MOTORS_OWNER_CONTROL);
    if (!ok) {
      telemetry_text(BL_TEXT_WARN, "ARM controle recusado: saida ocupada ou nao zerada");
    }
  }
  return ok;
}

bool control_takeoff(void) {
  return control_takeoff_to(0.0F);
}

bool control_takeoff_to(float height_m) {
  const bool sane = (height_m == 0.0F) ||
                    ((height_m >= LQG_PARAM_HEIGHT_MIN_M) && (height_m <= LQG_PARAM_HEIGHT_MAX_M));
  taskENTER_CRITICAL();
  const bool ok = sane && (published_state == LQG_STATE_ARMED) && published_ready;
  if (ok) {
    pending_takeoff_m = height_m;
    request = REQ_TAKEOFF;
  }
  taskEXIT_CRITICAL();
  return ok;
}

bool control_land(void) {
  taskENTER_CRITICAL();
  const lqg_state_t s = published_state;
  const bool ok = (s == LQG_STATE_SPOOL) || (s == LQG_STATE_CLIMB) || (s == LQG_STATE_HOVER);
  if (ok) {
    request = REQ_LAND;
  }
  taskEXIT_CRITICAL();
  return ok;
}

bool control_goto(float x, float y, float height_m, float psi, float speed) {
  const bool sane = (isfinite(x) != 0) && (isfinite(y) != 0) && (isfinite(height_m) != 0) &&
                    (isfinite(psi) != 0) && (isfinite(speed) != 0) && (fabsf(x) <= LQG_XY_LIMIT_M) &&
                    (fabsf(y) <= LQG_XY_LIMIT_M) && (height_m >= LQG_PARAM_HEIGHT_MIN_M) &&
                    (height_m <= LQG_PARAM_HEIGHT_MAX_M) && (speed >= 0.0F) && (speed <= 0.5F);
  taskENTER_CRITICAL();
  const lqg_state_t s = published_state;
  const bool ok = sane && ((s == LQG_STATE_CLIMB) || (s == LQG_STATE_HOVER));
  if (ok) {
    pending_goto[0] = x;
    pending_goto[1] = y;
    pending_goto[2] = height_m;
    pending_goto[3] = psi;
    pending_goto[4] = speed;
    request = REQ_GOTO;
  }
  taskEXIT_CRITICAL();
  return ok;
}

bool control_hold(void) {
  taskENTER_CRITICAL();
  const lqg_state_t s = published_state;
  const bool ok = (s == LQG_STATE_CLIMB) || (s == LQG_STATE_HOVER);
  if (ok) {
    request = REQ_HOLD;
  }
  taskEXIT_CRITICAL();
  return ok;
}

void control_get_snapshot(control_snapshot_t *out) {
  taskENTER_CRITICAL();
  *out = published_snapshot;
  taskEXIT_CRITICAL();
}

bool control_set_params(const lqg_params_t *params) {
  const bool ok = lqg_params_valid(params);
  if (ok) {
    taskENTER_CRITICAL();
    pending_params = *params;
    params_pending = true;
    taskEXIT_CRITICAL();
  }
  return ok;
}

bool control_set_gains(const lqg_gains_t *gains) {
  const bool ok = lqg_gains_valid(gains);
  if (ok) {
    taskENTER_CRITICAL();
    pending_gains = *gains;
    gains_pending = true;
    taskEXIT_CRITICAL();
  }
  return ok;
}

/** @brief Aplica parâmetros e pedidos que outras tasks deixaram. */
static void take_requests(void) {
  taskENTER_CRITICAL();
  const uint8_t req = request;
  request = 0U;
  const bool has_params = params_pending;
  const lqg_params_t params = pending_params;
  params_pending = false;
  const bool has_gains = gains_pending;
  const lqg_gains_t gains = pending_gains;
  gains_pending = false;
  float go[5];
  for (uint32_t i = 0U; i < 5U; ++i) {
    go[i] = pending_goto[i];
  }
  const float takeoff_m = pending_takeoff_m;
  taskEXIT_CRITICAL();

  if (has_gains) {
    (void)lqg_ctrl_set_gains(&ctrl, &gains);
  }

  if (has_params && lqg_ctrl_set_params(&ctrl, &params)) {
    (void)motors_control_cap((uint16_t)(params.cap_permille + 0.5F));
  }
  if (req == REQ_TAKEOFF) {
    if (lqg_ctrl_takeoff(&ctrl)) {
      if (takeoff_m > 0.0F) {
        ctrl.h_goal = takeoff_m; /* alvo da Pi; o parâmetro da IHM fica como está */
      }
      telemetry_text(BL_TEXT_INFO, "decolagem: subindo a aceleracao");
    } else {
      telemetry_text(BL_TEXT_WARN, "decolagem recusada: sem chao medido ou sensor invalido");
    }
  } else if (req == REQ_LAND) {
    if (lqg_ctrl_land(&ctrl)) {
      telemetry_text(BL_TEXT_INFO, "pouso");
    }
  } else if (req == REQ_GOTO) {
    if (!lqg_ctrl_goto(&ctrl, go[0], go[1], go[2], go[3], go[4])) {
      telemetry_text(BL_TEXT_WARN, "alvo recusado: fora de voo ou fora dos limites");
    }
  } else if (req == REQ_HOLD) {
    (void)lqg_ctrl_hold(&ctrl);
  } else {
    /* sem pedido */
  }
}

/** @brief Envia a mensagem CTRL com o resultado de um passo. */
static void send_ctrl(const lqg_out_t *out, uint8_t last_fail) {
  bl_ctrl_t m;
  (void)memset(&m, 0, sizeof(m));
  m.t_ms = HAL_GetTick();
  m.state = (uint8_t)out->state;
  m.flags = out->flags;
  m.fail = (out->fail != LQG_FAIL_NONE) ? out->fail : last_fail;
  for (uint32_t i = 0U; i < 9U; ++i) {
    m.y[i] = out->y[i];
  }
  for (uint32_t i = 0U; i < 3U; ++i) {
    m.pos[i] = out->pos[i];
    m.vel[i] = out->vel[i];
  }
  m.h_ref = out->h_ref;
  m.trim = out->trim;
  m.base = out->base;
  m.ref_x = out->ref_xy[0];
  m.ref_y = out->ref_xy[1];
  m.ref_psi = out->ref_psi;
  m.vbat_factor = out->vbat_factor;
  uint8_t payload[BL_MAX_PAYLOAD];
  (void)telemetry_send(BL_MSG_CTRL, payload, bl_encode_ctrl(&m, payload));
}

/** @brief Escreve os motores conforme o resultado do passo. */
static void drive_motors(const lqg_out_t *out, bool *need_release) {
  if (out->stop != NULL) {
    motors_stop(out->stop);
    *need_release = true;
    telemetry_text((out->fail == LQG_FAIL_LANDED) ? BL_TEXT_INFO : BL_TEXT_ERROR, out->stop);
  } else {
    uint16_t pm[MOTORS_COUNT];
    for (uint32_t i = 0U; i < MOTORS_COUNT; ++i) {
      const float v = out->permille[i];
      /* FLT_SIM: a planta recebe o comando; os motores reais, zero (DShot 0 não gira). */
      pm[i] = ((v > 0.0F) && (FLT_SIM == 0)) ? (uint16_t)(v + 0.5F) : 0U;
    }
    /* Mesmo armado e em zero, o controle renova o prazo de 100 ms do árbitro. */
    (void)motors_control_set(pm);
  }
}

static void control_task(void *argument) {
  (void)argument;
  lqg_ctrl_init(&ctrl);
  {
    /* Sem IHM (voo só pela Pi) a placa já liga com os valores testados com cabo. */
    const lqg_params_t boot_params = LQG_BOOT_PARAMS;
    const lqg_gains_t boot_gains = LQG_BOOT_GAINS;
    (void)lqg_ctrl_set_params(&ctrl, &boot_params);
    (void)lqg_ctrl_set_gains(&ctrl, &boot_gains);
    (void)motors_control_cap((uint16_t)(boot_params.cap_permille + 0.5F));
  }
  bool need_release = false;
  uint8_t last_fail = LQG_FAIL_NONE;
  lqg_state_t last_state = LQG_STATE_IDLE;
  uint32_t tlm = 0U;
  const char *note = NULL;
  uint32_t note_seq = 0U;
  uint32_t step = 0U;
  bool note_critical = false;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));
    const motors_owner_t owner = motors_owner();
    if (owner != MOTORS_OWNER_CONTROL) {
      need_release = false;
      if (lqg_ctrl_state(&ctrl) != LQG_STATE_IDLE) {
        lqg_ctrl_abort(&ctrl);
      }
    } else if ((lqg_ctrl_state(&ctrl) == LQG_STATE_IDLE) && !need_release) {
      (void)lqg_ctrl_arm(&ctrl);
      last_fail = LQG_FAIL_NONE;
#if FLT_SIM
      lqg_sim_init(&sim, SIM_HOVER_PERMILLE);
      telemetry_text(BL_TEXT_WARN, "FLT_SIM: sensores simulados, motores em zero");
#endif
      telemetry_text(BL_TEXT_INFO, "controle armado: medindo o chao, motores em zero");
    } else {
      /* dono e fase em acordo */
    }
    take_requests();

    lqg_sensors_t in;
    lqg_out_t out;
    read_sensors(&in);
    lqg_ctrl_step(&ctrl, &in, &out);
    if (out.fail != LQG_FAIL_NONE) {
      last_fail = out.fail;
    }
    if (owner == MOTORS_OWNER_CONTROL) {
      drive_motors(&out, &need_release);
      if (out.note != NULL) {
        telemetry_text(BL_TEXT_INFO, out.note);
      }
#if FLT_SIM
      if ((out.stop == NULL) && (out.state >= LQG_STATE_SPOOL)) {
        lqg_sim_step(&sim, out.permille);
      }
#endif
    }

    if (owner == MOTORS_OWNER_CONTROL) {
      if (out.stop != NULL) {
        note = out.stop;
        note_critical = out.fail != LQG_FAIL_LANDED;
        ++note_seq;
      } else if (out.note != NULL) {
        note = out.note;
        note_critical = false;
        ++note_seq;
      } else {
        /* sem aviso novo */
      }
    }
    control_snapshot_t snap;
    ++step;
    snap.step = step;
    snap.t_ms = HAL_GetTick();
    for (uint32_t i = 0U; i < 9U; ++i) {
      snap.y[i] = out.y[i];
    }
    for (uint32_t i = 0U; i < 12U; ++i) {
      snap.x_hat[i] = ctrl.est[i];
    }
    for (uint32_t i = 0U; i < 4U; ++i) {
      snap.permille[i] = out.permille[i];
    }
    snap.base = out.base * out.vbat_factor;
    snap.note = note;
    snap.note_seq = note_seq;
    snap.note_critical = note_critical;
    snap.state = lqg_ctrl_state(&ctrl);
    snap.armed = owner == MOTORS_OWNER_CONTROL;
    snap.airborne = ctrl.airborne;
    snap.flags = out.flags;
    snap.fail = last_fail;
    snap.simulated = FLT_SIM != 0;
    lqg_ctrl_nav(&ctrl, &snap.nav);
    snap.phi = out.y[LQG_Y_PHI];
    snap.theta = out.y[LQG_Y_THETA];
    snap.p = out.y[LQG_Y_P];
    snap.q = out.y[LQG_Y_Q];
    snap.vbat_factor = out.vbat_factor;
    taskENTER_CRITICAL();
    published_state = snap.state;
    published_ready = (ctrl.ground_n >= LQG_GROUND_SAMPLES) &&
                      ((ctrl.last_flags & (LQG_MEAS_ATTITUDE | LQG_MEAS_GYRO | LQG_MEAS_RANGE)) ==
                       (LQG_MEAS_ATTITUDE | LQG_MEAS_GYRO | LQG_MEAS_RANGE));
    published_snapshot = snap;
    taskEXIT_CRITICAL();

    if ((++tlm >= TLM_EVERY) || (out.state != last_state)) {
      tlm = 0U;
      send_ctrl(&out, last_fail);
    }
    last_state = out.state;
  }
}

/** Controle da task (alocação estática). */
static StaticTask_t control_tcb;
/** Pilha da task, em words (o EKF3 usa matrizes 15x15 float na pilha). */
static StackType_t control_stack[2048];

bool control_task_start(void) {
  return xTaskCreateStatic(control_task, "Control",
                           sizeof(control_stack) / sizeof(control_stack[0]), NULL,
                           tskIDLE_PRIORITY + 4U, control_stack, &control_tcb) != NULL;
}
