/**
 * @file flight_ctrl.c
 * @brief Malha de voo e máquina de estados (contrato em control/flight_ctrl.h).
 *
 * Divisão de responsabilidades entre tasks:
 *
 * - As funções públicas (chamadas pelo console USB e pelo DroneLink) só
 *   validam o pedido contra o estado atual e o colocam numa fila.
 * - A task do voo é a única que muda o estado, a referência e o filtro.
 *   Ela atende a fila no começo de cada passo, então um pedido leva no máximo
 *   10 ms para ter efeito.
 */
#include "control/flight_ctrl.h"

#include "control/ctrl_gains.h"
#include "control/attitude_mode.h"
#include "control/ctrl_params.h"
#include "control/lqg_tuning.h"
#include "control/lqg_dist.h"
#include "control/meas.h"
#include "control/sim_plant.h"
#include "control/throttle.h"
#include "control/trim.h"
#include "attitude.h"
#include "bno08x.h"
#include "flight_config.h"
#include "main.h"
#include "control/mixer.h"
#include "motors.h"
#include "mtf01.h"
#include "sensor_log.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/** Passos seguidos sem BNO que disparam o failsafe (50 ms). */
#define BNO_TIMEOUT_STEPS 5U
/** Tempo sem telêmetro, em voo, que dispara o pouso, em ms. */
#define MTF_LOST_MS 300U
/** Tempo em MOT_IDLE depois de armar, antes de a decolagem começar, em ms. */
#define SPINUP_MS 1000U
/** Armado no chão sem decolar por este tempo, desarma, em ms. */
#define ARMED_IDLE_TIMEOUT_MS 15000U
/** Tempo perto do chão que encerra o pouso, em ms. */
#define LAND_TOUCH_MS 500U
/** Pouso sem telêmetro: tempo extra depois de a rampa chegar ao chão, em ms. */
#define LAND_BLIND_EXTRA_MS 3000U
/** Menor velocidade de descida no pouso, em m/s (FLT_Z_RATE = 0 seria um degrau). */
#define LAND_MIN_RATE_M_S FLIGHT_LAND_MIN_RATE_M_S
/** Tolerância padrão de posição para concluir decolagem e trechos, em m. */
#define DEFAULT_POS_TOL_M 0.05F
/** Tolerância padrão de guinada, em rad. */
#define DEFAULT_YAW_TOL_RAD 0.05F
/** Intervalo mínimo entre avisos de fluxo do MTF ausente com o XY ligado, em ms. */
#define FLOW_LOST_REPORT_MS 2000U
/** Erro de altura que ainda conta como "alvo acima" para a rampa do estimador no chão, em m. */
#define DIST_GROUND_ERROR_M 0.02F
/** Intervalo do relato do trim no log, em ms. */
#define TRIM_REPORT_MS 2000U
/** Pedidos que cabem na fila. */
#define REQUEST_QUEUE_DEPTH 4U
/** Distância máxima do ponto de decolagem aceita em um setpoint, em m. */
#define SETPOINT_XY_MAX_M 3.0F
/** Período do passo, em s. */
#define DT_S ((float)LQG_TS_S)

/** @brief Tipos de pedido vindos de outras tasks. */
typedef enum {
  REQ_ARM,      /**< Armar. */
  REQ_DISARM,   /**< Desarmar. */
  REQ_TAKEOFF,  /**< Decolar. */
  REQ_LEG,      /**< Executar um trecho. */
  REQ_HOLD,     /**< Parar e manter. */
  REQ_LAND,     /**< Pousar. */
  REQ_CANCEL,   /**< Cancelar o trecho ativo. */
  REQ_XY_ZERO   /**< Zerar a posição horizontal: o ponto atual vira o (0, 0). */
} request_type_t;

/** @brief Pedido na fila. */
typedef struct {
  request_type_t type;      /**< Tipo do pedido. */
  flight_origin_t origin;   /**< REQ_ARM. */
  bool dry;                 /**< REQ_ARM: ensaio a seco. */
  float height;             /**< REQ_TAKEOFF: altura, em m. */
  float pos_tol;            /**< REQ_TAKEOFF: tolerância, em m (0 = padrão). */
  uint32_t timeout_ms;      /**< REQ_TAKEOFF: tempo máximo (0 = sem limite). */
  flight_leg_t leg;         /**< REQ_LEG. */
  bool height_mode;         /**< REQ_LEG: leg.z é a altura acima do chão, não o z local. */
  flight_reason_t reason;   /**< REQ_HOLD e REQ_LAND. */
  const char *why;          /**< REQ_DISARM. */
} request_t;

/** @brief Alvo em andamento (decolagem, trecho ou pouso). */
typedef struct {
  bool active;              /**< Há um alvo sendo perseguido. */
  float target[LQG_NR];     /**< Referência final. */
  float speed_xy;           /**< Rampa horizontal, em m/s (0 = degrau). */
  float pos_tol;            /**< Tolerância de posição, em m. */
  float yaw_tol;            /**< Tolerância de guinada, em rad. */
  bool check_xy;            /**< Conferir x e y ao concluir (não na decolagem). */
  uint32_t start_ms;        /**< Início. */
  uint32_t timeout_ms;      /**< Tempo máximo (0 = sem limite). */
} goal_t;

/* ---- Compartilhado com outras tasks ----------------------------------- */

/** Estado atual (::flight_state_t); só a task do voo escreve. */
static volatile uint8_t state = FLIGHT_STATE_BOOT;
/** Armado em ensaio a seco. */
static volatile bool dry_run;
/** Há um pedido de armar na fila: a decolagem já pode ser pedida. */
static volatile bool arm_pending;
/** Fila de pedidos. */
static QueueHandle_t requests;
/** Controle da fila (alocação estática). */
static StaticQueue_t requests_memory;
/** Armazenamento da fila. */
static uint8_t requests_storage[REQUEST_QUEUE_DEPTH * sizeof(request_t)];
/** Fotografia do último passo; protegida por seção crítica. */
static flight_telemetry_t telemetry;
/** Evento de fim de trecho pendente; protegido por seção crítica. */
static flight_leg_event_t leg_event;
/** Há um evento em ::leg_event esperando o DroneLink. */
static bool leg_event_pending;

/* ---- Só a task do voo -------------------------------------------------- */

/** Filtro e controlador do TCC. */
static lqg_ert_t lqg;
/** Cópia dos ganhos em uso. */
static lqg_gains_t gains;
/** Ganhos aplicados neste passo: os de ::gains, sem os termos horizontais se CTL_XY = 0. */
static lqg_gains_t gains_effective;
/** Geração da cópia (control/ctrl_gains.h). */
static uint32_t gains_generation;
/** Conversão dos sensores. */
static meas_state_t meas;
/** Ajuste lento do hover e do equilíbrio (control/trim.h). */
static trim_state_t trim;
/** O drone já subiu ::FLIGHT_LIFTED_HEIGHT_M nesta decolagem. */
static bool lifted;
/** Tick em que a descida suave do pouso começou (0 = ainda não). */
static uint32_t land_spindown_ms;
/**
 * Equilíbrio (arfagem e rolagem do trim) do último instante em que o drone
 * estava em HOLD, na altura do alvo, nivelado e assentado. O próximo voo
 * começa dele, em vez de reaprender do zero com o lado leve levantando (ensaio
 * de 04/11). Vale até reiniciar a placa.
 */
static trim_state_t trim_learned;
/** ::trim_learned tem valor. */
static bool trim_learned_valid;
/** Kd/Kfd do LQG ajustado em uso: os gerados ou os do painel (control/lqg_tuning.h). */
static lqg_aj_gains_t lqg_aj_gains;
/** Geração de ::lqg_aj_gains. */
static uint32_t lqg_aj_generation;
/** O estimador de perturbação está em uso neste passo (CTL_DIST). */
static bool dist_active;
/** Estimador de perturbação do "LQG melhorado" (control/lqg_dist.h). */
static lqg_dist_t dist;
/** O drone já se soltou do chão neste voo: o estimador pode aprender. */
static bool dist_released;
/** O XY já entrou neste voo (drone fora do chão); a posição foi zerada ali. */
static bool xy_engaged;
/** Tick do último aviso de fluxo ausente com o XY ligado. */
static uint32_t flow_lost_said_ms;
/** Altura medida (z NED) deste passo, em m. */
static float meas_z_now;
/** A altura medida deste passo é válida. */
static bool meas_range_ok;
/** Altura medida (z NED) filtrada, em m (::FLIGHT_MEAS_Z_TAU_S). */
static float meas_z_smooth;
/** Velocidade vertical (z NED) derivada de ::meas_z_smooth e filtrada, em m/s. */
static float meas_vz;
/** ::meas_z_smooth já tem valor neste armar. */
static bool meas_z_tracking;
/** Tick em que a altura parou de acompanhar o alvo na subida (0 = acompanhando). */
static uint32_t stuck_since_ms;
/** Altura filtrada no começo da contagem do travamento (z NED), em m. */
static float stuck_z0;
/** Hover do ajuste lento quando o travamento começou, em rad/s. */
static float stuck_trim_z;
/** Perturbação de altura estimada quando o travamento começou, em rad/s. */
static float stuck_dist_z;
/** Tick do último relato do trim no log. */
static uint32_t trim_report_ms;
/** O drone já se soltou do chão nesta decolagem (encerra a rampa de partida do hover). */
static bool trim_released;
/** Parâmetros deste passo. */
static ctrl_params_t prm;
/** Referência em uso (avança em rampa até o alvo). */
static float ref[LQG_NR];
/** Alvo em andamento. */
static goal_t goal;
/**
 * Altura (z NED) do chão onde armou, em m: média do segundo em marcha lenta.
 * Lida também pelo DroneLink (::leg_height_ok, float de 32 bits: leitura atômica).
 */
static volatile float z_ground;
/** Leituras do MTF que entraram na média de ::z_ground (0 = ainda não armou). */
static uint32_t ground_samples;
/** Quem armou; lido também pelo DroneLink (flight_ctrl_origin). */
static volatile flight_origin_t origin;
/** Tick em que armou. */
static uint32_t armed_ms;
/** Tick em que o pouso começou. */
static uint32_t land_start_ms;
/** Tick em que o drone chegou perto do chão (0 = ainda não). */
static uint32_t land_touch_ms;
/** Altura acima do chão no começo do pouso, em m. */
static float land_start_height;
/** Passos seguidos sem BNO08x. */
static uint32_t bno_bad_steps;
/** Tick em que o telêmetro parou de responder (0 = respondendo). */
static uint32_t mtf_lost_since_ms;
/** Fração acumulada para a linha EST. */
static float est_accumulator;
/** Última amostra do BNO08x (estática: grande para a pilha). */
static bno_sample_t bno;
/** Última amostra do MTF-01. */
static mtf_sample_t mtf;
/** Drone simulado (FLT_SIM, control/sim_plant.h). */
static sim_plant_t sim;
/** O drone simulado está em uso neste passo. */
static bool sim_active;
/** Tick desde quando a decolagem ou o trecho está no alvo e parado (0 = não está). */
static uint32_t settle_since_ms;
/**
 * Entrada do drone simulado neste passo, em rad/s por entrada u1..u4: o que
 * os motores recebem sem o desequilíbrio fixo (du aplicado + perturbação
 * estimada). O simulado é o drone com exatamente o desequilíbrio de
 * CTL_FF_TH/PHI/PSI, que a parte fixa cancela.
 */
static float sim_input[LQG_NU];

/* ---- Utilidades -------------------------------------------------------- */

/**
 * @brief Os motores (ou o drone simulado) seguem o controle neste voo.
 *
 * No ensaio a seco nada se move e o controle não pode aprender nada; com o
 * drone simulado, o modelo recebe o que os motores receberiam.
 */
static bool motors_follow(void) { return !dry_run || sim_active; }

/** @brief Monta um resultado de pedido. */
static flight_request_result_t result(flight_request_status_t status, flight_reason_t reason) {
  const flight_request_result_t r = {.status = status, .reason = reason};
  return r;
}

/** @brief Estados com o drone armado (motores podem girar). */
static bool is_armed_state(uint8_t s) {
  return s == FLIGHT_STATE_ARMED || s == FLIGHT_STATE_TAKEOFF || s == FLIGHT_STATE_HOLD ||
         s == FLIGHT_STATE_EXECUTING_LEG || s == FLIGHT_STATE_LAND;
}

/** @brief Estados em que o drone pode estar no ar. */
static bool is_airborne_state(uint8_t s) {
  return s == FLIGHT_STATE_TAKEOFF || s == FLIGHT_STATE_HOLD ||
         s == FLIGHT_STATE_EXECUTING_LEG || s == FLIGHT_STATE_LAND;
}

/** @brief Escreve uma linha de texto formatada no log. */
static void say(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void say(const char *fmt, ...) {
  char text[61];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);
  log_text(text);
}

/** @brief Coloca um pedido na fila sem esperar. */
static bool post(const request_t *req) {
  return requests != NULL && xQueueSend(requests, req, 0) == pdPASS;
}

/** @brief Aproxima @p cur de @p goal em no máximo @p max_step (<= 0: salta). */
static float approach(float cur, float target, float max_step) {
  if (!(max_step > 0.0F)) return target;
  const float d = target - cur;
  if (d > max_step) return cur + max_step;
  if (d < -max_step) return cur - max_step;
  return target;
}

/* ---- API pública (outras tasks) ---------------------------------------- */

bool flight_ctrl_is_armed(void) { return is_armed_state(state); }

flight_origin_t flight_ctrl_origin(void) { return origin; }

flight_request_result_t flight_ctrl_arm(flight_origin_t from) {
  const uint8_t s = state;
  if (s != FLIGHT_STATE_STANDBY && s != FLIGHT_STATE_FAILSAFE) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BAD_STATE);
  }
  flight_telemetry_t t;
  flight_ctrl_get_telemetry(&t);
  ctrl_params_t p;
  ctrl_params_snapshot(&p);
  /* O fluxo óptico não é exigido nem com CTL_XY: no chão o MTF fica perto
     demais (8–10 cm) e o fluxo só vale em 13–30% das leituras (06/11); a
     partir de ~2 cm de altura, em 97–100%. O XY só entra depois que o drone
     sai do chão (ver step()). */
  const uint8_t needed = MEAS_ATTITUDE | MEAS_GYRO | MEAS_RANGE;
  if ((t.meas_flags & needed) != needed) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_SENSOR_UNHEALTHY);
  }
  /* Simulado, os motores nunca giram. */
  const bool dry = p.dry_run >= 0.5F || !LQG_FLIGHT_ENABLED || p.sim >= 0.5F;
  if (!dry && !motors_arm(MOTORS_OWNER_FLIGHT)) {
    return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
  }
  const request_t req = {.type = REQ_ARM, .origin = from, .dry = dry};
  arm_pending = true;
  if (!post(&req)) {
    arm_pending = false;
    if (!dry) motors_stop("fila do voo cheia");
    return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
  }
  return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
}

void flight_ctrl_disarm(const char *why) {
  /* Os motores param já, sem esperar o próximo passo do voo. */
  if (motors_owner() == MOTORS_OWNER_FLIGHT) motors_stop(why);
  const request_t req = {.type = REQ_DISARM, .why = why};
  (void)post(&req);
}

/** @brief Armado no chão, ou com o armar ainda na fila (a decolagem vem depois dele). */
static bool can_take_off(void) {
  return state == FLIGHT_STATE_ARMED || arm_pending;
}

flight_request_result_t flight_ctrl_takeoff(float height_m) {
  if (!can_take_off()) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BAD_STATE);
  }
  if (!(height_m >= 0.0F && height_m <= FLIGHT_MAX_HEIGHT_M + FLIGHT_HEIGHT_TOL_M)) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_OUT_OF_BOUNDS);
  }
  const request_t req = {.type = REQ_TAKEOFF, .height = height_m};
  if (!post(&req)) return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
  return result(FLIGHT_REQUEST_IN_PROGRESS, FLIGHT_REASON_NONE);
}

/**
 * @brief A altura final do trecho fica entre a de pouso e ::FLIGHT_MAX_HEIGHT_M?
 *
 * A mesma conta de goal_from_leg(), feita já no pedido: sem ela a Pi veria
 * "em andamento" e logo depois "falhou". Usa a referência da última
 * telemetria e o chão de quando armou (escritos pela task do voo).
 */
static bool leg_height_ok(const flight_leg_t *leg) {
  if (leg->action == FLIGHT_ACTION_ROTATE_RELATIVE) return true;
  const float ground = z_ground;
  if (!isfinite(ground)) return true;  /* a task do voo confere de novo */
  flight_telemetry_t t;
  flight_ctrl_get_telemetry(&t);
  ctrl_params_t p;
  ctrl_params_snapshot(&p);
  const bool absolute = leg->action == FLIGHT_ACTION_GOTO_LOCAL &&
                        leg->frame != FLIGHT_FRAME_BODY_FRD;
  const float z = absolute ? leg->z : t.ref[LQG_R_Z] + leg->z;
  return isfinite(z) && z <= ground - p.land_height &&
         z >= ground - (FLIGHT_MAX_HEIGHT_M + FLIGHT_HEIGHT_TOL_M);
}

flight_request_result_t flight_ctrl_leg(const flight_leg_t *leg) {
  if (leg->action == FLIGHT_ACTION_TAKEOFF_RELATIVE) {
    /* Em FRD, z positivo é para baixo: subir h metros é z = -h. */
    if (!can_take_off()) {
      return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BAD_STATE);
    }
    if (!(leg->z < 0.0F && leg->z >= -(FLIGHT_MAX_HEIGHT_M + FLIGHT_HEIGHT_TOL_M))) {
      return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_OUT_OF_BOUNDS);
    }
    const request_t req = {.type = REQ_TAKEOFF, .height = -leg->z,
                           .pos_tol = leg->position_tolerance,
                           .timeout_ms = leg->timeout_ms};
    if (!post(&req)) return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
    return result(FLIGHT_REQUEST_IN_PROGRESS, FLIGHT_REASON_NONE);
  }
  if (leg->action == FLIGHT_ACTION_HOLD) return flight_ctrl_hold(FLIGHT_REASON_NONE);
  if (leg->action == FLIGHT_ACTION_LAND) return flight_ctrl_land(FLIGHT_REASON_NONE);
  if (state == FLIGHT_STATE_EXECUTING_LEG || state == FLIGHT_STATE_TAKEOFF) {
    return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
  }
  if (state != FLIGHT_STATE_HOLD) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BAD_STATE);
  }
  if (!leg_height_ok(leg)) {
    /* Recusa já na primeira resposta (a task do voo também confere, em goal_from_leg). */
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_OUT_OF_BOUNDS);
  }
  request_t req = {.type = REQ_LEG};
  req.leg = *leg;
  if (!post(&req)) return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
  return result(FLIGHT_REQUEST_IN_PROGRESS, FLIGHT_REASON_NONE);
}

/**
 * @brief Um setpoint pode entrar agora?
 *
 * Em HOLD e com um trecho em andamento, sim (troca o alvo). Na decolagem, só
 * depois da rampa de partida e com o drone fora do chão: se ele não alcança
 * a altura pedida (W0 baixo, cabo segurando), a decolagem não termina, e sem
 * isso X, Y e a guinada do setpoint nunca entrariam (06/11).
 */
static bool setpoint_allowed(uint32_t now) {
  if (state == FLIGHT_STATE_HOLD || state == FLIGHT_STATE_EXECUTING_LEG) return true;
  return state == FLIGHT_STATE_TAKEOFF && lifted &&
         now - armed_ms >= SPINUP_MS + FLIGHT_TAKEOFF_RAMP_MS;
}

flight_request_result_t flight_ctrl_setpoint(float x_m, float y_m, float height_m, float yaw_rad) {
  if (!setpoint_allowed(HAL_GetTick())) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BAD_STATE);
  }
  if (!(fabsf(x_m) <= SETPOINT_XY_MAX_M && fabsf(y_m) <= SETPOINT_XY_MAX_M && isfinite(yaw_rad))) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_OUT_OF_BOUNDS);
  }
  request_t req = {.type = REQ_LEG, .height_mode = true};
  req.leg = (flight_leg_t){.action = FLIGHT_ACTION_GOTO_LOCAL, .frame = FLIGHT_FRAME_LOCAL_FRD,
                           .x = x_m, .y = y_m, .z = height_m, .yaw = yaw_rad};
  if (!post(&req)) return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
  return result(FLIGHT_REQUEST_IN_PROGRESS, FLIGHT_REASON_NONE);
}

flight_request_result_t flight_ctrl_xy_zero(void) {
  const request_t req = {.type = REQ_XY_ZERO};
  if (!post(&req)) return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
  return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
}

flight_request_result_t flight_ctrl_hold(flight_reason_t reason) {
  if (!is_airborne_state(state)) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_BAD_STATE);
  }
  const request_t req = {.type = REQ_HOLD, .reason = reason};
  if (!post(&req)) return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
  return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
}

flight_request_result_t flight_ctrl_land(flight_reason_t reason) {
  if (!is_armed_state(state)) {
    return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);  /* já está no chão */
  }
  const request_t req = {.type = REQ_LAND, .reason = reason};
  if (!post(&req)) return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
  return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
}

flight_request_result_t flight_ctrl_cancel(void) {
  if (state != FLIGHT_STATE_TAKEOFF && state != FLIGHT_STATE_EXECUTING_LEG) {
    return result(FLIGHT_REQUEST_DENIED, FLIGHT_REASON_SEQUENCE_ERROR);
  }
  const request_t req = {.type = REQ_CANCEL};
  if (!post(&req)) return result(FLIGHT_REQUEST_TEMPORARILY_REJECTED, FLIGHT_REASON_BUSY);
  return result(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
}

void flight_ctrl_get_telemetry(flight_telemetry_t *out) {
  taskENTER_CRITICAL();
  *out = telemetry;
  taskEXIT_CRITICAL();
}

bool flight_ctrl_take_leg_event(flight_leg_event_t *out) {
  bool pending;
  taskENTER_CRITICAL();
  pending = leg_event_pending;
  if (pending) {
    *out = leg_event;
    leg_event_pending = false;
  }
  taskEXIT_CRITICAL();
  return pending;
}

const char *flight_ctrl_state_name(uint8_t s) {
  static const char *const names[] = {"BOOT", "SELF_TEST", "STANDBY", "PREARM", "ARMED",
                                      "TAKEOFF", "HOLD", "LEG", "LAND", "FAILSAFE",
                                      "TERMINATION"};
  return s < sizeof(names) / sizeof(names[0]) ? names[s] : "?";
}

/* ---- Transições (só a task do voo) ------------------------------------ */

/**
 * @brief Encerra o alvo em andamento e, se ele era um trecho, avisa o DroneLink.
 * @param[in] status  Situação final.
 * @param[in] reason  Motivo.
 */
static void finish_goal(flight_request_status_t status, flight_reason_t reason) {
  const bool was_leg = goal.active &&
                       (state == FLIGHT_STATE_TAKEOFF || state == FLIGHT_STATE_EXECUTING_LEG);
  goal.active = false;
  if (!was_leg) return;
  taskENTER_CRITICAL();
  leg_event.status = status;
  leg_event.reason = reason;
  leg_event_pending = true;
  taskEXIT_CRITICAL();
}

/** @brief Fica parado na referência atual. */
static void enter_hold(void) {
  goal.active = false;
  memcpy(goal.target, ref, sizeof(ref));
  state = FLIGHT_STATE_HOLD;
}

/**
 * @brief Registra o ajuste lento aprendido e o zera.
 *
 * O hover real é CTL_W0 + trim.z: se o ajuste for sempre parecido, vale
 * passar esse valor para CTL_W0.
 */
static void report_and_reset_trim(void) {
  if (trim.z != 0.0F || trim.theta != 0.0F || trim.phi != 0.0F) {
    say("trim z=%d th=%d phi=%d rad/s", (int)trim.z, (int)trim.theta, (int)trim.phi);
  }
  if (dist.started[LQG_DIST_Z] || dist.started[LQG_DIST_THETA]) {
    say("dist z=%d th=%d phi=%d psi=%d rad/s", (int)lqg_dist_get(&dist, LQG_DIST_Z),
        (int)lqg_dist_get(&dist, LQG_DIST_THETA), (int)lqg_dist_get(&dist, LQG_DIST_PHI),
        (int)lqg_dist_get(&dist, LQG_DIST_PSI));
  }
  lqg_dist_reset(&dist);
  trim_reset(&trim);
  trim_released = false;
  land_spindown_ms = 0U;
}

/** @brief Desarma e volta a STANDBY. */
static void enter_standby(const char *why) {
  if (!dry_run && motors_owner() == MOTORS_OWNER_FLIGHT) motors_stop(why);
  finish_goal(FLIGHT_REQUEST_FAILED, FLIGHT_REASON_CANCELLED);
  if (is_armed_state(state)) say("voo: desarmado (%s)", why);
  report_and_reset_trim();
  state = FLIGHT_STATE_STANDBY;
}

/** @brief Corta os motores e entra em FAILSAFE. */
static void enter_failsafe(const char *why, flight_reason_t reason) {
  if (!dry_run) motors_stop(why);
  finish_goal(FLIGHT_REQUEST_FAILED, reason);
  say("FAILSAFE: %s", why);
  report_and_reset_trim();
  state = FLIGHT_STATE_FAILSAFE;
}

/** @brief Começa o pouso a partir da referência atual. */
static void enter_land(flight_reason_t reason) {
  finish_goal(FLIGHT_REQUEST_FAILED,
              reason == FLIGHT_REASON_NONE ? FLIGHT_REASON_CANCELLED : reason);
  memcpy(goal.target, ref, sizeof(ref));
  goal.target[LQG_R_Z] = z_ground;
  goal.active = true;
  goal.start_ms = HAL_GetTick();
  land_start_ms = goal.start_ms;
  land_touch_ms = 0U;
  land_spindown_ms = 0U;
  land_start_height = z_ground - ref[LQG_R_Z];
  state = FLIGHT_STATE_LAND;
}

/** @brief Começa a descida suave da aceleração (o desarme vem ao fim da rampa). */
static void begin_spindown(uint32_t now) {
  if (land_spindown_ms != 0U) return;
  land_spindown_ms = now != 0U ? now : 1U;
  say("pouso: reduzindo motores");
}

/**
 * @brief Converte um trecho no alvo local.
 * @return false se o trecho não faz sentido (altura abaixo do chão etc.).
 */
static bool goal_from_leg(const flight_leg_t *leg) {
  float t[LQG_NR];
  memcpy(t, ref, sizeof(t));
  const float psi = ref[LQG_R_PSI];
  const bool body = leg->frame == FLIGHT_FRAME_BODY_FRD;
  /* Deslocamento no corpo (frente, direita) -> referencial local. */
  const float dx = body ? leg->x * cosf(psi) - leg->y * sinf(psi) : leg->x;
  const float dy = body ? leg->x * sinf(psi) + leg->y * cosf(psi) : leg->y;
  switch (leg->action) {
    case FLIGHT_ACTION_GOTO_LOCAL:
      if (body) {
        t[LQG_R_X] += dx;
        t[LQG_R_Y] += dy;
        t[LQG_R_Z] += leg->z;
        t[LQG_R_PSI] += leg->yaw;
      } else {
        t[LQG_R_X] = leg->x;
        t[LQG_R_Y] = leg->y;
        t[LQG_R_Z] = leg->z;
        t[LQG_R_PSI] = leg->yaw;
      }
      break;
    case FLIGHT_ACTION_TRANSLATE_RELATIVE:
      t[LQG_R_X] += dx;
      t[LQG_R_Y] += dy;
      t[LQG_R_Z] += leg->z;
      break;
    case FLIGHT_ACTION_ROTATE_RELATIVE:
      t[LQG_R_PSI] += leg->yaw;
      break;
    default:
      return false;
  }
  /* Não descer abaixo da altura de pouso nem subir acima de FLIGHT_MAX_HEIGHT_M. */
  if (t[LQG_R_Z] > z_ground - prm.land_height ||
      t[LQG_R_Z] < z_ground - (FLIGHT_MAX_HEIGHT_M + FLIGHT_HEIGHT_TOL_M)) {
    return false;
  }
  memcpy(goal.target, t, sizeof(t));
  goal.speed_xy = leg->speed > 0.0F ? leg->speed : prm.xy_rate;
  goal.pos_tol = leg->position_tolerance > 0.0F ? leg->position_tolerance : DEFAULT_POS_TOL_M;
  goal.yaw_tol = leg->yaw_tolerance > 0.0F ? leg->yaw_tolerance : DEFAULT_YAW_TOL_RAD;
  goal.check_xy = true;
  goal.timeout_ms = leg->timeout_ms;
  return true;
}

/** @brief Atende um pedido da fila. */
static void handle_request(const request_t *req, uint32_t now) {
  switch (req->type) {
    case REQ_ARM:
      arm_pending = false;
      if (state != FLIGHT_STATE_STANDBY && state != FLIGHT_STATE_FAILSAFE) break;
      /* Referencial local novo: Kalman e posição começam aqui. A guinada
         continua zerada na energização (ver meas_rearm). */
      lqg_ert_init(&lqg);
      meas_rearm(&meas);
      /* O estimador começa do zero a cada voo, por cima do desequilíbrio fixo
         (CTL_FF_TH/PHI). Lembrar o do voo anterior já trouxe valores de quando
         o drone estava preso (05/11). */
      lqg_dist_reset(&dist);
      dist_released = false;
      xy_engaged = false;
      trim_reset(&trim);
      if (trim_learned_valid) {
        trim.theta = trim_learned.theta;
        trim.phi = trim_learned.phi;
        say("trim inicial th=%d phi=%d (voo anterior)", (int)trim.theta, (int)trim.phi);
      }
      trim_released = false;
      land_spindown_ms = 0U;
      stuck_since_ms = 0U;
      meas_z_tracking = false;
      lifted = false;
      memset(ref, 0, sizeof(ref));
      memset(&goal, 0, sizeof(goal));
      settle_since_ms = 0U;
      if (sim_active) {
        /* O drone simulado volta ao (0, 0), no chão, com a guinada que tinha. */
        sim_plant_reset(&sim, SIM_PLANT_GROUND_Z, (float)sim.x[LQG_X_PSI]);
      }
      origin = req->origin;
      dry_run = req->dry;
      armed_ms = now;
      bno_bad_steps = 0U;
      mtf_lost_since_ms = 0U;
      z_ground = NAN;  /* definido com a primeira medida deste passo */
      ground_samples = 0U;
      state = FLIGHT_STATE_ARMED;
      say("voo: ARMADO%s (%s)", sim_active ? " SIMULADO" : dry_run ? " a seco" : "",
          origin == FLIGHT_ORIGIN_PI ? "Pi" : "USB");
      break;
    case REQ_DISARM:
      enter_standby(req->why != NULL ? req->why : "disarm");
      break;
    case REQ_TAKEOFF:
      if (state != FLIGHT_STATE_ARMED) break;
      memcpy(goal.target, ref, sizeof(ref));
      {
        const float wanted = req->height > 0.0F ? req->height : prm.hover_height;
        goal.target[LQG_R_Z] = z_ground - fminf(wanted, FLIGHT_MAX_HEIGHT_M);
      }
      goal.speed_xy = 0.0F;
      goal.pos_tol = req->pos_tol > 0.0F ? req->pos_tol : DEFAULT_POS_TOL_M;
      goal.yaw_tol = DEFAULT_YAW_TOL_RAD;
      goal.check_xy = false;
      goal.timeout_ms = req->timeout_ms;
      goal.start_ms = now;
      goal.active = true;
      state = FLIGHT_STATE_TAKEOFF;
      say("voo: decolando para %d cm", (int)((z_ground - goal.target[LQG_R_Z]) * 100.0F));
      break;
    case REQ_LEG: {
      /* Um setpoint também vale com um trecho em andamento ou, fora do chão,
         na decolagem (ver setpoint_allowed): troca o alvo. */
      if (state != FLIGHT_STATE_HOLD && !(req->height_mode && setpoint_allowed(now))) {
        if (req->height_mode) say("voo: setpoint recusado no estado %s", flight_ctrl_state_name(state));
        break;
      }
      flight_leg_t leg = req->leg;
      if (req->height_mode) leg.z = z_ground - leg.z;
      const bool was_takeoff = state == FLIGHT_STATE_TAKEOFF;
      if (req->height_mode && !goal_from_leg(&leg)) {
        say("voo: setpoint fora dos limites");
        break;
      }
      if (!req->height_mode && !goal_from_leg(&leg)) {
        taskENTER_CRITICAL();
        leg_event.status = FLIGHT_REQUEST_DENIED;
        leg_event.reason = FLIGHT_REASON_OUT_OF_BOUNDS;
        leg_event_pending = true;
        taskEXIT_CRITICAL();
        break;
      }
      if (req->height_mode && prm.use_xy < 0.5F) {
        /* Sem o XY na malha, x e y não são seguidos: não esperar por eles. */
        goal.target[LQG_R_X] = ref[LQG_R_X];
        goal.target[LQG_R_Y] = ref[LQG_R_Y];
        goal.check_xy = false;
      }
      if (was_takeoff) {
        /* A decolagem acaba aqui; o setpoint segue dali (já fora do chão). */
        goal.active = true;
        finish_goal(FLIGHT_REQUEST_FAILED, FLIGHT_REASON_CANCELLED);
      }
      goal.start_ms = now;
      goal.active = true;
      state = FLIGHT_STATE_EXECUTING_LEG;
      if (req->height_mode) {
        say("voo: setpoint x=%d y=%d cm, altura %d cm, guinada %d graus",
            (int)(goal.target[LQG_R_X] * 100.0F), (int)(goal.target[LQG_R_Y] * 100.0F),
            (int)((z_ground - goal.target[LQG_R_Z]) * 100.0F),
            (int)(goal.target[LQG_R_PSI] * 57.29578F));
      }
      break;
    }
    case REQ_HOLD:
      if (!is_airborne_state(state)) break;
      finish_goal(FLIGHT_REQUEST_FAILED,
                  req->reason == FLIGHT_REASON_NONE ? FLIGHT_REASON_CANCELLED : req->reason);
      enter_hold();
      break;
    case REQ_LAND:
      if (state == FLIGHT_STATE_ARMED) {
        enter_standby("pouso no chao");
      } else if (is_airborne_state(state) && state != FLIGHT_STATE_LAND) {
        enter_land(req->reason);
        if (lifted) {
          say("voo: pousando");
        } else {
          /* Ainda no chão: não há o que descer, só reduzir os motores. */
          begin_spindown(now);
        }
      }
      break;
    case REQ_CANCEL:
      if (state == FLIGHT_STATE_TAKEOFF || state == FLIGHT_STATE_EXECUTING_LEG) {
        goal.active = false;  /* o DroneLink publica o cancelamento */
        enter_hold();
      }
      break;
    case REQ_XY_ZERO: {
      /* A posição vem só da soma do fluxo e deriva: o operador declara o
         ponto atual como (0, 0). A referência e o alvo vão junto, então o
         drone passa a segurar onde está; as velocidades (medidas) ficam. */
      lqg.x_filt[LQG_X_X] = lqg.x_filt[LQG_X_Y] = 0.0;
      lqg.x_pred[LQG_X_X] = lqg.x_pred[LQG_X_Y] = 0.0;
      ref[LQG_R_X] = ref[LQG_R_Y] = 0.0F;
      goal.target[LQG_R_X] = goal.target[LQG_R_Y] = 0.0F;
      say("XY: zerado, o ponto atual e o (0,0)");
      break;
    }
    default:
      break;
  }
}

/* ---- Passo da malha ---------------------------------------------------- */

/** @brief Copia as leituras dos drivers para a entrada da conversão. */
static void read_sensors(meas_input_t *in) {
  bno_get_sample(&bno);
  mtf_get_sample(&mtf);
  memcpy(in->q, bno.q, sizeof(in->q));
  in->roll = bno.euler.angle.roll;
  in->pitch = bno.euler.angle.pitch;
  in->yaw = bno.euler.angle.yaw;
  memcpy(in->gyro, bno.gyro, sizeof(in->gyro));
  in->attitude_valid = bno.online && bno.euler_valid;
  in->gyro_valid = bno.online;
  in->range_m = (float)mtf.distance_mm * 0.001F;
  in->range_valid = mtf.range_valid;
  /* O MTF-01 informa o fluxo em cm/s a 1 m: dividido por 100 é rad/s. */
  in->flow_x = (float)mtf.flow_x * 0.01F;
  in->flow_y = (float)mtf.flow_y * 0.01F;
  in->flow_valid = mtf.flow_valid;
}

/**
 * @brief Failsafes e limites de tempo.
 * @param[in] flags  Medidas válidas neste passo.
 * @param[in] y      Medidas.
 * @param[in] now    Tick atual.
 */
static void supervise(uint8_t flags, const lqg_real_t y[LQG_NY], uint32_t now) {
  if (!is_armed_state(state)) return;

  const bool bno_ok = (flags & (MEAS_ATTITUDE | MEAS_GYRO)) == (MEAS_ATTITUDE | MEAS_GYRO);
  bno_bad_steps = bno_ok ? 0U : bno_bad_steps + 1U;
  if (bno_bad_steps >= BNO_TIMEOUT_STEPS) {
    enter_failsafe("BNO sem dado", FLIGHT_REASON_SENSOR_UNHEALTHY);
    return;
  }
  const bool active = state != FLIGHT_STATE_ARMED;
  if (active && (fabsf((float)y[LQG_Y_PHI]) > prm.tilt_max ||
                 fabsf((float)y[LQG_Y_THETA]) > prm.tilt_max)) {
    enter_failsafe("inclinacao excessiva", FLIGHT_REASON_OUT_OF_BOUNDS);
    return;
  }
  if (is_airborne_state(state) && state != FLIGHT_STATE_LAND && (flags & MEAS_RANGE) != 0U &&
      isfinite(z_ground) &&
      z_ground - (float)y[LQG_Y_Z] > FLIGHT_MAX_HEIGHT_M + FLIGHT_OVERSHOOT_M) {
    say("altura acima do limite: pousando");
    enter_land(FLIGHT_REASON_OUT_OF_BOUNDS);
    return;
  }
  if (origin == FLIGHT_ORIGIN_USB && !CDC_HostDTR()) {
    enter_failsafe("porta USB fechada", FLIGHT_REASON_LINK_LOST);
    return;
  }
  if (!dry_run && motors_owner() != MOTORS_OWNER_FLIGHT) {
    enter_failsafe("motores parados", FLIGHT_REASON_CANCELLED);
    return;
  }
  if (state == FLIGHT_STATE_ARMED && now - armed_ms > ARMED_IDLE_TIMEOUT_MS) {
    enter_standby("armado sem decolar");
    return;
  }
  if ((flags & MEAS_RANGE) != 0U) {
    mtf_lost_since_ms = 0U;
  } else if (mtf_lost_since_ms == 0U) {
    mtf_lost_since_ms = now;
  }
  if (is_airborne_state(state) && state != FLIGHT_STATE_LAND && mtf_lost_since_ms != 0U &&
      now - mtf_lost_since_ms > MTF_LOST_MS) {
    say("MTF sem dado: pousando");
    enter_land(FLIGHT_REASON_SENSOR_UNHEALTHY);
  }
}

/** @brief Avança a referência em rampa até o alvo. */
static void update_reference(uint32_t now) {
  /* A referência só sobe depois da marcha lenta e da rampa de partida. */
  if (state == FLIGHT_STATE_TAKEOFF && now - armed_ms < SPINUP_MS + FLIGHT_TAKEOFF_RAMP_MS) return;
  if (!goal.active && state != FLIGHT_STATE_HOLD) return;

  /* Horizontal: em linha reta, na velocidade do trecho. */
  const float ex = goal.target[LQG_R_X] - ref[LQG_R_X];
  const float ey = goal.target[LQG_R_Y] - ref[LQG_R_Y];
  const float dist = sqrtf(ex * ex + ey * ey);
  const float step_xy = goal.speed_xy * DT_S;
  if (!(goal.speed_xy > 0.0F) || dist <= step_xy) {
    ref[LQG_R_X] = goal.target[LQG_R_X];
    ref[LQG_R_Y] = goal.target[LQG_R_Y];
  } else {
    ref[LQG_R_X] += ex / dist * step_xy;
    ref[LQG_R_Y] += ey / dist * step_xy;
  }
  float z_rate = prm.z_rate;
  if (state == FLIGHT_STATE_LAND && z_rate < LAND_MIN_RATE_M_S) z_rate = LAND_MIN_RATE_M_S;
  ref[LQG_R_Z] = approach(ref[LQG_R_Z], goal.target[LQG_R_Z], z_rate * DT_S);
  ref[LQG_R_PSI] = approach(ref[LQG_R_PSI], goal.target[LQG_R_PSI], prm.yaw_rate * DT_S);
}

/** @brief Confere se a decolagem ou o trecho terminou (ou estourou o tempo). */
static void check_goal(const float x[LQG_NX], uint32_t now) {
  if (!goal.active ||
      (state != FLIGHT_STATE_TAKEOFF && state != FLIGHT_STATE_EXECUTING_LEG)) {
    return;
  }
  if (goal.timeout_ms != 0U && now - goal.start_ms > goal.timeout_ms) {
    finish_goal(FLIGHT_REQUEST_FAILED, FLIGHT_REASON_TIMEOUT);
    enter_hold();
    say("voo: trecho estourou o tempo");
    return;
  }
  const bool ramp_done = memcmp(ref, goal.target, sizeof(ref)) == 0;
  const bool z_ok = fabsf(x[LQG_X_Z] - goal.target[LQG_R_Z]) < goal.pos_tol;
  const bool xy_ok = !goal.check_xy ||
                     (fabsf(x[LQG_X_X] - goal.target[LQG_R_X]) < goal.pos_tol &&
                      fabsf(x[LQG_X_Y] - goal.target[LQG_R_Y]) < goal.pos_tol);
  const bool yaw_ok =
      fabsf(attitude_wrap_pi(x[LQG_X_PSI] - goal.target[LQG_R_PSI])) < goal.yaw_tol;
  /* Estável: no alvo e parado por FLIGHT_SETTLE_MS. Só então o trecho termina
     e a Pi pode mandar o próximo (girar ou andar sem estar parado no ar não
     faz sentido). Enquanto isso o voo segue o alvo do mesmo jeito. */
  const bool still = fabsf(x[LQG_X_VX]) < FLIGHT_SETTLE_SPEED_M_S &&
                     fabsf(x[LQG_X_VY]) < FLIGHT_SETTLE_SPEED_M_S &&
                     fabsf(x[LQG_X_VZ]) < FLIGHT_SETTLE_SPEED_M_S &&
                     fabsf(x[LQG_X_P]) < FLIGHT_SETTLE_RATE_RAD_S &&
                     fabsf(x[LQG_X_Q]) < FLIGHT_SETTLE_RATE_RAD_S &&
                     fabsf(x[LQG_X_R]) < FLIGHT_SETTLE_RATE_RAD_S;
  if (!(ramp_done && z_ok && xy_ok && yaw_ok && still)) {
    settle_since_ms = 0U;
    return;
  }
  if (settle_since_ms == 0U) settle_since_ms = now != 0U ? now : 1U;
  if (now - settle_since_ms < FLIGHT_SETTLE_MS) return;
  settle_since_ms = 0U;
  finish_goal(FLIGHT_REQUEST_ACCEPTED, FLIGHT_REASON_NONE);
  enter_hold();
  say("voo: estavel em HOLD a %d cm", (int)((z_ground - ref[LQG_R_Z]) * 100.0F));
}


/**
 * @brief Filtra a altura medida e deriva dela a velocidade vertical.
 *
 * O Kalman não serve para dizer se o drone sobe ou desce quando algo o segura:
 * o modelo prevê o movimento que o LQR pede, a altura medida só corrige a
 * posição, e a velocidade estimada fica enviesada (05/11: 4–5 cm/s de subida
 * com o drone parado nas cordinhas). Sem medida neste passo, mantém o valor.
 *
 * @param[in] flags  Medidas válidas neste passo.
 * @param[in] y      Medidas.
 */
static void track_height(uint8_t flags, const lqg_real_t y[LQG_NY]) {
  if ((flags & MEAS_RANGE) == 0U) return;
  const float z = (float)y[LQG_Y_Z];
  if (!meas_z_tracking) {
    meas_z_smooth = z;
    meas_vz = 0.0F;
    meas_z_tracking = true;
    return;
  }
  const float a = DT_S / FLIGHT_MEAS_Z_TAU_S;
  const float previous = meas_z_smooth;
  meas_z_smooth += a * (z - meas_z_smooth);
  meas_vz += a * ((meas_z_smooth - previous) / DT_S - meas_vz);
}

/**
 * @brief Na subida, segura na altura medida se o drone parou de subir.
 *
 * Preso por cordinhas ou num teto, o drone não acompanha a referência e o
 * integrador de altura cresce sem parar, até a inclinação passar do corte
 * (ensaio de 05/11). Ver ::FLIGHT_STUCK_ERROR_M. Só na decolagem e nos
 * trechos: em HOLD o integrador precisa poder crescer (bateria caindo).
 *
 * @param[in] now  Tick atual.
 */
static void check_stuck(uint32_t now) {
  const bool chasing = state == FLIGHT_STATE_TAKEOFF || state == FLIGHT_STATE_EXECUTING_LEG;
  const float z = meas_z_smooth;
  const bool behind = chasing && prm.stuck_detect >= 0.5F && motors_follow() && meas_z_tracking &&
                      isfinite(z_ground) &&
                      z_ground - z >= FLIGHT_STUCK_MIN_HEIGHT_M &&
                      z - ref[LQG_R_Z] > FLIGHT_STUCK_ERROR_M;
  if (!behind) {
    stuck_since_ms = 0U;
    return;
  }
  /* Começa (ou recomeça, se subiu) a contagem guardando o empuxo de agora. */
  if (stuck_since_ms == 0U || stuck_z0 - z > FLIGHT_STUCK_RISE_M) {
    stuck_since_ms = now != 0U ? now : 1U;
    stuck_z0 = z;
    stuck_dist_z = lqg_dist_get(&dist, LQG_DIST_Z);
    stuck_trim_z = trim.z;
    return;
  }
  if (now - stuck_since_ms < FLIGHT_STUCK_MS) return;
  /* O empuxo acumulado desde o travamento só empurrava contra o que prende. */
  if (lqg_dist_get(&dist, LQG_DIST_Z) > stuck_dist_z) lqg_dist_set(&dist, LQG_DIST_Z, stuck_dist_z);
  if (trim.z > stuck_trim_z) trim.z = stuck_trim_z;
  ref[LQG_R_Z] = z;
  finish_goal(FLIGHT_REQUEST_FAILED, FLIGHT_REASON_OUT_OF_BOUNDS);
  enter_hold();
  stuck_since_ms = 0U;
  say("altura nao sobe (preso?): HOLD em %d cm", (int)((z_ground - z) * 100.0F));
}

/**
 * @brief No pouso, desconta o hover dos integradores se o drone não desce.
 *
 * O integrador de altura guarda o empuxo extra do hover e desfaz devagar: com
 * a referência já no chão, o drone ficava parado no alto e a descida só vinha
 * de repente (ensaio de 05/11). Ver ::FLIGHT_LAND_BLEED_RATE. Sem integrador
 * (nem estimador nem ajuste lento) não há o que descontar.
 */
static void land_bleed(void) {
  if (state != FLIGHT_STATE_LAND || land_spindown_ms != 0U || !motors_follow() ||
      !meas_z_tracking) {
    return;
  }
  /* z NED: a referência maior que a medida é o drone acima dela. A velocidade
     é a medida (track_height): a do Kalman acha que ele desce quando o LQR pede. */
  const bool above = ref[LQG_R_Z] - meas_z_smooth > FLIGHT_LAND_ABOVE_M;
  const bool descending = meas_vz > FLIGHT_LAND_DESCENT_M_S;
  if (!above || descending) return;
  const float delta = FLIGHT_LAND_BLEED_RATE * DT_S;
  if (dist_active) {
    lqg_dist_set(&dist, LQG_DIST_Z, fmaxf(lqg_dist_get(&dist, LQG_DIST_Z) - delta, -prm.dist_max));
  } else if (prm.trim >= 0.5F) {
    trim.z = fmaxf(trim.z - delta, -prm.trim_max);
  }
}

/** @brief Termina o pouso ao encostar no chão (ou pelo tempo, sem telêmetro). */
static void check_land(uint8_t flags, const lqg_real_t y[LQG_NY], uint32_t now) {
  if (state != FLIGHT_STATE_LAND) return;
  if (land_spindown_ms != 0U) {
    /* Encostou: a aceleração desce em rampa (compute_output) e só então desarma. */
    if (now - land_spindown_ms >= FLIGHT_LAND_SPINDOWN_MS) enter_standby("pouso concluido");
    return;
  }
  const bool ramp_down = ref[LQG_R_Z] == goal.target[LQG_R_Z];
  if ((flags & MEAS_RANGE) != 0U) {
    const float height = z_ground - (float)y[LQG_Y_Z];
    if (ramp_down && height < prm.land_height) {
      if (land_touch_ms == 0U) land_touch_ms = now;
      if (now - land_touch_ms > LAND_TOUCH_MS) begin_spindown(now);
    } else {
      land_touch_ms = 0U;
    }
    return;
  }
  /* Sem telêmetro: espera a rampa chegar ao chão e mais um pouco. */
  float rate = prm.z_rate < LAND_MIN_RATE_M_S ? LAND_MIN_RATE_M_S : prm.z_rate;
  const uint32_t ramp_ms = (uint32_t)(land_start_height / rate * 1000.0F);
  if (now - land_start_ms > ramp_ms + LAND_BLIND_EXTRA_MS) {
    begin_spindown(now);
  }
}

/**
 * @brief O equilíbrio deste passo vale para o próximo voo.
 *
 * Só em HOLD, com a altura medida no alvo, nivelado e assentado: na subida
 * presa (05/11) o equilíbrio guardado era o de quem empurra uma cordinha.
 */
static bool balance_is_learnable(void) {
  return lifted && state == FLIGHT_STATE_HOLD && meas_range_ok &&
         fabsf(meas_z_now - ref[LQG_R_Z]) < TRIM_LEARN_Z_ERROR_M &&
         fabsf((float)lqg.x_filt[LQG_X_THETA]) < TRIM_LEARN_LEVEL_RAD &&
         fabsf((float)lqg.x_filt[LQG_X_PHI]) < TRIM_LEARN_LEVEL_RAD &&
         fabsf((float)lqg.x_filt[LQG_X_VZ]) < TRIM_SETTLED_VZ;
}

/**
 * @brief Avança o ajuste lento do hover e do equilíbrio.
 *
 * Só integra no ar, com motores reais, sem saturação e com altura medida. O
 * desprendimento do chão é detectado pela altura estimada e travado até desarmar.
 *
 * @param[in] enabled  O controle está ativo e nada saturou neste passo.
 */
static void update_trim(bool enabled) {
  if (prm.trim < 0.5F) return;
  const float height = isfinite(z_ground) ? z_ground - (float)lqg.x_filt[LQG_X_Z] : 0.0F;
  if (enabled && height >= TRIM_RELEASE_HEIGHT_M) trim_released = true;
  const trim_config_t cfg = {.ki_z = prm.trim_ki_z, .ki_att = prm.trim_ki_att,
                             .ground_rate = TRIM_GROUND_RATE, .settled_vz = TRIM_SETTLED_VZ,
                             .max_z = prm.trim_max, .max_att = prm.trim_att_max,
                             .att_error_max = TRIM_ATT_ERROR_MAX_RAD};
  const uint32_t now = HAL_GetTick();
  if (enabled && now - trim_report_ms >= TRIM_REPORT_MS) {
    trim_report_ms = now;
    const bool at_limit = fabsf(trim.z) >= prm.trim_max - 1.0F ||
                          fabsf(trim.theta) >= prm.trim_att_max - 1.0F ||
                          fabsf(trim.phi) >= prm.trim_att_max - 1.0F;
    say("trim z=%d th=%d phi=%d%s", (int)trim.z, (int)trim.theta, (int)trim.phi,
        at_limit ? " NO LIMITE" : "");
  }
  if (enabled && balance_is_learnable()) {
    trim_learned = trim;
    trim_learned_valid = true;
  }
  trim_update(&trim, &cfg, enabled && mtf_lost_since_ms == 0U, DT_S, !trim_released,
              (float)lqg.x_filt[LQG_X_VZ], (float)lqg.x_filt[LQG_X_Z] - ref[LQG_R_Z],
              (float)lqg.x_filt[LQG_X_PHI], (float)lqg.x_filt[LQG_X_THETA]);
}

/**
 * @brief Prediz o estimador de perturbação com o comando aplicado e relata.
 *
 * Mesmas condições dos integradores: a perturbação só muda no ar, com motores
 * reais, sem rampa e sem saturação; a de altura só com o MTF respondendo.
 * Posição e velocidade do estimador continuam acompanhando sempre.
 *
 * @param[in] enabled  O controle está ativo e nada saturou neste passo.
 * @param[in] u        Desvio aplicado em u1..u4 sem o desequilíbrio fixo, em rad/s.
 */
static void update_dist(bool enabled, const float u[LQG_NU]) {
  const uint32_t now = HAL_GetTick();
  if (enabled && now - trim_report_ms >= TRIM_REPORT_MS) {
    trim_report_ms = now;
    say("dist z=%d th=%d phi=%d psi=%d", (int)lqg_dist_get(&dist, LQG_DIST_Z),
        (int)lqg_dist_get(&dist, LQG_DIST_THETA), (int)lqg_dist_get(&dist, LQG_DIST_PHI),
        (int)lqg_dist_get(&dist, LQG_DIST_PSI));
  }
  /* No chão o apoio segura o drone, e o estimador leria isso como empuxo
     sobrando (05/11: d̂ de altura em −160 rad/s e o drone não decolava). Até
     ele se soltar, nada é estimado; com o alvo acima, o hover sobe numa rampa
     fixa, como no ajuste lento. */
  const float height = isfinite(z_ground) ? z_ground - meas_z_smooth : 0.0F;
  if (enabled && meas_z_tracking && height >= TRIM_RELEASE_HEIGHT_M) dist_released = true;
  lqg_dist_predict(&dist, u, !enabled || !dist_released, !meas_range_ok, prm.dist_max);
  if (enabled && !dist_released && meas_z_tracking &&
      meas_z_smooth - ref[LQG_R_Z] > DIST_GROUND_ERROR_M) {
    lqg_dist_set(&dist, LQG_DIST_Z,
                 fminf(lqg_dist_get(&dist, LQG_DIST_Z) + TRIM_GROUND_RATE * DT_S, prm.dist_max));
  }
}

/**
 * @brief Converte du em aceleração dos motores e diz qual entrada foi aplicada.
 *
 * Em voo, a predição do Kalman usa o du que os motores realmente recebem
 * (igual ao do LQG enquanto não satura, como no Simulink). No chão e no
 * ensaio a seco, os motores não seguem o LQG, então a predição usa zero.
 *
 * @param[in]  du          Saída do LQG.
 * @param[out] throttle    Aceleração de cada motor (0..1).
 * @param[out] du_applied  Entrada aplicada, para a predição.
 * @param[out] flags       Recebe ::FLIGHT_FLAG_SATURATED e ::FLIGHT_FLAG_CONTROL_ACTIVE.
 */
static void compute_output(const lqg_real_t du[LQG_NU], float throttle[LQG_NU],
                           lqg_real_t du_applied[LQG_NU], uint8_t *flags) {
  const uint32_t now = HAL_GetTick();
  const uint32_t since_arm = now - armed_ms;
  const bool taking_off = state == FLIGHT_STATE_TAKEOFF;
  const bool spinning_up = taking_off && since_arm < SPINUP_MS;
  const bool ramping_up = taking_off && !spinning_up && since_arm < SPINUP_MS + FLIGHT_TAKEOFF_RAMP_MS;
  const bool spinning_down = state == FLIGHT_STATE_LAND && land_spindown_ms != 0U;
  const bool active = is_airborne_state(state) && !spinning_up;
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    du_applied[i] = 0.0;
    sim_input[i] = 0.0F;
  }
  if (!active) {
    for (unsigned i = 0U; i < LQG_NU; ++i) {
      throttle[i] = is_armed_state(state) ? prm.mot_idle : 0.0F;
    }
    if (dist_active) {
      const float idle[LQG_NU] = {0.0F, 0.0F, 0.0F, 0.0F};
      update_dist(false, idle);
    } else {
      update_trim(false);
    }
    return;
  }

  land_bleed();
  const float span = prm.mot_w_at_100 - prm.mot_w_at_0;
  /* Somado ao du: o desequilíbrio fixo (CTL_FF_TH/PHI, o ponto de operação do
     drone com o peso fora do centro) e a correção lenta: a perturbação
     estimada (LQG melhorado) ou o ajuste lento. Tudo fica fora
     da predição do Kalman (ver du_applied): é um peso que o modelo não
     conhece, não uma entrada. */
  float trim_w[LQG_NU];
  const trim_state_t feedforward = {.z = 0.0F, .theta = prm.ff_theta, .phi = prm.ff_phi};
  trim_values(&feedforward, trim_w);
  float slow_w[LQG_NU] = {0.0F, 0.0F, 0.0F, 0.0F};
  if (dist_active) {
    lqg_dist_output(&dist, slow_w);
  } else if (prm.trim >= 0.5F) {
    trim_values(&trim, slow_w);
  }
  for (unsigned i = 0U; i < LQG_NU; ++i) trim_w[i] += slow_w[i];
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    const float w = prm.w0 + (float)du[i] + trim_w[i];
    throttle[i] = span > 0.0F ? (w - prm.mot_w_at_0) / span : 0.0F;
  }
  /* Um motor no teto desloca os quatro: a diferença que segura o nível fica. */
  bool modified = throttle_desaturate(throttle, LQG_NU, prm.mot_min, prm.mot_max);
  if (modified) *flags |= FLIGHT_FLAG_SATURATED;
  if (ramping_up || spinning_down) {
    const float factor = ramping_up
        ? throttle_ramp_up(since_arm - SPINUP_MS, FLIGHT_TAKEOFF_RAMP_MS)
        : throttle_ramp_down(now - land_spindown_ms, FLIGHT_LAND_SPINDOWN_MS);
    for (unsigned i = 0U; i < LQG_NU; ++i) {
      throttle[i] = throttle_blend(throttle[i], prm.mot_idle, factor);
    }
    modified = true;
  }
  if (motors_follow()) {
    for (unsigned i = 0U; i < LQG_NU; ++i) {
      /* A predição do Kalman recebe só o du: o trim é o novo ponto de hover
         (compensa um peso que o filtro não conhece), não uma entrada. Sem
         saturação nem rampa, é exatamente o du do LQG. */
      du_applied[i] = modified
          ? (lqg_real_t)(prm.mot_w_at_0 + throttle[i] * span - prm.w0 - trim_w[i])
          : du[i];
      sim_input[i] = (float)du_applied[i] + slow_w[i];
    }
  }
  *flags |= FLIGHT_FLAG_CONTROL_ACTIVE;
  const bool integrate = motors_follow() && !ramping_up && !spinning_down &&
                         (*flags & FLIGHT_FLAG_SATURATED) == 0U;
  if (dist_active) {
    float u[LQG_NU];
    for (unsigned i = 0U; i < LQG_NU; ++i) u[i] = (float)du_applied[i] + slow_w[i];
    update_dist(integrate, u);
  } else {
    update_trim(integrate);
  }
}

/** @brief Publica a fotografia do passo e, na taxa pedida, a linha EST. */
static void publish(uint32_t now, uint32_t step_count, uint8_t meas_flags, uint8_t flags,
                    const lqg_real_t y[LQG_NY], const lqg_real_t du[LQG_NU],
                    const float throttle[LQG_NU], uint32_t step_us) {
  static flight_telemetry_t t;
  t.time_ms = now;
  t.step = step_count;
  t.state = state;
  t.meas_flags = meas_flags;
  t.flags = flags;
  for (unsigned i = 0U; i < LQG_NY; ++i) t.y[i] = (float)y[i];
  for (unsigned i = 0U; i < LQG_NX; ++i) t.x_hat[i] = (float)lqg.x_filt[i];
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    t.du[i] = (float)du[i];
    t.throttle[i] = throttle[i];
  }
  memcpy(t.ref, ref, sizeof(t.ref));
  memcpy(t.q, bno.q, sizeof(t.q));
  t.z_ground = z_ground;
  if (step_us > t.max_step_us) t.max_step_us = step_us;
  if ((meas_flags & (MEAS_ATTITUDE | MEAS_GYRO)) != (MEAS_ATTITUDE | MEAS_GYRO)) ++t.bno_misses;
  if ((meas_flags & MEAS_RANGE) == 0U) ++t.mtf_misses;

  taskENTER_CRITICAL();
  telemetry = t;
  taskEXIT_CRITICAL();

  est_accumulator += prm.log_est_hz * DT_S;
  if (est_accumulator >= 1.0F) {
    est_accumulator -= 1.0F;
    if (est_accumulator > 1.0F) est_accumulator = 0.0F;
    log_est(t.state, t.meas_flags, t.flags, t.y, t.x_hat, t.du, t.ref);
  }
}

/** @brief Um passo de 10 ms: pedidos, sensores, segurança, LQG e saída. */
static void step(uint32_t step_count) {
  const uint32_t start_cycles = DWT->CYCCNT;
  const uint32_t now = HAL_GetTick();

  ctrl_params_snapshot(&prm);
  if (ctrl_gains_generation() != gains_generation) {
    ctrl_gains_copy_active(&gains, &gains_generation);
    if (is_armed_state(state)) say("voo: ganhos novos em uso");
  }
  if (lqg_tuning_generation() != lqg_aj_generation && !is_armed_state(state)) {
    lqg_tuning_copy_active(&lqg_aj_gains, &lqg_aj_generation);
    say("lqg: Kd/Kfd %s em uso", lqg_tuning_is_default() ? "padrao" : "do painel (pesos Q)");
  }

  /* Depois de um armar, os pedidos seguintes esperam o próximo passo: a
     decolagem precisa da altura do chão, que só é medida neste passo. */
  request_t req;
  while (xQueuePeek(requests, &req, 0) == pdPASS) {
    if (state == FLIGHT_STATE_ARMED && isnan(z_ground)) break;
    (void)xQueueReceive(requests, &req, 0);
    handle_request(&req, now);
  }

  lqg_real_t y[LQG_NY];
  uint8_t meas_flags;
  /* O simulado só liga e desliga desarmado (FLT_SIM é DISARMED_ONLY). */
  if (!is_armed_state(state)) sim_active = prm.sim >= 0.5F;
  if (sim_active) {
    sim_plant_measure(&sim, y);
    meas_flags = MEAS_ATTITUDE | MEAS_GYRO | MEAS_RANGE | MEAS_FLOW;
  } else {
    meas_input_t in;
    read_sensors(&in);
    meas_flags = meas_update(&meas, &in, &prm, y);
  }
  dist_active = prm.use_dist >= 0.5F;
  const bool use_xy = prm.use_xy >= 0.5F;
  gains_effective = gains;
  /* Ganhos enviados pela Pi (DRONE_GAIN_*) são um conjunto completo (Kd, Kfd,
     L, M) projetado para o modo em uso e valem como vieram. Sem eles, o
     ajustado troca Kd/Kfd (gerados ou do painel USB) e o Kalman. */
  if (prm.lqg_adjusted >= 0.5F && ctrl_gains_active_is_default()) {
    lqg_tuning_apply(&lqg_aj_gains, &gains_effective);
    lqg_dist_apply_kalman(&gains_effective);
  }
  meas_z_now = (float)y[LQG_Y_Z];
  meas_range_ok = (meas_flags & MEAS_RANGE) != 0U;
  /* O XY (x, y, vx, vy na malha) só entra depois que o drone sai do chão: lá
     o fluxo do MTF não vale (sensor a 8–10 cm do piso) e, sem ele, o Kalman
     integraria a inclinação da mesa como velocidade durante a rampa de
     partida. Ao entrar, a posição é zerada: o ponto de decolagem é o (0, 0). */
  if (use_xy && is_airborne_state(state) && lifted && !xy_engaged) {
    static const unsigned horizontal[] = {LQG_X_X, LQG_X_Y, LQG_X_VX, LQG_X_VY};
    for (unsigned k = 0U; k < sizeof(horizontal) / sizeof(horizontal[0]); ++k) {
      lqg.x_filt[horizontal[k]] = 0.0;
      lqg.x_pred[horizontal[k]] = 0.0;
    }
    ref[LQG_R_X] = ref[LQG_R_Y] = 0.0F;
    goal.target[LQG_R_X] = goal.target[LQG_R_Y] = 0.0F;
    xy_engaged = true;
    say("XY: ligado, ponto (0,0) na decolagem");
  }
  if (!use_xy || !xy_engaged) {
    /* Só atitude e altura. Os estados horizontais ficam em zero: nada os
       mede, e a predição sozinha derivaria sem sentido (e apareceria no log). */
    static const unsigned horizontal[] = {LQG_X_X, LQG_X_Y, LQG_X_VX, LQG_X_VY};
    for (unsigned k = 0U; k < sizeof(horizontal) / sizeof(horizontal[0]); ++k) {
      lqg.x_pred[horizontal[k]] = 0.0;
    }
    attitude_mode_mask_gains(&gains_effective);
    attitude_mode_repeat_prediction(&lqg, y);
  } else if ((meas_flags & MEAS_FLOW) == 0U) {
    /* Sem fluxo válido neste passo, meas_update repetiria o último vx, vy:
       o controle seguiria empurrando com uma velocidade velha. A predição do
       Kalman entra no lugar (a posição só deriva com o modelo até o fluxo voltar). */
    attitude_mode_repeat_prediction(&lqg, y);
    if (is_airborne_state(state) && now - flow_lost_said_ms >= FLOW_LOST_REPORT_MS) {
      flow_lost_said_ms = now;
      say("XY: fluxo do MTF sem dado, vx/vy pela predicao");
    }
  }

  if (state == FLIGHT_STATE_ARMED && isnan(z_ground)) {
    /* Primeira medida depois de armar: o chão e a referência ficam aqui. */
    z_ground = (float)y[LQG_Y_Z];
    ref[LQG_R_Z] = z_ground;
    /* A decolagem mantém a proa de quando armou: girar no chão arrasta o
       drone, e a decolagem só termina com a guinada no alvo. O setpoint leva
       a guinada a outro valor (o zero da energização, no painel) já no ar. */
    ref[LQG_R_PSI] = (float)y[LQG_Y_PSI];
    ground_samples = 1U;
  } else if (is_armed_state(state) && ground_samples > 0U && now - armed_ms < SPINUP_MS &&
             (meas_flags & MEAS_RANGE) != 0U) {
    /* Parado, o MTF varia ~2 cm entre leituras: o chão é a média do segundo
       em marcha lenta, com o drone ainda apoiado. A referência e o alvo da
       decolagem andam junto, então a altura pedida continua a mesma. */
    ++ground_samples;
    const float shift = ((float)y[LQG_Y_Z] - z_ground) / (float)ground_samples;
    z_ground += shift;
    ref[LQG_R_Z] += shift;
    if (goal.active) goal.target[LQG_R_Z] += shift;
  }
  if (!is_armed_state(state)) {
    /* Desarmado, a referência acompanha a estimativa: du mostra o que o
       controle faria para segurar o drone onde ele está. */
    ref[LQG_R_X] = (float)lqg.x_filt[LQG_X_X];
    ref[LQG_R_Y] = (float)lqg.x_filt[LQG_X_Y];
    ref[LQG_R_Z] = (float)y[LQG_Y_Z];
    ref[LQG_R_PSI] = (float)lqg.x_filt[LQG_X_PSI];
  }

  if (is_airborne_state(state) && isfinite(z_ground) &&
      z_ground - (float)lqg.x_filt[LQG_X_Z] >= FLIGHT_LIFTED_HEIGHT_M) {
    lifted = true;
  }
  supervise(meas_flags, y, now);
  track_height(meas_flags, y);
  if (dist_active && is_armed_state(state)) {
    lqg_dist_correct(&dist, y, (meas_flags & MEAS_RANGE) != 0U);
  }
  check_stuck(now);
  update_reference(now);

  lqg_real_t r[LQG_NR];
  for (unsigned i = 0U; i < LQG_NR; ++i) r[i] = (lqg_real_t)ref[i];
  /* Com o XY na malha, o LQG trabalha no referencial da proa (ver
     attitude_mode_rotate_horizontal). ψ é o mesmo que girou o fluxo em meas. */
  const bool heading_frame = use_xy && xy_engaged;
  const float psi_now = (float)y[LQG_Y_PSI];
  if (heading_frame) {
    attitude_mode_rotate_pair(&y[LQG_Y_VX], &y[LQG_Y_VY], -psi_now);
    attitude_mode_rotate_pair(&r[LQG_R_X], &r[LQG_R_Y], -psi_now);
    if (!lqg.ic_load) attitude_mode_rotate_horizontal(lqg.x_pred, -psi_now);
  }
  lqg_real_t du[LQG_NU];
  lqg_ert_output(&lqg, &gains_effective, y, r, du);

  uint8_t flags = 0U;
  if (dry_run || !is_armed_state(state)) flags |= FLIGHT_FLAG_DRY_RUN;
  if (!ctrl_gains_active_is_default()) flags |= FLIGHT_FLAG_CUSTOM_GAINS;
  float throttle[LQG_NU];
  lqg_real_t du_applied[LQG_NU];
  compute_output(du, throttle, du_applied, &flags);
  if (!dry_run && motors_owner() == MOTORS_OWNER_FLIGHT) {
    float motor_throttle[MIXER_COUNT];
    mixer_model_to_motors(throttle, motor_throttle);
    motors_set_flight_throttle(motor_throttle);
  }

  if (prm.lqg_adjusted >= 0.5F) {
    /* Ganhos de guinada para a autoridade real: a predição também a usa. */
    lqg_real_t du_model[LQG_NU];
    lqg_dist_kalman_input(du_applied, du_model);
    lqg_ert_update(&lqg, &gains_effective, du_model);
  } else {
    lqg_ert_update(&lqg, &gains_effective, du_applied);
  }
  if (sim_active) {
    /* O drone simulado é o modelo do TCC; com o ajustado, na escala real de
       guinada e altura (a mesma que o projeto do ajustado supõe). */
    lqg_real_t u_plant[LQG_NU];
    for (unsigned i = 0U; i < LQG_NU; ++i) u_plant[i] = (lqg_real_t)sim_input[i];
    if (prm.lqg_adjusted >= 0.5F) {
      lqg_real_t u_scaled[LQG_NU];
      lqg_dist_kalman_input(u_plant, u_scaled);
      sim_plant_step(&sim, u_scaled);
    } else {
      sim_plant_step(&sim, u_plant);
    }
  }
  if (heading_frame) {
    attitude_mode_rotate_horizontal(lqg.x_pred, psi_now);
    attitude_mode_rotate_horizontal(lqg.x_filt, psi_now);
    attitude_mode_rotate_pair(&y[LQG_Y_VX], &y[LQG_Y_VY], psi_now);  /* o log mostra o local */
  }

  float x[LQG_NX];
  for (unsigned i = 0U; i < LQG_NX; ++i) x[i] = (float)lqg.x_filt[i];
  check_goal(x, now);
  check_land(meas_flags, y, now);

  const uint32_t cycles = DWT->CYCCNT - start_cycles;
  publish(now, step_count, meas_flags, flags, y, du, throttle,
          cycles / (SystemCoreClock / 1000000U));
}

/** @brief Task do voo: um passo a cada Ts = 10 ms. */
static void flight_task(void *argument) {
  (void)argument;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  lqg_ert_init(&lqg);
  meas_reset(&meas);
  sim_plant_reset(&sim, SIM_PLANT_GROUND_Z, 0.0F);
  ctrl_gains_copy_active(&gains, &gains_generation);
  lqg_tuning_copy_active(&lqg_aj_gains, &lqg_aj_generation);
  state = FLIGHT_STATE_STANDBY;
  uint32_t count = 0U;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(LQG_TS_MS));
    step(++count);
  }
}

/** Controle da task do voo (alocação estática). */
static StaticTask_t flight_tcb;
/** Pilha da task do voo, em words. */
static StackType_t flight_stack[1024];

bool flight_ctrl_start(void) {
  ctrl_params_init();
  ctrl_gains_init();
  lqg_tuning_init();
  requests = xQueueCreateStatic(REQUEST_QUEUE_DEPTH, sizeof(request_t), requests_storage,
                                &requests_memory);
  return requests != NULL &&
         xTaskCreateStatic(flight_task, "Flight", sizeof(flight_stack) / sizeof(flight_stack[0]),
                           NULL, tskIDLE_PRIORITY + 4, flight_stack, &flight_tcb) != NULL;
}
