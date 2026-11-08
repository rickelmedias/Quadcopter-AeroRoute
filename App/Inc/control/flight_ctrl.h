/**
 * @file flight_ctrl.h
 * @brief Malha de voo: sensores -> Kalman/LQR do TCC -> motores, a 100 Hz.
 *
 * Uma task roda a cada Ts = 10 ms (o período do modelo do Simulink):
 *
 * 1. atende os pedidos pendentes (armar, decolar, trechos, pouso...);
 * 2. lê o BNO08x e o MTF-01 e monta y (control/meas.h);
 * 3. confere os failsafes;
 * 4. avança a referência r = [x, y, z, ψ] em rampa até o alvo;
 * 5. roda o passo do LQG (control/lqg_ert.h) e converte ω0 + du em
 *    aceleração DShot pela curva do motor;
 * 6. publica a telemetria (log USB e DRONE_CTRL_DEBUG para a Pi).
 *
 * O Kalman roda desde o boot, mesmo desarmado, para que a filtragem dos
 * sensores possa ser vista sem voar. Desarmado, a predição usa du = 0 (o
 * drone parado no chão está em equilíbrio, sustentado pelo chão).
 *
 * @par Estados (os mesmos de DRONE_FLIGHT_STATE)
 * @code
 *  STANDBY --armar--> ARMED --decolar--> TAKEOFF --chegou--> HOLD <--> EXECUTING_LEG
 *     ^                 |                                      |
 *     +----desarmar-----+<-------------- LAND <----pousar------+
 *  qualquer estado armado --falha grave--> FAILSAFE (motores cortados)
 * @endcode
 *
 * @par Ensaio a seco
 * Com FLT_DRY_RUN = 1 (padrão) ou LQG_FLIGHT_ENABLED = 0 (flight_config.h),
 * toda a sequência acontece, mas os motores não são armados: dá para
 * acompanhar estimativas e comandos com o drone na mão.
 *
 * @par Referencial local
 * Ao armar, o Kalman e a guinada são zerados: x, y e ψ passam a ser medidos a
 * partir de onde o drone está, e z continua sendo a altura medida pelo MTF-01
 * (NED, negativa acima do chão).
 */
#ifndef CONTROL_FLIGHT_CTRL_H
#define CONTROL_FLIGHT_CTRL_H

#include "control/lqg_ert.h"
#include "dronelink/flight_command_api.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief Quem armou: define as proteções de enlace. */
typedef enum {
  /** Console USB: fechar a porta (DTR) corta tudo. */
  FLIGHT_ORIGIN_USB = 0,
  /** Raspberry Pi: o DroneLink supervisiona o enlace e a autorização. */
  FLIGHT_ORIGIN_PI = 1
} flight_origin_t;

/** @brief Situação do controle (mesmos bits de DRONE_CTRL_FLAG). */
typedef enum {
  FLIGHT_FLAG_DRY_RUN = 1U << 0,         /**< Motores não seguem o controle. */
  FLIGHT_FLAG_CONTROL_ACTIVE = 1U << 1,  /**< A saída do LQG está sendo aplicada. */
  FLIGHT_FLAG_CUSTOM_GAINS = 1U << 2,    /**< Ganhos enviados pela Pi, não os do Simulink. */
  FLIGHT_FLAG_SATURATED = 1U << 3        /**< Algum motor bateu em MOT_MIN ou MOT_MAX. */
} flight_flags_t;

/** @brief Fotografia de um passo da malha. */
typedef struct {
  /** Tick do passo, em ms. */
  uint32_t time_ms;
  /** Passos executados desde o boot. */
  uint32_t step;
  /** Estado do voo (::flight_state_t). */
  uint8_t state;
  /** Medidas válidas neste passo (::meas_flags_t). */
  uint8_t meas_flags;
  /** Combinação de ::flight_flags_t. */
  uint8_t flags;
  /** Medidas y (::lqg_y_t). */
  float y[LQG_NY];
  /** Estimativa x̂[k|k] (::lqg_x_t). */
  float x_hat[LQG_NX];
  /** Saída du do LQG, em rad/s. */
  float du[LQG_NU];
  /** Referência r em uso (::lqg_ref_t). */
  float ref[LQG_NR];
  /** Aceleração calculada para cada motor (0..1), mesmo no ensaio a seco. */
  float throttle[LQG_NU];
  /** Quatérnio do BNO08x (w, x, y, z), para telemetria. */
  float q[4];
  /** Maior tempo de um passo desde o boot, em us. */
  uint32_t max_step_us;
  /** Passos em que o BNO08x não tinha dado novo. */
  uint32_t bno_misses;
  /** Passos em que o MTF-01 não tinha dado válido. */
  uint32_t mtf_misses;
  /** z NED do chão onde armou, em m; NAN até a primeira medida após armar. */
  float z_ground;
} flight_telemetry_t;

/** @brief Fim de um trecho (TAKEOFF ou leg), para o DroneLink publicar. */
typedef struct {
  /** Situação final (aceito = concluído). */
  flight_request_status_t status;
  /** Motivo, quando não concluído. */
  flight_reason_t reason;
} flight_leg_event_t;

/**
 * @brief Cria a task do voo. Chamar depois de sensors_init() e motors_start().
 * @return true se a task foi criada.
 */
bool flight_ctrl_start(void);

/**
 * @brief Arma. Exige sensores saudáveis e o drone desarmado.
 *
 * Fora do ensaio a seco, arma também os motores, que ficam em MOT_IDLE.
 *
 * @param[in] origin  Quem pediu.
 * @return Resultado imediato.
 */
flight_request_result_t flight_ctrl_arm(flight_origin_t origin);

/**
 * @brief Desarma na hora, em qualquer estado.
 * @param[in] why  Motivo para o log (texto fixo).
 */
void flight_ctrl_disarm(const char *why);

/**
 * @brief Decola até @p height_m acima do chão onde armou e fica em HOLD.
 *
 * O fim é informado por ::flight_ctrl_take_leg_event().
 *
 * @param[in] height_m  Altura, em m (0 = FLT_HOVER_H).
 * @return ::FLIGHT_REQUEST_IN_PROGRESS se aceito.
 */
flight_request_result_t flight_ctrl_takeoff(float height_m);

/**
 * @brief Executa um trecho (girar, transladar ou ir a um ponto local).
 * @param[in] leg  Trecho já validado pelo DroneLink ou montado pelo console.
 * @return ::FLIGHT_REQUEST_IN_PROGRESS se aceito.
 */
flight_request_result_t flight_ctrl_leg(const flight_leg_t *leg);

/**
 * @brief Muda o setpoint do voo: posição local, altura e guinada.
 *
 * Vale em HOLD ou com um trecho em andamento (troca o alvo). x e y são em
 * relação ao ponto de decolagem (0, 0) e só são seguidos com CTL_XY ligado;
 * @p height_m é a altura acima do chão onde armou; @p yaw_rad é relativo à
 * guinada de quando o STM32 foi energizado (zero). A referência anda em rampa
 * (FLT_XY_RATE, FLT_Z_RATE, FLT_YAW_RATE).
 *
 * @return ::FLIGHT_REQUEST_IN_PROGRESS se aceito.
 */
flight_request_result_t flight_ctrl_setpoint(float x_m, float y_m, float height_m, float yaw_rad);

/**
 * @brief Zera a posição horizontal acumulada: o ponto atual vira o (0, 0).
 *
 * A posição x, y vem só da soma do fluxo e deriva com o tempo. Estimativa,
 * referência e alvo horizontais vão a zero juntos (o drone segura onde está);
 * velocidades e o resto do estado não mudam. Vale em qualquer estado.
 *
 * @return ::FLIGHT_REQUEST_ACCEPTED se o pedido entrou na fila.
 */
flight_request_result_t flight_ctrl_xy_zero(void);

/**
 * @brief Para onde está a referência e mantém (HOLD).
 * @param[in] reason  Motivo (::FLIGHT_REASON_NONE quando pedido pelo operador).
 */
flight_request_result_t flight_ctrl_hold(flight_reason_t reason);

/**
 * @brief Pousa onde está e desarma ao tocar o chão.
 * @param[in] reason  Motivo (::FLIGHT_REASON_NONE quando pedido pelo operador).
 */
flight_request_result_t flight_ctrl_land(flight_reason_t reason);

/** @brief Cancela o trecho ativo e fica em HOLD (sem evento de fim). */
flight_request_result_t flight_ctrl_cancel(void);

/** @brief Copia a fotografia do último passo. */
void flight_ctrl_get_telemetry(flight_telemetry_t *out);

/**
 * @brief Retira o evento de fim de trecho pendente, se houver.
 * @param[out] out  Evento.
 * @return true se havia evento.
 */
bool flight_ctrl_take_leg_event(flight_leg_event_t *out);

/** @brief Verdadeiro em qualquer estado armado (ARMED até LAND). */
bool flight_ctrl_is_armed(void);

/** @brief Quem armou o voo atual (vale enquanto armado). */
flight_origin_t flight_ctrl_origin(void);

/** @brief Nome curto de um estado, para o console. */
const char *flight_ctrl_state_name(uint8_t state);

#endif
