/**
 * @file flight_command_api.h
 * @brief Contrato entre o protocolo DroneLink e o controle de voo.
 *
 * O protocolo chama estas funções para armar, executar trechos, cancelar,
 * segurar posição e pousar, e para ler o estado do voo. A implementação fica
 * em App/Src/control/flight_command.c e repassa tudo para
 * control/flight_ctrl.h.
 *
 * Os valores dos enums são os mesmos do dialeto MAVLink
 * (protocol/drone_controller.xml) e vão direto para a Pi.
 */
#ifndef FLIGHT_COMMAND_API_H
#define FLIGHT_COMMAND_API_H

#include "attitude.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Ação de um trecho (DRONE_ACTION).
 */
typedef enum {
  /** Girar ψ em relação à referência atual. */
  FLIGHT_ACTION_ROTATE_RELATIVE = 0,
  /** Transladar em relação à referência atual. */
  FLIGHT_ACTION_TRANSLATE_RELATIVE = 1,
  /** Ir a um ponto do referencial local. */
  FLIGHT_ACTION_GOTO_LOCAL = 2,
  /** Decolar (z negativo = subir, em FRD). */
  FLIGHT_ACTION_TAKEOFF_RELATIVE = 3,
  /** Parar e manter a posição. */
  FLIGHT_ACTION_HOLD = 4,
  /** Pousar. */
  FLIGHT_ACTION_LAND = 5
} flight_action_t;

/** Referencial do corpo: x frente, y direita, z baixo (MAV_FRAME_BODY_FRD). */
#define FLIGHT_FRAME_BODY_FRD  12U
/** Referencial local fixo no ponto em que armou (MAV_FRAME_LOCAL_FRD). */
#define FLIGHT_FRAME_LOCAL_FRD 20U

/**
 * @brief Situação de um pedido feito ao controle de voo.
 */
typedef enum {
  /** Pedido aceito e já concluído. */
  FLIGHT_REQUEST_ACCEPTED = 0,
  /** Pedido aceito e em execução; o fim é informado depois por
      dronelink_protocol_publish_leg_result(). */
  FLIGHT_REQUEST_IN_PROGRESS,
  /** Recusado agora, pode ser aceito mais tarde. */
  FLIGHT_REQUEST_TEMPORARILY_REJECTED,
  /** Recusado. */
  FLIGHT_REQUEST_DENIED,
  /** Aceito, mas falhou. */
  FLIGHT_REQUEST_FAILED
} flight_request_status_t;

/**
 * @brief Motivo de recusa, falha ou mudança de estado (DRONE_FAILURE_REASON).
 */
typedef enum {
  /** Nenhum motivo. */
  FLIGHT_REASON_NONE = 0,
  /** O estado atual de voo não permite o pedido. */
  FLIGHT_REASON_BAD_STATE = 1,
  /** Algum sensor necessário não está saudável. */
  FLIGHT_REASON_SENSOR_UNHEALTHY = 2,
  /** Valor fora dos limites aceitos (distância, velocidade, timeout, quadro...). */
  FLIGHT_REASON_OUT_OF_BOUNDS = 3,
  /** Bateria baixa. */
  FLIGHT_REASON_BATTERY_LOW = 4,
  /** Ocupado com outro pedido. */
  FLIGHT_REASON_BUSY = 5,
  /** O comando foi feito para outro boot do STM32. */
  FLIGHT_REASON_STALE_BOOT = 6,
  /** Trecho fora de ordem ou cancelamento de um trecho que não está ativo. */
  FLIGHT_REASON_SEQUENCE_ERROR = 7,
  /** Algum número do comando não é finito. */
  FLIGHT_REASON_INVALID_NUMBER = 8,
  /** Mesmos IDs de um comando anterior com conteúdo diferente. */
  FLIGHT_REASON_ID_COLLISION = 9,
  /** Função ainda não implementada (ou identidade de boot não persistente). */
  FLIGHT_REASON_NOT_IMPLEMENTED = 10,
  /** Sem autorização válida do operador. */
  FLIGHT_REASON_OPERATOR_LEASE_LOST = 11,
  /** Enlace com a Pi perdido ou degradado. */
  FLIGHT_REASON_LINK_LOST = 12,
  /** O trecho estourou o tempo. */
  FLIGHT_REASON_TIMEOUT = 13,
  /** O comando não era para este componente. */
  FLIGHT_REASON_WRONG_TARGET = 14,
  /** O trecho foi cancelado. */
  FLIGHT_REASON_CANCELLED = 15
} flight_reason_t;

/**
 * @brief Estado da máquina de voo (DRONE_FLIGHT_STATE).
 */
typedef enum {
  /** Inicializando. */
  FLIGHT_STATE_BOOT = 0,
  /** Autoteste. */
  FLIGHT_STATE_SELF_TEST = 1,
  /** Parado e desarmado, pronto para receber pedidos. */
  FLIGHT_STATE_STANDBY = 2,
  /** Verificações antes de armar. */
  FLIGHT_STATE_PREARM = 3,
  /** Armado no chão. */
  FLIGHT_STATE_ARMED = 4,
  /** Decolando. */
  FLIGHT_STATE_TAKEOFF = 5,
  /** Parado no ar, mantendo posição. */
  FLIGHT_STATE_HOLD = 6,
  /** Executando um trecho. */
  FLIGHT_STATE_EXECUTING_LEG = 7,
  /** Pousando. */
  FLIGHT_STATE_LAND = 8,
  /** Modo de segurança. */
  FLIGHT_STATE_FAILSAFE = 9,
  /** Interrupção do voo. */
  FLIGHT_STATE_FLIGHT_TERMINATION = 10
} flight_state_t;

/**
 * @brief Um trecho (leg) da rota, já validado pelo protocolo.
 */
typedef struct {
  /** Ação de guiagem (DRONE_ACTION: girar, transladar, ir a um ponto local,
      decolar, segurar, pousar). */
  uint8_t action;
  /** Referencial das coordenadas: ::FLIGHT_FRAME_BODY_FRD ou ::FLIGHT_FRAME_LOCAL_FRD. */
  uint8_t frame;
  /** Objetivo X no referencial escolhido, em m. */
  float x;
  /** Objetivo Y no referencial escolhido, em m. */
  float y;
  /** Objetivo Z no referencial escolhido, em m. */
  float z;
  /** Yaw desejado ou variação relativa, em rad (|yaw| <= pi). */
  float yaw;
  /** Velocidade de translação máxima, em m/s (0..3). */
  float speed;
  /** Aceleração de translação máxima, em m/s^2 (0..5). */
  float acceleration;
  /** Tolerância de posição para considerar o trecho concluído, em m (0..1). */
  float position_tolerance;
  /** Tolerância de yaw para considerar o trecho concluído, em rad (0..pi/4). */
  float yaw_tolerance;
  /** Tempo máximo de execução, em ms (1..120000). */
  uint32_t timeout_ms;
} flight_leg_t;

/**
 * @brief Estado do voo lido pelo protocolo para telemetria e supervisão.
 */
typedef struct {
  /** Estado da máquina de voo (::flight_state_t). */
  uint8_t state;
  /** Motores armados. */
  bool armed;
  /** Drone no ar. Sem isso, a supervisão de enlace não age. */
  bool airborne;
  /**
   * O voo foi armado pela Raspberry Pi. Voo armado pelo console USB tem as
   * proteções dele (DTR e watchdog do console) e a supervisão de enlace da Pi
   * não age: sem Pi nenhuma, ela pousaria o drone logo depois de decolar.
   */
  bool commanded_by_pi;
  /** Posição confiável. */
  bool position_valid;
  /** Atitude confiável. */
  bool attitude_valid;
  /** Altitude confiável. */
  bool altitude_valid;
  /** Posição X local, em m. */
  float x;
  /** Posição Y local, em m. */
  float y;
  /** Posição Z local, em m. */
  float z;
  /** Velocidade X local, em m/s. */
  float vx;
  /** Velocidade Y local, em m/s. */
  float vy;
  /** Velocidade Z local, em m/s. */
  float vz;
  /** Atitude em Euler ZYX, em radianos. É esta representação que o futuro
      controlador de espaço de estados deve expor e consumir. */
  euler_t attitude;
  /** Representação interna equivalente como quatérnio w,x,y,z. Mantida para
      telemetria MAVLink, interpolação e rotação de vetores; o LQR não precisa
      consumi-la. */
  float q[4];
  /** Velocidade angular p do corpo, em rad/s. Próximo do hover, p ~= roll_dot. */
  float roll_speed;
  /** Velocidade angular q do corpo, em rad/s. Próximo do hover, q ~= pitch_dot. */
  float pitch_speed;
  /** Velocidade angular r do corpo, em rad/s. Próximo do hover, r ~= yaw_dot. */
  float yaw_speed;
} flight_snapshot_t;

/**
 * @brief Resposta do controle de voo a um pedido.
 */
typedef struct {
  /** Situação do pedido. */
  flight_request_status_t status;
  /** Motivo, quando não aceito. */
  flight_reason_t reason;
} flight_request_result_t;

/**
 * @brief Pede para armar ou desarmar os motores.
 *
 * O protocolo já recusa armar sem identidade de boot persistente ou sem
 * autorização do operador. Armar exige sensores saudáveis; com FLT_DRY_RUN
 * ligado, o voo arma sem girar os motores (ensaio a seco).
 *
 * @param[in] arm  true para armar, false para desarmar.
 * @return Resultado do pedido.
 */
flight_request_result_t flight_request_arm(bool arm);

/**
 * @brief Pede a execução de um trecho.
 *
 * Retornar ::FLIGHT_REQUEST_IN_PROGRESS torna o trecho ativo; o fim é
 * informado depois por ::flight_poll_leg_result(), que o protocolo consulta
 * periodicamente.
 *
 * @param[in] leg  Trecho já validado pelo protocolo.
 * @return Resultado do pedido.
 */
flight_request_result_t flight_request_leg(const flight_leg_t *leg);

/**
 * @brief Pede o cancelamento do trecho ativo; o drone fica em HOLD.
 *
 * @return Resultado do pedido.
 */
flight_request_result_t flight_request_cancel(void);

/**
 * @brief Pede para parar e manter a posição.
 *
 * @param[in] reason  Por que o HOLD foi pedido (::FLIGHT_REASON_NONE quando
 *                    veio de um comando da Pi).
 * @return Resultado do pedido.
 */
flight_request_result_t flight_request_hold(flight_reason_t reason);

/**
 * @brief Pede pouso; ao tocar o chão o drone desarma.
 *
 * @param[in] reason  Por que o pouso foi pedido (::FLIGHT_REASON_NONE quando
 *                    veio de um comando da Pi).
 * @return Resultado do pedido.
 */
flight_request_result_t flight_request_land(flight_reason_t reason);

/**
 * @brief Lê o estado atual do voo.
 *
 * Posição, velocidade, atitude e velocidades angulares vêm da estimativa
 * x̂ do Kalman, no referencial local (NED a partir de onde armou).
 *
 * @param[out] snapshot  Destino (NULL é ignorado).
 */
void flight_get_snapshot(flight_snapshot_t *snapshot);

/**
 * @brief Informa se os motores podem ser acionados.
 *
 * @return true se LQG_FLIGHT_ENABLED = 1 e FLT_DRY_RUN = 0.
 */
bool flight_actuation_is_available(void);

/**
 * @brief Retira o resultado final do trecho ativo, se ele terminou.
 *
 * @param[out] status  Situação final (aceito = concluído).
 * @param[out] reason  Motivo, quando não concluído.
 * @return true se havia um resultado pendente.
 */
bool flight_poll_leg_result(flight_request_status_t *status, flight_reason_t *reason);

#endif
