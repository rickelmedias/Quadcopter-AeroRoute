/**
 * @file dronelink_protocol.c
 * @brief Protocolo DroneLink sobre MAVLink 2 (contratos em
 *        dronelink_protocol.h).
 */
#include "dronelink/dronelink_protocol.h"

#include <math.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#include "dronelink/dronelink_config.h"
#include "dronelink/dronelink_mavlink.h"
#include "dronelink/dronelink_telemetry.h"
#include "dronelink/dronelink_transaction.h"
#include "dronelink/dronelink_transport.h"
#include "dronelink/dronelink_tuning.h"
#include "dronelink/flight_command_api.h"
#include "power.h"

/* As ações e os referenciais do contrato de voo têm os valores do dialeto. */
_Static_assert((int)FLIGHT_ACTION_LAND == (int)DRONE_ACTION_LAND, "DRONE_ACTION mudou");
_Static_assert((int)FLIGHT_ACTION_TAKEOFF_RELATIVE == (int)DRONE_ACTION_TAKEOFF_RELATIVE,
               "DRONE_ACTION mudou");
_Static_assert(FLIGHT_FRAME_BODY_FRD == MAV_FRAME_BODY_FRD, "MAV_FRAME mudou");
_Static_assert(FLIGHT_FRAME_LOCAL_FRD == MAV_FRAME_LOCAL_FRD, "MAV_FRAME mudou");

/** @brief Trecho em execução (aceito como "em andamento"). */
typedef struct {
  /** Há um trecho em execução. */
  bool active;
  /** Missão do trecho. */
  uint32_t mission_id;
  /** Comando do trecho. */
  uint32_t command_id;
  /** Índice do trecho. */
  uint16_t leg_index;
} active_leg_t;

/** Estado dos canais MAVLink de todo o firmware (ver dronelink_mavlink.h). */
static mavlink_status_t channel_status[MAVLINK_COMM_NUM_BUFFERS];

struct __mavlink_status *dronelink_mavlink_status(void) { return channel_status; }

/** Estado do decodificador MAVLink. */
static mavlink_status_t parser_status;
/** Mensagem MAVLink em montagem. */
static mavlink_message_t parser_message;
/** Identidade deste boot (0 = inválida). */
static uint32_t current_boot_id;
/** A identidade foi gravada na flash. */
static bool persistent_boot_id;
/** Tick do último HEARTBEAT recebido da Pi. */
static uint32_t last_pi_heartbeat_ms;
/** Tick da última renovação da autorização do operador. */
static uint32_t last_lease_ms;
/** Validade da autorização informada pela Pi, em ms. */
static uint32_t lease_timeout_ms;
/** Alguma autorização já foi recebida neste boot. */
static bool lease_seen;
/** A última autorização recebida estava ativa. */
static bool lease_active;
/** Tick em que a autorização foi perdida; 0 enquanto não foi. */
static uint32_t lease_loss_started_ms;
/** HOLD por perda de enlace já foi pedido. */
static bool link_hold_requested;
/** Pouso por perda de enlace já foi pedido. */
static bool link_land_requested;
/** HOLD por perda de autorização já foi pedido. */
static bool lease_hold_requested;
/** Pouso por perda de autorização já foi pedido. */
static bool lease_land_requested;
/** Tick do último HEARTBEAT enviado. */
static uint32_t last_heartbeat_tx_ms;
/** Tick do último SYS_STATUS enviado. */
static uint32_t last_status_tx_ms;
/** Tick do último DRONE_PROTOCOL_INFO enviado. */
static uint32_t last_protocol_info_tx_ms;
/** Missão do último trecho aceito. */
static uint32_t accepted_mission_id;
/** Índice do último trecho aceito; o próximo precisa ser este + 1. */
static uint16_t last_accepted_leg_index;
/** Trecho em execução. */
static active_leg_t active_leg;

/**
 * @brief Diz se já passou um intervalo, sem erro na volta do contador.
 * @param[in] now       Tick atual.
 * @param[in] since     Tick de referência.
 * @param[in] interval  Intervalo, em ms.
 * @return true se now - since >= interval.
 */
static bool elapsed(uint32_t now, uint32_t since, uint32_t interval) {
  return (uint32_t)(now - since) >= interval;
}

/**
 * @brief Diz se uma mensagem é endereçada a esta controladora.
 * @param[in] system     Sistema de destino.
 * @param[in] component  Componente de destino (0 = todos).
 * @return true se o destino é este sistema e esta controladora (ou todos).
 */
static bool target_is_fc(uint8_t system, uint8_t component) {
  return system == DRONELINK_SYSTEM_ID &&
         (component == DRONELINK_FC_COMPONENT_ID || component == 0U);
}

/**
 * @brief Diz se uma mensagem veio da Raspberry Pi.
 * @param[in] message  Mensagem recebida.
 * @return true se sistema e componente de origem são os da Pi.
 */
static bool source_is_pi(const mavlink_message_t *message) {
  return message->sysid == DRONELINK_SYSTEM_ID &&
         message->compid == DRONELINK_PI_COMPONENT_ID;
}

/** @brief Serializa e enfileira uma mensagem (ver dronelink_mavlink.h). */
void dronelink_send(const mavlink_message_t *message, dronelink_tx_priority_t priority) {
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const uint16_t length = mavlink_msg_to_send_buffer(buffer, message);
  (void)dronelink_transport_send(buffer, length, priority);
}

/**
 * @brief Converte a situação de um pedido de voo para MAV_RESULT.
 * @param[in] status  Situação.
 * @return O valor MAV_RESULT correspondente.
 */
static uint8_t mav_result(flight_request_status_t status) {
  switch (status) {
    case FLIGHT_REQUEST_ACCEPTED:
      return MAV_RESULT_ACCEPTED;
    case FLIGHT_REQUEST_IN_PROGRESS:
      return MAV_RESULT_IN_PROGRESS;
    case FLIGHT_REQUEST_TEMPORARILY_REJECTED:
      return MAV_RESULT_TEMPORARILY_REJECTED;
    case FLIGHT_REQUEST_DENIED:
      return MAV_RESULT_DENIED;
    case FLIGHT_REQUEST_FAILED:
    default:
      return MAV_RESULT_FAILED;
  }
}

/**
 * @brief Envia DRONE_LEG_ACK com os dados de uma transação (fila crítica).
 * @param[in] record  Transação.
 */
static void send_leg_ack_record(const dronelink_transaction_record_t *record) {
  mavlink_message_t message;
  mavlink_msg_drone_leg_ack_pack(
      DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &message,
      current_boot_id, record->mission_id, record->command_id, record->eta_ms,
      record->remaining_distance, record->leg_index, record->reason,
      record->result, record->progress, record->flight_state,
      DRONELINK_SYSTEM_ID, DRONELINK_PI_COMPONENT_ID);
  dronelink_send(&message, DRONELINK_TX_CRITICAL);
}

/**
 * @brief Guarda a resposta de um trecho no histórico e a envia.
 *
 * O tempo e a distância restantes vão como desconhecidos, e o estado de voo
 * é lido na hora.
 *
 * @param[in] mission_id    Missão.
 * @param[in] command_id    Comando.
 * @param[in] leg_index     Índice do trecho.
 * @param[in] payload_hash  Hash da carga do comando.
 * @param[in] result        Resultado MAV_RESULT.
 * @param[in] reason        Motivo.
 * @param[in] progress      Progresso 0..100.
 */
static void reply_leg(uint32_t mission_id, uint32_t command_id,
                      uint16_t leg_index, uint32_t payload_hash,
                      uint8_t result, uint16_t reason, uint8_t progress) {
  flight_snapshot_t snapshot;
  flight_get_snapshot(&snapshot);
  const dronelink_transaction_record_t record = {
      .mission_id = mission_id,
      .command_id = command_id,
      .payload_hash = payload_hash,
      .eta_ms = UINT32_MAX,
      .remaining_distance = NAN,
      .leg_index = leg_index,
      .reason = reason,
      .result = result,
      .progress = progress,
      .flight_state = snapshot.state,
  };
  dronelink_transaction_store(&record);
  send_leg_ack_record(&record);
}

/**
 * @brief Diz se há autorização do operador válida agora.
 * @param[in] now_ms  Tick atual.
 * @return true se foi recebida, está ativa e não expirou.
 */
static bool lease_is_valid(uint32_t now_ms) {
  return lease_seen && lease_active && !elapsed(now_ms, last_lease_ms, lease_timeout_ms);
}

/**
 * @brief Diz se todos os números de um comando de trecho são finitos.
 * @param[in] command  Comando.
 * @return true se nenhum é NaN ou infinito.
 */
static bool finite_leg(const mavlink_drone_leg_command_t *command) {
  return isfinite(command->x) && isfinite(command->y) && isfinite(command->z) &&
         isfinite(command->yaw) && isfinite(command->speed) &&
         isfinite(command->acceleration) && isfinite(command->position_tolerance) &&
         isfinite(command->yaw_tolerance);
}

/**
 * @brief Valida um comando de trecho antes de repassá-lo ao controle de voo.
 *
 * Verifica, nesta ordem: destino; identidade de boot; identidade persistente;
 * números finitos; timeout (1..120000 ms), ação e referencial; autorização do
 * operador e enlace recente (exceto para HOLD e LAND); ordem dos trechos (o
 * próximo índice da mesma missão, ou 1 numa missão nova); e limites (até 20 m
 * de translação, |yaw| <= pi, velocidade 0..3 m/s, aceleração 0..5 m/s^2,
 * tolerância de posição 0..1 m e de yaw 0..pi/4).
 *
 * @param[in] command  Comando recebido.
 * @param[in] now_ms   Tick atual.
 * @return ::FLIGHT_REASON_NONE se válido; senão, o motivo da recusa.
 */
static flight_reason_t validate_leg(const mavlink_drone_leg_command_t *command,
                                    uint32_t now_ms) {
  if (!target_is_fc(command->target_system, command->target_component)) {
    return FLIGHT_REASON_WRONG_TARGET;
  }
  if (command->boot_id != current_boot_id || current_boot_id == 0U) {
    return FLIGHT_REASON_STALE_BOOT;
  }
  if (!persistent_boot_id) {
    return FLIGHT_REASON_NOT_IMPLEMENTED;
  }
  if (!finite_leg(command)) {
    return FLIGHT_REASON_INVALID_NUMBER;
  }
  if (command->timeout_ms == 0U || command->timeout_ms > DRONELINK_MAX_LEG_TIMEOUT_MS ||
      command->action > DRONE_ACTION_LAND ||
      (command->frame != MAV_FRAME_BODY_FRD &&
       command->frame != MAV_FRAME_LOCAL_FRD)) {
    return FLIGHT_REASON_OUT_OF_BOUNDS;
  }
  const bool safety_action = command->action == DRONE_ACTION_HOLD ||
                             command->action == DRONE_ACTION_LAND;
  if (!safety_action && !lease_is_valid(now_ms)) {
    return FLIGHT_REASON_OPERATOR_LEASE_LOST;
  }
  if (!safety_action &&
      elapsed(now_ms, last_pi_heartbeat_ms, DRONELINK_LINK_DEGRADED_MS)) {
    return FLIGHT_REASON_LINK_LOST;
  }
  if (accepted_mission_id == command->mission_id) {
    if (command->leg_index != (uint16_t)(last_accepted_leg_index + 1U)) {
      return FLIGHT_REASON_SEQUENCE_ERROR;
    }
  } else if (command->leg_index != 1U) {
    return FLIGHT_REASON_SEQUENCE_ERROR;
  }
  const float translation = sqrtf(command->x * command->x + command->y * command->y +
                                  command->z * command->z);
  if (translation > 20.0F || fabsf(command->yaw) > 3.141593F ||
      command->speed < 0.0F || command->speed > 3.0F ||
      command->acceleration < 0.0F || command->acceleration > 5.0F ||
      command->position_tolerance < 0.0F || command->position_tolerance > 1.0F ||
      command->yaw_tolerance < 0.0F || command->yaw_tolerance > 0.785399F) {
    return FLIGHT_REASON_OUT_OF_BOUNDS;
  }
  return FLIGHT_REASON_NONE;
}

/** @brief Trata HEARTBEAT da Pi: renova o enlace e libera novos pedidos de HOLD/pouso por perda de enlace. */
static void handle_heartbeat(void) {
  last_pi_heartbeat_ms = HAL_GetTick();
  link_hold_requested = false;
  link_land_requested = false;
}

/**
 * @brief Responde a um pedido de TIMESYNC (tc1 == 0) com a hora local em ns.
 * @param[in] message  Mensagem TIMESYNC.
 */
static void handle_timesync(const mavlink_message_t *message) {
  mavlink_timesync_t request;
  mavlink_msg_timesync_decode(message, &request);
  if (request.tc1 != 0) {
    return;
  }
  mavlink_message_t response;
  const int64_t now_ns = (int64_t)HAL_GetTick() * 1000000LL;
  mavlink_msg_timesync_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID,
                            &response, now_ns, request.ts1);
  dronelink_send(&response, DRONELINK_TX_CRITICAL);
}

/**
 * @brief Trata DRONE_CONTROL_LEASE: valida destino, boot e validade
 *        (100..2000 ms) e registra a renovação ou a perda da autorização.
 * @param[in] message  Mensagem recebida.
 */
static void handle_lease(const mavlink_message_t *message) {
  mavlink_drone_control_lease_t lease;
  mavlink_msg_drone_control_lease_decode(message, &lease);
  if (!target_is_fc(lease.target_system, lease.target_component) ||
      lease.boot_id != current_boot_id || lease.timeout_ms < 100U ||
      lease.timeout_ms > 2000U) {
    return;
  }
  last_lease_ms = HAL_GetTick();
  lease_timeout_ms = lease.timeout_ms;
  lease_seen = true;
  lease_active = (lease.flags & DRONE_LEASE_FLAG_ACTIVE) != 0U;
  if (lease_active) {
    lease_loss_started_ms = 0U;
    lease_hold_requested = false;
    lease_land_requested = false;
  } else if (lease_loss_started_ms == 0U) {
    lease_loss_started_ms = last_lease_ms;
  }
}

/**
 * @brief Trata COMMAND_LONG de armar/desarmar e responde com COMMAND_ACK.
 *
 * Armar exige identidade de boot persistente e autorização válida.
 *
 * @param[in] message  Mensagem recebida.
 */
static void handle_arm_command(const mavlink_message_t *message) {
  mavlink_command_long_t command;
  mavlink_msg_command_long_decode(message, &command);
  if (!target_is_fc(command.target_system, command.target_component) ||
      command.command != MAV_CMD_COMPONENT_ARM_DISARM) {
    return;
  }
  const bool arm = command.param1 >= 0.5F;
  flight_request_result_t result;
  if (arm && (!persistent_boot_id || !lease_is_valid(HAL_GetTick()))) {
    result.status = FLIGHT_REQUEST_DENIED;
    result.reason = !persistent_boot_id ? FLIGHT_REASON_NOT_IMPLEMENTED
                                        : FLIGHT_REASON_OPERATOR_LEASE_LOST;
  } else {
    result = flight_request_arm(arm);
  }
  mavlink_message_t response;
  mavlink_msg_command_ack_pack(
      DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &response,
      MAV_CMD_COMPONENT_ARM_DISARM, mav_result(result.status), 0U,
      (int32_t)result.reason, DRONELINK_SYSTEM_ID, DRONELINK_PI_COMPONENT_ID);
  dronelink_send(&response, DRONELINK_TX_CRITICAL);
}

/**
 * @brief Trata DRONE_LEG_COMMAND.
 *
 * Repete a resposta de um comando duplicado, recusa colisão de IDs, valida o
 * comando, repassa ao controle de voo (HOLD, LAND ou trecho) e responde. Um
 * trecho aceito avança a sequência; um trecho em andamento vira o trecho
 * ativo.
 *
 * @param[in] message  Mensagem recebida.
 */
static void handle_leg(const mavlink_message_t *message) {
  mavlink_drone_leg_command_t command;
  mavlink_msg_drone_leg_command_decode(message, &command);
  const uint32_t payload_hash = dronelink_transaction_hash(
      (const uint8_t *)_MAV_PAYLOAD(message), message->len);
  dronelink_transaction_record_t previous;
  const dronelink_transaction_lookup_t lookup = dronelink_transaction_find(
      command.mission_id, command.command_id, payload_hash, &previous);
  if (lookup == DRONELINK_TRANSACTION_DUPLICATE) {
    send_leg_ack_record(&previous);
    return;
  }
  if (lookup == DRONELINK_TRANSACTION_COLLISION) {
    reply_leg(command.mission_id, command.command_id, command.leg_index,
              payload_hash, MAV_RESULT_DENIED, FLIGHT_REASON_ID_COLLISION, 0U);
    return;
  }
  const flight_reason_t validation = validate_leg(&command, HAL_GetTick());
  if (validation != FLIGHT_REASON_NONE) {
    reply_leg(command.mission_id, command.command_id, command.leg_index,
              payload_hash, MAV_RESULT_DENIED, validation, 0U);
    return;
  }

  const flight_leg_t leg = {
      .action = command.action,
      .frame = command.frame,
      .x = command.x,
      .y = command.y,
      .z = command.z,
      .yaw = command.yaw,
      .speed = command.speed,
      .acceleration = command.acceleration,
      .position_tolerance = command.position_tolerance,
      .yaw_tolerance = command.yaw_tolerance,
      .timeout_ms = command.timeout_ms,
  };
  flight_request_result_t result;
  if (command.action == DRONE_ACTION_HOLD) {
    result = flight_request_hold(FLIGHT_REASON_NONE);
  } else if (command.action == DRONE_ACTION_LAND) {
    result = flight_request_land(FLIGHT_REASON_NONE);
  } else {
    result = flight_request_leg(&leg);
  }
  const uint8_t wire_result = mav_result(result.status);
  reply_leg(command.mission_id, command.command_id, command.leg_index,
            payload_hash, wire_result, result.reason,
            wire_result == MAV_RESULT_ACCEPTED ? 100U : 0U);
  if (wire_result == MAV_RESULT_ACCEPTED || wire_result == MAV_RESULT_IN_PROGRESS) {
    accepted_mission_id = command.mission_id;
    last_accepted_leg_index = command.leg_index;
  }
  if (wire_result == MAV_RESULT_IN_PROGRESS) {
    active_leg.active = true;
    active_leg.mission_id = command.mission_id;
    active_leg.command_id = command.command_id;
    active_leg.leg_index = command.leg_index;
  }
}

/**
 * @brief Trata DRONE_LEG_CANCEL.
 *
 * Recusa boot errado ou trecho que não é o ativo, repete a resposta de um
 * cancelamento já feito, e senão pede o cancelamento ao controle de voo e
 * publica o resultado.
 *
 * @param[in] message  Mensagem recebida.
 */
static void handle_cancel(const mavlink_message_t *message) {
  mavlink_drone_leg_cancel_t cancel;
  mavlink_msg_drone_leg_cancel_decode(message, &cancel);
  if (!target_is_fc(cancel.target_system, cancel.target_component)) {
    return;
  }
  flight_snapshot_t snapshot;
  flight_get_snapshot(&snapshot);
  if (cancel.boot_id != current_boot_id || current_boot_id == 0U) {
    const dronelink_transaction_record_t response = {
        .mission_id = cancel.mission_id,
        .command_id = cancel.command_id,
        .eta_ms = UINT32_MAX,
        .remaining_distance = NAN,
        .leg_index = cancel.leg_index,
        .reason = FLIGHT_REASON_STALE_BOOT,
        .result = MAV_RESULT_DENIED,
        .progress = 0U,
        .flight_state = snapshot.state,
    };
    send_leg_ack_record(&response);
    return;
  }
  dronelink_transaction_record_t previous;
  if (dronelink_transaction_get(cancel.mission_id, cancel.command_id, &previous) &&
      previous.leg_index == cancel.leg_index &&
      previous.reason == FLIGHT_REASON_CANCELLED) {
    send_leg_ack_record(&previous);
    return;
  }
  if (!active_leg.active || active_leg.mission_id != cancel.mission_id ||
      active_leg.command_id != cancel.command_id ||
      active_leg.leg_index != cancel.leg_index) {
    const dronelink_transaction_record_t response = {
        .mission_id = cancel.mission_id,
        .command_id = cancel.command_id,
        .eta_ms = UINT32_MAX,
        .remaining_distance = NAN,
        .leg_index = cancel.leg_index,
        .reason = FLIGHT_REASON_SEQUENCE_ERROR,
        .result = MAV_RESULT_DENIED,
        .progress = 0U,
        .flight_state = snapshot.state,
    };
    send_leg_ack_record(&response);
    return;
  }
  const flight_request_result_t result = flight_request_cancel();
  dronelink_protocol_publish_leg_result(
      cancel.mission_id, cancel.command_id, cancel.leg_index,
      mav_result(result.status),
      result.status == FLIGHT_REQUEST_ACCEPTED ? FLIGHT_REASON_CANCELLED
                                               : result.reason,
      result.status == FLIGHT_REQUEST_ACCEPTED ? 100U : 0U, NAN, UINT32_MAX);
  if (result.status == FLIGHT_REQUEST_ACCEPTED) {
    active_leg.active = false;
  }
}

/**
 * @brief Encaminha uma mensagem da Pi para o tratador do seu tipo; as demais
 *        são ignoradas.
 * @param[in] message  Mensagem recebida.
 */
static void handle_message(const mavlink_message_t *message) {
  if (!source_is_pi(message)) {
    return;
  }
  if (dronelink_tuning_handle(message)) {
    return;
  }
  switch (message->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      handle_heartbeat();
      break;
    case MAVLINK_MSG_ID_TIMESYNC:
      handle_timesync(message);
      break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
      handle_arm_command(message);
      break;
    case MAVLINK_MSG_ID_DRONE_CONTROL_LEASE:
      handle_lease(message);
      break;
    case MAVLINK_MSG_ID_DRONE_LEG_COMMAND:
      handle_leg(message);
      break;
    case MAVLINK_MSG_ID_DRONE_LEG_CANCEL:
      handle_cancel(message);
      break;
    default:
      break;
  }
}

/** @brief Envia HEARTBEAT com o estado de voo e o indicador de armado (fila crítica). */
static void publish_heartbeat(void) {
  flight_snapshot_t snapshot;
  flight_get_snapshot(&snapshot);
  const uint8_t base_mode = snapshot.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0U;
  const uint8_t system_status = snapshot.state == DRONE_FLIGHT_STATE_FAILSAFE
                                    ? MAV_STATE_CRITICAL
                                    : MAV_STATE_ACTIVE;
  mavlink_message_t message;
  mavlink_msg_heartbeat_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID,
                             &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
                             base_mode, snapshot.state, system_status);
  dronelink_send(&message, DRONELINK_TX_CRITICAL);
}

/** @brief Envia DRONE_PROTOCOL_INFO com identidade de boot, capacidades e versão. */
static void publish_protocol_info(void) {
  flight_snapshot_t snapshot;
  flight_get_snapshot(&snapshot);
  uint32_t capabilities = DRONE_PROTOCOL_CAPABILITY_MAVLINK2 |
                          DRONE_PROTOCOL_CAPABILITY_TRANSACTIONAL_LEGS |
                          DRONE_PROTOCOL_CAPABILITY_OPERATOR_LEASE |
                          DRONE_PROTOCOL_CAPABILITY_TUNING;
  if (persistent_boot_id) {
    capabilities |= DRONE_PROTOCOL_CAPABILITY_PERSISTENT_BOOT_ID;
  }
  if (flight_actuation_is_available()) {
    capabilities |= DRONE_PROTOCOL_CAPABILITY_ACTUATION;
  }
  if (snapshot.position_valid) {
    capabilities |= DRONE_PROTOCOL_CAPABILITY_LOCAL_POSITION;
  }
  mavlink_message_t message;
  mavlink_msg_drone_protocol_info_pack(
      DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &message,
      current_boot_id, capabilities, DRONELINK_MAX_LEG_TIMEOUT_MS,
      DRONELINK_PROTOCOL_MAJOR, DRONELINK_PROTOCOL_MINOR, snapshot.state,
      0U, 0U);
  dronelink_send(&message, DRONELINK_TX_NORMAL);
}

/**
 * @brief Envia SYS_STATUS com o total de perdas do enlace e, quando a leitura
 *        estiver ligada (power.h), a tensão e a corrente da bateria.
 */
static void publish_status(void) {
  mavlink_message_t message;
  const uint32_t errors = dronelink_transport_rx_dropped() +
                          dronelink_transport_tx_dropped();
  power_sample_t power;
  power_get(&power);
  /* Sem leitura: UINT16_MAX e -1 significam "desconhecido" no MAVLink. */
  const uint16_t millivolts =
      power.voltage_valid ? (uint16_t)(power.voltage_v * 1000.0F) : UINT16_MAX;
  const int16_t centiamps =
      power.current_valid ? (int16_t)(power.current_a * 100.0F) : -1;
  mavlink_msg_sys_status_pack(
      DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &message,
      0U, 0U, 0U, 0U, millivolts, centiamps, -1, 0U,
      (uint16_t)(errors > UINT16_MAX ? UINT16_MAX : errors),
      0U, 0U, 0U, 0U, 0U, 0U, 0U);
  dronelink_send(&message, DRONELINK_TX_NORMAL);
}

/**
 * @brief Supervisiona o enlace e a autorização enquanto o drone está no ar.
 *
 * Só age em voo armado pela Pi: um voo armado pelo console USB não tem Pi
 * nenhuma, e antes era mandado pousar assim que decolava (ensaio de 04/11).
 * Sem HEARTBEAT por 1 s pede HOLD; por 2 s com navegação degradada, ou por
 * 5 s em qualquer caso, pede pouso. Sem autorização pede HOLD, e após 5 s sem
 * ela pede pouso. Cada pedido é feito uma única vez.
 *
 * @param[in] now_ms  Tick atual.
 */
static void supervise_links(uint32_t now_ms) {
  flight_snapshot_t snapshot;
  flight_get_snapshot(&snapshot);
  if (!snapshot.airborne || !snapshot.commanded_by_pi) {
    return;
  }
  const uint32_t link_age = now_ms - last_pi_heartbeat_ms;
  if (link_age >= DRONELINK_LINK_DEGRADED_MS && !link_hold_requested) {
    (void)flight_request_hold(FLIGHT_REASON_LINK_LOST);
    link_hold_requested = true;
  }
  const bool navigation_degraded = !snapshot.position_valid ||
                                   !snapshot.altitude_valid ||
                                   !snapshot.attitude_valid;
  if (((link_age >= DRONELINK_LINK_LOST_MS && navigation_degraded) ||
       link_age >= DRONELINK_FAILSAFE_LAND_MS) && !link_land_requested) {
    (void)flight_request_land(FLIGHT_REASON_LINK_LOST);
    link_land_requested = true;
  }
  if (lease_seen) {
    const uint32_t lease_age = now_ms - last_lease_ms;
    const bool lease_lost = !lease_active || lease_age >= lease_timeout_ms;
    if (lease_lost && lease_loss_started_ms == 0U) {
      lease_loss_started_ms = lease_active
                                  ? last_lease_ms + lease_timeout_ms
                                  : now_ms;
    }
    if (lease_lost && !lease_hold_requested) {
      (void)flight_request_hold(FLIGHT_REASON_OPERATOR_LEASE_LOST);
      lease_hold_requested = true;
    }
    if (lease_lost &&
        elapsed(now_ms, lease_loss_started_ms, DRONELINK_FAILSAFE_LAND_MS) &&
        !lease_land_requested) {
      (void)flight_request_land(FLIGHT_REASON_OPERATOR_LEASE_LOST);
      lease_land_requested = true;
    }
  }
}

/**
 * @brief Publica o fim do trecho ativo quando o controle de voo o informa.
 *
 * Concluído vira MAV_RESULT_ACCEPTED com progresso 100; qualquer outro fim
 * vira MAV_RESULT_FAILED com o motivo dado pelo voo.
 */
static void publish_finished_leg(void) {
  flight_request_status_t status;
  flight_reason_t reason;
  if (!flight_poll_leg_result(&status, &reason) || !active_leg.active) {
    return;
  }
  const bool done = status == FLIGHT_REQUEST_ACCEPTED;
  dronelink_protocol_publish_leg_result(active_leg.mission_id, active_leg.command_id,
                                        active_leg.leg_index,
                                        done ? MAV_RESULT_ACCEPTED : mav_result(status),
                                        (uint16_t)reason, done ? 100U : 0U, done ? 0.0F : NAN,
                                        done ? 0U : UINT32_MAX);
}

/** @brief Zera o estado do protocolo (ver dronelink_protocol.h). */
void dronelink_protocol_init(uint32_t boot_id, bool boot_id_is_persistent) {
  memset(&parser_status, 0, sizeof(parser_status));
  memset(&parser_message, 0, sizeof(parser_message));
  current_boot_id = boot_id;
  persistent_boot_id = boot_id_is_persistent;
  last_pi_heartbeat_ms = HAL_GetTick();
  last_lease_ms = 0U;
  lease_timeout_ms = DRONELINK_LINK_DEGRADED_MS;
  lease_seen = false;
  lease_active = false;
  lease_loss_started_ms = 0U;
  link_hold_requested = false;
  link_land_requested = false;
  lease_hold_requested = false;
  lease_land_requested = false;
  last_heartbeat_tx_ms = 0U;
  last_status_tx_ms = 0U;
  last_protocol_info_tx_ms = 0U;
  accepted_mission_id = 0U;
  last_accepted_leg_index = 0U;
  memset(&active_leg, 0, sizeof(active_leg));
  dronelink_transaction_init();
  dronelink_tuning_init();
  dronelink_telemetry_init();
}

/** @brief Decodifica bytes recebidos e trata as mensagens (ver dronelink_protocol.h). */
void dronelink_protocol_consume(const uint8_t *data, uint16_t length) {
  if (data == NULL) {
    return;
  }
  for (uint16_t index = 0U; index < length; ++index) {
    if (mavlink_parse_char(MAVLINK_COMM_0, data[index], &parser_message,
                           &parser_status) != 0U) {
      handle_message(&parser_message);
    }
  }
}

/** @brief Telemetria periódica e supervisão (ver dronelink_protocol.h). */
void dronelink_protocol_tick(uint32_t now_ms) {
  if (elapsed(now_ms, last_heartbeat_tx_ms, DRONELINK_HEARTBEAT_PERIOD_MS)) {
    publish_heartbeat();
    last_heartbeat_tx_ms = now_ms;
  }
  if (elapsed(now_ms, last_status_tx_ms, DRONELINK_STATUS_PERIOD_MS)) {
    publish_status();
    last_status_tx_ms = now_ms;
  }
  if (elapsed(now_ms, last_protocol_info_tx_ms, DRONELINK_PROTOCOL_INFO_PERIOD_MS)) {
    publish_protocol_info();
    last_protocol_info_tx_ms = now_ms;
  }
  supervise_links(now_ms);
  publish_finished_leg();
  dronelink_tuning_tick(now_ms);
  dronelink_telemetry_tick(now_ms);
}

/** @brief Publica progresso ou resultado de um trecho (ver dronelink_protocol.h). */
void dronelink_protocol_publish_leg_result(
    uint32_t mission_id, uint32_t command_id, uint16_t leg_index,
    uint8_t result, uint16_t reason, uint8_t progress,
    float remaining_distance, uint32_t eta_ms) {
  flight_snapshot_t snapshot;
  flight_get_snapshot(&snapshot);
  dronelink_transaction_record_t record;
  if (!dronelink_transaction_update(mission_id, command_id, result, reason,
                                    progress, snapshot.state,
                                    remaining_distance, eta_ms, &record)) {
    return;
  }
  send_leg_ack_record(&record);
  if (result != MAV_RESULT_IN_PROGRESS) {
    active_leg.active = false;
  }
  (void)leg_index;
}
