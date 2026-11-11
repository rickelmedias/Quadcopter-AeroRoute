/**
 * @file dronelink_tuning.c
 * @brief Parâmetros e ganhos pela Pi (contrato em dronelink/dronelink_tuning.h).
 */
#include "dronelink/dronelink_tuning.h"

#include "control/ctrl_gains.h"
#include "control/ctrl_params.h"
#include "control/flight_ctrl.h"
#include "dronelink/dronelink_config.h"
#include "sensor_log.h"

#include <stdio.h>
#include <string.h>

/** Intervalo entre dois PARAM_VALUE da lista, em ms. A 115200 bit/s cada um
    leva ~3,2 ms; 8 ms deixa espaço para a telemetria e não enche a fila. */
#define PARAM_LIST_PERIOD_MS  8U
/** Valor do campo action no DRONE_GAIN_ACK que responde a uma linha recusada. */
#define GAIN_ACK_ROW_REJECTED 255U

/** Próximo parâmetro a enviar na lista. */
static size_t list_next;
/** Há uma lista em andamento. */
static bool list_active;
/** Tick do último PARAM_VALUE da lista. */
static uint32_t list_last_ms;

/**
 * @brief Diz se uma mensagem é endereçada a esta controladora.
 * @param[in] system     Sistema de destino.
 * @param[in] component  Componente de destino (0 = todos).
 */
static bool for_fc(uint8_t system, uint8_t component) {
  return system == DRONELINK_SYSTEM_ID &&
         (component == DRONELINK_FC_COMPONENT_ID || component == 0U);
}

/** @brief Envia o PARAM_VALUE de um parâmetro. */
static void send_param(size_t index) {
  const ctrl_param_desc_t *d = ctrl_params_desc(index);
  if (d == NULL)
    return;
  char id[CTRL_PARAM_NAME_MAX + 1U] = {0};
  strncpy(id, d->name, CTRL_PARAM_NAME_MAX);
  mavlink_message_t m;
  mavlink_msg_param_value_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &m, id,
                               ctrl_params_get(index), MAV_PARAM_TYPE_REAL32,
                               (uint16_t)ctrl_params_count(), (uint16_t)index);
  dronelink_send(&m, DRONELINK_TX_NORMAL);
}

/** @brief Envia um STATUSTEXT (aparece na tela da ferramenta da Pi). */
static void send_text(uint8_t severity, const char *text) {
  char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1U] = {0};
  strncpy(buffer, text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
  mavlink_message_t m;
  mavlink_msg_statustext_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &m, severity, buffer,
                              0U, 0U);
  dronelink_send(&m, DRONELINK_TX_NORMAL);
}

/** @brief Envia DRONE_GAIN_ACK com a situação atual dos ganhos. */
static void send_gain_ack(uint32_t session, uint8_t action, ctrl_gains_status_t status,
                          uint32_t staged_crc, uint32_t rows) {
  mavlink_message_t m;
  mavlink_msg_drone_gain_ack_pack(DRONELINK_SYSTEM_ID, DRONELINK_FC_COMPONENT_ID, &m, session,
                                  staged_crc, ctrl_gains_active_crc(), rows, action,
                                  (uint8_t)status, ctrl_gains_active_is_default() ? 0U : 1U,
                                  DRONELINK_SYSTEM_ID, DRONELINK_PI_COMPONENT_ID);
  dronelink_send(&m, DRONELINK_TX_CRITICAL);
}

/** @brief PARAM_REQUEST_LIST: começa a enviar todos os parâmetros. */
static void handle_request_list(const mavlink_message_t *message) {
  mavlink_param_request_list_t req;
  mavlink_msg_param_request_list_decode(message, &req);
  if (!for_fc(req.target_system, req.target_component))
    return;
  list_next = 0U;
  list_active = true;
}

/** @brief PARAM_REQUEST_READ: envia um parâmetro, pelo índice ou pelo nome. */
static void handle_request_read(const mavlink_message_t *message) {
  mavlink_param_request_read_t req;
  mavlink_msg_param_request_read_decode(message, &req);
  if (!for_fc(req.target_system, req.target_component))
    return;
  const int index =
      req.param_index >= 0 ? req.param_index : ctrl_params_find(req.param_id, sizeof(req.param_id));
  if (index >= 0 && (size_t)index < ctrl_params_count()) {
    send_param((size_t)index);
  }
}

/** @brief PARAM_SET: aplica e responde com o valor em uso. */
static void handle_param_set(const mavlink_message_t *message) {
  mavlink_param_set_t req;
  mavlink_msg_param_set_decode(message, &req);
  if (!for_fc(req.target_system, req.target_component))
    return;
  const int index = ctrl_params_find(req.param_id, sizeof(req.param_id));
  char text[64];
  if (index < 0) {
    snprintf(text, sizeof(text), "param %.16s: desconhecido", req.param_id);
    send_text(MAV_SEVERITY_WARNING, text);
    return;
  }
  const ctrl_param_status_t status =
      ctrl_params_set((size_t)index, req.param_value, flight_ctrl_is_armed());
  send_param((size_t)index);
  if (status != CTRL_PARAM_OK) {
    snprintf(text, sizeof(text), "%s: %s", ctrl_params_desc((size_t)index)->name,
             ctrl_params_status_text(status));
    send_text(MAV_SEVERITY_WARNING, text);
  }
}

/** @brief DRONE_GAIN_ROW: guarda a linha no preparo. */
static void handle_gain_row(const mavlink_message_t *message) {
  mavlink_drone_gain_row_t row;
  mavlink_msg_drone_gain_row_decode(message, &row);
  if (!for_fc(row.target_system, row.target_component))
    return;
  const uint8_t count = row.count > CTRL_GAINS_MAX_COLS ? CTRL_GAINS_MAX_COLS + 1U : row.count;
  const ctrl_gains_status_t status = ctrl_gains_stage_row(
      row.session_id, (ctrl_gain_matrix_t)row.matrix, row.row, row.values, count);
  if (status != CTRL_GAINS_OK) {
    /* Linhas aceitas não são respondidas, para não ocupar o enlace. */
    send_gain_ack(row.session_id, GAIN_ACK_ROW_REJECTED, status, 0U, 0U);
  }
}

/** @brief DRONE_GAIN_COMMIT: confirma, aborta, restaura ou consulta. */
static void handle_gain_commit(const mavlink_message_t *message) {
  mavlink_drone_gain_commit_t req;
  mavlink_msg_drone_gain_commit_decode(message, &req);
  if (!for_fc(req.target_system, req.target_component))
    return;
  ctrl_gains_status_t status = CTRL_GAINS_OK;
  uint32_t staged_crc = 0U;
  uint32_t rows = 0U;
  char text[61];
  switch (req.action) {
  case DRONE_GAIN_ACTION_COMMIT:
    status = ctrl_gains_commit(req.session_id, req.crc32, &staged_crc, &rows);
    if (status == CTRL_GAINS_OK) {
      snprintf(text, sizeof(text), "Pi: ganhos novos crc=%08lX", (unsigned long)req.crc32);
      log_text(text);
    }
    break;
  case DRONE_GAIN_ACTION_ABORT:
    ctrl_gains_abort();
    break;
  case DRONE_GAIN_ACTION_RESTORE_DEFAULT:
    ctrl_gains_restore_default();
    log_text("Pi: ganhos do Simulink restaurados");
    break;
  case DRONE_GAIN_ACTION_QUERY:
    break;
  default:
    status = CTRL_GAINS_BAD_INDEX;
    break;
  }
  send_gain_ack(req.session_id, req.action, status, staged_crc, rows);
}

void dronelink_tuning_init(void) {
  list_next = 0U;
  list_active = false;
  ctrl_gains_abort();
}

bool dronelink_tuning_handle(const mavlink_message_t *message) {
  switch (message->msgid) {
  case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    handle_request_list(message);
    return true;
  case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    handle_request_read(message);
    return true;
  case MAVLINK_MSG_ID_PARAM_SET:
    handle_param_set(message);
    return true;
  case MAVLINK_MSG_ID_DRONE_GAIN_ROW:
    handle_gain_row(message);
    return true;
  case MAVLINK_MSG_ID_DRONE_GAIN_COMMIT:
    handle_gain_commit(message);
    return true;
  default:
    return false;
  }
}

void dronelink_tuning_tick(uint32_t now_ms) {
  if (!list_active || now_ms - list_last_ms < PARAM_LIST_PERIOD_MS)
    return;
  list_last_ms = now_ms;
  send_param(list_next);
  if (++list_next >= ctrl_params_count())
    list_active = false;
}
