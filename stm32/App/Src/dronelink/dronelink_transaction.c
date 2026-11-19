/**
 * @file dronelink_transaction.c
 * @brief Histórico circular de transações de trecho (contratos em
 *        dronelink_transaction.h).
 *
 * Todo acesso ao histórico é feito em seção crítica.
 */
#include "dronelink/dronelink_transaction.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

/** @brief Uma posição do histórico. */
typedef struct {
  /** Posição ocupada. */
  bool valid;
  /** Transação guardada. */
  dronelink_transaction_record_t record;
} transaction_slot_t;

/** Histórico circular de transações. */
static transaction_slot_t history[DRONELINK_TRANSACTION_HISTORY];
/** Próxima posição a ser ocupada (sobrescreve a mais antiga). */
static uint8_t next_slot;

/** @brief Esvazia o histórico (ver dronelink_transaction.h). */
void dronelink_transaction_init(void) {
  memset(history, 0, sizeof(history));
  next_slot = 0U;
}

/** @brief Hash FNV-1a de 32 bits (ver dronelink_transaction.h). */
uint32_t dronelink_transaction_hash(const uint8_t *data, uint16_t length) {
  uint32_t hash = 2166136261UL;
  for (uint16_t index = 0U; index < length; ++index) {
    hash ^= data[index];
    hash *= 16777619UL;
  }
  return hash;
}

/** @brief Procura uma transação e compara o conteúdo (ver dronelink_transaction.h). */
dronelink_transaction_lookup_t dronelink_transaction_find(
    uint32_t mission_id, uint32_t command_id, uint32_t payload_hash,
    dronelink_transaction_record_t *record) {
  taskENTER_CRITICAL();
  for (uint8_t index = 0U; index < DRONELINK_TRANSACTION_HISTORY; ++index) {
    if (!history[index].valid || history[index].record.mission_id != mission_id ||
        history[index].record.command_id != command_id) {
      continue;
    }
    if (history[index].record.payload_hash != payload_hash) {
      taskEXIT_CRITICAL();
      return DRONELINK_TRANSACTION_COLLISION;
    }
    if (record != NULL) {
      *record = history[index].record;
    }
    taskEXIT_CRITICAL();
    return DRONELINK_TRANSACTION_DUPLICATE;
  }
  taskEXIT_CRITICAL();
  return DRONELINK_TRANSACTION_NEW;
}

/** @brief Guarda uma transação (ver dronelink_transaction.h). */
void dronelink_transaction_store(const dronelink_transaction_record_t *record) {
  if (record == NULL) {
    return;
  }
  taskENTER_CRITICAL();
  history[next_slot].valid = true;
  history[next_slot].record = *record;
  next_slot = (uint8_t)((next_slot + 1U) % DRONELINK_TRANSACTION_HISTORY);
  taskEXIT_CRITICAL();
}

/** @brief Atualiza uma transação guardada (ver dronelink_transaction.h). */
bool dronelink_transaction_update(uint32_t mission_id, uint32_t command_id,
                                  uint8_t result, uint16_t reason,
                                  uint8_t progress, uint8_t flight_state,
                                  float remaining_distance, uint32_t eta_ms,
                                  dronelink_transaction_record_t *updated_record) {
  taskENTER_CRITICAL();
  for (uint8_t index = 0U; index < DRONELINK_TRANSACTION_HISTORY; ++index) {
    dronelink_transaction_record_t *record = &history[index].record;
    if (history[index].valid && record->mission_id == mission_id &&
        record->command_id == command_id) {
      record->result = result;
      record->reason = reason;
      record->progress = progress;
      record->flight_state = flight_state;
      record->remaining_distance = remaining_distance;
      record->eta_ms = eta_ms;
      if (updated_record != NULL) {
        *updated_record = *record;
      }
      taskEXIT_CRITICAL();
      return true;
    }
  }
  taskEXIT_CRITICAL();
  return false;
}

/** @brief Lê uma transação guardada (ver dronelink_transaction.h). */
bool dronelink_transaction_get(uint32_t mission_id, uint32_t command_id,
                               dronelink_transaction_record_t *record) {
  taskENTER_CRITICAL();
  for (uint8_t index = 0U; index < DRONELINK_TRANSACTION_HISTORY; ++index) {
    if (history[index].valid && history[index].record.mission_id == mission_id &&
        history[index].record.command_id == command_id) {
      if (record != NULL) {
        *record = history[index].record;
      }
      taskEXIT_CRITICAL();
      return true;
    }
  }
  taskEXIT_CRITICAL();
  return false;
}
