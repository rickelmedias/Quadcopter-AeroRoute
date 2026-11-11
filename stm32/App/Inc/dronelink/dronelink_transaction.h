/**
 * @file dronelink_transaction.h
 * @brief Histórico das últimas transações de trecho, para respostas
 *        idempotentes.
 *
 * Se a Pi reenviar um comando (por exemplo, por não ter recebido a resposta),
 * o firmware responde de novo com o mesmo resultado em vez de executar duas
 * vezes. Um comando com os mesmos IDs mas conteúdo diferente é tratado como
 * colisão. Todas as funções são seguras entre tasks.
 */
#ifndef DRONELINK_TRANSACTION_H
#define DRONELINK_TRANSACTION_H

#include <stdbool.h>
#include <stdint.h>

/** Quantas transações são lembradas; as mais antigas são sobrescritas. */
#define DRONELINK_TRANSACTION_HISTORY 32U

/**
 * @brief Uma transação de trecho e a última resposta enviada para ela.
 */
typedef struct {
  /** Missão do trecho. */
  uint32_t mission_id;
  /** Comando dentro da missão. */
  uint32_t command_id;
  /** Hash da carga útil do comando, para detectar colisão de IDs. */
  uint32_t payload_hash;
  /** Tempo restante estimado, em ms; UINT32_MAX se desconhecido. */
  uint32_t eta_ms;
  /** Distância restante, em m; NaN se desconhecida. */
  float remaining_distance;
  /** Índice do trecho na rota. */
  uint16_t leg_index;
  /** Motivo da recusa ou falha (::flight_reason_t). */
  uint16_t reason;
  /** Resultado no formato MAV_RESULT. */
  uint8_t result;
  /** Progresso 0..100, ou 255 se desconhecido. */
  uint8_t progress;
  /** Estado de voo no momento da resposta (::flight_state_t). */
  uint8_t flight_state;
} dronelink_transaction_record_t;

/**
 * @brief Resultado da busca de uma transação.
 */
typedef enum {
  /** Transação desconhecida: deve ser processada. */
  DRONELINK_TRANSACTION_NEW = 0,
  /** Mesmos IDs e mesmo conteúdo: repetir a resposta anterior. */
  DRONELINK_TRANSACTION_DUPLICATE,
  /** Mesmos IDs com conteúdo diferente: recusar. */
  DRONELINK_TRANSACTION_COLLISION
} dronelink_transaction_lookup_t;

/**
 * @brief Esvazia o histórico.
 */
void dronelink_transaction_init(void);

/**
 * @brief Calcula o hash FNV-1a de 32 bits de um bloco de bytes.
 *
 * @param[in] data    Bytes.
 * @param[in] length  Quantidade de bytes.
 * @return O hash.
 */
uint32_t dronelink_transaction_hash(const uint8_t *data, uint16_t length);

/**
 * @brief Procura uma transação pelos IDs e compara o conteúdo.
 *
 * @param[in]  mission_id    Missão.
 * @param[in]  command_id    Comando.
 * @param[in]  payload_hash  Hash da carga do comando recebido.
 * @param[out] record        Opcional (pode ser NULL). Recebe o registro
 *                           guardado quando o resultado é DUPLICATE.
 * @return NEW, DUPLICATE ou COLLISION.
 */
dronelink_transaction_lookup_t dronelink_transaction_find(uint32_t mission_id, uint32_t command_id,
                                                          uint32_t payload_hash,
                                                          dronelink_transaction_record_t *record);

/**
 * @brief Guarda uma transação, sobrescrevendo a mais antiga se o histórico
 *        estiver cheio.
 *
 * @param[in] record  Registro a guardar (NULL é ignorado).
 */
void dronelink_transaction_store(const dronelink_transaction_record_t *record);

/**
 * @brief Atualiza o resultado e o progresso de uma transação guardada.
 *
 * @param[in]  mission_id          Missão.
 * @param[in]  command_id          Comando.
 * @param[in]  result              Novo resultado (MAV_RESULT).
 * @param[in]  reason              Novo motivo.
 * @param[in]  progress            Novo progresso.
 * @param[in]  flight_state        Estado de voo atual.
 * @param[in]  remaining_distance  Distância restante, em m.
 * @param[in]  eta_ms              Tempo restante, em ms.
 * @param[out] updated_record      Opcional (pode ser NULL). Recebe o registro
 *                                 já atualizado.
 * @return false se a transação não estiver no histórico.
 */
bool dronelink_transaction_update(uint32_t mission_id, uint32_t command_id, uint8_t result,
                                  uint16_t reason, uint8_t progress, uint8_t flight_state,
                                  float remaining_distance, uint32_t eta_ms,
                                  dronelink_transaction_record_t *updated_record);

/**
 * @brief Lê uma transação guardada pelos IDs.
 *
 * @param[in]  mission_id  Missão.
 * @param[in]  command_id  Comando.
 * @param[out] record      Opcional (pode ser NULL). Recebe o registro.
 * @return false se a transação não estiver no histórico.
 */
bool dronelink_transaction_get(uint32_t mission_id, uint32_t command_id,
                               dronelink_transaction_record_t *record);

#endif
