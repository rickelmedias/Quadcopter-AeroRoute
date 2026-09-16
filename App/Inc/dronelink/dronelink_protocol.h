/**
 * @file dronelink_protocol.h
 * @brief Protocolo DroneLink sobre MAVLink 2 (dialeto em
 *        protocol/drone_controller.xml).
 *
 * Trata as mensagens da Raspberry Pi (HEARTBEAT, TIMESYNC, armar/desarmar,
 * autorização do operador, comando e cancelamento de trechos), publica a
 * telemetria periódica e supervisiona o enlace. Os pedidos de voo são
 * repassados para flight_command_api.h.
 */
#ifndef DRONELINK_PROTOCOL_H
#define DRONELINK_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Zera o estado do protocolo e o histórico de transações.
 *
 * @param[in] boot_id               Identidade deste boot (0 = inválida).
 * @param[in] boot_id_persistent    Se a identidade foi gravada na flash. Sem
 *                                  ela, armar e executar trechos é negado.
 */
void dronelink_protocol_init(uint32_t boot_id, bool boot_id_persistent);

/**
 * @brief Entrega bytes recebidos ao decodificador MAVLink.
 *
 * Cada mensagem completa é tratada na hora. Mensagens que não vêm da Pi
 * (sistema/componente diferentes) são ignoradas.
 *
 * @param[in] data    Bytes recebidos (NULL é ignorado).
 * @param[in] length  Quantidade de bytes.
 */
void dronelink_protocol_consume(const uint8_t *data, uint16_t length);

/**
 * @brief Executa as tarefas periódicas do protocolo.
 *
 * Envia HEARTBEAT e SYS_STATUS a cada 500 ms e DRONE_PROTOCOL_INFO a cada
 * 1 s. Com o drone em voo, supervisiona o enlace e a autorização do operador
 * e pede HOLD ou pouso quando eles se perdem (ver dronelink_config.h).
 *
 * @param[in] now_ms  Tick atual, em ms.
 */
void dronelink_protocol_tick(uint32_t now_ms);

/**
 * @brief Informa o progresso ou o resultado de um trecho em execução.
 *
 * Chamada pelo controle de voo para um trecho que foi aceito com
 * FLIGHT_REQUEST_IN_PROGRESS. Atualiza a transação e envia DRONE_LEG_ACK à
 * Pi. Um resultado diferente de "em andamento" encerra o trecho ativo. Se a
 * transação não estiver no histórico, nada é enviado.
 *
 * @param[in] mission_id          Missão do trecho.
 * @param[in] command_id          Comando do trecho.
 * @param[in] leg_index           Índice do trecho (atualmente não usado).
 * @param[in] result              Resultado no formato MAV_RESULT.
 * @param[in] reason              Motivo (::flight_reason_t).
 * @param[in] progress            Progresso 0..100, ou 255 se desconhecido.
 * @param[in] remaining_distance  Distância restante, em m, ou NaN.
 * @param[in] eta_ms              Tempo restante estimado, em ms, ou UINT32_MAX.
 */
void dronelink_protocol_publish_leg_result(
    uint32_t mission_id, uint32_t command_id, uint16_t leg_index,
    uint8_t result, uint16_t reason, uint8_t progress,
    float remaining_distance, uint32_t eta_ms);

#endif
