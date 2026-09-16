/**
 * @file dronelink_transport.h
 * @brief Transporte serial do DroneLink: recepção por DMA e transmissão com
 *        duas filas de prioridade.
 *
 * Também é dono dos callbacks globais de UART da HAL
 * (HAL_UARTEx_RxEventCallback, HAL_UART_TxCpltCallback e
 * HAL_UART_ErrorCallback), que repassam os eventos da USART2 ao MTF-01.
 */
#ifndef DRONELINK_TRANSPORT_H
#define DRONELINK_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/**
 * @brief Prioridade de um frame na transmissão.
 */
typedef enum {
  /** Telemetria periódica; enviada quando não há frame crítico na fila. */
  DRONELINK_TX_NORMAL = 0,
  /** Respostas a comandos, HEARTBEAT e TIMESYNC; sempre enviados primeiro. */
  DRONELINK_TX_CRITICAL = 1
} dronelink_tx_priority_t;

/**
 * @brief Cria o stream buffer e as filas e inicia a recepção por DMA.
 *
 * @param[in] uart  UART do enlace (a USART1).
 * @return false se a UART for NULL, se algum recurso não puder ser criado ou
 *         se a recepção não iniciar.
 */
bool dronelink_transport_init(UART_HandleTypeDef *uart);

/**
 * @brief Lê bytes recebidos, esperando até @p timeout_ms se não houver nenhum.
 *
 * @param[out] destination  Destino dos bytes.
 * @param[in]  capacity     Tamanho do destino.
 * @param[in]  timeout_ms   Espera máxima, em ms.
 * @return Quantidade de bytes lidos (0 em caso de timeout ou argumento
 *         inválido).
 */
size_t dronelink_transport_receive(uint8_t *destination, size_t capacity,
                                   uint32_t timeout_ms);

/**
 * @brief Coloca um frame na fila de transmissão, sem esperar.
 *
 * @param[in] data      Frame MAVLink já serializado.
 * @param[in] length    Tamanho do frame (1..280 bytes).
 * @param[in] priority  Fila de destino.
 * @return false se o frame for inválido ou a fila estiver cheia (conta em
 *         dronelink_transport_tx_dropped()).
 */
bool dronelink_transport_send(const uint8_t *data, uint16_t length,
                              dronelink_tx_priority_t priority);

/**
 * @brief Envia o próximo frame da fila, se a UART estiver livre.
 *
 * Deve ser chamada periodicamente pela task do DroneLink. Também reinicia a
 * recepção quando um erro de UART pediu isso.
 */
void dronelink_transport_service_tx(void);

/**
 * @brief Bytes recebidos perdidos (stream buffer cheio) mais erros de UART.
 * @return O contador.
 */
uint32_t dronelink_transport_rx_dropped(void);

/**
 * @brief Frames que não entraram na fila ou cuja transmissão falhou.
 * @return O contador.
 */
uint32_t dronelink_transport_tx_dropped(void);

#endif
