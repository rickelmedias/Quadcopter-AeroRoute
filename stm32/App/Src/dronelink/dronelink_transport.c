/**
 * @file dronelink_transport.c
 * @brief Transporte serial do DroneLink na USART1 (contratos em
 *        dronelink_transport.h).
 *
 * Recepção: DMA circular -> interrupção -> stream buffer -> task. Transmissão:
 * duas filas (crítica e normal) -> DMA, um frame por vez. Os eventos da HAL
 * chegam por transport/uart_callbacks.c.
 */
#include "dronelink/dronelink_transport.h"

#include "dma_ring.h"
#include "dronelink/dronelink_config.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"
#include "task.h"

#include <string.h>

/** Maior frame MAVLink 2 aceito para envio, em bytes. */
#define MAVLINK_FRAME_MAX 280U

/** @brief Um frame guardado na fila de transmissão. */
typedef struct {
  /** Bytes usados em @c data. */
  uint16_t length;
  /** Frame já serializado. */
  uint8_t data[MAVLINK_FRAME_MAX];
} tx_frame_t;

/** UART do enlace; NULL até a inicialização. */
static UART_HandleTypeDef *link_uart;
/** Buffer circular de recepção preenchido pelo DMA. */
static uint8_t rx_dma[DRONELINK_RX_DMA_SIZE];
/** Posição do DMA no evento anterior (ver dma_ring.h). */
static volatile uint16_t rx_last_position;
/** Bytes recebidos à espera da task. */
static StreamBufferHandle_t rx_stream;
static StaticStreamBuffer_t rx_stream_control;
static uint8_t rx_stream_storage[DRONELINK_RX_STREAM_SIZE + 1U];
/** Fila de frames críticos (enviados primeiro) e de telemetria. */
static QueueHandle_t tx_critical_queue;
static QueueHandle_t tx_normal_queue;
static StaticQueue_t tx_critical_control;
static StaticQueue_t tx_normal_control;
static uint8_t tx_critical_storage[DRONELINK_TX_QUEUE_DEPTH * sizeof(tx_frame_t)];
static uint8_t tx_normal_storage[DRONELINK_TX_QUEUE_DEPTH * sizeof(tx_frame_t)];
/** Frame em transmissão; precisa continuar válido até o fim do DMA. */
static tx_frame_t tx_active_frame;
/** A UART está transmitindo um frame. */
static volatile bool tx_busy;
/** Um erro de UART pediu o reinício da recepção. */
static volatile bool rx_restart_requested;
/** Bytes recebidos perdidos mais erros de UART. */
static volatile uint32_t rx_dropped;
/** Frames não enfileirados ou com falha de envio. */
static volatile uint32_t tx_dropped;

bool dronelink_transport_init(UART_HandleTypeDef *uart) {
  if (uart == NULL) {
    return false;
  }
  rx_stream = xStreamBufferCreateStatic(sizeof(rx_stream_storage), 1U, rx_stream_storage,
                                        &rx_stream_control);
  tx_critical_queue = xQueueCreateStatic(DRONELINK_TX_QUEUE_DEPTH, sizeof(tx_frame_t),
                                         tx_critical_storage, &tx_critical_control);
  tx_normal_queue = xQueueCreateStatic(DRONELINK_TX_QUEUE_DEPTH, sizeof(tx_frame_t),
                                       tx_normal_storage, &tx_normal_control);
  if ((rx_stream == NULL) || (tx_critical_queue == NULL) || (tx_normal_queue == NULL)) {
    return false;
  }
  rx_last_position = 0U;
  tx_busy = false;
  rx_restart_requested = false;
  link_uart = uart;
  (void)HAL_UART_AbortReceive(link_uart);
  const bool ok = HAL_UARTEx_ReceiveToIdle_DMA(link_uart, rx_dma, sizeof(rx_dma)) == HAL_OK;
  if (!ok) {
    rx_restart_requested = true; /* tenta de novo em dronelink_transport_service_tx() */
  }
  return true;
}

size_t dronelink_transport_receive(uint8_t *destination, size_t capacity, uint32_t timeout_ms) {
  if ((rx_stream == NULL) || (destination == NULL) || (capacity == 0U)) {
    return 0U;
  }
  return xStreamBufferReceive(rx_stream, destination, capacity, pdMS_TO_TICKS(timeout_ms));
}

bool dronelink_transport_send(const uint8_t *data, uint16_t length,
                              dronelink_tx_priority_t priority) {
  if ((data == NULL) || (length == 0U) || (length > MAVLINK_FRAME_MAX)) {
    return false;
  }
  tx_frame_t frame = {.length = length};
  (void)memcpy(frame.data, data, length);
  QueueHandle_t queue = (priority == DRONELINK_TX_CRITICAL) ? tx_critical_queue : tx_normal_queue;
  if ((queue == NULL) || (xQueueSend(queue, &frame, 0U) != pdPASS)) {
    ++tx_dropped;
    return false;
  }
  return true;
}

void dronelink_transport_service_tx(void) {
  if (rx_restart_requested && (link_uart != NULL)) {
    taskENTER_CRITICAL();
    rx_restart_requested = false;
    taskEXIT_CRITICAL();
    (void)HAL_UART_AbortReceive(link_uart);
    rx_last_position = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(link_uart, rx_dma, sizeof(rx_dma)) != HAL_OK) {
      rx_restart_requested = true;
    }
  }
  if (tx_busy || (link_uart == NULL)) {
    return;
  }
  BaseType_t received = xQueueReceive(tx_critical_queue, &tx_active_frame, 0U);
  if (received != pdPASS) {
    received = xQueueReceive(tx_normal_queue, &tx_active_frame, 0U);
  }
  if (received == pdPASS) {
    tx_busy = true;
    if (HAL_UART_Transmit_DMA(link_uart, tx_active_frame.data, tx_active_frame.length) != HAL_OK) {
      tx_busy = false;
      ++tx_dropped;
    }
  }
}

uint32_t dronelink_transport_rx_dropped(void) {
  return rx_dropped;
}

uint32_t dronelink_transport_tx_dropped(void) {
  return tx_dropped;
}

void dronelink_transport_rx_event(UART_HandleTypeDef *uart, uint16_t position) {
  if ((uart != link_uart) || (link_uart == NULL) || (rx_stream == NULL) ||
      (position > DRONELINK_RX_DMA_SIZE)) {
    return;
  }
  /* Mesmo dma_ring.h da recepção do MTF-01 (evento repetido da HAL não reenvia o buffer). */
  dma_ring_segment_t seg[2];
  const unsigned count =
      dma_ring_segments(rx_last_position, position, DRONELINK_RX_DMA_SIZE, seg);
  BaseType_t woken = pdFALSE;
  for (unsigned i = 0U; i < count; ++i) {
    const size_t length = seg[i].length;
    const size_t sent =
        xStreamBufferSendFromISR(rx_stream, &rx_dma[seg[i].offset], length, &woken);
    if (sent < length) {
      rx_dropped += (uint32_t)(length - sent);
    }
  }
  rx_last_position = position;
  portYIELD_FROM_ISR(woken);
}

void dronelink_transport_tx_done(UART_HandleTypeDef *uart) {
  if ((uart == link_uart) && (link_uart != NULL)) {
    tx_busy = false;
  }
}

void dronelink_transport_uart_error(UART_HandleTypeDef *uart) {
  if ((uart != link_uart) || (link_uart == NULL)) {
    return;
  }
  ++rx_dropped;
  rx_restart_requested = true;
}
