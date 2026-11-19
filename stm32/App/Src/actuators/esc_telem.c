/**
 * @file esc_telem.c
 * @brief Telemetria KISS na USART1 (contrato em actuators/esc_telem.h).
 */
#include "actuators/esc_telem.h"

#include "dma_ring.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

/** DMA da recepção da USART1 (DMA2 Stream2), iniciado em usart.c. */
extern DMA_HandleTypeDef hdma_usart1_rx;

/** Tamanho do buffer circular do DMA, em bytes. */
#define DMA_SIZE 64U

static uint8_t dma_buffer[DMA_SIZE];
static uint16_t last_position;
/** Estado da agenda; acessado pela task dos motores e pela interrupção. */
static esc_kiss_t kiss;
static volatile uint32_t uart_errors;
static volatile bool restart;
static bool started;

/** @brief (Re)inicia a recepção; sem meia transferência para não cortar quadros. */
static bool start_rx(void) {
  (void)HAL_UART_AbortReceive(&huart1);
  last_position = 0U;
  bool ok = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_buffer, DMA_SIZE) == HAL_OK;
  if (ok) {
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  }
  return ok;
}

bool esc_telem_start(void) {
  esc_kiss_init(&kiss, HAL_GetTick());
  if (ESC_TELEM_UART == 0) {
    return true; /* USART1 do DroneLink */
  }
  started = start_rx();
  if (!started) {
    ++uart_errors;
    restart = true;
  }
  return true; /* sem fio ligado o resto do firmware continua útil */
}

void esc_telem_configure(bool enabled, uint16_t period_ms) {
  taskENTER_CRITICAL();
  /* Sem a UART, pedir telemetria só geraria timeouts. */
  esc_kiss_configure(&kiss, enabled && (ESC_TELEM_UART != 0), period_ms, HAL_GetTick());
  taskEXIT_CRITICAL();
}

uint8_t esc_telem_frame_mask(bool command_frame, uint32_t now_ms) {
  taskENTER_CRITICAL();
  const uint8_t mask = esc_kiss_request(&kiss, command_frame, now_ms);
  taskEXIT_CRITICAL();
  return mask;
}

void esc_telem_take(esc_kiss_t *out) {
  taskENTER_CRITICAL();
  *out = kiss;
  kiss.fresh = 0U;
  taskEXIT_CRITICAL();
}

bool esc_telem_enabled(void) {
  return kiss.enabled;
}

uint32_t esc_telem_uart_errors(void) {
  return uart_errors;
}

void esc_telem_service(void) {
  if ((ESC_TELEM_UART != 0) && restart) {
    restart = false;
    if (!start_rx()) {
      ++uart_errors;
      restart = true;
    }
  }
}

void esc_telem_rx_event(UART_HandleTypeDef *uart, uint16_t position) {
  if ((ESC_TELEM_UART != 0) && (uart == &huart1) && (position <= DMA_SIZE)) {
    dma_ring_segment_t seg[2];
    const unsigned count = dma_ring_segments(last_position, position, DMA_SIZE, seg);
    const uint32_t now = HAL_GetTick();
    const UBaseType_t state = taskENTER_CRITICAL_FROM_ISR();
    for (unsigned i = 0U; i < count; ++i) {
      esc_kiss_rx_bytes(&kiss, &dma_buffer[seg[i].offset], seg[i].length);
    }
    if (HAL_UARTEx_GetRxEventType(uart) == HAL_UART_RXEVENT_IDLE) {
      esc_kiss_rx_idle(&kiss, now);
    }
    taskEXIT_CRITICAL_FROM_ISR(state);
    last_position = position; /* nunca normalizar: ver dma_ring.h */
  }
}

void esc_telem_uart_error(UART_HandleTypeDef *uart) {
  if ((ESC_TELEM_UART != 0) && (uart == &huart1)) {
    ++uart_errors;
    restart = true;
  }
}
