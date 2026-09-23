/**
 * @file mtf01.c
 * @brief Driver do MTF-01 na USART2 (contratos em mtf01.h).
 *
 * A interrupção de recepção copia os bytes novos do DMA para um stream
 * buffer; a task decodifica, grava na captura e publica a última amostra.
 */
#include "mtf01.h"
#include "dma_ring.h"
#include "mtf_parser.h"
#include "capture.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "usart.h"
/** Tamanho do buffer DMA circular, em bytes (~9 frames Micolink). */
#define DMA_SIZE 256U
/** Buffer circular preenchido pelo DMA. */
static uint8_t dma_buffer[DMA_SIZE];
/** Posição do DMA no evento anterior (nunca normalizada; ver dma_ring.h). */
static uint16_t last_position;
/** Bytes recebidos à espera da task. */
static StreamBufferHandle_t stream;
/** Pedido para a task reiniciar a recepção (erro de UART ou perda de bytes). */
static volatile bool restart;
/** Erros da UART e falhas ao religar a recepção. */
static volatile uint32_t errors;
/** Bytes que não couberam no stream buffer. */
static volatile uint32_t dropped;
/** Estado do decodificador; usado só pela task. */
static mtf_parser_t parser;
/** Última amostra publicada; protegida por seção crítica. */
static mtf_sample_t published;

/** @brief Repassa bytes novos do DMA para a task (ver mtf01.h). */
void mtf_rx_event(UART_HandleTypeDef *uart, uint16_t position) {
  if (uart != &huart2 || !stream || position > DMA_SIZE || restart) return;
  /* O cálculo dos trechos fica em dma_ring.h para que os dois caminhos de
     recepção das USARTs usem uma única implementação testada - antes não
     usavam, e um deles tinha um defeito de dados duplicados que o outro já
     tinha corrigido. */
  dma_ring_segment_t seg[2];
  const unsigned count = dma_ring_segments(last_position, position, DMA_SIZE, seg);
  BaseType_t wake = pdFALSE;
  for (unsigned i = 0; i < count; ++i) {
    const size_t n = seg[i].length;
    dropped += n - xStreamBufferSendFromISR(stream, dma_buffer + seg[i].offset, n, &wake);
  }
  last_position = position;
  portYIELD_FROM_ISR(wake);
}

/** @brief Conta um erro da USART2 e pede o reinício da recepção (ver mtf01.h). */
void mtf_uart_error(UART_HandleTypeDef *uart) {
  if (uart == &huart2) { ++errors; restart = true; }
}

/** @brief Cópia segura da última amostra, com validade por idade (ver mtf01.h). */
void mtf_get_sample(mtf_sample_t *out) {
  taskENTER_CRITICAL(); *out = published; taskEXIT_CRITICAL();
  if ((uint32_t)(HAL_GetTick()-out->received_ms) > 250U) {
    out->range_valid = out->flow_valid = false;
  }
}

/**
 * @brief Task do MTF-01: (re)inicia a recepção, decodifica os bytes, grava
 *        cada frame na captura e publica a última amostra.
 *
 * @param[in] argument  Não usado.
 */
static void mtf_task(void *argument) {
  (void)argument;
  uint8_t bytes[128];
  uint32_t previous_dropped = 0;
  uint32_t last_diag_ms = 0;
  restart = true;
  for (;;) {
    capture_diag_poll(&last_diag_ms, CAP_CLOCK_RESYNCS_UNKNOWN);
    if (restart) {
      HAL_UART_AbortReceive(&huart2);
      xStreamBufferReset(stream);
      last_position = 0;
      mtf_parser_reset(&parser);
      parser.sample.range_valid = parser.sample.flow_valid = false;
      restart = false;
      if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_buffer, DMA_SIZE) != HAL_OK) {
        ++errors; restart = true; vTaskDelay(pdMS_TO_TICKS(100));
      }
    }
    size_t n = xStreamBufferReceive(stream, bytes, sizeof(bytes), pdMS_TO_TICKS(5));
    if (previous_dropped != dropped) {
      previous_dropped = dropped;
      /* Descarta o fluxo descontínuo antes de aceitar outro frame. */
      restart = true; n = 0;
    }
    for (size_t i=0; i<n; ++i) {
      /* Cada frame aceito é gravado aqui, e não a partir de uma fotografia:
         este é o único lugar que vê todos os frames, com o relógio do sensor
         ainda separado do nosso tick de recepção. */
      if (mtf_parser_byte(&parser, bytes[i], HAL_GetTick())) {
        parser.sample.received_us = HAL_GetTick()*1000U;
        capture_mtf(parser.sample.flow_x, parser.sample.flow_y,
                    parser.sample.distance_mm, parser.sample.quality,
                    parser.sample.distance_status, parser.sample.strength,
                    parser.sample.precision, parser.sample.range_valid,
                    parser.sample.flow_valid, parser.sample.time_ms,
                    parser.sample.frames);
      }
    }
    parser.sample.uart_errors = errors; parser.sample.dropped_bytes = dropped;
    taskENTER_CRITICAL(); published = parser.sample; taskEXIT_CRITICAL();
  }
}

/** @brief Cria o stream buffer e a task (ver mtf01.h). */
bool mtf_start(void) {
  stream = xStreamBufferCreate(1024, 1);
  return stream && xTaskCreate(mtf_task, "MTF01", 512, NULL, tskIDLE_PRIORITY+3, NULL) == pdPASS;
}
