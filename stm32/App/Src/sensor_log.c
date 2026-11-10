/**
 * @file sensor_log.c
 * @brief Task de log bruto do MTF-01 pela USB (contrato em sensor_log.h).
 *
 * A cada 100 ms publica a última leitura do MTF. Se a USB estiver ocupada ou
 * desconectada, a mensagem é descartada; a aquisição nunca espera pelo PC.
 */
#include "sensor_log.h"
#include "mtf01.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

/** Período do log, em ms. */
#define LOG_PERIOD_MS 100U

/* Os floats são impressos como inteiros escalados de propósito: a newlib-nano
   é ligada sem suporte a %f, então uma conversão de float de verdade não
   imprimiria nada útil. q_x10000=9670 se lê 0,9670; mm e mm/s já são
   unidades inteiras. */

/**
 * @brief Acrescenta um grupo à mensagem, acumulando o tamanho em @c n.
 *
 * @c n pode passar do tamanho do buffer, que é como o snprintf indica
 * truncamento; a proteção mantém a aritmética de ponteiro dentro da faixa, e
 * quem chama confere o tamanho final. Usa as variáveis locais @c line e @c n.
 */
#define APPEND(...)                                                        \
  do {                                                                     \
    if (n >= 0 && n < (int)sizeof(line)) {                                 \
      int r_ = snprintf(line + n, sizeof(line) - (size_t)n, __VA_ARGS__);  \
      n = (r_ < 0) ? -1 : n + r_;                                          \
    }                                                                      \
  } while (0)

#if !CAPTURE_ENABLED
/**
 * @brief Task de log: lê o MTF, monta a mensagem e a envia pela USB
 *        a cada ::LOG_PERIOD_MS.
 *
 * @param[in] argument  Não usado.
 */
static void log_task(void *argument) {
  (void)argument;
  static char line[320];
  static uint8_t pending[320];
  mtf_sample_t m;
  uint32_t dropped_logs = 0;

  for (;;) {
    mtf_get_sample(&m);
    const uint32_t now = HAL_GetTick();
    int n = 0;
    (void)m; (void)now; (void)dropped_logs;

#if LOG_MTF
    /* CSV (não imprimir cabeçalho):
       t_ms;sensor_time_ms;received_ms;distance_mm;
       flow_x_cm_s_at_1m;flow_y_cm_s_at_1m;strength;precision;
       distance_status;quality;flow_status;range_valid;flow_valid;seen;
       frames;crc_errors;gaps;uart_errors;dropped_bytes */
    APPEND("%lu;%lu;%lu;%lu;%d;%d;%u;%u;%u;%u;%u;%u;%u;%u;%lu;%lu;%lu;%lu;%lu\r\n",
      (unsigned long)now,
      (unsigned long)m.time_ms, (unsigned long)m.received_ms,
      (unsigned long)m.distance_mm, (int)m.flow_x, (int)m.flow_y,
      (unsigned)m.strength, (unsigned)m.precision, (unsigned)m.distance_status,
      (unsigned)m.quality, (unsigned)m.flow_status,
      m.range_valid ? 1U : 0U, m.flow_valid ? 1U : 0U, m.seen ? 1U : 0U,
      (unsigned long)m.frames, (unsigned long)m.crc_errors,
      (unsigned long)m.gaps, (unsigned long)m.uart_errors,
      (unsigned long)m.dropped_bytes);
#endif


    uint8_t sent = USBD_OK;
    if (n > 0 && n < (int)sizeof(line)) {
      sent = USBD_BUSY;
      /* Entrega um buffer que continua válido depois desta iteração (versões
         anteriores de CDC_Transmit_FS guardavam o ponteiro de quem chamava) e
         faz a verificação de estado atômica em relação à interrupção USB. */
      taskENTER_CRITICAL();
      USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
      if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc && !cdc->TxState) {
        memcpy(pending, line, (size_t)n);
        sent = CDC_Transmit_FS(pending, (uint16_t)n);
      }
      taskEXIT_CRITICAL();
    }
    if (sent != USBD_OK) ++dropped_logs;
    vTaskDelay(pdMS_TO_TICKS(LOG_PERIOD_MS));
  }
}

/** @brief Cria a task de log (ver sensor_log.h). */
bool sensor_log_start(void) {
  return xTaskCreate(log_task, "SensorLog", 1024, NULL, tskIDLE_PRIORITY+1, NULL) == pdPASS;
}
#else
/** @brief Build Capture: a captura binária é dona da porta USB, então a task
    de log não é criada (economiza 4 KiB do heap do FreeRTOS). */
bool sensor_log_start(void) { return true; }
#endif
