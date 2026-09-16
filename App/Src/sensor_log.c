/**
 * @file sensor_log.c
 * @brief Task de log de texto pela USB (contrato e chaves em sensor_log.h).
 *
 * A cada 500 ms monta uma mensagem com os grupos ligados e a envia pela porta
 * CDC. Se a USB estiver ocupada ou desconectada, a mensagem é descartada e
 * contada; a aquisição nunca espera pelo PC.
 */
#include "sensor_log.h"
#include "sensors.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dronelink/dronelink_transport.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/** Período do log, em ms. */
#define LOG_PERIOD_MS 500U
/** Fator de radianos para milésimos de grau. */
#define RAD_TO_MDEG   57295.7795F

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
 * @brief Task de log: lê as três fontes, monta a mensagem e a envia pela USB
 *        a cada ::LOG_PERIOD_MS.
 *
 * @param[in] argument  Não usado.
 */
static void log_task(void *argument) {
  (void)argument;
  /* Dimensionado para todos os grupos ligados ao mesmo tempo com todos os
     contadores no máximo, o que hoje mede 1656 bytes. Num estouro, a
     verificação de tamanho abaixo descarta a mensagem inteira, então um buffer
     pequeno demais faz o log sumir em vez de sair truncado - mantenha folga
     real ao acrescentar campos. */
  static char line[2048];
  static uint8_t pending[2048];
  bno_sample_t b;
  mtf_sample_t m;
  displacement_report_t d;
  uint32_t dropped_logs = 0;

  for (;;) {
    bno_get_sample(&b);
    mtf_get_sample(&m);
    displacement_get(&d);
    const uint32_t now = HAL_GetTick();
    int n = 0;
    (void)b; (void)m; (void)d; (void)now; (void)dropped_logs;

#if LOG_BNO_STATUS
    APPEND("t=%lu BNO online=%u fault=%u addr=0x%02X part=%lu fw=%u.%u.%u build=%lu "
           "probe_ms=%u err=%lu resets=%lu\r\n",
      (unsigned long)now, b.online, (unsigned)b.fault, b.address,
      (unsigned long)b.part, b.version_major, b.version_minor, b.version_patch,
      (unsigned long)b.build, b.probe_ms, (unsigned long)b.errors, (unsigned long)b.resets);
#endif

#if LOG_BNO_DATA
    APPEND("BNO q_x10000=%ld,%ld,%ld,%ld gyro_mrad_s=%ld,%ld,%ld lin_accel_mm_s2=%ld,%ld,%ld "
           "acc=%u,%u,%u n=%lu,%lu,%lu gaps=%lu,%lu,%lu age_ms=%lu,%lu,%lu\r\n",
      (long)(b.q[0]*10000), (long)(b.q[1]*10000), (long)(b.q[2]*10000), (long)(b.q[3]*10000),
      (long)(b.gyro[0]*1000), (long)(b.gyro[1]*1000), (long)(b.gyro[2]*1000),
      (long)(b.lin_accel[0]*1000), (long)(b.lin_accel[1]*1000), (long)(b.lin_accel[2]*1000),
      b.rotation.accuracy, b.angular.accuracy, b.acceleration.accuracy,
      (unsigned long)b.rotation.count, (unsigned long)b.angular.count,
      (unsigned long)b.acceleration.count,
      (unsigned long)b.rotation.gaps, (unsigned long)b.angular.gaps,
      (unsigned long)b.acceleration.gaps,
      (unsigned long)(now-b.rotation.last_ms), (unsigned long)(now-b.angular.last_ms),
      (unsigned long)(now-b.acceleration.last_ms));
#endif

#if LOG_MTF
    APPEND("MTF seen=%u age_ms=%lu range_ok=%u mm=%lu status=%u strength=%u precision=%u "
           "flow_ok=%u flow_cm_s_at_1m=%d,%d quality=%u n=%lu crc_err=%lu gaps=%lu "
           "uart_err=%lu drop=%lu\r\n",
      m.seen, (unsigned long)(now-m.received_ms), m.range_valid, (unsigned long)m.distance_mm,
      m.distance_status, m.strength, m.precision,
      m.flow_valid, m.flow_x, m.flow_y, m.quality, (unsigned long)m.frames,
      (unsigned long)m.crc_errors, (unsigned long)m.gaps, (unsigned long)m.uart_errors,
      (unsigned long)m.dropped_bytes);
#endif

#if LOG_ATTITUDE
    APPEND("ATT euler_mdeg=%ld,%ld,%ld q_x10000=%ld,%ld,%ld,%ld\r\n",
      (long)(d.attitude.roll*RAD_TO_MDEG), (long)(d.attitude.pitch*RAD_TO_MDEG),
      (long)(d.attitude.yaw*RAD_TO_MDEG),
      (long)(d.q[0]*10000), (long)(d.q[1]*10000), (long)(d.q[2]*10000), (long)(d.q[3]*10000));
#endif

#if LOG_HEIGHT
    /* v_lsb é quanto uma contagem de fluxo vale nesta distância: o piso do que
       o par consegue resolver. Nada mais lento é mensurável, e ele cresce com
       a altura, então é o primeiro número a olhar quando os resultados
       parecem ruidosos. */
    APPEND("HEIGHT range_mm=%ld height_mm=%ld tilt_mdeg=%ld,%ld quality=%u v_lsb_mm_s=%ld\r\n",
      (long)(d.range_m*1000), (long)(d.height_m*1000),
      (long)(d.attitude.roll*RAD_TO_MDEG), (long)(d.attitude.pitch*RAD_TO_MDEG), d.quality,
      (long)(d.v_lsb_m_s*1000));
#endif

#if LOG_POSITION
    APPEND("POS raw_mm=%ld,%ld fused_mm=%ld,%ld ekf_mm=%ld,%ld n=%lu,%lu rej=%lu valid=%u health=%u age_ms=%lu corr_age_ms=%lu resync=%lu\r\n",
      (long)(d.raw.x*1000), (long)(d.raw.y*1000),
      (long)(d.fused.x*1000), (long)(d.fused.y*1000),
      (long)(d.ekf.axis[0].p*1000), (long)(d.ekf.axis[1].p*1000),
      (unsigned long)d.raw.samples, (unsigned long)d.fused.samples,
      (unsigned long)d.fused.rejected,
      d.valid, (unsigned)d.health, (unsigned long)d.age_ms,
      (unsigned long)d.correction_age_ms, (unsigned long)d.clock_resyncs);
#endif

#if LOG_POSITION_D
    APPEND("POS_D raw_d_um=%ld,%ld fused_d_um=%ld,%ld raw_v_mm_s=%ld,%ld "
           "fused_v_mm_s=%ld,%ld ekf_v_mm_s=%ld,%ld dt_ms=%ld scale_x1000=%ld,%ld gyro_sign=%ld,%ld\r\n"
           "EKF conv=%u upd=%lu rej=%lu resets=%lu ratio_x100=%ld,%ld sigma_v_mm_s=%ld,%ld\r\n",
      (long)(d.raw.last_dx*1000000), (long)(d.raw.last_dy*1000000),
      (long)(d.fused.last_dx*1000000), (long)(d.fused.last_dy*1000000),
      (long)(d.raw.vx*1000), (long)(d.raw.vy*1000),
      (long)(d.fused.vx*1000), (long)(d.fused.vy*1000),
      (long)(d.ekf.axis[0].v*1000), (long)(d.ekf.axis[1].v*1000),
      (long)(d.dt_s*1000),
      (long)(displacement_get_flow_scale_x()*1000), (long)(displacement_get_flow_scale_y()*1000),
      (long)displacement_get_gyro_sign_x(), (long)displacement_get_gyro_sign_y(),
      d.ekf.converged, (unsigned long)d.ekf.updates, (unsigned long)d.ekf.rejected,
      (unsigned long)d.ekf.resets,
      (long)(d.ekf.test_ratio[0]*100), (long)(d.ekf.test_ratio[1]*100),
      (long)(sqrtf(d.ekf.axis[0].P11)*1000), (long)(sqrtf(d.ekf.axis[1].P11)*1000));
#endif

#if LOG_CALIBRATION
    /* Balance em rolagem/arfagem, evite yaw, não translade. scalar_x1000 é o
       que displacement_set_flow_scale() aplicaria agora; ele se firma
       conforme as amostras se acumulam e não significa nada antes de
       cal_running=1. */
    APPEND("CAL running=%u samples=%lu,%lu valid=%u,%u scalar_x1000=%ld,%ld "
           "residual_mrad_s=%ld,%ld raw_scalar_x1000=%ld,%ld\r\n",
      d.cal_running, (unsigned long)d.cal_samples[0], (unsigned long)d.cal_samples[1],
      d.cal_result_valid[0], d.cal_result_valid[1],
      d.cal_result_valid[0] ? (long)(d.cal_scalar[0]*1000) : 0L,
      d.cal_result_valid[1] ? (long)(d.cal_scalar[1]*1000) : 0L,
      d.cal_result_valid[0] ? (long)(d.cal_residual[0]*1000) : 0L,
      d.cal_result_valid[1] ? (long)(d.cal_residual[1]*1000) : 0L,
      /* Diferente de scalar_x1000, este ignora o limite de faixa - um número
         negativo aqui indica gyro_sign invertido, e não sensor fora de
         especificação. Só para diagnóstico; nunca aplique este valor. */
      d.cal_raw_valid[0] ? (long)(d.cal_scalar_raw[0]*1000) : 0L,
      d.cal_raw_valid[1] ? (long)(d.cal_scalar_raw[1]*1000) : 0L);
#endif

#if LOG_SYS
    APPEND("SYS heap=%lu link_rx_drop=%lu link_tx_drop=%lu log_drop=%lu\r\n",
      (unsigned long)xPortGetFreeHeapSize(), (unsigned long)dronelink_transport_rx_dropped(),
      (unsigned long)dronelink_transport_tx_dropped(), (unsigned long)dropped_logs);
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
