/** @file uart_callbacks.c @brief Roteador único dos callbacks UART da plataforma. */
#include "actuators/esc_telem.h"
#include "dronelink/dronelink_transport.h"
#include "mtf01.h"
#include "stm32f4xx_hal.h"

/* USART2 = MTF-01; USART1 = DroneLink (Raspberry Pi) ou, com ESC_TELEM_UART = 1,
   telemetria KISS dos ESCs. Cada driver ignora as outras UARTs. */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t position) {
  mtf_rx_event(uart, position);
  esc_telem_rx_event(uart, position);
  dronelink_transport_rx_event(uart, position);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) {
  dronelink_transport_tx_done(uart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
  mtf_uart_error(uart);
  esc_telem_uart_error(uart);
  dronelink_transport_uart_error(uart);
}
