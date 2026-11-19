/**
 * @file esc_telem.h
 * @brief Recepção da telemetria KISS dos ESCs na USART1 (PA10, 115200 8N1).
 *
 * Ligação: TX do KM55A II -> PA10 (USART1_RX). A USART1 é a única UART livre e
 * agora é do DroneLink (Raspberry Pi): com ::ESC_TELEM_UART em 0 (padrão) este
 * módulo não toca na UART e os pedidos ficam desligados. A telemetria do ESC
 * deve voltar pelo DShot bidirecional (eRPM, tensão e corrente sem fio extra).
 * Com 1, a recepção usa o DMA circular da USART1 com evento de linha ociosa
 * para delimitar quadros; a lógica fica em actuators/esc_kiss.h.
 *
 * Os pedidos começam desligados: liga-se pela IHM (BL_CMD_ESC_TELEM). Sem o
 * fio ligado, os pedidos só geram timeouts contados em SYS.
 */
#ifndef ACTUATORS_ESC_TELEM_H
#define ACTUATORS_ESC_TELEM_H

#include "actuators/esc_kiss.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef ESC_TELEM_UART
/** 1 = USART1 para a telemetria KISS (sem DroneLink); 0 = USART1 do DroneLink. */
#define ESC_TELEM_UART 0
#endif

/** @brief Inicia a recepção. Chamar depois de MX_USART1_UART_Init(). */
bool esc_telem_start(void);

/** @brief Liga/desliga os pedidos e ajusta o período entre eles. */
void esc_telem_configure(bool enabled, uint16_t period_ms);

/**
 * @brief Máscara de telemetria do próximo quadro DShot.
 *
 * Chamada pela task dos motores (pode estar em seção crítica).
 * @param[in] command_frame  O árbitro já pediu telemetria nos quatro (comando especial).
 */
uint8_t esc_telem_frame_mask(bool command_frame, uint32_t now_ms);

/** @brief Cópia do estado (contadores e últimos quadros). Limpa os bits fresh. */
void esc_telem_take(esc_kiss_t *out);

/** @brief Pedidos de telemetria ligados. */
bool esc_telem_enabled(void);

/** @brief Erros de UART desde o boot. */
uint32_t esc_telem_uart_errors(void);

/** @brief Religa a recepção depois de erro (chamar periodicamente de uma task). */
void esc_telem_service(void);

/** @brief Evento de recepção da HAL; ignora outras UARTs. Interrupção. */
void esc_telem_rx_event(UART_HandleTypeDef *uart, uint16_t position);

/** @brief Erro da HAL; ignora outras UARTs. Interrupção. */
void esc_telem_uart_error(UART_HandleTypeDef *uart);

#endif
