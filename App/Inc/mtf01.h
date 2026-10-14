/**
 * @file mtf01.h
 * @brief Driver do sensor de fluxo óptico + telêmetro MTF-01 na USART2.
 *
 * Os bytes chegam por DMA num buffer circular e são decodificados pelo parser
 * puro de mtf_parser.h (protocolo Micolink).
 */
#ifndef MTF01_H
#define MTF01_H
#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/**
 * @brief Último frame decodificado do MTF-01 e contadores de saúde do enlace.
 */
typedef struct mtf_sample_s {
  /** Relógio do próprio sensor no frame, em ms (não é o tick do STM32). */
  uint32_t time_ms;
  /** Tick do STM32 (ms) em que o frame foi decodificado. */
  uint32_t received_ms;
  /** Mesmo instante em us, na linha de tempo usada pelo SH-2 (resolução 1 ms). */
  uint32_t received_us;
  /** Distância medida pelo telêmetro, no eixo do sensor, em mm. */
  uint32_t distance_mm;
  /** Frames válidos decodificados desde o boot. Também serve de identificador
      do frame (usado pela captura e pelo replay). */
  uint32_t frames;
  /** Frames descartados por checksum inválido. */
  uint32_t crc_errors;
  /** Frames que o sensor numerou mas não chegaram. */
  uint32_t gaps;
  /** Erros reportados pela UART e falhas ao religar a recepção por DMA. */
  uint32_t uart_errors;
  /** Bytes perdidos por falta de espaço no stream buffer de recepção. */
  uint32_t dropped_bytes;
  /** Fluxo no eixo X, em cm/s a 1 m de distância. NÃO é velocidade do veículo. */
  int16_t flow_x;
  /** Fluxo no eixo Y, em cm/s a 1 m de distância. NÃO é velocidade do veículo. */
  int16_t flow_y;
  /** Intensidade do sinal do telêmetro, como o sensor informa. */
  uint8_t strength;
  /** Precisão do telêmetro, como o sensor informa. */
  uint8_t precision;
  /** Estado da medida de distância; 1 significa válida. */
  uint8_t distance_status;
  /** Qualidade da imagem do fluxo, 0..255. */
  uint8_t quality;
  /** Estado da medida de fluxo; 1 significa válida. */
  uint8_t flow_status;
  /** Distância utilizável: estado 1, entre 10 e 8000 mm e dado recente. */
  bool range_valid;
  /** Fluxo utilizável: estado 1, qualidade maior que zero e dado recente. */
  bool flow_valid;
  /** Verdadeiro depois do primeiro frame válido. */
  bool seen;
} mtf_sample_t;

/**
 * @brief Cria o stream buffer de recepção e a task do MTF-01.
 *
 * @return true se os dois foram criados.
 */
bool mtf_start(void);

/**
 * @brief Copia o frame mais recente, de forma segura entre tasks.
 *
 * Os indicadores de validade são zerados aqui quando o frame mais novo tem
 * mais de 250 ms, então um sensor morto nunca aparece como válido.
 *
 * @param[out] out  Destino da cópia.
 */
void mtf_get_sample(mtf_sample_t *out);

/**
 * @brief Repassa bytes novos recebidos por DMA (chamada em interrupção).
 *
 * Chamada pelo callback compartilhado de recepção das USARTs; ignora eventos
 * de outras UARTs.
 *
 * @param[in] uart      UART que gerou o evento.
 * @param[in] position  Posição atual do DMA informada pela HAL.
 */
void mtf_rx_event(UART_HandleTypeDef *uart, uint16_t position);

/**
 * @brief Registra um erro da USART2 e pede o reinício da recepção.
 *
 * Chamada pelo callback compartilhado de erro das UARTs; ignora outras UARTs.
 *
 * @param[in] uart  UART que gerou o erro.
 */
void mtf_uart_error(UART_HandleTypeDef *uart);
#endif
