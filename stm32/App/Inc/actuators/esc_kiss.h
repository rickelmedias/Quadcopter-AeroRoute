/**
 * @file esc_kiss.h
 * @brief Telemetria serial KISS do AM32: decodificação e agenda de pedidos.
 *
 * Quadro de 10 bytes a 115200 baud (AM32 Src/kiss_telemetry.c): temperatura
 * (int8, °C), tensão (cV), corrente (cA), consumo (mAh), eRPM/100, todos
 * big-endian, e CRC8 (polinômio 0x07, semente 0) sobre os 9 primeiros bytes.
 * O ESC responde uma vez a cada quadro DShot com o bit de telemetria.
 *
 * Os quatro ESCs da placa 4 em 1 dividem a linha TX, então só um motor é
 * consultado por vez; a resposta é atribuída ao motor pendente. Quadros com
 * comando especial (bit de telemetria nos quatro) abrem uma janela de silêncio
 * em que o que chegar é contado como não solicitado e descartado.
 *
 * Código puro (tests/test_esc_kiss.c). O tempo entra como parâmetro.
 */
#ifndef ACTUATORS_ESC_KISS_H
#define ACTUATORS_ESC_KISS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Bytes de um quadro KISS. */
#define ESC_KISS_FRAME_LEN      10U
/** Espera máxima pela resposta, em ms. */
#define ESC_KISS_TIMEOUT_MS     8U
/** Janela de silêncio depois de um comando especial, em ms. */
#define ESC_KISS_QUIET_MS       20U
/** Período padrão entre pedidos (um motor por vez), em ms. */
#define ESC_KISS_DEFAULT_PERIOD 10U

/** @brief Conteúdo de um quadro KISS, nas unidades do protocolo. */
typedef struct {
  int8_t temperature_c;
  uint16_t voltage_cv;
  uint16_t current_ca;
  uint16_t consumption_mah;
  uint16_t erpm_100;
} esc_kiss_frame_t;

/** @brief Estado da agenda e dos contadores. */
typedef struct {
  bool enabled;
  uint16_t period_ms;
  uint32_t next_ms;
  uint8_t next_motor;
  int8_t pending; /**< Motor aguardando resposta, ou -1. */
  uint32_t pending_ms;
  uint32_t quiet_until_ms;
  bool quiet;
  uint8_t rx[16];
  size_t rx_length;
  bool rx_overflow;
  esc_kiss_frame_t frame[4];
  uint32_t frame_ms[4];
  uint8_t fresh; /**< Bit i: quadro novo do motor i ainda não consumido. */
  uint32_t frames_ok[4];
  uint32_t timeouts[4];
  uint32_t crc_errors;
  uint32_t length_errors;
  uint32_t unsolicited;
} esc_kiss_t;

/** @brief CRC8 do AM32 (polinômio 0x07, semente 0, MSB primeiro). */
uint8_t esc_kiss_crc8(const uint8_t *data, size_t length);

/** @brief Decodifica um quadro de exatamente ::ESC_KISS_FRAME_LEN bytes com CRC válido. */
bool esc_kiss_decode(const uint8_t *data, size_t length, esc_kiss_frame_t *out);

/** @brief Estado inicial: desligado, período padrão. */
void esc_kiss_init(esc_kiss_t *k, uint32_t now_ms);

/** @brief Liga/desliga os pedidos; cancela o pedido pendente. */
void esc_kiss_configure(esc_kiss_t *k, bool enabled, uint16_t period_ms, uint32_t now_ms);

/**
 * @brief Chamada a cada quadro DShot: decide o bit de telemetria.
 * @param[in] command_frame  O quadro já é um comando especial com telemetria nos quatro.
 * @return Máscara de telemetria a enviar (0 ou um único bit), ou 0x0F em comando.
 */
uint8_t esc_kiss_request(esc_kiss_t *k, bool command_frame, uint32_t now_ms);

/** @brief Acumula bytes recebidos (em interrupção). */
void esc_kiss_rx_bytes(esc_kiss_t *k, const uint8_t *data, size_t length);

/** @brief Linha ociosa: fecha o quadro acumulado e o atribui ao motor pendente. */
void esc_kiss_rx_idle(esc_kiss_t *k, uint32_t now_ms);

#endif
