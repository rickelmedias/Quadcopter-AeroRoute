/**
 * @file bench_frame.h
 * @brief Enquadramento do enlace USB de bancada (bench-link v1): COBS + CRC16.
 *
 * Pacote (antes do COBS): id (1 byte), seq (1 byte), payload (0..::BL_MAX_PAYLOAD)
 * e CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) em little-endian sobre id,
 * seq e payload. No fio, o pacote vai codificado em COBS e terminado por 0x00.
 * Um 0x00 isolado (quadro vazio) é ignorado e serve para ressincronizar.
 *
 * Código puro, sem HAL nem FreeRTOS (tests/test_bench_link.c). A mesma
 * especificação está em hmi/benchlink/protocol.py e docs/phase2/bench-link.md.
 */
#ifndef LINK_BENCH_FRAME_H
#define LINK_BENCH_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Maior payload, em bytes. */
#define BL_MAX_PAYLOAD 96U
/** Maior pacote decodificado: id, seq, payload e CRC. */
#define BL_PACKET_MAX  (BL_MAX_PAYLOAD + 4U)
/** Maior quadro no fio: COBS acrescenta 1 byte a cada 254 e o delimitador. */
#define BL_FRAME_MAX   (BL_PACKET_MAX + (BL_PACKET_MAX / 254U) + 2U)

/** @brief CRC-16/CCITT-FALSE. */
uint16_t bl_crc16(const uint8_t *data, size_t length);

/**
 * @brief Codifica em COBS, sem o delimitador.
 * @return Bytes escritos, ou 0 se @p capacity não bastar.
 */
size_t bl_cobs_encode(const uint8_t *in, size_t length, uint8_t *out, size_t capacity);

/**
 * @brief Decodifica um quadro COBS (sem o delimitador).
 * @return Bytes decodificados, ou 0 se o quadro for inválido ou não couber.
 */
size_t bl_cobs_decode(const uint8_t *in, size_t length, uint8_t *out, size_t capacity);

/**
 * @brief Monta o quadro completo (COBS + 0x00) de uma mensagem.
 * @return Bytes escritos, ou 0 se o payload for grande demais ou não couber.
 */
size_t bl_frame_encode(uint8_t id, uint8_t seq, const uint8_t *payload, size_t length, uint8_t *out,
                       size_t capacity);

/** @brief Resultado de um byte recebido. */
typedef enum {
  BL_RX_NONE = 0, /**< Nada completo ainda. */
  BL_RX_PACKET,   /**< Pacote válido em bl_decoder_t::packet. */
  BL_RX_ERROR     /**< Quadro descartado: COBS, tamanho ou CRC inválidos. */
} bl_rx_event_t;

/** @brief Decodificador incremental de quadros. */
typedef struct {
  uint8_t frame[BL_FRAME_MAX];   /**< Bytes do quadro em montagem. */
  size_t used;                   /**< Bytes em ::frame. */
  bool overflow;                 /**< Quadro atual passou do tamanho máximo. */
  uint8_t packet[BL_PACKET_MAX]; /**< Último pacote válido: id, seq, payload. */
  size_t packet_length;          /**< id + seq + payload (sem CRC). */
  uint32_t packets;              /**< Pacotes válidos. */
  uint32_t errors;               /**< Quadros descartados. */
} bl_decoder_t;

/** @brief Estado inicial (também descarta quadro parcial). */
void bl_decoder_reset(bl_decoder_t *decoder);

/** @brief Consome um byte do fio. */
bl_rx_event_t bl_decoder_push(bl_decoder_t *decoder, uint8_t byte);

#endif
