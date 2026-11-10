/**
 * @file mtf_parser.c
 * @brief Decodificador do protocolo Micolink (contratos em mtf_parser.h).
 */
#include "mtf_parser.h"
#include <string.h>

/**
 * @brief Lê um u32 little-endian.
 * @param[in] p  Primeiro byte.
 * @return O valor.
 */
static uint32_t le32(const uint8_t *p) {
  return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}

/**
 * @brief Lê um i16 little-endian (complemento de dois), sem depender de
 *        conversão definida pela implementação.
 * @param[in] p  Primeiro byte.
 * @return O valor com sinal.
 */
static int16_t sle16(const uint8_t *p) {
  uint16_t u = (uint16_t)p[0] | (uint16_t)p[1]<<8;
  return (int16_t)((int32_t)u - ((u & 0x8000U) ? 65536 : 0));
}

/** @brief Descarta o frame em montagem e o histórico de sequência (ver mtf_parser.h). */
void mtf_parser_reset(mtf_parser_t *p) {
  p->used = 0; p->have_sequence = false;
}

/**
 * @brief Entrega um byte ao decodificador (ver mtf_parser.h).
 *
 * Layout do frame: 0xEF, 4 bytes de cabeçalho (o byte 3 é a mensagem e o 4 é
 * a sequência), o byte 5 é o tamanho da carga, depois a carga e por fim o
 * checksum, que é a soma de todos os bytes anteriores.
 */
bool mtf_parser_byte(mtf_parser_t *p, uint8_t byte, uint32_t now) {
  if ((uint32_t)(now-p->last_byte_ms) > 50U) p->used = 0;
  p->last_byte_ms = now;
  if (!p->used && byte != 0xEF) return false;
  p->data[p->used++] = byte;
  if (p->used < 6) return false;
  if (p->data[5] > 64) { p->used = 0; return false; }
  unsigned total = 7U + p->data[5];
  if (p->used < total) return false;
  uint8_t sum = 0;
  for (unsigned i=0; i<total-1; ++i) sum += p->data[i];
  p->used = 0;
  if (sum != byte) {
    ++p->sample.crc_errors;
    /* Aproveita um possível cabeçalho seguinte depois da corrupção. */
    for (unsigned i=1; i<total; ++i) if (p->data[i] == 0xEF) {
      memmove(p->data, p->data+i, total-i); p->used = (uint8_t)(total-i); break;
    }
    return false;
  }
  if (p->data[3] != 0x51 || p->data[5] != 20) return false;
  uint8_t seq = p->data[4];
  uint32_t sensor_time = le32(p->data+6);
  /* Um relógio do sensor que anda para trás é reinício, não milhares de
     frames perdidos. */
  if (p->have_sequence && (int32_t)(sensor_time-p->sample.time_ms) >= 0) {
    uint8_t step = (uint8_t)(seq-p->previous_seq);
    if (!step) return false;
    if (step < 128U) p->sample.gaps += step-1U;
  }
  p->have_sequence = true; p->previous_seq = seq;
  const uint8_t *v = p->data+6;
  p->sample.time_ms = sensor_time;
  p->sample.received_ms = now;
  p->sample.distance_mm = le32(v+4);
  p->sample.strength = v[8]; p->sample.precision = v[9];
  p->sample.distance_status = v[10];
  p->sample.flow_x = sle16(v+12); p->sample.flow_y = sle16(v+14);
  p->sample.quality = v[16]; p->sample.flow_status = v[17];
  p->sample.range_valid = v[10] == 1 && p->sample.distance_mm >= 10 && p->sample.distance_mm <= 8000;
  p->sample.flow_valid = v[17] == 1 && v[16] > 0;
  p->sample.seen = true; ++p->sample.frames;
  return true;
}
