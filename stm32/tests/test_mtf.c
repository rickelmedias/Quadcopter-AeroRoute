/**
 * @file test_mtf.c
 * @brief Testes de host do decodificador Micolink do MTF-01.
 */
#include "mtf_parser.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
/**
 * @brief Entrega @p n bytes ao decodificador.
 * @param p  Decodificador.
 * @param b  Bytes.
 * @param n  Quantidade.
 * @param t  Tick usado para todos os bytes.
 * @return Quantos frames válidos foram completados.
 */
static unsigned feed(mtf_parser_t *p, const uint8_t *b, unsigned n, uint32_t t) {
  unsigned accepted = 0;
  for (unsigned i = 0; i < n; ++i)
    accepted += mtf_parser_byte(p, b[i], t);
  return accepted;
}
/**
 * @brief Recalcula o checksum (byte 26) de um frame de 27 bytes.
 * @param b  Frame.
 */
static void checksum(uint8_t *b) {
  b[26] = 0;
  for (unsigned i = 0; i < 26; ++i)
    b[26] += b[i];
}
/**
 * @brief Roda os casos: fragmentação, valores com sinal, checksum, volta da
 *        sequência, duplicata, reinício, timeout, tamanho inválido e ruído.
 */
int main(void) {
  mtf_parser_t p = {0};
  uint8_t b[27] = {0xEF, 0x01, 0, 0x51, 254,  20,   100,  0, 0,   0, 0xE8, 3, 0, 0,
                   90,   2,    1, 0,    0x85, 0xFF, 0xC8, 0, 100, 1, 0,    0, 0};
  checksum(b);
  assert(feed(&p, b, 5, 10) == 0);
  assert(feed(&p, b + 5, 22, 11) == 1);
  assert(p.sample.distance_mm == 1000 && p.sample.flow_x == -123 && p.sample.flow_y == 200);
  assert(p.sample.range_valid && p.sample.flow_valid);
  assert(feed(&p, b, 27, 12) == 0); /* duplicata */
  b[4] = 1;
  b[6] = 110;
  checksum(b);
  assert(feed(&p, b, 27, 20) == 1 && p.sample.gaps == 2); /* volta da sequência */
  b[26] ^= 1;
  assert(feed(&p, b, 27, 30) == 0 && p.sample.crc_errors == 1);
  b[4] = 2;
  b[6] = 120;
  b[16] = 0;
  b[23] = 0;
  checksum(b);
  assert(feed(&p, b, 27, 40) == 1 && !p.sample.range_valid && !p.sample.flow_valid);
  b[4] = 0;
  b[6] = 1;
  checksum(b); /* reinício do sensor */
  assert(feed(&p, b, 27, 50) == 1 && p.sample.gaps == 2);
  assert(feed(&p, b, 10, 60) == 0);
  b[4] = 1;
  b[6] = 2;
  checksum(b);
  assert(feed(&p, b, 27, 200) == 1); /* timeout de frame parcial */
  uint8_t badlen[] = {0xEF, 0, 0, 0x51, 0, 255};
  assert(feed(&p, badlen, sizeof(badlen), 201) == 0);
  b[4] = 2;
  checksum(b);
  assert(feed(&p, b, 27, 202) == 1);
  /* Carga truncada/desconhecida e ruído determinístico nunca podem estourar
     o buffer. */
  uint32_t rng = 1;
  for (unsigned i = 0; i < 1000000; ++i) {
    rng = rng * 1664525U + 1013904223U;
    mtf_parser_byte(&p, (uint8_t)(rng >> 24), 203 + i / 100);
    assert(p.used < sizeof(p.data));
  }
  mtf_parser_reset(&p);
  b[4] = 3;
  checksum(b);
  assert(feed(&p, b, 27, 30000) == 1);
  puts("MTF parser: fragmentation, signed values, checksum, sequence wrap, duplicate, reboot, "
       "timeout, invalid length and 1M noise bytes passed");
}
