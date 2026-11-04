/**
 * @file test_regressions.c
 * @brief Regressões dos defeitos encontrados na revisão técnica externa.
 *
 * Todo teste aqui chama a MESMA função que o firmware roda - as proteções
 * compartilhadas ficam em headers (dma_ring.h, bno_i2c_header.h) exatamente
 * por isso. Os casos do antigo estimador de deslocamento saíram junto com
 * ele.
 */
#include "dma_ring.h"
#include "bno_i2c_header.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>

/* ---- 1. Posição de DMA repetida --------------------------------------- */

/* A HAL pode informar a mesma posição do anel duas vezes na borda do buffer
   - um evento de fim de transferência seguido de um de linha ociosa.
   Normalizar a posição de buffer cheio para 0 fazia o segundo parecer
   "0 -> size" e reenviar o buffer inteiro como dado novo. Os dois caminhos de
   USART agora usam dma_ring_segments(); estes testes a chamam diretamente. */

/** Tamanho do anel usado nos testes. */
#define RING 256U

/**
 * @brief Soma o tamanho dos trechos.
 * @param seg    Trechos.
 * @param count  Quantidade de trechos.
 * @return Total de bytes.
 */
static size_t total_length(const dma_ring_segment_t *seg, unsigned count) {
  size_t n = 0;
  for (unsigned i = 0; i < count; ++i) n += seg[i].length;
  return n;
}

/** @brief Uma posição repetida na borda do buffer não pode reenviar nada. */
static void test_repeated_dma_position_forwards_nothing(void) {
  dma_ring_segment_t seg[2];
  uint16_t last = 200;

  /* Primeiro evento: a cauda realmente nova. */
  unsigned count = dma_ring_segments(last, RING, RING, seg);
  assert(count == 1 && seg[0].offset == 200 && seg[0].length == 56);
  last = RING; /* guardada sem alteração, de propósito sem normalizar */

  /* Repetição da mesma posição: nada novo. Esta é a regressão. */
  count = dma_ring_segments(last, RING, RING, seg);
  assert(count == 0 && total_length(seg, count) == 0);

  /* A volta do anel que vem depois ainda entrega o início, sem cauda
     fantasma. */
  count = dma_ring_segments(last, 30, RING, seg);
  assert(count == 1 && seg[0].offset == 0 && seg[0].length == 30);
}

/** @brief Casos comuns do anel e a garantia de nunca apontar fora do buffer. */
static void test_dma_ring_ordinary_cases(void) {
  dma_ring_segment_t seg[2];

  /* Avanço simples. */
  unsigned count = dma_ring_segments(10, 50, RING, seg);
  assert(count == 1 && seg[0].offset == 10 && seg[0].length == 40);

  /* Uma volta real no meio do buffer: cauda e depois início, nessa ordem. */
  count = dma_ring_segments(200, 20, RING, seg);
  assert(count == 2);
  assert(seg[0].offset == 200 && seg[0].length == 56);
  assert(seg[1].offset == 0 && seg[1].length == 20);
  assert(total_length(seg, count) == 76);

  /* Dar a volta exatamente até 0 tem cauda, mas não início. */
  count = dma_ring_segments(200, 0, RING, seg);
  assert(count == 1 && seg[0].offset == 200 && seg[0].length == 56);

  /* Sem movimento, e posições fora da faixa, não entregam nada. */
  assert(dma_ring_segments(42, 42, RING, seg) == 0);
  assert(dma_ring_segments(0, RING + 1, RING, seg) == 0);
  assert(dma_ring_segments(RING + 1, 0, RING, seg) == 0);

  /* Sejam quais forem as entradas, um trecho nunca pode apontar para fora do
     buffer. */
  for (unsigned last = 0; last <= RING; ++last) {
    for (unsigned pos = 0; pos <= RING; ++pos) {
      const unsigned n = dma_ring_segments((uint16_t)last, (uint16_t)pos, RING, seg);
      assert(n <= 2);
      for (unsigned i = 0; i < n; ++i) {
        assert(seg[i].length > 0);
        assert((size_t)seg[i].offset + seg[i].length <= RING);
      }
    }
  }
}

/* ---- 2. Canal SHTP sem limite ----------------------------------------- */

/**
 * @brief O canal SHTP precisa ser limitado antes de ser usado.
 *
 * O parser SHTP incorporado indexa o vetor por canal com um byte vindo
 * direto do barramento e só confere a faixa depois. hub_read() rejeita antes
 * um canal fora da faixa, usando esta verificação compartilhada.
 */
static void test_shtp_channel_is_bounded_before_use(void) {
  uint8_t header[4] = {0x10, 0x00, 0x00, 0x00};
  for (unsigned chan = 0; chan < BNO_SHTP_MAX_CHANS; ++chan) {
    header[2] = (uint8_t)chan;
    assert(bno_i2c_header_channel_valid(header));
  }
  /* Um cabeçalho corrompido pode trazer qualquer byte. */
  for (unsigned chan = BNO_SHTP_MAX_CHANS; chan <= 255U; ++chan) {
    header[2] = (uint8_t)chan;
    assert(!bno_i2c_header_channel_valid(header));
  }

  /* A leitura completa é comparada com o cabeçalho espiado, e essa
     comparação exige o byte de canal idêntico - então validar o cabeçalho
     espiado cobre o pacote que o parser realmente recebe. */
  const uint8_t peeked[4] = {0x0F, 0x00, 0x02, 0x9A};
  uint8_t repeated[4] = {0x0F, 0x80, 0x02, 0x9B};
  assert(bno_i2c_header_matches(peeked, repeated));
  repeated[2] = 0x09; /* um canal que a proteção teria rejeitado */
  assert(!bno_i2c_header_matches(peeked, repeated));
}

/** @brief Roda todos os testes; qualquer falha interrompe pelo assert. */
int main(void) {
  test_repeated_dma_position_forwards_nothing();
  test_dma_ring_ordinary_cases();
  test_shtp_channel_is_bounded_before_use();
  puts("Regressions: duplicated and ordinary DMA positions and SHTP channel\n"
       "bounds - all driving the shared production guards");
}
