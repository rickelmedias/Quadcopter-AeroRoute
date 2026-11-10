/**
 * @file mtf_parser.h
 * @brief Decodificador puro do protocolo Micolink do MTF-01.
 *
 * Sem HAL: recebe um byte por vez e é testado no PC (tests/test_mtf.c).
 * Aceita somente a mensagem 0x51 (fluxo + distância) com 20 bytes de carga.
 */
#ifndef MTF_PARSER_H
#define MTF_PARSER_H
#include "mtf01.h"

/**
 * @brief Estado do decodificador.
 */
typedef struct {
  /** Frame em montagem: cabeçalho de 6 bytes, até 64 de carga e o checksum. */
  uint8_t data[71];
  /** Quantos bytes do frame atual já foram recebidos. */
  uint8_t used;
  /** Sequência do último frame aceito, para detectar perdas e duplicatas. */
  uint8_t previous_seq;
  /** Tick (ms) do último byte; um intervalo maior que 50 ms descarta o frame
      em montagem. */
  uint32_t last_byte_ms;
  /** Verdadeiro depois do primeiro frame aceito desde o último reset. */
  bool have_sequence;
  /** Último frame decodificado e contadores; é o que a task publica. */
  mtf_sample_t sample;
} mtf_parser_t;

/**
 * @brief Descarta o frame em montagem e o histórico de sequência.
 *
 * Os contadores e o último frame em @c sample são mantidos.
 *
 * @param[in,out] p  Decodificador.
 */
void mtf_parser_reset(mtf_parser_t *p);

/**
 * @brief Entrega um byte recebido ao decodificador.
 *
 * Valida checksum, tipo e tamanho da mensagem. Um relógio do sensor que anda
 * para trás é tratado como reinício do sensor, não como milhares de frames
 * perdidos. Frames duplicados (mesma sequência) são ignorados.
 *
 * @param[in,out] p     Decodificador.
 * @param[in]     byte  Byte recebido.
 * @param[in]     now   Tick atual em ms.
 * @return true quando este byte completou um frame válido, já copiado para
 *         @c p->sample.
 */
bool mtf_parser_byte(mtf_parser_t *p, uint8_t byte, uint32_t now);
#endif
