/**
 * @file bno_i2c_header.h
 * @brief Regras do cabeçalho SHTP no transporte I2C do BNO08x.
 *
 * Só header, sem HAL, para ser testado no PC (tests/test_bno_header.c e
 * tests/test_regressions.c).
 */
#ifndef BNO_I2C_HEADER_H
#define BNO_I2C_HEADER_H
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Confere se o cabeçalho repetido numa leitura completa combina com o
 *        cabeçalho lido antes.
 *
 * Uma leitura I2C só do cabeçalho não consome carga útil. A transferência
 * seguinte traz o mesmo tamanho e canal, com o bit de continuação ligado e a
 * sequência avançada em 1. Também são aceitos dispositivos que repetem o
 * cabeçalho sem mudança. Referência: SHTP v1.10, seção 2.3.1.
 *
 * @param[in] first  Cabeçalho de 4 bytes da leitura só de cabeçalho.
 * @param[in] next   Cabeçalho de 4 bytes da leitura completa.
 * @return true se os dois cabeçalhos são consistentes.
 */
static inline bool bno_i2c_header_matches(const uint8_t first[4], const uint8_t next[4]) {
  if (first[0] != next[0] || (first[1] & 0x7FU) != (next[1] & 0x7FU) || first[2] != next[2])
    return false;
  if (first[1] == next[1] && first[3] == next[3])
    return true;
  return (next[1] & 0x80U) && next[3] == (uint8_t)(first[3] + 1U);
}

/**
 * @brief Número máximo de canais SHTP do parser incorporado.
 *
 * Espelha SHTP_MAX_CHANS, que é privado ao .c da biblioteca da CEVA. Se aquele
 * valor mudar, este precisa acompanhar.
 */
#define BNO_SHTP_MAX_CHANS 8U

/**
 * @brief Confere se o canal de um cabeçalho SHTP cabe no vetor por canal do
 *        parser incorporado.
 *
 * O parser da CEVA indexa o vetor em shtp.c:201 e só confere a faixa em
 * shtp.c:216, então um cabeçalho corrompido com canal além do fim seria lido
 * fora dos limites antes de ser rejeitado. A validação é feita aqui, na
 * fronteira do transporte, o que mantém App/Src/sh2/ sem modificação e fecha
 * a exposição: todo byte que o parser vê passa por essa fronteira.
 *
 * @param[in] header  Cabeçalho SHTP de 4 bytes.
 * @return true se o canal é válido.
 */
static inline bool bno_i2c_header_channel_valid(const uint8_t header[4]) {
  return header[2] < BNO_SHTP_MAX_CHANS;
}
#endif
