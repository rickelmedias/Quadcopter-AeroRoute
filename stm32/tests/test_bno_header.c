/**
 * @file test_bno_header.c
 * @brief Testes de host da regra de continuação do cabeçalho SHTP.
 */
#include "bno_i2c_header.h"
#include <assert.h>
#include <stdio.h>
/**
 * @brief Roda os casos: falha real de bancada, cabeçalho repetido, volta da
 *        sequência e cabeçalhos inválidos.
 */
int main(void) {
  const uint8_t first[4]={0x0F,0,0,0x9A};
  uint8_t next[4]={0x0F,0x80,0,0x9B};
  assert(bno_i2c_header_matches(first,next)); /* a falha real vista na bancada */
  assert(bno_i2c_header_matches(first,first));
  next[3]=0x9C; assert(!bno_i2c_header_matches(first,next));
  next[3]=0x9B; next[1]=0; assert(!bno_i2c_header_matches(first,next));
  next[1]=0x80; next[2]=1; assert(!bno_i2c_header_matches(first,next));
  next[2]=0; next[0]=0x10; assert(!bno_i2c_header_matches(first,next));
  const uint8_t wrap[4]={15,0,3,255}, continuation[4]={15,128,3,0};
  assert(bno_i2c_header_matches(wrap,continuation));
  puts("BNO header: bench regression, repeated header, sequence wrap and invalid headers passed");
}
