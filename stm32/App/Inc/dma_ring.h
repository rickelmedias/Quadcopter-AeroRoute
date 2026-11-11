/**
 * @file dma_ring.h
 * @brief Cálculo dos bytes novos num buffer DMA circular.
 *
 * Função compartilhada, só em header, porque os dois caminhos de recepção das
 * USARTs (MTF-01 e DroneLink) precisam da mesma lógica, e um deles divergiu em
 * silêncio: uma versão antiga normalizava a posição @c size para 0 antes que o
 * callback seguinte pudesse compará-la. A HAL pode informar a mesma posição
 * duas vezes na borda do buffer (fim de transferência do DMA seguido de linha
 * ociosa, ambos em @c size), e aquela normalização fazia o segundo evento
 * parecer "0 -> size", reenviando o buffer inteiro como se fosse dado novo.
 * Sem normalizar, a repetição não produz nada, e a volta do anel é tratada na
 * chamada seguinte.
 */
#ifndef DMA_RING_H
#define DMA_RING_H
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Trecho contínuo de dados novos dentro do buffer DMA.
 */
typedef struct {
  /** Posição inicial do trecho no buffer. */
  uint16_t offset;
  /** Quantidade de bytes do trecho. */
  uint16_t length;
} dma_ring_segment_t;

/**
 * @brief Calcula quais trechos do buffer são novos entre duas posições da HAL.
 *
 * Quem chama deve guardar @p position sem alteração como o próximo
 * @p last_position; esta função nunca o normaliza.
 *
 * @param[in]  last_position  Posição informada no evento anterior (0..size).
 * @param[in]  position       Posição informada no evento atual (0..size).
 * @param[in]  size           Tamanho do buffer DMA.
 * @param[out] out            Até dois trechos com os dados novos, em ordem.
 * @return Quantos trechos foram preenchidos (0, 1 ou 2). Retorna 0 quando a
 *         posição não mudou (inclusive no evento duplicado) ou está fora da
 *         faixa.
 */
static inline unsigned dma_ring_segments(uint16_t last_position, uint16_t position, uint16_t size,
                                         dma_ring_segment_t out[2]) {
  if (position > size || last_position > size)
    return 0;
  if (position == last_position)
    return 0;

  unsigned count = 0;
  if (position < last_position) {
    /* O anel deu a volta. A cauda fica vazia quando o evento anterior terminou
       exatamente na borda, que é o que a proteção contra evento duplicado
       acima deixa em last_position. */
    const uint16_t tail = (uint16_t)(size - last_position);
    if (tail) {
      out[count].offset = last_position;
      out[count].length = tail;
      ++count;
    }
    if (position) {
      out[count].offset = 0;
      out[count].length = position;
      ++count;
    }
  } else {
    out[count].offset = last_position;
    out[count].length = (uint16_t)(position - last_position);
    ++count;
  }
  return count;
}
#endif
