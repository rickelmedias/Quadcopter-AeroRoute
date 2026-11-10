/**
 * @file range_filter.h
 * @brief Filtro e gate da distância do MTF-01 (restaurado do estimador de
 *        deslocamento da branch `mtf-bno-test`).
 *
 * Um salto isolado maior que a margem (10 cm mais 5 m/s × dt) é recusado: o
 * telêmetro devolve leituras soltas quando o feixe pega a borda de um objeto
 * ou outra superfície. Cinco leituras coerentes num patamar novo reancoram o
 * filtro, o que recupera uma mudança real de altura ou a volta do telêmetro
 * depois de uma perda. As leituras aceitas passam por um passa-baixas de
 * 50 ms, que tira o ruído sem esconder a dinâmica vertical.
 *
 * Código puro, testado em tests/test_nav_ekf.c.
 */
#ifndef CONTROL_RANGE_FILTER_H
#define CONTROL_RANGE_FILTER_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Estado do filtro. */
typedef struct {
  float value_m;            /**< Distância filtrada, em m. */
  float last_raw_m;         /**< Última leitura aceita, em m. */
  float candidate_m;        /**< Média das leituras do patamar candidato, em m. */
  uint32_t accepted;        /**< Leituras aceitas. */
  uint32_t rejected;        /**< Leituras recusadas (inválidas ou saltos). */
  uint32_t reanchors;       /**< Vezes em que um patamar novo foi adotado. */
  uint8_t candidate_count;  /**< Leituras coerentes no patamar candidato. */
  bool initialized;         /**< Já recebeu a primeira leitura. */
} range_filter_t;

/**
 * @brief Esquece tudo.
 * @param[out] f  Filtro.
 */
void range_filter_reset(range_filter_t *f);

/**
 * @brief Entrega uma leitura do telêmetro.
 *
 * @param[in,out] f            Filtro.
 * @param[in]     measurement  Distância lida, em m.
 * @param[in]     dt_s         Tempo desde a leitura anterior, em s.
 * @param[in]     valid        O sensor marcou a leitura como válida.
 * @param[in]     min_m        Menor distância aceita, em m.
 * @param[out]    filtered_m   Distância filtrada (só escrita se aceitar).
 * @return false em leitura inválida ou salto isolado.
 */
bool range_filter_update(range_filter_t *f, float measurement, float dt_s, bool valid,
                         float min_m, float *filtered_m);

#endif
