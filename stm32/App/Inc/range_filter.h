/** @file range_filter.h @brief Filtro e gate do alcance usado pelo fluxo. */
#ifndef RANGE_FILTER_H
#define RANGE_FILTER_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
  float value_m;
  float last_raw_m;
  float candidate_m;
  uint32_t accepted;
  uint32_t rejected;
  uint32_t reanchors;
  uint8_t candidate_count;
  bool initialized;
} range_filter_t;
void range_filter_reset(range_filter_t *f);
/** Retorna false em medida inválida ou salto isolado. Cinco medidas coerentes
 * em um novo patamar reancoram o filtro, permitindo recuperar uma mudança
 * real de altura ou uma volta do telêmetro após perda temporária. */
bool range_filter_update(range_filter_t *f,float measurement_m,float dt_s,
                         bool measurement_valid,float *filtered_m);
#endif
