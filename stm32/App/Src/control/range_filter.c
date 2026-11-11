/**
 * @file range_filter.c
 * @brief Filtro e gate da distância do MTF-01 (contrato em control/range_filter.h).
 */
#include "control/range_filter.h"

#include <math.h>
#include <string.h>

/** Constante de tempo do passa-baixas: tira o jitter sem esconder a dinâmica vertical. */
#define RANGE_TAU_S            0.05F
/** Salto admitido = margem fixa + velocidade vertical extrema × dt. */
#define RANGE_STEP_MARGIN_M    0.10F
/** Velocidade vertical extrema considerada, em m/s. */
#define RANGE_MAX_RATE_M_S     5.0F
/** Leituras coerentes num patamar novo para reancorar. */
#define RANGE_REANCHOR_SAMPLES 5U
/** Maior distância do telêmetro, em m. */
#define RANGE_MAX_M            8.0F

void range_filter_reset(range_filter_t *f) {
  memset(f, 0, sizeof(*f));
}

bool range_filter_update(range_filter_t *f, float measurement, float dt_s, bool valid, float min_m,
                         float *filtered_m) {
  if (!valid || !isfinite(measurement) || !isfinite(dt_s) || dt_s <= 0.0F || measurement < min_m ||
      measurement > RANGE_MAX_M) {
    ++f->rejected;
    return false;
  }
  if (!f->initialized) {
    f->value_m = f->last_raw_m = measurement;
    f->initialized = true;
    ++f->accepted;
    *filtered_m = measurement;
    return true;
  }
  const float allowed = RANGE_STEP_MARGIN_M + RANGE_MAX_RATE_M_S * dt_s;
  if (fabsf(measurement - f->last_raw_m) > allowed) {
    ++f->rejected;
    if (f->candidate_count == 0U || fabsf(measurement - f->candidate_m) > allowed) {
      f->candidate_m = measurement;
      f->candidate_count = 1U;
    } else {
      f->candidate_m += (measurement - f->candidate_m) / (float)(f->candidate_count + 1U);
      if (f->candidate_count < UINT8_MAX)
        ++f->candidate_count;
    }
    if (f->candidate_count < RANGE_REANCHOR_SAMPLES)
      return false;
    f->value_m = f->last_raw_m = f->candidate_m;
    f->candidate_count = 0U;
    ++f->reanchors;
    ++f->accepted;
    *filtered_m = f->value_m;
    return true;
  }
  f->candidate_count = 0U;
  f->last_raw_m = measurement;
  const float alpha = dt_s / (RANGE_TAU_S + dt_s);
  f->value_m += alpha * (measurement - f->value_m);
  ++f->accepted;
  *filtered_m = f->value_m;
  return true;
}
