/**
 * @file lqi_tuning.c
 * @brief Ganhos do LQI em uso e envio pelo console (contrato em control/lqi_tuning.h).
 *
 * O preparo guarda os valores exatamente como chegaram (float32), na ordem
 * canônica do CRC. Só no commit eles viram ::lqg_real_t e substituem os
 * ganhos em uso, dentro de uma seção crítica.
 */
#include "control/lqi_tuning.h"

#include "control/ctrl_gains.h"
#include "control/ctrl_port.h"

#include <math.h>
#include <string.h>

/** @brief Forma de uma matriz e sua posição no formato canônico. */
typedef struct {
  uint8_t rows;     /**< Linhas. */
  uint8_t cols;     /**< Colunas. */
  uint8_t offset;   /**< Primeiro valor no vetor canônico. */
} matrix_layout_t;

/** Formato de cada matriz, na ordem Kx, Kfd, Ki. */
static const matrix_layout_t LAYOUT[LQI_TUNING_COUNT] = {
    [LQI_TUNING_KX] = {LQG_NU, LQG_NX, 0U},
    [LQI_TUNING_KFD] = {LQG_NU, LQG_NR, 48U},
    [LQI_TUNING_KI] = {LQG_NU, LQI_NI, 64U},
};

/** Ganhos em uso; protegidos por seção crítica. */
static lqi_gains_t active;
/** Geração dos ganhos em uso. */
static volatile uint32_t generation;
/** Os ganhos em uso são os padrão. */
static volatile bool active_default;
/** CRC dos ganhos em uso. */
static volatile uint32_t active_crc;

/** Valores preparados, no formato canônico. Só a task do console mexe. */
static float staged[LQI_TUNING_VALUES];
/** Quais valores de ::staged chegaram desde o último begin. */
static bool staged_set[LQI_TUNING_VALUES];
/** Ganhos montados no commit. */
static lqi_gains_t incoming;

/** @brief Primeiro elemento de uma matriz; as linhas são contíguas. */
static lqg_real_t *matrix_data(lqi_gains_t *g, lqi_tuning_matrix_t m) {
  switch (m) {
    case LQI_TUNING_KX:
      return &g->Kx[0][0];
    case LQI_TUNING_KFD:
      return &g->Kfd[0][0];
    case LQI_TUNING_KI:
    default:
      return &g->Ki[0][0];
  }
}

/** @brief Acrescenta um float32 ao CRC em little-endian. */
static uint32_t crc_float(uint32_t crc, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  const uint8_t bytes[4] = {(uint8_t)bits, (uint8_t)(bits >> 8),
                            (uint8_t)(bits >> 16), (uint8_t)(bits >> 24)};
  return ctrl_crc32(crc, bytes, sizeof(bytes));
}

uint32_t lqi_tuning_crc(const lqi_gains_t *g) {
  uint32_t crc = 0U;
  for (unsigned m = 0U; m < LQI_TUNING_COUNT; ++m) {
    const lqg_real_t *v = matrix_data((lqi_gains_t *)g, (lqi_tuning_matrix_t)m);
    const unsigned n = (unsigned)LAYOUT[m].rows * LAYOUT[m].cols;
    for (unsigned i = 0U; i < n; ++i) crc = crc_float(crc, (float)v[i]);
  }
  return crc;
}

/** @brief Publica um conjunto como ganhos em uso. */
static void install(const lqi_gains_t *g, bool is_default, uint32_t crc) {
  CTRL_ENTER_CRITICAL();
  active = *g;
  active_default = is_default;
  active_crc = crc;
  generation = generation + 1U;
  CTRL_EXIT_CRITICAL();
}

void lqi_tuning_init(void) {
  lqi_tuning_begin();
  lqi_tuning_restore_default();
}

uint32_t lqi_tuning_generation(void) { return generation; }

void lqi_tuning_copy_active(lqi_gains_t *out, uint32_t *gen) {
  CTRL_ENTER_CRITICAL();
  *out = active;
  if (gen != NULL) *gen = generation;
  CTRL_EXIT_CRITICAL();
}

bool lqi_tuning_is_default(void) { return active_default; }

uint32_t lqi_tuning_active_crc(void) { return active_crc; }

void lqi_tuning_begin(void) {
  memset(staged, 0, sizeof(staged));
  memset(staged_set, 0, sizeof(staged_set));
}

lqi_tuning_status_t lqi_tuning_stage(lqi_tuning_matrix_t m, unsigned row, unsigned col,
                                     float value) {
  if ((unsigned)m >= LQI_TUNING_COUNT || row >= LAYOUT[m].rows || col >= LAYOUT[m].cols) {
    return LQI_TUNING_BAD_INDEX;
  }
  const unsigned i = LAYOUT[m].offset + row * LAYOUT[m].cols + col;
  staged[i] = value;
  staged_set[i] = true;
  return LQI_TUNING_OK;
}

lqi_tuning_status_t lqi_tuning_commit(uint32_t expected_crc, uint32_t *computed_crc) {
  uint32_t crc = 0U;
  for (unsigned i = 0U; i < LQI_TUNING_VALUES; ++i) crc = crc_float(crc, staged[i]);
  if (computed_crc != NULL) *computed_crc = crc;
  for (unsigned i = 0U; i < LQI_TUNING_VALUES; ++i) {
    if (!staged_set[i]) return LQI_TUNING_INCOMPLETE;
  }
  for (unsigned i = 0U; i < LQI_TUNING_VALUES; ++i) {
    if (!isfinite(staged[i])) return LQI_TUNING_NOT_FINITE;
  }
  if (crc != expected_crc) return LQI_TUNING_CRC_MISMATCH;
  for (unsigned m = 0U; m < LQI_TUNING_COUNT; ++m) {
    lqg_real_t *dst = matrix_data(&incoming, (lqi_tuning_matrix_t)m);
    const unsigned n = (unsigned)LAYOUT[m].rows * LAYOUT[m].cols;
    for (unsigned i = 0U; i < n; ++i) dst[i] = (lqg_real_t)staged[LAYOUT[m].offset + i];
  }
  install(&incoming, false, crc);
  lqi_tuning_begin();
  return LQI_TUNING_OK;
}

void lqi_tuning_restore_default(void) {
  install(&LQI_GAINS, true, lqi_tuning_crc(&LQI_GAINS));
}

const char *lqi_tuning_status_text(lqi_tuning_status_t s) {
  static const char *const text[] = {"ok", "indice invalido", "incompleto", "CRC nao confere",
                                     "valor nao finito"};
  return (unsigned)s < sizeof(text) / sizeof(text[0]) ? text[s] : "?";
}
