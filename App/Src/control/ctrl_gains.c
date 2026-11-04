/**
 * @file ctrl_gains.c
 * @brief Ganhos em uso e envio atômico pela Pi (contrato em control/ctrl_gains.h).
 *
 * O preparo guarda os valores exatamente como chegaram (float32), na ordem
 * canônica do CRC. Só no commit eles viram ::lqg_real_t e substituem os
 * ganhos em uso, dentro de uma seção crítica.
 */
#include "control/ctrl_gains.h"

#include "control/ctrl_port.h"
#include "control/lqg_ert_model.h"

#include <math.h>
#include <string.h>

/** Quantidade de valores de um conjunto completo (48 + 16 + 108 + 108). */
#define TOTAL_VALUES 280U
/** Máscara com as 32 linhas recebidas. */
#define ALL_ROWS 0xFFFFFFFFUL

/** @brief Forma de uma matriz e sua posição no formato canônico. */
typedef struct {
  uint8_t rows;        /**< Linhas. */
  uint8_t cols;        /**< Colunas. */
  uint16_t offset;     /**< Primeiro valor no vetor canônico. */
  uint8_t first_row;   /**< Primeiro bit da matriz na máscara de linhas. */
} matrix_layout_t;

/** Formato de cada matriz, na ordem Kd, Kfd, L, M. */
static const matrix_layout_t LAYOUT[CTRL_GAIN_COUNT] = {
    [CTRL_GAIN_KD] = {LQG_NU, LQG_NX, 0U, 0U},
    [CTRL_GAIN_KFD] = {LQG_NU, LQG_NR, 48U, 4U},
    [CTRL_GAIN_L] = {LQG_NX, LQG_NY, 64U, 8U},
    [CTRL_GAIN_M] = {LQG_NX, LQG_NY, 172U, 20U},
};

/** Ganhos em uso; protegidos por seção crítica. */
static lqg_gains_t active;
/** Geração dos ganhos em uso. */
static volatile uint32_t generation;
/** Os ganhos em uso são os do Simulink. */
static volatile bool active_default;
/** CRC dos ganhos em uso. */
static volatile uint32_t active_crc;

/** Valores preparados, no formato canônico. Só a task do DroneLink mexe. */
static float staged[TOTAL_VALUES];
/** Sessão em preparo (0 = nenhuma). */
static uint32_t staged_session;
/** Linhas preparadas da sessão atual. */
static uint32_t staged_rows;
/** Ganhos montados no commit (estático: grande demais para a pilha). */
static lqg_gains_t incoming;

/**
 * @brief Primeiro elemento de uma matriz dentro de um conjunto de ganhos.
 * @param[in] g  Ganhos.
 * @param[in] m  Matriz.
 * @return Ponteiro para o elemento [0][0]; as linhas são contíguas.
 */
static lqg_real_t *matrix_data(lqg_gains_t *g, ctrl_gain_matrix_t m) {
  switch (m) {
    case CTRL_GAIN_KD:
      return &g->Kd[0][0];
    case CTRL_GAIN_KFD:
      return &g->Kfd[0][0];
    case CTRL_GAIN_L:
      return &g->L[0][0];
    case CTRL_GAIN_M:
    default:
      return &g->M[0][0];
  }
}

uint32_t ctrl_crc32(uint32_t crc, const void *data, uint32_t len) {
  const uint8_t *p = (const uint8_t *)data;
  crc = ~crc;
  for (uint32_t i = 0U; i < len; ++i) {
    crc ^= p[i];
    for (unsigned bit = 0U; bit < 8U; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
  }
  return ~crc;
}

/**
 * @brief Acrescenta um float32 ao CRC em little-endian, qualquer que seja a
 *        ordem de bytes da máquina.
 */
static uint32_t crc_float(uint32_t crc, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  const uint8_t bytes[4] = {(uint8_t)bits, (uint8_t)(bits >> 8),
                            (uint8_t)(bits >> 16), (uint8_t)(bits >> 24)};
  return ctrl_crc32(crc, bytes, sizeof(bytes));
}

uint32_t ctrl_gains_crc(const lqg_gains_t *g) {
  uint32_t crc = 0U;
  for (unsigned m = 0U; m < CTRL_GAIN_COUNT; ++m) {
    const lqg_real_t *v = matrix_data((lqg_gains_t *)g, (ctrl_gain_matrix_t)m);
    const unsigned n = (unsigned)LAYOUT[m].rows * LAYOUT[m].cols;
    for (unsigned i = 0U; i < n; ++i) {
      crc = crc_float(crc, (float)v[i]);
    }
  }
  return crc;
}

/** @brief CRC do vetor preparado. */
static uint32_t staged_crc(void) {
  uint32_t crc = 0U;
  for (unsigned i = 0U; i < TOTAL_VALUES; ++i) {
    crc = crc_float(crc, staged[i]);
  }
  return crc;
}

/** @brief Publica um conjunto como ganhos em uso. */
static void install(const lqg_gains_t *g, bool is_default, uint32_t crc) {
  CTRL_ENTER_CRITICAL();
  active = *g;
  active_default = is_default;
  active_crc = crc;
  generation = generation + 1U;
  CTRL_EXIT_CRITICAL();
}

void ctrl_gains_init(void) {
  ctrl_gains_abort();
  ctrl_gains_restore_default();
}

bool ctrl_gains_shape(ctrl_gain_matrix_t m, uint8_t *rows, uint8_t *cols) {
  if ((unsigned)m >= CTRL_GAIN_COUNT) {
    return false;
  }
  if (rows != NULL) *rows = LAYOUT[m].rows;
  if (cols != NULL) *cols = LAYOUT[m].cols;
  return true;
}

uint32_t ctrl_gains_generation(void) { return generation; }

void ctrl_gains_copy_active(lqg_gains_t *out, uint32_t *gen) {
  CTRL_ENTER_CRITICAL();
  *out = active;
  if (gen != NULL) *gen = generation;
  CTRL_EXIT_CRITICAL();
}

bool ctrl_gains_active_is_default(void) { return active_default; }

uint32_t ctrl_gains_active_crc(void) { return active_crc; }

ctrl_gains_status_t ctrl_gains_stage_row(uint32_t session, ctrl_gain_matrix_t m,
                                         uint8_t row, const float *values,
                                         uint8_t count) {
  if (session == 0U) {
    return CTRL_GAINS_BAD_SESSION;
  }
  if ((unsigned)m >= CTRL_GAIN_COUNT || row >= LAYOUT[m].rows ||
      count != LAYOUT[m].cols || values == NULL) {
    return CTRL_GAINS_BAD_INDEX;
  }
  if (session != staged_session) {
    staged_session = session;
    staged_rows = 0U;
    memset(staged, 0, sizeof(staged));
  }
  memcpy(&staged[LAYOUT[m].offset + (unsigned)row * LAYOUT[m].cols], values,
         (size_t)count * sizeof(float));
  staged_rows |= 1UL << (LAYOUT[m].first_row + row);
  return CTRL_GAINS_OK;
}

ctrl_gains_status_t ctrl_gains_commit(uint32_t session, uint32_t expected_crc,
                                      uint32_t *computed_crc,
                                      uint32_t *rows_received) {
  const bool same_session = session != 0U && session == staged_session;
  const uint32_t crc = same_session ? staged_crc() : 0U;
  if (rows_received != NULL) *rows_received = same_session ? staged_rows : 0U;
  if (computed_crc != NULL) *computed_crc = crc;
  if (!same_session) {
    return CTRL_GAINS_BAD_SESSION;
  }
  if (staged_rows != ALL_ROWS) {
    return CTRL_GAINS_INCOMPLETE;
  }
  for (unsigned i = 0U; i < TOTAL_VALUES; ++i) {
    if (!isfinite(staged[i])) {
      return CTRL_GAINS_NOT_FINITE;
    }
  }
  if (crc != expected_crc) {
    return CTRL_GAINS_CRC_MISMATCH;
  }
  for (unsigned m = 0U; m < CTRL_GAIN_COUNT; ++m) {
    lqg_real_t *dst = matrix_data(&incoming, (ctrl_gain_matrix_t)m);
    const unsigned n = (unsigned)LAYOUT[m].rows * LAYOUT[m].cols;
    for (unsigned i = 0U; i < n; ++i) {
      dst[i] = (lqg_real_t)staged[LAYOUT[m].offset + i];
    }
  }
  install(&incoming, false, crc);
  ctrl_gains_abort();
  return CTRL_GAINS_OK;
}

void ctrl_gains_abort(void) {
  staged_session = 0U;
  staged_rows = 0U;
}

void ctrl_gains_restore_default(void) {
  install(&LQG_ERT_DEFAULT_GAINS, true, ctrl_gains_crc(&LQG_ERT_DEFAULT_GAINS));
}
