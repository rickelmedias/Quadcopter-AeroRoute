/**
 * @file lqg_tuning.c
 * @brief Ganhos do LQG ajustado em uso e envio pelo console (contrato em control/lqg_tuning.h).
 *
 * O preparo guarda os valores exatamente como chegaram (float32), na ordem
 * canônica do CRC. Só no commit eles viram ::lqg_real_t e substituem os
 * ganhos em uso, dentro de uma seção crítica.
 */
#include "control/lqg_tuning.h"

#include "control/ctrl_gains.h"
#include "control/ctrl_port.h"
#include "control/lqg_dist.h"

#include <math.h>
#include <string.h>

/** Posição de Kfd no vetor canônico (depois dos 48 de Kd). */
#define KFD_OFFSET (LQG_NU * LQG_NX)

/** Ganhos em uso; protegidos por seção crítica. */
static lqg_aj_gains_t active;
/** Geração dos ganhos em uso. */
static volatile uint32_t generation;
/** Os ganhos em uso são os gerados. */
static volatile bool active_default;
/** CRC dos ganhos em uso. */
static volatile uint32_t active_crc;

/** Valores preparados, no formato canônico. Só a task do console mexe. */
static float staged[LQG_TUNING_VALUES];
/** Quais valores de ::staged chegaram desde o último begin. */
static bool staged_set[LQG_TUNING_VALUES];
/** Ganhos montados no commit. */
static lqg_aj_gains_t incoming;
/** Ganhos gerados, no formato de ::lqg_aj_gains_t. */
static lqg_aj_gains_t generated;

/** @brief Acrescenta um float32 ao CRC em little-endian. */
static uint32_t crc_float(uint32_t crc, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  const uint8_t bytes[4] = {(uint8_t)bits, (uint8_t)(bits >> 8),
                            (uint8_t)(bits >> 16), (uint8_t)(bits >> 24)};
  return ctrl_crc32(crc, bytes, sizeof(bytes));
}

uint32_t lqg_tuning_crc(const lqg_aj_gains_t *g) {
  uint32_t crc = 0U;
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned j = 0U; j < LQG_NX; ++j) crc = crc_float(crc, (float)g->Kd[i][j]);
  }
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned j = 0U; j < LQG_NR; ++j) crc = crc_float(crc, (float)g->Kfd[i][j]);
  }
  return crc;
}

/** @brief Publica um conjunto como ganhos em uso. */
static void install(const lqg_aj_gains_t *g, bool is_default, uint32_t crc) {
  CTRL_ENTER_CRITICAL();
  active = *g;
  active_default = is_default;
  active_crc = crc;
  generation = generation + 1U;
  CTRL_EXIT_CRITICAL();
}

void lqg_tuning_init(void) {
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned j = 0U; j < LQG_NX; ++j) generated.Kd[i][j] = LQG_AJ_KD[i][j];
    for (unsigned j = 0U; j < LQG_NR; ++j) generated.Kfd[i][j] = LQG_AJ_KFD[i][j];
  }
  lqg_tuning_begin();
  lqg_tuning_restore_default();
}

uint32_t lqg_tuning_generation(void) { return generation; }

void lqg_tuning_copy_active(lqg_aj_gains_t *out, uint32_t *gen) {
  CTRL_ENTER_CRITICAL();
  *out = active;
  if (gen != NULL) *gen = generation;
  CTRL_EXIT_CRITICAL();
}

bool lqg_tuning_is_default(void) { return active_default; }

uint32_t lqg_tuning_active_crc(void) { return active_crc; }

void lqg_tuning_begin(void) {
  memset(staged, 0, sizeof(staged));
  memset(staged_set, 0, sizeof(staged_set));
}

lqg_tuning_status_t lqg_tuning_stage(lqg_tuning_matrix_t m, unsigned row, unsigned col,
                                     float value) {
  const unsigned cols = m == LQG_TUNING_KD ? LQG_NX : LQG_NR;
  if ((unsigned)m >= LQG_TUNING_COUNT || row >= LQG_NU || col >= cols) {
    return LQG_TUNING_BAD_INDEX;
  }
  const unsigned i = (m == LQG_TUNING_KD ? 0U : KFD_OFFSET) + row * cols + col;
  staged[i] = value;
  staged_set[i] = true;
  return LQG_TUNING_OK;
}

lqg_tuning_status_t lqg_tuning_commit(uint32_t expected_crc, uint32_t *computed) {
  uint32_t crc = 0U;
  for (unsigned i = 0U; i < LQG_TUNING_VALUES; ++i) crc = crc_float(crc, staged[i]);
  if (computed != NULL) *computed = crc;
  for (unsigned i = 0U; i < LQG_TUNING_VALUES; ++i) {
    if (!staged_set[i]) return LQG_TUNING_INCOMPLETE;
  }
  for (unsigned i = 0U; i < LQG_TUNING_VALUES; ++i) {
    if (!isfinite(staged[i])) return LQG_TUNING_NOT_FINITE;
  }
  if (crc != expected_crc) return LQG_TUNING_CRC_MISMATCH;
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned j = 0U; j < LQG_NX; ++j) incoming.Kd[i][j] = (lqg_real_t)staged[i * LQG_NX + j];
    for (unsigned j = 0U; j < LQG_NR; ++j) {
      incoming.Kfd[i][j] = (lqg_real_t)staged[KFD_OFFSET + i * LQG_NR + j];
    }
  }
  install(&incoming, false, crc);
  lqg_tuning_begin();
  return LQG_TUNING_OK;
}

void lqg_tuning_restore_default(void) {
  install(&generated, true, lqg_tuning_crc(&generated));
}

void lqg_tuning_apply(const lqg_aj_gains_t *aj, lqg_gains_t *gains) {
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    for (unsigned j = 0U; j < LQG_NX; ++j) gains->Kd[i][j] = aj->Kd[i][j];
    for (unsigned j = 0U; j < LQG_NR; ++j) gains->Kfd[i][j] = aj->Kfd[i][j];
  }
}

const char *lqg_tuning_status_text(lqg_tuning_status_t s) {
  static const char *const text[] = {"ok", "indice invalido", "incompleto", "CRC nao confere",
                                     "valor nao finito"};
  return (unsigned)s < sizeof(text) / sizeof(text[0]) ? text[s] : "?";
}
