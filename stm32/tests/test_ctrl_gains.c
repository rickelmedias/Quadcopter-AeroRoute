/**
 * @file test_ctrl_gains.c
 * @brief Envio atômico de ganhos: sessões, linhas, CRC e restauração.
 *
 * Imprime também o CRC dos ganhos do Simulink ("default_crc=..."), que
 * tests/run.sh compara com o calculado em Python pela ferramenta da Pi: os
 * dois lados precisam concordar sobre o formato do envio.
 */
#include "control/ctrl_gains.h"
#include "control/lqg_ert_model.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Valores de uma linha dos ganhos padrão, em float32. */
static void default_row(ctrl_gain_matrix_t m, uint8_t row, float *out, uint8_t *count) {
  uint8_t rows, cols;
  assert(ctrl_gains_shape(m, &rows, &cols));
  const lqg_gains_t *g = &LQG_ERT_DEFAULT_GAINS;
  for (uint8_t c = 0; c < cols; ++c) {
    switch (m) {
    case CTRL_GAIN_KD:
      out[c] = (float)g->Kd[row][c];
      break;
    case CTRL_GAIN_KFD:
      out[c] = (float)g->Kfd[row][c];
      break;
    case CTRL_GAIN_L:
      out[c] = (float)g->L[row][c];
      break;
    default:
      out[c] = (float)g->M[row][c];
      break;
    }
  }
  *count = cols;
}

/** @brief Envia todas as linhas dos ganhos padrão, multiplicadas por @p scale. */
static void stage_all(uint32_t session, float scale, int skip_matrix, int skip_row) {
  for (int m = 0; m < CTRL_GAIN_COUNT; ++m) {
    uint8_t rows;
    assert(ctrl_gains_shape((ctrl_gain_matrix_t)m, &rows, NULL));
    for (uint8_t r = 0; r < rows; ++r) {
      if (m == skip_matrix && r == skip_row)
        continue;
      float v[CTRL_GAINS_MAX_COLS];
      uint8_t n;
      default_row((ctrl_gain_matrix_t)m, r, v, &n);
      for (uint8_t i = 0; i < n; ++i)
        v[i] *= scale;
      assert(ctrl_gains_stage_row(session, (ctrl_gain_matrix_t)m, r, v, n) == CTRL_GAINS_OK);
    }
  }
}

/** @brief CRC dos ganhos padrão multiplicados por @p scale, como a Pi calcularia. */
static uint32_t expected_crc(float scale) {
  static lqg_gains_t g;
  g = LQG_ERT_DEFAULT_GAINS;
  lqg_real_t *v = &g.Kd[0][0];
  for (unsigned i = 0; i < sizeof(g.Kd) / sizeof(lqg_real_t); ++i)
    v[i] = (float)v[i] * scale;
  v = &g.Kfd[0][0];
  for (unsigned i = 0; i < sizeof(g.Kfd) / sizeof(lqg_real_t); ++i)
    v[i] = (float)v[i] * scale;
  v = &g.L[0][0];
  for (unsigned i = 0; i < sizeof(g.L) / sizeof(lqg_real_t); ++i)
    v[i] = (float)v[i] * scale;
  v = &g.M[0][0];
  for (unsigned i = 0; i < sizeof(g.M) / sizeof(lqg_real_t); ++i)
    v[i] = (float)v[i] * scale;
  return ctrl_gains_crc(&g);
}

/** @brief CRC-32 confere com o valor de referência do zlib. */
static void test_crc32_reference_vector(void) {
  assert(ctrl_crc32(0U, "123456789", 9U) == 0xCBF43926UL);
  /* Incremental é igual a de uma vez. */
  const uint32_t part = ctrl_crc32(0U, "1234", 4U);
  assert(ctrl_crc32(part, "56789", 5U) == 0xCBF43926UL);
}

/** @brief Começa com os ganhos do Simulink. */
static void test_starts_with_simulink_gains(void) {
  ctrl_gains_init();
  assert(ctrl_gains_active_is_default());
  assert(ctrl_gains_active_crc() == ctrl_gains_crc(&LQG_ERT_DEFAULT_GAINS));
  static lqg_gains_t g;
  ctrl_gains_copy_active(&g, NULL);
  assert(memcmp(&g, &LQG_ERT_DEFAULT_GAINS, sizeof(g)) == 0);
}

/** @brief Um envio completo e íntegro troca os ganhos de uma vez. */
static void test_full_upload_commits(void) {
  ctrl_gains_init();
  const uint32_t gen = ctrl_gains_generation();
  stage_all(7U, 2.0F, -1, -1);
  /* Antes do commit nada muda. */
  assert(ctrl_gains_generation() == gen && ctrl_gains_active_is_default());
  uint32_t crc, rows;
  assert(ctrl_gains_commit(7U, expected_crc(2.0F), &crc, &rows) == CTRL_GAINS_OK);
  assert(rows == 0xFFFFFFFFUL && crc == expected_crc(2.0F));
  assert(ctrl_gains_generation() != gen);
  assert(!ctrl_gains_active_is_default());
  assert(ctrl_gains_active_crc() == expected_crc(2.0F));
  static lqg_gains_t g;
  ctrl_gains_copy_active(&g, NULL);
  assert(g.Kd[0][2] == (lqg_real_t)((float)LQG_ERT_DEFAULT_GAINS.Kd[0][2] * 2.0F));
  assert(g.M[11][8] == (lqg_real_t)((float)LQG_ERT_DEFAULT_GAINS.M[11][8] * 2.0F));
  /* A sessão foi consumida: repetir o commit não faz nada. */
  assert(ctrl_gains_commit(7U, expected_crc(2.0F), NULL, NULL) == CTRL_GAINS_BAD_SESSION);
  ctrl_gains_restore_default();
  assert(ctrl_gains_active_is_default());
}

/** @brief CRC errado, linha faltando ou valor não finito não trocam nada. */
static void test_rejected_commits_keep_active_gains(void) {
  ctrl_gains_init();
  const uint32_t gen = ctrl_gains_generation();

  stage_all(9U, 1.5F, -1, -1);
  assert(ctrl_gains_commit(9U, expected_crc(1.5F) ^ 1U, NULL, NULL) == CTRL_GAINS_CRC_MISMATCH);

  stage_all(10U, 1.5F, CTRL_GAIN_L, 5);
  uint32_t rows;
  assert(ctrl_gains_commit(10U, expected_crc(1.5F), NULL, &rows) == CTRL_GAINS_INCOMPLETE);
  assert(rows == (0xFFFFFFFFUL & ~(1UL << (8 + 5))));

  stage_all(11U, 1.0F, -1, -1);
  float bad[CTRL_GAINS_MAX_COLS];
  uint8_t n;
  default_row(CTRL_GAIN_KFD, 2, bad, &n);
  bad[1] = NAN;
  assert(ctrl_gains_stage_row(11U, CTRL_GAIN_KFD, 2, bad, n) == CTRL_GAINS_OK);
  assert(ctrl_gains_commit(11U, 0U, NULL, NULL) == CTRL_GAINS_NOT_FINITE);

  assert(ctrl_gains_generation() == gen && ctrl_gains_active_is_default());
}

/** @brief Linhas fora do formato e sessões trocadas são recusadas. */
static void test_bad_rows_and_sessions(void) {
  ctrl_gains_init();
  float v[CTRL_GAINS_MAX_COLS] = {0};
  assert(ctrl_gains_stage_row(0U, CTRL_GAIN_KD, 0, v, 12) == CTRL_GAINS_BAD_SESSION);
  assert(ctrl_gains_stage_row(1U, CTRL_GAIN_KD, 4, v, 12) == CTRL_GAINS_BAD_INDEX);
  assert(ctrl_gains_stage_row(1U, CTRL_GAIN_KD, 0, v, 9) == CTRL_GAINS_BAD_INDEX);
  assert(ctrl_gains_stage_row(1U, CTRL_GAIN_COUNT, 0, v, 12) == CTRL_GAINS_BAD_INDEX);
  assert(ctrl_gains_stage_row(1U, CTRL_GAIN_M, 12, v, 9) == CTRL_GAINS_BAD_INDEX);

  /* Uma sessão nova descarta a anterior. */
  stage_all(20U, 1.0F, -1, -1);
  assert(ctrl_gains_stage_row(21U, CTRL_GAIN_KD, 0, v, 12) == CTRL_GAINS_OK);
  assert(ctrl_gains_commit(20U, expected_crc(1.0F), NULL, NULL) == CTRL_GAINS_BAD_SESSION);
  uint32_t rows;
  assert(ctrl_gains_commit(21U, 0U, NULL, &rows) == CTRL_GAINS_INCOMPLETE && rows == 1U);

  /* Abortar também descarta. */
  stage_all(22U, 1.0F, -1, -1);
  ctrl_gains_abort();
  assert(ctrl_gains_commit(22U, expected_crc(1.0F), NULL, NULL) == CTRL_GAINS_BAD_SESSION);
}

int main(void) {
  test_crc32_reference_vector();
  test_starts_with_simulink_gains();
  test_full_upload_commits();
  test_rejected_commits_keep_active_gains();
  test_bad_rows_and_sessions();
  printf("default_crc=%08lX\n", (unsigned long)ctrl_gains_crc(&LQG_ERT_DEFAULT_GAINS));
  printf("test_ctrl_gains: ok\n");
  return 0;
}
