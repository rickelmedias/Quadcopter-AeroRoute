/**
 * @file test_ctrl_params.c
 * @brief Tabela de parâmetros: nomes, limites, travas e arredondamentos.
 */
#include "control/ctrl_params.h"
#include "flight_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Índice de um parâmetro que precisa existir. */
static size_t idx(const char *name) {
  const int i = ctrl_params_find(name, strlen(name));
  assert(i >= 0);
  return (size_t)i;
}

/** @brief Nomes cabem no MAVLink, são únicos e os padrões respeitam os limites. */
static void test_table_is_consistent(void) {
  ctrl_params_init();
  const size_t n = ctrl_params_count();
  assert(n > 0U);
  for (size_t i = 0U; i < n; ++i) {
    const ctrl_param_desc_t *d = ctrl_params_desc(i);
    assert(d != NULL && d->name != NULL && d->help != NULL);
    assert(strlen(d->name) <= CTRL_PARAM_NAME_MAX);
    assert(d->min <= d->def && d->def <= d->max);
    assert(ctrl_params_get(i) == d->def);
    for (size_t j = i + 1U; j < n; ++j)
      assert(strcmp(d->name, ctrl_params_desc(j)->name) != 0);
  }
  assert(ctrl_params_desc(n) == NULL);
  assert(isnan(ctrl_params_get(n)));
}

/** @brief Busca sem diferenciar maiúsculas e com nome sem terminador (MAVLink). */
static void test_find(void) {
  assert(ctrl_params_find("ctl_w0", 16) == ctrl_params_find("CTL_W0", 16));
  assert(ctrl_params_find("CTL_W", 16) < 0);
  assert(ctrl_params_find("CTL_W0X", 16) < 0);
  assert(ctrl_params_find(NULL, 16) < 0);
  /* Campo de 16 bytes sem '\0', como no PARAM_SET. */
  char raw[16];
  memcpy(raw, "FLT_DRY_RUN\0\0\0\0\0", 16);
  assert(ctrl_params_find(raw, sizeof(raw)) == (int)idx("FLT_DRY_RUN"));
}

/** @brief Limites, travas com o drone armado e arredondamentos. */
static void test_set_rules(void) {
  ctrl_params_init();
  const size_t w0 = idx("CTL_W0");
  assert(ctrl_params_set(w0, 650.0F, true) == CTRL_PARAM_OK); /* ajustável em voo */
  assert(ctrl_params_get(w0) == 650.0F);
  assert(ctrl_params_set(w0, 99.0F, false) == CTRL_PARAM_OUT_OF_RANGE);
  assert(ctrl_params_set(w0, NAN, false) == CTRL_PARAM_OUT_OF_RANGE);
  assert(ctrl_params_get(w0) == 650.0F);

  const size_t sign = idx("SNS_SGN_PITCH");
  assert(ctrl_params_set(sign, 0.3F, true) == CTRL_PARAM_LOCKED);
  assert(ctrl_params_set(sign, 0.3F, false) == CTRL_PARAM_OK);
  assert(ctrl_params_get(sign) == 1.0F);
  assert(ctrl_params_set(sign, -0.2F, false) == CTRL_PARAM_OK);
  assert(ctrl_params_get(sign) == -1.0F);

  const size_t dry = idx("FLT_DRY_RUN");
  assert(ctrl_params_get(dry) == DEFAULT_DRY_RUN);
  assert(ctrl_params_set(dry, 0.0F, true) == CTRL_PARAM_LOCKED);
  assert(ctrl_params_set(dry, 0.4F, false) == CTRL_PARAM_OK && ctrl_params_get(dry) == 0.0F);
  assert(ctrl_params_set(dry, 0.6F, false) == CTRL_PARAM_OK && ctrl_params_get(dry) == 1.0F);

  assert(ctrl_params_set(ctrl_params_count(), 1.0F, false) == CTRL_PARAM_UNKNOWN);

  ctrl_params_t snap;
  ctrl_params_snapshot(&snap);
  assert(snap.w0 == 650.0F && snap.sgn_pitch == -1.0F && snap.dry_run == 1.0F);
}

int main(void) {
  test_table_is_consistent();
  test_find();
  test_set_rules();
  printf("test_ctrl_params: ok (%u parametros)\n", (unsigned)ctrl_params_count());
  return 0;
}
