/**
 * @file lqi_gen.c
 * @brief Gera App/Src/control/lqi_gains.c a partir do projeto do LQI.
 *
 * Uso (a partir da raiz do repositório):
 *
 * @verbatim
 *   gcc -std=c11 -O2 -IApp/Inc -Itools/lqi tools/lqi/lqi_gen.c tools/lqi/lqi_design.c \
 *       tools/lqi/lqr_design.c App/Src/control/lqg_ert_model.c -lm -o /tmp/lqi_gen
 *   /tmp/lqi_gen App/Src/control/lqi_gains.c          # gera
 *   /tmp/lqi_gen --check App/Src/control/lqi_gains.c  # confere (tests/run.sh)
 * @endverbatim
 */
#include "lqi_design.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Tamanho máximo do arquivo gerado. */
#define MAX_TEXT 32768

/** Nomes dos estados, para os comentários. */
static const char *const STATE_NAMES[LQG_NX] = {"x", "y", "z", "phi", "theta", "psi",
                                                "vx", "vy", "vz", "p", "q", "r"};

/** @brief Acrescenta texto formatado a @p buf. */
static void put(char *buf, size_t *used, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
#include <stdarg.h>
static void put(char *buf, size_t *used, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(buf + *used, MAX_TEXT - *used, fmt, ap);
  va_end(ap);
  if (n > 0) *used += (size_t)n;
}

/** @brief Monta o texto do arquivo. */
static void render(const lqi_weights_t *w, const lqi_gains_t *g, char *buf) {
  size_t used = 0U;
  put(buf, &used, "/**\n * @file lqi_gains.c\n * @brief Ganhos do LQI (contrato em control/lqi.h).\n *\n");
  put(buf, &used, " * GERADO por tools/lqi/lqi_gen.c; não edite à mão. Pesos:\n *\n");
  put(buf, &used, " * - Q (estados do modelo, só atitude e altura entram):");
  for (unsigned i = 0U; i < LQG_NX; ++i) put(buf, &used, " %s=%g", STATE_NAMES[i], w->q[i]);
  put(buf, &used, "\n * - R: %g %g %g %g\n", w->r[0], w->r[1], w->r[2], w->r[3]);
  put(buf, &used, " * - integradores: int_z=%g int_phi=%g int_theta=%g\n */\n",
      w->q_int[0], w->q_int[1], w->q_int[2]);
  put(buf, &used, "#include \"control/lqi.h\"\n\nconst lqi_gains_t LQI_GAINS = {\n");
  put(buf, &used, "  /* Kx: colunas x y z phi theta psi vx vy vz p q r */\n  .Kx = {\n");
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    put(buf, &used, "    {");
    for (unsigned j = 0U; j < LQG_NX; ++j) put(buf, &used, "%s%.17g", j ? ", " : " ", g->Kx[i][j]);
    put(buf, &used, " },  /* u%u */\n", i + 1U);
  }
  put(buf, &used, "  },\n  /* Kfd: colunas r_x r_y r_z r_psi */\n  .Kfd = {\n");
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    put(buf, &used, "    {");
    for (unsigned j = 0U; j < LQG_NR; ++j) put(buf, &used, "%s%.17g", j ? ", " : " ", g->Kfd[i][j]);
    put(buf, &used, " },  /* u%u */\n", i + 1U);
  }
  put(buf, &used, "  },\n  /* Ki: colunas int_z int_phi int_theta */\n  .Ki = {\n");
  for (unsigned i = 0U; i < LQG_NU; ++i) {
    put(buf, &used, "    {");
    for (unsigned j = 0U; j < LQI_NI; ++j) put(buf, &used, "%s%.17g", j ? ", " : " ", g->Ki[i][j]);
    put(buf, &used, " },  /* u%u */\n", i + 1U);
  }
  put(buf, &used, "  },\n};\n");
}

int main(int argc, char **argv) {
  const int check = argc == 3 && strcmp(argv[1], "--check") == 0;
  if (!(argc == 2 || check)) {
    fprintf(stderr, "uso: %s [--check] App/Src/control/lqi_gains.c\n", argv[0]);
    return 2;
  }
  const char *path = argv[argc - 1];
  lqi_weights_t w;
  lqi_default_weights(&w);
  lqi_gains_t g;
  if (lqi_design(&w, &g) < 0) {
    fprintf(stderr, "dlqr não convergiu\n");
    return 1;
  }
  static char text[MAX_TEXT];
  render(&w, &g, text);
  if (check) {
    static char existing[MAX_TEXT];
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "%s não existe\n", path); return 1; }
    const size_t n = fread(existing, 1, sizeof(existing) - 1U, f);
    fclose(f);
    existing[n] = '\0';
    if (strcmp(existing, text) != 0) {
      fprintf(stderr, "%s desatualizado: rode tools/lqi/lqi_gen\n", path);
      return 1;
    }
    printf("%s confere com o projeto\n", path);
    return 0;
  }
  FILE *f = fopen(path, "wb");
  if (!f) { perror(path); return 1; }
  fputs(text, f);
  fclose(f);
  printf("gerado %s\n", path);
  return 0;
}
