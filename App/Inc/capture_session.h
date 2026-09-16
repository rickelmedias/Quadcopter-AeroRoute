/**
 * @file capture_session.h
 * @brief Máquina de estados da sessão de captura binária.
 *
 * Código puro, testado em tests/test_capture_session.c. Regra: uma sessão por
 * boot. Um terminal fechado e reaberto não pode emendar em silêncio uma nova
 * época do estimador numa gravação já existente.
 */
#ifndef CAPTURE_SESSION_H
#define CAPTURE_SESSION_H
#include <stdbool.h>

/**
 * @brief Estado da sessão de captura.
 */
typedef struct {
  /** Verdadeiro depois que a sessão começou; nunca volta a falso no mesmo boot. */
  bool started;
  /** Verdadeiro enquanto os produtores podem gravar. Lido por várias tasks. */
  volatile bool active;
} capture_session_t;

/**
 * @brief Ação que a task de captura deve executar após uma atualização.
 */
typedef enum {
  /** Nada a fazer. */
  CAP_SESSION_IDLE,
  /** A sessão acabou de começar: gravar a SESSION e então ativar os produtores. */
  CAP_SESSION_START,
  /** A sessão terminou: os produtores já foram desativados. */
  CAP_SESSION_STOP
} capture_session_action_t;

/**
 * @brief Atualiza a sessão conforme o sinal DTR do PC.
 *
 * O primeiro DTR ativo do boot inicia a sessão, mas @c active só deve ser
 * ligado pela task depois de enfileirar a SESSION. A queda do DTR encerra a
 * sessão, e um novo DTR no mesmo boot não reinicia nada.
 *
 * @param[in,out] s    Estado da sessão.
 * @param[in]     dtr  Estado atual do DTR (já combinado com a verificação de
 *                     que a conexão é a mesma).
 * @return A ação a executar.
 */
static inline capture_session_action_t capture_session_update(capture_session_t *s, bool dtr) {
  if (!s->started && dtr) { s->started = true; return CAP_SESSION_START; }
  if (s->active && !dtr) { s->active = false; return CAP_SESSION_STOP; }
  return CAP_SESSION_IDLE;
}
#endif
