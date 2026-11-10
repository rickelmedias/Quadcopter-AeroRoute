/**
 * @file test_capture_session.c
 * @brief Testes de host da máquina de estados da sessão de captura.
 */
#include "capture_session.h"
#include <assert.h>
#include <stdio.h>
/**
 * @brief Roda os casos: espera pelo DTR, ordem de ativação, parada e uma
 *        única sessão por boot.
 */
int main(void) {
  capture_session_t s = {0};
  for (unsigned i=0; i<10000; ++i) {
    assert(capture_session_update(&s, false) == CAP_SESSION_IDLE);
    assert(!s.started && !s.active);
  }
  assert(capture_session_update(&s, true) == CAP_SESSION_START);
  assert(s.started && !s.active); /* a SESSION precisa entrar na fila antes da ativação */
  s.active = true;
  assert(capture_session_update(&s, true) == CAP_SESSION_IDLE);
  assert(capture_session_update(&s, false) == CAP_SESSION_STOP);
  assert(!s.active);
  assert(capture_session_update(&s, true) == CAP_SESSION_IDLE);
  assert(!s.active); /* reabrir a porta não cria uma segunda sessão sem reiniciar */
  puts("Capture session: waits for DTR, ordered activation, stop, one session per boot");
}
