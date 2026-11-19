/**
 * @file test_motor_arbiter.c
 * @brief Regras de segurança da saída dos motores (actuators/motor_arbiter.h).
 */
#include "actuators/motor_arbiter.h"
#include "dshot.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>

/** Roda @p n quadros de 1 ms a partir de @p *now; devolve o último. */
static motor_arb_frame_t run(motor_arb_t *a, uint32_t *now, uint32_t n) {
  motor_arb_frame_t f = {0};
  for (uint32_t i = 0U; i < n; ++i) {
    f = motor_arb_frame(a, (*now)++, NULL);
  }
  return f;
}

/** @brief Igual a run(), mas renova o keepalive a cada 20 ms como o console. */
static motor_arb_frame_t run_alive(motor_arb_t *a, uint32_t *now, uint32_t n) {
  motor_arb_frame_t f = {0};
  for (uint32_t i = 0U; i < n; ++i) {
    if ((*now % 20U) == 0U) {
      motor_arb_bench_keepalive(a, *now);
    }
    f = motor_arb_frame(a, (*now)++, NULL);
  }
  return f;
}

static int all_values(const motor_arb_frame_t *f, uint16_t v) {
  for (uint32_t i = 0U; i < MOTOR_ARB_COUNT; ++i) {
    if (f->value[i] != v) {
      return 0;
    }
  }
  return 1;
}

/** @brief Desarmado, todo quadro é DShot 0 e comandos de bancada são recusados. */
static void test_disarmed_sends_zero(void) {
  motor_arb_t a;
  uint32_t now = 1000U;
  motor_arb_init(&a, now);
  assert(!motor_arb_bench_set(&a, 0x0FU, 300U));
  for (uint32_t i = 0U; i < 2000U; ++i) {
    const motor_arb_frame_t f = motor_arb_frame(&a, now++, NULL);
    assert(all_values(&f, 0U) && !f.telemetry && f.stopped == NULL);
  }
}

/** @brief Ao ligar o limite é 40 %: pedir 100 % para em 40 %. */
static void test_default_cap(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(a.cap == MOTOR_ARB_BENCH_DEFAULT_CAP);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
  motor_arb_bench_rate(&a, 1000U);
  assert(motor_arb_bench_set(&a, 0x0FU, 1000U));
  const motor_arb_frame_t f = run_alive(&a, &now, 2000U);
  assert(a.sent_permille[0] == MOTOR_ARB_BENCH_DEFAULT_CAP);
  assert(f.value[0] == dshot_from_permille(MOTOR_ARB_BENCH_DEFAULT_CAP));
}

/** @brief O limite ajustável vai até 100 % e não passa disso. */
static void test_hard_cap(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(motor_arb_bench_cap(&a, 2000U) == MOTOR_ARB_BENCH_HARD_MAX);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
  motor_arb_bench_rate(&a, 1000U);
  assert(motor_arb_bench_set(&a, 0x0FU, 1500U));
  const motor_arb_frame_t f = run_alive(&a, &now, 2000U);
  assert(a.sent_permille[0] == MOTOR_ARB_BENCH_HARD_MAX);
  assert(f.value[0] == DSHOT_THROTTLE_MAX);
}

/** @brief Abaixar o limite abaixa alvos já dados e a varredura. */
static void test_lower_cap_clamps(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
  assert(motor_arb_bench_set(&a, 0x01U, 300U));
  assert(motor_arb_bench_cap(&a, 150U) == 150U);
  assert(a.target[0] == 150U);
  assert(motor_arb_bench_set(&a, 0x02U, 400U));
  assert(a.target[1] == 150U);
  assert(motor_arb_bench_sweep(&a, 0x0FU, 400U, 100U, 10U, now));
  assert(a.sweep.max == 150U);
}

/** @brief A máscara escolhe qualquer conjunto de motores. */
static void test_group_mask(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
  assert(motor_arb_bench_set(&a, 0x05U, 200U)); /* M1 e M3 */
  assert(a.target[0] == 200U && a.target[1] == 0U && a.target[2] == 200U && a.target[3] == 0U);
}

/** @brief A rampa sobe no máximo `rate` por segundo. */
static void test_ramp(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
  motor_arb_bench_rate(&a, 200U); /* 20 %/s */
  assert(motor_arb_bench_set(&a, 0x01U, 300U));
  (void)run_alive(&a, &now, 500U);
  assert(a.sent_permille[0] == 100U); /* 0,5 s a 20 %/s = 10 % */
  (void)run_alive(&a, &now, 2000U);
  assert(a.sent_permille[0] == 300U);
}

/** @brief Sem keepalive por 300 ms a bancada desarma e volta a mandar zero. */
static void test_bench_keepalive(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
  assert(motor_arb_bench_set(&a, 0x0FU, 200U));
  (void)run_alive(&a, &now, 1000U);
  assert(a.owner == MOTOR_ARB_OWNER_BENCH);
  const uint32_t last = now - 1U - ((now - 1U) % 20U);
  motor_arb_frame_t f = {0};
  const char *stopped = NULL;
  while (now <= last + MOTOR_ARB_BENCH_KEEPALIVE_MS) {
    f = motor_arb_frame(&a, now++, NULL);
    if (f.stopped != NULL) {
      stopped = f.stopped;
    }
  }
  assert(stopped != NULL);
  assert(a.owner == MOTOR_ARB_OWNER_NONE && all_values(&f, 0U));
  assert(a.target[0] == 0U && a.current[0] == 0U);
}

/** @brief O pedido de parada zera no mesmo quadro. */
static void test_stop(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
  assert(motor_arb_bench_set(&a, 0x0FU, 300U));
  (void)run_alive(&a, &now, 500U);
  const motor_arb_frame_t f = motor_arb_frame(&a, now++, "teste");
  assert(f.stopped != NULL && all_values(&f, 0U) && a.owner == MOTOR_ARB_OWNER_NONE);
}

/** @brief Armar exige tudo em zero; um segundo dono é recusado. */
static void test_arm_rules(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(!motor_arb_arm(&a, MOTOR_ARB_OWNER_NONE, now));
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
  motor_arb_disarm(&a);
  assert(motor_arb_beep(&a, 0x01U));
  assert(!motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now)); /* comando pendente */
  (void)run(&a, &now, MOTOR_ARB_CMD_FRAMES);
  assert(!motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now)); /* último quadro ainda era beep */
  (void)run(&a, &now, 1U); /* zero antes de permitir novo armamento */
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
}

/** @brief Sentido: 7/8 por motor com o bit de telemetria, pausa, e 12 se gravar. */
static void test_direction(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(motor_arb_set_direction(&a, 0x09U, 0x01U, true)); /* M1 invertido, M4 normal */
  for (uint32_t i = 0U; i < MOTOR_ARB_CMD_FRAMES; ++i) {
    const motor_arb_frame_t f = motor_arb_frame(&a, now++, NULL);
    assert(f.telemetry);
    assert(f.value[0] == MOTOR_ARB_CMD_DIR_REVERSED && f.value[1] == 0U && f.value[2] == 0U &&
           f.value[3] == MOTOR_ARB_CMD_DIR_NORMAL);
  }
  for (uint32_t i = 0U; i < MOTOR_ARB_DIR_GAP_FRAMES; ++i) {
    const motor_arb_frame_t f = motor_arb_frame(&a, now++, NULL);
    assert(!f.telemetry && all_values(&f, 0U));
  }
  for (uint32_t i = 0U; i < MOTOR_ARB_CMD_FRAMES; ++i) {
    const motor_arb_frame_t f = motor_arb_frame(&a, now++, NULL);
    assert(f.telemetry && f.value[0] == MOTOR_ARB_CMD_SAVE && f.value[1] == 0U &&
           f.value[3] == MOTOR_ARB_CMD_SAVE);
  }
  (void)run(&a, &now, MOTOR_ARB_SAVE_GAP_FRAMES);
  assert(a.queue_len == 0U);
  assert(a.reversed == 0x01U && a.reversed_known == 0U); /* sem readback do ESC */
  const motor_arb_frame_t f = run(&a, &now, 100U);
  assert(all_values(&f, 0U) && !f.telemetry);
}

/** @brief Sentido é recusado armado; parar no meio descarta a fila. */
static void test_direction_rules(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(!motor_arb_set_direction(&a, 0x00U, 0x00U, false));
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
  assert(!motor_arb_set_direction(&a, 0x01U, 0x01U, false));
  motor_arb_disarm(&a);
  assert(motor_arb_set_direction(&a, 0x01U, 0x01U, true));
  (void)run(&a, &now, 3U);
  const motor_arb_frame_t f = motor_arb_frame(&a, now++, "stop");
  assert(all_values(&f, 0U) && a.queue_len == 0U);
}

/** @brief Limite reduzido restringe a próxima saída, sem aguardar rampa. */
static void test_cap_restricts_output_immediately(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  (void)motor_arb_bench_cap(&a, 1000U);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
  motor_arb_bench_rate(&a, 1000U);
  assert(motor_arb_bench_set(&a, 1U, 1000U));
  (void)run_alive(&a, &now, 1100U);
  assert(a.sent_permille[0] == 1000U);
  (void)motor_arb_bench_cap(&a, 100U);
  (void)run_alive(&a, &now, 1U);
  assert(a.sent_permille[0] == 100U);
  (void)motor_arb_bench_cap(&a, 0U);
  const motor_arb_frame_t f = run_alive(&a, &now, 1U);
  assert(all_values(&f, 0U));
}

static bool accept_frame(const uint16_t value[MOTOR_ARB_COUNT], bool telemetry) {
  (void)value;
  (void)telemetry;
  return true;
}

static bool reject_frame(const uint16_t value[MOTOR_ARB_COUNT], bool telemetry) {
  (void)value;
  (void)telemetry;
  return false;
}

/** @brief Falha DMA não vira atuação fictícia e impede retomar saída antiga. */
static void test_transport_failure(void) {
  motor_arb_t a;
  motor_arb_init(&a, 0U);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, 0U));
  assert(motor_arb_bench_set(&a, 1U, 200U));
  for (uint32_t t = 0U; t < 100U; ++t) {
    motor_arb_bench_keepalive(&a, t);
    const motor_arb_frame_t f = motor_arb_submit(&a, t, NULL, accept_frame);
    assert(f.accepted);
  }
  const uint16_t last = a.sent_dshot[0];
  assert(last != 0U);
  motor_arb_frame_t f = motor_arb_submit(&a, 100U, NULL, reject_frame);
  assert(!f.accepted && f.stopped != NULL);
  assert(a.owner == MOTOR_ARB_OWNER_NONE && a.sent_dshot[0] == last);
  assert(a.target[0] == 0U && a.tx_failures == 1U);
  assert(!motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, 100U)); /* zero ainda não aceito */
  f = motor_arb_submit(&a, 101U, NULL, accept_frame);
  assert(f.accepted && all_values(&f, 0U) && a.sent_dshot[0] == 0U);
}

/** @brief Inversão não enviada ou cancelada nunca aparece como confirmada. */
static void test_direction_transport(void) {
  motor_arb_t a;
  motor_arb_init(&a, 0U);
  assert(motor_arb_set_direction(&a, 1U, 1U, true));
  assert(a.reversed_known == 0U && a.direction_transmitted == 0U);
  motor_arb_frame_t f = motor_arb_submit(&a, 0U, NULL, reject_frame);
  assert(!f.accepted && a.direction_transmitted == 0U && a.queue_len == 0U);
  assert(motor_arb_set_direction(&a, 1U, 1U, false));
  f = motor_arb_submit(&a, 1U, "cancelar", accept_frame);
  assert(f.accepted && all_values(&f, 0U));
  assert(a.reversed_known == 0U && a.direction_transmitted == 0U);
  assert(motor_arb_set_direction(&a, 1U, 1U, false));
  assert(!motor_arb_set_direction(&a, 1U, 0U, false)); /* sem pedidos concorrentes */
  for (uint32_t t = 2U; t < 2U + MOTOR_ARB_CMD_FRAMES; ++t) {
    f = motor_arb_submit(&a, t, NULL, accept_frame);
    assert(f.accepted);
  }
  assert(a.direction_transmitted == 1U && a.reversed_known == 0U);
}

/** @brief Timeout é modular e não depende do consumidor renovar a saída. */
static void test_keepalive_wraparound(void) {
  motor_arb_t a;
  const uint32_t start = UINT32_MAX - 100U;
  motor_arb_init(&a, start);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, start));
  motor_arb_frame_t f = motor_arb_frame(&a, start + 299U, NULL);
  assert(f.stopped == NULL && a.owner == MOTOR_ARB_OWNER_BENCH);
  f = motor_arb_frame(&a, start + 300U, NULL);
  assert(f.stopped != NULL && all_values(&f, 0U));
}

/** @brief Dono controle: escrita direta, limite próprio, prazos de 100 ms e 300 ms. */
static void test_control_owner(void) {
  motor_arb_t a;
  uint32_t now = 1000U;
  motor_arb_init(&a, now);
  const uint16_t want[4] = {500U, 600U, 990U, 0U};
  assert(!motor_arb_control_set(&a, want, now)); /* desarmado */
  assert(a.ctrl_cap == MOTOR_ARB_CONTROL_DEFAULT_CAP);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_CONTROL, now));
  assert(!motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now)); /* já há dono */
  assert(!motor_arb_bench_set(&a, 1U, 100U));              /* não é a bancada */
  motor_arb_bench_keepalive(&a, now);
  assert(motor_arb_control_set(&a, want, now));
  motor_arb_frame_t f = motor_arb_frame(&a, now++, NULL);
  /* sem rampa: vale no primeiro quadro; 990 é limitado a 950 (95 %) */
  assert(a.sent_permille[0] == 500U && a.sent_permille[1] == 600U);
  assert(a.sent_permille[2] == MOTOR_ARB_CONTROL_DEFAULT_CAP && a.sent_permille[3] == 0U);
  assert(f.value[0] == dshot_from_permille(500U));
  /* limite do controle ajustável até 100 % */
  assert(motor_arb_control_cap(&a, 2000U) == MOTOR_ARB_BENCH_HARD_MAX);
  assert(motor_arb_control_set(&a, want, now));
  (void)motor_arb_frame(&a, now++, NULL);
  assert(a.sent_permille[2] == 990U);
  assert(motor_arb_control_cap(&a, 550U) == 550U);
  assert(a.target[2] == 550U && a.target[1] == 550U);
  /* sem atualização do controle por 100 ms desarma, mesmo com keepalive */
  const uint32_t last = now;
  while (now < (last + MOTOR_ARB_CONTROL_TIMEOUT_MS + 5U)) {
    motor_arb_bench_keepalive(&a, now);
    f = motor_arb_frame(&a, now++, NULL);
  }
  assert(a.owner == MOTOR_ARB_OWNER_NONE && all_values(&f, 0U));
  assert(!motor_arb_control_set(&a, want, now));
  /* sem keepalive da bancada (HEARTBEAT) por 300 ms também desarma */
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_CONTROL, now));
  const uint32_t start = now;
  while (now < (start + MOTOR_ARB_BENCH_KEEPALIVE_MS + 5U)) {
    (void)motor_arb_control_set(&a, want, now);
    f = motor_arb_frame(&a, now++, NULL);
  }
  assert(a.owner == MOTOR_ARB_OWNER_NONE && all_values(&f, 0U));
  /* STOP zera na hora */
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_CONTROL, now));
  motor_arb_bench_keepalive(&a, now);
  assert(motor_arb_control_set(&a, want, now));
  f = motor_arb_frame(&a, now++, "STOP");
  assert(f.stopped != NULL && all_values(&f, 0U) && a.owner == MOTOR_ARB_OWNER_NONE);
}

int main(void) {
  const uint16_t values[4] = {0U, 48U, 1000U, 2047U};
  for (uint8_t mask = 0U; mask < 16U; ++mask) {
    uint16_t packets[4];
    dshot_packets(values, mask, packets);
    for (uint32_t i = 0U; i < 4U; ++i) {
      const bool telemetry = ((mask >> i) & 1U) != 0U;
      assert(packets[i] == dshot_packet(values[i], telemetry));
      assert(((packets[i] >> 4U) & 1U) == (telemetry ? 1U : 0U));
    }
  }
  test_cap_restricts_output_immediately();
  test_transport_failure();
  test_direction_transport();
  test_keepalive_wraparound();
  test_disarmed_sends_zero();
  test_default_cap();
  test_hard_cap();
  test_lower_cap_clamps();
  test_group_mask();
  test_ramp();
  test_bench_keepalive();
  test_stop();
  test_arm_rules();
  test_direction();
  test_direction_rules();
  test_control_owner();
  puts("test_motor_arbiter ok");
  return 0;
}
