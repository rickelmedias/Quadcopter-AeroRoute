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

/** @brief O alvo nunca passa do limite fixo de 40 %, nem pedindo 100 %. */
static void test_hard_cap(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(motor_arb_bench_cap(&a, 1000U) == MOTOR_ARB_BENCH_HARD_MAX);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now));
  motor_arb_bench_rate(&a, 1000U);
  assert(motor_arb_bench_set(&a, 0x0FU, 1000U));
  const motor_arb_frame_t f = run_alive(&a, &now, 2000U);
  assert(a.sent_permille[0] == MOTOR_ARB_BENCH_HARD_MAX);
  assert(f.value[0] == dshot_from_permille(MOTOR_ARB_BENCH_HARD_MAX));
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

/** @brief O voo desarma sem atualização por 100 ms. */
static void test_flight_timeout(void) {
  motor_arb_t a;
  uint32_t now = 0U;
  motor_arb_init(&a, now);
  assert(motor_arb_arm(&a, MOTOR_ARB_OWNER_FLIGHT, now));
  const float t[MOTOR_ARB_COUNT] = {0.5F, 0.5F, 0.5F, 0.5F};
  motor_arb_set_flight(&a, t, now);
  motor_arb_frame_t f = run(&a, &now, MOTOR_ARB_FLIGHT_TIMEOUT_MS);
  assert(a.owner == MOTOR_ARB_OWNER_FLIGHT && f.value[0] > DSHOT_THROTTLE_MIN);
  f = run(&a, &now, 1U);
  assert(a.owner == MOTOR_ARB_OWNER_NONE && f.stopped != NULL && all_values(&f, 0U));
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
  assert(!motor_arb_arm(&a, MOTOR_ARB_OWNER_FLIGHT, now));
  motor_arb_disarm(&a);
  assert(motor_arb_beep(&a, 0x01U));
  assert(!motor_arb_arm(&a, MOTOR_ARB_OWNER_BENCH, now)); /* comando pendente */
  (void)run(&a, &now, MOTOR_ARB_CMD_FRAMES);
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
  assert(a.reversed == 0x01U && a.reversed_known == 0x09U);
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

int main(void) {
  test_disarmed_sends_zero();
  test_hard_cap();
  test_lower_cap_clamps();
  test_group_mask();
  test_ramp();
  test_bench_keepalive();
  test_flight_timeout();
  test_stop();
  test_arm_rules();
  test_direction();
  test_direction_rules();
  puts("test_motor_arbiter ok");
  return 0;
}
