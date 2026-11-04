/**
 * @file usb_console.c
 * @brief Console de texto pela USB para o teste de motores (comandos em usb_console.h).
 */
#include "usb_console.h"

#include "dshot.h"
#include "motors.h"
#include "sensor_log.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Maior linha aceita, em caracteres. */
#define LINE_MAX 64U
/** Palavras por linha. */
#define MAX_ARGS 5
/** Menor e maior prazo do watchdog, em ms. */
#define WATCHDOG_MIN_MS 200U
#define WATCHDOG_MAX_MS 5000U

/** Bytes digitados no PC, vindos da interrupção USB. */
static StreamBufferHandle_t rx;
/** Prazo do watchdog em ms (0 = desligado). */
static uint32_t watchdog_ms;
/** Instante da última linha recebida (tick). */
static TickType_t last_line_tick;

/** @brief Bytes da USB (interrupção): só copia para o stream buffer. */
void CDC_OnReceive(const uint8_t *data, uint32_t length) {
  if (!rx) return;
  BaseType_t wake = pdFALSE;
  (void)xStreamBufferSendFromISR(rx, data, length, &wake);
  portYIELD_FROM_ISR(wake);
}

/** @brief Escreve uma resposta formatada no log (até 60 caracteres). */
static void say(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void say(const char *fmt, ...) {
  char text[61];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);
  log_text(text);
}

/** @brief Para os motores. */
static void stop_all(const char *why) {
  motors_stop(why);
  say("PARADO: %s", why);
}

/** @brief Converte "1".."4" em máscara de um motor e "0" em todos; 0 se inválido. */
static uint8_t motor_mask(const char *s) {
  if (!s) return 0;
  const long m = strtol(s, NULL, 10);
  if (m == 0 && s[0] == '0') return 0x0F;
  return (m >= 1 && m <= 4) ? (uint8_t)(1U << (m - 1)) : 0;
}

/** @brief Converte por cento (texto) em décimos de por cento, limitado a 0..100%. */
static uint16_t pct_to_permille(const char *s, uint16_t fallback) {
  if (!s) return fallback;
  long p = strtol(s, NULL, 10);
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return (uint16_t)(p * 10);
}

/** @brief Lista os comandos. */
static void help(void) {
  log_text("arm | m <0-4> <pct> | d <0-4> <dshot 0|48-2047>");
  log_text("sweep <0-4> [max] [passo] [seg] | rate <pct/s>");
  log_text("beep <1-4> | dir <0-4> n|r | wd <ms> | ping");
  log_text("log on|off | status | stop (ou Enter vazio)");
}

/** @brief Estado da saída dos motores. */
static void status(void) {
  motors_status_t m;
  motors_get_status(&m);
  say("motores dono=%u rate=%lu%%/s sweep=%u wd=%lums", (unsigned)m.owner,
      (unsigned long)(m.bench_rate / 10U), m.sweeping, (unsigned long)watchdog_ms);
  say("acel %%=%u,%u,%u,%u", m.permille[0] / 10U, m.permille[1] / 10U, m.permille[2] / 10U,
      m.permille[3] / 10U);
  say("dshot=%u,%u,%u,%u", m.dshot[0], m.dshot[1], m.dshot[2], m.dshot[3]);
}

/** @brief Comandos de armamento e de alvo; devolve false se não for um deles. */
static bool drive_command(int argc, char **argv) {
  const char *c = argv[0];
  if (!strcmp(c, "arm")) {
    if (!CDC_HostDTR()) { say("abra a porta com DTR ligado"); return true; }
    if (!motors_arm(MOTORS_OWNER_BENCH)) {
      stop_all("arm com motor girando ou sentido em andamento");
      return true;
    }
    say("BANCADA ARMADA (sem helices!). Enter vazio para parar");
    return true;
  }
  if (!strcmp(c, "m")) {
    const uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    if (!mask || argc < 3) { stop_all("uso: m <0-4> <pct>"); return true; }
    const uint16_t v = pct_to_permille(argv[2], 0);
    if (!motors_bench_set(mask, v)) { say("arme antes (arm)"); return true; }
    say("alvo=%u%% mascara=0x%X", v / 10U, mask);
    return true;
  }
  if (!strcmp(c, "d")) {
    const uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    const long v = argc > 2 ? strtol(argv[2], NULL, 10) : -1;
    if (!mask || v < 0 || v > (long)DSHOT_THROTTLE_MAX || (v > 0 && v < (long)DSHOT_THROTTLE_MIN)) {
      stop_all("uso: d <0-4> <0|48-2047>");
      return true;
    }
    if (!motors_bench_set_dshot(mask, (uint16_t)v)) { say("arme antes (arm)"); return true; }
    say("dshot=%ld mascara=0x%X", v, mask);
    return true;
  }
  if (!strcmp(c, "sweep")) {
    const uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    if (!mask) { stop_all("uso: sweep <0-4> [max] [passo] [seg]"); return true; }
    const uint16_t max = pct_to_permille(argc > 2 ? argv[2] : NULL, MOTORS_BENCH_MAX_PERMILLE);
    const uint16_t step = pct_to_permille(argc > 3 ? argv[3] : NULL, 100U);
    const long secs = argc > 4 ? strtol(argv[4], NULL, 10) : 3;
    const uint32_t hold = (uint32_t)(secs < 1 ? 1 : (secs > 30 ? 30 : secs)) * 1000U;
    if (!motors_bench_sweep(mask, max, step, hold)) { say("arme antes (arm)"); return true; }
    say("sweep ate %u%% passo %u%% %lus", max / 10U, step / 10U, (unsigned long)(hold / 1000U));
    return true;
  }
  if (!strcmp(c, "rate")) {
    const long r = argc > 1 ? strtol(argv[1], NULL, 10) : 0;
    if (r < 1 || r > 100) { say("rate deve ser 1..100 %%/s"); return true; }
    motors_bench_rate((uint32_t)r * 10U);
    say("rate=%ld%%/s", r);
    return true;
  }
  return false;
}

/** @brief Comandos com a saída desarmada e utilitários; false se não for um deles. */
static bool utility_command(int argc, char **argv) {
  const char *c = argv[0];
  if (!strcmp(c, "beep")) {
    const uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    if (!mask) { say("uso: beep <1-4>"); return true; }
    if (!motors_beep(mask)) say("beep so desarmado");
    return true;
  }
  if (!strcmp(c, "dir")) {
    const uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    const bool known = argc > 2 && (argv[2][0] == 'n' || argv[2][0] == 'r');
    if (!mask || !known) { say("uso: dir <0-4> n|r"); return true; }
    if (!motors_set_direction(mask, argv[2][0] == 'r')) {
      say("dir so desarmado e com motores parados");
    } else {
      say("sentido %s gravando no ESC (mascara 0x%X)", argv[2][0] == 'r' ? "invertido" : "normal",
          mask);
    }
    return true;
  }
  if (!strcmp(c, "wd")) {
    const long ms = argc > 1 ? strtol(argv[1], NULL, 10) : -1;
    if (ms != 0 && (ms < (long)WATCHDOG_MIN_MS || ms > (long)WATCHDOG_MAX_MS)) {
      say("wd: 0 ou %u..%u ms", WATCHDOG_MIN_MS, WATCHDOG_MAX_MS);
      return true;
    }
    watchdog_ms = (uint32_t)ms;
    say("watchdog %lums", (unsigned long)watchdog_ms);
    return true;
  }
  if (!strcmp(c, "ping")) return true;
  return false;
}

/** @brief Interpreta uma linha digitada. */
static void command(char *line) {
  char *argv[MAX_ARGS] = {0};
  int argc = 0;
  for (char *t = strtok(line, " \t"); t && argc < MAX_ARGS; t = strtok(NULL, " \t")) {
    argv[argc++] = t;
  }
  if (argc == 0) { stop_all("Enter"); return; }
  const char *c = argv[0];
  if (!strcmp(c, "stop") || !strcmp(c, "disarm") || !strcmp(c, "s")) { stop_all(c); return; }
  if (!strcmp(c, "help") || !strcmp(c, "?")) { help(); return; }
  if (!strcmp(c, "status")) { status(); return; }
  if (!strcmp(c, "log")) {
    const bool on = argc > 1 && !strcmp(argv[1], "on");
    log_set_stream(on);
    say("log %s", on ? "on" : "off");
    return;
  }
  if (drive_command(argc, argv) || utility_command(argc, argv)) return;
  stop_all("comando desconhecido");
}

/** @brief Task do console: monta linhas e corta a bancada se a porta fechar ou o watchdog vencer. */
static void console_task(void *argument) {
  (void)argument;
  char line[LINE_MAX];
  size_t used = 0U;
  char previous = 0;
  last_line_tick = xTaskGetTickCount();
  log_text("console: SEM HELICES na bancada. digite help");
  for (;;) {
    char c;
    while (xStreamBufferReceive(rx, &c, 1, pdMS_TO_TICKS(20)) == 1) {
      if (c == '\n' && previous == '\r') { previous = c; continue; }
      previous = c;
      if (c == '\r' || c == '\n') {
        last_line_tick = xTaskGetTickCount();
        line[used] = '\0';
        command(line);
        used = 0U;
      } else if (used < sizeof(line) - 1U) {
        line[used++] = c;
      }
    }
    if (motors_owner() == MOTORS_OWNER_BENCH) {
      if (!CDC_HostDTR()) {
        motors_stop("porta USB fechada");
      } else if (watchdog_ms != 0U &&
                 (xTaskGetTickCount() - last_line_tick) > pdMS_TO_TICKS(watchdog_ms)) {
        motors_stop("watchdog: PC parou de falar");
      }
    }
  }
}

/** Controle da task do console (alocação estática). */
static StaticTask_t console_tcb;
/** Pilha da task do console, em words. */
static StackType_t console_stack[512];

bool usb_console_start(void) {
  rx = xStreamBufferCreate(256, 1);
  return rx && xTaskCreateStatic(console_task, "Console",
                                 sizeof(console_stack) / sizeof(console_stack[0]), NULL,
                                 tskIDLE_PRIORITY + 2, console_stack, &console_tcb) != NULL;
}
