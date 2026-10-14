/**
 * @file motor_test.c
 * @brief Teste de bancada dos motores por comandos USB (contrato em motor_test.h).
 */
#include "motor_test.h"
#include "dshot.h"
#include "sensor_log.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Período da task e dos quadros DShot, em ms (1 kHz). */
#define PERIOD_MS 1U
/** Uma linha MOT a cada N quadros (100 Hz). */
#define LOG_EVERY 10U
/** Rampa padrão, em décimos de por cento por segundo (20 %/s). */
#define DEFAULT_RATE 200U
/** Quadros com o comando de beep (o ESC precisa recebê-lo repetido). */
#define BEEP_FRAMES 10U

/** Bytes digitados no PC, vindos da interrupção USB. */
static StreamBufferHandle_t rx;

static bool armed;
/** Alvo de cada motor, em décimos de por cento. */
static uint16_t target[4];
/** Valor atual de cada motor, em milésimos de décimo de por cento (rampa fina). */
static uint32_t current[4];
/** Rampa, em décimos de por cento por segundo. */
static uint32_t rate = DEFAULT_RATE;

/** Estado da varredura automática (comando sweep). */
static struct {
  bool active;
  uint8_t mask;       /* bit 0 = motor 1 */
  uint16_t max, step; /* décimos de por cento */
  uint32_t hold_ms;
  uint16_t level;
  bool down;
  uint32_t next_ms;
} sweep;

/** Motor do beep (máscara) e quadros restantes. */
static uint8_t beep_mask;
static uint32_t beep_left;

/** @brief Bytes da USB (interrupção): só copia para o stream buffer. */
void CDC_OnReceive(const uint8_t *data, uint32_t length) {
  if (!rx) return;
  BaseType_t wake = pdFALSE;
  (void)xStreamBufferSendFromISR(rx, data, length, &wake);
  portYIELD_FROM_ISR(wake);
}

/** @brief Escreve uma resposta formatada no log. */
static void say(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void say(const char *fmt, ...) {
  char text[61];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);
  log_text(text);
}

/** @brief Zera tudo na hora e desarma. */
static void stop(const char *why) {
  armed = false;
  sweep.active = false;
  for (int i = 0; i < 4; ++i) { target[i] = 0; current[i] = 0; }
  say("PARADO: %s", why);
}

/** @brief Converte "1".."4" em máscara de um motor e "0" em todos; 0 se inválido. */
static uint8_t motor_mask(const char *s) {
  if (!s) return 0;
  long m = strtol(s, NULL, 10);
  if (m == 0 && s[0] == '0') return 0x0F;
  return (m >= 1 && m <= 4) ? (uint8_t)(1U << (m - 1)) : 0;
}

/** @brief Converte por cento (texto) em décimos de por cento, limitado. */
static uint16_t pct_to_permille(const char *s, uint16_t fallback) {
  if (!s) return fallback;
  long p = strtol(s, NULL, 10);
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  uint32_t pm = (uint32_t)p * 10U;
  return (uint16_t)(pm > MOTOR_TEST_MAX_PERMILLE ? MOTOR_TEST_MAX_PERMILLE : pm);
}

static bool all_zero(void) {
  for (int i = 0; i < 4; ++i) if (target[i] || current[i]) return false;
  return true;
}

static void set_targets(uint8_t mask, uint16_t value) {
  for (int i = 0; i < 4; ++i) if (mask & (1U << i)) target[i] = value;
}

static void help(void) {
  log_text("arm | m <0-4> <pct> | sweep <0-4> [max] [passo] [seg]");
  log_text("rate <pct/s> | beep <1-4> | log on|off | status");
  log_text("stop, disarm ou Enter vazio: para e desarma");
}

static void status(void) {
  say("armed=%u rate=%lu%%/s sweep=%u", armed, (unsigned long)(rate / 10U), sweep.active);
  say("alvo %%=%u,%u,%u,%u", target[0] / 10U, target[1] / 10U, target[2] / 10U, target[3] / 10U);
}

/** @brief Interpreta uma linha digitada. */
static void command(char *line) {
  char *argv[5] = {0};
  int argc = 0;
  for (char *t = strtok(line, " \t"); t && argc < 5; t = strtok(NULL, " \t")) argv[argc++] = t;
  if (argc == 0) { stop("Enter"); return; }
  const char *c = argv[0];

  if (!strcmp(c, "stop") || !strcmp(c, "disarm") || !strcmp(c, "s")) { stop(c); return; }
  if (!strcmp(c, "help") || !strcmp(c, "?")) { help(); return; }
  if (!strcmp(c, "status")) { status(); return; }
  if (!strcmp(c, "log")) {
    bool on = argc > 1 && !strcmp(argv[1], "on");
    log_set_stream(on);
    say("log %s", on ? "on" : "off");
    return;
  }
  if (!strcmp(c, "rate")) {
    long r = argc > 1 ? strtol(argv[1], NULL, 10) : 0;
    if (r < 1 || r > 100) { say("rate deve ser 1..100 %%/s"); return; }
    rate = (uint32_t)r * 10U;
    say("rate=%ld%%/s", r);
    return;
  }
  if (!strcmp(c, "arm")) {
    if (!CDC_HostDTR()) { say("abra a porta com DTR ligado"); return; }
    if (!all_zero()) { stop("arm com motor girando"); return; }
    armed = true;
    say("ARMADO (sem helices!). Enter vazio para parar");
    return;
  }
  if (!strcmp(c, "beep")) {
    uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    if (armed || !all_zero()) { say("beep so desarmado"); return; }
    if (!mask) { say("uso: beep <1-4>"); return; }
    beep_mask = mask;
    beep_left = BEEP_FRAMES;
    return;
  }
  if (!strcmp(c, "m")) {
    uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    if (!armed) { say("arme antes (arm)"); return; }
    if (!mask || argc < 3) { stop("uso: m <0-4> <pct>"); return; }
    sweep.active = false;
    uint16_t v = pct_to_permille(argv[2], 0);
    set_targets(mask, v);
    say("alvo=%u%% mascara=0x%X", v / 10U, mask);
    return;
  }
  if (!strcmp(c, "sweep")) {
    uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    if (!armed) { say("arme antes (arm)"); return; }
    if (!mask) { stop("uso: sweep <0-4> [max] [passo] [seg]"); return; }
    sweep.mask = mask;
    sweep.max = pct_to_permille(argc > 2 ? argv[2] : NULL, MOTOR_TEST_MAX_PERMILLE);
    sweep.step = pct_to_permille(argc > 3 ? argv[3] : NULL, 100U);
    if (sweep.step == 0) sweep.step = 100U;
    long secs = argc > 4 ? strtol(argv[4], NULL, 10) : 3;
    sweep.hold_ms = (uint32_t)(secs < 1 ? 1 : (secs > 30 ? 30 : secs)) * 1000U;
    sweep.level = 0;
    sweep.down = false;
    sweep.next_ms = HAL_GetTick();
    sweep.active = true;
    set_targets(0x0F, 0);
    say("sweep ate %u%% passo %u%% %lus", sweep.max / 10U, sweep.step / 10U,
        (unsigned long)(sweep.hold_ms / 1000U));
    return;
  }
  stop("comando desconhecido");
}

/** @brief Avança a varredura: um degrau a cada hold_ms, sobe e depois desce. */
static void sweep_step(uint32_t now) {
  if (!sweep.active || (int32_t)(now - sweep.next_ms) < 0) return;
  if (!sweep.down) {
    if (sweep.level >= sweep.max) {
      sweep.down = true;
    } else {
      uint32_t next = (uint32_t)sweep.level + sweep.step;
      sweep.level = (uint16_t)(next > sweep.max ? sweep.max : next);
    }
  }
  if (sweep.down) {
    if (sweep.level == 0) {
      sweep.active = false;
      say("sweep terminou");
      return;
    }
    sweep.level = (uint16_t)(sweep.level > sweep.step ? sweep.level - sweep.step : 0);
  }
  set_targets(sweep.mask, sweep.level);
  say("sweep degrau %u%%", sweep.level / 10U);
  /* O tempo de cada degrau conta a partir do fim da rampa até ele. */
  sweep.next_ms = now + sweep.hold_ms + (uint32_t)sweep.step * 1000U / rate;
}

/** @brief Task dos motores: lê comandos, aplica a rampa e envia um quadro por ms. */
static void motor_task(void *argument) {
  (void)argument;
  char line[64];
  size_t used = 0;
  char previous = 0;
  uint32_t frames = 0;
  TickType_t wake = xTaskGetTickCount();
  log_text("teste de motores: SEM HELICES. digite help");

  for (;;) {
    char c;
    while (xStreamBufferReceive(rx, &c, 1, 0) == 1) {
      if (c == '\n' && previous == '\r') { previous = c; continue; }
      previous = c;
      if (c == '\r' || c == '\n') {
        line[used] = '\0';
        command(line);
        used = 0;
      } else if (used < sizeof(line) - 1U) {
        line[used++] = c;
      }
    }

    if (armed && !CDC_HostDTR()) stop("porta USB fechada");
    uint32_t now = HAL_GetTick();
    if (armed) sweep_step(now);

    uint16_t value[4] = {0, 0, 0, 0};
    bool telemetry = false;
    if (armed) {
      for (int i = 0; i < 4; ++i) {
        uint32_t goal = (uint32_t)target[i] * 1000U;
        if (current[i] < goal) current[i] = (goal - current[i] > rate) ? current[i] + rate : goal;
        else if (current[i] > goal) current[i] = (current[i] - goal > rate) ? current[i] - rate : goal;
        value[i] = dshot_from_permille((uint16_t)(current[i] / 1000U));
      }
    } else if (beep_left) {
      --beep_left;
      telemetry = true;  /* comandos especiais exigem o bit de telemetria */
      for (int i = 0; i < 4; ++i) if (beep_mask & (1U << i)) value[i] = DSHOT_CMD_BEEP1;
    }
    (void)dshot_write(value, telemetry);

    if (++frames >= LOG_EVERY) {
      frames = 0;
      log_motor_t m;
      m.armed = armed;
      for (int i = 0; i < 4; ++i) {
        m.throttle_permille[i] = (uint16_t)(current[i] / 1000U);
        m.dshot[i] = value[i];
      }
      log_motor(&m);
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));
  }
}

static StaticTask_t motor_tcb;
static StackType_t motor_stack[512];

bool motor_test_start(void) {
  dshot_init();
  rx = xStreamBufferCreate(256, 1);
  return rx && xTaskCreateStatic(motor_task, "Motors", sizeof(motor_stack) / sizeof(motor_stack[0]),
                                 NULL, tskIDLE_PRIORITY + 4, motor_stack, &motor_tcb) != NULL;
}
