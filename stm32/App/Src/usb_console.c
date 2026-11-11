/**
 * @file usb_console.c
 * @brief Console de texto pela USB (comandos em usb_console.h).
 */
#include "usb_console.h"

#include "control/ctrl_gains.h"
#include "control/ctrl_params.h"
#include "control/flight_ctrl.h"
#include "control/lqg_tuning.h"
#include "motors.h"
#include "power.h"
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
#define LINE_MAX       64U
/** Palavras por linha (`lq set <matriz> <linha> <coluna> <valor>` usa 6). */
#define MAX_ARGS       6
/** Tempo máximo de espera pelo voo armar antes de decolar, em ms. */
#define ARM_WAIT_MS    200U
/** Tempo máximo de um trecho pedido pelo console, em ms. */
#define LEG_TIMEOUT_MS 30000U
/** Graus para radianos. */
#define DEG_TO_RAD     0.017453293F

/** Menor e maior prazo do watchdog, em ms. */
#define WATCHDOG_MIN_MS 200U
#define WATCHDOG_MAX_MS 5000U

/** Bytes digitados no PC, vindos da interrupção USB. */
static StreamBufferHandle_t rx;
/** Prazo do watchdog em ms (0 = desligado). */
static uint32_t watchdog_ms;
/** Instante da última linha recebida (tick). */
static TickType_t last_line_tick;
/** O watchdog já agiu e ainda não chegou linha nova. */
static bool watchdog_fired;

/** @brief Bytes da USB (interrupção): só copia para o stream buffer. */
void CDC_OnReceive(const uint8_t *data, uint32_t length) {
  if (!rx)
    return;
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

/** @brief Valor em milésimos, com sinal, para imprimir sem %f (newlib-nano). */
static long milli(float v) {
  return (long)(v * 1000.0F + (v >= 0.0F ? 0.5F : -0.5F));
}

/** @brief Para tudo: motores e voo. */
static void stop_all(const char *why) {
  motors_stop(why);
  flight_ctrl_disarm(why);
  say("PARADO: %s", why);
}

/** @brief Converte "1".."4" em máscara de um motor e "0" em todos; 0 se inválido. */
static uint8_t motor_mask(const char *s) {
  if (!s)
    return 0;
  const long m = strtol(s, NULL, 10);
  if (m == 0 && s[0] == '0')
    return 0x0F;
  return (m >= 1 && m <= 4) ? (uint8_t)(1U << (m - 1)) : 0;
}

/** @brief Converte por cento (texto) em décimos de por cento, limitado a 0..100%. */
static uint16_t pct_to_permille(const char *s, uint16_t fallback) {
  if (!s)
    return fallback;
  long p = strtol(s, NULL, 10);
  if (p < 0)
    p = 0;
  if (p > 100)
    p = 100;
  return (uint16_t)(p * 10);
}

/** @brief Imprime um parâmetro: nome, valor (x1000) e ajuda. */
static void print_param(size_t index) {
  const ctrl_param_desc_t *d = ctrl_params_desc(index);
  say("%s=%ld/1000 %s", d->name, milli(ctrl_params_get(index)), d->help);
}

/* ---- Bancada ----------------------------------------------------------- */

/** @brief Lista os comandos. */
static void help(void) {
  log_text("bancada: arm | m <0-4> <pct> | sweep <0-4> [max] [passo] [seg]");
  log_text("rate <pct/s> | beep <1-4> | log on|off | status");
  log_text("cap <pct>: limite da bancada (max 40) | dir <0-4> n|r [save]: sentido");
  log_text("voo: hover [h] | dry [h] | land | hold | lqg");
  log_text("goto <x> <y> <z> | move <dx> <dy> [dz] | yaw <graus>");
  log_text("sp <x> <y> <altura> <guinada graus>: setpoint (m, m, m)");
  log_text("xyzero: o ponto atual vira o (0,0)");
  log_text("param list|get N|set N V | gains [default]");
  log_text("lq [default|begin] | lq set kd|kfd L C V | lq commit <crc>: Kd/Kfd (Q)");
  log_text("wd <ms>: sem linha por <ms> pousa (bancada: para); ping");
  log_text("stop, disarm ou Enter vazio: para tudo");
}

/** @brief Estado da saída dos motores. */
static void status(void) {
  motors_status_t m;
  motors_get_status(&m);
  say("motores dono=%u rate=%lu%%/s sweep=%u", (unsigned)m.owner,
      (unsigned long)(m.bench_rate / 10U), m.sweeping);
  say("acel %%=%u,%u,%u,%u", m.permille[0] / 10U, m.permille[1] / 10U, m.permille[2] / 10U,
      m.permille[3] / 10U);
  power_sample_t p;
  power_get(&p);
  if (p.voltage_valid || p.current_valid) {
    say("bateria %ld mV %ld mA", p.voltage_valid ? milli(p.voltage_v) : -1L,
        p.current_valid ? milli(p.current_a) : -1L);
  } else {
    say("bateria: leitura desligada (power.h)");
  }
}

/** @brief Comandos de bancada; devolve false se não for um deles. */
static bool bench_command(int argc, char **argv) {
  const char *c = argv[0];
  if (!strcmp(c, "arm")) {
    if (!CDC_HostDTR()) {
      say("abra a porta com DTR ligado");
      return true;
    }
    if (flight_ctrl_is_armed() || !motors_arm(MOTORS_OWNER_BENCH)) {
      stop_all("arm com motor girando ou voo armado");
      return true;
    }
    say("BANCADA ARMADA (sem helices!). Enter vazio para parar");
    return true;
  }
  if (!strcmp(c, "rate")) {
    const long r = argc > 1 ? strtol(argv[1], NULL, 10) : 0;
    if (r < 1 || r > 100) {
      say("rate deve ser 1..100 %%/s");
      return true;
    }
    motors_bench_rate((uint32_t)r * 10U);
    say("rate=%ld%%/s", r);
    return true;
  }
  if (!strcmp(c, "cap")) {
    if (argc < 2) {
      say("uso: cap <pct>");
      return true;
    }
    const uint16_t cap = motors_bench_cap(pct_to_permille(argv[1], 0U));
    say("cap=%u%%", cap / 10U);
    return true;
  }
  if (!strcmp(c, "dir")) {
    const uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    const char *way = argc > 2 ? argv[2] : "";
    if (!mask || (strcmp(way, "n") != 0 && strcmp(way, "r") != 0)) {
      say("uso: dir <0-4> n|r [save]");
      return true;
    }
    const bool save = argc > 3 && !strcmp(argv[3], "save");
    const uint8_t reversed = !strcmp(way, "r") ? mask : 0U;
    if (!motors_set_direction(mask, reversed, save)) {
      say("dir so desarmado e parado");
      return true;
    }
    say("dir mascara=0x%X %s%s", mask, !strcmp(way, "r") ? "invertido" : "normal",
        save ? " (gravado no ESC)" : "");
    return true;
  }
  if (!strcmp(c, "beep")) {
    const uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    if (!mask) {
      say("uso: beep <1-4>");
      return true;
    }
    if (!motors_beep(mask))
      say("beep so desarmado");
    return true;
  }
  if (!strcmp(c, "m")) {
    const uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    if (!mask || argc < 3) {
      stop_all("uso: m <0-4> <pct>");
      return true;
    }
    const uint16_t v = pct_to_permille(argv[2], 0);
    if (!motors_bench_set(mask, v)) {
      say("arme antes (arm)");
      return true;
    }
    say("alvo=%u%% mascara=0x%X", v / 10U, mask);
    return true;
  }
  if (!strcmp(c, "sweep")) {
    const uint8_t mask = motor_mask(argc > 1 ? argv[1] : NULL);
    if (!mask) {
      stop_all("uso: sweep <0-4> [max] [passo] [seg]");
      return true;
    }
    const uint16_t max = pct_to_permille(argc > 2 ? argv[2] : NULL, MOTORS_BENCH_MAX_PERMILLE);
    const uint16_t step = pct_to_permille(argc > 3 ? argv[3] : NULL, 100U);
    const long secs = argc > 4 ? strtol(argv[4], NULL, 10) : 3;
    const uint32_t hold = (uint32_t)(secs < 1 ? 1 : (secs > 30 ? 30 : secs)) * 1000U;
    if (!motors_bench_sweep(mask, max, step, hold)) {
      say("arme antes (arm)");
      return true;
    }
    say("sweep ate %u%% passo %u%% %lus", max / 10U, step / 10U, (unsigned long)(hold / 1000U));
    return true;
  }
  return false;
}

/* ---- Voo --------------------------------------------------------------- */

/** @brief Texto de um resultado de pedido. */
static void report(const char *what, flight_request_result_t r) {
  static const char *const status_text[] = {"aceito", "em andamento", "tente de novo", "negado",
                                            "falhou"};
  say("%s: %s (motivo %u)", what, status_text[r.status], (unsigned)r.reason);
}

/** @brief Arma pela USB, espera o voo armar e decola. */
static void hover(float height) {
  if (!CDC_HostDTR()) {
    say("abra a porta com DTR ligado");
    return;
  }
  if (motors_owner() == MOTORS_OWNER_BENCH) {
    say("pare a bancada antes");
    return;
  }
  flight_request_result_t r = flight_ctrl_arm(FLIGHT_ORIGIN_USB);
  if (r.status != FLIGHT_REQUEST_ACCEPTED) {
    report("arm", r);
    return;
  }
  flight_telemetry_t t;
  for (uint32_t waited = 0U; waited < ARM_WAIT_MS; waited += 10U) {
    flight_ctrl_get_telemetry(&t);
    if (t.state == FLIGHT_STATE_ARMED)
      break;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  r = flight_ctrl_takeoff(height);
  report("decolar", r);
}

/** @brief Monta e envia um trecho. */
static void leg(uint8_t action, uint8_t frame, float x, float y, float z, float yaw) {
  const flight_leg_t l = {.action = action,
                          .frame = frame,
                          .x = x,
                          .y = y,
                          .z = z,
                          .yaw = yaw,
                          .timeout_ms = LEG_TIMEOUT_MS};
  report("trecho", flight_ctrl_leg(&l));
}

/** @brief Estado do voo, estimativa e contadores. */
static void lqg_status(void) {
  flight_telemetry_t t;
  flight_ctrl_get_telemetry(&t);
  say("voo %s flags=0x%X meas=0x%X passo=%lu", flight_ctrl_state_name(t.state), t.flags,
      t.meas_flags, (unsigned long)t.step);
  say("x,y,z mm=%ld,%ld,%ld ref z=%ld", milli(t.x_hat[LQG_X_X]), milli(t.x_hat[LQG_X_Y]),
      milli(t.x_hat[LQG_X_Z]), milli(t.ref[LQG_R_Z]));
  say("phi,theta,psi mrad=%ld,%ld,%ld", milli(t.x_hat[LQG_X_PHI]), milli(t.x_hat[LQG_X_THETA]),
      milli(t.x_hat[LQG_X_PSI]));
  say("du mrad/s=%ld,%ld,%ld,%ld", milli(t.du[0]), milli(t.du[1]), milli(t.du[2]), milli(t.du[3]));
  say("passo max=%luus bno_falha=%lu mtf_falha=%lu", (unsigned long)t.max_step_us,
      (unsigned long)t.bno_misses, (unsigned long)t.mtf_misses);
}

/** @brief Comandos de voo; devolve false se não for um deles. */
static bool flight_command(int argc, char **argv) {
  const char *c = argv[0];
  const float a1 = argc > 1 ? strtof(argv[1], NULL) : 0.0F;
  const float a2 = argc > 2 ? strtof(argv[2], NULL) : 0.0F;
  const float a3 = argc > 3 ? strtof(argv[3], NULL) : 0.0F;
  if (!strcmp(c, "hover")) {
    hover(a1);
    return true;
  }
  if (!strcmp(c, "dry")) {
    const int i = ctrl_params_find("FLT_DRY_RUN", CTRL_PARAM_NAME_MAX);
    if (i < 0 || ctrl_params_set((size_t)i, 1.0F, flight_ctrl_is_armed()) != CTRL_PARAM_OK) {
      say("dry: desarme antes");
      return true;
    }
    hover(a1);
    return true;
  }
  if (!strcmp(c, "land")) {
    report("pousar", flight_ctrl_land(FLIGHT_REASON_NONE));
    return true;
  }
  if (!strcmp(c, "xyzero")) {
    report("xyzero", flight_ctrl_xy_zero());
    return true;
  }
  if (!strcmp(c, "hold")) {
    report("hold", flight_ctrl_hold(FLIGHT_REASON_NONE));
    return true;
  }
  if (!strcmp(c, "lqg")) {
    lqg_status();
    return true;
  }
  if (!strcmp(c, "goto") && argc >= 4) {
    flight_telemetry_t t;
    flight_ctrl_get_telemetry(&t);
    leg(FLIGHT_ACTION_GOTO_LOCAL, FLIGHT_FRAME_LOCAL_FRD, a1, a2, a3, t.ref[LQG_R_PSI]);
    return true;
  }
  if (!strcmp(c, "sp") && argc >= 5) {
    /* sp <x m> <y m> <altura m> <guinada graus>: setpoint (ver flight_ctrl_setpoint). */
    const float a4 = strtof(argv[4], NULL);
    report("setpoint", flight_ctrl_setpoint(a1, a2, a3, a4 * DEG_TO_RAD));
    return true;
  }
  if (!strcmp(c, "move") && argc >= 3) {
    leg(FLIGHT_ACTION_TRANSLATE_RELATIVE, FLIGHT_FRAME_BODY_FRD, a1, a2, a3, 0.0F);
    return true;
  }
  if (!strcmp(c, "yaw") && argc >= 2) {
    leg(FLIGHT_ACTION_ROTATE_RELATIVE, FLIGHT_FRAME_BODY_FRD, 0.0F, 0.0F, 0.0F, a1 * DEG_TO_RAD);
    return true;
  }
  return false;
}

/* ---- Ajuste ------------------------------------------------------------ */

/**
 * @brief Envio de Kd/Kfd do LQG ajustado pelo painel (control/lqg_tuning.h).
 *
 * `lq set` não responde quando dá certo: são 64 linhas seguidas, e o CRC do
 * commit já diz se alguma se perdeu.
 */
static void lq_command(int argc, char **argv) {
  const char *sub = argc > 1 ? argv[1] : "";
  if (!strcmp(sub, "begin")) {
    lqg_tuning_begin();
    return;
  }
  if (!strcmp(sub, "set") && argc >= 6) {
    const lqg_tuning_matrix_t m = !strcmp(argv[2], "kd")    ? LQG_TUNING_KD
                                  : !strcmp(argv[2], "kfd") ? LQG_TUNING_KFD
                                                            : LQG_TUNING_COUNT;
    const lqg_tuning_status_t s =
        lqg_tuning_stage(m, (unsigned)strtoul(argv[3], NULL, 10),
                         (unsigned)strtoul(argv[4], NULL, 10), strtof(argv[5], NULL));
    if (s != LQG_TUNING_OK)
      say("lq set: %s", lqg_tuning_status_text(s));
    return;
  }
  if (!strcmp(sub, "commit") || !strcmp(sub, "default")) {
    if (flight_ctrl_is_armed()) {
      say("lq: desarme antes");
      return;
    }
    if (!strcmp(sub, "default")) {
      lqg_tuning_restore_default();
    } else {
      uint32_t computed = 0U;
      const uint32_t expected = argc > 2 ? (uint32_t)strtoul(argv[2], NULL, 16) : 0U;
      const lqg_tuning_status_t s = lqg_tuning_commit(expected, &computed);
      if (s != LQG_TUNING_OK) {
        say("lq commit: %s (crc=%08lX)", lqg_tuning_status_text(s), (unsigned long)computed);
        return;
      }
    }
  }
  say("lq ganhos %s crc=%08lX", lqg_tuning_is_default() ? "padrao" : "do painel",
      (unsigned long)lqg_tuning_active_crc());
}

/** @brief Comandos de parâmetros e ganhos; devolve false se não for um deles. */
static bool tuning_command(int argc, char **argv) {
  const char *c = argv[0];
  if (!strcmp(c, "param") && argc >= 2) {
    if (!strcmp(argv[1], "list")) {
      for (size_t i = 0U; i < ctrl_params_count(); ++i)
        print_param(i);
      return true;
    }
    const int index = argc >= 3 ? ctrl_params_find(argv[2], CTRL_PARAM_NAME_MAX) : -1;
    if (index < 0) {
      say("parametro desconhecido");
      return true;
    }
    if (!strcmp(argv[1], "get")) {
      print_param((size_t)index);
      return true;
    }
    if (!strcmp(argv[1], "set") && argc >= 4) {
      const ctrl_param_status_t s =
          ctrl_params_set((size_t)index, strtof(argv[3], NULL), flight_ctrl_is_armed());
      if (s != CTRL_PARAM_OK)
        say("%s: %s", argv[2], ctrl_params_status_text(s));
      print_param((size_t)index);
      return true;
    }
    return false;
  }
  if (!strcmp(c, "gains")) {
    if (argc >= 2 && !strcmp(argv[1], "default"))
      ctrl_gains_restore_default();
    say("ganhos %s crc=%08lX", ctrl_gains_active_is_default() ? "do Simulink" : "da Pi",
        (unsigned long)ctrl_gains_active_crc());
    return true;
  }
  if (!strcmp(c, "lq")) {
    lq_command(argc, argv);
    return true;
  }
  return false;
}

/** @brief Comandos do enlace com o PC (watchdog); false se não for um deles. */
static bool link_command(int argc, char **argv) {
  const char *c = argv[0];
  if (!strcmp(c, "ping"))
    return true;
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
  return false;
}

/**
 * @brief Watchdog do PC: sem linha por watchdog_ms, o voo pousa em rampa e a
 *        bancada para. Age uma vez por silêncio.
 */
static void check_watchdog(void) {
  if (watchdog_ms == 0U || watchdog_fired)
    return;
  if ((xTaskGetTickCount() - last_line_tick) <= pdMS_TO_TICKS(watchdog_ms))
    return;
  if (motors_owner() == MOTORS_OWNER_BENCH) {
    motors_stop("watchdog: PC parou de falar");
    watchdog_fired = true;
  } else if (flight_ctrl_is_armed()) {
    (void)flight_ctrl_land(FLIGHT_REASON_LINK_LOST);
    say("watchdog: PC parou de falar, pousando");
    watchdog_fired = true;
  }
}

/* ---- Laço do console --------------------------------------------------- */

/** @brief Interpreta uma linha digitada. */
static void command(char *line) {
  char *argv[MAX_ARGS] = {0};
  int argc = 0;
  for (char *t = strtok(line, " \t"); t && argc < MAX_ARGS; t = strtok(NULL, " \t")) {
    argv[argc++] = t;
  }
  if (argc == 0) {
    stop_all("Enter");
    return;
  }
  const char *c = argv[0];
  if (!strcmp(c, "stop") || !strcmp(c, "disarm") || !strcmp(c, "s")) {
    stop_all(c);
    return;
  }
  if (!strcmp(c, "help") || !strcmp(c, "?")) {
    help();
    return;
  }
  if (!strcmp(c, "status")) {
    status();
    return;
  }
  if (!strcmp(c, "log")) {
    const bool on = argc > 1 && !strcmp(argv[1], "on");
    log_set_stream(on);
    say("log %s", on ? "on" : "off");
    return;
  }
  if (bench_command(argc, argv) || flight_command(argc, argv) || tuning_command(argc, argv) ||
      link_command(argc, argv)) {
    return;
  }
  stop_all("comando desconhecido");
}

/** @brief Task do console: monta linhas e corta a bancada se a porta fechar. */
static void console_task(void *argument) {
  (void)argument;
  char line[LINE_MAX];
  size_t used = 0U;
  char previous = 0;
  log_text("console: SEM HELICES na bancada. digite help");
  last_line_tick = xTaskGetTickCount();
  for (;;) {
    char c;
    while (xStreamBufferReceive(rx, &c, 1, pdMS_TO_TICKS(20)) == 1) {
      if (c == '\n' && previous == '\r') {
        previous = c;
        continue;
      }
      previous = c;
      if (c == '\r' || c == '\n') {
        last_line_tick = xTaskGetTickCount();
        watchdog_fired = false;
        line[used] = '\0';
        command(line);
        used = 0U;
      } else if (used < sizeof(line) - 1U) {
        line[used++] = c;
      }
    }
    if (motors_owner() == MOTORS_OWNER_BENCH) {
      if (CDC_HostDTR()) {
        /* Esta task roda a cada <= 20 ms; se ela travar, a bancada desarma. */
        motors_bench_keepalive();
      } else {
        motors_stop("porta USB fechada");
      }
    }
    check_watchdog();
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
