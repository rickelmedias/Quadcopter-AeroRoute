/**
 * @file bench_link.c
 * @brief Sessão USB da bancada (contrato em app/bench_link.h).
 */
#include "app/bench_link.h"
#include "app/startup.h"

#include "actuators/esc_telem.h"
#include "app/control_task.h"
#include "bno08x.h"
#include "link/bench_frame.h"
#include "link/bench_proto.h"
#include "motors.h"
#include "power.h"
#include "telemetry.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include <string.h>

#ifndef AEROROUTE_FIRMWARE_ID
/** Identidade gravada pelo tools/build.py (hash dos fontes); CubeIDE não define. */
#define AEROROUTE_FIRMWARE_ID "unversioned"
#endif
#ifndef AEROROUTE_BUILD_VARIANT
#define AEROROUTE_BUILD_VARIANT "ide"
#endif

/** Sem HEARTBEAT por este tempo, ARM é recusado (o árbitro usa o mesmo prazo). */
#define HEARTBEAT_TIMEOUT_MS MOTOR_ARB_BENCH_KEEPALIVE_MS
#define STATUS_PERIOD_MS     100U
#define POWER_PERIOD_MS      100U
#define SYS_PERIOD_MS        1000U

static StreamBufferHandle_t rx;
static volatile bool overflow;
static volatile uint32_t overflows;

void CDC_OnReceive(const uint8_t *data, uint32_t length) {
  if (rx != NULL) {
    BaseType_t wake = pdFALSE;
    if (xStreamBufferSendFromISR(rx, data, length, &wake) != length) {
      overflow = true;
      ++overflows;
    }
    portYIELD_FROM_ISR(wake);
  }
}

/** Estado da sessão; usado só pela task do enlace. */
typedef struct {
  bool connected;
  bool blocked;
  bool heartbeat_seen;
  uint32_t epoch;
  uint32_t heartbeat_ms;
  uint32_t next_status_ms;
  uint32_t next_power_ms;
  uint32_t next_sys_ms;
  bl_decoder_t decoder;
} session_t;

static session_t session;

static bool reached(uint32_t now, uint32_t when) {
  return (now - when) < 0x80000000U;
}

static void send(uint8_t id, const uint8_t *payload, size_t length) {
  (void)telemetry_send(id, payload, length);
}

static void ack(const bl_command_t *c, bl_result_t result, uint16_t value) {
  const bl_ack_t a = {c->id, c->seq, (uint8_t)result, value};
  uint8_t payload[BL_MAX_PAYLOAD];
  send(BL_MSG_ACK, payload, bl_encode_ack(&a, payload));
}

/** @brief Copia texto para campo de largura fixa (sem terminador obrigatório). */
static void copy_field(char *field, size_t width, const char *text) {
  const size_t n = strlen(text);
  (void)memcpy(field, text, (n < width) ? n : width);
}

static void send_hello(uint32_t now) {
  bl_hello_t h;
  (void)memset(&h, 0, sizeof(h));
  h.capabilities = (uint16_t)(BL_CAP_MOTORS | BL_CAP_ESC_KISS | BL_CAP_POWER_ADC | BL_CAP_CONTROL);
  h.uptime_ms = now;
  copy_field(h.firmware_id, sizeof(h.firmware_id), AEROROUTE_FIRMWARE_ID);
  copy_field(h.variant, sizeof(h.variant), AEROROUTE_BUILD_VARIANT);
  uint8_t payload[BL_MAX_PAYLOAD];
  send(BL_MSG_HELLO, payload, bl_encode_hello(&h, payload));
  /* Toda sessão nova mostra por que a placa reiniciou da última vez. */
  telemetry_text(BL_TEXT_INFO, app_reset_cause());
}

static bool heartbeat_fresh(const session_t *s, uint32_t now) {
  return s->heartbeat_seen && ((now - s->heartbeat_ms) < HEARTBEAT_TIMEOUT_MS);
}

/** @brief Condições comuns de armar (bancada e controle): HEARTBEAT e tensão. */
static bool arm_allowed(const session_t *s, uint32_t now) {
  power_sample_t p;
  power_cal_t cal;
  power_get(&p);
  power_get_cal(&cal);
  bool ok = true;
  if (!heartbeat_fresh(s, now)) {
    telemetry_text(BL_TEXT_WARN, "ARM recusado: sem HEARTBEAT recente");
    ok = false;
  } else if (!power_cal_arm_allowed(&cal, &p)) {
    telemetry_text(BL_TEXT_WARN, "ARM recusado: tensao abaixo do minimo ou invalida");
    ok = false;
  } else {
    /* pode armar */
  }
  return ok;
}

/** @brief Executa um comando válido; devolve o resultado e o valor do ACK. */
static bl_result_t execute(session_t *s, const bl_command_t *c, uint32_t now, uint16_t *value) {
  bool ok = true;
  switch (c->id) {
  case BL_CMD_HELLO:
    send_hello(now);
    break;
  case BL_CMD_ARM:
    ok = arm_allowed(s, now);
    if (ok) {
      ok = motors_arm(MOTORS_OWNER_BENCH);
      if (!ok) {
        telemetry_text(BL_TEXT_WARN, "ARM recusado: saida ocupada, nao zerada ou comando pendente");
      }
    }
    break;
  case BL_CMD_CTRL:
    ok = heartbeat_fresh(s, now);
    if (!ok) {
      telemetry_text(BL_TEXT_WARN, "controle recusado: sem HEARTBEAT recente");
    } else if (c->flags == BL_CTRL_ARM) {
      ok = arm_allowed(s, now) && control_arm();
    } else if (c->flags == BL_CTRL_TAKEOFF) {
      ok = control_takeoff();
    } else {
      ok = control_land();
    }
    break;
  case BL_CMD_CTRL_PARAMS: {
    const lqg_params_t params = {c->ctrl[0],
                                 c->ctrl[1],
                                 c->ctrl[2],
                                 c->ctrl[3],
                                 c->ctrl[4],
                                 c->ctrl[5],
                                 c->ctrl[6],
                                 c->ctrl[7],
                                 c->ctrl[8],
                                 (c->flags & BL_CTRL_PARAM_AUTOTRIM) != 0U,
                                 (c->flags & BL_CTRL_PARAM_USE_EKF) != 0U};
    ok = control_set_params(&params);
    break;
  }
  case BL_CMD_CTRL_GAINS: {
    const lqg_gains_t gains = {c->gains[0], c->gains[1], c->gains[2], c->gains[3],
                               c->gains[4], c->gains[5], c->gains[6], c->gains[7]};
    ok = control_set_gains(&gains);
    break;
  }
  case BL_CMD_STOP:
    motors_stop("STOP pela IHM");
    break;
  case BL_CMD_MOTOR_SET:
    for (uint32_t i = 0U; i < MOTORS_COUNT; ++i) {
      if ((((uint32_t)c->mask >> i) & 1U) != 0U) {
        ok = motors_bench_set((uint8_t)(1U << i), c->value[i]) && ok;
      }
    }
    break;
  case BL_CMD_CAP:
    *value = motors_bench_cap(c->value[0]);
    break;
  case BL_CMD_RATE:
    motors_bench_rate(c->value[0]);
    break;
  case BL_CMD_DIRECTION:
    ok = motors_set_direction(c->mask, (uint8_t)c->value[0], c->flags != 0U);
    break;
  case BL_CMD_BEEP:
    ok = motors_beep(c->mask);
    break;
  case BL_CMD_SWEEP:
    ok = motors_bench_sweep(c->mask, c->value[0], c->step, c->hold_ms);
    break;
  case BL_CMD_STREAM:
    telemetry_set_stream(c->flags != 0U, (uint8_t)c->value[0]);
    break;
  case BL_CMD_POWER_CAL: {
    const power_cal_t cal = {c->calib[0],
                             c->calib[1],
                             c->calib[2],
                             c->calib[3],
                             (c->flags & 0x01U) != 0U,
                             (c->flags & 0x02U) != 0U};
    power_set_cal(&cal);
    break;
  }
  case BL_CMD_ESC_TELEM:
    esc_telem_configure(c->flags != 0U, c->value[0]);
    break;
  default:
    ok = false;
    break;
  }
  return ok ? BL_RESULT_OK : BL_RESULT_REJECTED;
}

static void handle_packet(session_t *s, uint32_t now) {
  bl_command_t c;
  const bl_result_t parsed = bl_command_parse(s->decoder.packet, s->decoder.packet_length, &c);
  if (parsed != BL_RESULT_OK) {
    motors_stop("comando USB invalido");
    telemetry_text(BL_TEXT_ERROR, "comando invalido: saida parada");
    ack(&c, parsed, 0U);
  } else if (c.id == BL_CMD_HEARTBEAT) {
    motors_bench_keepalive();
    s->heartbeat_seen = true;
    s->heartbeat_ms = now;
  } else {
    uint16_t value = 0U;
    const bl_result_t result = execute(s, &c, now, &value);
    ack(&c, result, value);
  }
}

static void send_status(const session_t *s, uint32_t now) {
  motors_status_t m;
  motors_get_status(&m);
  power_sample_t p;
  power_cal_t cal;
  power_get(&p);
  power_get_cal(&cal);
  bl_status_t st;
  (void)memset(&st, 0, sizeof(st));
  st.t_ms = now;
  st.owner = (uint8_t)m.owner;
  st.flags = (uint8_t)((m.owner != MOTORS_OWNER_NONE) ? BL_STATUS_ARMED : 0U);
  st.flags |= m.sweeping ? BL_STATUS_SWEEPING : 0U;
  st.flags |= heartbeat_fresh(s, now) ? BL_STATUS_HEARTBEAT_OK : 0U;
  st.flags |= s->blocked ? BL_STATUS_RX_BLOCKED : 0U;
  st.flags |= telemetry_stream_enabled() ? BL_STATUS_STREAM : 0U;
  st.flags |= power_cal_arm_allowed(&cal, &p) ? 0U : BL_STATUS_POWER_LOW;
  st.flags |= esc_telem_enabled() ? BL_STATUS_ESC_TELEM : 0U;
  st.cap_permille = m.bench_cap;
  st.rate_permille_s = (uint16_t)m.bench_rate;
  for (uint32_t i = 0U; i < MOTORS_COUNT; ++i) {
    st.target_permille[i] = m.bench_target[i];
  }
  st.reversed = m.reversed;
  st.direction_sent = m.direction_transmitted;
  st.direction_known = m.reversed_known;
  st.commands_pending = m.commands_pending;
  st.tx_failures = m.tx_failures;
  st.heartbeat_age_ms = s->heartbeat_seen ? (now - s->heartbeat_ms) : 0xFFFFFFFFU;
  st.bno_divider = telemetry_bno_divider();
  uint8_t payload[BL_MAX_PAYLOAD];
  send(BL_MSG_STATUS, payload, bl_encode_status(&st, payload));
}

static void send_power(uint32_t now) {
  power_sample_t p;
  power_get(&p);
  bl_power_t m;
  m.t_ms = now;
  m.vbat_pin_v = p.vbat_pin_v;
  m.current_pin_v = p.current_pin_v;
  m.vbat_v = p.vbat_v;
  m.current_a = p.current_a;
  m.flags = (uint8_t)((p.vbat_read ? BL_POWER_VBAT_READ : 0U) |
                      (p.current_read ? BL_POWER_CURRENT_READ : 0U) |
                      (p.vbat_calibrated ? BL_POWER_VBAT_CAL : 0U) |
                      (p.current_calibrated ? BL_POWER_CURRENT_CAL : 0U));
  uint8_t payload[BL_MAX_PAYLOAD];
  send(BL_MSG_POWER, payload, bl_encode_power(&m, payload));
}

static void send_esc_and_sys(const session_t *s, uint32_t now, bool with_sys) {
  esc_kiss_t k;
  esc_telem_take(&k);
  uint8_t payload[BL_MAX_PAYLOAD];
  for (uint8_t i = 0U; i < 4U; ++i) {
    if (((uint32_t)k.fresh & (1UL << i)) != 0U) {
      bl_esc_t e;
      e.t_ms = k.frame_ms[i];
      e.motor = i;
      e.temperature_c = k.frame[i].temperature_c;
      e.voltage_cv = k.frame[i].voltage_cv;
      e.current_ca = k.frame[i].current_ca;
      e.consumption_mah = k.frame[i].consumption_mah;
      e.erpm_100 = k.frame[i].erpm_100;
      e.frames_ok = k.frames_ok[i];
      e.timeouts = k.timeouts[i];
      e.reserved = 0U;
      send(BL_MSG_ESC, payload, bl_encode_esc(&e, payload));
    }
  }
  if (with_sys) {
    bno_sample_t b;
    bno_get_sample(&b);
    bl_sys_t m;
    (void)memset(&m, 0, sizeof(m));
    m.t_ms = now;
    m.telemetry_dropped = telemetry_dropped();
    m.heap_free = (uint32_t)xPortGetFreeHeapSize();
    m.link_packets = s->decoder.packets;
    m.link_errors = s->decoder.errors;
    m.link_overflows = overflows;
    m.esc_crc_errors = k.crc_errors;
    m.esc_length_errors = k.length_errors;
    m.esc_unsolicited = k.unsolicited;
    m.esc_uart_errors = esc_telem_uart_errors();
    for (uint32_t i = 0U; i < 4U; ++i) {
      m.esc_timeouts[i] = k.timeouts[i];
    }
    m.bno_errors = b.errors;
    m.bno_resets = b.resets;
    m.bno_fault = (uint8_t)b.fault;
    send(BL_MSG_SYS, payload, bl_encode_sys(&m, payload));
  }
}

/** @brief Começa ou encerra sessão; bytes da sessão anterior nunca viram comandos. */
static void reset_session(session_t *s, bool connected, uint32_t epoch) {
  motors_stop(connected ? "nova sessao USB" : "sessao USB encerrada");
  taskENTER_CRITICAL();
  (void)xStreamBufferReset(rx);
  overflow = false;
  taskEXIT_CRITICAL();
  bl_decoder_reset(&s->decoder);
  s->connected = connected;
  s->blocked = false;
  s->heartbeat_seen = false;
  s->epoch = epoch;
  if (connected) {
    telemetry_flush_queue();
    send_hello(HAL_GetTick());
    telemetry_text(BL_TEXT_INFO, "bench-link v1; SEM HELICES sem suporte fixo");
  }
}

static void link_task(void *argument) {
  (void)argument;
  session_t *s = &session;
  s->epoch = CDC_CloseEpoch();
  for (;;) {
    uint8_t bytes[64];
    const size_t n = xStreamBufferReceive(rx, bytes, sizeof(bytes), pdMS_TO_TICKS(5U));
    const uint32_t now = HAL_GetTick();
    const bool dtr = CDC_HostDTR();
    const uint32_t epoch = CDC_CloseEpoch();
    esc_telem_service();
    if (dtr != s->connected || (epoch != s->epoch)) {
      reset_session(s, dtr, epoch);
    } else if (s->connected) {
      if (overflow && !s->blocked) {
        motors_stop("overflow na recepcao USB");
        telemetry_text(BL_TEXT_ERROR, "overflow USB: comandos bloqueados ate reconectar");
        s->blocked = true;
      }
      for (size_t i = 0U; (i < n) && !s->blocked; ++i) {
        const bl_rx_event_t ev = bl_decoder_push(&s->decoder, bytes[i]);
        if (ev == BL_RX_PACKET) {
          handle_packet(s, now);
        } else if (ev == BL_RX_ERROR) {
          motors_stop("quadro USB corrompido");
          telemetry_text(BL_TEXT_ERROR, "quadro corrompido (CRC/COBS): saida parada");
        } else {
          /* quadro ainda incompleto */
        }
      }
      const bool sys_due = reached(now, s->next_sys_ms);
      if (sys_due) {
        s->next_sys_ms = now + SYS_PERIOD_MS;
      }
      send_esc_and_sys(s, now, sys_due);
      if (reached(now, s->next_status_ms)) {
        s->next_status_ms = now + STATUS_PERIOD_MS;
        send_status(s, now);
      }
      if (reached(now, s->next_power_ms)) {
        s->next_power_ms = now + POWER_PERIOD_MS;
        send_power(now);
      }
    } else {
      /* porta fechada: nada a fazer */
    }
  }
}

static StaticTask_t link_tcb;
static StackType_t link_stack[768];

bool bench_link_start(void) {
  rx = xStreamBufferCreate(512U, 1U);
  return (rx != NULL) &&
         (xTaskCreateStatic(link_task, "BenchLink", sizeof(link_stack) / sizeof(link_stack[0]),
                            NULL, tskIDLE_PRIORITY + 2U, link_stack, &link_tcb) != NULL);
}
