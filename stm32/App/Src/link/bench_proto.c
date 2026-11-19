/**
 * @file bench_proto.c
 * @brief Codificação das mensagens do enlace de bancada (contrato em link/bench_proto.h).
 */
#include "link/bench_proto.h"

#include "control/lqg_ctrl.h"

#include <math.h>
#include <string.h>

/** @brief Escritor sequencial little-endian sobre um payload. */
typedef struct {
  uint8_t *out;
  size_t used;
} writer_t;

static void put_u8(writer_t *w, uint8_t v) {
  w->out[w->used] = v;
  ++w->used;
}

static void put_u16(writer_t *w, uint16_t v) {
  put_u8(w, (uint8_t)(v & 0xFFU));
  put_u8(w, (uint8_t)(v >> 8));
}

static void put_u32(writer_t *w, uint32_t v) {
  for (uint32_t i = 0U; i < 4U; ++i) {
    put_u8(w, (uint8_t)((v >> (8U * i)) & 0xFFU));
  }
}

/* Desvio MISRA 21.15 documentado: memcpy entre float e uint32_t é a forma
   definida de copiar os bits IEEE 754 (union violaria a regra 19.2). */
static void put_f32(writer_t *w, float v) {
  uint32_t bits;
  (void)memcpy(&bits, &v, sizeof(bits));
  put_u32(w, bits);
}

static void put_text(writer_t *w, const char *text, size_t width) {
  bool ended = false;
  for (size_t i = 0U; i < width; ++i) {
    if (!ended && (text[i] == '\0')) {
      ended = true;
    }
    put_u8(w, ended ? 0U : (uint8_t)text[i]);
  }
}

size_t bl_encode_hello(const bl_hello_t *m, uint8_t out[BL_MAX_PAYLOAD]) {
  writer_t w = {out, 0U};
  put_u8(&w, BL_PROTO_MAJOR);
  put_u8(&w, BL_PROTO_MINOR);
  put_u16(&w, m->capabilities);
  put_u32(&w, m->uptime_ms);
  put_text(&w, m->firmware_id, BL_FIRMWARE_ID_LEN);
  put_text(&w, m->variant, BL_VARIANT_LEN);
  return w.used;
}

size_t bl_encode_status(const bl_status_t *m, uint8_t out[BL_MAX_PAYLOAD]) {
  writer_t w = {out, 0U};
  put_u32(&w, m->t_ms);
  put_u8(&w, m->flags);
  put_u8(&w, m->owner);
  put_u16(&w, m->cap_permille);
  put_u16(&w, m->rate_permille_s);
  for (uint32_t i = 0U; i < 4U; ++i) {
    put_u16(&w, m->target_permille[i]);
  }
  put_u8(&w, m->reversed);
  put_u8(&w, m->direction_sent);
  put_u8(&w, m->direction_known);
  put_u8(&w, m->commands_pending);
  put_u32(&w, m->tx_failures);
  put_u32(&w, m->heartbeat_age_ms);
  put_u8(&w, m->bno_divider);
  put_u8(&w, m->reserved);
  return w.used;
}

size_t bl_encode_imu(const bl_imu_t *m, uint8_t out[BL_MAX_PAYLOAD]) {
  writer_t w = {out, 0U};
  put_u32(&w, m->t_ms);
  for (uint32_t i = 0U; i < 4U; ++i) {
    put_f32(&w, m->q[i]);
  }
  for (uint32_t i = 0U; i < 3U; ++i) {
    put_f32(&w, m->gyro[i]);
  }
  for (uint32_t i = 0U; i < 3U; ++i) {
    put_f32(&w, m->lin_accel[i]);
  }
  for (uint32_t i = 0U; i < 3U; ++i) {
    put_u8(&w, m->accuracy[i]);
  }
  put_u8(&w, m->flags);
  for (uint32_t i = 0U; i < 3U; ++i) {
    put_u32(&w, m->count[i]);
  }
  for (uint32_t i = 0U; i < 3U; ++i) {
    put_u32(&w, m->gaps[i]);
  }
  return w.used;
}

size_t bl_encode_range(const bl_range_t *m, uint8_t out[BL_MAX_PAYLOAD]) {
  writer_t w = {out, 0U};
  put_u32(&w, m->t_ms);
  put_u32(&w, m->sensor_ms);
  put_u32(&w, m->distance_mm);
  put_u16(&w, (uint16_t)m->flow_x);
  put_u16(&w, (uint16_t)m->flow_y);
  put_u8(&w, m->quality);
  put_u8(&w, m->distance_status);
  put_u8(&w, m->flow_status);
  put_u8(&w, m->strength);
  put_u8(&w, m->precision);
  put_u8(&w, m->flags);
  put_u32(&w, m->frames);
  put_u32(&w, m->crc_errors);
  put_u32(&w, m->gaps);
  put_u32(&w, m->uart_errors);
  put_u32(&w, m->dropped_bytes);
  return w.used;
}

size_t bl_encode_motor(const bl_motor_t *m, uint8_t out[BL_MAX_PAYLOAD]) {
  writer_t w = {out, 0U};
  put_u32(&w, m->t_ms);
  put_u8(&w, m->flags);
  for (uint32_t i = 0U; i < 4U; ++i) {
    put_u16(&w, m->permille[i]);
  }
  for (uint32_t i = 0U; i < 4U; ++i) {
    put_u16(&w, m->dshot[i]);
  }
  put_u8(&w, m->telemetry_mask);
  return w.used;
}

size_t bl_encode_esc(const bl_esc_t *m, uint8_t out[BL_MAX_PAYLOAD]) {
  writer_t w = {out, 0U};
  put_u32(&w, m->t_ms);
  put_u8(&w, m->motor);
  put_u8(&w, (uint8_t)m->temperature_c);
  put_u16(&w, m->voltage_cv);
  put_u16(&w, m->current_ca);
  put_u16(&w, m->consumption_mah);
  put_u16(&w, m->erpm_100);
  put_u32(&w, m->frames_ok);
  put_u32(&w, m->timeouts);
  put_u8(&w, m->reserved);
  return w.used;
}

size_t bl_encode_power(const bl_power_t *m, uint8_t out[BL_MAX_PAYLOAD]) {
  writer_t w = {out, 0U};
  put_u32(&w, m->t_ms);
  put_f32(&w, m->vbat_pin_v);
  put_f32(&w, m->current_pin_v);
  put_f32(&w, m->vbat_v);
  put_f32(&w, m->current_a);
  put_u8(&w, m->flags);
  return w.used;
}

size_t bl_encode_sys(const bl_sys_t *m, uint8_t out[BL_MAX_PAYLOAD]) {
  writer_t w = {out, 0U};
  put_u32(&w, m->t_ms);
  put_u32(&w, m->telemetry_dropped);
  put_u32(&w, m->heap_free);
  put_u32(&w, m->link_packets);
  put_u32(&w, m->link_errors);
  put_u32(&w, m->link_overflows);
  put_u32(&w, m->esc_crc_errors);
  put_u32(&w, m->esc_length_errors);
  put_u32(&w, m->esc_unsolicited);
  put_u32(&w, m->esc_uart_errors);
  for (uint32_t i = 0U; i < 4U; ++i) {
    put_u32(&w, m->esc_timeouts[i]);
  }
  put_u32(&w, m->bno_errors);
  put_u32(&w, m->bno_resets);
  put_u8(&w, m->bno_fault);
  for (uint32_t i = 0U; i < 3U; ++i) {
    put_u8(&w, m->reserved[i]);
  }
  return w.used;
}

size_t bl_encode_ctrl(const bl_ctrl_t *m, uint8_t out[BL_MAX_PAYLOAD]) {
  writer_t w = {out, 0U};
  put_u32(&w, m->t_ms);
  put_u8(&w, m->state);
  put_u8(&w, m->flags);
  put_u8(&w, m->fail);
  put_u8(&w, 0U);
  for (uint32_t i = 0U; i < 9U; ++i) {
    put_f32(&w, m->y[i]);
  }
  for (uint32_t i = 0U; i < 3U; ++i) {
    put_f32(&w, m->pos[i]);
  }
  for (uint32_t i = 0U; i < 3U; ++i) {
    put_f32(&w, m->vel[i]);
  }
  put_f32(&w, m->h_ref);
  put_f32(&w, m->trim);
  put_f32(&w, m->base);
  put_f32(&w, m->ref_x);
  put_f32(&w, m->ref_y);
  put_f32(&w, m->ref_psi);
  put_f32(&w, m->vbat_factor);
  return w.used;
}

size_t bl_encode_ack(const bl_ack_t *m, uint8_t out[BL_MAX_PAYLOAD]) {
  writer_t w = {out, 0U};
  put_u8(&w, m->command);
  put_u8(&w, m->seq);
  put_u8(&w, m->result);
  put_u16(&w, m->value);
  return w.used;
}

size_t bl_encode_text(uint32_t t_ms, uint8_t level, const char *text, uint8_t out[BL_MAX_PAYLOAD]) {
  writer_t w = {out, 0U};
  put_u32(&w, t_ms);
  put_u8(&w, level);
  for (size_t i = 0U; (i < BL_TEXT_MAX) && (text[i] != '\0'); ++i) {
    put_u8(&w, (uint8_t)text[i]);
  }
  return w.used;
}

/* ---- Comandos ------------------------------------------------------------- */

static uint16_t get_u16(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static float get_f32(const uint8_t *p) {
  const uint32_t bits = get_u32(p);
  float v;
  (void)memcpy(&v, &bits, sizeof(v));
  return v;
}

static bool valid_mask(uint8_t mask) {
  return (mask != 0U) && (mask <= 0x0FU);
}

/** @brief Tamanho exigido do payload de cada comando; -1 se o id não existe. */
static int32_t expected_length(uint8_t id) {
  int32_t n = -1;
  switch (id) {
  case BL_CMD_HELLO:
  case BL_CMD_HEARTBEAT:
  case BL_CMD_ARM:
  case BL_CMD_STOP:
    n = 0;
    break;
  case BL_CMD_MOTOR_SET:
    n = (int32_t)BL_LEN_CMD_MOTOR_SET;
    break;
  case BL_CMD_CAP:
    n = (int32_t)BL_LEN_CMD_CAP;
    break;
  case BL_CMD_RATE:
    n = (int32_t)BL_LEN_CMD_RATE;
    break;
  case BL_CMD_DIRECTION:
    n = (int32_t)BL_LEN_CMD_DIRECTION;
    break;
  case BL_CMD_BEEP:
    n = (int32_t)BL_LEN_CMD_BEEP;
    break;
  case BL_CMD_SWEEP:
    n = (int32_t)BL_LEN_CMD_SWEEP;
    break;
  case BL_CMD_STREAM:
    n = (int32_t)BL_LEN_CMD_STREAM;
    break;
  case BL_CMD_POWER_CAL:
    n = (int32_t)BL_LEN_CMD_POWER_CAL;
    break;
  case BL_CMD_ESC_TELEM:
    n = (int32_t)BL_LEN_CMD_ESC_TELEM;
    break;
  case BL_CMD_CTRL:
    n = (int32_t)BL_LEN_CMD_CTRL;
    break;
  case BL_CMD_CTRL_PARAMS:
    n = (int32_t)BL_LEN_CMD_CTRL_PARAMS;
    break;
  case BL_CMD_CTRL_GAINS:
    n = (int32_t)BL_LEN_CMD_CTRL_GAINS;
    break;
  default:
    n = -1;
    break;
  }
  return n;
}

/** @brief Valida os campos de um payload já com o tamanho certo. */
static bool parse_fields(const uint8_t *p, bl_command_t *c) {
  bool ok = true;
  switch (c->id) {
  case BL_CMD_MOTOR_SET:
    c->mask = p[0];
    ok = valid_mask(c->mask);
    for (uint32_t i = 0U; i < 4U; ++i) {
      c->value[i] = get_u16(&p[1U + (2U * i)]);
      ok = ok && (c->value[i] <= 1000U);
    }
    break;
  case BL_CMD_CAP:
    c->value[0] = get_u16(p);
    ok = c->value[0] <= 1000U;
    break;
  case BL_CMD_RATE:
    c->value[0] = get_u16(p);
    ok = (c->value[0] >= 10U) && (c->value[0] <= 1000U);
    break;
  case BL_CMD_DIRECTION:
    c->mask = p[0];
    c->value[0] = p[1];
    c->flags = p[2];
    ok = valid_mask(c->mask) && ((p[1] & (uint8_t)~c->mask) == 0U) && (p[2] <= 1U);
    break;
  case BL_CMD_BEEP:
    c->mask = p[0];
    ok = valid_mask(c->mask);
    break;
  case BL_CMD_SWEEP:
    c->mask = p[0];
    c->value[0] = get_u16(&p[1]);
    c->step = get_u16(&p[3]);
    c->hold_ms = get_u32(&p[5]);
    ok = valid_mask(c->mask) && (c->value[0] <= 1000U) && (c->step >= 1U) && (c->step <= 1000U) &&
         (c->hold_ms >= 100U) && (c->hold_ms <= 30000U);
    break;
  case BL_CMD_STREAM:
    c->flags = p[0];
    c->value[0] = p[1];
    ok = (p[0] <= 1U) && (p[1] >= 1U) && (p[1] <= 16U);
    break;
  case BL_CMD_POWER_CAL:
    for (uint32_t i = 0U; i < 4U; ++i) {
      c->calib[i] = get_f32(&p[4U * i]);
      ok = ok && (isfinite(c->calib[i]) != 0);
    }
    c->flags = p[16];
    ok = ok && (c->calib[0] >= 0.0F) && (c->calib[1] >= 0.0F) && (c->calib[3] >= 0.0F) &&
         ((p[16] & (uint8_t)~0x03U) == 0U);
    break;
  case BL_CMD_ESC_TELEM:
    c->flags = p[0];
    c->value[0] = get_u16(&p[1]);
    ok = (p[0] <= 1U) && (c->value[0] >= 4U) && (c->value[0] <= 1000U);
    break;
  case BL_CMD_CTRL:
    c->flags = p[0];
    ok = (p[0] >= BL_CTRL_ARM) && (p[0] <= BL_CTRL_LAND);
    break;
  case BL_CMD_CTRL_PARAMS: {
    for (uint32_t i = 0U; i < 9U; ++i) {
      c->ctrl[i] = get_f32(&p[4U * i]);
    }
    c->flags = p[36];
    const lqg_params_t prm = {c->ctrl[0],
                              c->ctrl[1],
                              c->ctrl[2],
                              c->ctrl[3],
                              c->ctrl[4],
                              c->ctrl[5],
                              c->ctrl[6],
                              c->ctrl[7],
                              c->ctrl[8],
                              (p[36] & BL_CTRL_PARAM_AUTOTRIM) != 0U,
                              (p[36] & BL_CTRL_PARAM_USE_EKF) != 0U};
    const uint8_t known = BL_CTRL_PARAM_AUTOTRIM | BL_CTRL_PARAM_USE_EKF;
    ok = lqg_params_valid(&prm) && ((p[36] & (uint8_t)~known) == 0U);
    break;
  }
  case BL_CMD_CTRL_GAINS: {
    for (uint32_t i = 0U; i < 8U; ++i) {
      c->gains[i] = get_f32(&p[4U * i]);
    }
    const lqg_gains_t g = {c->gains[0], c->gains[1], c->gains[2], c->gains[3],
                           c->gains[4], c->gains[5], c->gains[6], c->gains[7]};
    ok = lqg_gains_valid(&g);
    break;
  }
  default:
    /* Comandos sem payload. */
    break;
  }
  return ok;
}

bl_result_t bl_command_parse(const uint8_t *packet, size_t length, bl_command_t *out) {
  bl_result_t result = BL_RESULT_INVALID;
  (void)memset(out, 0, sizeof(*out));
  if (length >= 2U) {
    out->id = packet[0];
    out->seq = packet[1];
    const int32_t expected = expected_length(out->id);
    if (expected < 0) {
      result = BL_RESULT_UNKNOWN;
    } else if ((length - 2U) == (size_t)expected) {
      result = parse_fields(&packet[2], out) ? BL_RESULT_OK : BL_RESULT_INVALID;
    } else {
      result = BL_RESULT_INVALID;
    }
  }
  return result;
}
