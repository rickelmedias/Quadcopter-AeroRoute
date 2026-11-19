/**
 * @file esc_kiss.c
 * @brief Telemetria KISS: decodificação e agenda (contrato em actuators/esc_kiss.h).
 */
#include "actuators/esc_kiss.h"

#include <string.h>

/** @brief a já alcançou b na aritmética modular de 32 bits. */
static bool reached(uint32_t a, uint32_t b) {
  return (a - b) < 0x80000000U;
}

uint8_t esc_kiss_crc8(const uint8_t *data, size_t length) {
  uint8_t crc = 0U;
  for (size_t i = 0U; i < length; ++i) {
    crc ^= data[i];
    for (uint32_t bit = 0U; bit < 8U; ++bit) {
      if ((crc & 0x80U) != 0U) {
        crc = (uint8_t)((uint8_t)(crc << 1) ^ 0x07U);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

static uint16_t be16(const uint8_t *p) {
  return (uint16_t)((uint16_t)((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

bool esc_kiss_decode(const uint8_t *data, size_t length, esc_kiss_frame_t *out) {
  bool ok = (length == ESC_KISS_FRAME_LEN) &&
            (esc_kiss_crc8(data, ESC_KISS_FRAME_LEN - 1U) == data[ESC_KISS_FRAME_LEN - 1U]);
  if (ok) {
    out->temperature_c = (int8_t)data[0];
    out->voltage_cv = be16(&data[1]);
    out->current_ca = be16(&data[3]);
    out->consumption_mah = be16(&data[5]);
    out->erpm_100 = be16(&data[7]);
  }
  return ok;
}

void esc_kiss_init(esc_kiss_t *k, uint32_t now_ms) {
  (void)memset(k, 0, sizeof(*k));
  k->period_ms = ESC_KISS_DEFAULT_PERIOD;
  k->next_ms = now_ms;
  k->pending = -1;
}

void esc_kiss_configure(esc_kiss_t *k, bool enabled, uint16_t period_ms, uint32_t now_ms) {
  k->enabled = enabled;
  k->period_ms = (period_ms < 4U) ? 4U : period_ms;
  k->pending = -1;
  k->next_ms = now_ms;
  k->rx_length = 0U;
  k->rx_overflow = false;
}

uint8_t esc_kiss_request(esc_kiss_t *k, bool command_frame, uint32_t now_ms) {
  uint8_t mask = 0U;
  if ((k->pending >= 0) && reached(now_ms, k->pending_ms + ESC_KISS_TIMEOUT_MS)) {
    ++k->timeouts[(uint8_t)k->pending];
    k->pending = -1;
  }
  if (command_frame) {
    /* Todos os ESCs respondem ao mesmo tempo: nada pode ser atribuído. */
    k->pending = -1;
    k->quiet = true;
    k->quiet_until_ms = now_ms + ESC_KISS_QUIET_MS;
    mask = 0x0FU;
  } else if (k->enabled && (k->pending < 0) && reached(now_ms, k->next_ms) &&
             (!k->quiet || reached(now_ms, k->quiet_until_ms))) {
    k->quiet = false;
    k->pending = (int8_t)k->next_motor;
    k->pending_ms = now_ms;
    k->next_motor = (uint8_t)((k->next_motor + 1U) % 4U);
    k->next_ms = now_ms + k->period_ms;
    mask = (uint8_t)(1U << (uint8_t)k->pending);
  } else {
    /* Nenhum pedido neste quadro. */
  }
  return mask;
}

void esc_kiss_rx_bytes(esc_kiss_t *k, const uint8_t *data, size_t length) {
  for (size_t i = 0U; i < length; ++i) {
    if (k->rx_length < sizeof(k->rx)) {
      k->rx[k->rx_length] = data[i];
      ++k->rx_length;
    } else {
      k->rx_overflow = true;
    }
  }
}

void esc_kiss_rx_idle(esc_kiss_t *k, uint32_t now_ms) {
  if ((k->rx_length > 0U) || k->rx_overflow) {
    esc_kiss_frame_t frame;
    if (k->quiet && !reached(now_ms, k->quiet_until_ms)) {
      ++k->unsolicited;
    } else if (k->rx_overflow || (k->rx_length != ESC_KISS_FRAME_LEN)) {
      ++k->length_errors;
    } else if (!esc_kiss_decode(k->rx, k->rx_length, &frame)) {
      ++k->crc_errors;
    } else if (k->pending < 0) {
      ++k->unsolicited;
    } else {
      const uint8_t m = (uint8_t)k->pending;
      k->frame[m] = frame;
      k->frame_ms[m] = now_ms;
      k->fresh |= (uint8_t)(1U << m);
      ++k->frames_ok[m];
      k->pending = -1;
    }
    k->rx_length = 0U;
    k->rx_overflow = false;
  }
}
