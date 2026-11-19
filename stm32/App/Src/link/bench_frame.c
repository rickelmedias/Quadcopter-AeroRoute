/**
 * @file bench_frame.c
 * @brief COBS + CRC16 do enlace de bancada (contrato em link/bench_frame.h).
 */
#include "link/bench_frame.h"

uint16_t bl_crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0U; i < length; ++i) {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (uint32_t bit = 0U; bit < 8U; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021U);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

size_t bl_cobs_encode(const uint8_t *in, size_t length, uint8_t *out, size_t capacity) {
  size_t code_index = 0U;
  size_t used = 1U;
  uint8_t code = 1U;
  bool ok = capacity > 0U;
  for (size_t i = 0U; ok && (i < length); ++i) {
    if (in[i] == 0U) {
      out[code_index] = code;
      code_index = used;
      code = 1U;
      ok = used < capacity;
      ++used;
    } else {
      ok = used < capacity;
      if (ok) {
        out[used] = in[i];
        ++used;
        ++code;
        if (code == 0xFFU) {
          out[code_index] = code;
          code_index = used;
          code = 1U;
          ok = used < capacity;
          ++used;
        }
      }
    }
  }
  if (ok) {
    out[code_index] = code;
  }
  return ok ? used : 0U;
}

size_t bl_cobs_decode(const uint8_t *in, size_t length, uint8_t *out, size_t capacity) {
  size_t i = 0U;
  size_t used = 0U;
  bool ok = length > 0U;
  while (ok && (i < length)) {
    const size_t code = in[i];
    ++i;
    if ((code == 0U) || ((i + code - 1U) > length) || ((used + code - 1U) > capacity)) {
      ok = false;
    } else {
      for (size_t k = 1U; ok && (k < code); ++k) {
        if (in[i] == 0U) {
          ok = false;
        } else {
          out[used] = in[i];
          ++used;
          ++i;
        }
      }
      if (ok && (code < 0xFFU) && (i < length)) {
        ok = used < capacity;
        if (ok) {
          out[used] = 0U;
          ++used;
        }
      }
    }
  }
  return ok ? used : 0U;
}

size_t bl_frame_encode(uint8_t id, uint8_t seq, const uint8_t *payload, size_t length, uint8_t *out,
                       size_t capacity) {
  size_t written = 0U;
  if ((length <= BL_MAX_PAYLOAD) && (capacity > 1U)) {
    uint8_t packet[BL_PACKET_MAX];
    packet[0] = id;
    packet[1] = seq;
    for (size_t i = 0U; i < length; ++i) {
      packet[2U + i] = payload[i];
    }
    const uint16_t crc = bl_crc16(packet, length + 2U);
    packet[length + 2U] = (uint8_t)(crc & 0xFFU);
    packet[length + 3U] = (uint8_t)(crc >> 8);
    written = bl_cobs_encode(packet, length + 4U, out, capacity - 1U);
    if (written > 0U) {
      out[written] = 0U;
      ++written;
    }
  }
  return written;
}

void bl_decoder_reset(bl_decoder_t *decoder) {
  decoder->used = 0U;
  decoder->overflow = false;
  decoder->packet_length = 0U;
}

/** @brief Valida um quadro completo e copia o pacote. */
static bl_rx_event_t finish(bl_decoder_t *d) {
  bl_rx_event_t event = BL_RX_ERROR;
  uint8_t packet[BL_PACKET_MAX];
  const size_t n = d->overflow ? 0U : bl_cobs_decode(d->frame, d->used, packet, sizeof(packet));
  if (n >= 4U) {
    const uint16_t crc = (uint16_t)packet[n - 2U] | (uint16_t)((uint16_t)packet[n - 1U] << 8);
    if (crc == bl_crc16(packet, n - 2U)) {
      for (size_t i = 0U; i < (n - 2U); ++i) {
        d->packet[i] = packet[i];
      }
      d->packet_length = n - 2U;
      ++d->packets;
      event = BL_RX_PACKET;
    }
  }
  if (event == BL_RX_ERROR) {
    ++d->errors;
  }
  d->used = 0U;
  d->overflow = false;
  return event;
}

bl_rx_event_t bl_decoder_push(bl_decoder_t *decoder, uint8_t byte) {
  bl_rx_event_t event = BL_RX_NONE;
  if (byte == 0U) {
    if ((decoder->used > 0U) || decoder->overflow) {
      event = finish(decoder);
    }
  } else if (decoder->used < sizeof(decoder->frame)) {
    decoder->frame[decoder->used] = byte;
    ++decoder->used;
  } else {
    decoder->overflow = true;
  }
  return event;
}
