/**
 * @file test_esc_kiss.c
 * @brief Telemetria KISS: CRC8 do AM32, decodificação, agenda e atribuição.
 */
#include "actuators/esc_kiss.h"

#include <assert.h>
#include <string.h>

/** @brief Quadro como o AM32 monta em makeTelemPackage(). */
static void make_frame(uint8_t out[ESC_KISS_FRAME_LEN], int8_t temp, uint16_t cv, uint16_t ca,
                       uint16_t mah, uint16_t erpm) {
  out[0] = (uint8_t)temp;
  out[1] = (uint8_t)(cv >> 8);
  out[2] = (uint8_t)cv;
  out[3] = (uint8_t)(ca >> 8);
  out[4] = (uint8_t)ca;
  out[5] = (uint8_t)(mah >> 8);
  out[6] = (uint8_t)mah;
  out[7] = (uint8_t)(erpm >> 8);
  out[8] = (uint8_t)erpm;
  out[9] = esc_kiss_crc8(out, 9U);
}

/** @brief Referência bit a bit do update_crc8() do AM32. */
static uint8_t am32_crc8(const uint8_t *buf, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; ++i) {
    uint8_t c = (uint8_t)(buf[i] ^ crc);
    for (int b = 0; b < 8; ++b) {
      c = (c & 0x80U) ? (uint8_t)(0x07U ^ (uint8_t)(c << 1)) : (uint8_t)(c << 1);
    }
    crc = c;
  }
  return crc;
}

static void test_decode(void) {
  uint8_t f[ESC_KISS_FRAME_LEN];
  make_frame(f, -3, 1680U, 1234U, 56U, 789U);
  assert(esc_kiss_crc8(f, 9U) == am32_crc8(f, 9U));
  esc_kiss_frame_t d;
  assert(esc_kiss_decode(f, sizeof(f), &d));
  assert(d.temperature_c == -3 && d.voltage_cv == 1680U && d.current_ca == 1234U);
  assert(d.consumption_mah == 56U && d.erpm_100 == 789U);
  f[4] ^= 1U;
  assert(!esc_kiss_decode(f, sizeof(f), &d));
  assert(!esc_kiss_decode(f, 9U, &d));
}

static void feed(esc_kiss_t *k, const uint8_t *data, size_t n, uint32_t now) {
  esc_kiss_rx_bytes(k, data, n);
  esc_kiss_rx_idle(k, now);
}

static void test_schedule(void) {
  esc_kiss_t k;
  esc_kiss_init(&k, 0U);
  assert(esc_kiss_request(&k, false, 0U) == 0U); /* desligado por padrão */
  esc_kiss_configure(&k, true, 10U, 0U);

  /* Um motor por vez, em rodízio, no período configurado. */
  assert(esc_kiss_request(&k, false, 0U) == 0x01U);
  assert(esc_kiss_request(&k, false, 1U) == 0U); /* aguardando resposta */
  uint8_t f[ESC_KISS_FRAME_LEN];
  make_frame(f, 30, 1600U, 100U, 1U, 50U);
  feed(&k, f, sizeof(f), 2U);
  assert(k.frames_ok[0] == 1U && (k.fresh & 0x01U) && k.frame[0].erpm_100 == 50U);
  assert(esc_kiss_request(&k, false, 5U) == 0U); /* período ainda não venceu */
  assert(esc_kiss_request(&k, false, 10U) == 0x02U);

  /* Sem resposta: timeout conta para o motor pendente e libera o próximo. */
  assert(esc_kiss_request(&k, false, 10U + ESC_KISS_TIMEOUT_MS - 1U) == 0U);
  assert(esc_kiss_request(&k, false, 20U) == 0x04U);
  assert(k.timeouts[1] == 1U);

  /* Quadro em dois pedaços (meia/volta do DMA) só fecha na linha ociosa. */
  make_frame(f, 31, 1601U, 101U, 2U, 51U);
  esc_kiss_rx_bytes(&k, f, 4U);
  esc_kiss_rx_bytes(&k, &f[4], 6U);
  esc_kiss_rx_idle(&k, 21U);
  assert(k.frames_ok[2] == 1U && k.frame[2].voltage_cv == 1601U);

  /* Tamanho errado, CRC errado e resposta sem pedido são contados, nunca atribuídos. */
  feed(&k, f, 7U, 22U);
  assert(k.length_errors == 1U);
  assert(esc_kiss_request(&k, false, 30U) == 0x08U);
  f[9] ^= 0xFFU;
  feed(&k, f, sizeof(f), 31U);
  assert(k.crc_errors == 1U && k.frames_ok[3] == 0U);
  f[9] ^= 0xFFU;
  assert(esc_kiss_request(&k, false, 39U) == 0U); /* timeout do motor 4 */
  assert(k.timeouts[3] == 1U);
  feed(&k, f, sizeof(f), 39U);
  assert(k.unsolicited == 1U);

  /* Comando especial: telemetria nos quatro, janela de silêncio e nada atribuído. */
  assert(esc_kiss_request(&k, true, 50U) == 0x0FU);
  feed(&k, f, sizeof(f), 51U);
  assert(k.unsolicited == 2U);
  assert(esc_kiss_request(&k, false, 60U) == 0U);
  assert(esc_kiss_request(&k, false, 50U + ESC_KISS_QUIET_MS) == 0x01U);

  /* Overflow do acumulador vira erro de tamanho. */
  uint8_t junk[40] = {0};
  feed(&k, junk, sizeof(junk), 80U);
  assert(k.length_errors == 2U);

  /* Desligar cancela o pendente sem contar timeout. */
  const uint32_t before = k.timeouts[0];
  esc_kiss_configure(&k, false, 10U, 90U);
  assert(esc_kiss_request(&k, false, 200U) == 0U && k.timeouts[0] == before);
}

int main(void) {
  test_decode();
  test_schedule();
  return 0;
}
