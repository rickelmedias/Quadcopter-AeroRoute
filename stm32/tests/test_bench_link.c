/**
 * @file test_bench_link.c
 * @brief bench-link v1: quadros idênticos aos vetores do host, decodificador e
 *        validação de comandos.
 *
 * Os vetores (tests/vectors/bench_link.txt) são gerados pela implementação
 * Python (hmi/tests/vectors.py); aqui as mesmas mensagens são montadas pelos
 * codificadores do firmware e precisam dar os mesmos bytes.
 */
#include "link/bench_frame.h"
#include "link/bench_proto.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_VECTORS 32

typedef struct {
  char name[32];
  uint8_t bytes[BL_FRAME_MAX];
  size_t length;
} vector_t;

static vector_t vectors[MAX_VECTORS];
static size_t vector_count;

static void load_vectors(const char *path) {
  FILE *f = fopen(path, "r");
  assert(f != NULL);
  char line[512];
  while (fgets(line, sizeof(line), f) != NULL) {
    if ((line[0] == '#') || (line[0] == '\n')) {
      continue;
    }
    vector_t *v = &vectors[vector_count++];
    char hex[400];
    assert(sscanf(line, "%31s %399s", v->name, hex) == 2);
    const size_t n = strlen(hex) / 2U;
    assert(n <= sizeof(v->bytes));
    for (size_t i = 0; i < n; ++i) {
      unsigned byte;
      assert(sscanf(&hex[2U * i], "%2x", &byte) == 1);
      v->bytes[i] = (uint8_t)byte;
    }
    v->length = n;
  }
  fclose(f);
}

static const vector_t *find(const char *name) {
  for (size_t i = 0; i < vector_count; ++i) {
    if (strcmp(vectors[i].name, name) == 0) {
      return &vectors[i];
    }
  }
  fprintf(stderr, "vetor ausente: %s\n", name);
  abort();
}

static void expect_frame(const char *name, uint8_t id, uint8_t seq, const uint8_t *payload,
                         size_t length) {
  uint8_t frame[BL_FRAME_MAX];
  const size_t n = bl_frame_encode(id, seq, payload, length, frame, sizeof(frame));
  const vector_t *v = find(name);
  if ((n != v->length) || (memcmp(frame, v->bytes, n) != 0)) {
    fprintf(stderr, "quadro diferente do host: %s\n", name);
    abort();
  }
}

static void test_vectors(void) {
  uint8_t p[BL_MAX_PAYLOAD];

  bl_hello_t hello = {7U, 123456U, {0}, {0}};
  memcpy(hello.firmware_id, "0123456789abcdef01234567", 24);
  strcpy(hello.variant, "Release");
  size_t n = bl_encode_hello(&hello, p);
  assert(n == BL_LEN_HELLO);
  expect_frame("hello", BL_MSG_HELLO, 1U, p, n);

  const bl_status_t status = {1000U, BL_STATUS_ARMED | BL_STATUS_HEARTBEAT_OK | BL_STATUS_STREAM,
                              1U,    400U,
                              200U,  {100U, 0U, 250U, 400U},
                              9U,    9U,
                              0U,    0U,
                              3U,    42U,
                              1U,    0U};
  n = bl_encode_status(&status, p);
  assert(n == BL_LEN_STATUS);
  expect_frame("status", BL_MSG_STATUS, 2U, p, n);

  const bl_imu_t imu = {2000U,
                        {1.0F, 0.0F, -0.5F, 0.25F},
                        {0.125F, -0.0625F, 2.0F},
                        {-9.5F, 0.0F, 0.75F},
                        {3U, 2U, 1U},
                        1U,
                        {400U, 401U, 402U},
                        {0U, 1U, 2U}};
  n = bl_encode_imu(&imu, p);
  assert(n == BL_LEN_IMU);
  expect_frame("imu", BL_MSG_IMU, 3U, p, n);

  const bl_range_t range = {3000U, 2990U, 1234U, -12,  34, 200U, 1U, 1U,
                            90U,   5U,    3U,    100U, 1U, 2U,   0U, 0U};
  n = bl_encode_range(&range, p);
  assert(n == BL_LEN_RANGE);
  expect_frame("range", BL_MSG_RANGE, 4U, p, n);

  const bl_motor_t motor = {4000U, 1U, {100U, 200U, 0U, 1000U}, {248U, 448U, 0U, 2047U}, 4U};
  n = bl_encode_motor(&motor, p);
  assert(n == BL_LEN_MOTOR);
  expect_frame("motor", BL_MSG_MOTOR, 5U, p, n);

  const bl_esc_t esc = {5000U, 2U, -5, 1680U, 250U, 12U, 345U, 77U, 3U, 0U};
  n = bl_encode_esc(&esc, p);
  assert(n == BL_LEN_ESC);
  expect_frame("esc", BL_MSG_ESC, 6U, p, n);

  const bl_power_t power = {6000U, 1.5F, 0.25F, 16.5F, -2.0F, 0x0FU};
  n = bl_encode_power(&power, p);
  assert(n == BL_LEN_POWER);
  expect_frame("power", BL_MSG_POWER, 7U, p, n);

  const bl_sys_t sys = {7000U, 1U,  20000U, 500U,        2U, 0U, 3U, 4U, 5U, 6U, {7U, 8U, 9U, 10U},
                        11U,   12U, 7U,     {0U, 0U, 0U}};
  n = bl_encode_sys(&sys, p);
  assert(n == BL_LEN_SYS);
  expect_frame("sys", BL_MSG_SYS, 8U, p, n);

  const bl_ctrl_t ctrl = {9000U,
                          4U,
                          0x1FU,
                          5U,
                          {-0.25F, 0.0625F, -0.125F, 0.5F, 0.03125F, -0.015625F, 0.25F, -0.5F, 1.0F},
                          {0.5F, -0.25F, -0.3125F},
                          {0.0625F, 0.0F, -0.125F},
                          0.3125F,
                          -12.5F,
                          537.5F,
                          0.25F,
                          -0.125F,
                          0.5F,
                          1.0625F};
  n = bl_encode_ctrl(&ctrl, p);
  assert(n == BL_LEN_CTRL);
  expect_frame("ctrl", BL_MSG_CTRL, 11U, p, n);

  const bl_ack_t a = {BL_CMD_CAP, 17U, BL_RESULT_OK, 400U};
  n = bl_encode_ack(&a, p);
  assert(n == BL_LEN_ACK);
  expect_frame("ack", BL_MSG_ACK, 9U, p, n);

  n = bl_encode_text(8000U, 1U, "ola bancada", p);
  expect_frame("text", BL_MSG_TEXT, 10U, p, n);
}

/** @brief Decodifica um vetor de comando com o parser do firmware. */
static bl_result_t parse_vector(const char *name, bl_command_t *c) {
  const vector_t *v = find(name);
  bl_decoder_t d;
  memset(&d, 0, sizeof(d));
  bl_rx_event_t ev = BL_RX_NONE;
  for (size_t i = 0; i < v->length; ++i) {
    ev = bl_decoder_push(&d, v->bytes[i]);
  }
  assert(ev == BL_RX_PACKET);
  return bl_command_parse(d.packet, d.packet_length, c);
}

static void test_commands(void) {
  bl_command_t c;
  assert(parse_vector("cmd_motor_set", &c) == BL_RESULT_OK);
  assert(c.id == BL_CMD_MOTOR_SET && c.seq == 20U && c.mask == 5U);
  assert(c.value[0] == 100U && c.value[2] == 250U);
  assert(parse_vector("cmd_cap", &c) == BL_RESULT_OK && c.value[0] == 400U);
  assert(parse_vector("cmd_direction", &c) == BL_RESULT_OK);
  assert(c.mask == 9U && c.value[0] == 1U && c.flags == 0U);
  assert(parse_vector("cmd_sweep", &c) == BL_RESULT_OK);
  assert(c.mask == 2U && c.value[0] == 300U && c.step == 50U && c.hold_ms == 2000U);
  assert(parse_vector("cmd_power_cal", &c) == BL_RESULT_OK);
  assert(c.calib[0] == 11.0F && c.calib[1] == 50.0F && c.calib[2] == 0.5F && c.calib[3] == 10.5F);
  assert(c.flags == 3U);
  assert(parse_vector("cmd_heartbeat", &c) == BL_RESULT_OK && c.id == BL_CMD_HEARTBEAT);
  assert(parse_vector("cmd_ctrl_takeoff", &c) == BL_RESULT_OK);
  assert(c.id == BL_CMD_CTRL && c.flags == BL_CTRL_TAKEOFF);
  assert(parse_vector("cmd_ctrl_params", &c) == BL_RESULT_OK);
  assert(c.id == BL_CMD_CTRL_PARAMS && c.ctrl[0] == 0.5F && c.ctrl[1] == 600.0F);
  assert(c.ctrl[2] == 800.0F && c.ctrl[3] == 0.125F && c.ctrl[4] == 0.25F);
  assert(c.ctrl[5] == 50.0F && c.ctrl[6] == 10.0F && c.ctrl[7] == 0.125F && c.ctrl[8] == 0.25F);
  assert(c.flags == (BL_CTRL_PARAM_AUTOTRIM | BL_CTRL_PARAM_USE_EKF));

  /* Faixas e tamanhos. */
  const uint8_t bad_mask[] = {BL_CMD_MOTOR_SET, 1, 0x10, 0, 0, 0, 0, 0, 0, 0, 0};
  assert(bl_command_parse(bad_mask, sizeof(bad_mask), &c) == BL_RESULT_INVALID);
  const uint8_t too_fast[] = {BL_CMD_MOTOR_SET, 1, 0x01, 0xE9, 0x03, 0, 0, 0, 0, 0, 0}; /* 1001 */
  assert(bl_command_parse(too_fast, sizeof(too_fast), &c) == BL_RESULT_INVALID);
  const uint8_t short_cap[] = {BL_CMD_CAP, 1, 0x10};
  assert(bl_command_parse(short_cap, sizeof(short_cap), &c) == BL_RESULT_INVALID);
  const uint8_t arm_extra[] = {BL_CMD_ARM, 1, 0};
  assert(bl_command_parse(arm_extra, sizeof(arm_extra), &c) == BL_RESULT_INVALID);
  const uint8_t unknown[] = {0x85, 1};
  assert(bl_command_parse(unknown, sizeof(unknown), &c) == BL_RESULT_UNKNOWN);
  const uint8_t telemetry_id[] = {BL_MSG_STATUS, 1};
  assert(bl_command_parse(telemetry_id, sizeof(telemetry_id), &c) == BL_RESULT_UNKNOWN);
  const uint8_t dir_outside[] = {BL_CMD_DIRECTION, 1, 0x01, 0x02, 0};
  assert(bl_command_parse(dir_outside, sizeof(dir_outside), &c) == BL_RESULT_INVALID);
  const uint8_t rate_low[] = {BL_CMD_RATE, 1, 9, 0};
  assert(bl_command_parse(rate_low, sizeof(rate_low), &c) == BL_RESULT_INVALID);
  const uint8_t sweep_hold[] = {BL_CMD_SWEEP, 1, 1, 100, 0, 10, 0, 50, 0, 0, 0}; /* 50 ms */
  assert(bl_command_parse(sweep_hold, sizeof(sweep_hold), &c) == BL_RESULT_INVALID);
  const uint8_t stream_div[] = {BL_CMD_STREAM, 1, 1, 17};
  assert(bl_command_parse(stream_div, sizeof(stream_div), &c) == BL_RESULT_INVALID);
  uint8_t nan_cal[] = {
      BL_CMD_POWER_CAL, 1, 0, 0, 0xC0, 0x7F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  assert(bl_command_parse(nan_cal, sizeof(nan_cal), &c) == BL_RESULT_INVALID);
  const uint8_t esc_period[] = {BL_CMD_ESC_TELEM, 1, 1, 3, 0};
  assert(bl_command_parse(esc_period, sizeof(esc_period), &c) == BL_RESULT_INVALID);
  assert(bl_command_parse(unknown, 1U, &c) == BL_RESULT_INVALID);

  assert(parse_vector("cmd_ctrl_gains", &c) == BL_RESULT_OK);
  assert(c.id == BL_CMD_CTRL_GAINS && c.gains[0] == 8.0F && c.gains[1] == 4.0F);
  assert(c.gains[2] == 1.5F && c.gains[7] == 0.5F);
  uint8_t gains_bad[2U + BL_LEN_CMD_CTRL_GAINS] = {BL_CMD_CTRL_GAINS, 1};
  const float g_att_low = 0.1F; /* atitude abaixo de 0,25 */
  for (uint32_t i = 0U; i < 8U; ++i) {
    const float one = 1.0F;
    memcpy(&gains_bad[2U + (4U * i)], &one, sizeof(one));
  }
  assert(bl_command_parse(gains_bad, sizeof(gains_bad), &c) == BL_RESULT_OK);
  memcpy(&gains_bad[2U + 8U], &g_att_low, sizeof(g_att_low));
  assert(bl_command_parse(gains_bad, sizeof(gains_bad), &c) == BL_RESULT_INVALID);

  /* Controle: ação 1..3 e parâmetros dentro dos limites. */
  const uint8_t ctrl_zero[] = {BL_CMD_CTRL, 1, 0};
  assert(bl_command_parse(ctrl_zero, sizeof(ctrl_zero), &c) == BL_RESULT_INVALID);
  const uint8_t ctrl_four[] = {BL_CMD_CTRL, 1, 4};
  assert(bl_command_parse(ctrl_four, sizeof(ctrl_four), &c) == BL_RESULT_INVALID);
  const uint8_t ctrl_land[] = {BL_CMD_CTRL, 1, BL_CTRL_LAND};
  assert(bl_command_parse(ctrl_land, sizeof(ctrl_land), &c) == BL_RESULT_OK);
  const vector_t *good = find("cmd_ctrl_params");
  bl_decoder_t dec;
  memset(&dec, 0, sizeof(dec));
  for (size_t i = 0; i < good->length; ++i) {
    (void)bl_decoder_push(&dec, good->bytes[i]);
  }
  uint8_t params[BL_MAX_PAYLOAD];
  memcpy(params, dec.packet, dec.packet_length);
  assert(bl_command_parse(params, dec.packet_length, &c) == BL_RESULT_OK);
  const float height_bad = 0.0F; /* abaixo de 0,10 m */
  memcpy(&params[2], &height_bad, sizeof(height_bad));
  assert(bl_command_parse(params, dec.packet_length, &c) == BL_RESULT_INVALID);
  const float height_nan = NAN;
  memcpy(&params[2], &height_nan, sizeof(height_nan));
  assert(bl_command_parse(params, dec.packet_length, &c) == BL_RESULT_INVALID);
  const float height_ok = 0.4F;
  memcpy(&params[2], &height_ok, sizeof(height_ok));
  assert(bl_command_parse(params, dec.packet_length, &c) == BL_RESULT_OK);
  params[2U + 36U] = 4U; /* flag desconhecida */
  assert(bl_command_parse(params, dec.packet_length, &c) == BL_RESULT_INVALID);
}

static void test_decoder(void) {
  bl_decoder_t d;
  memset(&d, 0, sizeof(d));
  const vector_t *v = find("cmd_cap");

  /* Lixo antes, ressincronização por 0x00 e quadro válido depois. */
  const uint8_t junk[] = {0x13, 0x37, 0x00};
  assert(bl_decoder_push(&d, junk[0]) == BL_RX_NONE);
  assert(bl_decoder_push(&d, junk[1]) == BL_RX_NONE);
  assert(bl_decoder_push(&d, junk[2]) == BL_RX_ERROR);
  assert(bl_decoder_push(&d, 0x00) == BL_RX_NONE); /* quadro vazio é ignorado */
  bl_rx_event_t ev = BL_RX_NONE;
  for (size_t i = 0; i < v->length; ++i) {
    ev = bl_decoder_push(&d, v->bytes[i]);
  }
  assert(ev == BL_RX_PACKET && d.packet[0] == BL_CMD_CAP && d.packet_length == 4U);

  /* Um bit trocado derruba o CRC. */
  uint8_t corrupt[BL_FRAME_MAX];
  memcpy(corrupt, v->bytes, v->length);
  corrupt[3] ^= 0x01U;
  for (size_t i = 0; i < v->length; ++i) {
    ev = bl_decoder_push(&d, corrupt[i]);
  }
  assert(ev == BL_RX_ERROR);

  /* Quadro longo demais é descartado inteiro e o seguinte ainda é aceito. */
  for (size_t i = 0; i < (BL_FRAME_MAX + 20U); ++i) {
    assert(bl_decoder_push(&d, 0x55U) == BL_RX_NONE);
  }
  assert(bl_decoder_push(&d, 0x00U) == BL_RX_ERROR);
  for (size_t i = 0; i < v->length; ++i) {
    ev = bl_decoder_push(&d, v->bytes[i]);
  }
  assert(ev == BL_RX_PACKET);

  /* COBS: ida e volta com zeros e com bloco máximo. */
  uint8_t in[300];
  uint8_t enc[310];
  uint8_t dec[300];
  for (size_t i = 0; i < sizeof(in); ++i) {
    in[i] = (uint8_t)(((i % 97U) == 0U) ? 0U : (i & 0xFFU) | 1U);
  }
  const size_t ne = bl_cobs_encode(in, sizeof(in), enc, sizeof(enc));
  assert(ne > 0U && memchr(enc, 0, ne) == NULL);
  assert(bl_cobs_decode(enc, ne, dec, sizeof(dec)) == sizeof(in));
  assert(memcmp(in, dec, sizeof(in)) == 0);
  memset(in, 0xAB, 254);
  const size_t nb = bl_cobs_encode(in, 254U, enc, sizeof(enc));
  assert(bl_cobs_decode(enc, nb, dec, sizeof(dec)) == 254U && memcmp(in, dec, 254U) == 0);
  assert(bl_cobs_encode(in, 254U, enc, 10U) == 0U); /* sem espaço */
  const uint8_t bad_cobs[] = {0x05, 0x01};
  assert(bl_cobs_decode(bad_cobs, sizeof(bad_cobs), dec, sizeof(dec)) == 0U);

  /* Payload acima do máximo não é enquadrado. */
  uint8_t big[BL_MAX_PAYLOAD + 1U] = {0};
  uint8_t frame[BL_FRAME_MAX + 8U];
  assert(bl_frame_encode(1U, 0U, big, sizeof(big), frame, sizeof(frame)) == 0U);

  /* CRC-16/CCITT-FALSE de "123456789" = 0x29B1. */
  assert(bl_crc16((const uint8_t *)"123456789", 9U) == 0x29B1U);
}

int main(void) {
  load_vectors("tests/vectors/bench_link.txt");
  test_vectors();
  test_commands();
  test_decoder();
  return 0;
}
