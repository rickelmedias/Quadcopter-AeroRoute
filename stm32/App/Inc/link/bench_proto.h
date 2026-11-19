/**
 * @file bench_proto.h
 * @brief Mensagens do enlace USB de bancada (bench-link v1).
 *
 * Todos os campos multibyte são little-endian; float é IEEE 754 de 32 bits.
 * Tamanhos fixos (BL_LEN_*) são verificados pelo host contra este arquivo
 * (hmi/tests/test_contract.py). Especificação: docs/phase2/bench-link.md.
 *
 * Unidades: tempo em ms do tick do STM32; ângulos em rad; velocidade angular
 * em rad/s nos eixos do BNO085; aceleração linear em m/s^2 nos eixos do
 * BNO085, sem gravidade; distância do MTF-01 em mm no eixo do sensor; fluxo em
 * cm/s a 1 m (não é velocidade); aceleração dos motores em décimos de %.
 */
#ifndef LINK_BENCH_PROTO_H
#define LINK_BENCH_PROTO_H

#include "link/bench_frame.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BL_PROTO_MAJOR 1U
#define BL_PROTO_MINOR 4U

/* ---- Dispositivo -> host ------------------------------------------------- */
#define BL_MSG_HELLO  0x01U
#define BL_MSG_STATUS 0x02U
#define BL_MSG_IMU    0x10U
#define BL_MSG_RANGE  0x11U
#define BL_MSG_MOTOR  0x12U
#define BL_MSG_ESC    0x13U
#define BL_MSG_POWER  0x14U
#define BL_MSG_SYS    0x15U
#define BL_MSG_CTRL   0x16U
#define BL_MSG_TEXT   0x20U
#define BL_MSG_ACK    0x21U

/* ---- Host -> dispositivo ------------------------------------------------- */
#define BL_CMD_HELLO     0x80U
#define BL_CMD_HEARTBEAT 0x81U
#define BL_CMD_ARM       0x82U
#define BL_CMD_STOP      0x83U
#define BL_CMD_MOTOR_SET 0x84U
#define BL_CMD_CAP       0x86U
#define BL_CMD_RATE      0x87U
#define BL_CMD_DIRECTION 0x88U
#define BL_CMD_BEEP      0x89U
#define BL_CMD_SWEEP     0x8AU
#define BL_CMD_STREAM    0x8BU
#define BL_CMD_POWER_CAL 0x8CU
#define BL_CMD_ESC_TELEM 0x8DU
#define BL_CMD_CTRL        0x90U
#define BL_CMD_CTRL_PARAMS 0x91U
#define BL_CMD_CTRL_GAINS  0x92U

/* ---- Tamanhos dos payloads, em bytes ------------------------------------- */
#define BL_LEN_HELLO         40U
#define BL_LEN_STATUS        32U
#define BL_LEN_IMU           72U
#define BL_LEN_RANGE         42U
#define BL_LEN_MOTOR         22U
#define BL_LEN_ESC           23U
#define BL_LEN_POWER         21U
#define BL_LEN_SYS           68U
#define BL_LEN_CTRL          96U
#define BL_LEN_TEXT_MAX      65U
#define BL_LEN_ACK           5U
#define BL_LEN_CMD_MOTOR_SET 9U
#define BL_LEN_CMD_CAP       2U
#define BL_LEN_CMD_RATE      2U
#define BL_LEN_CMD_DIRECTION 3U
#define BL_LEN_CMD_BEEP      1U
#define BL_LEN_CMD_SWEEP     9U
#define BL_LEN_CMD_STREAM    2U
#define BL_LEN_CMD_POWER_CAL 17U
#define BL_LEN_CMD_ESC_TELEM 3U
#define BL_LEN_CMD_CTRL        1U
#define BL_LEN_CMD_CTRL_PARAMS 37U
#define BL_LEN_CMD_CTRL_GAINS  32U
/** Texto livre de TEXT, sem terminador. */
#define BL_TEXT_MAX          60U
/** Identidade do firmware em HELLO (ASCII, preenchido com zeros). */
#define BL_FIRMWARE_ID_LEN   24U
/** Variante do build em HELLO (ASCII, preenchido com zeros). */
#define BL_VARIANT_LEN       8U

/* ---- Bits ----------------------------------------------------------------- */
#define BL_STATUS_ARMED        0x01U
#define BL_STATUS_SWEEPING     0x02U
#define BL_STATUS_HEARTBEAT_OK 0x04U
#define BL_STATUS_RX_BLOCKED   0x08U
#define BL_STATUS_ESC_TELEM    0x10U
#define BL_STATUS_STREAM       0x20U
#define BL_STATUS_POWER_LOW    0x40U

#define BL_IMU_ONLINE 0x01U

#define BL_RANGE_VALID 0x01U
#define BL_FLOW_VALID  0x02U

#define BL_MOTOR_ARMED 0x01U

#define BL_POWER_VBAT_READ    0x01U
#define BL_POWER_CURRENT_READ 0x02U
#define BL_POWER_VBAT_CAL     0x04U
#define BL_POWER_CURRENT_CAL  0x08U

/** Capacidades em HELLO. */
#define BL_CAP_MOTORS    0x0001U
#define BL_CAP_ESC_KISS  0x0002U
#define BL_CAP_POWER_ADC 0x0004U
#define BL_CAP_CONTROL   0x0008U

/** Ações de BL_CMD_CTRL. */
#define BL_CTRL_ARM     1U
#define BL_CTRL_TAKEOFF 2U
#define BL_CTRL_LAND    3U

/** Bit 0 do campo flags de BL_CMD_CTRL_PARAMS: ajuste lento do hover ligado. */
#define BL_CTRL_PARAM_AUTOTRIM 0x01U
/** Bit 1: usa o EKF (senão, o Kalman estacionário do modelo gerado). */
#define BL_CTRL_PARAM_USE_EKF  0x02U

/** Resultado de um comando (ACK). */
typedef enum {
  BL_RESULT_OK = 0,       /**< Aceito. */
  BL_RESULT_REJECTED = 1, /**< Bem formado, mas recusado no estado atual. */
  BL_RESULT_INVALID = 2,  /**< Tamanho ou faixa inválidos; a saída foi parada. */
  BL_RESULT_UNKNOWN = 3   /**< Id desconhecido; a saída foi parada. */
} bl_result_t;

/** Níveis de TEXT. */
typedef enum { BL_TEXT_INFO = 0, BL_TEXT_WARN = 1, BL_TEXT_ERROR = 2 } bl_text_level_t;

/* ---- Telemetria ------------------------------------------------------------ */

typedef struct {
  uint16_t capabilities;
  uint32_t uptime_ms;
  char firmware_id[BL_FIRMWARE_ID_LEN];
  char variant[BL_VARIANT_LEN];
} bl_hello_t;

typedef struct {
  uint32_t t_ms;
  uint8_t flags; /**< BL_STATUS_*. */
  uint8_t owner;
  uint16_t cap_permille;
  uint16_t rate_permille_s;
  uint16_t target_permille[4];
  uint8_t reversed;
  uint8_t direction_sent;
  uint8_t direction_known;
  uint8_t commands_pending;
  uint32_t tx_failures;
  uint32_t heartbeat_age_ms;
  uint8_t bno_divider;
  uint8_t reserved;
} bl_status_t;

typedef struct {
  uint32_t t_ms;
  float q[4];         /**< w, x, y, z. */
  float gyro[3];      /**< rad/s. */
  float lin_accel[3]; /**< m/s^2, sem gravidade. */
  uint8_t accuracy[3];
  uint8_t flags; /**< BL_IMU_*. */
  uint32_t count[3];
  uint32_t gaps[3];
} bl_imu_t;

typedef struct {
  uint32_t t_ms;
  uint32_t sensor_ms;
  uint32_t distance_mm;
  int16_t flow_x;
  int16_t flow_y;
  uint8_t quality;
  uint8_t distance_status;
  uint8_t flow_status;
  uint8_t strength;
  uint8_t precision;
  uint8_t flags; /**< BL_RANGE_VALID | BL_FLOW_VALID. */
  uint32_t frames;
  uint32_t crc_errors;
  uint32_t gaps;
  uint32_t uart_errors;
  uint32_t dropped_bytes;
} bl_range_t;

typedef struct {
  uint32_t t_ms;
  uint8_t flags; /**< BL_MOTOR_*. */
  uint16_t permille[4];
  uint16_t dshot[4];
  uint8_t telemetry_mask; /**< Motor com pedido KISS neste quadro. */
} bl_motor_t;

typedef struct {
  uint32_t t_ms;
  uint8_t motor; /**< 0..3. */
  int8_t temperature_c;
  uint16_t voltage_cv;
  uint16_t current_ca;
  uint16_t consumption_mah;
  uint16_t erpm_100; /**< eRPM / 100. */
  uint32_t frames_ok;
  uint32_t timeouts;
  uint8_t reserved;
} bl_esc_t;

typedef struct {
  uint32_t t_ms;
  float vbat_pin_v;
  float current_pin_v;
  float vbat_v;    /**< Só vale com BL_POWER_VBAT_CAL. */
  float current_a; /**< Só vale com BL_POWER_CURRENT_CAL. */
  uint8_t flags;   /**< BL_POWER_*. */
} bl_power_t;

typedef struct {
  uint32_t t_ms;
  uint32_t telemetry_dropped;
  uint32_t heap_free;
  uint32_t link_packets;
  uint32_t link_errors;
  uint32_t link_overflows;
  uint32_t esc_crc_errors;
  uint32_t esc_length_errors;
  uint32_t esc_unsolicited;
  uint32_t esc_uart_errors;
  uint32_t esc_timeouts[4];
  uint32_t bno_errors;
  uint32_t bno_resets;
  uint8_t bno_fault;
  uint8_t reserved[3];
} bl_sys_t;

/**
 * Controle LQR (modo com cabo), 20 Hz. Eixos NED/FRD, SI. @c y são as medidas
 * que o modelo recebeu [z, phi, theta, psi, u, v, p, q, r] (z em m para baixo,
 * ângulos em rad, velocidades em m/s e rad/s); @c pos e @c vel são a estimativa
 * do Kalman (x frente, y direita, z para baixo).
 */
typedef struct {
  uint32_t t_ms;
  uint8_t state; /**< 0 desarmado, 1 armado, 2 subindo, 3 subida, 4 hover, 5 pouso. */
  uint8_t flags; /**< Bits de lqg_ctrl.h (LQG_TLM_*). */
  uint8_t fail;  /**< Motivo da última parada (LQG_FAIL_*). */
  float y[9];
  float pos[3];
  float vel[3];
  float h_ref; /**< Referência de altura, m para cima. */
  float trim;  /**< Ajuste lento do hover, ‰. */
  float base;  /**< Aceleração base, ‰ (antes da compensação da bateria). */
  float ref_x; /**< Referência x local, m (desde 1.4). */
  float ref_y; /**< Referência y local, m. */
  float ref_psi; /**< Referência de guinada, rad. */
  float vbat_factor; /**< Fator da compensação da bateria aplicado à saída (1 sem medida). */
} bl_ctrl_t;

typedef struct {
  uint8_t command;
  uint8_t seq;
  uint8_t result; /**< ::bl_result_t. */
  uint16_t value;
} bl_ack_t;

size_t bl_encode_hello(const bl_hello_t *m, uint8_t out[BL_MAX_PAYLOAD]);
size_t bl_encode_status(const bl_status_t *m, uint8_t out[BL_MAX_PAYLOAD]);
size_t bl_encode_imu(const bl_imu_t *m, uint8_t out[BL_MAX_PAYLOAD]);
size_t bl_encode_range(const bl_range_t *m, uint8_t out[BL_MAX_PAYLOAD]);
size_t bl_encode_motor(const bl_motor_t *m, uint8_t out[BL_MAX_PAYLOAD]);
size_t bl_encode_esc(const bl_esc_t *m, uint8_t out[BL_MAX_PAYLOAD]);
size_t bl_encode_power(const bl_power_t *m, uint8_t out[BL_MAX_PAYLOAD]);
size_t bl_encode_sys(const bl_sys_t *m, uint8_t out[BL_MAX_PAYLOAD]);
size_t bl_encode_ctrl(const bl_ctrl_t *m, uint8_t out[BL_MAX_PAYLOAD]);
size_t bl_encode_ack(const bl_ack_t *m, uint8_t out[BL_MAX_PAYLOAD]);
/** TEXT: t_ms, nível e até ::BL_TEXT_MAX caracteres (cortados). */
size_t bl_encode_text(uint32_t t_ms, uint8_t level, const char *text, uint8_t out[BL_MAX_PAYLOAD]);

/* ---- Comandos -------------------------------------------------------------- */

/** @brief Comando já validado. Campos não usados ficam em zero. */
typedef struct {
  uint8_t id;  /**< BL_CMD_*. */
  uint8_t seq; /**< Eco no ACK. */
  uint8_t mask;
  uint8_t flags;
  uint16_t value[4];
  uint16_t step;
  uint32_t hold_ms;
  float calib[4];
  /** CTRL_PARAMS: altura m, hover ‰, limite ‰, subida m/s, descida m/s, rampa ‰/s, ganho trim ‰/(m·s), ganho XY 1/s, ganho guinada 1/s. */
  float ctrl[9];
  /** CTRL_GAINS: multiplicadores z, w, att, rate, yaw, yaw_rate, xy, vel. */
  float gains[8];
} bl_command_t;

/**
 * @brief Interpreta e valida um pacote de comando (id, seq, payload).
 *
 * Faixas: máscara 1..15; aceleração 0..1000; limite 0..1000; rampa 10..1000;
 * direção com bits fora da máscara rejeitada; varredura com passo 1..1000 e
 * espera 100..30000 ms; STREAM com divisor 1..16; ESC_TELEM com período
 * 4..1000 ms; POWER_CAL com valores finitos e ganhos/limite >= 0; CTRL com
 * ação 1..3; CTRL_PARAMS e CTRL_GAINS com valores finitos dentro dos
 * limites de control/lqg_ctrl.h.
 *
 * @return ::BL_RESULT_OK, ::BL_RESULT_INVALID ou ::BL_RESULT_UNKNOWN.
 */
bl_result_t bl_command_parse(const uint8_t *packet, size_t length, bl_command_t *out);

#endif
