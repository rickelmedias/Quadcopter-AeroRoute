/**
 * @file ctrl_params.c
 * @brief Tabela de parâmetros do voo (contrato em control/ctrl_params.h).
 */
#include "control/ctrl_params.h"

#include "control/ctrl_port.h"
#include "flight_config.h"

#include <ctype.h>
#include <math.h>
#include <string.h>

/** Monta uma entrada da tabela a partir do campo de ::ctrl_params_t. */
#define PARAM(name, field, min, max, def, flags, help) \
  {name, offsetof(ctrl_params_t, field), min, max, def, flags, help}

/** Atalho para os sinais dos sensores. */
#define SIGN_PARAM(name, field, def, help) \
  PARAM(name, field, -1.0F, 1.0F, def, CTRL_PARAM_SIGN | CTRL_PARAM_DISARMED_ONLY, help)

/** Tabela de parâmetros. A ordem define o índice no MAVLink. */
static const ctrl_param_desc_t TABLE[] = {
    PARAM("CTL_W0", w0, 100.0F, 3000.0F, DEFAULT_CTL_W0, CTRL_PARAM_LIVE,
          "rotacao de hover w0 (rad/s)"),
    PARAM("MOT_W_AT_0", mot_w_at_0, 0.0F, 2000.0F, DEFAULT_MOT_W_AT_0, CTRL_PARAM_LIVE,
          "rotacao em 0% DShot (rad/s)"),
    PARAM("MOT_W_AT_100", mot_w_at_100, 100.0F, 8000.0F, DEFAULT_MOT_W_AT_100, CTRL_PARAM_LIVE,
          "rotacao em 100% DShot (rad/s)"),
    PARAM("MOT_MIN", mot_min, 0.0F, 0.3F, DEFAULT_MOT_MIN, CTRL_PARAM_LIVE,
          "aceleracao minima em voo (0..1)"),
    PARAM("MOT_MAX", mot_max, 0.1F, 1.0F, DEFAULT_MOT_MAX, CTRL_PARAM_LIVE,
          "aceleracao maxima em voo (0..1)"),
    PARAM("MOT_IDLE", mot_idle, 0.0F, 0.2F, DEFAULT_MOT_IDLE, CTRL_PARAM_LIVE,
          "aceleracao armado no chao (0..1)"),
    SIGN_PARAM("SNS_SGN_ROLL", sgn_roll, DEFAULT_SGN_ROLL, "sinal phi (pitch do BNO)"),
    SIGN_PARAM("SNS_SGN_PITCH", sgn_pitch, DEFAULT_SGN_PITCH, "sinal theta (roll do BNO)"),
    SIGN_PARAM("SNS_SGN_YAW", sgn_yaw, DEFAULT_SGN_YAW, "sinal guinada BNO->NED"),
    SIGN_PARAM("SNS_SGN_P", sgn_p, DEFAULT_SGN_P, "sinal p (gyro y do BNO)"),
    SIGN_PARAM("SNS_SGN_Q", sgn_q, DEFAULT_SGN_Q, "sinal q (gyro x do BNO)"),
    SIGN_PARAM("SNS_SGN_R", sgn_r, DEFAULT_SGN_R, "sinal gyro r BNO->NED"),
    SIGN_PARAM("SNS_SGN_VX", sgn_vx, DEFAULT_SGN_VX, "sinal vx fluxo->NED"),
    SIGN_PARAM("SNS_SGN_VY", sgn_vy, DEFAULT_SGN_VY, "sinal vy fluxo->NED"),
    PARAM("FLOW_SCALE_X", flow_scale_x, 0.5F, 2.0F, DEFAULT_FLOW_SCALE_X, CTRL_PARAM_LIVE,
          "escala do fluxo X"),
    PARAM("FLOW_SCALE_Y", flow_scale_y, 0.5F, 2.0F, DEFAULT_FLOW_SCALE_Y, CTRL_PARAM_LIVE,
          "escala do fluxo Y"),
    PARAM("FLOW_GYRO_X", flow_gyro_x, -2.0F, 2.0F, DEFAULT_FLOW_GYRO_X, CTRL_PARAM_LIVE,
          "compensacao gyro no fluxo X (0 desliga)"),
    PARAM("FLOW_GYRO_Y", flow_gyro_y, -2.0F, 2.0F, DEFAULT_FLOW_GYRO_Y, CTRL_PARAM_LIVE,
          "compensacao gyro no fluxo Y (0 desliga)"),
    PARAM("FLT_HOVER_H", hover_height, 0.1F, 1.5F, DEFAULT_HOVER_HEIGHT_M, CTRL_PARAM_LIVE,
          "altura padrao do hover (m)"),
    PARAM("FLT_Z_RATE", z_rate, 0.0F, 1.0F, DEFAULT_Z_RATE_M_S, CTRL_PARAM_LIVE,
          "rampa vertical (m/s, 0=degrau)"),
    PARAM("FLT_XY_RATE", xy_rate, 0.0F, 2.0F, DEFAULT_XY_RATE_M_S, CTRL_PARAM_LIVE,
          "rampa horizontal padrao (m/s)"),
    PARAM("FLT_YAW_RATE", yaw_rate, 0.0F, 3.0F, DEFAULT_YAW_RATE_RAD_S, CTRL_PARAM_LIVE,
          "rampa de guinada (rad/s, 0=degrau)"),
    PARAM("FLT_TILT_MAX", tilt_max, 0.2F, 1.2F, DEFAULT_TILT_MAX_RAD, CTRL_PARAM_LIVE,
          "inclinacao que corta motores (rad)"),
    PARAM("FLT_LAND_H", land_height, 0.02F, 0.3F, DEFAULT_LAND_HEIGHT_M, CTRL_PARAM_LIVE,
          "altura de fim do pouso (m)"),
    PARAM("FLT_DRY_RUN", dry_run, 0.0F, 1.0F, DEFAULT_DRY_RUN,
          CTRL_PARAM_BOOL | CTRL_PARAM_DISARMED_ONLY, "1 = LQG sem girar motores"),
    PARAM("DBG_RATE_HZ", debug_rate_hz, 0.0F, 50.0F, DEFAULT_DEBUG_RATE_HZ, CTRL_PARAM_LIVE,
          "DRONE_CTRL_DEBUG para a Pi (Hz)"),
    PARAM("LOG_EST_HZ", log_est_hz, 0.0F, 50.0F, DEFAULT_LOG_EST_HZ, CTRL_PARAM_LIVE,
          "linha EST no log USB (Hz)"),
    PARAM("SNS_ZERO_ROLL", zero_roll_deg, -180.0F, 180.0F, DEFAULT_ZERO_ROLL_DEG,
          CTRL_PARAM_DISARMED_ONLY, "roll cru do BNO nivelado (graus)"),
    PARAM("SNS_ZERO_PITCH", zero_pitch_deg, -90.0F, 90.0F, DEFAULT_ZERO_PITCH_DEG,
          CTRL_PARAM_DISARMED_ONLY, "pitch cru do BNO nivelado (graus)"),
};

/** Quantidade de entradas da tabela. */
#define TABLE_COUNT (sizeof(TABLE) / sizeof(TABLE[0]))

/** Valores atuais; protegidos por seção crítica. */
static ctrl_params_t values;

/** @brief Campo float de um parâmetro dentro de ::values. */
static float *field(size_t index) {
  return (float *)((uint8_t *)&values + TABLE[index].offset);
}

void ctrl_params_init(void) {
  CTRL_ENTER_CRITICAL();
  for (size_t i = 0U; i < TABLE_COUNT; ++i) {
    *field(i) = TABLE[i].def;
  }
  CTRL_EXIT_CRITICAL();
}

size_t ctrl_params_count(void) { return TABLE_COUNT; }

const ctrl_param_desc_t *ctrl_params_desc(size_t index) {
  return index < TABLE_COUNT ? &TABLE[index] : NULL;
}

int ctrl_params_find(const char *name, size_t len) {
  if (name == NULL) {
    return -1;
  }
  size_t n = 0U;
  while (n < len && n < CTRL_PARAM_NAME_MAX && name[n] != '\0') {
    ++n;
  }
  for (size_t i = 0U; i < TABLE_COUNT; ++i) {
    const char *candidate = TABLE[i].name;
    if (strlen(candidate) != n) {
      continue;
    }
    size_t k = 0U;
    while (k < n && toupper((unsigned char)name[k]) == candidate[k]) {
      ++k;
    }
    if (k == n) {
      return (int)i;
    }
  }
  return -1;
}

float ctrl_params_get(size_t index) {
  if (index >= TABLE_COUNT) {
    return NAN;
  }
  CTRL_ENTER_CRITICAL();
  const float v = *field(index);
  CTRL_EXIT_CRITICAL();
  return v;
}

ctrl_param_status_t ctrl_params_set(size_t index, float value, bool armed) {
  if (index >= TABLE_COUNT) {
    return CTRL_PARAM_UNKNOWN;
  }
  const ctrl_param_desc_t *d = &TABLE[index];
  if (armed && (d->flags & CTRL_PARAM_DISARMED_ONLY) != 0U) {
    return CTRL_PARAM_LOCKED;
  }
  if (!isfinite(value) || value < d->min || value > d->max) {
    return CTRL_PARAM_OUT_OF_RANGE;
  }
  if ((d->flags & CTRL_PARAM_SIGN) != 0U) {
    value = value >= 0.0F ? 1.0F : -1.0F;
  } else if ((d->flags & CTRL_PARAM_BOOL) != 0U) {
    value = value >= 0.5F ? 1.0F : 0.0F;
  }
  CTRL_ENTER_CRITICAL();
  *field(index) = value;
  CTRL_EXIT_CRITICAL();
  return CTRL_PARAM_OK;
}

void ctrl_params_snapshot(ctrl_params_t *out) {
  CTRL_ENTER_CRITICAL();
  *out = values;
  CTRL_EXIT_CRITICAL();
}

const char *ctrl_params_status_text(ctrl_param_status_t status) {
  switch (status) {
    case CTRL_PARAM_OK:
      return "ok";
    case CTRL_PARAM_UNKNOWN:
      return "parametro desconhecido";
    case CTRL_PARAM_OUT_OF_RANGE:
      return "fora dos limites";
    case CTRL_PARAM_LOCKED:
    default:
      return "so com o drone desarmado";
  }
}
