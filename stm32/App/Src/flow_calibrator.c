/**
 * @file flow_calibrator.c
 * @brief Implementação da calibração de escala do fluxo (contratos em
 *        flow_calibrator.h).
 */
#include "flow_calibrator.h"
#include <math.h>

/** @brief Zera os acumuladores (ver flow_calibrator.h). */
void flow_cal_reset(flow_calibrator_t *c) {
  for (int i = 0; i < 2; ++i) {
    c->axis[i].weighted_scalar_sum = 0.0;
    c->axis[i].residual_sum = 0.0;
    c->axis[i].samples = 0;
  }
}

/** @brief Oferece uma amostra para um eixo (ver flow_calibrator.h). */
bool flow_cal_add_sample(flow_calibrator_t *c, int axis, float raw_flow,
                          float expected_flow, float yaw_rate) {
  if (axis < 0 || axis > 1) return false;
  if (!isfinite(raw_flow) || !isfinite(expected_flow) || !isfinite(yaw_rate)) return false;
  if (fabsf(yaw_rate) > FLOW_CAL_MAX_YAW_RAD_S) return false;
  /* Os dois lados precisam girar rápido o bastante para ser sinal e não
     ruído - como o ArduPilot, que exige o limite tanto na taxa do corpo
     quanto na taxa do fluxo. */
  if (fabsf(raw_flow) < FLOW_CAL_MIN_RATE_RAD_S) return false;
  if (fabsf(expected_flow) < FLOW_CAL_MIN_RATE_RAD_S) return false;

  /* Acúmulo ponderado incremental, matematicamente idêntico à versão em duas
     passadas do ArduPilot (soma os resíduos e depois faz a média do melhor
     fator de cada amostra, ponderada pelo próprio resíduo), mas sem guardar
     todas as amostras: a média ponderada se reduz a duas somas correntes. */
  const float residual = expected_flow - raw_flow; /* quanto o fator 1 erra */
  const float scalar = expected_flow / raw_flow;   /* o fator que acerta esta amostra */
  const float weight = fabsf(residual);
  c->axis[axis].weighted_scalar_sum += (double)(scalar * weight);
  c->axis[axis].residual_sum += (double)weight;
  ++c->axis[axis].samples;
  return true;
}

/** @brief Fator de correção de um eixo, com limite de faixa (ver flow_calibrator.h). */
bool flow_cal_result(const flow_calibrator_t *c, int axis, float *scalar_out,
                      float *residual_out) {
  if (axis < 0 || axis > 1 || c->axis[axis].samples == 0) return false;

  float scalar;
  if (c->axis[axis].residual_sum < 1e-9) {
    /* Todas as amostras aceitas já batiam exatamente com fator 1 - não há o
       que corrigir, e a média ponderada abaixo seria 0/0. */
    scalar = 1.0F;
  } else {
    scalar = (float)(c->axis[axis].weighted_scalar_sum / c->axis[axis].residual_sum);
  }
  if (!isfinite(scalar) || scalar < FLOW_CAL_SCALE_MIN || scalar > FLOW_CAL_SCALE_MAX) {
    return false;
  }
  if (scalar_out) *scalar_out = scalar;
  if (residual_out) *residual_out = (float)(c->axis[axis].residual_sum / c->axis[axis].samples);
  return true;
}

/** @brief Fator de correção de um eixo, sem limite de faixa (ver flow_calibrator.h). */
bool flow_cal_raw_scalar(const flow_calibrator_t *c, int axis, float *scalar_out) {
  if (axis < 0 || axis > 1 || c->axis[axis].samples == 0) return false;
  float scalar = (c->axis[axis].residual_sum < 1e-9)
    ? 1.0F
    : (float)(c->axis[axis].weighted_scalar_sum / c->axis[axis].residual_sum);
  if (!isfinite(scalar)) return false;
  if (scalar_out) *scalar_out = scalar;
  return true;
}
