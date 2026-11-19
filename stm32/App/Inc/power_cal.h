/**
 * @file power_cal.h
 * @brief Conversão pura das leituras do ADC em tensão da bateria e corrente.
 *
 * V_bateria = V_PA1 · vbat_gain (razão do divisor, (R_alto + R_baixo)/R_baixo).
 * I_ESC = (V_PB0 − current_offset_v) · current_a_per_v.
 * A escala do CURR do KM55A II não é documentada: calibrar contra a corrente
 * da telemetria KISS ou um multímetro. Testado em tests/test_power_cal.c.
 */
#ifndef POWER_CAL_H
#define POWER_CAL_H

#include <stdbool.h>

/** @brief Calibração em uso. */
typedef struct {
  float vbat_gain;        /**< V/V. */
  float current_a_per_v;  /**< A/V. */
  float current_offset_v; /**< V no pino com corrente zero. */
  float vbat_min_v;       /**< Tensão mínima para armar; 0 = sem verificação. */
  bool vbat_valid;        /**< vbat_gain foi informado. */
  bool current_valid;     /**< current_a_per_v e offset foram informados. */
} power_cal_t;

/** @brief Uma leitura. Valores físicos só valem com o respectivo *_cal. */
typedef struct {
  float vbat_pin_v;    /**< V no PA1. */
  float current_pin_v; /**< V no PB0. */
  float vbat_v;        /**< V da bateria. */
  float current_a;     /**< A consumidos pelo ESC. */
  bool vbat_read;      /**< Conversão do PA1 concluída. */
  bool current_read;   /**< Conversão do PB0 concluída. */
  bool vbat_calibrated;       /**< vbat_v é válido. */
  bool current_calibrated;    /**< current_a é válido. */
} power_sample_t;

/** @brief Calibração vazia: nada convertido, sem limite de armar. */
void power_cal_default(power_cal_t *config);

/** @brief Preenche vbat_v/current_a e os indicadores conforme @p config. */
void power_cal_apply(const power_cal_t *config, power_sample_t *sample);

/**
 * @brief Diz se a tensão permite armar.
 *
 * Sem limite (vbat_min_v <= 0) sempre permite. Com limite, exige tensão
 * calibrada, lida e acima do mínimo: limite configurado com leitura inválida
 * não arma.
 */
bool power_cal_arm_allowed(const power_cal_t *config, const power_sample_t *sample);

#endif
