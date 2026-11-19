/**
 * @file power.h
 * @brief Tensão da bateria (PA1) e corrente do ESC (PB0) pelo ADC1.
 *
 * | Sinal | Pino | Canal | Circuito |
 * |---|---|---|---|
 * | Bateria | PA1 | ADC1_IN1 | divisor de tensão |
 * | Corrente (CURR do ESC) | PB0 | ADC1_IN8 | resistor série de 1 kΩ |
 *
 * O ADC1 não está no CubeMX; este módulo programa os registradores direto
 * (conversão única, 12 bits, 480 ciclos, média de ::POWER_ADC_SAMPLES a 10 Hz).
 *
 * A leitura bruta (V no pino) sempre é publicada. A conversão para V da
 * bateria e A do ESC usa uma calibração enviada pela IHM em tempo de execução
 * (power_cal.h); sem calibração os valores físicos aparecem como inválidos.
 * Pino sem nada ligado lê ruído: só confie na tensão bruta depois de ligar o
 * divisor e o CURR.
 */
#ifndef POWER_H
#define POWER_H

#include "power_cal.h"

#include <stdbool.h>

#ifndef POWER_ADC_ENABLED
/** 0 = não mexer no ADC nem em PA1/PB0. */
#define POWER_ADC_ENABLED 1
#endif

/** Tensão de referência do ADC (VDDA da Black Pill), em V. */
#define POWER_ADC_VREF_V  3.3F
/** Leituras somadas em cada média. */
#define POWER_ADC_SAMPLES 16U

/**
 * @brief Prepara o ADC e cria a task de leitura.
 * @return true se a task foi criada (ou se o ADC está desligado no build).
 */
bool power_start(void);

/** @brief Última leitura, já com a calibração ativa aplicada. */
void power_get(power_sample_t *out);

/** @brief Troca a calibração ativa (vale a partir da próxima leitura de power_get). */
void power_set_cal(const power_cal_t *cal);

/** @brief Calibração ativa. */
void power_get_cal(power_cal_t *out);

#endif
