/**
 * @file power.h
 * @brief Tensão da bateria (PA1) e corrente do ESC (PB0) pelo ADC1.
 *
 * **AINDA NÃO LIGADO NO HARDWARE.** As duas leituras ficam desligadas pelas
 * chaves ::POWER_VBAT_ENABLED e ::POWER_CURRENT_ENABLED: com elas em 0, nada
 * mexe no ADC nem nos pinos, nenhuma task é criada e as leituras aparecem
 * como inválidas (o SYS_STATUS informa "desconhecido" à Pi).
 *
 * Ligações previstas:
 *
 * | Sinal | Pino | Canal | Circuito |
 * |---|---|---|---|
 * | Bateria | PA1 | ADC1_IN1 | divisor de tensão (::POWER_VBAT_DIVIDER) |
 * | Corrente (CURR do ESC) | PB0 | ADC1_IN8 | resistor série de 1 kΩ |
 *
 * O ADC1 não está configurado no CubeMX (e o driver HAL do ADC não faz parte
 * do projeto), então este módulo programa os registradores diretamente:
 * conversão única, 12 bits, 480 ciclos de amostragem (adequado à impedância
 * do divisor) e média de ::POWER_ADC_SAMPLES leituras a 10 Hz.
 *
 * Para ligar: confira o divisor e a escala do sensor de corrente do ESC,
 * ajuste as constantes abaixo e mude a chave correspondente para 1.
 */
#ifndef POWER_H
#define POWER_H

#include <stdbool.h>

#ifndef POWER_VBAT_ENABLED
/** 0 = divisor da bateria não ligado ao PA1: não ler. */
#define POWER_VBAT_ENABLED 0
#endif

#ifndef POWER_CURRENT_ENABLED
/** 0 = saída CURR do ESC não ligada ao PB0: não ler. */
#define POWER_CURRENT_ENABLED 0
#endif

/** Tensão de referência do ADC (VDDA da Black Pill), em V. */
#define POWER_ADC_VREF_V 3.3F

/**
 * Razão do divisor da bateria: V_bateria = V_PA1 · razão.
 * PROVISÓRIO: 11 corresponde a 10 kΩ / 1 kΩ (até ~36 V). Use os resistores
 * montados: razão = (R_alto + R_baixo) / R_baixo.
 */
#define POWER_VBAT_DIVIDER 11.0F

/**
 * Escala da saída de corrente do ESC, em A por V no PB0.
 * PROVISÓRIO: confira no manual do ESC (o resistor de 1 kΩ só protege a
 * entrada; com a alta impedância do ADC ele não muda a tensão).
 */
#define POWER_CURRENT_A_PER_V 50.0F

/** Tensão do sensor de corrente com corrente zero, em V. */
#define POWER_CURRENT_OFFSET_V 0.0F

/** Leituras somadas em cada média. */
#define POWER_ADC_SAMPLES 16U

/** @brief Última leitura de bateria e corrente. */
typedef struct {
  /** Tensão da bateria, em V (NAN se inválida). */
  float voltage_v;
  /** Corrente consumida, em A (NAN se inválida). */
  float current_a;
  /** A tensão foi lida (chave ligada e ADC respondendo). */
  bool voltage_valid;
  /** A corrente foi lida. */
  bool current_valid;
} power_sample_t;

/**
 * @brief Prepara o ADC e cria a task de leitura, se alguma chave estiver ligada.
 * @return true se não há nada a fazer ou se a task foi criada.
 */
bool power_start(void);

/**
 * @brief Copia a última leitura, de forma segura entre tasks.
 * @param[out] out  Destino.
 */
void power_get(power_sample_t *out);

#endif
