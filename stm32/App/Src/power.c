/**
 * @file power.c
 * @brief Leitura de bateria e corrente pelo ADC1 (contrato em power.h).
 */
#include "power.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

/** Período da leitura, em ms (10 Hz). */
#define PERIOD_MS       100U
/** Canal do ADC1 da bateria (PA1). */
#define VBAT_CHANNEL    1U
/** Canal do ADC1 da corrente (PB0). */
#define CURRENT_CHANNEL 8U
/** Espera máxima por uma conversão, em voltas do laço (~20 us bastam). */
#define EOC_TIMEOUT     10000U
/** Fundo de escala do ADC de 12 bits. */
#define ADC_FULL_SCALE  4095.0F

/** Última leitura; protegida por seção crítica. */
static power_sample_t latest = {NAN, NAN, false, false};

void power_get(power_sample_t *out) {
  taskENTER_CRITICAL();
  *out = latest;
  taskEXIT_CRITICAL();
}

#if POWER_VBAT_ENABLED || POWER_CURRENT_ENABLED

/** @brief Liga o ADC1 e põe os pinos usados em modo analógico. */
static void adc_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
#if POWER_VBAT_ENABLED
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER |= 3UL << (1U * 2U); /* PA1 analógico */
  GPIOA->PUPDR &= ~(3UL << (1U * 2U));
#endif
#if POWER_CURRENT_ENABLED
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  GPIOB->MODER |= 3UL << (0U * 2U); /* PB0 analógico */
  GPIOB->PUPDR &= ~(3UL << (0U * 2U));
#endif
  /* PCLK2 = 96 MHz / 4 = 24 MHz (o ADC aceita até 36 MHz). */
  ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;
  ADC1->CR1 = 0U;  /* 12 bits, sem varredura */
  ADC1->SQR1 = 0U; /* uma conversão por vez */
  /* 480 ciclos de amostragem nos dois canais: o divisor tem impedância alta. */
  ADC1->SMPR2 |= (7UL << (3U * VBAT_CHANNEL)) | (7UL << (3U * CURRENT_CHANNEL));
  ADC1->CR2 = ADC_CR2_ADON;
}

/**
 * @brief Converte um canal e devolve a média em volts no pino.
 * @param[in]  channel  Canal do ADC1.
 * @param[out] volts    Tensão média.
 * @return false se alguma conversão não terminou.
 */
static bool adc_read(uint32_t channel, float *volts) {
  uint32_t sum = 0U;
  ADC1->SQR3 = channel;
  for (unsigned n = 0U; n < POWER_ADC_SAMPLES; ++n) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    uint32_t wait = 0U;
    while ((ADC1->SR & ADC_SR_EOC) == 0U) {
      if (++wait > EOC_TIMEOUT)
        return false;
    }
    sum += ADC1->DR & 0xFFFU;
  }
  *volts = (float)sum / (float)POWER_ADC_SAMPLES / ADC_FULL_SCALE * POWER_ADC_VREF_V;
  return true;
}

/** @brief Task de leitura: média dos canais ligados a cada 100 ms. */
static void power_task(void *argument) {
  (void)argument;
  adc_init();
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    power_sample_t s = {NAN, NAN, false, false};
    float volts;
#if POWER_VBAT_ENABLED
    if (adc_read(VBAT_CHANNEL, &volts)) {
      s.voltage_v = volts * POWER_VBAT_DIVIDER;
      s.voltage_valid = true;
    }
#endif
#if POWER_CURRENT_ENABLED
    if (adc_read(CURRENT_CHANNEL, &volts)) {
      s.current_a = (volts - POWER_CURRENT_OFFSET_V) * POWER_CURRENT_A_PER_V;
      s.current_valid = true;
    }
#endif
    (void)volts;
    taskENTER_CRITICAL();
    latest = s;
    taskEXIT_CRITICAL();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));
  }
}

/** Controle da task de leitura (alocação estática). */
static StaticTask_t power_tcb;
/** Pilha da task de leitura, em words. */
static StackType_t power_stack[256];

bool power_start(void) {
  return xTaskCreateStatic(power_task, "Power", sizeof(power_stack) / sizeof(power_stack[0]), NULL,
                           tskIDLE_PRIORITY + 1, power_stack, &power_tcb) != NULL;
}

#else /* nenhuma leitura ligada */

bool power_start(void) {
  return true;
}

#endif
