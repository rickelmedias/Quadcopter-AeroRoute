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

/** Última leitura bruta; protegida por seção crítica. */
static power_sample_t latest = {NAN, NAN, NAN, NAN, false, false, false, false};
/** Calibração ativa; protegida por seção crítica. */
static power_cal_t active_cal = {0.0F, 0.0F, 0.0F, 0.0F, false, false};

void power_get(power_sample_t *out) {
  taskENTER_CRITICAL();
  *out = latest;
  const power_cal_t cal = active_cal;
  taskEXIT_CRITICAL();
  power_cal_apply(&cal, out);
}

void power_set_cal(const power_cal_t *cal) {
  taskENTER_CRITICAL();
  active_cal = *cal;
  taskEXIT_CRITICAL();
}

void power_get_cal(power_cal_t *out) {
  taskENTER_CRITICAL();
  *out = active_cal;
  taskEXIT_CRITICAL();
}

#if POWER_ADC_ENABLED

/** @brief Liga o ADC1 e põe PA1 e PB0 em modo analógico. */
static void adc_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
  GPIOA->MODER |= 3UL << (1U * 2U); /* PA1 analógico */
  GPIOA->PUPDR &= ~(3UL << (1U * 2U));
  GPIOB->MODER |= 3UL << (0U * 2U); /* PB0 analógico */
  GPIOB->PUPDR &= ~(3UL << (0U * 2U));
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
 * @return false se alguma conversão não terminou.
 */
static bool adc_read(uint32_t channel, float *volts) {
  uint32_t sum = 0U;
  bool ok = true;
  ADC1->SQR3 = channel;
  for (unsigned n = 0U; ok && (n < POWER_ADC_SAMPLES); ++n) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    uint32_t wait = 0U;
    while (ok && ((ADC1->SR & ADC_SR_EOC) == 0U)) {
      ok = ++wait <= EOC_TIMEOUT;
    }
    sum += ADC1->DR & 0xFFFU;
  }
  *volts = (float)sum / (float)POWER_ADC_SAMPLES / ADC_FULL_SCALE * POWER_ADC_VREF_V;
  return ok;
}

/** @brief Task de leitura: média dos dois canais a cada 100 ms. */
static void power_task(void *argument) {
  (void)argument;
  adc_init();
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    power_sample_t s = {NAN, NAN, NAN, NAN, false, false, false, false};
    float volts = 0.0F;
    if (adc_read(VBAT_CHANNEL, &volts)) {
      s.vbat_pin_v = volts;
      s.vbat_read = true;
    }
    if (adc_read(CURRENT_CHANNEL, &volts)) {
      s.current_pin_v = volts;
      s.current_read = true;
    }
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

#else

bool power_start(void) {
  return true;
}

#endif
