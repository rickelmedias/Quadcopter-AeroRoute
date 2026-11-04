/**
 * @file dshot.c
 * @brief DShot600 no TIM4 com DMA burst (contrato em dshot.h).
 */
#include "dshot.h"
#include "tim.h"

/** DMA do update do TIM4 (DMA1 Stream6, canal 2), iniciado em tim.c. */
extern DMA_HandleTypeDef hdma_tim4_up;

/** Bits de um pacote DShot. */
#define FRAME_BITS 16U
/** Períodos extras em zero no fim, para a linha voltar a nível baixo. */
#define PAD_BITS 2U
/** Flags do Stream6 no HIFCR: FEIF, DMEIF, TEIF, HTIF e TCIF. */
#define STREAM6_FLAGS (DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6 | \
                       DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6)

/** Um grupo de quatro CCR por período de bit, na ordem CCR1..CCR4. */
static uint16_t frame[(FRAME_BITS + PAD_BITS) * 4U];
/** Comparação do bit 0 (37,5% do período). Derivada do período do TIM4: com
    ARR=159 dá DShot600 (60); para DShot300 basta ARR=319 no CubeMX, sem
    mexer aqui. */
static uint16_t t0h;
/** Comparação do bit 1 (75% do período; 120 em DShot600). */
static uint16_t t1h;

void dshot_init(void) {
  DMA_Stream_TypeDef *s = hdma_tim4_up.Instance;
  s->CR &= ~DMA_SxCR_EN;
  while (s->CR & DMA_SxCR_EN) {}
  DMA1->HIFCR = STREAM6_FLAGS;
  s->PAR = (uint32_t)&TIM4->DMAR;
  s->M0AR = (uint32_t)frame;
  const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
  t0h = (uint16_t)(period * 3U / 8U);
  t1h = (uint16_t)(period * 3U / 4U);
  /* Cada pedido de DMA no update escreve 4 meias-palavras a partir do CCR1. */
  TIM4->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_UPDATE);
}

uint16_t dshot_packet(uint16_t value, bool telemetry) {
  uint16_t p = (uint16_t)(((value & 0x7FFU) << 1) | (telemetry ? 1U : 0U));
  uint16_t crc = (uint16_t)((p ^ (p >> 4) ^ (p >> 8)) & 0x0FU);
  return (uint16_t)((p << 4) | crc);
}

bool dshot_write(const uint16_t value[4], bool telemetry) {
  DMA_Stream_TypeDef *s = hdma_tim4_up.Instance;
  if (s->CR & DMA_SxCR_EN) return false;  /* quadro anterior em andamento */
  for (unsigned m = 0; m < 4U; ++m) {
    uint16_t p = dshot_packet(value[m], telemetry);
    for (unsigned b = 0; b < FRAME_BITS; ++b) {
      frame[b * 4U + m] = (p & (0x8000U >> b)) ? t1h : t0h;
    }
    for (unsigned b = FRAME_BITS; b < FRAME_BITS + PAD_BITS; ++b) frame[b * 4U + m] = 0;
  }
  DMA1->HIFCR = STREAM6_FLAGS;
  s->NDTR = (FRAME_BITS + PAD_BITS) * 4U;
  s->CR |= DMA_SxCR_EN;
  return true;
}

uint16_t dshot_to_permille(uint16_t value) {
  if (value < DSHOT_THROTTLE_MIN) return 0U;
  if (value > DSHOT_THROTTLE_MAX) value = DSHOT_THROTTLE_MAX;
  const uint32_t span = DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN;
  return (uint16_t)(((uint32_t)(value - DSHOT_THROTTLE_MIN) * 1000U + span / 2U) / span);
}

uint16_t dshot_from_permille(uint16_t permille) {
  if (permille == 0U) return 0U;
  if (permille > 1000U) permille = 1000U;
  return (uint16_t)(DSHOT_THROTTLE_MIN +
      ((uint32_t)permille * (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN) + 500U) / 1000U);
}
