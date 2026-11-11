/**
 * @file dshot_codec.c
 * @brief Partes puras do DShot (pacote e conversão), sem HAL, para os testes
 *        de host. O envio pelo TIM4 está em dshot.c.
 */
#include "dshot.h"

uint16_t dshot_packet(uint16_t value, bool telemetry) {
  uint16_t p = (uint16_t)(((value & 0x7FFU) << 1) | (telemetry ? 1U : 0U));
  uint16_t crc = (uint16_t)((p ^ (p >> 4) ^ (p >> 8)) & 0x0FU);
  return (uint16_t)((p << 4) | crc);
}

uint16_t dshot_from_permille(uint16_t permille) {
  uint16_t value = 0U;
  if (permille != 0U) {
    const uint32_t p = (permille > 1000U) ? 1000U : (uint32_t)permille;
    const uint32_t steps = ((p * (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)) + 500U) / 1000U;
    value = (uint16_t)(DSHOT_THROTTLE_MIN + steps);
  }
  return value;
}
