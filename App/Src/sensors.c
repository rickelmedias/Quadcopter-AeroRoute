/**
 * @file sensors.c
 * @brief Inicialização do MTF-01 no perfil de bancada (contrato em sensors.h).
 */
#include "sensors.h"

/** @brief Inicia somente a aquisição do MTF-01 (ver sensors.h). */
bool sensors_init(void) {
  return mtf_start();
}
