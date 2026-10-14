/**
 * @file sensors.c
 * @brief Inicialização do BNO08x no perfil de bancada (contrato em sensors.h).
 */
#include "sensors.h"

/** @brief Inicia somente a aquisição do BNO08x (ver sensors.h). */
bool sensors_init(void) {
  return bno_start();
}
