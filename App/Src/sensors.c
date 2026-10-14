/**
 * @file sensors.c
 * @brief Inicialização do MTF-01 e do BNO08x no perfil dos motores
 *        (contrato em sensors.h).
 */
#include "sensors.h"

/** @brief Inicia os dois sensores independentemente (ver sensors.h). */
bool sensors_init(void) {
  bool mtf = mtf_start();
  bool bno = bno_start();
  return mtf && bno;
}
