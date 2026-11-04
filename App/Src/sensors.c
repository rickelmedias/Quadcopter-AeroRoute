/**
 * @file sensors.c
 * @brief Inicialização dos sensores (contrato em sensors.h).
 */
#include "sensors.h"

/** @brief Inicia MTF-01 e BNO08x (ver sensors.h). */
bool sensors_init(void) {
  /* Inicia os dois independentemente do resultado do outro e só depois
     informa o resultado combinado. */
  const bool mtf = mtf_start();
  const bool bno = bno_start();
  return mtf && bno;
}
