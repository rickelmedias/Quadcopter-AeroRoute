/**
 * @file sensors.c
 * @brief Inicialização dos sensores e da estimação (contrato em sensors.h).
 */
#include "sensors.h"
#include "capture.h"

/** @brief Inicia captura, sensores e deslocamento (ver sensors.h). */
bool sensors_init(void) {
  if (!capture_start()) return false;
  /* Inicia os dois sensores independentemente do resultado do outro e só
     depois informa o resultado combinado: um sensor que não subiu não é
     motivo para deixar o outro parado, e o log mostra qual faltou. */
  bool mtf = mtf_start();
  bool bno = bno_start();
  /* O deslocamento consome as duas fotografias, então é iniciado por último
     e só quando os dois produtores existem - sem eles não há o que
     integrar. */
  bool disp = (mtf && bno) ? displacement_start() : false;
  return mtf && bno && disp;
}
