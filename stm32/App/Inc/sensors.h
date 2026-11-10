/**
 * @file sensors.h
 * @brief Ponto único de inicialização dos sensores.
 */
#ifndef SENSORS_H
#define SENSORS_H
#include <stdbool.h>
#include "bno08x.h"
#include "mtf01.h"

/**
 * @brief Cria as tasks do MTF-01 e do BNO08x.
 *
 * Cada módulo é dono do seu periférico, então os dois são iniciados
 * independentemente: a falha de um não impede o outro, e o log mostra qual
 * faltou. A combinação das leituras fica no controle de voo
 * (control/meas.h).
 *
 * Chamada em MX_FREERTOS_Init(), antes de o scheduler iniciar.
 *
 * @return true se os dois sensores foram criados.
 */
bool sensors_init(void);
#endif
