/**
 * @file sensors.h
 * @brief Ponto único de inicialização dos sensores e da estimação.
 */
#ifndef SENSORS_H
#define SENSORS_H
#include <stdbool.h>
#include "bno08x.h"
#include "mtf01.h"
#include "displacement.h"

/**
 * @brief Inicia a captura (no build Capture) e as tasks dos sensores.
 *
 * Cada módulo é dono do seu periférico, então MTF-01 e BNO08x são iniciados
 * independentemente: a falha de um não impede o outro. A task de
 * deslocamento só é criada se os dois sensores subirem.
 *
 * Chamada em MX_FREERTOS_Init(), antes de o scheduler iniciar.
 *
 * @return true se tudo foi criado; false se a captura, algum sensor ou a
 *         task de deslocamento falhou.
 */
bool sensors_init(void);
#endif
