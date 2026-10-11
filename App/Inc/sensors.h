/**
 * @file sensors.h
 * @brief Ponto único de inicialização do sensor ativo no perfil de bancada.
 */
#ifndef SENSORS_H
#define SENSORS_H
#include <stdbool.h>
#include "mtf01.h"

/**
 * @brief Inicia somente a task de aquisição do MTF-01.
 *
 * Este perfil existe para a caracterização isolada do MTF-01: não inicializa
 * BNO08x, captura, deslocamento, filtros ou estimadores de posição.
 *
 * Chamada em MX_FREERTOS_Init(), antes de o scheduler iniciar.
 *
 * @return true se a task do MTF-01 foi criada.
 */
bool sensors_init(void);
#endif
