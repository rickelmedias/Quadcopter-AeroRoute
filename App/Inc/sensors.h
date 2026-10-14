/**
 * @file sensors.h
 * @brief Ponto único de inicialização do sensor ativo no perfil de bancada.
 */
#ifndef SENSORS_H
#define SENSORS_H
#include <stdbool.h>
#include "bno08x.h"

/**
 * @brief Inicia somente a task de aquisição do BNO08x.
 *
 * Este perfil existe para a caracterização isolada do BNO08x a 400 Hz: não
 * inicializa MTF-01, captura, deslocamento, filtros ou estimadores.
 *
 * Chamada em MX_FREERTOS_Init(), antes de o scheduler iniciar.
 *
 * @return true se a task do BNO08x foi criada.
 */
bool sensors_init(void);
#endif
