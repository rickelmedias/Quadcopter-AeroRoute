/**
 * @file sensors.h
 * @brief Ponto único de inicialização do perfil de bancada dos motores.
 */
#ifndef SENSORS_H
#define SENSORS_H
#include <stdbool.h>
#include "mtf01.h"
#include "bno08x.h"

/**
 * @brief Inicia as tasks do MTF-01 e do BNO08x (só leitura e log).
 *
 * Perfil de teste dos motores: os sensores são lidos e registrados para ver
 * vibração e interferência com os motores girando, mas não há estimador nem
 * controle (sem LQG). Os motores são iniciados à parte por motor_test_start().
 *
 * Chamada em MX_FREERTOS_Init(), antes de o scheduler iniciar.
 *
 * @return true se as duas tasks foram criadas.
 */
bool sensors_init(void);
#endif
