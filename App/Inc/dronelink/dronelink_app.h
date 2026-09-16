/**
 * @file dronelink_app.h
 * @brief Task do DroneLink: comunicação MAVLink 2 com a Raspberry Pi na USART1.
 */
#ifndef DRONELINK_APP_H
#define DRONELINK_APP_H

#include <stdbool.h>

/**
 * @brief Cria a task do DroneLink.
 *
 * A task inicia o transporte na USART1, obtém a identidade de boot na flash
 * externa, inicia o protocolo e então repete: recebe bytes (espera até 5 ms),
 * entrega ao protocolo, executa as tarefas periódicas e envia o que estiver
 * na fila. Se o transporte não puder ser iniciado, a task se encerra.
 *
 * @return true se a task foi criada.
 */
bool dronelink_app_init(void);

#endif
