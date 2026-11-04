/**
 * @file dronelink_tuning.h
 * @brief Ajuste do controle pela Raspberry Pi: parâmetros e ganhos do LQG.
 *
 * - Parâmetros (control/ctrl_params.h) pelo protocolo padrão do MAVLink:
 *   PARAM_REQUEST_LIST, PARAM_REQUEST_READ e PARAM_SET, respondidos com
 *   PARAM_VALUE. Um PARAM_SET recusado responde com o valor que continua em
 *   uso e um STATUSTEXT com o motivo.
 * - Ganhos (control/ctrl_gains.h) pelas mensagens do dialeto:
 *   DRONE_GAIN_ROW (uma linha por mensagem) e DRONE_GAIN_COMMIT (confirmar,
 *   abortar, restaurar os do Simulink ou só consultar), respondidos com
 *   DRONE_GAIN_ACK.
 *
 * Roda dentro da task do DroneLink.
 */
#ifndef DRONELINK_TUNING_H
#define DRONELINK_TUNING_H

#include "dronelink/dronelink_mavlink.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief Zera o envio da lista de parâmetros. */
void dronelink_tuning_init(void);

/**
 * @brief Trata uma mensagem de ajuste.
 * @param[in] message  Mensagem da Pi.
 * @return true se a mensagem era de ajuste (tratada aqui).
 */
bool dronelink_tuning_handle(const mavlink_message_t *message);

/**
 * @brief Envia aos poucos (um a cada 8 ms) a lista de parâmetros pedida por
 *        PARAM_REQUEST_LIST.
 * @param[in] now_ms  Tick atual.
 */
void dronelink_tuning_tick(uint32_t now_ms);

#endif
