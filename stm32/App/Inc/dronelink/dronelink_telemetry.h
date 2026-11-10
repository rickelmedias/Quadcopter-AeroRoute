/**
 * @file dronelink_telemetry.h
 * @brief Telemetria do controle para a Raspberry Pi.
 *
 * - ATTITUDE e LOCAL_POSITION_NED a 5 Hz, sempre: a estimativa do Kalman em
 *   mensagens padrão, legíveis por qualquer ferramenta MAVLink.
 * - DRONE_CTRL_DEBUG na taxa do parâmetro DBG_RATE_HZ (0 = desligado): o
 *   passo completo do LQG (y, x̂, du, r, aceleração e flags), para ver a
 *   filtragem dos sensores e ajustar o controle em tempo real.
 *
 * A 115200 bit/s cada DRONE_CTRL_DEBUG ocupa ~13 ms do enlace; a taxa é
 * limitada a 50 Hz e o padrão de ajuste da Pi é 25 Hz.
 */
#ifndef DRONELINK_TELEMETRY_H
#define DRONELINK_TELEMETRY_H

#include <stdint.h>

/** @brief Zera os temporizadores. */
void dronelink_telemetry_init(void);

/**
 * @brief Envia o que estiver no horário.
 * @param[in] now_ms  Tick atual.
 */
void dronelink_telemetry_tick(uint32_t now_ms);

#endif
