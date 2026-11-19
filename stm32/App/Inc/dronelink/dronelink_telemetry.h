/**
 * @file dronelink_telemetry.h
 * @brief Telemetria do controle para a Raspberry Pi.
 *
 * - ATTITUDE e LOCAL_POSITION_NED a 5 Hz: a estimativa do EKF em mensagens
 *   padrão (z relativo ao chão do armamento: 0 no chão, −h a h metros).
 * - DRONE_CTRL_DEBUG a 10 Hz: medidas y, estimativa x̂, Δω aplicado, referência
 *   (x, y, z NED, ψ), aceleração dos motores M1..M4 e flags. Cada mensagem ocupa
 *   ~13 ms do enlace a 115200 bit/s.
 * - STATUSTEXT com os avisos do controle (descolou, pousando, cortes).
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
