/**
 * @file flight_ctrl.h
 * @brief Malha de controle LQG: sensores -> Kalman -> LQR -> motores.
 *
 * A task acorda a cada relatório de rotação do BNO08x (400 Hz), faz a
 * predição do Kalman, corrige com as medidas que chegaram (BNO sempre; altura
 * e velocidades do MTF-01 quando há frame novo do deslocamento), calcula
 * u = u0 - K (x_hat - x_ref) e converte cada rotação em DShot.
 *
 * O Kalman roda desde o boot, então a estimativa já está assentada quando o
 * controle é ligado. Comandos pela USB (além dos do teste de motores):
 *
 *   dry              ensaio a seco: LQG ligado, motores parados, linha EST no log
 *   fly              voo: só com LQG_FLIGHT_ENABLED = 1 em flight_config.h
 *   ref z <m>        altura desejada acima do chão (0 a REF_Z_MAX_M)
 *   ref xy <x> <y>   posição desejada, em m, no referencial do início
 *   ref yaw <graus>  guinada desejada, relativa à do momento do dry/fly
 *   lqg              mostra estado e contadores
 *   stop ou Enter    para tudo (igual ao teste de motores)
 *
 * Failsafes: inclinação acima de FAILSAFE_TILT_RAD, BNO sem dado por 20 ms,
 * porta USB fechada ou qualquer comando desconhecido cortam os motores.
 */
#ifndef FLIGHT_CTRL_H
#define FLIGHT_CTRL_H
#include <stdbool.h>

/**
 * @brief Cria a task do controle. Chamar depois de sensors_init() e
 *        motor_test_start().
 * @return true se a task foi criada.
 */
bool flight_ctrl_start(void);
#endif
