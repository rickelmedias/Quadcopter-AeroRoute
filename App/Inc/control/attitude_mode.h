/**
 * @file attitude_mode.h
 * @brief Modo de teste "só atitude e altura": o LQG ignora a posição horizontal.
 *
 * No modo completo o LQG do TCC também segura x e y, usando o fluxo óptico do
 * MTF-01. Para testar primeiro só a estabilização (rolagem, arfagem, guinada e
 * altura) este modo tira a posição horizontal da malha:
 *
 * - ::attitude_mode_mask_gains() zera, no ganho do LQR, as colunas dos estados
 *   x, y, vx e vy e, no pré-compensador, as colunas das referências x e y. A
 *   lei de controle deixa de depender deles; o resto (φ, θ, ψ, z, vz, p, q, r)
 *   fica exatamente como o Simulink exportou.
 * - ::attitude_mode_repeat_prediction() troca as medidas vx e vy pela própria
 *   predição do Kalman. A inovação delas fica zero, então um fluxo óptico ainda
 *   não validado nunca contamina a estimativa de φ e θ.
 *
 * Não são os ganhos validados no MATLAB: são os mesmos com os termos
 * horizontais removidos. Código puro, testado em tests/test_attitude_mode.c.
 */
#ifndef CONTROL_ATTITUDE_MODE_H
#define CONTROL_ATTITUDE_MODE_H

#include "control/lqg_ert.h"

/**
 * @brief Zera os termos horizontais (x, y, vx, vy) do LQR e do pré-compensador.
 *
 * @param[in,out] gains  Cópia dos ganhos em uso (nunca os ganhos originais).
 */
void attitude_mode_mask_gains(lqg_gains_t *gains);

/**
 * @brief Substitui as medidas vx e vy pelo valor previsto pelo Kalman.
 *
 * Chamar antes de ::lqg_ert_output(). No primeiro passo (a estimativa parte de
 * zero) a medida vira 0.
 *
 * @param[in]     s  Filtro (lê a predição x̂[k|k-1]).
 * @param[in,out] y  Medidas do passo.
 */
void attitude_mode_repeat_prediction(const lqg_ert_t *s, lqg_real_t y[LQG_NY]);

#endif
