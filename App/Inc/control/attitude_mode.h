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

/**
 * @brief Gira o par (@p a, @p b) de @p angle rad no plano horizontal.
 *
 * Com ψ do modelo (horário visto de cima, NED), girar de +ψ leva da proa
 * (frente, direita) ao local (norte, leste); de −ψ, do local à proa.
 */
void attitude_mode_rotate_pair(lqg_real_t *a, lqg_real_t *b, float angle);

/**
 * @brief Gira x, y, vx, vy de um vetor de estado de @p angle rad.
 *
 * O LQG do TCC foi linearizado com ψ = 0: nele, θ empurra só x e φ só y. Com
 * o drone virado, o mesmo θ empurra em outra direção do local, e o controle
 * de posição corrige para o lado errado (com ψ = 90° a correção sai girada de
 * 90°; acima disso ela realimenta o erro). Com o XY na malha, o voo põe os
 * estados horizontais, as medidas vx, vy e a referência x, y no referencial
 * da proa (−ψ) antes do LQG e devolve a estimativa ao local (+ψ) depois.
 */
void attitude_mode_rotate_horizontal(lqg_real_t x[LQG_NX], float angle);

#endif
