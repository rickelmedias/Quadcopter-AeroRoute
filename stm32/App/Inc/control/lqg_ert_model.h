/**
 * @file lqg_ert_model.h
 * @brief Modelo e ganhos do TCC, exatamente como exportados pelo Simulink.
 *
 * As tabelas ficam em lqg_ert_model.c, GERADO por `tools/ert_import.py` a
 * partir de `reference/STM32_ert_rtw/STM32_data.c` (Simulink Coder 24.1,
 * modelo "STM32" versão 2.17). Para trocar o modelo, exporte de novo no
 * Simulink, copie os fontes para `reference/STM32_ert_rtw` e rode o script.
 *
 * @par Modelo
 * Linearização no hover, discretizada com ZOH em Ts = 10 ms. Estados em NED
 * (x frente, y direita, z para baixo). O modelo contínuo recuperado de A tem
 * um pequeno amortecimento de -0,001 1/s em x e y, que torna esses dois
 * estados detectáveis para o projeto do Kalman (eles não são medidos).
 *
 * @par Entradas e rotação de hover
 * As entradas são os desvios da rotação de cada motor em relação ao hover, em
 * rad/s. Na linearização, o termo de B em vz vale g / (2 ω0)
 * independentemente da massa e do Kt; com o B exportado
 * (b_z = 6,9872e-3 1/s² por rad/s) isso dá ω0 ≈ 702 rad/s (~6700 rpm). Esse é o
 * valor padrão do parâmetro `CTL_W0` (control/ctrl_params.h); confira com o
 * ω0 do script do MATLAB.
 *
 * @par Ordem dos motores (colunas de B)
 * Vista de cima, nariz para cima: M1 frente-esquerda (horário), M2
 * frente-direita (anti-horário), M3 trás-direita (horário), M4 trás-esquerda
 * (anti-horário). M1..M4 são PB6..PB9.
 *
 * @par Como os ganhos foram projetados
 * Os ganhos exportados são reproduzidos com erro relativo < 1e-11 por (ver
 * `tools/pi/dronectl/design.py`):
 * - LQR: `Kd = dlqr(A, B, Q, R)`, Q = diag(5, 5, 500, 500, 500, 50, 5, 5, 5,
 *   10, 10, 5) e R = 0,1·I.
 * - Pré-compensador: `Kfd = inv(Cr (I - A + B Kd)^-1 B)`, com Cr escolhendo
 *   x, y, z e ψ (ganho DC unitário da referência).
 * - Kalman de regime com G = I: Qn = diag(1e-5 ×6, 1e-3 ×6) e Rn = variâncias
 *   medidas dos sensores (1,15e-4; 5e-8; 1,94e-7; 6,04e-8; 1,28e-2; 2,2e-3;
 *   2,06e-6; 3,24e-7; 2,38e-7) na ordem de ::lqg_y_t. M = P C' (C P C' + Rn)^-1
 *   e L = A M.
 * É por isso que a Raspberry Pi consegue recalcular ganhos fiéis ao MATLAB a
 * partir de novos pesos durante o ajuste.
 */
#ifndef CONTROL_LQG_ERT_MODEL_H
#define CONTROL_LQG_ERT_MODEL_H

#include "control/lqg_ert.h"

/** Matriz de estado discreta A (bloco S2/A do Simulink). */
extern const lqg_real_t LQG_ERT_A[LQG_NX][LQG_NX];
/** Matriz de entrada discreta B (bloco S2/B do Simulink). */
extern const lqg_real_t LQG_ERT_B[LQG_NX][LQG_NU];
/** Matriz de saída C (bloco S2/C do Simulink); D = 0. */
extern const lqg_real_t LQG_ERT_C[LQG_NY][LQG_NX];
/** Ganhos Kd, Kfd, L e M exportados do Simulink. */
extern const lqg_gains_t LQG_ERT_DEFAULT_GAINS;

#endif
