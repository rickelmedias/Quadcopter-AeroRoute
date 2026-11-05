/**
 * @file lqi_design.h
 * @brief Projeto do LQI do teste de controle (roda no PC).
 *
 * Mesmo modelo A, B do Simulink, subsistema de atitude e altura
 * (z, φ, θ, ψ, vz, p, q, r), aumentado com ∫(z - r_z), ∫φ e ∫θ:
 *
 * @verbatim
 *   Aa = [ A_r      0 ]    Ba = [ B_r ]    ξ[k+1] = ξ[k] + Ts · (z, φ, θ)[k]
 *        [ Ts·Ci    I ]         [  0  ]
 *   [Kx_r Ki] = dlqr(Aa, Ba, diag(Q_r, Q_i), R)
 * @endverbatim
 *
 * Q_r e R são os pesos do TCC (os mesmos que reproduzem o Kd do Simulink);
 * só os pesos dos integradores são novos. tools/lqi/projeto_lqi.m faz o mesmo
 * cálculo no MATLAB.
 */
#ifndef TOOLS_LQI_DESIGN_H
#define TOOLS_LQI_DESIGN_H

#include "control/lqi.h"

/** @brief Pesos do projeto. */
typedef struct {
  /** Diagonal de Q para os 12 estados do modelo (só os de atitude e altura são usados). */
  double q[LQG_NX];
  /** Diagonal de R. */
  double r[LQG_NU];
  /** Pesos de ∫z, ∫φ e ∫θ. */
  double q_int[LQI_NI];
} lqi_weights_t;

/**
 * @brief Pesos do TCC (os que reproduzem o Kd do Simulink) mais os
 *        integradores escolhidos em simulação (∫z = 1000, ∫φ = ∫θ = 10000).
 */
void lqi_tcc_weights(lqi_weights_t *w);

/**
 * @brief Pesos padrão do LQI: os de ::lqi_tcc_weights com a atitude mais
 *        firme (φ, θ = 8000 e p, q = 100), escolhidos depois dos ensaios de
 *        05/11 (ver docs/controle_lqi.md).
 */
void lqi_default_weights(lqi_weights_t *w);

/**
 * @brief Calcula Kx, Kfd e Ki.
 *
 * @param[in]  w    Pesos.
 * @param[out] out  Ganhos, nos índices do modelo completo.
 * @return Iterações do dlqr, ou -1 se não convergiu.
 */
int lqi_design(const lqi_weights_t *w, lqi_gains_t *out);

#endif
