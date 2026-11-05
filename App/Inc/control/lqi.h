/**
 * @file lqi.h
 * @brief LQI: o LQR do TCC com integradores de altura, rolagem e arfagem.
 *
 * O LQR exportado pelo Simulink é só proporcional ao estado. Com o centro de
 * gravidade fora do centro, ou motores diferentes, sobra um torque constante
 * que ele não zera (ensaios de 04/11: θ parava em 10–16° com a traseira
 * pesada). O LQI acrescenta ao estado as integrais do erro de altura, de φ e
 * de θ e projeta tudo junto pelo mesmo `dlqr`:
 *
 * @verbatim
 *   estado aumentado  [ z φ θ ψ vz p q r | ∫(z - r_z) ∫φ ∫θ ]
 *   lei de controle   du = -Kx (x̂ - x_ref) - Ki ξ
 * @endverbatim
 *
 * O projeto usa o subsistema de atitude e altura do mesmo A, B do Simulink:
 * no modelo do TCC, x, y, vx e vy não realimentam a atitude, e a integral de θ
 * já é (a menos de constante) a velocidade vx, então um integrador de θ no
 * modelo completo seria redundante. Por isso a posição horizontal fica fora
 * deste modo (como no modo só atitude, control/attitude_mode.h).
 *
 * Os ganhos são gerados por tools/lqi/lqi_gen.c (o mesmo cálculo está em
 * tools/lqi/projeto_lqi.m para o MATLAB). O Kalman (M, L) continua o do
 * Simulink e recebe só a parte do LQR do du: a parte do integrador compensa
 * uma perturbação que o modelo não tem, e passá-la ao filtro enviesaria a
 * estimativa.
 *
 * Código puro (sem HAL nem FreeRTOS), testado em tests/test_lqi.c.
 */
#ifndef CONTROL_LQI_H
#define CONTROL_LQI_H

#include "control/lqg_ert.h"

#include <stdbool.h>

/** Quantidade de integradores (altura, φ, θ). */
#define LQI_NI 3U

/** @brief Índice de cada integrador. */
typedef enum {
  LQI_I_Z = 0,  /**< ∫(z - r_z), em m·s (z NED). */
  LQI_I_PHI,    /**< ∫φ, em rad·s. */
  LQI_I_THETA   /**< ∫θ, em rad·s. */
} lqi_integrator_t;

/** @brief Ganhos do LQI, nos índices do modelo completo. */
typedef struct {
  /** Ganho do estado (x, y, vx, vy em zero), u = -Kx x̂. */
  lqg_real_t Kx[LQG_NU][LQG_NX];
  /** Referência como desvio de estado: colunas z e ψ de Kx (x e y em zero). */
  lqg_real_t Kfd[LQG_NU][LQG_NR];
  /** Ganho dos integradores, u = -Ki ξ. */
  lqg_real_t Ki[LQG_NU][LQI_NI];
} lqi_gains_t;

/** Ganhos gerados por tools/lqi/lqi_gen.c (App/Src/control/lqi_gains.c). */
extern const lqi_gains_t LQI_GAINS;

/** @brief Estado dos integradores. */
typedef struct {
  /** ξ: integral dos erros, nas unidades de ::lqi_integrator_t. */
  float xi[LQI_NI];
} lqi_state_t;

/** @brief Zera os integradores. */
void lqi_reset(lqi_state_t *s);

/**
 * @brief Troca o LQR do Simulink pelo do LQI numa cópia dos ganhos.
 *
 * Kd e Kfd passam a ser os do LQI; o Kalman (L, M) não muda.
 *
 * @param[in]     g     Ganhos do LQI.
 * @param[in,out] gains Cópia dos ganhos em uso.
 */
void lqi_apply_gains(const lqi_gains_t *g, lqg_gains_t *gains);

/**
 * @brief Integra um passo.
 *
 * Os erros são as medidas diretas (MTF e BNO), não a estimativa: assim o
 * erro em regime vai a zero no que o sensor mede, mesmo com o modelo errado.
 * Cada integrador é limitado para que a correção dele em cada motor não passe
 * de @p limit_rad_s.
 *
 * @param[in,out] s            Integradores.
 * @param[in]     g            Ganhos (para o limite).
 * @param[in]     enabled      Integrar neste passo (falso congela).
 * @param[in]     dt           Período, em s.
 * @param[in]     error        Erros de z (m, NED), φ e θ (rad).
 * @param[in]     limit_rad_s  Maior correção de um integrador em um motor, em rad/s.
 */
void lqi_update(lqi_state_t *s, const lqi_gains_t *g, bool enabled, float dt,
                const float error[LQI_NI], float limit_rad_s);

/**
 * @brief Correção dos integradores em cada entrada do modelo, -Ki ξ, em rad/s.
 *
 * @param[in]  s    Integradores.
 * @param[in]  g    Ganhos.
 * @param[out] out  Valor de u1..u4.
 */
void lqi_output(const lqi_state_t *s, const lqi_gains_t *g, float out[LQG_NU]);

/**
 * @brief Diz se algum integrador está no limite.
 *
 * @param[in] s            Integradores.
 * @param[in] g            Ganhos.
 * @param[in] limit_rad_s  O mesmo limite de ::lqi_update.
 */
bool lqi_at_limit(const lqi_state_t *s, const lqi_gains_t *g, float limit_rad_s);

#endif
