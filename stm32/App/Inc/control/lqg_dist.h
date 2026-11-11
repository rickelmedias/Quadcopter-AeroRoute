/**
 * @file lqg_dist.h
 * @brief LQG ajustado e estimador de perturbação ("LQG melhorado").
 *
 * O LQG do TCC é só proporcional ao estado: com o hover (CTL_W0) errado, o
 * peso fora do centro ou motores diferentes, sobra uma força constante que ele
 * não sabe que existe, e o drone para fora do alvo (em simulação, 30 rad/s de
 * hover errado deixam 14–20 cm de erro de altura mesmo com ganhos maiores).
 *
 * O "LQG melhorado" continua sendo LQG (estimador + LQR), com o estimador
 * também estimando essa perturbação. Para cada canal (altura, φ, θ, ψ), um Kalman
 * de regime com posição, velocidade e uma perturbação constante d nos motores,
 * no padrão daquele canal (os quatro iguais; frente contra trás; esquerda
 * contra direita):
 *
 * @verbatim
 *   [pos; vel; d][k+1] = Ac [pos; vel; d][k] + Bc U[k],   U = Σ_j padrão_j u_j
 *   Ac = [A_pp  A_pv  -4 b_p;  A_vp  A_vv  -4 b_v;  0  0  1],   Bc = [b_p; b_v; 0]
 *   correção:  s = s⁻ + M (y - C s⁻)
 *   controle:  u_j = du_j (LQR do TCC/ajustado) + d̂ · padrão_j
 * @endverbatim
 *
 * A, B, Qn e Rn são os do Kalman do TCC; só a variância de d é nova. Os ganhos
 * são gerados por tools/lqg_ajustado.py (App/Src/control/lqg_ajustado_gains.c),
 * junto com os do "LQG ajustado": o mesmo dlqr do TCC com outros pesos Q.
 *
 * Como o ajuste lento, a correção d̂ fica fora da
 * predição do Kalman do TCC (é uma força que o modelo não tem).
 *
 * Código puro (sem HAL), testado em tests/test_lqg_dist.c.
 */
#ifndef CONTROL_LQG_DIST_H
#define CONTROL_LQG_DIST_H

#include "control/lqg_ert.h"

#include <stdbool.h>
#include <stdint.h>

/** Canais do estimador. */
#define LQG_DIST_NC 4U

/** @brief Índice de cada canal. */
typedef enum {
  LQG_DIST_Z = 0,   /**< Altura: hover errado (os quatro motores iguais). */
  LQG_DIST_PHI,     /**< Rolagem: esquerda (u1, u4) contra direita (u2, u3). */
  LQG_DIST_THETA,   /**< Arfagem: frente (u1, u2) contra trás (u3, u4). */
  LQG_DIST_PSI      /**< Guinada: u2, u4 contra u1, u3 (torque das hélices). */
} lqg_dist_index_t;

/** @brief Modelo e ganho de regime de um canal. */
typedef struct {
  uint8_t pos;          /**< Estado de posição no modelo (LQG_X_*). */
  uint8_t vel;          /**< Estado de velocidade no modelo. */
  uint8_t ny;           /**< Medidas usadas (1: só posição; 2: posição e velocidade). */
  uint8_t y[2];         /**< Índice de cada medida em y (LQG_Y_*). */
  float pattern[LQG_NU];  /**< Padrão da perturbação em u1..u4 (+1 ou -1). */
  float A[3][3];        /**< Ac. */
  float B[3];           /**< Bc. */
  float M[3][2];        /**< Ganho de correção de regime. */
} lqg_dist_channel_t;

/** Canais gerados por tools/lqg_ajustado.py. */
extern const lqg_dist_channel_t LQG_DIST_CHANNELS[LQG_DIST_NC];

/**
 * @brief Limita a parcela de guinada do du a ±@p limit, sem mexer nas demais.
 *
 * A guinada tem pouca autoridade no drone real, então um giro súbito leva o
 * LQG a pedir centenas de rad/s nos quatro motores e isso arrasta a atitude
 * (roll e pitch) junto. A parcela de guinada é a projeção de @p du no padrão do
 * canal ::LQG_DIST_PSI; só ela é encurtada.
 *
 * @param[in]  du     Saída do LQG.
 * @param[out] out    du com a guinada limitada (igual a @p du se não precisar).
 * @param[in]  limit  Limite em rad/s (<= 0 desliga).
 * @return true se algum valor mudou.
 */
bool lqg_dist_limit_yaw(const lqg_real_t du[LQG_NU], lqg_real_t out[LQG_NU], float limit);
/** Kd do LQG ajustado (gerado). */
extern const lqg_real_t LQG_AJ_KD[LQG_NU][LQG_NX];
/** Kfd do LQG ajustado (gerado). */
extern const lqg_real_t LQG_AJ_KFD[LQG_NU][LQG_NR];
/** M do Kalman do LQG ajustado: o do TCC com o ruído de vy igual ao de vx (gerado). */
extern const lqg_real_t LQG_AJ_M[LQG_NX][LQG_NY];
/** L do Kalman do LQG ajustado (gerado). */
extern const lqg_real_t LQG_AJ_L[LQG_NX][LQG_NY];
/** M do Kalman para a velocidade fundida (FLOW_EKF): vx, vy menos ruidosos (gerado). */
extern const lqg_real_t LQG_AJ_M_FUSED[LQG_NX][LQG_NY];
/** L do Kalman para a velocidade fundida (gerado). */
extern const lqg_real_t LQG_AJ_L_FUSED[LQG_NX][LQG_NY];
/**
 * Autoridade real de guinada dividida pela do B do TCC (gerado, ~3e-4). O B do
 * TCC põe ~2000 vezes mais aceleração de guinada por rad/s do que o drone tem;
 * o LQR e o estimador do ajustado usam a real (tools/lqg_ajustado.py).
 */
extern const float LQG_AJ_YAW_SCALE;
/**
 * Resposta vertical real dividida pela do B do TCC (gerado, ~0,7): o B do TCC
 * usa ω0 = 702 rad/s e o hover real com o chumbo está em ~1000.
 */
extern const float LQG_AJ_Z_SCALE;

/** @brief Estado do estimador: [posição, velocidade, d] de cada canal. */
typedef struct {
  float s[LQG_DIST_NC][3];  /**< Estados; s[c][2] é d̂, em rad/s por motor. */
  float d_before[LQG_DIST_NC];  /**< d̂ antes da última correção (para congelar). */
  bool started[LQG_DIST_NC];  /**< O canal já recebeu a primeira medida. */
} lqg_dist_t;

/** @brief Troca Kd e Kfd pelos do LQG ajustado numa cópia dos ganhos (L e M não mudam). */
void lqg_dist_apply_gains(lqg_gains_t *gains);

/**
 * @brief Troca L e M pelos do Kalman do LQG ajustado numa cópia dos ganhos.
 *
 * Sem fusão, só as linhas de y e vy diferem das do Simulink: o TCC supunha o
 * fluxo em vy bem menos ruidoso que em vx, e no drone os dois têm o mesmo
 * ruído. Com @p fused (FLOW_EKF), vx e vy vêm do filtro de fusão e o Kalman
 * confia mais neles.
 *
 * @param[in,out] gains  Cópia dos ganhos em uso.
 * @param[in]     fused  Velocidade horizontal fundida com o acelerômetro.
 */
void lqg_dist_apply_kalman(lqg_gains_t *gains, bool fused);

/**
 * @brief Entrada da predição do Kalman com a guinada e a altura na escala real.
 *
 * A predição do Kalman usa o B do TCC. Com os ganhos de guinada do ajustado
 * (bem maiores, para a autoridade real), o B do TCC preveria ψ e r disparando
 * a cada passo; na altura, preveria o drone subindo mais do que sobe. As
 * componentes de @p du no padrão de guinada e no coletivo são multiplicadas por
 * ::LQG_AJ_YAW_SCALE e ::LQG_AJ_Z_SCALE; os quatro padrões (coletivo, φ, θ, ψ)
 * são ortogonais, então φ e θ não mudam.
 *
 * @param[in]  du   Entrada aplicada (u1..u4), em rad/s.
 * @param[out] out  Entrada para lqg_ert_update().
 */
void lqg_dist_kalman_input(const lqg_real_t du[LQG_NU], lqg_real_t out[LQG_NU]);

/** @brief Zera o estimador (a primeira medida de cada canal vira a posição). */
void lqg_dist_reset(lqg_dist_t *e);

/**
 * @brief Corrige com as medidas do passo.
 * @param[in,out] e         Estimador.
 * @param[in]     y         Medidas.
 * @param[in]     range_ok  A altura medida vale neste passo (senão o canal z só prediz).
 */
void lqg_dist_correct(lqg_dist_t *e, const lqg_real_t y[LQG_NY], bool range_ok);

/**
 * @brief Prediz o próximo passo com o comando aplicado.
 *
 * Congelar desfaz também o que a última correção (::lqg_dist_correct) fez em
 * d̂: no chão, o apoio segura o drone e a correção leria isso como empuxo
 * sobrando (ensaio de 05/11: d̂ de altura em −160 rad/s no fim da rampa de
 * partida, e o drone não decolava).
 *
 * @param[in,out] e        Estimador.
 * @param[in]     u        Desvio de rotação aplicado em u1..u4 (du mais a
 *                         própria correção; sem o desequilíbrio fixo), em rad/s.
 * @param[in]     freeze   Não mexe em nenhum d̂ (no chão, rampa, saturação, a seco).
 * @param[in]     freeze_z Não mexe no d̂ de altura (sem medida de altura).
 * @param[in]     limit    Maior |d̂|, em rad/s por motor.
 */
void lqg_dist_predict(lqg_dist_t *e, const float u[LQG_NU], bool freeze, bool freeze_z,
                      float limit);

/** @brief Correção d̂ · padrão em cada entrada u1..u4, em rad/s. */
void lqg_dist_output(const lqg_dist_t *e, float out[LQG_NU]);

/** @brief d̂ de um canal, em rad/s por motor. */
float lqg_dist_get(const lqg_dist_t *e, lqg_dist_index_t c);

/** @brief Muda o d̂ de um canal (pouso e subida presa), em rad/s por motor. */
void lqg_dist_set(lqg_dist_t *e, lqg_dist_index_t c, float value);

#endif
