/**
 * @file ekf3.h
 * @brief EKF de erro de estado com quatérnio, no estilo do EKF3 do ArduPilot, para o modo com cabo.
 *
 * Código puro, float, testado no PC. Substitui o EKF de 12 estados em ângulos de
 * Euler (control/ekf.h, que continua na branch phase2/control-lqg). Ts = ::LQG_TS_S.
 *
 * Estado nominal (referencial local NED: origem e guinada do armamento; corpo FRD, SI):
 *
 *   q   quatérnio corpo -> local                           (4)
 *   v   velocidade local, m/s                              (3)
 *   p   posição local, m (z para baixo)                    (3)
 *   bg  viés do giroscópio, rad/s, eixos do corpo          (3)
 *   ba  viés da aceleração linear, m/s², eixos do corpo    (3)
 *
 * Estado de erro (15): δθ (rotação no corpo, q = q̂ ⊗ exp(δθ)), δv, δp, δbg, δba.
 * A covariância é do estado de erro; depois de cada correção o erro entra no
 * estado nominal e volta a zero.
 *
 * Diferenças para o EKF anterior:
 *
 * - **Predição por strapdown:** giro calibrado e aceleração linear do BNO085
 *   (sem gravidade, ver bno_sample_t::lin_accel) propagam atitude, velocidade e
 *   posição. O modelo A/B do Simulink e o Δω dos motores não entram no filtro.
 * - **Quatérnio:** sem ângulos de Euler no estado, sem desenrolar de guinada.
 * - **Vieses** de giro e de aceleração estimados. Como no EKF3 sem GPS, o viés de
 *   aceleração em x, y fica congelado (variância zero) enquanto não há fluxo
 *   recente: só a velocidade do fluxo o torna observável.
 * - **Medidas:** atitude (quatérnio do BNO; resíduo = vetor de rotação do erro),
 *   distância do MTF-01 (inclinação e braço do sensor pela matriz de rotação) e
 *   velocidade do fluxo no corpo (R^T v), cada uma escalar e com teste de
 *   inovação (qui-quadrado) como antes.
 *
 * Não é o EKF3 completo: não há GPS, barômetro, magnetômetro, campo magnético
 * nem múltiplos núcleos. Posição XY continua sem referência absoluta.
 *
 * Ordem por passo: ekf3_update() com as medidas -> controle com a estimativa
 * -> ekf3_predict() com o IMU do mesmo passo.
 */
#ifndef CONTROL_EKF3_H
#define CONTROL_EKF3_H

#include <stdbool.h>
#include <stdint.h>

/** Estados de erro. */
#define EKF3_NX 15U

/** Blocos do estado de erro. */
#define EKF3_DTH 0U
#define EKF3_DV  3U
#define EKF3_DP  6U
#define EKF3_DBG 9U
#define EKF3_DBA 12U

/** Vetor de estimativa de 12 posições (::ekf3_estimate): mesma ordem do modelo do Simulink. */
#define EKF3_PX    0U
#define EKF3_PY    1U
#define EKF3_PZ    2U
#define EKF3_PHI   3U
#define EKF3_THETA 4U
#define EKF3_PSI   5U
#define EKF3_U     6U
#define EKF3_V     7U
#define EKF3_W     8U
#define EKF3_P     9U
#define EKF3_Q     10U
#define EKF3_R     11U
#define EKF3_NEST  12U

/** @brief Filtro. Campos internos: ler só pelas funções abaixo. */
typedef struct {
  float q[4];
  float v[3];
  float p[3];
  float bg[3];
  float ba[3];
  /** Último giro corrigido do viés (p, q, r), em rad/s. */
  float rate[3];
  float P[EKF3_NX][EKF3_NX];
  /** Rejeições seguidas por medida (distância, fluxo frente, fluxo direita). */
  uint32_t rejected[3];
  /** Total de medidas rejeitadas desde ekf3_seed(), para diagnóstico. */
  uint32_t rejected_total;
  /** Atualizações puladas porque a variância ficaria negativa (erro numérico), para diagnóstico. */
  uint32_t variance_skips;
  /** Passos desde o último fluxo oferecido; o viés de aceleração em x, y só aprende com fluxo recente. */
  uint32_t flow_age;
  /** Viés de aceleração em x, y congelado (sem fluxo recente). */
  bool ba_xy_off;
} ekf3_t;

/** @brief Medidas e IMU de um passo; só as com o bit @c valid correspondente entram. */
typedef struct {
  /** Giro do corpo (p, q, r), em rad/s, e aceleração linear sem gravidade (corpo FRD), em m/s². */
  bool gyro_valid;
  float gyro[3];
  bool accel_valid;
  float accel[3];
  /** Atitude medida (corpo -> local, nivelada e relativa ao armamento). */
  bool attitude_valid;
  float q_meas[4];
  /** Multiplicador do ruído da atitude (precisão do BNO); 0 vale 1. */
  float att_r_scale;
  /** Distância do MTF-01 no eixo do sensor, em m, e o chão do armamento. */
  bool range_valid;
  float range_m;
  float ground_m;
  /** Posição do MTF-01 em relação ao centro de massa, em m (x frente, y direita). */
  float range_pos_x, range_pos_y;
  /** Velocidade do corpo (frente, direita) do fluxo, em m/s, e a qualidade 0..255. */
  bool flow_valid;
  float flow_fwd, flow_right;
  uint8_t flow_quality;
} ekf3_meas_t;

/**
 * @brief Estado e incerteza iniciais: atitude @p q (NULL = identidade), altura local
 *        @p pz (m, para baixo), velocidade zero e vieses @p bg, @p ba (NULL = zero).
 */
void ekf3_seed(ekf3_t *e, const float q[4], float pz, const float bg[3], const float ba[3]);

/** @brief Nova origem horizontal: x = y = 0, incerteza inicial e sem correlação com o resto. */
void ekf3_reset_xy(ekf3_t *e);

/** @brief Correção com as medidas de um passo (atualização sequencial, escalar). */
void ekf3_update(ekf3_t *e, const ekf3_meas_t *m);

/** @brief Predição um passo à frente com o giro e a aceleração de @p m (campos de IMU). */
void ekf3_predict(ekf3_t *e, const ekf3_meas_t *m);

/** @brief [px py pz φ θ ψ u v w p q r] (::EKF3_PX ...), para telemetria e para os ganhos do modelo. */
void ekf3_estimate(const ekf3_t *e, float x[EKF3_NEST]);

/**
 * @brief Inclinação e guinada da estimativa.
 * @param[out] tilt  Vetor de rotação da inclinação em torno dos eixos x, y do corpo, em rad
 *                   (equivale a φ, θ sem a singularidade, ψ fora).
 * @param[out] yaw   Guinada, em rad (-π, π].
 */
void ekf3_attitude(const ekf3_t *e, float tilt[2], float *yaw);

#endif
