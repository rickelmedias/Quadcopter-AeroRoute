/**
 * @file nav_ekf.h
 * @brief EKF de navegação: posição, velocidade e viés do acelerômetro em x, y
 *        (fluxo óptico) e z (telêmetro), com a aceleração linear do BNO08x.
 *
 * Volta do filtro de fluxo da branch `mtf-bno-test` (removido no commit
 * 0f1f5f9), com as mesmas técnicas e duas adições:
 *
 * - **Da versão antiga:** predição pela IMU e correção pela velocidade do
 *   fluxo; ruído do fluxo dado em rad/s e escalado pela distância (o mesmo
 *   erro angular é um erro de velocidade maior quanto mais alto); variância
 *   de quantização h²/3 quando o MTF lê zero (ele só reporta degraus de
 *   0,2 rad/s); gate de inovação aplicado ao **par** x/y (fundir só metade do
 *   vetor puxaria a estimativa para o eixo que passou); depois de
 *   ::NAV_EKF_MAX_REJECT recusas seguidas a medida é adotada (quem está errada
 *   é a predição); flag de convergência.
 * - **Viés do acelerômetro como estado** (por eixo): o documento do filtro
 *   antigo registra uma captura parada em que um viés pequeno integrou 763 mm
 *   de deriva.
 * - **Eixo z**: o mesmo filtro com a distância (já passada pelo
 *   control/range_filter.h e corrigida pela inclinação) e a aceleração
 *   vertical. Serve para validar o z e o vz do LQG e medir a qualidade do Z.
 *
 * Cada eixo é um Kalman de 3 estados [posição, velocidade, viés]:
 *
 *     p[k+1] = p + v dt + (a - b) dt²/2
 *     v[k+1] = v + (a - b) dt
 *     b[k+1] = b                       (passeio aleatório)
 *
 * x e y estão no referencial local do firmware (o de meas.c: fixo na
 * direção em que o STM32 ligou), z é NED (negativo acima do chão). A
 * posição horizontal continua sendo a integral da velocidade: não é
 * observável sem referência absoluta e deriva.
 *
 * Código puro, testado em tests/test_nav_ekf.c.
 */
#ifndef CONTROL_NAV_EKF_H
#define CONTROL_NAV_EKF_H

#include "control/range_filter.h"

#include <stdbool.h>
#include <stdint.h>

/** Recusas seguidas até adotar a medida (0,5 s a 100 Hz). */
#define NAV_EKF_MAX_REJECT 50U
/** Variância de velocidade abaixo da qual x, y contam como convergidos ((0,3 m/s)²). */
#define NAV_EKF_CONVERGED_VEL_VAR 0.09F
/** Passos sem correção de fluxo aceita até a velocidade x, y deixar de valer. */
#define NAV_EKF_FLOW_STALE_STEPS 20U
/** Passos (1 s a 100 Hz) em que a velocidade só pela inércia ainda vale. */
#define NAV_EKF_INERTIAL_STEPS 100U
/** Desvio da velocidade zero do ZUPT, em m/s. */
#define NAV_EKF_ZUPT_SIGMA 0.03F
/** Passos sem correção de distância aceita até o z deixar de valer. */
#define NAV_EKF_RANGE_STALE_STEPS 20U

/** @brief Ruídos e gate do filtro. */
typedef struct {
  /** Ruído da aceleração linear horizontal, em m/s². */
  float accel_noise;
  /** Ruído da aceleração vertical, em m/s². */
  float accel_noise_z;
  /** Passeio aleatório do viés do acelerômetro, em m/s² por raiz de s. */
  float bias_noise;
  /** Ruído do fluxo como taxa angular, em rad/s (vezes a distância = m/s). */
  float flow_noise_rad_s;
  /** Ruído da distância, em m. */
  float range_noise;
  /** Gate de inovação, em desvios (0 desliga). */
  float gate;
} nav_ekf_config_t;

/** @brief Um eixo: estado [posição, velocidade, viés] e covariância. */
typedef struct {
  float x[3];     /**< Posição (m), velocidade (m/s), viés do acelerômetro (m/s²). */
  float P[3][3];  /**< Covariância. */
} nav_ekf_axis_t;

/** @brief Estado do filtro. */
typedef struct {
  nav_ekf_axis_t h[2];      /**< Eixos x, y do referencial local. */
  nav_ekf_axis_t z;         /**< Eixo z (NED, negativo acima do chão). */
  range_filter_t range;     /**< Gate e filtro da distância. */
  float flow_innov[2];      /**< Última inovação de velocidade x, y, em m/s. */
  float flow_ratio;         /**< Última razão de teste do par x, y (acima de 1: recusada). */
  float z_innov;            /**< Última inovação de z, em m. */
  float z_ratio;            /**< Última razão de teste de z. */
  uint32_t flow_updates;    /**< Correções de fluxo aceitas. */
  uint32_t flow_rejected;   /**< Correções de fluxo recusadas pelo gate. */
  uint32_t flow_resets;     /**< Medidas de fluxo adotadas depois de recusas seguidas. */
  uint32_t z_updates;       /**< Correções de distância aceitas. */
  uint32_t z_rejected;      /**< Correções de distância recusadas pelo gate. */
  uint32_t z_resets;        /**< Distâncias adotadas depois de recusas seguidas. */
  uint16_t flow_run;        /**< Recusas seguidas de fluxo. */
  uint16_t z_run;           /**< Recusas seguidas de distância. */
  uint16_t flow_age;        /**< Passos desde a última correção de fluxo aceita. */
  uint16_t z_age;           /**< Passos desde a última correção de distância aceita. */
  bool h_started;           /**< x, y já receberam a primeira medida. */
  bool z_started;           /**< z já recebeu a primeira medida. */
} nav_ekf_t;

/**
 * @brief Esquece tudo, inclusive os vieses.
 * @param[out] f  Filtro.
 */
void nav_ekf_reset(nav_ekf_t *f);

/**
 * @brief Esquece posição, velocidade e distância, e mantém os vieses aprendidos.
 *
 * Chamada ao armar: o viés é do sensor e continua valendo; a posição recomeça
 * na origem.
 *
 * @param[in,out] f  Filtro.
 */
void nav_ekf_rearm(nav_ekf_t *f);

/**
 * @brief Zera a posição horizontal (a velocidade e o viés ficam).
 *
 * Chamada quando o XY liga: o LQG zera x, y ali (o ponto de decolagem é a
 * origem), e o EKF precisa da mesma origem para os dois poderem ser comparados.
 * Sem isso, o EKF carrega o que andou entre armar e sair do chão.
 *
 * @param[in,out] f  Filtro.
 */
void nav_ekf_zero_position(nav_ekf_t *f);

/**
 * @brief Predição de um passo pela aceleração linear.
 *
 * Chamada a cada passo do controle, com ou sem medidas novas.
 *
 * @param[in,out] f       Filtro.
 * @param[in]     ax      Aceleração em x local, em m/s².
 * @param[in]     ay      Aceleração em y local, em m/s².
 * @param[in]     a_down  Aceleração para baixo, em m/s² (gravidade já removida).
 * @param[in]     dt      Passo, em s.
 * @param[in]     cfg     Ruídos.
 */
void nav_ekf_predict(nav_ekf_t *f, float ax, float ay, float a_down, float dt,
                     const nav_ekf_config_t *cfg);

/**
 * @brief Correção com a velocidade do fluxo no referencial local.
 *
 * @param[in,out] f        Filtro.
 * @param[in]     vx       Velocidade x local medida, em m/s.
 * @param[in]     vy       Velocidade y local medida, em m/s.
 * @param[in]     range_m  Distância do sensor, em m (escala o ruído).
 * @param[in]     half_x   Meia largura da célula de quantização em x, em m/s (0 se não leu zero).
 * @param[in]     half_y   Idem em y.
 * @param[in]     cfg      Ruídos e gate.
 * @return true se a medida foi fundida (ou adotada depois de recusas seguidas).
 */
bool nav_ekf_update_flow(nav_ekf_t *f, float vx, float vy, float range_m, float half_x,
                         float half_y, const nav_ekf_config_t *cfg);

/**
 * @brief Correção com a altura medida (z NED = -distância × cos da inclinação).
 *
 * @param[in,out] f       Filtro.
 * @param[in]     z_meas  z medido, em m (negativo acima do chão).
 * @param[in]     cfg     Ruídos e gate.
 * @return true se a medida foi fundida (ou adotada depois de recusas seguidas).
 */
bool nav_ekf_update_z(nav_ekf_t *f, float z_meas, const nav_ekf_config_t *cfg);

/**
 * @brief Velocidade zero conhecida (drone apoiado no chão): corrige x, y com v = 0.
 *
 * Mantém o filtro iniciado e convergido enquanto o fluxo do MTF não vale (sensor
 * a menos de ~10 cm do piso) e deixa o acelerômetro estimar o viés parado.
 *
 * @param[in,out] f  Filtro.
 */
void nav_ekf_zupt(nav_ekf_t *f);

/**
 * @brief Velocidade x, y só pela inércia: iniciada e com a última correção de fluxo ou ZUPT há
 *        no máximo ::NAV_EKF_INERTIAL_STEPS passos.
 * @param[in] f  Filtro.
 */
bool nav_ekf_xy_inertial(const nav_ekf_t *f);

/**
 * @brief x, y convergidos e corrigidos há pouco: a velocidade pode ir para o controle.
 * @param[in] f  Filtro.
 */
bool nav_ekf_xy_usable(const nav_ekf_t *f);

/**
 * @brief z corrigido há pouco: a altura filtrada pode ir para o controle.
 * @param[in] f  Filtro.
 */
bool nav_ekf_z_usable(const nav_ekf_t *f);

#endif
