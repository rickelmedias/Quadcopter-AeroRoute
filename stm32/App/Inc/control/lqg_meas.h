/**
 * @file lqg_meas.h
 * @brief Leituras dos sensores -> vetor de medidas do modelo LQR/Kalman.
 *
 * Código puro (sem HAL nem FreeRTOS), testado no PC. Leva o BNO085 e o MTF-01
 * para as medidas do modelo gerado pelo Simulink, em NED/FRD e SI:
 *
 *   y = [z  phi  theta  psi  u  v  p  q  r]
 *
 * z em m (para baixo, relativo ao chão do armamento), ângulos em rad, u e v em
 * m/s no referencial local (guinada zero no armamento), p, q, r em rad/s.
 *
 * Mapa de eixos e sinais: os mesmos do firmware da submissão, conferidos no
 * drone real (ver control/lqg_config.h). O BNO está de cabeça para baixo e
 * girado 90 graus: o roll do sensor é a arfagem θ do modelo e o pitch do
 * sensor é a rolagem φ.
 */
#ifndef CONTROL_LQG_MEAS_H
#define CONTROL_LQG_MEAS_H

#include <stdbool.h>
#include <stdint.h>

/** Quantidade de medidas do modelo. */
#define LQG_NY 9U

/** Índices de ::lqg_meas_out_t::y (mesma ordem de STM32_U.y). */
#define LQG_Y_Z     0U
#define LQG_Y_PHI   1U
#define LQG_Y_THETA 2U
#define LQG_Y_PSI   3U
#define LQG_Y_U     4U
#define LQG_Y_V     5U
#define LQG_Y_P     6U
#define LQG_Y_Q     7U
#define LQG_Y_R     8U

/** Bits de ::lqg_meas_out_t::flags: medidas utilizáveis neste passo. */
#define LQG_MEAS_ATTITUDE 0x01U
#define LQG_MEAS_GYRO     0x02U
#define LQG_MEAS_RANGE    0x04U
#define LQG_MEAS_FLOW     0x08U
/** Quadro novo do MTF-01 neste passo (distância e fluxo ainda não usados). */
#define LQG_MEAS_FRESH    0x10U
/** Altura suficiente para o fluxo valer (::LQG_FLOW_MIN_HEIGHT_M, com histerese). */
#define LQG_MEAS_FLOW_HEIGHT 0x20U
/** Aceleração linear do BNO válida e em dia (::lqg_meas_out_t::accel_body). */
#define LQG_MEAS_ACCEL       0x40U

/** @brief Leituras cruas, nos eixos e unidades dos drivers. */
typedef struct {
  /** Roll, pitch e yaw ZYX do BNO, em rad, nos eixos do sensor. */
  float roll, pitch, yaw;
  /** Giro do BNO nos eixos do sensor, em rad/s. */
  float gyro[3];
  /** Quatérnio do BNO (w, x, y, z), sensor -> mundo do BNO (game rotation vector). */
  float q[4];
  /** Quatérnio válido e em dia; falso: a atitude do EKF3 sai dos ângulos de Euler acima. */
  bool q_valid;
  /** Status de precisão do relatório de rotação do BNO, 0..3. */
  uint8_t att_accuracy;
  /** Aceleração linear do BNO (sem gravidade), em m/s², nos eixos do sensor. */
  float accel[3];
  /** Rotação e giro do BNO válidos e em dia. */
  bool attitude_valid, gyro_valid;
  /** Aceleração linear válida e em dia. */
  bool accel_valid;
  /** Distância do MTF-01 no eixo do sensor, em m. */
  float range_m;
  /** Distância utilizável. */
  bool range_valid;
  /** Fluxo do MTF-01 em rad/s (cm/s a 1 m dividido por 100). */
  float flow_x, flow_y;
  /** Fluxo utilizável. */
  bool flow_valid;
  /** Qualidade da imagem do fluxo, 0..255. */
  uint8_t flow_quality;
  /** Contador de quadros do MTF-01; 0 = desconhecido (todo passo conta como novo). */
  uint32_t mtf_frames;
  /** Tensão da bateria, em V (PA1 calibrado). */
  float vbat_v;
  /** Tensão lida e calibrada. */
  bool vbat_valid;
} lqg_sensors_t;

/** Tamanho do histórico de giro para a compensação do fluxo. */
#define LQG_GYRO_HISTORY 4U

/** @brief Estado da conversão (origem da guinada e chão de referência). */
typedef struct {
  bool yaw_ready;
  float yaw_last;
  float yaw_unwrapped;
  /** Distância corrigida pela inclinação lida no chão, em m. */
  float ground_m;
  /** φ e θ lidos parado no chão ao armar, em rad: o zero de nível deste voo. */
  float level_phi, level_theta;
  /** Último quadro do MTF-01 visto (::lqg_sensors_t::mtf_frames). */
  uint32_t last_frames;
  /** Altura acima do chão suficiente para o fluxo (com histerese). */
  bool flow_height_ok;
  /** Giro x, y do sensor nos últimos passos (circular), em rad/s. */
  float gyro_hist[LQG_GYRO_HISTORY][2];
  uint32_t gyro_n;
  /** Caminho em quatérnio: guinada de referência (conjugado) e nível, ambos em w, x, y, z. */
  bool q_init, q_ready;
  float q_yaw0_inv[4];
  float q_level_inv[4];
  /** Inclinação do corpo na pose de ::LQG_ZERO_ROLL_DEG, ::LQG_ZERO_PITCH_DEG (conjugado). */
  float q_zero_inv[4];
} lqg_meas_t;

/** @brief Resultado de um passo. */
typedef struct {
  /** Medidas do modelo; só valem as que têm o bit correspondente em @c flags. */
  float y[LQG_NY];
  /** ::LQG_MEAS_ATTITUDE e demais. */
  uint8_t flags;
  /**
   * Altura do centro de massa equivalente medida pelo MTF-01, em m: distância
   * vezes cos(φ)cos(θ), menos o deslocamento vertical do sensor fora do centro
   * (::LQG_RANGE_POS_X_M). 0 sem ::LQG_MEAS_RANGE.
   */
  float range_corrected_m;
  /** Velocidade do corpo (frente, direita) do fluxo, em m/s, antes de girar pela guinada. */
  float flow_body[2];
  /**
   * Quatérnio corpo -> local, relativo ao armamento (guinada) e ao nível, válido com
   * ::LQG_MEAS_ATTITUDE. Com ::lqg_sensors_t::q_valid vem direto do quatérnio do BNO
   * (@c q_native); sem ele é montado de y[φ θ ψ] (ZYX).
   */
  float q[4];
  /** O quatérnio acima veio do BNO, sem passar por Euler. */
  bool q_native;
  /** Vetor de rotação da inclinação sem o nível do voo (corpo, rad), com @c q_native. */
  float tilt_raw[3];
  /** Status de precisão do BNO da atitude, 0..3 (3 sem @c q_native). */
  uint8_t att_accuracy;
  /** Aceleração linear sem gravidade nos eixos do corpo (FRD), em m/s², com ::LQG_MEAS_ACCEL. */
  float accel_body[3];
} lqg_meas_out_t;

/** @brief Zera a origem da guinada e o chão (chamar ao armar). */
void lqg_meas_reset(lqg_meas_t *m);

/** @brief Define o chão de referência, em m (distância corrigida lida parado). */
void lqg_meas_set_ground(lqg_meas_t *m, float ground_m);

/** @brief Define o zero de nível do voo (φ, θ lidos parado no chão), em rad. */
void lqg_meas_set_level(lqg_meas_t *m, float phi, float theta);

/** @brief Nível do voo no caminho em quatérnio: vetor de rotação da inclinação no chão, no corpo, em rad. */
void lqg_meas_set_level_q(lqg_meas_t *m, const float rv[3]);

/** @brief Converte as leituras de um passo em medidas do modelo. */
void lqg_meas_update(lqg_meas_t *m, const lqg_sensors_t *in, lqg_meas_out_t *out);

#endif
