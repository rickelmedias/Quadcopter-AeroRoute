/**
 * @file meas.h
 * @brief Converte as leituras do BNO08x e do MTF-01 no vetor de medidas y.
 *
 * O modelo do TCC supõe sensores que medem os estados diretamente
 * (y = C x). Este módulo só faz a conversão de unidades, eixos e geometria
 * para chegar lá; **não filtra nada** (a filtragem é o Kalman do
 * control/lqg_ert.h):
 *
 * | Medida      | Origem                                                          |
 * |-------------|-----------------------------------------------------------------|
 * | z           | -(distância do MTF-01 · cos(inclinação)), em m (NED: z para baixo) |
 * | φ           | pitch do BNO08x menos o zero, com o sinal SNS_SGN_ROLL           |
 * | θ           | roll do BNO08x menos o zero, com o sinal SNS_SGN_PITCH           |
 * | ψ           | guinada do BNO08x relativa à energização, sem saltos em ±π       |
 * | vx, vy      | (escala · fluxo + ganho · giro) · distância, girada pela guinada |
 * | p, q, r     | giro y, giro x e giro z do BNO08x, com os sinais SNS_SGN_P/Q/R   |
 *
 * O BNO está montado de cabeça para baixo e girado 90 graus em torno da
 * vertical: o eixo x dele aponta para o lado do drone, então roll e pitch do
 * sensor trocam de papel (e o giro x/y também). Com o drone nivelado o roll
 * do sensor lê ~ -174,5 graus; o zero é SNS_ZERO_ROLL/SNS_ZERO_PITCH.
 *
 * Quando um sensor não tem leitura válida, a medida correspondente repete o
 * último valor (segurador de ordem zero, como uma entrada amostrada no
 * Simulink) e o bit dela fica zerado em ::meas_flags_t.
 *
 * Código puro (sem HAL nem FreeRTOS), testado em tests/test_meas.c.
 */
#ifndef CONTROL_MEAS_H
#define CONTROL_MEAS_H

#include "control/ctrl_params.h"
#include "control/lqg_ert.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief Medidas válidas neste passo (mesmos bits de DRONE_MEAS_FLAG). */
typedef enum {
  MEAS_ATTITUDE = 1U << 0,  /**< φ, θ, ψ do BNO08x. */
  MEAS_GYRO = 1U << 1,      /**< p, q, r do BNO08x. */
  MEAS_RANGE = 1U << 2,     /**< z do MTF-01. */
  MEAS_FLOW = 1U << 3       /**< vx, vy do MTF-01. */
} meas_flags_t;

/** @brief Leituras cruas de um passo, já copiadas dos drivers. */
typedef struct {
  /** Quatérnio w, x, y, z do BNO08x. */
  float q[4];
  /** Rolagem do BNO08x, em rad, eixos do sensor. */
  float roll;
  /** Arfagem do BNO08x, em rad. */
  float pitch;
  /** Guinada do BNO08x, em rad. */
  float yaw;
  /** Giroscópio x, y, z do BNO08x, em rad/s, eixos do sensor. */
  float gyro[3];
  /** Atitude válida (BNO online e Euler sem singularidade). */
  bool attitude_valid;
  /** Giroscópio válido (BNO online). */
  bool gyro_valid;
  /** Distância do telêmetro, em m, no eixo do sensor. */
  float range_m;
  /** Distância utilizável. */
  bool range_valid;
  /** Fluxo óptico X, em rad/s (cm/s a 1 m dividido por 100). */
  float flow_x;
  /** Fluxo óptico Y, em rad/s. */
  float flow_y;
  /** Fluxo utilizável. */
  bool flow_valid;
} meas_input_t;

/** @brief Estado da conversão (referência da guinada e última medida). */
typedef struct {
  /** Última medida de cada saída (repetida quando o sensor falha). */
  float y[LQG_NY];
  /** Guinada do sensor na energização (primeira leitura válida), em rad. */
  float yaw_origin;
  /** Última guinada do sensor vista, em rad, para desenrolar. */
  float yaw_last;
  /** Guinada desenrolada relativa à origem, em rad (contínua). */
  float yaw_unwrapped;
  /** A origem da guinada já foi definida. */
  bool yaw_ready;
} meas_state_t;

/**
 * @brief Zera tudo e define que a próxima guinada lida é o zero.
 *
 * Chamada uma vez, na energização: a guinada zero é a direção em que o drone
 * foi ligado.
 *
 * @param[out] s  Estado.
 */
void meas_reset(meas_state_t *s);

/**
 * @brief Esquece as últimas medidas, mas mantém a origem da guinada.
 *
 * Chamada ao armar: a guinada continua sendo a medida desde a energização,
 * então a referência ψ = 0 leva o drone de volta à direção em que foi ligado.
 *
 * @param[in,out] s  Estado.
 */
void meas_rearm(meas_state_t *s);

/**
 * @brief Monta y a partir das leituras de um passo.
 *
 * @param[in,out] s   Estado.
 * @param[in]     in  Leituras.
 * @param[in]     p   Parâmetros (sinais, escalas do fluxo).
 * @param[out]    y   Medidas, na ordem de ::lqg_y_t.
 * @return Combinação de ::meas_flags_t com as medidas atualizadas agora.
 */
uint8_t meas_update(meas_state_t *s, const meas_input_t *in, const ctrl_params_t *p,
                    lqg_real_t y[LQG_NY]);

#endif
