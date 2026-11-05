/**
 * @file trim.h
 * @brief Ajuste lento (integrador) da rotação de hover e do equilíbrio da atitude.
 *
 * O LQR do TCC não tem integrador: com a rotação de hover (`CTL_W0`) um pouco
 * errada, ou com o peso mal distribuído, o drone para num erro de altura ou de
 * inclinação que o ganho proporcional não fecha (cada 10 rad/s de erro no hover
 * dão ~30 cm de erro de altura). Este módulo soma três termos lentos à rotação
 * de cada motor, em rad/s:
 *
 * - `z`: igual nos quatro, integra o erro de altura (hover);
 * - `theta`: soma na frente e subtrai atrás, integra a arfagem θ;
 * - `phi`: soma na esquerda e subtrai na direita, integra a rolagem φ.
 *
 * Os sinais seguem as colunas da matriz B do modelo (u1..u4), então valem para
 * qualquer ordem física dos motores (o mixer cuida disso depois).
 *
 * A malha de altura do LQR é lenta (frequência natural de ~1 rad/s), então o
 * ganho do termo `z` precisa ficar abaixo de ~50 rad/s por (m·s) para não
 * oscilar. Isso deixaria a saída do chão lenta demais (tens de segundos), por
 * isso, enquanto o drone ainda está no chão e o alvo está acima, `z` sobe numa
 * rampa fixa (`ground_rate`) até ele se soltar; no ar volta o integrador lento.
 *
 * Enquanto a arfagem ou a rolagem estiverem no limite, o hover não sobe: mais
 * empuxo igual nos quatro iria todo para o lado leve.
 *
 * Não é parte do controle validado no MATLAB: é um acréscimo do teste, ligado
 * por CTL_TRIM (padrão desligado). Código puro, testado em tests/test_trim.c.
 */
#ifndef CONTROL_TRIM_H
#define CONTROL_TRIM_H

#include <stdbool.h>

/** Quantidade de entradas do modelo. */
#define TRIM_INPUTS 4U

/** @brief Termos acumulados, em rad/s. */
typedef struct {
  float z;      /**< Hover: somado igualmente nas quatro entradas. */
  float theta;  /**< Arfagem: + na frente (u1, u2), - atrás (u3, u4). */
  float phi;    /**< Rolagem: + na esquerda (u1, u4), - na direita (u2, u3). */
} trim_state_t;

/** @brief Ganhos e limites. */
typedef struct {
  float ki_z;        /**< Ganho do hover, em rad/s por (m·s) de erro de altura. */
  float ki_att;      /**< Ganho da atitude, em rad/s por (rad·s) de erro de ângulo. */
  float ground_rate; /**< Rampa de `z` enquanto o drone não se soltou do chão, em rad/s por s. */
  float settled_vz;  /**< `z` só integra no ar com |vz| abaixo disto, em m/s (assentado). */
  float max_z;       /**< Limite de |z|, em rad/s. */
  float max_att;     /**< Limite de |theta| e |phi|, em rad/s. */
} trim_config_t;

/** @brief Zera os termos. */
void trim_reset(trim_state_t *t);

/**
 * @brief Avança um passo.
 *
 * Congela (não integra) enquanto @p enabled for falso, por exemplo com os
 * motores saturados, em ensaio a seco ou sem medida de altura.
 *
 * @param[in,out] t         Termos.
 * @param[in]     cfg       Ganhos e limites.
 * @param[in]     enabled   Integrar neste passo.
 * @param[in]     dt        Período do passo, em s.
 * @param[in]     grounded  O drone ainda está no chão (ainda não se soltou).
 * @param[in]     vz        Velocidade vertical estimada, em m/s (z NED).
 * @param[in]     z_error   Altura estimada menos referência, z NED (m): positivo = abaixo do alvo.
 * @param[in]     phi       Rolagem estimada, em rad.
 * @param[in]     theta     Arfagem estimada, em rad.
 */
void trim_update(trim_state_t *t, const trim_config_t *cfg, bool enabled, float dt,
                 bool grounded, float vz, float z_error, float phi,
                 float theta);

/**
 * @brief Soma por entrada do modelo (u1..u4), em rad/s, a ser acrescentada ao du.
 *
 * @param[in]  t     Termos.
 * @param[out] out   Valor de cada entrada.
 */
void trim_values(const trim_state_t *t, float out[TRIM_INPUTS]);

#endif
