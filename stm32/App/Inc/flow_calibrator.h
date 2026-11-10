/**
 * @file flow_calibrator.h
 * @brief Calibração da escala do fluxo do MTF-01 usando o giroscópio do BNO.
 *
 * Não usa régua nem distância medida por pessoa. Durante uma rotação pura (a
 * placa girando no lugar, sem transladar), um sensor de fluxo com escala
 * perfeita informaria exatamente a velocidade angular que o giroscópio mede;
 * essa é a base física da compensação de rotação em disp_estimator.c.
 * Qualquer diferença constante entre os dois, na média de muitas amostras, é
 * o erro de escala do próprio sensor, independente de quanto algo se moveu.
 *
 * Por isso este método é melhor que empurrar a placa por uma distância
 * marcada: aquilo é tão preciso quanto quem empurra (foi o que fez os fatores
 * 0,79 e 0,55 de dois ensaios manuais discordarem), enquanto o giroscópio é
 * uma referência independente e precisa.
 *
 * Os limites e filtros são os mesmos do AP_OpticalFlow_Calibrator.cpp do
 * ArduPilot, então uma manobra aceita por ele também é aceita aqui.
 */
#ifndef FLOW_CALIBRATOR_H
#define FLOW_CALIBRATOR_H
#include <stdbool.h>
#include <stdint.h>

/** Taxa mínima, tanto no fluxo lido quanto no esperado pelo giroscópio, para
    uma amostra ser sinal e não ruído (20 graus/s). Equivale a
    AP_OPTICALFLOW_CAL_ROLLPITCH_MIN_RADS. */
#define FLOW_CAL_MIN_RATE_RAD_S 0.349066F
/** Guinada máxima permitida durante a amostra (20 graus/s). Equivale a
    AP_OPTICALFLOW_CAL_YAW_MAX_RADS. */
#define FLOW_CAL_MAX_YAW_RAD_S  0.349066F
/** Menor fator de escala aceito. Equivale a AP_OPTICALFLOW_CAL_SCALE_MIN. */
#define FLOW_CAL_SCALE_MIN      0.20F
/** Maior fator de escala aceito. Equivale a AP_OPTICALFLOW_CAL_SCALE_MAX. */
#define FLOW_CAL_SCALE_MAX      4.00F

/**
 * @brief Acumuladores da calibração de um eixo.
 */
typedef struct {
  /** Soma de (melhor fator da amostra x |resíduo|). */
  double weighted_scalar_sum;
  /** Soma de |resíduo|, que também é o peso total. */
  double residual_sum;
  /** Amostras aceitas. */
  uint32_t samples;
} flow_cal_axis_t;

/**
 * @brief Estado completo da calibração, com os dois eixos.
 */
typedef struct {
  /** Índice 0 = X e 1 = Y, na numeração de eixos que quem chama usar. */
  flow_cal_axis_t axis[2];
} flow_calibrator_t;

/**
 * @brief Zera os acumuladores dos dois eixos.
 *
 * @param[out] c  Calibrador.
 */
void flow_cal_reset(flow_calibrator_t *c);

/**
 * @brief Oferece uma amostra candidata para um eixo.
 *
 * @p expected_flow deve ser calculado por quem chama, a partir do giroscópio,
 * com a mesma convenção de sinal da compensação de rotação do estimador
 * (esperado = -gyro_sign * giro do eixo correspondente). Este módulo aceita a
 * comparação como vier e nunca usa gyro_sign, para que um erro de sinal num
 * lugar não cancele em silêncio um erro no outro.
 *
 * A amostra é recusada se algum valor não for finito, se o fluxo lido ou o
 * esperado estiver abaixo de ::FLOW_CAL_MIN_RATE_RAD_S, ou se a guinada
 * passar de ::FLOW_CAL_MAX_YAW_RAD_S.
 *
 * @param[in,out] c              Calibrador.
 * @param[in]     axis           Eixo: 0 ou 1.
 * @param[in]     raw_flow       Leitura do sensor neste eixo, sem escala, em rad/s.
 * @param[in]     expected_flow  O que um sensor com escala perfeita leria no
 *                               mesmo instante, em rad/s.
 * @param[in]     yaw_rate       Velocidade de guinada, em rad/s. Usada só como
 *                               filtro: acima de 20 graus/s a amostra é
 *                               descartada, porque girar em torno do yaw não
 *                               calibra este eixo e só adiciona ruído.
 * @return true se a amostra foi aceita. Serve só para diagnóstico; aceitação
 *         baixa significa "gire mais rápido na rolagem/arfagem e mais devagar
 *         no yaw".
 */
bool flow_cal_add_sample(flow_calibrator_t *c, int axis, float raw_flow,
                          float expected_flow, float yaw_rate);

/**
 * @brief Calcula o fator de correção de um eixo.
 *
 * Multiplicar o fluxo bruto do sensor por este fator corrige a escala. O
 * resultado é recusado fora de [::FLOW_CAL_SCALE_MIN, ::FLOW_CAL_SCALE_MAX],
 * o mesmo limite do ArduPilot: um sensor (ou uma manobra) que exige correção
 * maior que 5x para qualquer lado está com defeito ou foi mal executado.
 *
 * @param[in]  c             Calibrador.
 * @param[in]  axis          Eixo: 0 ou 1.
 * @param[out] scalar_out    Fator calculado. Não é alterado se a função
 *                           retornar false.
 * @param[out] residual_out  Opcional (pode ser NULL). Recebe a média de
 *                           |esperado - bruto| antes da correção, em rad/s.
 *                           Não é erro: quanto mais brusca a manobra ou mais
 *                           ruidoso o sensor, maior esse valor.
 * @return false se não houver amostras aceitas ou se o fator sair da faixa.
 */
bool flow_cal_result(const flow_calibrator_t *c, int axis, float *scalar_out,
                      float *residual_out);

/**
 * @brief Mesmo cálculo de flow_cal_result(), mas sem o limite de faixa.
 *
 * Serve para diagnosticar por que uma calibração não converge, não para uso
 * real. Um valor negativo quase sempre indica que o gyro_sign (em
 * disp_config_t) está com o sinal errado, e não que o sensor esteja muito
 * fora de especificação. Um módulo muito grande (dezenas ou mais) costuma
 * indicar que a manobra misturou translação real; a média ponderada é
 * sensível a isso, então segure a placa mais parada no lugar.
 *
 * @param[in]  c           Calibrador.
 * @param[in]  axis        Eixo: 0 ou 1.
 * @param[out] scalar_out  Fator calculado, mesmo fora da faixa.
 * @return false somente quando não há nenhuma amostra.
 */
bool flow_cal_raw_scalar(const flow_calibrator_t *c, int axis, float *scalar_out);
#endif
