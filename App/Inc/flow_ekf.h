/**
 * @file flow_ekf.h
 * @brief Filtro de Kalman de posição e velocidade horizontais, com fluxo
 *        óptico e IMU.
 *
 * **O que ele entrega**, comparado a passar o mesmo sinal por um integrador
 * simples: a velocidade estimada cai dos 160 mm/s de oscilação de
 * quantização para 5,6 mm/s (redução de 28x) e continua sem viés quando
 * medidas são descartadas. Um integrador simples perde exatamente o valor de
 * cada amostra descartada, e foi assim que uma placa parada na bancada chegou
 * a 21 mm de "deslocamento" em dois minutos.
 *
 * **O que ele não entrega, e não tem como:** posição sem deriva. Posição não
 * é observável só a partir de medidas de velocidade, então o erro dela é a
 * integral do erro de velocidade que sobrar e cresce sem limite. No mesmo
 * sinal sintético o filtro derivou 65 mm em 60 s, contra 59 mm do integrador:
 * não é melhor. Velocidade é a saída honesta aqui, e é dela que um
 * controlador precisa; posição confiável exige referência absoluta.
 *
 * A estrutura segue o AP_NavEKF3_OptFlowFusion.cpp do ArduPilot: predição do
 * estado pelos dados inerciais e fusão do fluxo só quando a inovação passa num
 * gate de consistência (FLOW_I_GATE, lá).
 *
 * Os dois eixos horizontais são independentes: a dinâmica não os acopla e o
 * fluxo mede cada um diretamente. Por isso, em vez de um filtro 4x4, são dois
 * filtros de 2 estados; a matemática é a mesma, sem a contabilidade de
 * matrizes.
 *
 * Os eixos são do referencial do mundo, fixados pelo yaw da IMU, para que um
 * veículo que gira não embaralhe a própria trajetória.
 */
#ifndef FLOW_EKF_H
#define FLOW_EKF_H
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Estado e covariância de um eixo.
 *
 * A covariância é simétrica, então P10 == P01 e não é guardado.
 */
typedef struct {
  /** Posição no eixo, em m. */
  float p;
  /** Velocidade no eixo, em m/s. */
  float v;
  /** Variância da posição. */
  float P00;
  /** Covariância entre posição e velocidade. */
  float P01;
  /** Variância da velocidade. */
  float P11;
} flow_ekf_axis_t;

/**
 * @brief Estado completo do filtro.
 *
 * Contrato de inicialização: todo flow_ekf_t precisa ser zerado (armazenamento
 * estático ou `= {0}` explícito) ou passar por flow_ekf_reset() antes do uso.
 */
typedef struct {
  /** Eixos do mundo: índice 0 = X, 1 = Y. */
  flow_ekf_axis_t axis[2];
  /** Medidas de fluxo fundidas. */
  uint32_t updates;
  /** Medidas reprovadas no gate de inovação. */
  uint32_t rejected;
  /** Quantas vezes uma sequência de rejeições forçou o estado a adotar o sensor. */
  uint32_t resets;
  /** Rejeições consecutivas neste momento. */
  uint32_t reject_run;
  /** Última inovação de cada eixo, em m/s. */
  float innov[2];
  /** Última inovação normalizada de cada eixo; acima de 1.0 a medida é
      rejeitada. */
  float test_ratio[2];
  /** Verdadeiro quando a variância da velocidade caiu a um nível utilizável
      (desvio de ~0,3 m/s). Recalculado também na predição. */
  bool converged;
  /**
   * Protege apenas contra memória ZERADA. Vale false numa struct zerada (o que
   * o armazenamento estático entrega no boot, e o que `= {0}` entrega em
   * qualquer lugar), e então predict/update se autoinicializam. Isso fecha o
   * risco específico de um filtro zerado, cuja variância de velocidade zero
   * anula o ganho de Kalman e ao mesmo tempo satisfaz o teste de convergência.
   *
   * NÃO torna segura uma variável automática não inicializada:
   *
   *     flow_ekf_t f;                 // valor indeterminado - ler .initialized
   *     flow_ekf_predict(&f, ...);    // já é comportamento indefinido
   *
   * É uma rede de segurança para o primeiro caso, não licença para pular a
   * inicialização.
   */
  bool initialized;
} flow_ekf_t;

/**
 * @brief Parâmetros de ajuste do filtro.
 */
typedef struct {
  /** Desvio padrão da aceleração que alimenta a predição, em m/s^2. Cobre o
      ruído do acelerômetro e tudo o que o modelo deixa de fora; é o ajuste de
      "quão rápido o filtro pode mudar de ideia". */
  float accel_noise;
  /** Ruído da medida de fluxo como taxa angular, em rad/s. Vira ruído de
      velocidade ao ser multiplicado pela distância. O padrão do ArduPilot
      (FLOW_M_NSE) é 0,25. */
  float flow_noise_rad_s;
  /** Gate de inovação, em desvios padrão. O padrão do ArduPilot (FLOW_I_GATE
      = 300) é este valor vezes 100. */
  float gate_sigma;
} flow_ekf_config_t;

/** Parâmetros padrão: accel_noise 0,5; flow_noise_rad_s 0,25; gate_sigma 3,0. */
extern const flow_ekf_config_t FLOW_EKF_DEFAULTS;

/**
 * @brief Coloca o filtro no estado inicial.
 *
 * Posição zero e conhecida (é a origem por definição), velocidade zero com
 * variância 1, contadores zerados e @c converged falso.
 *
 * @param[out] f  Filtro.
 */
void flow_ekf_reset(flow_ekf_t *f);

/**
 * @brief Etapa de predição.
 *
 * Pode ser chamada com aceleração zero: o estado segue por inércia e a
 * incerteza cresce, que é exatamente o que deve acontecer quando nada se
 * sabe. Um @p dt não positivo ou não finito é ignorado, e acelerações não
 * finitas são tratadas como zero.
 *
 * @param[in,out] f    Filtro (autoinicializado se estiver zerado).
 * @param[in]     ax   Aceleração horizontal X no referencial do mundo, em
 *                     m/s^2, sem gravidade.
 * @param[in]     ay   Aceleração horizontal Y no referencial do mundo, em
 *                     m/s^2, sem gravidade.
 * @param[in]     dt   Intervalo, em s.
 * @param[in]     cfg  Parâmetros do filtro.
 */
void flow_ekf_predict(flow_ekf_t *f, float ax, float ay, float dt,
                      const flow_ekf_config_t *cfg);

/**
 * @brief Etapa de correção com uma velocidade derivada do fluxo.
 *
 * O gate é aplicado ao par X/Y de uma vez: a medida é uma observação de um
 * único vetor de velocidade, e fundir só metade dela puxaria a estimativa
 * para o eixo que passou. Após 50 rejeições seguidas (~0,5 s a 100 Hz) o
 * filtro conclui que a predição é que está errada, adota a medida, alarga a
 * incerteza e retorna true (conta em @c resets). Valores não finitos ou
 * distância não positiva são rejeitados.
 *
 * @param[in,out] f        Filtro (autoinicializado se estiver zerado).
 * @param[in]     vx       Velocidade X no referencial do mundo, em m/s.
 * @param[in]     vy       Velocidade Y no referencial do mundo, em m/s.
 * @param[in]     range_m  Distância do sensor, em m. Escala o ruído da
 *                         medida, porque o mesmo erro angular vira um erro de
 *                         velocidade maior quanto mais alto se está.
 * @param[in]     cfg      Parâmetros do filtro.
 * @return false quando o gate de inovação rejeitou a medida. Não é erro: o
 *         filtro apenas mantém a predição.
 */
bool flow_ekf_update(flow_ekf_t *f, float vx, float vy, float range_m,
                     const flow_ekf_config_t *cfg);

/** Correção com célula de quantização por eixo.
 *
 * A meia largura soma variância uniforme (h²/3) à medida, sem criar uma zona
 * morta de inovação. @p quality é mantido no contrato e na captura, mas só o
 * gate com histerese do estimador o usa enquanto não houver calibração que
 * relacione a escala 0..255 a uma variância física.
 */
bool flow_ekf_update_observation(flow_ekf_t *f, float vx, float vy,
                                 float range_m, uint8_t quality,
                                 float half_width_x, float half_width_y,
                                 const flow_ekf_config_t *cfg);
#endif
