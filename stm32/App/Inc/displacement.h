/**
 * @file displacement.h
 * @brief Deslocamento horizontal a partir de fluxo óptico, telêmetro e IMU.
 *
 * Um sensor de fluxo óptico não mede translação. Ele mede a velocidade
 * angular com que a cena passa pela imagem, e essa taxa é a soma de duas
 * coisas: o veículo transladando sobre o chão e o veículo girando no lugar.
 * Uma câmera voltada para baixo que inclina vê o chão se mover exatamente
 * como uma câmera que deslizou de lado. Integrar a leitura bruta, portanto,
 * acumula cada rolagem e arfagem como se fosse deslocamento; é o motivo
 * típico de um movimento de 300 mm ser lido como 350 mm ou 250 mm.
 *
 * A correção, a mesma que o ArduPilot faz antes de levar o fluxo ao seu EKF,
 * é subtrair a velocidade angular medida pelo giroscópio. O que sobra é só a
 * parte da translação, e multiplicá-la pela distância dá uma velocidade real.
 * Outras duas correções importam tanto quanto essa:
 *
 *   - a velocidade está nos eixos do *corpo*, então precisa ser girada para um
 *     referencial fixo pelo yaw atual antes de ser acumulada, senão um veículo
 *     que gira embaralha a própria trajetória;
 *   - amostras com imagem ruim, distância inválida ou inclinação grande são
 *     piores que nenhuma amostra, então são rejeitadas em vez de integradas.
 *
 * Três estimadores rodam ao mesmo tempo e são lidos separadamente (RAW, FUSED
 * e EKF), para comparar na bancada o mesmo movimento físico.
 *
 * Funções puras (disp_raw_update(), disp_fused_update() e as inline) não usam
 * HAL e são testadas no PC. As demais funções pertencem à task de
 * deslocamento (displacement.c).
 */
#ifndef DISPLACEMENT_H
#define DISPLACEMENT_H
#include <stdbool.h>
#include <stdint.h>
#include "attitude.h"
#include "flow_ekf.h"
#include "flow_calibrator.h"

/**
 * @brief Identifica os três estimadores.
 *
 * @note Não é usado pelo código atual; os três estimadores são lidos pelos
 *       campos de displacement_report_t.
 */
typedef enum {
  /** Fluxo x distância, eixos do corpo, sem nenhuma compensação. */
  DISP_METHOD_RAW = 0,
  /** Compensado pelo giroscópio e girado pelo yaw, depois integrado. */
  DISP_METHOD_FUSED,
  /** A mesma medida do FUSED, fundida no filtro de flow_ekf.h. */
  DISP_METHOD_EKF
} disp_method_t;

/**
 * @brief Estado acumulado de um estimador integrador (RAW ou FUSED).
 */
typedef struct {
  /** Deslocamento acumulado no eixo X, em m. */
  float x;
  /** Deslocamento acumulado no eixo Y, em m. */
  float y;
  /** Velocidade X do frame aceito mais recente, em m/s. */
  float vx;
  /** Velocidade Y do frame aceito mais recente, em m/s. */
  float vy;
  /** Quanto esse frame somou em x, em m. */
  float last_dx;
  /** Quanto esse frame somou em y, em m. */
  float last_dy;
  /** Frames integrados. */
  uint32_t samples;
  /** Frames rejeitados pelos filtros de validade; sempre 0 no estimador RAW. */
  uint32_t rejected;
  /** Meia largura da célula de quantização da velocidade, por eixo do
      referencial local. Zero quando o eixo não veio quantizado como zero. */
  float quant_half_width[2];
  /** Yaw que define o referencial local desde o primeiro frame aceito. */
  float yaw_origin;
  /** O referencial local já foi ancorado. */
  bool yaw_origin_set;
  /** Estado da histerese de qualidade da imagem. */
  bool quality_usable;
} disp_state_t;

/**
 * @brief Um frame de fluxo óptico já convertido das unidades do sensor.
 *
 * Separado para que os estimadores sejam funções puras que os testes no PC
 * conseguem alimentar diretamente.
 */
typedef struct {
  /** Fluxo X em rad/s, eixos do sensor, sem escala. */
  float flow_x;
  /** Fluxo Y em rad/s, eixos do sensor, sem escala. */
  float flow_y;
  /** Velocidade angular X do corpo, em rad/s, no mesmo instante. */
  float gyro_x;
  /** Velocidade angular Y do corpo, em rad/s, no mesmo instante. */
  float gyro_y;
  /** Velocidade angular Z média do corpo, em rad/s, no mesmo intervalo. */
  float gyro_z;
  /** Atitude w,x,y,z no mesmo instante. */
  float q[4];
  /** Distância medida no eixo do sensor, em m. */
  float range_m;
  /** Intervalo coberto por este frame, em s. */
  float dt_s;
  /** Qualidade da imagem informada pelo sensor, 0..255. */
  uint8_t quality;
  /** Frame utilizável pelo caminho FUSED. A task combina aqui distância
      válida, fluxo válido e IMU online. */
  bool range_valid;
} disp_input_t;

/**
 * @brief Menor passo de fluxo que o MTF-01 informa, em rad/s.
 *
 * O MTF-01 quantiza o fluxo em passos de 20 na sua unidade "cm/s a 1 m", o que
 * dá 0,2 rad/s por contagem. Medido em bancada (34 de 34 amostras, e depois
 * 9032 de 9032 na captura de 2026-09-16). A 0,84 m, uma contagem já vale
 * 168 mm/s, e movimentos mais lentos que isso ficam praticamente invisíveis.
 */
#define DISP_FLOW_LSB_RAD_S 0.2F

/** Atraso inicial entre o intervalo óptico e a recepção do frame. Nas duas
 * capturas reais, 10 ms eliminou falhas de cobertura BNO que ocorriam usando
 * o instante de execução da task. Deve ser recalibrado com captura dedicada. */
#define DISP_MTF_DELAY_US 10000U

/**
 * @brief Maior aceleração horizontal que um frame pode sugerir antes de ser
 *        tratado como movimento da cena, em m/s^2.
 *
 * Um sensor de fluxo não distingue "eu me movi sobre uma cena parada" de "a
 * cena se moveu sob mim parado". Medido em bancada: uma mão passando sob o
 * sensor produziu 2496 mm/s num frame de 10 ms (~250 m/s^2) com a placa
 * imóvel.
 *
 * @note Não é usado pelo código atual. O filtro de plausibilidade no
 *       integrador foi removido (enviesava o total); a rejeição de valores
 *       absurdos agora é feita pelo gate de inovação de flow_ekf.h.
 */
#define DISP_MAX_ACCEL_M_S2 40.0F

/**
 * @brief Maior intervalo aceitável entre dois frames do MTF-01, em ms.
 *
 * O sensor carimba cada frame com o próprio relógio, que volta a zero quando
 * ele reinicia; uma subtração sem sinal através desse reinício daria ~49,7
 * dias em vez de um intervalo curto, e o filtro produziria posições
 * astronômicas. Acima deste valor, perdemos a sincronia com o relógio do
 * sensor. A ~100 Hz, um intervalo real desse tamanho já são dezenas de frames.
 */
#define DISP_MAX_FRAME_GAP_MS 250U

/**
 * @brief Tempo máximo sem correção aceita pelo filtro para a estimativa ainda
 *        ser considerada válida, em ms.
 *
 * Passado esse tempo, a estimativa não é segura para guiar o veículo, seja
 * qual for a covariância.
 */
#define DISP_MAX_CORRECTION_AGE_MS 250U

/**
 * @brief Diz se o intervalo entre dois carimbos de tempo do sensor pode ser
 *        integrado.
 *
 * Compartilhada entre a task e os testes, para que a proteção testada seja a
 * mesma que roda.
 *
 * @param[in] gap_ms  Diferença entre os carimbos, em ms (sem sinal).
 * @return true se 0 < gap_ms <= ::DISP_MAX_FRAME_GAP_MS.
 */
static inline bool disp_frame_gap_usable(uint32_t gap_ms) {
  return gap_ms > 0U && gap_ms <= DISP_MAX_FRAME_GAP_MS;
}

/**
 * @brief Motivo pelo qual a estimativa não é utilizável.
 *
 * Permite que um controlador registre um motivo, e não só um indicador.
 */
typedef enum {
  /** Estimativa utilizável. */
  DISP_HEALTH_OK = 0,
  /** O filtro está só seguindo por inércia: nenhuma correção de fluxo foi
      aceita recentemente. */
  DISP_HEALTH_NO_CORRECTION,
  /** Covariância acima do limite do próprio filtro. */
  DISP_HEALTH_NOT_CONVERGED
} disp_health_t;

/**
 * @brief Decide a saúde da estimativa. É o único lugar onde a validade é
 *        decidida.
 *
 * Centralizar evita que o indicador publicado e qualquer consumidor futuro
 * divirjam.
 *
 * @param[in] converged          Estado de convergência do filtro.
 * @param[in] correction_age_ms  Idade da última correção que o filtro
 *                               ACEITOU, não do último frame recebido. Frames
 *                               continuam chegando com a IMU offline ou o
 *                               fluxo rejeitado, enquanto o filtro só segue
 *                               por inércia.
 * @return ::DISP_HEALTH_NOT_CONVERGED, ::DISP_HEALTH_NO_CORRECTION ou
 *         ::DISP_HEALTH_OK, nesta ordem de prioridade.
 */
static inline disp_health_t disp_health(bool converged, uint32_t correction_age_ms) {
  if (!converged) return DISP_HEALTH_NOT_CONVERGED;
  if (correction_age_ms > DISP_MAX_CORRECTION_AGE_MS) return DISP_HEALTH_NO_CORRECTION;
  return DISP_HEALTH_OK;
}

/**
 * @brief Calibração e ajustes aplicados pelos estimadores.
 */
typedef struct {
  /** Correção de escala do eixo X (1.0 = nenhuma). Lente e sensor podem errar
      diferente em cada eixo. Medida pelo procedimento de flow_calibrator.h
      (referenciado no giroscópio, não em régua). */
  float flow_scale_x;
  /** Correção de escala do eixo Y (1.0 = nenhuma). */
  float flow_scale_y;
  /** Termos fora da diagonal da matriz de alinhamento 2x2. Zero mantém a
      calibração diagonal histórica. */
  float flow_cross_xy;
  float flow_cross_yx;
  /**
   * Sinal do termo do giroscópio no eixo X: +1 ou -1.
   *
   * Os sinais são independentes por eixo, não um valor compartilhado: na
   * bancada os dois eixos discordaram de sinal nesta placa (fluxo X oposto ao
   * giro X e fluxo Y no mesmo sentido do giro Y, 98% e 99% consistentes,
   * centenas de amostras cada). Isso é coerente com os dois sensores montados
   * com rotação relativa de 90 graus, que inverte exatamente um dos eixos. Ver
   * displacement_set_gyro_sign().
   */
  float gyro_sign_x;
  /** Sinal do termo do giroscópio no eixo Y: +1 ou -1. */
  float gyro_sign_y;
  /**
   * Zona morta opcional sobre a taxa já *compensada*, em rad/s. Padrão 0
   * (desligada).
   *
   * Na bancada, aplicar aqui um limite de uma contagem de fluxo piorou a
   * deriva parada: uma contagem é a leitura mais comum, e valores minúsculos
   * do giroscópio passavam a decidir ao acaso quais amostras sobreviviam. O
   * movimento inventado que ela deveria impedir é tratado sempre, confiando no
   * zero que o sensor reportou. Mantida só para experimentos.
   */
  float deadband_rad_s;
  /** Posição do ponto focal do MTF em relação ao centro do veículo, em m. */
  float sensor_offset_m[3];
} disp_config_t;

/**
 * @brief Estimador ingênuo (RAW): fluxo x distância, integrado nos eixos do
 *        corpo.
 *
 * Não remove rotação, não rejeita frames e não considera o yaw. Existe como
 * referência para comparar com o caminho compensado.
 *
 * @param[in,out] s    Estado acumulado.
 * @param[in]     in   Frame de entrada.
 * @param[in]     cfg  Calibração (só a escala é usada).
 */
void disp_raw_update(disp_state_t *s, const disp_input_t *in, const disp_config_t *cfg);

/**
 * @brief Estimador compensado (FUSED): remove a rotação, gira pelo yaw e
 *        integra no referencial do mundo.
 *
 * Rejeita o frame (e soma em @c rejected) se: @c range_valid for falso,
 * qualidade não passar pela histerese 30/15, inclinação maior que ~37 graus, distância fora de
 * 0,08..8 m, @c dt_s fora de 0,001..0,25 s, ou fluxo não finito ou acima de
 * 4,2 rad/s. A meia célula de quantização é publicada para o EKF aumentar a
 * incerteza da medida sem criar uma zona morta de correção.
 *
 * @param[in,out] s    Estado acumulado.
 * @param[in]     in   Frame de entrada.
 * @param[in]     cfg  Calibração e ajustes.
 * @return true se o frame foi integrado; false se foi rejeitado.
 */
bool disp_fused_update(disp_state_t *s, const disp_input_t *in, const disp_config_t *cfg);

/* ---- API da task ------------------------------------------------------ */

/**
 * @brief Tudo o que um consumidor precisa, numa única cópia consistente.
 *
 * Os estimadores, a atitude usada por eles e o frame de origem saem juntos,
 * para não discordarem entre si.
 */
typedef struct {
  /** Estimador RAW (eixos do corpo). */
  disp_state_t raw;
  /** Estimador FUSED (eixos do mundo). */
  disp_state_t fused;
  /** Terceiro estimador: mesma entrada, filtro de Kalman. */
  flow_ekf_t ekf;
  /** Atitude do último frame, em ângulos de Euler. */
  euler_t attitude;
  /** A mesma atitude como quatérnio w,x,y,z. */
  float q[4];
  /** Distância bruta do último frame, no eixo do sensor, em m. */
  float range_m;
  /** A mesma distância corrigida pela inclinação (range x cos(inclinação)), em m. */
  float height_m;
  /** Intervalo coberto pelo último frame, em s. */
  float dt_s;
  /** Velocidade que uma contagem de fluxo representa nesta altura, em m/s.
      Cresce com a altura e é o piso honesto do que o par consegue resolver
      agora: nada mais lento é mensurável, por mais que se integre. */
  float v_lsb_m_s;
  /** Qualidade da imagem do último frame. */
  uint8_t quality;
  /** Tempo desde o último frame integrado, em ms. Útil para detectar um
      enlace de sensor morto, mas NÃO basta para confiar na estimativa; ver
      @c correction_age_ms. */
  uint32_t age_ms;
  /** Tempo desde a última correção ACEITA pelo filtro, em ms. É esta idade
      que importa: frames podem continuar chegando e todos serem rejeitados
      (IMU offline, distância ruim, gate), e o filtro então só segue por
      inércia. */
  uint32_t correction_age_ms;
  /** Por que a estimativa não é utilizável, quando não é. */
  disp_health_t health;
  /** Frames descartados porque o relógio do MTF-01 saltou (reinício do
      sensor) ou o intervalo era longo demais para integrar. Contagem
      subindo indica enlace ruim com o sensor, não movimento. */
  uint32_t clock_resyncs;
  /** Frames descartados porque o histórico do BNO não cobria o intervalo. */
  uint32_t imu_alignment_rejects;
  /** Leituras de alcance rejeitadas pelo filtro. */
  uint32_t range_rejects;
  /** Mudanças persistentes de alcance aceitas como um novo patamar. */
  uint32_t range_reanchors;
  /** O único indicador em que um controlador deve se basear. Igual a
      disp_health(...) == ::DISP_HEALTH_OK. */
  bool valid;
  /** Calibração ao vivo em andamento; ver displacement_calibrate_start().
      Os campos cal_* podem ser ignorados quando ela não está rodando. */
  bool cal_running;
  /** Por eixo: flow_cal_result() já teve sucesso. */
  bool cal_result_valid[2];
  /** Por eixo: fator de escala que seria aplicado agora. */
  float cal_scalar[2];
  /** Por eixo: existe fator calculado sem o limite de faixa (basta uma
      amostra). Ver flow_cal_raw_scalar(). */
  bool cal_raw_valid[2];
  /** Por eixo: fator calculado sem o limite de faixa, para diagnosticar uma
      calibração presa em valid=0. */
  float cal_scalar_raw[2];
  /** Por eixo: discordância média antes da correção, em rad/s. */
  float cal_residual[2];
  /** Por eixo: amostras aceitas até agora. */
  uint32_t cal_samples[2];
} displacement_report_t;

/**
 * @brief Troca os parâmetros do filtro. Vale a partir do próximo frame.
 *
 * @param[in] cfg  Parâmetros (ver flow_ekf.h). O ponteiro é guardado, não
 *                 copiado: deve apontar para memória que continue válida.
 *                 NULL é ignorado.
 */
void displacement_set_ekf_config(const flow_ekf_config_t *cfg);

/**
 * @brief Cria a task de deslocamento.
 *
 * Deve ser chamada depois de mtf_start() e bno_start(). No build Capture, a
 * task só integra enquanto a sessão de captura está ativa.
 *
 * @return true se a task foi criada.
 */
bool displacement_start(void);

/**
 * @brief Copia o relatório mais recente, de forma segura entre tasks.
 *
 * @param[out] out  Destino da cópia.
 */
void displacement_get(displacement_report_t *out);

/**
 * @brief Zera os estimadores RAW e FUSED e reinicia o filtro.
 *
 * A origem da posição passa a ser o ponto atual. Hoje nada no firmware chama
 * esta função.
 *
 * @warning A task altera o estado fora da seção crítica; chamar isto de outra
 *          task pode interromper uma atualização no meio.
 */
void displacement_reset(void);

/**
 * @brief Define a correção de escala do fluxo, por eixo.
 *
 * Corrige o erro sistemático do tipo "300 mm lidos como 350 mm". Prefira o
 * procedimento referenciado no giroscópio de flow_calibrator.h a medir uma
 * distância empurrada à mão: dois ensaios manuais de ~200 mm deram 0,79 e
 * 0,55, que nem concordaram entre si, porque o "quanto eu realmente empurrei"
 * tinha tanta incerteza quanto o que se queria medir. Vale para o FUSED e
 * para o EKF (que funde a velocidade do FUSED) e também para o RAW, mantendo
 * a comparação na bancada.
 *
 * @param[in] scale_x  Fator do eixo X; valores não finitos ou <= 0 são ignorados.
 * @param[in] scale_y  Fator do eixo Y; valores não finitos ou <= 0 são ignorados.
 */
void displacement_set_flow_scale(float scale_x, float scale_y);

/** Define a matriz 2x2 sensor->corpo: [xx xy; yx yy]. */
void displacement_set_flow_matrix(float xx, float xy, float yx, float yy);

/** Define o braço do ponto focal do MTF em relação ao centro do veículo. */
void displacement_set_sensor_offset(float x_m, float y_m, float z_m);

/**
 * @brief Lê a correção de escala do eixo X em uso.
 * @return Fator de escala do eixo X.
 */
float displacement_get_flow_scale_x(void);

/**
 * @brief Lê a correção de escala do eixo Y em uso.
 * @return Fator de escala do eixo Y.
 */
float displacement_get_flow_scale_y(void);

/**
 * @brief Inicia a calibração ao vivo de flow_calibrator.h com os dados que
 *        estão chegando.
 *
 * Depois de chamar, balance o veículo em rolagem e em arfagem, um eixo por
 * vez, sem transladar e sem girar no yaw, por algumas dezenas de segundos.
 * Com LOG_CALIBRATION ligado em sensor_log.h, o progresso aparece no log.
 * Os acumuladores anteriores são zerados.
 */
void displacement_calibrate_start(void);

/**
 * @brief Interrompe a calibração ao vivo, mantendo o que foi acumulado.
 */
void displacement_calibrate_stop(void);

/**
 * @brief Informa se a calibração ao vivo está rodando.
 * @return true enquanto a calibração estiver ativa.
 */
bool displacement_calibrate_is_running(void);

/**
 * @brief Aplica o resultado da calibração via displacement_set_flow_scale().
 *
 * Seguro para chamar enquanto se acompanha o log: se algum eixo ainda não
 * tiver resultado válido, nada muda.
 *
 * @return true se os dois eixos tinham resultado válido e a escala foi
 *         aplicada. A escala não persiste após reiniciar.
 */
bool displacement_calibrate_apply(void);

/**
 * @brief Define o sinal do termo do giroscópio na compensação, por eixo.
 *
 * Ver o comentário de disp_config_t::gyro_sign_x sobre por que um valor
 * único não funciona nesta placa. Para conferir: na bancada, isole um eixo
 * por vez (só arfagem, depois só rolagem, sem sair do lugar) e observe a
 * velocidade FUSED daquele eixo. Ela deve ficar perto de zero; com o sinal
 * errado, fica aproximadamente o dobro da leitura RAW. Valores medidos
 * assim e usados por padrão: +1 (X) e -1 (Y).
 *
 * @param[in] sign_x  Sinal do eixo X; >= 0 vira +1, negativo vira -1.
 * @param[in] sign_y  Sinal do eixo Y; >= 0 vira +1, negativo vira -1.
 */
void displacement_set_gyro_sign(float sign_x, float sign_y);

/**
 * @brief Lê o sinal do giroscópio em uso no eixo X.
 * @return +1 ou -1.
 */
float displacement_get_gyro_sign_x(void);

/**
 * @brief Lê o sinal do giroscópio em uso no eixo Y.
 * @return +1 ou -1.
 */
float displacement_get_gyro_sign_y(void);

/**
 * @brief Define a zona morta opcional sobre a taxa compensada.
 *
 * @param[in] rad_s  Zona morta em rad/s; 0 desliga (padrão). Valores não
 *                   finitos ou negativos são ignorados.
 */
void displacement_set_deadband(float rad_s);

/**
 * @brief Lê a zona morta em uso.
 * @return Zona morta em rad/s.
 */
float displacement_get_deadband(void);

/**
 * @brief Retorna os parâmetros do filtro em uso.
 *
 * Retornado por ponteiro porque quem chama só lê. A captura usa isto para
 * gravar o que o replay precisa reproduzir.
 *
 * @return Ponteiro para os parâmetros atuais (não pode ser alterado).
 */
const flow_ekf_config_t *displacement_get_ekf_config(void);
#endif
