/**
 * @file lqg_ctrl.h
 * @brief Controle LQR + filtro de Kalman do modo de teste com cabo.
 *
 * Código puro (sem HAL nem FreeRTOS), testado no PC. Envolve o modelo gerado
 * pelo Simulink (control/lqg_model, `STM32_step()` a 100 Hz):
 *
 *   x̂ = Kalman(y)            12 estados [x y z φ θ ψ u v w p q r], NED/FRD
 *   Δω = Kfd·r − Kd·x̂        r = [x_ref y_ref z_ref ψ_ref]
 *   motor = hover + Δω / ::LQG_W_PER_PERMILLE
 *
 * Em volta do modelo ficam só o que ele não tem: sequência de armar, decolar,
 * pairar e pousar, conversão dos sensores (control/lqg_meas.h), mistura nos
 * motores físicos e proteções (inclinação, sensores, altura, tempo).
 *
 * **Extra fora do modelo gerado:** o LQR não tem ação integral, então um erro
 * no hover configurado vira erro de altura (~20 ‰ de aceleração por metro).
 * Um integrador de altura (@c trim, ::lqg_params_t::trim_ki) acha o hover
 * real e pode ser desligado na IHM; o ganho efetivo é limitado a uma fração
 * do limite de estabilidade da malha com os multiplicadores em uso. Ele não
 * entra no Kalman. A saída é escalada pela tensão da bateria quando medida.
 *
 * Unidades: altura e distâncias em m (altura positiva para cima, o modelo usa
 * z para baixo), velocidades em m/s, ângulos em rad, aceleração dos motores em
 * décimos de por cento (‰), tempo em passos de ::LQG_TS_S.
 *
 * O modelo gerado usa variáveis globais: existe uma única instância e
 * ::lqg_ctrl_step() não é reentrante.
 */
#ifndef CONTROL_LQG_CTRL_H
#define CONTROL_LQG_CTRL_H

#include "control/ekf3.h"
#include "control/lqg_meas.h"

#include <stdbool.h>
#include <stdint.h>

/** Limites dos parâmetros editáveis (também validados em link/bench_proto.c e na IHM). */
#define LQG_PARAM_HEIGHT_MIN_M  (0.10F)
#define LQG_PARAM_HEIGHT_MAX_M  (0.80F)
#define LQG_PARAM_HOVER_MIN     (100.0F)
#define LQG_PARAM_HOVER_MAX     (900.0F)
#define LQG_PARAM_CAP_MIN       (200.0F)
#define LQG_PARAM_CAP_MAX       (1000.0F)
#define LQG_PARAM_CLIMB_MIN_M_S (0.02F)
#define LQG_PARAM_CLIMB_MAX_M_S (0.50F)
#define LQG_PARAM_SPOOL_MIN     (5.0F)
#define LQG_PARAM_SPOOL_MAX     (200.0F)
/**
 * Teto do parâmetro do integrador de altura, em ‰/(m·s). O valor usado é o menor
 * entre o parâmetro e 10 % do limite de estabilidade (::lqg_ctrl_trim_ki):
 * ~2,6 com os ganhos gerados, ~131 com altura P 10× e D 5×.
 */
#define LQG_PARAM_TRIM_KI_MAX   (200.0F)
#define LQG_PARAM_KI_XY_MAX     (0.5F)
#define LQG_PARAM_KI_YAW_MAX    (2.0F)

/** @brief Parâmetros editáveis em voo (valem a partir do próximo passo). */
typedef struct {
  /** Altura do hover acima do chão do armamento, em m. */
  float height_m;
  /**
   * Aceleração de hover, em ‰, na tensão de referência (::LQG_VBAT_REF_V_PER_CELL
   * por célula). Só define até onde a rampa de decolagem sobe rápido: no ar, a
   * base parte da aceleração em que o drone de fato descolou.
   */
  float hover_permille;
  /** Limite de aceleração de qualquer motor, em ‰. */
  float cap_permille;
  /** Velocidade da referência de altura ao subir, em m/s. */
  float climb_m_s;
  /** Velocidade da referência de altura ao descer, em m/s. */
  float land_m_s;
  /** Aumento da aceleração até descolar, em ‰/s. */
  float spool_permille_s;
  /** Ganho do integrador de altura, em ‰ por (m·s) de erro (limitado, ver ::lqg_ctrl_trim_ki). */
  float trim_ki;
  /** Ganho do integrador de posição XY, em 1/s (equivalente em posição por m·s de erro). */
  float ki_xy;
  /** Ganho do integrador de guinada, em 1/s. */
  float ki_yaw;
  /** Liga o ajuste lento do hover. */
  bool autotrim;
  /** Usa o EKF3 com quatérnio (control/ekf3.h); desligado, usa o Kalman estacionário do modelo gerado. */
  bool use_ekf;
} lqg_params_t;

/**
 * @brief Multiplicadores dos ganhos Kd gerados, por grupo de estados (1 = original).
 *
 * Só valem com o EKF (com "usar EKF" desligado o modelo gerado usa os próprios
 * ganhos). O controle usa a forma de erro Δω = −Kd·(x − x_ref): cada grupo
 * escala as colunas de Kd dos seus estados, e a referência de altura acompanha
 * o ganho de z. P = posição/ângulo, D = velocidade/taxa.
 */
typedef struct {
  float z;     /**< Altura (coluna z). */
  float w;     /**< Velocidade vertical (coluna w). */
  float att;   /**< Rolagem e arfagem (colunas phi, theta). */
  float rate;  /**< Taxas de rolagem e arfagem (colunas p, q). */
  float yaw;   /**< Guinada (coluna psi). */
  float yaw_rate; /**< Taxa de guinada (coluna r). */
  /**
   * Posição horizontal (colunas x, y e os integradores XY), RELATIVA a @c att.
   * O controle de posição pede inclinação: graus por metro = Kx/Kθ. Escalar só
   * a atitude deixava a posição proporcionalmente mais fraca (atitude 5× e XY 1×
   * davam 1/5 da inclinação por metro do projeto). O efeito é g.xy · g.att.
   */
  float xy;
  float vel;   /**< Velocidade horizontal (colunas u, v), também relativa a @c att. */
} lqg_gains_t;

/** Limites dos multiplicadores (ver ::lqg_gains_valid). */
#define LQG_GAIN_MAX       (20.0F)
#define LQG_GAIN_ATT_MIN   (0.25F)
#define LQG_GAIN_ATT_MAX   (5.0F)
#define LQG_GAIN_OTHER_MAX (10.0F)

/** @brief Fase do voo. */
typedef enum {
  LQG_STATE_IDLE = 0, /**< Desarmado. */
  LQG_STATE_ARMED,    /**< Armado no chão, motores em zero, medindo o chão. */
  LQG_STATE_SPOOL,    /**< Subindo a aceleração até descolar. */
  LQG_STATE_CLIMB,    /**< No ar, referência de altura ainda em movimento. */
  LQG_STATE_HOVER,    /**< No ar, referência de altura no alvo. */
  LQG_STATE_LAND      /**< Descendo ou reduzindo os motores depois do toque. */
} lqg_state_t;

/** Motivos de parada (::lqg_out_t::fail). */
#define LQG_FAIL_NONE       0U
#define LQG_FAIL_IMU        1U
#define LQG_FAIL_TILT       2U
#define LQG_FAIL_RANGE      3U
#define LQG_FAIL_NO_LIFTOFF 4U
#define LQG_FAIL_LANDED     5U
#define LQG_FAIL_ESTIMATOR  6U

/** Bits de ::lqg_out_t::flags. */
#define LQG_TLM_FILTER_FREE 0x01U /**< Kalman em voo livre (não travado no chão). */
#define LQG_TLM_RANGE       0x02U /**< Distância válida neste passo. */
#define LQG_TLM_FLOW        0x04U /**< Fluxo válido neste passo. */
#define LQG_TLM_ATTITUDE    0x08U /**< BNO (atitude e giro) válido neste passo. */
#define LQG_TLM_TRIM        0x10U /**< Ajuste lento do hover integrando. */
#define LQG_TLM_SATURATED   0x20U /**< Algum motor limitado pelos limites. */
#define LQG_TLM_XY_ACTIVE   0x40U /**< Posição XY no controle (fluxo acima de ::LQG_FLOW_MIN_HEIGHT_M). */
#define LQG_TLM_VBAT        0x80U /**< Compensação da tensão da bateria ativa. */

/** @brief Controlador. Campos internos: não acessar diretamente. */
typedef struct {
  lqg_params_t prm;
  lqg_gains_t gains;
  /** Trim de hover de u1..u4 pelo CG deslocado, em ‰ da aceleração (soma de zero). */
  float cg_trim[4];
  lqg_state_t state;
  lqg_meas_t meas;
  uint32_t ground_n;
  float ground_sum;
  /** Soma de φ e θ crus no chão (zero de nível) e amostras usadas. */
  float level_sum[2];
  /** Soma do vetor de rotação da inclinação no chão (quatérnio do BNO), em rad. */
  float level_rv_sum[3];
  uint32_t level_n;
  float h_ref;
  /** Altura alvo, em m para cima: do parâmetro ou de ::lqg_ctrl_goto. */
  float h_goal;
  /** Referência horizontal (local, origem e guinada do armamento), em m, e a guinada, em rad. */
  float x_ref, y_ref, psi_ref;
  /** Alvo horizontal e de guinada; a referência anda até ele. */
  float x_goal, y_goal, psi_goal;
  /** Velocidade da referência horizontal, em m/s. */
  float xy_speed;
  float base;
  float trim;
  float land_bias;
  float h_est;
  ekf3_t ekf;
  float est[12];
  /** Guinada do filtro desenrolada (como a do modelo), em rad, e a última lida do quatérnio. */
  float psi_unw, psi_last;
  /** Soma de giro (p, q, r) e aceleração linear (corpo) com o drone parado e armado, e amostras. */
  float rest_sum[6];
  uint32_t rest_n;
  /** Integradores, como deslocamento equivalente: x, y locais em m e guinada em rad. */
  float ix, iy, ipsi;
  bool ekf_on;
  uint32_t imu_bad;
  uint32_t range_bad;
  uint32_t flow_bad;
  uint32_t lift_n;
  /** Base da altura na rampa (média lenta), em m: a decolagem é medida acima dela. */
  float h_floor;
  uint32_t touch_n;
  uint32_t low_batt_n;
  uint8_t last_flags;
  bool airborne;
  bool spindown;
  bool sat;
  /** Algum motor no teto / no piso neste passo (anti-windup por direção). */
  bool sat_hi, sat_lo;
  /** Origem XY fixada na primeira medida de fluxo em voo. */
  bool xy_anchored;
  /** Posição e velocidade XY entram no controle (origem fixada e altura para o fluxo). */
  bool xy_active;
  /** Tensão filtrada, em V, células e fator aplicado à saída. */
  float vbat_filt;
  uint32_t cells;
  float vbat_factor;
} lqg_ctrl_t;

/** @brief Resultado de um passo. */
typedef struct {
  /** Fase depois deste passo. */
  lqg_state_t state;
  /** Aceleração de cada motor físico (M1..M4), em ‰, já limitada. */
  float permille[4];
  /** Se não for NULL, a saída deve ser desarmada agora (texto fixo). */
  const char *stop;
  /** Se não for NULL, aviso de mudança de fase para o log (texto fixo). */
  const char *note;
  /** ::LQG_FAIL_NONE ou o motivo de @c stop. */
  uint8_t fail;
  /** ::LQG_TLM_FILTER_FREE e demais. */
  uint8_t flags;
  /** Medidas do modelo vistas neste passo (zero onde inválidas). */
  float y[LQG_NY];
  /** Estimativa de posição, em m (x frente, y direita, z PARA BAIXO, relativos ao armamento). */
  float pos[3];
  /** Estimativa de velocidade u, v, w em m/s (w para baixo). */
  float vel[3];
  /** Referência de altura, em m para cima. */
  float h_ref;
  /** Ajuste lento do hover, em ‰. */
  float trim;
  /** Aceleração base (hover + trim) antes do Δω e da compensação da bateria, em ‰. */
  float base;
  /** Referência horizontal x, y (m, local) e de guinada (rad). */
  float ref_xy[2];
  float ref_psi;
  /** Fator aplicado à saída pela tensão da bateria (1 sem medida). */
  float vbat_factor;
} lqg_out_t;

/** @brief Parâmetros padrão: 30 cm, hover 55 %, limite 95 %, integrador de altura ligado (60 ‰/(m·s)). */
lqg_params_t lqg_params_default(void);

/** @brief Todos os campos finitos e dentro dos limites ::LQG_PARAM_HEIGHT_MIN_M etc. */
bool lqg_params_valid(const lqg_params_t *p);

/** @brief Zera tudo, volta ao estado desarmado e inicializa o modelo gerado. */
void lqg_ctrl_init(lqg_ctrl_t *c);

/** @brief Todos os multiplicadores iguais a 1 (ganhos gerados). */
lqg_gains_t lqg_gains_default(void);

/**
 * @brief Multiplicadores finitos: altura (z, w) em 0..::LQG_GAIN_MAX; atitude
 *        (att, rate) em ::LQG_GAIN_ATT_MIN..::LQG_GAIN_ATT_MAX; guinada e XY em
 *        0..::LQG_GAIN_OTHER_MAX.
 */
bool lqg_gains_valid(const lqg_gains_t *g);

/** @brief Troca os multiplicadores (valem no próximo passo); false se inválidos. */
bool lqg_ctrl_set_gains(lqg_ctrl_t *c, const lqg_gains_t *g);

/**
 * @brief Fator do trim por motor: (base/::LQG_CG_TRIM_REF_PERMILLE)^4, razão limitada a
 *        0..::LQG_CG_TRIM_RATIO_MAX. Usado pelo controle e pelas plantas simuladas.
 */
float lqg_cg_trim_scale(float base);

/**
 * @brief Trim de hover por motor (u1..u4, em ‰), somado à base de cada motor e escalado por
 *        base/hover, para que na rampa e no pouso ele cresça e caia junto com a aceleração.
 *        Compensa o CG fora do centro dos motores (valor de ::LQG_HOVER_TRIM_PERMILLE).
 *        Aceita só valores finitos com módulo até ::LQG_CG_TRIM_MAX_PERMILLE; false se não.
 */
bool lqg_ctrl_set_cg_trim(lqg_ctrl_t *c, const float trim[4]);

/** @brief Troca os parâmetros; ignora e devolve false se inválidos. */
bool lqg_ctrl_set_params(lqg_ctrl_t *c, const lqg_params_t *p);

/** @brief Desarmado -> armado: zera a origem da guinada e começa a medir o chão. */
bool lqg_ctrl_arm(lqg_ctrl_t *c);

/**
 * @brief Armado -> começa a subir a aceleração até descolar.
 *
 * Recusa sem ::LQG_GROUND_SAMPLES leituras do chão ou sem BNO e distância válidos.
 */
bool lqg_ctrl_takeoff(lqg_ctrl_t *c);

/** @brief Em decolagem, subida ou hover: pousa. No chão ainda, só reduz os motores. */
bool lqg_ctrl_land(lqg_ctrl_t *c);

/** @brief Volta a desarmado sem passo (saída desarmada por outro motivo). */
void lqg_ctrl_abort(lqg_ctrl_t *c);

/** @brief Fase atual. */
lqg_state_t lqg_ctrl_state(const lqg_ctrl_t *c);

/**
 * @brief Ganho do integrador de altura em uso, em ‰/(m·s): o parâmetro, limitado
 *        a ::LQG_TRIM_STABILITY_FRACTION do limite de estabilidade com os
 *        multiplicadores de altura em uso (z e w; 1 com o Kalman gerado).
 */
float lqg_ctrl_trim_ki(const lqg_ctrl_t *c);

/**
 * @brief Novo alvo em voo (subida ou hover): posição local, altura e guinada.
 *
 * x, y em m no referencial local (origem e guinada do armamento; x frente,
 * y direita), altura em m para cima, guinada em rad (horário visto de cima
 * positivo). A referência anda até o alvo a @p speed m/s (XY), na velocidade de
 * subida/descida (altura) e a ::LQG_YAW_RATE_RAD_S (guinada). Recusa fora de
 * voo, números não finitos, |x|,|y| > ::LQG_XY_LIMIT_M, altura fora de
 * ::LQG_PARAM_HEIGHT_MIN_M..::LQG_PARAM_HEIGHT_MAX_M ou velocidade fora de 0..0,5 m/s
 * (0 = ::LQG_XY_SPEED_M_S).
 */
bool lqg_ctrl_goto(lqg_ctrl_t *c, float x, float y, float height_m, float psi, float speed);

/** @brief Em voo: o alvo vira a referência atual (para onde está agora). */
bool lqg_ctrl_hold(lqg_ctrl_t *c);

/** @brief Referência e estimativa atuais, para os comandos da Pi (ver ::lqg_out_t). */
typedef struct {
  float pos[3];  /**< x, y local (m) e altura (m para cima). */
  float vel[3];  /**< u, v local e subida (m/s, para cima). */
  float psi;     /**< Guinada estimada, rad. */
  float r;       /**< Taxa de guinada estimada, rad/s. */
  float ref[4];  /**< Referência x, y, altura, guinada. */
  float goal[4]; /**< Alvo x, y, altura, guinada. */
  bool ref_at_goal;
} lqg_nav_t;

/** @brief Navegação atual (estimativa, referência e alvo). */
void lqg_ctrl_nav(const lqg_ctrl_t *c, lqg_nav_t *nav);

/**
 * @brief Um passo de 10 ms. Chamar sempre, em qualquer fase, a cada ::LQG_TS_S.
 *
 * @param[in,out] c    Controlador.
 * @param[in]     in   Leituras dos sensores neste instante.
 * @param[out]    out  Comandos e telemetria.
 */
void lqg_ctrl_step(lqg_ctrl_t *c, const lqg_sensors_t *in, lqg_out_t *out);

#endif
