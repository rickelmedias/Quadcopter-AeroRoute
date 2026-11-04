/**
 * @file ctrl_params.h
 * @brief Parâmetros do voo ajustáveis em tempo de execução.
 *
 * São os números que dependem do drone físico e da bancada (rotação de
 * hover, curva do motor, sinais dos sensores, rampas, limites e taxas de
 * telemetria). Eles podem ser lidos e alterados:
 *
 * - pela Raspberry Pi, com o protocolo de parâmetros do MAVLink
 *   (PARAM_REQUEST_LIST, PARAM_REQUEST_READ, PARAM_SET, PARAM_VALUE);
 * - pelo console USB: `param list`, `param get NOME`, `param set NOME VALOR`.
 *
 * Os ganhos do LQG não são parâmetros: eles são matrizes recalculadas pela Pi
 * e trocadas de uma vez (control/ctrl_gains.h).
 *
 * Os valores ficam só na RAM e voltam ao padrão a cada boot; a ferramenta da
 * Pi guarda perfis de ajuste e os reaplica (tools/pi/README.md).
 *
 * Os padrões estão em flight_config.h.
 */
#ifndef CONTROL_CTRL_PARAMS_H
#define CONTROL_CTRL_PARAMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Maior nome de parâmetro (limite do MAVLink, sem o terminador). */
#define CTRL_PARAM_NAME_MAX 16U

/**
 * @brief Valores atuais de todos os parâmetros.
 *
 * O controle tira uma cópia por passo com ::ctrl_params_snapshot().
 */
typedef struct {
  /** CTL_W0: rotação de hover ω0, em rad/s. A rotação comandada é ω0 + du. */
  float w0;
  /** MOT_W_AT_0: rotação correspondente a 0% de aceleração DShot, em rad/s. */
  float mot_w_at_0;
  /** MOT_W_AT_100: rotação correspondente a 100% de aceleração, em rad/s. */
  float mot_w_at_100;
  /** MOT_MIN: menor aceleração com o controle ligado, em fração (0..1). */
  float mot_min;
  /** MOT_MAX: maior aceleração com o controle ligado, em fração (0..1). */
  float mot_max;
  /** MOT_IDLE: aceleração com o drone armado no chão, em fração (0..1). */
  float mot_idle;
  /** SNS_SGN_ROLL: sinal de φ, que vem do pitch do BNO08x (+1/-1). */
  float sgn_roll;
  /** SNS_SGN_PITCH: sinal de θ, que vem do roll do BNO08x. */
  float sgn_pitch;
  /** SNS_SGN_YAW: sinal da guinada. */
  float sgn_yaw;
  /** SNS_SGN_P: sinal de p, que vem do giro y do BNO08x. */
  float sgn_p;
  /** SNS_SGN_Q: sinal de q, que vem do giro x do BNO08x. */
  float sgn_q;
  /** SNS_SGN_R: sinal do giro r. */
  float sgn_r;
  /** SNS_SGN_VX: sinal da velocidade x do fluxo. */
  float sgn_vx;
  /** SNS_SGN_VY: sinal da velocidade y do fluxo. */
  float sgn_vy;
  /** FLOW_SCALE_X: escala do fluxo óptico em X. */
  float flow_scale_x;
  /** FLOW_SCALE_Y: escala do fluxo óptico em Y. */
  float flow_scale_y;
  /** FLOW_GYRO_X: ganho (com sinal) da compensação de rotação do fluxo X; 0 desliga. */
  float flow_gyro_x;
  /** FLOW_GYRO_Y: idem para Y. */
  float flow_gyro_y;
  /** FLT_HOVER_H: altura padrão do comando hover, em m. */
  float hover_height;
  /** FLT_Z_RATE: velocidade da rampa de referência vertical, em m/s (0 = degrau). */
  float z_rate;
  /** FLT_XY_RATE: velocidade padrão da rampa horizontal, em m/s (0 = degrau). */
  float xy_rate;
  /** FLT_YAW_RATE: velocidade da rampa de guinada, em rad/s (0 = degrau). */
  float yaw_rate;
  /** FLT_TILT_MAX: inclinação que corta os motores, em rad. */
  float tilt_max;
  /** FLT_LAND_H: altura acima do chão em que o pouso termina, em m. */
  float land_height;
  /** FLT_DRY_RUN: 1 = o LQG roda, mas os motores ficam parados. */
  float dry_run;
  /** DBG_RATE_HZ: taxa do DRONE_CTRL_DEBUG para a Pi, em Hz (0 desliga). */
  float debug_rate_hz;
  /** LOG_EST_HZ: taxa da linha EST no log USB, em Hz (0 desliga). */
  float log_est_hz;
  /** SNS_ZERO_ROLL: roll cru do BNO08x com o drone nivelado, em graus. */
  float zero_roll_deg;
  /** SNS_ZERO_PITCH: pitch cru do BNO08x com o drone nivelado, em graus. */
  float zero_pitch_deg;
} ctrl_params_t;

/** @brief Regras de um parâmetro. */
typedef enum {
  /** Pode mudar a qualquer momento, inclusive em voo. */
  CTRL_PARAM_LIVE = 0,
  /** Só muda com o drone desarmado. */
  CTRL_PARAM_DISARMED_ONLY = 1U << 0,
  /** O valor é arredondado para +1 ou -1. */
  CTRL_PARAM_SIGN = 1U << 1,
  /** O valor é arredondado para 0 ou 1. */
  CTRL_PARAM_BOOL = 1U << 2
} ctrl_param_flags_t;

/** @brief Descrição de um parâmetro. */
typedef struct {
  /** Nome, até ::CTRL_PARAM_NAME_MAX caracteres. */
  const char *name;
  /** Posição do campo em ::ctrl_params_t. */
  size_t offset;
  /** Menor valor aceito. */
  float min;
  /** Maior valor aceito. */
  float max;
  /** Valor padrão. */
  float def;
  /** Combinação de ::ctrl_param_flags_t. */
  uint8_t flags;
  /** Unidade e significado, para o console e a documentação. */
  const char *help;
} ctrl_param_desc_t;

/** @brief Resultado de ::ctrl_params_set(). */
typedef enum {
  CTRL_PARAM_OK = 0,        /**< Valor aplicado. */
  CTRL_PARAM_UNKNOWN,       /**< Índice ou nome inexistente. */
  CTRL_PARAM_OUT_OF_RANGE,  /**< Fora de [min, max] ou não finito. */
  CTRL_PARAM_LOCKED         /**< Só pode mudar desarmado. */
} ctrl_param_status_t;

/** @brief Põe todos os parâmetros no valor padrão. */
void ctrl_params_init(void);

/** @brief Quantidade de parâmetros. */
size_t ctrl_params_count(void);

/**
 * @brief Descrição do parâmetro de um índice.
 * @return NULL se o índice não existe.
 */
const ctrl_param_desc_t *ctrl_params_desc(size_t index);

/**
 * @brief Índice de um parâmetro pelo nome (sem diferenciar maiúsculas).
 *
 * @param[in] name  Nome; aceita nomes sem terminador de até
 *                  ::CTRL_PARAM_NAME_MAX caracteres, como chegam no MAVLink.
 * @param[in] len   Tamanho máximo a comparar.
 * @return O índice, ou -1.
 */
int ctrl_params_find(const char *name, size_t len);

/**
 * @brief Valor atual de um parâmetro.
 * @return O valor; NAN se o índice não existe.
 */
float ctrl_params_get(size_t index);

/**
 * @brief Altera um parâmetro.
 *
 * @param[in] index   Índice.
 * @param[in] value   Novo valor (arredondado para sinal ou booleano, se for o caso).
 * @param[in] armed   Se o drone está armado agora (para ::CTRL_PARAM_DISARMED_ONLY).
 * @return ::CTRL_PARAM_OK se aplicado.
 */
ctrl_param_status_t ctrl_params_set(size_t index, float value, bool armed);

/**
 * @brief Copia todos os valores de uma vez, de forma segura entre tasks.
 * @param[out] out  Destino.
 */
void ctrl_params_snapshot(ctrl_params_t *out);

/** @brief Texto curto de um ::ctrl_param_status_t, para o console e o log. */
const char *ctrl_params_status_text(ctrl_param_status_t status);

#endif
