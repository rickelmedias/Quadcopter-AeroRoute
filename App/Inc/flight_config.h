/**
 * @file flight_config.h
 * @brief Chave de segurança do voo e valores padrão dos parâmetros.
 *
 * O LQG (control/lqg_ert_model.h) usa o modelo do TCC em eixos NED: x para a
 * frente, y para a direita, z para baixo; rolagem positiva abaixa o lado
 * direito, arfagem positiva levanta o nariz, guinada positiva gira no sentido
 * horário visto de cima. Os sinais abaixo levam as leituras dos sensores para
 * essa convenção. CONFIRA CADA UM NA BANCADA antes de voar (roteiro em
 * docs/controle_lqg.md).
 *
 * Os valores abaixo são só o ponto de partida: todos podem ser alterados em
 * tempo de execução (control/ctrl_params.h), sem regravar o firmware.
 */
#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

/**
 * @brief 0 = só ensaio a seco: o LQG roda e a telemetria sai, mas os motores
 *        nunca recebem o comando do controle, qualquer que seja FLT_DRY_RUN.
 *        1 = com FLT_DRY_RUN = 0, os motores seguem o LQG.
 *
 * Mudar para 1 é uma decisão consciente, com a curva do motor medida, os
 * sinais conferidos e o drone preso.
 */
#define LQG_FLIGHT_ENABLED 0

/* ---- Motores ------------------------------------------------------------ */

/**
 * Rotação de hover ω0, em rad/s. Derivada do B exportado pelo Simulink
 * (b_z = g / 2ω0, ver control/lqg_ert_model.h). Confira com o MATLAB.
 */
#define DEFAULT_CTL_W0 702.0F

/**
 * Curva rotação -> aceleração DShot, linear entre os dois pontos.
 * VALORES PROVISÓRIOS: escolhidos para que ω0 caia em ~40% de aceleração.
 * Substitua pelos pontos medidos no dinamômetro (ou ajuste MOT_W_AT_100 em
 * voo até o hover ficar parado na altura pedida).
 */
#define DEFAULT_MOT_W_AT_0    0.0F
/** Rotação em 100% de aceleração, em rad/s (ver DEFAULT_MOT_W_AT_0). */
#define DEFAULT_MOT_W_AT_100  1755.0F

/** Aceleração mínima com o controle ligado (fração 0..1). */
#define DEFAULT_MOT_MIN   0.03F
/** Aceleração máxima com o controle ligado (fração 0..1). */
#define DEFAULT_MOT_MAX   1.0F
/** Aceleração com o drone armado e parado no chão (fração 0..1). */
#define DEFAULT_MOT_IDLE  0.05F

/* ---- Sinais dos sensores para NED ---------------------------------------
   Padrão para o BNO montado com x para a frente e z para cima (FLU):
   rolagem igual, arfagem e guinada invertidas. */
/** Sinal da rolagem do BNO08x. */
#define DEFAULT_SGN_ROLL   (+1.0F)
/** Sinal da arfagem do BNO08x. */
#define DEFAULT_SGN_PITCH  (-1.0F)
/** Sinal da guinada do BNO08x. */
#define DEFAULT_SGN_YAW    (-1.0F)
/** Sinal do giro p. */
#define DEFAULT_SGN_P      (+1.0F)
/** Sinal do giro q. */
#define DEFAULT_SGN_Q      (-1.0F)
/** Sinal do giro r. */
#define DEFAULT_SGN_R      (-1.0F)
/** Sinal da velocidade x do fluxo (já girada pela guinada) para NED. */
#define DEFAULT_SGN_VX     (+1.0F)
/** Sinal da velocidade y do fluxo para NED. */
#define DEFAULT_SGN_VY     (+1.0F)

/**
 * Escala do fluxo óptico do MTF-01, medida contra o giroscópio na bancada
 * (0,944 em X e 0,942 em Y, ver docs/acompanhamento_tecnico.md).
 */
#define DEFAULT_FLOW_SCALE_X 0.944F
/** Escala do fluxo em Y (ver DEFAULT_FLOW_SCALE_X). */
#define DEFAULT_FLOW_SCALE_Y 0.942F
/**
 * Compensação da rotação no fluxo: a rotação do corpo também move a imagem.
 * Nesta placa o pareamento é no mesmo eixo, com X invertido e Y não (ensaios
 * isolados por eixo, ver docs/acompanhamento_tecnico.md).
 */
#define DEFAULT_FLOW_GYRO_X  (+1.0F)
/** Compensação da rotação no fluxo Y (ver DEFAULT_FLOW_GYRO_X). */
#define DEFAULT_FLOW_GYRO_Y  (-1.0F)

/* ---- Voo ---------------------------------------------------------------- */

/** Altura padrão do hover, em m. */
#define DEFAULT_HOVER_HEIGHT_M 0.5F
/** Rampa da referência vertical, em m/s (0 = degrau, como na simulação). */
#define DEFAULT_Z_RATE_M_S     0.3F
/** Rampa padrão da referência horizontal, em m/s. */
#define DEFAULT_XY_RATE_M_S    0.3F
/** Rampa da referência de guinada, em rad/s. */
#define DEFAULT_YAW_RATE_RAD_S 0.5F
/** Inclinação acima da qual os motores são cortados, em rad (45 graus). */
#define DEFAULT_TILT_MAX_RAD   0.785F
/** Altura acima do chão em que o pouso termina, em m. */
#define DEFAULT_LAND_HEIGHT_M  0.08F
/** Começa em ensaio a seco: os motores só giram depois de FLT_DRY_RUN = 0. */
#define DEFAULT_DRY_RUN        1.0F

/* ---- Telemetria --------------------------------------------------------- */

/** Taxa do DRONE_CTRL_DEBUG para a Pi, em Hz (0 = desligado até pedirem). */
#define DEFAULT_DEBUG_RATE_HZ  0.0F
/** Taxa da linha EST no log USB, em Hz. */
#define DEFAULT_LOG_EST_HZ     10.0F

#endif
