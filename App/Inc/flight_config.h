/**
 * @file flight_config.h
 * @brief Parâmetros do voo que dependem do drone físico e precisam ser medidos.
 *
 * O LQG (lqg_gains.h) usa o modelo do TCC em eixos NED: x para a frente,
 * y para a direita, z para baixo; rolagem positiva abaixa o lado direito,
 * arfagem positiva levanta o nariz, guinada positiva gira no sentido horário
 * visto de cima. Os sinais abaixo levam as leituras dos sensores para essa
 * convenção. CONFIRA CADA UM NA BANCADA com o comando "dry" antes de voar
 * (roteiro em docs/lqg_firmware.md).
 */
#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

/**
 * @brief 0 = só ensaio a seco: o LQG roda e registra os comandos, mas os
 *        motores ficam parados. 1 = o comando "fly" liga os motores.
 *
 * Fica em 0 até a curva rotação x aceleração e os sinais abaixo serem
 * conferidos. Mudar para 1 é uma decisão consciente, com o drone preso.
 */
#define LQG_FLIGHT_ENABLED 0

/* Sinais BNO08x -> NED. Padrão para o BNO montado com x para a frente e z
   para cima (FLU): rolagem igual, arfagem e guinada invertidas. */
#define SIGN_ROLL   (+1.0F)
#define SIGN_PITCH  (-1.0F)
#define SIGN_YAW    (-1.0F)
#define SIGN_P      (+1.0F)
#define SIGN_Q      (-1.0F)
#define SIGN_R      (-1.0F)

/* Sinais das velocidades do fluxo (displacement, caminho FUSED) -> NED. */
#define SIGN_VX     (+1.0F)
#define SIGN_VY     (+1.0F)

/**
 * @brief Curva rotação -> DShot, linear entre os dois pontos abaixo.
 *
 * VALORES PROVISÓRIOS: escolhidos só para que a rotação de hover do modelo
 * (w0 = 146 rad/s com o Kt do TCC) caia em ~40% de aceleração. Substitua
 * pelos pontos medidos no dinamômetro (rotação em 0,1% e em 100%).
 */
#define MOTOR_OMEGA_AT_MIN_RAD_S  0.0F
#define MOTOR_OMEGA_AT_MAX_RAD_S  365.0F

/** Aceleração mínima durante o voo, em décimos de %, para nenhum motor parar. */
#define FLY_MIN_PERMILLE 30U

/** Inclinação acima da qual os motores são cortados, em rad (45 graus). */
#define FAILSAFE_TILT_RAD 0.785F

/** Altura máxima aceita em "ref z", em m. */
#define REF_Z_MAX_M 1.5F
#endif
