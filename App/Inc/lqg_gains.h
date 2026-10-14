/**
 * @file lqg_gains.h
 * @brief Modelo e ganhos do LQG. GERADO por tools/lqg_design.py; não edite à mão.
 *
 * Ts = 2.500 ms, w0 = 146.0244 rad/s.
 * Estados (desvios do hover, eixos NED: x frente, y direita, z para baixo):
 *   0 x, 1 y, 2 z, 3 phi, 4 theta, 5 psi, 6 vx, 7 vy, 8 vz, 9 p, 10 q, 11 r
 * Entradas: rotação de cada motor, em rad/s (motor 1..4, ver docs/lqg_firmware.md).
 * Saídas: 0 z, 1 vx, 2 vy, 3 phi, 4 theta, 5 psi, 6 p, 7 q, 8 r.
 */
#ifndef LQG_GAINS_H
#define LQG_GAINS_H
#include <stdbool.h>

#define LQG_NX 12U
#define LQG_NU 4U
#define LQG_NY 9U
#define LQG_TS_S 2.500000000e-03F

enum { LQG_X_X, LQG_X_Y, LQG_X_Z, LQG_X_PHI, LQG_X_THETA, LQG_X_PSI,
       LQG_X_VX, LQG_X_VY, LQG_X_VZ, LQG_X_P, LQG_X_Q, LQG_X_R };
enum { LQG_Y_Z, LQG_Y_VX, LQG_Y_VY, LQG_Y_PHI, LQG_Y_THETA, LQG_Y_PSI,
       LQG_Y_P, LQG_Y_Q, LQG_Y_R };

extern const float LQG_AD[LQG_NX][LQG_NX];
extern const float LQG_BD[LQG_NX][LQG_NU];
extern const float LQG_C[LQG_NY][LQG_NX];
extern const float LQG_K[LQG_NU][LQG_NX];
extern const float LQG_QW[LQG_NX];
extern const float LQG_RV[LQG_NY];
extern const float LQG_GATE_SIGMA[LQG_NY];
extern const float LQG_P0[LQG_NX];
extern const float LQG_P_MAX[LQG_NX];
extern const float LQG_U0[LQG_NU];
extern const float LQG_U_MIN[LQG_NU];
extern const float LQG_U_MAX[LQG_NU];
extern const bool LQG_ANGLE_STATE[LQG_NX];
#endif
