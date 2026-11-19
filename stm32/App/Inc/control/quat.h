/**
 * @file quat.h
 * @brief Matemática de quatérnios do EKF e do controle (float, sem HAL).
 *
 * Quatérnio unitário q = [w x y z] que leva vetores do corpo (FRD) para o
 * referencial local NED (origem e guinada do armamento): v_local = R(q) v_corpo.
 * Ângulos em rad, ZYX (guinada, arfagem, rolagem), a mesma convenção de
 * φ, θ, ψ do modelo do Simulink.
 */
#ifndef CONTROL_QUAT_H
#define CONTROL_QUAT_H

/** @brief Quatérnio identidade. */
void quat_identity(float q[4]);

/** @brief q = a ⊗ b (aplica b e depois a, no sentido de rotações encadeadas). */
void quat_mul(const float a[4], const float b[4], float q[4]);

/** @brief Conjugado (inverso, para quatérnio unitário). */
void quat_conj(const float q[4], float out[4]);

/** @brief Normaliza; um quatérnio nulo ou não finito vira a identidade. */
void quat_normalize(float q[4]);

/** @brief Quatérnio da rotação ZYX de @p phi (x), @p theta (y), @p psi (z), em rad. */
void quat_from_euler(float phi, float theta, float psi, float q[4]);

/** @brief φ, θ e ψ (rad) de @p q; a arfagem satura em ±π/2. Qualquer ponteiro de saída pode ser NULL. */
void quat_to_euler(const float q[4], float *phi, float *theta, float *psi);

/** @brief Guinada de @p q (rad, ZYX), sem calcular o resto. */
float quat_yaw(const float q[4]);

/** @brief Quatérnio de uma rotação em torno de z (guinada pura), em rad. */
void quat_from_yaw(float psi, float q[4]);

/** @brief Matriz de rotação corpo -> local (linha a linha). */
void quat_to_dcm(const float q[4], float r[3][3]);

/** @brief v_local = R(q) v_corpo. */
void quat_rotate(const float q[4], const float v[3], float out[3]);

/** @brief v_corpo = R(q)ᵀ v_local. */
void quat_rotate_inv(const float q[4], const float v[3], float out[3]);

/**
 * @brief Quatérnio da rotação (vetor de rotação) @p rv, em rad, no sentido do corpo.
 *        Exato para qualquer módulo; série de Taylor perto de zero.
 */
void quat_from_rotvec(const float rv[3], float q[4]);

/**
 * @brief Vetor de rotação (rad) de @p q, no ramo de menor ângulo (|rv| <= π).
 */
void quat_to_rotvec(const float q[4], float rv[3]);

#endif
