/**
 * @file sim_plant.h
 * @brief Drone simulado para testar a comunicação Pi ↔ STM32 na bancada.
 *
 * Com FLT_SIM = 1 os motores não giram e o voo usa este drone no lugar dos
 * sensores: o mesmo LQG do TCC controla o modelo linear do TCC
 * (x[k+1] = A x[k] + B du[k], tabelas de control/lqg_ert_model.h) e as
 * medidas saem de y = C x. A Pi, a UART, o DroneLink e a máquina de voo são
 * os de verdade; só o drone é de mentira. Assim decolar, estabilizar, girar e
 * andar levam tempos realistas e as respostas do STM32 podem ser conferidas
 * sem hélice.
 *
 * O chão é um limite: z (NED) não passa de @c ground_z, e a velocidade
 * vertical para baixo é zerada ali. Apoiado no chão, o drone também fica
 * nivelado e parado (φ, θ, p, q, r, vx e vy zerados), como sobre as pernas;
 * x, y e ψ ficam onde estavam.
 *
 * Código puro (sem HAL), testado em tests/test_sim_plant.c.
 */
#ifndef CONTROL_SIM_PLANT_H
#define CONTROL_SIM_PLANT_H

#include "control/lqg_ert.h"

/** Altura do sensor ao chão no drone simulado (z NED do chão), em m. */
#define SIM_PLANT_GROUND_Z (-0.08F)

/** @brief Estado do drone simulado. */
typedef struct {
  lqg_real_t x[LQG_NX]; /**< Estado no referencial do modelo (x y z φ θ ψ vx vy vz p q r). */
  float ground_z;       /**< z (NED) do chão, em m. */
} sim_plant_t;

/**
 * @brief Põe o drone parado no chão, em x = y = 0, com a guinada @p psi.
 * @param[out] s         Estado.
 * @param[in]  ground_z  z (NED) do chão, em m (normalmente ::SIM_PLANT_GROUND_Z).
 * @param[in]  psi       Guinada inicial, em rad.
 */
void sim_plant_reset(sim_plant_t *s, float ground_z, float psi);

/**
 * @brief Avança um passo (Ts) com o desvio de rotação aplicado nos motores.
 * @param[in,out] s   Estado.
 * @param[in]     du  Desvio de cada entrada do modelo em relação ao hover, em rad/s.
 */
void sim_plant_step(sim_plant_t *s, const lqg_real_t du[LQG_NU]);

/**
 * @brief Medidas do drone simulado, na ordem do vetor y do LQG.
 * @param[in]  s  Estado.
 * @param[out] y  Medidas (z φ θ ψ vx vy p q r).
 */
void sim_plant_measure(const sim_plant_t *s, lqg_real_t y[LQG_NY]);

#endif /* CONTROL_SIM_PLANT_H */
