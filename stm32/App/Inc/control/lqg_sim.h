/**
 * @file lqg_sim.h
 * @brief Planta simulada para o modo FLT_SIM (sensores falsos, motores parados).
 *
 * Código puro, testado no PC. A planta é a linear do modelo gerado
 * (x+ = A x + B Δω, matrizes de control/lqg_model), com a gravidade inclinada
 * girada pela guinada e o chão (altura não fica negativa; apoiado, não gira nem
 * translada). Os sensores saem pelo mapa inverso de control/lqg_config.h, sem
 * ruído. Serve para exercitar a sequência, o DroneLink e a Raspberry Pi na
 * placa real com os motores em zero; não diz nada sobre o voo real.
 */
#ifndef CONTROL_LQG_SIM_H
#define CONTROL_LQG_SIM_H

#include "control/lqg_meas.h"

#include <stdbool.h>
#include <stdint.h>

/** Distância do MTF-01 ao chão com o drone apoiado, em m. */
#define LQG_SIM_GROUND_RANGE_M 0.07F

/** @brief Estado da planta. */
typedef struct {
  /** [x y z φ θ ψ u v w p q r], NED/FRD, SI. */
  double x[12];
  /** Aceleração em que a planta paira, em ‰. */
  float hover_permille;
  /** Já saiu do chão uma vez. */
  bool lifted;
  /** Quadros do MTF-01 simulados. */
  uint32_t frames;
  /** Aceleração cinemática local (NED) do último passo, em m/s². */
  double acc[3];
} lqg_sim_t;

/** @brief Drone parado no chão, nivelado, guinada zero; paira em @p hover_permille ‰. */
void lqg_sim_init(lqg_sim_t *s, float hover_permille);

/** @brief Leituras que os drivers dariam no estado atual (conta um quadro do MTF). */
void lqg_sim_sensors(lqg_sim_t *s, lqg_sensors_t *in);

/** @brief Avança 10 ms com a aceleração dos motores físicos @p permille (M1..M4, ‰). */
void lqg_sim_step(lqg_sim_t *s, const float permille[4]);

#endif
