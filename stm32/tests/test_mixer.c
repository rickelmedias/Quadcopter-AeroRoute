/**
 * @file test_mixer.c
 * @brief Mapa entre as entradas do modelo e os motores físicos.
 *
 * Confere, a partir dos sinais da matriz B exportada pelo Simulink, que cada
 * efeito do modelo (arfagem, rolagem) cai nos motores certos do drone.
 */
#include "control/mixer.h"
#include "control/lqg_ert_model.h"

#include <assert.h>
#include <stdio.h>

/** Frente física: M2 e M4. Direita física: M1 e M2 (índices 0-based). */
static int is_front(unsigned motor) {
  return motor == 1U || motor == 3U;
}
static int is_right(unsigned motor) {
  return motor == 0U || motor == 1U;
}

/** @brief Todas as entradas vão para motores diferentes, e M1..M4 são todos usados. */
static void test_is_a_permutation(void) {
  unsigned seen = 0U;
  for (unsigned i = 0U; i < MIXER_COUNT; ++i)
    seen |= 1U << mixer_motor_of_input(i);
  assert(seen == 0x0FU);
  assert(mixer_motor_of_input(MIXER_COUNT) == 0U);
}

/** @brief Cada valor sai no motor esperado. */
static void test_reorder(void) {
  const float model[MIXER_COUNT] = {1.0F, 2.0F, 3.0F, 4.0F};
  float motor[MIXER_COUNT];
  mixer_model_to_motors(model, motor);
  assert(motor[3] == 1.0F && motor[1] == 2.0F && motor[0] == 3.0F && motor[2] == 4.0F);
}

/**
 * @brief O sinal da matriz B de θ (arfagem, nariz para cima) é positivo na
 *        frente física, e o de φ (lado direito para baixo) é positivo à esquerda.
 */
static void test_b_matrix_matches_layout(void) {
  for (unsigned input = 0U; input < MIXER_COUNT; ++input) {
    const unsigned motor = mixer_motor_of_input(input);
    const double theta = LQG_ERT_B[LQG_X_THETA][input];
    const double phi = LQG_ERT_B[LQG_X_PHI][input];
    assert((theta > 0.0) == (is_front(motor) != 0));
    assert((phi > 0.0) == (is_right(motor) == 0));
  }
}

/** @brief Hélices de mesma cor (M2 e M3; M1 e M4) têm o mesmo sinal de ψ na matriz B. */
static void test_yaw_pairs_follow_propeller_colour(void) {
  double psi_of_motor[MIXER_COUNT];
  for (unsigned input = 0U; input < MIXER_COUNT; ++input) {
    psi_of_motor[mixer_motor_of_input(input)] = LQG_ERT_B[LQG_X_PSI][input];
  }
  assert((psi_of_motor[1] > 0.0) == (psi_of_motor[2] > 0.0)); /* pretas */
  assert((psi_of_motor[0] > 0.0) == (psi_of_motor[3] > 0.0)); /* amarelas */
  assert((psi_of_motor[1] > 0.0) != (psi_of_motor[0] > 0.0));
}

int main(void) {
  test_is_a_permutation();
  test_reorder();
  test_b_matrix_matches_layout();
  test_yaw_pairs_follow_propeller_colour();
  printf("test_mixer: ok\n");
  return 0;
}
