/**
 * @file mixer.c
 * @brief Entradas do modelo -> motores físicos (contrato em control/mixer.h).
 */
#include "control/mixer.h"

/** Motor físico (índice 0 = M1) de cada entrada do modelo: u1 -> M4, u2 -> M2, u3 -> M1, u4 -> M3. */
static const unsigned MOTOR_OF_INPUT[MIXER_COUNT] = {3U, 1U, 0U, 2U};

unsigned mixer_motor_of_input(unsigned input) {
  return input < MIXER_COUNT ? MOTOR_OF_INPUT[input] : 0U;
}

void mixer_model_to_motors(const float model[MIXER_COUNT], float motor[MIXER_COUNT]) {
  for (unsigned i = 0U; i < MIXER_COUNT; ++i) {
    motor[MOTOR_OF_INPUT[i]] = model[i];
  }
}
