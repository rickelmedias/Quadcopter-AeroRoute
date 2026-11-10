/**
 * @file mixer.h
 * @brief Distribuição das quatro entradas do modelo nos quatro motores físicos.
 *
 * O modelo do TCC numera as entradas u1..u4 pela matriz B
 * (control/lqg_ert_model.h): u1 e u2 na frente, u3 e u4 atrás, u1 e u4 à
 * esquerda, u2 e u3 à direita, e u2 e u4 aumentam ψ. A numeração física dos
 * motores no drone é outra (vista de cima, frente do drone para cima):
 *
 * @verbatim
 *        frente
 *    M4 (am.)   M2 (pr.)
 *    esquerda      direita
 *    M3 (pr.)   M1 (am.)
 *         trás
 * @endverbatim
 *
 * M1 e M4 têm hélice amarela (invertida), M2 e M3 têm hélice preta. Frente =
 * M2 e M4, trás = M1 e M3, direita = M1 e M2, esquerda = M3 e M4. Cada
 * entrada do modelo vai para o motor que ocupa o mesmo canto:
 *
 * | Entrada | Canto do modelo     | Motor físico |
 * |---------|---------------------|--------------|
 * | u1      | frente / esquerda   | M4           |
 * | u2      | frente / direita    | M2           |
 * | u3      | trás / direita      | M1           |
 * | u4      | trás / esquerda     | M3           |
 *
 * Os pares diagonais (u2, u4) e (u1, u3) caem em (M2, M3) e (M4, M1), ou seja,
 * em hélices de mesma cor, como o sinal de ψ da matriz B exige. Esse sinal
 * supõe que as hélices pretas (M2 e M3) giram no sentido anti-horário visto de
 * cima: confira na bancada (docs/controle_lqg.md, seção 9).
 *
 * Código puro (sem HAL nem FreeRTOS), testado em tests/test_mixer.c.
 */
#ifndef CONTROL_MIXER_H
#define CONTROL_MIXER_H

/** Quantidade de entradas do modelo e de motores. */
#define MIXER_COUNT 4U

/**
 * @brief Reordena um valor por entrada do modelo em um valor por motor.
 *
 * @param[in]  model  Valor de u1..u4 (índice 0 = u1).
 * @param[out] motor  Valor de M1..M4 (índice 0 = M1). Não pode ser o mesmo
 *                    vetor de @p model.
 */
void mixer_model_to_motors(const float model[MIXER_COUNT], float motor[MIXER_COUNT]);

/**
 * @brief Motor físico (0..3, índice 0 = M1) que recebe a entrada @p input do modelo.
 *
 * @param[in] input  Entrada do modelo, 0..3 (u1..u4).
 * @return Índice do motor, ou 0 se @p input for inválida.
 */
unsigned mixer_motor_of_input(unsigned input);

#endif
