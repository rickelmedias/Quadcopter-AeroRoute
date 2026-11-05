/**
 * @file spindown.h
 * @brief Descida suave da aceleração no fim do pouso.
 *
 * Ao encostar no chão, o voo não corta os motores de uma vez: a aceleração
 * calculada pelo controle desce em rampa linear até a marcha lenta e só então
 * desarma. Código puro, testado em tests/test_spindown.c.
 */
#ifndef CONTROL_SPINDOWN_H
#define CONTROL_SPINDOWN_H

#include <stdint.h>

/**
 * @brief Fração da aceleração que ainda vale: 1 no começo, 0 no fim da rampa.
 *
 * @param[in] elapsed_ms  Tempo desde o começo da descida.
 * @param[in] total_ms    Duração da rampa (0 = corta de uma vez).
 * @return Valor em [0, 1].
 */
float spindown_factor(uint32_t elapsed_ms, uint32_t total_ms);

/**
 * @brief Aplica a fração entre a marcha lenta e a aceleração calculada.
 *
 * Nunca devolve mais que @p throttle nem menos que @p idle (se @p throttle já
 * for menor que @p idle, não muda).
 *
 * @param[in] throttle  Aceleração calculada pelo controle (fração 0..1).
 * @param[in] idle      Marcha lenta (fração 0..1).
 * @param[in] factor    Saída de ::spindown_factor().
 */
float spindown_throttle(float throttle, float idle, float factor);

#endif
