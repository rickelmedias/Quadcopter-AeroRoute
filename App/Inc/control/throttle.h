/**
 * @file throttle.h
 * @brief Pós-processamento da aceleração dos motores: rampas e dessaturação.
 *
 * - **Rampa de partida:** depois da marcha lenta, a aceleração calculada pelo
 *   controle entra aos poucos (::throttle_ramp_up), em vez de saltar de 5%
 *   para o hover de uma vez. Com um lado mais leve, o degrau fazia esse lado
 *   pular e passar do corte de inclinação.
 * - **Rampa de pouso:** ao encostar no chão a aceleração desce até a marcha
 *   lenta (::throttle_ramp_down) antes de desarmar.
 * - **Dessaturação:** se um motor pede mais que o máximo (ou menos que o
 *   mínimo), os quatro são deslocados juntos (::throttle_desaturate). A
 *   diferença entre eles, que é o que segura o nível, é preservada; perde-se
 *   empuxo total. Antes cada motor era cortado sozinho, e o lado pesado ficava
 *   sem a diferença de que precisava.
 *
 * Código puro, testado em tests/test_throttle.c.
 */
#ifndef CONTROL_THROTTLE_H
#define CONTROL_THROTTLE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Fração da rampa de descida: 1 no começo, 0 ao fim.
 *
 * @param[in] elapsed_ms  Tempo desde o começo.
 * @param[in] total_ms    Duração (0 = imediato).
 * @return Valor em [0, 1].
 */
float throttle_ramp_down(uint32_t elapsed_ms, uint32_t total_ms);

/**
 * @brief Fração da rampa de subida: 0 no começo, 1 ao fim.
 *
 * @param[in] elapsed_ms  Tempo desde o começo.
 * @param[in] total_ms    Duração (0 = imediato).
 * @return Valor em [0, 1].
 */
float throttle_ramp_up(uint32_t elapsed_ms, uint32_t total_ms);

/**
 * @brief Mistura a marcha lenta e a aceleração calculada pela fração da rampa.
 *
 * Nunca devolve mais que @p throttle nem menos que @p idle (se @p throttle já
 * for menor que @p idle, não muda).
 *
 * @param[in] throttle  Aceleração calculada pelo controle (fração 0..1).
 * @param[in] idle      Marcha lenta (fração 0..1).
 * @param[in] factor    Fração da rampa, 0..1.
 */
float throttle_blend(float throttle, float idle, float factor);

/**
 * @brief Desloca os motores juntos para caberem em [min, max].
 *
 * Valores não finitos viram @p min. Se a diferença entre o maior e o menor
 * não couber na faixa, o maior vai a @p max e os de baixo são limitados a @p min.
 *
 * @param[in,out] throttle  Aceleração de cada motor (fração 0..1).
 * @param[in]     count     Quantidade de motores.
 * @param[in]     min       Menor aceleração permitida.
 * @param[in]     max       Maior aceleração permitida.
 * @return true se algum valor foi mudado (saturação).
 */
bool throttle_desaturate(float *throttle, unsigned count, float min, float max);

#endif
