/**
 * @file dshot.h
 * @brief Saída DShot600 para os quatro ESCs no TIM4 (PB6..PB9) com DMA burst.
 *
 * O TIM4 conta a 96 MHz com período de 160 contagens, ou seja 600 kHz: um bit
 * DShot600 por período (1,67 us). Cada bit vira um pulso de 37,5% (bit 0) ou
 * 75% (bit 1). A cada evento de atualização o DMA1 Stream6 grava os quatro
 * CCR de uma vez (DMA burst pelo registrador DMAR), então os quatro motores
 * recebem o quadro ao mesmo tempo. Um quadro tem 16 bits + 2 períodos em zero
 * e leva ~30 us; a CPU só prepara o buffer.
 *
 * Valores de 11 bits: 0 = motor parado (também usado para armar o ESC),
 * 1..47 = comandos especiais (beep, sentido etc.), 48..2047 = aceleração.
 */
#ifndef DSHOT_H
#define DSHOT_H
#include <stdbool.h>
#include <stdint.h>

/** Menor valor de aceleração DShot. */
#define DSHOT_THROTTLE_MIN 48U
/** Maior valor de aceleração DShot. */
#define DSHOT_THROTTLE_MAX 2047U
/** Comando DShot de beep 1 (só com o motor parado). */
#define DSHOT_CMD_BEEP1 1U
/** Comando DShot: sentido de giro normal (só com o motor parado; precisa ser salvo). */
#define DSHOT_CMD_SPIN_DIRECTION_1 7U
/** Comando DShot: sentido de giro invertido (só com o motor parado; precisa ser salvo). */
#define DSHOT_CMD_SPIN_DIRECTION_2 8U
/** Comando DShot: grava na memória do ESC o que foi mudado (sentido). */
#define DSHOT_CMD_SAVE_SETTINGS 12U

/**
 * @brief Liga os quatro canais PWM do TIM4 com saída em zero e prepara o DMA.
 *
 * Chamar depois de MX_TIM4_Init() e MX_DMA_Init().
 */
void dshot_init(void);

/**
 * @brief Monta o pacote de 16 bits: 11 bits de valor, 1 de telemetria e 4 de CRC.
 *
 * @param[in] value      0..2047.
 * @param[in] telemetry  Pede telemetria ao ESC (usado nos comandos especiais).
 * @return O pacote, bit mais significativo primeiro.
 */
uint16_t dshot_packet(uint16_t value, bool telemetry);

/**
 * @brief Envia um quadro com um valor por motor.
 *
 * @param[in] value      Valor de 11 bits de cada motor (índice 0 = motor 1).
 * @param[in] telemetry  Bit de telemetria de todos os pacotes.
 * @return false se o quadro anterior ainda estava sendo enviado (nada muda).
 */
bool dshot_write(const uint16_t value[4], bool telemetry);

/**
 * @brief Converte décimos de por cento (0..1000) em valor DShot.
 *
 * 0 vira 0 (parado); 1..1000 vira 48..2047, linear.
 */
uint16_t dshot_from_permille(uint16_t permille);

/**
 * @brief Converte um valor DShot de aceleração (48..2047) em décimos de por cento.
 *
 * Inversa aproximada de ::dshot_from_permille(): 0 e valores abaixo de 48
 * viram 0 (parado); acima de 2047 vira 1000.
 */
uint16_t dshot_to_permille(uint16_t value);
#endif
