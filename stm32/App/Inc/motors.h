/**
 * @file motors.h
 * @brief Saída dos quatro ESCs (DShot300) e quem pode comandá-los.
 *
 * Uma task de alta prioridade envia um quadro DShot a cada 1 ms. Enquanto
 * ninguém armou, ela manda só zeros, que é o que o ESC precisa para armar.
 * Dois clientes podem armar a saída, um de cada vez:
 *
 * - ::MOTORS_OWNER_BENCH: teste de bancada pelo console USB ou pela IHM. Cada
 *   motor segue um alvo em décimos de por cento, com rampa limitada, ou uma
 *   varredura, nunca acima do limite ajustável (40 % ao ligar, até 100 %). Desarma sem
 *   ::motors_bench_keepalive() por 300 ms.
 *
 * - ::MOTORS_OWNER_CONTROL: a task de controle escreve os quatro motores a cada
 *   10 ms com ::motors_control_set(), sem rampa, até o limite do controle
 *   (80 % ao ligar). Desarma sem atualização por 100 ms e sem o keepalive da IHM.
 *
 * ::motors_stop() pode ser chamada de qualquer task e pede zero no próximo
 * quadro. Se o DMA recusar, desarma e repete zero; status preserva o último
 * quadro aceito. No voo, também desarma se não receber atualização do controle
 * por 100 ms. O dono atual percebe a parada por ::motors_owner().
 *
 * Numeração: M1..M4 = PB6..PB9, a numeração física do drone.
 */
#ifndef MOTORS_H
#define MOTORS_H

#include "actuators/motor_arbiter.h"

#include <stdbool.h>
#include <stdint.h>

/** Quantidade de motores. */
#define MOTORS_COUNT              MOTOR_ARB_COUNT
/** Limite fixo de aceleração da bancada, em décimos de por cento (1000 = 100 %). */
#define MOTORS_BENCH_MAX_PERMILLE MOTOR_ARB_BENCH_HARD_MAX

/** @brief Quem está com a saída armada. */
typedef motor_arb_owner_t motors_owner_t;
/** Desarmado: só zeros. */
#define MOTORS_OWNER_NONE  MOTOR_ARB_OWNER_NONE
/** Teste de bancada (console USB ou IHM). */
#define MOTORS_OWNER_BENCH MOTOR_ARB_OWNER_BENCH
/** Controle LQR (modo de teste com cabo, task de controle a 100 Hz). */
#define MOTORS_OWNER_CONTROL MOTOR_ARB_OWNER_CONTROL

/** @brief Fotografia da saída, para status e log. */
typedef struct {
  /** Dono atual. */
  motors_owner_t owner;
  /** Aceleração atual de cada motor, em décimos de por cento. */
  uint16_t permille[MOTORS_COUNT];
  /** Valor DShot de 11 bits do último quadro aceito pelo DMA (não é ACK do ESC). */
  uint16_t dshot[MOTORS_COUNT];
  /** Alvo da bancada, em décimos de por cento. */
  uint16_t bench_target[MOTORS_COUNT];
  /** Rampa da bancada, em décimos de por cento por segundo. */
  uint32_t bench_rate;
  /** Varredura da bancada em andamento. */
  bool sweeping;
  /** Limite ajustável da bancada, em décimos de por cento. */
  uint16_t bench_cap;
  /** Último sentido pedido: bit i = motor i+1 invertido. */
  uint8_t reversed;
  /** Direção confirmada pelo ESC: 0 enquanto não houver readback. */
  uint8_t reversed_known;
  /** Pedido de direção transmitido por completo; não é confirmação do ESC. */
  uint8_t direction_transmitted;
  /** Comandos especiais (beep, sentido) ainda na fila. */
  uint8_t commands_pending;
  /** Quadros recusados pelo DMA; o árbitro desarma em cada falha. */
  uint32_t tx_failures;
} motors_status_t;

/**
 * @brief Inicia o DShot e cria a task dos motores.
 * @return true se a task foi criada.
 */
bool motors_start(void);

/**
 * @brief Arma a saída para um dono.
 *
 * Recusa se já houver dono ou se algum motor não estiver em zero.
 *
 * @param[in] owner  ::MOTORS_OWNER_BENCH.
 * @return true se armou.
 */
bool motors_arm(motors_owner_t owner);

/**
 * @brief Pede zero e desarma na próxima iteração. Não usar em interrupção.
 * @param[in] why  Motivo, registrado no log (texto fixo, não é copiado).
 */
void motors_stop(const char *why);

/** @brief Dono atual (::MOTORS_OWNER_NONE se desarmado). */
motors_owner_t motors_owner(void);

/**
 * @brief Alvo de bancada de um ou mais motores.
 * @param[in] mask      Bit i = motor i+1.
 * @param[in] permille  0..1000.
 * @return false se a bancada não está armada.
 */
bool motors_bench_set(uint8_t mask, uint16_t permille);

/**
 * @brief Velocidade máxima da rampa de bancada.
 * @param[in] permille_per_s  Décimos de por cento por segundo (10..1000).
 */
void motors_bench_rate(uint32_t permille_per_s);

/**
 * @brief Varredura de bancada: sobe de 0 até @p max em degraus, segura cada
 *        degrau e desce até 0.
 * @return false se a bancada não está armada.
 */
bool motors_bench_sweep(uint8_t mask, uint16_t max_permille, uint16_t step_permille,
                        uint32_t hold_ms);

/**
 * @brief Faz os ESCs apitarem para identificar os motores (só desarmado).
 * @param[in] mask  Bit i = motor i+1.
 * @return false se armado.
 */
bool motors_beep(uint8_t mask);

/**
 * @brief Renova o keepalive da bancada.
 *
 * Chamar somente ao receber um ping/heartbeat válido do cliente a cada < 300 ms.
 * DTR ativo não renova o prazo. Sem mensagens a saída desarma.
 */
void motors_bench_keepalive(void);

/**
 * @brief Limite ajustável da bancada (até ::MOTORS_BENCH_MAX_PERMILLE).
 * @return O limite aplicado, em décimos de por cento.
 */
uint16_t motors_bench_cap(uint16_t permille);

/**
 * @brief Sentido de rotação (comandos 7/8 do AM32; 12 grava na EEPROM do ESC).
 * @param[in] mask      Motores a configurar (bit i = motor i+1).
 * @param[in] reversed  Bit i = motor i+1 invertido.
 * @param[in] save      Grava no ESC; sem isso o sentido volta ao ligar.
 * @return false se armado, girando ou com fila cheia.
 */
bool motors_set_direction(uint8_t mask, uint8_t reversed, bool save);

/**
 * @brief Aceleração direta dos quatro motores pelo controle (só com dono controle).
 * @param[in] permille  Décimos de por cento de M1..M4, limitados ao limite do controle.
 * @return false se o controle não está armado.
 */
bool motors_control_set(const uint16_t permille[MOTORS_COUNT]);

/**
 * @brief Limite do controle (até ::MOTORS_BENCH_MAX_PERMILLE).
 * @return O limite aplicado, em décimos de por cento.
 */
uint16_t motors_control_cap(uint16_t permille);

/** @brief Copia o estado atual. */
void motors_get_status(motors_status_t *out);

#endif
