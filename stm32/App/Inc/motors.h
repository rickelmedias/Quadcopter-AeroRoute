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
 * - ::MOTORS_OWNER_FLIGHT: o controle de voo (control/flight_ctrl.h), que
 *   escreve a aceleração dos quatro motores a cada passo do LQG.
 *
 * ::motors_stop() pode ser chamada de qualquer task e zera tudo no próximo
 * quadro. No voo, a saída também desarma se não receber atualização do controle
 * por 100 ms. O dono atual percebe a parada por ::motors_owner().
 *
 * Numeração: M1..M4 = PB6..PB9, a numeração física do drone (frente = M2 e
 * M4, trás = M1 e M3, direita = M1 e M2, esquerda = M3 e M4; M1 e M4 têm
 * hélice amarela, invertida). As colunas da matriz B do modelo seguem outra
 * ordem: control/mixer.h faz a conversão antes de chamar
 * ::motors_set_flight_throttle().
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
#define MOTORS_OWNER_NONE   MOTOR_ARB_OWNER_NONE
/** Teste de bancada (console USB ou IHM). */
#define MOTORS_OWNER_BENCH  MOTOR_ARB_OWNER_BENCH
/** Controle de voo. */
#define MOTORS_OWNER_FLIGHT MOTOR_ARB_OWNER_FLIGHT

/** @brief Fotografia da saída, para status e log. */
typedef struct {
  /** Dono atual. */
  motors_owner_t owner;
  /** Aceleração atual de cada motor, em décimos de por cento. */
  uint16_t permille[MOTORS_COUNT];
  /** Valor DShot de 11 bits enviado no último quadro. */
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
  /** Motores cujo sentido foi pedido desde o boot (os outros: o do ESC). */
  uint8_t reversed_known;
  /** Comandos especiais (beep, sentido) ainda na fila. */
  uint8_t commands_pending;
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
 * @param[in] owner  ::MOTORS_OWNER_BENCH ou ::MOTORS_OWNER_FLIGHT.
 * @return true se armou.
 */
bool motors_arm(motors_owner_t owner);

/**
 * @brief Zera e desarma no próximo quadro. Pode ser chamada de qualquer task.
 * @param[in] why  Motivo, registrado no log (texto fixo, não é copiado).
 */
void motors_stop(const char *why);

/** @brief Dono atual (::MOTORS_OWNER_NONE se desarmado). */
motors_owner_t motors_owner(void);

/**
 * @brief Aceleração de cada motor pedida pelo controle de voo.
 *
 * Só tem efeito enquanto o dono é ::MOTORS_OWNER_FLIGHT. Os valores são
 * limitados a [0, 1] e convertidos com a resolução total do DShot (2000
 * degraus).
 *
 * @param[in] throttle  Fração 0..1 de cada motor.
 */
void motors_set_flight_throttle(const float throttle[MOTORS_COUNT]);

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
 * Quem armou a bancada precisa chamar isto a cada < 300 ms (console: enquanto
 * a porta tem DTR; IHM: a cada heartbeat). Sem isso a saída desarma.
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

/** @brief Copia o estado atual. */
void motors_get_status(motors_status_t *out);

#endif
