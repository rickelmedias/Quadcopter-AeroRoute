/**
 * @file motor_arbiter.h
 * @brief Lógica pura da saída dos motores: dono, rampa, limites, failsafes e
 *        comandos especiais do DShot.
 *
 * Não depende de HAL nem de FreeRTOS: o tempo entra como parâmetro e quem
 * chama (motors.c) protege a estrutura com seção crítica. Por isso pode ser
 * testada no PC (tests/test_motor_arbiter.c).
 *
 * Regras de segurança garantidas aqui:
 *
 * - Desarmado, todos os motores recebem DShot 0 em todo quadro, exceto
 *   durante um comando especial (beep, sentido), que só é aceito desarmado e
 *   com tudo parado.
 * - Armar exige dono livre, alvos e rampa em zero e nenhum comando especial
 *   pendente.
 * - A bancada nunca passa de ::MOTOR_ARB_BENCH_HARD_MAX. O limite ajustável
 *   (::motor_arb_bench_cap) só pode ser menor.
 * - A bancada desarma sem keepalive por ::MOTOR_ARB_BENCH_KEEPALIVE_MS e o
 *   voo sem atualização por ::MOTOR_ARB_FLIGHT_TIMEOUT_MS.
 */
#ifndef MOTOR_ARBITER_H
#define MOTOR_ARBITER_H

#include <stdbool.h>
#include <stdint.h>

/** Quantidade de motores. */
#define MOTOR_ARB_COUNT              4U
/** Limite fixo da bancada, em décimos de por cento (400 = 40 %). */
#define MOTOR_ARB_BENCH_HARD_MAX     400U
/** Sem keepalive da bancada por este tempo, em ms, desarma. */
#define MOTOR_ARB_BENCH_KEEPALIVE_MS 300U
/** Sem atualização do controle de voo por este tempo, em ms, desarma. */
#define MOTOR_ARB_FLIGHT_TIMEOUT_MS  100U
/** Rampa padrão da bancada, em décimos de por cento por segundo (20 %/s). */
#define MOTOR_ARB_DEFAULT_RATE       200U
/** Comandos especiais na fila. */
#define MOTOR_ARB_QUEUE_LEN          4U

/** Comando DShot: sentido normal (AM32, só na RAM do ESC). */
#define MOTOR_ARB_CMD_DIR_NORMAL   7U
/** Comando DShot: sentido invertido (AM32, só na RAM do ESC). */
#define MOTOR_ARB_CMD_DIR_REVERSED 8U
/** Comando DShot: grava a configuração na EEPROM do ESC. */
#define MOTOR_ARB_CMD_SAVE         12U
/** Quadros de cada comando especial (o protocolo pede pelo menos 6). */
#define MOTOR_ARB_CMD_FRAMES       10U
/** Quadros em zero depois do comando de sentido, em ms. */
#define MOTOR_ARB_DIR_GAP_FRAMES   10U
/** Quadros em zero depois de gravar a EEPROM, em ms. */
#define MOTOR_ARB_SAVE_GAP_FRAMES  100U

/** @brief Quem está com a saída armada. */
typedef enum {
  MOTOR_ARB_OWNER_NONE = 0, /**< Desarmado: só zeros. */
  MOTOR_ARB_OWNER_BENCH,    /**< Teste de bancada (console USB ou IHM). */
  MOTOR_ARB_OWNER_FLIGHT    /**< Controle de voo. */
} motor_arb_owner_t;

/** @brief Varredura automática da bancada. */
typedef struct {
  bool active;      /**< Varredura em andamento. */
  uint8_t mask;     /**< Bit 0 = motor 1. */
  uint16_t max;     /**< Décimos de por cento. */
  uint16_t step;    /**< Décimos de por cento. */
  uint32_t hold_ms; /**< Tempo em cada degrau. */
  uint16_t level;   /**< Degrau atual. */
  bool down;        /**< Já está descendo. */
  uint32_t next_ms; /**< Próxima troca de degrau. */
} motor_arb_sweep_t;

/** @brief Comando especial: um valor DShot por motor, repetido, e uma pausa em zero. */
typedef struct {
  uint16_t value[MOTOR_ARB_COUNT]; /**< 0 = motor sem comando neste item. */
  uint32_t frames;                 /**< Quadros com o comando que faltam. */
  uint32_t gap;                    /**< Quadros em zero que faltam depois. */
} motor_arb_cmd_t;

/** @brief Estado completo. Só acessar pelas funções abaixo. */
typedef struct {
  motor_arb_owner_t owner;
  uint16_t target[MOTOR_ARB_COUNT];  /**< Alvo da bancada, décimos de %. */
  uint32_t current[MOTOR_ARB_COUNT]; /**< Rampa, em milésimos de décimo de %. */
  uint32_t rate;                     /**< Rampa, décimos de % por segundo. */
  uint16_t cap;                      /**< Limite ajustável da bancada. */
  motor_arb_sweep_t sweep;
  float flight_throttle[MOTOR_ARB_COUNT]; /**< Pedido do voo, 0..1. */
  uint32_t flight_ms;                     /**< Última atualização do voo. */
  uint32_t bench_alive_ms;                /**< Último keepalive da bancada. */
  motor_arb_cmd_t queue[MOTOR_ARB_QUEUE_LEN];
  uint32_t queue_head;
  uint32_t queue_len;
  uint8_t reversed;       /**< Último sentido pedido (bit i = motor i+1 invertido). */
  uint8_t reversed_known; /**< Motores cujo sentido foi pedido desde o boot. */
  uint16_t sent_dshot[MOTOR_ARB_COUNT];
  uint16_t sent_permille[MOTOR_ARB_COUNT];
  int32_t sweep_note; /**< Degrau novo (>= 0), fim (-1) ou nada (-2). */
} motor_arb_t;

/** @brief Resultado de um quadro. */
typedef struct {
  uint16_t value[MOTOR_ARB_COUNT]; /**< Valor DShot de 11 bits de cada motor. */
  bool telemetry;                  /**< Bit de telemetria (exigido nos comandos). */
  const char *stopped;             /**< Motivo se desarmou neste quadro, senão NULL. */
} motor_arb_frame_t;

/** @brief Estado inicial: desarmado, limite = limite fixo. */
void motor_arb_init(motor_arb_t *a, uint32_t now_ms);

/**
 * @brief Arma para um dono.
 * @return false se já há dono, algo não está em zero ou há comando pendente.
 */
bool motor_arb_arm(motor_arb_t *a, motor_arb_owner_t owner, uint32_t now_ms);

/** @brief Zera tudo e desarma agora. */
void motor_arb_disarm(motor_arb_t *a);

/** @brief Pedido do controle de voo (só com dono voo). Valores fora de 0..1 são limitados. */
void motor_arb_set_flight(motor_arb_t *a, const float throttle[MOTOR_ARB_COUNT], uint32_t now_ms);

/** @brief Renova o keepalive da bancada. */
void motor_arb_bench_keepalive(motor_arb_t *a, uint32_t now_ms);

/**
 * @brief Alvo de bancada dos motores da máscara, limitado ao limite ativo.
 * @return false se a bancada não está armada.
 */
bool motor_arb_bench_set(motor_arb_t *a, uint8_t mask, uint16_t permille);

/** @brief Rampa da bancada, limitada a 10..1000 décimos de % por segundo. */
void motor_arb_bench_rate(motor_arb_t *a, uint32_t permille_per_s);

/**
 * @brief Limite ajustável da bancada, limitado a ::MOTOR_ARB_BENCH_HARD_MAX.
 *
 * Abaixar o limite também abaixa os alvos e a varredura em andamento.
 *
 * @return O limite aplicado.
 */
uint16_t motor_arb_bench_cap(motor_arb_t *a, uint16_t permille);

/**
 * @brief Varredura: sobe de 0 até @p max em degraus, segura cada um e desce.
 * @return false se a bancada não está armada.
 */
bool motor_arb_bench_sweep(motor_arb_t *a, uint8_t mask, uint16_t max_permille,
                           uint16_t step_permille, uint32_t hold_ms, uint32_t now_ms);

/**
 * @brief Beep nos motores da máscara.
 * @return false se armado, algo não está parado ou a fila está cheia.
 */
bool motor_arb_beep(motor_arb_t *a, uint8_t mask);

/**
 * @brief Sentido de rotação dos motores da máscara.
 *
 * @param[in] mask      Motores a configurar.
 * @param[in] reversed  Bit i = motor i+1 invertido (só os bits de @p mask contam).
 * @param[in] save      Também grava na EEPROM do ESC (senão volta ao ligar).
 * @return false se armado, algo não está parado ou a fila está cheia.
 */
bool motor_arb_set_direction(motor_arb_t *a, uint8_t mask, uint8_t reversed, bool save);

/**
 * @brief Calcula o quadro deste milissegundo e aplica os failsafes.
 * @param[in] stop  Pedido de parada (motivo) ou NULL.
 */
motor_arb_frame_t motor_arb_frame(motor_arb_t *a, uint32_t now_ms, const char *stop);

#endif
