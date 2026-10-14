/**
 * @file sensor_log.h
 * @brief Log de texto pela USB (CDC) alimentado por fila, e chaves dos perfis.
 *
 * Quem produz dado (tasks do MTF, do BNO e dos motores) só copia um registro
 * binário pequeno para uma fila, sem esperar e sem formatar texto. Uma task de
 * prioridade baixa esvazia a fila, formata as linhas e as envia pela USB em
 * blocos. Assim o log nunca atrasa a aquisição nem o controle: se a USB estiver
 * desconectada ou lenta, a fila enche e os registros novos são descartados e
 * contados (linha SYS), sem bloquear ninguém.
 *
 * Cada linha começa com uma etiqueta e usa ';' como separador, para que o
 * mesmo arquivo de log possa ser separado depois por tipo:
 *
 *   MTF;t_ms;sensor_ms;distance_mm;flow_x;flow_y;quality;dist_status;
 *       flow_status;strength;precision;range_ok;flow_ok;frames;crc;gaps;uart_err;drop
 *   BNO;t_ms;qw;qx;qy;qz (x10000);gx;gy;gz (mrad/s);ax;ay;az (mm/s^2);
 *       acc_rot;acc_gyro;acc_lin;n_rot;n_gyro;n_lin;gaps_rot;gaps_gyro;gaps_lin;online
 *   MOT;t_ms;armed;thr1..thr4 (0,1 %);dshot1..dshot4 (valor de 11 bits enviado)
 *   EST;t_ms;modo;x;y;z;phi;theta;psi;vx;vy;vz;p;q;r (x1000, SI);w1..w4 (rad/s x10)
 *   SYS;t_ms;log_drop;heap_livre
 *   # texto livre (respostas de comando, avisos)
 *
 * Com CAPTURE_BUILD definido (tools/build.py --capture), o log de texto é
 * desligado e a captura binária é ligada, sem editar este arquivo.
 */
#ifndef SENSOR_LOG_H
#define SENSOR_LOG_H
#include <stdbool.h>
#include <stdint.h>

#if defined(CAPTURE_BUILD)
#define LOG_MTF   0
#define LOG_BNO   0
#define LOG_MOTOR 0
#define LOG_SYS   0
#define LOG_EST   0
#else
/** Uma linha MTF a cada frame do MTF-01 (100 Hz). */
#define LOG_MTF   1
/** Uma linha BNO a cada LOG_BNO_DIVIDER relatórios de rotação. */
#define LOG_BNO   1
/** Uma linha MOT a cada período da task de motores. */
#define LOG_MOTOR 1
/** Uma linha EST a cada LOG_EST_DIVIDER passos do LQG. */
#define LOG_EST   1
/** Uma linha SYS por segundo: descartes do log e heap livre. */
#define LOG_SYS   1
#endif

/** Registra 1 a cada N amostras do BNO (1 = todas, 4 = 100 Hz com o BNO a 400 Hz). */
#define LOG_BNO_DIVIDER 1U

/** Registra 1 a cada N passos do LQG (8 = 50 Hz com a malha a 400 Hz). */
#define LOG_EST_DIVIDER 8U

/**
 * @brief Mantido em zero: os perfis de teste não criam estimadores de
 *        deslocamento.
 */
#define DISP_CALIBRATE_FLOW 0

/** Captura binária pela USB, só no build Capture (ver capture.h). */
#if defined(CAPTURE_BUILD)
#define CAPTURE_ENABLED 1
#else
#define CAPTURE_ENABLED 0
#endif

/** Estado dos motores registrado na linha MOT. */
typedef struct {
  /** Comando de cada motor, em décimos de por cento (0..1000). */
  uint16_t throttle_permille[4];
  /** Valor DShot de 11 bits efetivamente enviado (0 = parado, 48..2047). */
  uint16_t dshot[4];
  /** Verdadeiro quando a saída está armada. */
  bool armed;
} log_motor_t;

struct mtf_sample_s;
struct bno_sample_s;

/**
 * @brief Cria a fila e a task de log.
 *
 * Chamada em MX_FREERTOS_Init(), antes de o scheduler iniciar. No build
 * Capture não cria nada.
 *
 * @return true se a task foi criada (sempre true no build Capture).
 */
bool sensor_log_start(void);

/** @brief Enfileira uma linha MTF; nunca bloqueia. Chamar só de task. */
void log_mtf(const struct mtf_sample_s *m);

/** @brief Enfileira uma linha BNO (respeitando ::LOG_BNO_DIVIDER); nunca bloqueia. */
void log_bno(const struct bno_sample_s *b);

/** @brief Enfileira uma linha MOT; nunca bloqueia. */
void log_motor(const log_motor_t *m);

/**
 * @brief Enfileira uma linha EST: 12 estados estimados e 4 rotações; nunca bloqueia.
 *
 * @param[in] mode  0 parado, 1 ensaio a seco, 2 voo.
 */
void log_est(uint8_t mode, const float x[12], const float u[4]);

/**
 * @brief Enfileira uma linha de texto começando com "# ".
 *
 * Texto mais longo que 60 caracteres é cortado. Nunca bloqueia.
 */
void log_text(const char *text);

/** @brief Liga (true) ou silencia (false) as linhas MTF/BNO/MOT em tempo de execução. */
void log_set_stream(bool enabled);
#endif
