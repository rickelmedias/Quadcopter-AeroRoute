/**
 * @file bno_history.h
 * @brief Histórico temporal puro para alinhar BNO08x e fluxo óptico.
 *
 * Os timestamps SH-2 usados pelo driver são construídos a partir do instante
 * da interrupção medido pelo STM32. Portanto pertencem à mesma linha de tempo
 * de `HAL_GetTick()*1000` usada na recepção do MTF-01. Este módulo não conhece
 * HAL nem FreeRTOS e pode ser exercitado no PC e no replay.
 */
#ifndef BNO_HISTORY_H
#define BNO_HISTORY_H

#include <stdbool.h>
#include <stdint.h>

/** 80 ms a 400 Hz por tipo de relatório; cobre folgadamente um frame MTF. */
#define BNO_HISTORY_CAPACITY 32U
/** Extrapolação máxima tolerada numa borda do intervalo. */
#define BNO_HISTORY_MAX_EXTRAPOLATION_US 20000U

typedef struct {
  uint32_t time_us;
  uint32_t id;
  float v[4];
} bno_history_quat_sample_t;

typedef struct {
  uint32_t time_us;
  uint32_t id;
  float v[3];
} bno_history_vec3_sample_t;

typedef struct {
  bno_history_quat_sample_t rotation[BNO_HISTORY_CAPACITY];
  bno_history_vec3_sample_t gyro[BNO_HISTORY_CAPACITY];
  bno_history_vec3_sample_t accel[BNO_HISTORY_CAPACITY];
  uint8_t rotation_head, rotation_count;
  uint8_t gyro_head, gyro_count;
  uint8_t accel_head, accel_count;
} bno_history_t;

/** Resultado alinhado ao intervalo de integração do MTF. */
typedef struct {
  /** Atitude interpolada no ponto médio do intervalo. */
  float q[4];
  /** Velocidade angular média no intervalo. */
  float gyro[3];
  /** Aceleração linear média no intervalo. */
  float accel[3];
  /** IDs mais recentes envolvidos, para diagnóstico e captura. */
  uint32_t rotation_id, gyro_id, accel_id;
} bno_aligned_sample_t;

void bno_history_reset(bno_history_t *h);
void bno_history_push_rotation(bno_history_t *h, uint32_t time_us,
                               uint32_t id, const float q[4]);
void bno_history_push_gyro(bno_history_t *h, uint32_t time_us,
                           uint32_t id, const float gyro[3]);
void bno_history_push_accel(bno_history_t *h, uint32_t time_us,
                            uint32_t id, const float accel[3]);

/**
 * @brief Alinha IMU ao intervalo [end_us-interval_us, end_us] do MTF.
 *
 * Gyro e aceleração são integrados por trapézios e divididos pelo intervalo;
 * o quatérnio é interpolado no ponto médio pelo caminho curto e normalizado.
 * A aritmética de tempo suporta o wrap de 32 bits desde que o histórico seja
 * curto, como é o caso aqui.
 */
bool bno_history_align(const bno_history_t *h, uint32_t end_us,
                       uint32_t interval_us, bno_aligned_sample_t *out);

#endif
