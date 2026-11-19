/**
 * @file telemetry.h
 * @brief Saída binária pela USB (bench-link v1), alimentada por fila.
 *
 * Quem produz dado (tasks do MTF, do BNO, dos motores e do enlace) só copia
 * um payload já codificado para uma fila, sem esperar. Uma task de prioridade
 * baixa enquadra cada payload (link/bench_frame.h, com número de sequência) e
 * entrega blocos à USB CDC. Fila cheia ou USB parada descartam e contam
 * (SYS.telemetry_dropped); nunca bloqueiam quem produz.
 *
 * Sem DTR ativo os quadros são descartados: nada chega a um programa que não
 * abriu a porta. Formato das mensagens: link/bench_proto.h.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "link/bench_proto.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct mtf_sample_s;
struct bno_sample_s;

/** @brief Cria a fila e a task. Chamar antes de quem produz dados. */
bool telemetry_start(void);

/** @brief Enfileira uma mensagem; false se foi descartada. Nunca bloqueia. */
bool telemetry_send(uint8_t id, const uint8_t *payload, size_t length);

/** @brief TEXT com nível ::bl_text_level_t (cortado em ::BL_TEXT_MAX). */
void telemetry_text(uint8_t level, const char *text);

/** @brief IMU a cada relatório de rotação, respeitando o divisor. */
void telemetry_bno(const struct bno_sample_s *sample);

/** @brief RANGE a cada frame do MTF-01. */
void telemetry_mtf(const struct mtf_sample_s *sample);

/** @brief MOTOR (chamada pela task dos motores a 100 Hz). */
void telemetry_motor(const bl_motor_t *motor);

/** @brief Liga/desliga IMU, RANGE e MOTOR (STATUS, SYS, POWER, ESC e TEXT sempre saem). */
void telemetry_set_stream(bool enabled, uint8_t bno_divider);

/** @brief Fluxo de dados ligado. */
bool telemetry_stream_enabled(void);

/** @brief Divisor atual do IMU (1 = 400 Hz). */
uint8_t telemetry_bno_divider(void);

/** @brief Mensagens descartadas desde o boot. */
uint32_t telemetry_dropped(void);

/** @brief Descarta o que ainda estava na fila (nova sessão USB). */
void telemetry_flush_queue(void);

#endif
