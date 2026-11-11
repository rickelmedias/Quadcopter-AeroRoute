/**
 * @file sensor_log.c
 * @brief Task de log alimentada por fila (contrato e formato em sensor_log.h).
 *
 * Os produtores chamam log_mtf(), log_bno(), log_motor() ou log_text(), que só
 * convertem a amostra para um registro pequeno e o colocam na fila com espera
 * zero. A task de log, com a menor prioridade útil, formata os registros num
 * bloco de texto e o entrega à USB CDC, que transmite sem a CPU (o periférico
 * USB lê o buffer sozinho). Nada aqui pode atrasar quem produz o dado.
 */
#include "sensor_log.h"
#include "mtf01.h"
#include "bno08x.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>

/* Os floats viram inteiros escalados no produtor: a newlib-nano é ligada sem
   suporte a %f, e converter ali é barato. q=9670 se lê 0,9670. */

/** Registros que cabem na fila (~140 ms de folga com tudo ligado). */
#define QUEUE_LENGTH  96U
/** Tamanho do bloco entregue de uma vez à USB (o buffer da CDC tem 2048). */
#define OUT_SIZE      1536U
/** Maior linha formatada, em bytes. */
#define LINE_MAX      256U
/** Com a USB ocupada por mais que isto (porta fechada no PC), o bloco é descartado. */
#define USB_STALL_MS  50U
/** Período da linha SYS, em ms. */
#define SYS_PERIOD_MS 1000U

/** Valores da linha EST: 9 medidas, 12 estados, 4 saídas e 4 referências. */
#define EST_VALUES 29U

/** Tipo de registro (uma etiqueta de linha cada). */
typedef enum {
  REC_TEXT,  /**< "# texto". */
  REC_MTF,   /**< Linha MTF. */
  REC_BNO,   /**< Linha BNO. */
  REC_MOTOR, /**< Linha MOT. */
  REC_EST    /**< Linha EST. */
} rec_type_t;

/** Campos da linha MTF (ver o formato em sensor_log.h). */
typedef struct {
  uint32_t sensor_ms;   /**< Relógio do sensor. */
  uint32_t distance_mm; /**< Distância. */
  uint32_t frames;      /**< Frames válidos. */
  uint32_t crc;         /**< Erros de checksum. */
  uint32_t gaps;        /**< Frames perdidos. */
  uint32_t uart;        /**< Erros de UART. */
  uint32_t drop;        /**< Bytes descartados. */
  int16_t fx;           /**< Fluxo X. */
  int16_t fy;           /**< Fluxo Y. */
  uint8_t quality;      /**< Qualidade da imagem. */
  uint8_t dist_status;  /**< Estado da distância. */
  uint8_t flow_status;  /**< Estado do fluxo. */
  uint8_t strength;     /**< Intensidade do telêmetro. */
  uint8_t precision;    /**< Precisão do telêmetro. */
  uint8_t range_ok;     /**< Distância utilizável. */
  uint8_t flow_ok;      /**< Fluxo utilizável. */
} rec_mtf_t;

/** Campos da linha BNO. */
typedef struct {
  int16_t q[4];     /**< Quatérnio x10000. */
  int16_t g[3];     /**< Giroscópio em mrad/s. */
  int32_t a[3];     /**< Aceleração linear em mm/s^2. */
  uint32_t n[3];    /**< Relatórios recebidos (rotação, giro, aceleração). */
  uint32_t gaps[3]; /**< Relatórios perdidos. */
  uint8_t acc[3];   /**< Acurácia informada pelo sensor. */
  uint8_t online;   /**< Sensor online. */
} rec_bno_t;

/** Campos da linha EST. */
typedef struct {
  int16_t v[EST_VALUES]; /**< y, x̂ e r em milésimos; du em décimos. */
  uint8_t state;         /**< Estado do voo. */
  uint8_t meas;          /**< Medidas válidas. */
  uint8_t flags;         /**< Situação do controle. */
} rec_est_t;

/** Registro na fila: já em inteiros, pronto para virar texto. */
typedef struct {
  uint8_t type;  /**< ::rec_type_t. */
  uint32_t t_ms; /**< Tick da amostra. */
  union {
    char text[61];     /**< REC_TEXT. */
    rec_mtf_t mtf;     /**< REC_MTF. */
    rec_bno_t bno;     /**< REC_BNO. */
    log_motor_t motor; /**< REC_MOTOR. */
    rec_est_t est;     /**< REC_EST. */
  } u;                 /**< Conteúdo, conforme o tipo. */
} rec_t;

/** Controle da fila (alocação estática). */
static StaticQueue_t queue_memory;
/** Armazenamento da fila. */
static uint8_t queue_storage[QUEUE_LENGTH * sizeof(rec_t)];
/** Fila de registros entre os produtores e a task de log. */
static QueueHandle_t queue;
/** Registros perdidos por fila cheia ou USB parada. */
static volatile uint32_t dropped;
/** Linhas de dados ligadas; texto e SYS sempre saem. */
static volatile bool stream_enabled = true;
/** Contador do divisor do BNO; usado só pela task do BNO. */
static uint32_t bno_divider;

/** @brief Coloca na fila sem esperar; conta o descarte se estiver cheia. */
static void push(const rec_t *r) {
  if (!queue)
    return;
  if (xQueueSend(queue, r, 0) != pdPASS)
    __atomic_fetch_add(&dropped, 1U, __ATOMIC_RELAXED);
}

/** @brief Liga ou silencia as linhas de dados (ver sensor_log.h). */
void log_set_stream(bool enabled) {
  stream_enabled = enabled;
}

/** @brief Enfileira uma linha de texto (ver sensor_log.h). */
void log_text(const char *text) {
  rec_t r;
  r.type = REC_TEXT;
  r.t_ms = HAL_GetTick();
  strncpy(r.u.text, text, sizeof(r.u.text) - 1U);
  r.u.text[sizeof(r.u.text) - 1U] = '\0';
  push(&r);
}

/** @brief Enfileira uma linha MTF (ver sensor_log.h). */
void log_mtf(const mtf_sample_t *m) {
#if LOG_MTF
  if (!stream_enabled)
    return;
  rec_t r;
  r.type = REC_MTF;
  r.t_ms = m->received_ms;
  r.u.mtf.sensor_ms = m->time_ms;
  r.u.mtf.distance_mm = m->distance_mm;
  r.u.mtf.frames = m->frames;
  r.u.mtf.crc = m->crc_errors;
  r.u.mtf.gaps = m->gaps;
  r.u.mtf.uart = m->uart_errors;
  r.u.mtf.drop = m->dropped_bytes;
  r.u.mtf.fx = m->flow_x;
  r.u.mtf.fy = m->flow_y;
  r.u.mtf.quality = m->quality;
  r.u.mtf.dist_status = m->distance_status;
  r.u.mtf.flow_status = m->flow_status;
  r.u.mtf.strength = m->strength;
  r.u.mtf.precision = m->precision;
  r.u.mtf.range_ok = m->range_valid;
  r.u.mtf.flow_ok = m->flow_valid;
  push(&r);
#else
  (void)m;
#endif
}

/** @brief Enfileira uma linha BNO (ver sensor_log.h). */
void log_bno(const bno_sample_t *b) {
#if LOG_BNO
  if (!stream_enabled || ++bno_divider < LOG_BNO_DIVIDER)
    return;
  bno_divider = 0;
  rec_t r;
  r.type = REC_BNO;
  r.t_ms = b->rotation.last_ms;
  for (int i = 0; i < 4; ++i)
    r.u.bno.q[i] = (int16_t)(b->q[i] * 10000.0F);
  for (int i = 0; i < 3; ++i) {
    r.u.bno.g[i] = (int16_t)(b->gyro[i] * 1000.0F);
    r.u.bno.a[i] = (int32_t)(b->lin_accel[i] * 1000.0F);
  }
  r.u.bno.n[0] = b->rotation.count;
  r.u.bno.gaps[0] = b->rotation.gaps;
  r.u.bno.acc[0] = b->rotation.accuracy;
  r.u.bno.n[1] = b->angular.count;
  r.u.bno.gaps[1] = b->angular.gaps;
  r.u.bno.acc[1] = b->angular.accuracy;
  r.u.bno.n[2] = b->acceleration.count;
  r.u.bno.gaps[2] = b->acceleration.gaps;
  r.u.bno.acc[2] = b->acceleration.accuracy;
  r.u.bno.online = b->online;
  push(&r);
#else
  (void)b;
#endif
}

/** @brief Enfileira uma linha MOT (ver sensor_log.h). */
void log_motor(const log_motor_t *m) {
#if LOG_MOTOR
  if (!stream_enabled)
    return;
  rec_t r;
  r.type = REC_MOTOR;
  r.t_ms = HAL_GetTick();
  r.u.motor = *m;
  push(&r);
#else
  (void)m;
#endif
}

/** @brief Converte com escala e satura em int16. */
static int16_t scaled(float v, float scale) {
  const float s = v * scale;
  if (!(s > -32767.0F))
    return s > 0.0F ? 32767 : -32767; /* também pega NaN */
  if (s > 32767.0F)
    return 32767;
  return (int16_t)s;
}

/** @brief Enfileira uma linha EST (ver sensor_log.h). */
void log_est(uint8_t state, uint8_t meas, uint8_t flags, const float y[9], const float x[12],
             const float du[4], const float ref[4]) {
#if LOG_EST
  if (!stream_enabled)
    return;
  rec_t r;
  r.type = REC_EST;
  r.t_ms = HAL_GetTick();
  r.u.est.state = state;
  r.u.est.meas = meas;
  r.u.est.flags = flags;
  int16_t *v = r.u.est.v;
  for (int i = 0; i < 9; ++i)
    *v++ = scaled(y[i], 1000.0F);
  for (int i = 0; i < 12; ++i)
    *v++ = scaled(x[i], 1000.0F);
  for (int i = 0; i < 4; ++i)
    *v++ = scaled(du[i], 10.0F);
  for (int i = 0; i < 4; ++i)
    *v++ = scaled(ref[i], 1000.0F);
  push(&r);
#else
  (void)state;
  (void)meas;
  (void)flags;
  (void)y;
  (void)x;
  (void)du;
  (void)ref;
#endif
}

/** @brief Converte um registro em uma linha de texto; devolve o tamanho. */
static int format(const rec_t *r, char *line, size_t size) {
  switch (r->type) {
  case REC_TEXT:
    return snprintf(line, size, "# %s\r\n", r->u.text);
  case REC_MTF:
    return snprintf(
        line, size, "MTF;%lu;%lu;%lu;%d;%d;%u;%u;%u;%u;%u;%u;%u;%lu;%lu;%lu;%lu;%lu\r\n",
        (unsigned long)r->t_ms, (unsigned long)r->u.mtf.sensor_ms,
        (unsigned long)r->u.mtf.distance_mm, r->u.mtf.fx, r->u.mtf.fy, r->u.mtf.quality,
        r->u.mtf.dist_status, r->u.mtf.flow_status, r->u.mtf.strength, r->u.mtf.precision,
        r->u.mtf.range_ok, r->u.mtf.flow_ok, (unsigned long)r->u.mtf.frames,
        (unsigned long)r->u.mtf.crc, (unsigned long)r->u.mtf.gaps, (unsigned long)r->u.mtf.uart,
        (unsigned long)r->u.mtf.drop);
  case REC_BNO:
    return snprintf(
        line, size,
        "BNO;%lu;%d;%d;%d;%d;%d;%d;%d;%ld;%ld;%ld;%u;%u;%u;%lu;%lu;%lu;%lu;%lu;%lu;%u\r\n",
        (unsigned long)r->t_ms, r->u.bno.q[0], r->u.bno.q[1], r->u.bno.q[2], r->u.bno.q[3],
        r->u.bno.g[0], r->u.bno.g[1], r->u.bno.g[2], (long)r->u.bno.a[0], (long)r->u.bno.a[1],
        (long)r->u.bno.a[2], r->u.bno.acc[0], r->u.bno.acc[1], r->u.bno.acc[2],
        (unsigned long)r->u.bno.n[0], (unsigned long)r->u.bno.n[1], (unsigned long)r->u.bno.n[2],
        (unsigned long)r->u.bno.gaps[0], (unsigned long)r->u.bno.gaps[1],
        (unsigned long)r->u.bno.gaps[2], r->u.bno.online);
  case REC_MOTOR:
    return snprintf(line, size, "MOT;%lu;%u;%u;%u;%u;%u;%u;%u;%u;%u\r\n", (unsigned long)r->t_ms,
                    r->u.motor.armed, r->u.motor.throttle_permille[0],
                    r->u.motor.throttle_permille[1], r->u.motor.throttle_permille[2],
                    r->u.motor.throttle_permille[3], r->u.motor.dshot[0], r->u.motor.dshot[1],
                    r->u.motor.dshot[2], r->u.motor.dshot[3]);
  case REC_EST: {
    int n = snprintf(line, size, "EST;%lu;%u;%u;%u", (unsigned long)r->t_ms, r->u.est.state,
                     r->u.est.meas, r->u.est.flags);
    for (unsigned i = 0; i < EST_VALUES && n > 0 && n < (int)size; ++i) {
      n += snprintf(line + n, size - (size_t)n, ";%d", r->u.est.v[i]);
    }
    if (n > 0 && n < (int)size)
      n += snprintf(line + n, size - (size_t)n, "\r\n");
    return n;
  }
  default:
    return 0;
  }
}

/** Bloco em montagem para a USB; usado só pela task de log. */
static uint8_t out[OUT_SIZE];
/** Bytes ocupados em ::out. */
static size_t used;
/** Tick do último envio aceito pela USB (ou do último descarte). */
static uint32_t busy_since;

/**
 * @brief Tenta entregar o bloco à USB.
 *
 * CDC_Transmit_FS copia o bloco e volta na hora. Se a USB continuar ocupada
 * por ::USB_STALL_MS (nenhum programa com a porta aberta no PC), o bloco é
 * descartado para que a fila não fique parada.
 */
static void flush(void) {
  if (!used)
    return;
  uint32_t now = HAL_GetTick();
  if (CDC_Transmit_FS(out, (uint16_t)used) == USBD_OK) {
    used = 0;
    busy_since = now;
  } else if (now - busy_since > USB_STALL_MS) {
    __atomic_fetch_add(&dropped, 1U, __ATOMIC_RELAXED);
    used = 0;
    busy_since = now;
  }
}

/**
 * @brief Task de log: esvazia a fila, formata e envia em blocos.
 *
 * @param[in] argument  Não usado.
 */
static void log_task(void *argument) {
  (void)argument;
  static char line[LINE_MAX];
  rec_t r;
  uint32_t last_sys = 0;
  busy_since = HAL_GetTick();
  for (;;) {
    if (xQueueReceive(queue, &r, pdMS_TO_TICKS(2)) == pdPASS) {
      do {
        int n = format(&r, line, sizeof(line));
        if (n <= 0 || n >= (int)sizeof(line))
          continue;
        while (used + (size_t)n > sizeof(out)) { /* bloco cheio: espera a USB */
          flush();
          if (used + (size_t)n > sizeof(out))
            vTaskDelay(1);
        }
        memcpy(out + used, line, (size_t)n);
        used += (size_t)n;
      } while (used + LINE_MAX <= sizeof(out) && xQueueReceive(queue, &r, 0) == pdPASS);
    }
#if LOG_SYS
    uint32_t now = HAL_GetTick();
    if (now - last_sys >= SYS_PERIOD_MS && used + LINE_MAX <= sizeof(out)) {
      last_sys = now;
      int n = snprintf(line, sizeof(line), "SYS;%lu;%lu;%lu\r\n", (unsigned long)now,
                       (unsigned long)dropped, (unsigned long)xPortGetFreeHeapSize());
      if (n > 0 && n < (int)sizeof(line)) {
        memcpy(out + used, line, (size_t)n);
        used += (size_t)n;
      }
    }
#else
    (void)last_sys;
#endif
    flush();
  }
}

/** Controle da task de log (alocação estática). */
static StaticTask_t log_tcb;
/** Pilha da task de log, em words. */
static StackType_t log_stack[768];

/** @brief Cria a fila e a task de log (ver sensor_log.h). */
bool sensor_log_start(void) {
  queue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(rec_t), queue_storage, &queue_memory);
  return queue && xTaskCreateStatic(log_task, "Log", sizeof(log_stack) / sizeof(log_stack[0]), NULL,
                                    tskIDLE_PRIORITY + 1, log_stack, &log_tcb) != NULL;
}
