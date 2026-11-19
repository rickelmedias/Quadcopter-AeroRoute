/**
 * @file telemetry.c
 * @brief Task de saída binária pela USB (contrato em telemetry.h).
 */
#include "telemetry.h"

#include "bno08x.h"
#include "main.h"
#include "mtf01.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <string.h>

/** Mensagens que cabem na fila (~150 ms de folga com o IMU a 400 Hz). */
#define QUEUE_LENGTH 80U
/** Bloco entregue de uma vez à USB (o buffer da CDC tem 2048). */
#define OUT_SIZE     1536U
/** Com a USB ocupada por mais que isto, o bloco é descartado. */
#define USB_STALL_MS 50U

/** Uma mensagem já codificada. */
typedef struct {
  uint8_t id;
  uint8_t length;
  uint8_t payload[BL_MAX_PAYLOAD];
} record_t;

static StaticQueue_t queue_memory;
static uint8_t queue_storage[QUEUE_LENGTH * sizeof(record_t)];
static QueueHandle_t queue;
static volatile uint32_t dropped;
static volatile bool stream_enabled = true;
static volatile uint8_t bno_divider = 1U;
/** Contador do divisor; usado só pela task do BNO. */
static uint8_t bno_count;

bool telemetry_send(uint8_t id, const uint8_t *payload, size_t length) {
  bool ok = false;
  if ((queue != NULL) && (length <= BL_MAX_PAYLOAD)) {
    record_t r;
    r.id = id;
    r.length = (uint8_t)length;
    (void)memcpy(r.payload, payload, length);
    ok = xQueueSend(queue, &r, 0) == pdPASS;
  }
  if (!ok) {
    __atomic_fetch_add(&dropped, 1U, __ATOMIC_RELAXED);
  }
  return ok;
}

void telemetry_text(uint8_t level, const char *text) {
  uint8_t payload[BL_MAX_PAYLOAD];
  const size_t n = bl_encode_text(HAL_GetTick(), level, text, payload);
  (void)telemetry_send(BL_MSG_TEXT, payload, n);
}

void telemetry_set_stream(bool enabled, uint8_t divider) {
  stream_enabled = enabled;
  bno_divider = (divider == 0U) ? 1U : divider;
}

bool telemetry_stream_enabled(void) {
  return stream_enabled;
}

uint8_t telemetry_bno_divider(void) {
  return bno_divider;
}

uint32_t telemetry_dropped(void) {
  return dropped;
}

void telemetry_flush_queue(void) {
  if (queue != NULL) {
    (void)xQueueReset(queue);
  }
}

void telemetry_bno(const bno_sample_t *b) {
  ++bno_count;
  if (stream_enabled && (bno_count >= bno_divider)) {
    bno_count = 0U;
    bl_imu_t m;
    m.t_ms = b->rotation.last_ms;
    for (uint32_t i = 0U; i < 4U; ++i) {
      m.q[i] = b->q[i];
    }
    for (uint32_t i = 0U; i < 3U; ++i) {
      m.gyro[i] = b->gyro[i];
      m.lin_accel[i] = b->lin_accel[i];
    }
    m.accuracy[0] = b->rotation.accuracy;
    m.accuracy[1] = b->angular.accuracy;
    m.accuracy[2] = b->acceleration.accuracy;
    m.flags = b->online ? BL_IMU_ONLINE : 0U;
    m.count[0] = b->rotation.count;
    m.count[1] = b->angular.count;
    m.count[2] = b->acceleration.count;
    m.gaps[0] = b->rotation.gaps;
    m.gaps[1] = b->angular.gaps;
    m.gaps[2] = b->acceleration.gaps;
    uint8_t payload[BL_MAX_PAYLOAD];
    (void)telemetry_send(BL_MSG_IMU, payload, bl_encode_imu(&m, payload));
  }
}

void telemetry_mtf(const mtf_sample_t *s) {
  if (stream_enabled) {
    bl_range_t m;
    m.t_ms = s->received_ms;
    m.sensor_ms = s->time_ms;
    m.distance_mm = s->distance_mm;
    m.flow_x = s->flow_x;
    m.flow_y = s->flow_y;
    m.quality = s->quality;
    m.distance_status = s->distance_status;
    m.flow_status = s->flow_status;
    m.strength = s->strength;
    m.precision = s->precision;
    m.flags =
        (uint8_t)((s->range_valid ? BL_RANGE_VALID : 0U) | (s->flow_valid ? BL_FLOW_VALID : 0U));
    m.frames = s->frames;
    m.crc_errors = s->crc_errors;
    m.gaps = s->gaps;
    m.uart_errors = s->uart_errors;
    m.dropped_bytes = s->dropped_bytes;
    uint8_t payload[BL_MAX_PAYLOAD];
    (void)telemetry_send(BL_MSG_RANGE, payload, bl_encode_range(&m, payload));
  }
}

void telemetry_motor(const bl_motor_t *motor) {
  if (stream_enabled) {
    uint8_t payload[BL_MAX_PAYLOAD];
    (void)telemetry_send(BL_MSG_MOTOR, payload, bl_encode_motor(motor, payload));
  }
}

static uint8_t out[OUT_SIZE];
static size_t used;
static uint32_t busy_since;

/** @brief Entrega o bloco à USB; descarta se ninguém ler por ::USB_STALL_MS. */
static void flush(void) {
  if (used > 0U) {
    const uint32_t now = HAL_GetTick();
    if (!CDC_HostDTR()) {
      used = 0U; /* porta fechada: nada a entregar */
      busy_since = now;
    } else if (CDC_Transmit_FS(out, (uint16_t)used) == USBD_OK) {
      used = 0U;
      busy_since = now;
    } else if ((now - busy_since) > USB_STALL_MS) {
      __atomic_fetch_add(&dropped, 1U, __ATOMIC_RELAXED);
      used = 0U;
      busy_since = now;
    } else {
      /* USB ocupada: tenta de novo na próxima volta. */
    }
  }
}

static void telemetry_task(void *argument) {
  (void)argument;
  record_t r;
  uint8_t seq = 0U;
  busy_since = HAL_GetTick();
  for (;;) {
    if (xQueueReceive(queue, &r, pdMS_TO_TICKS(2)) == pdPASS) {
      bool more = true;
      while (more) {
        uint8_t frame[BL_FRAME_MAX];
        const size_t n = bl_frame_encode(r.id, seq, r.payload, r.length, frame, sizeof(frame));
        ++seq;
        while ((n > 0U) && ((used + n) > sizeof(out))) {
          flush();
          if ((used + n) > sizeof(out)) {
            vTaskDelay(1);
          }
        }
        if (n > 0U) {
          (void)memcpy(&out[used], frame, n);
          used += n;
        }
        more = ((used + BL_FRAME_MAX) <= sizeof(out)) && (xQueueReceive(queue, &r, 0) == pdPASS);
      }
    }
    flush();
  }
}

static StaticTask_t task_tcb;
static StackType_t task_stack[512];

bool telemetry_start(void) {
  queue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(record_t), queue_storage, &queue_memory);
  return (queue != NULL) &&
         (xTaskCreateStatic(telemetry_task, "Telemetry", sizeof(task_stack) / sizeof(task_stack[0]),
                            NULL, tskIDLE_PRIORITY + 1, task_stack, &task_tcb) != NULL);
}
