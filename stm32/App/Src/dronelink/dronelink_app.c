/**
 * @file dronelink_app.c
 * @brief Task do DroneLink (contrato em dronelink_app.h).
 */
#include "dronelink/dronelink_app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "dronelink/dronelink_boot_identity.h"
#include "dronelink/dronelink_protocol.h"
#include "dronelink/dronelink_transport.h"
#include "usart.h"

/** Pilha da task, em words. */
#define DRONELINK_TASK_STACK_WORDS 1024U
/** Prioridade da task (a mesma dos sensores). */
#define DRONELINK_TASK_PRIORITY    (tskIDLE_PRIORITY + 2U)

/** Handle da task criada. */
static TaskHandle_t dronelink_task_handle;

/**
 * @brief Task do DroneLink: inicia transporte, identidade de boot e
 *        protocolo, e então atende o enlace continuamente.
 *
 * Se o transporte não puder ser iniciado, a task se apaga.
 *
 * @param[in] argument  Não usado.
 */
static void dronelink_task(void *argument) {
  (void)argument;
  uint8_t input[128];

  if (!dronelink_transport_init(&huart1)) {
    vTaskDelete(NULL);
  }
  uint32_t boot_id = 0U;
  const bool boot_id_is_persistent = dronelink_boot_identity_init(&boot_id);
  dronelink_protocol_init(boot_id, boot_id_is_persistent);

  for (;;) {
    const size_t received = dronelink_transport_receive(input, sizeof(input), 5U);
    if (received > 0U) {
      dronelink_protocol_consume(input, (uint16_t)received);
    }
    dronelink_protocol_tick(HAL_GetTick());
    dronelink_transport_service_tx();
  }
}

/** @brief Cria a task do DroneLink (ver dronelink_app.h). */
bool dronelink_app_init(void) {
  return xTaskCreate(dronelink_task, "DroneLink", DRONELINK_TASK_STACK_WORDS, NULL,
                     DRONELINK_TASK_PRIORITY, &dronelink_task_handle) == pdPASS;
}
