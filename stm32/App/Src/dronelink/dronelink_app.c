/**
 * @file dronelink_app.c
 * @brief Task do DroneLink (contrato em dronelink_app.h).
 */
#include "dronelink/dronelink_app.h"

#include "app/startup.h"
#include "dronelink/flight_command_api.h"
#include "dronelink/dronelink_boot_identity.h"
#include "dronelink/dronelink_protocol.h"
#include "dronelink/dronelink_transport.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

/** Pilha da task, em words. */
#define DRONELINK_TASK_STACK_WORDS 1024U
/** Prioridade da task (abaixo do controle e dos motores). */
#define DRONELINK_TASK_PRIORITY    (tskIDLE_PRIORITY + 2U)

static StaticTask_t dronelink_tcb;
static StackType_t dronelink_stack[DRONELINK_TASK_STACK_WORDS];

/**
 * @brief Inicia transporte, identidade de boot e protocolo; depois atende o enlace.
 *
 * Se o transporte não puder ser iniciado, a task se apaga.
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
  /* A Pi vê o boot novo pelo boot_id; este texto diz por que ele aconteceu. */
  flight_push_text(app_reset_cause(), false);

  for (;;) {
    const size_t received = dronelink_transport_receive(input, sizeof(input), 5U);
    if (received > 0U) {
      dronelink_protocol_consume(input, (uint16_t)received);
    }
    dronelink_protocol_tick(HAL_GetTick());
    dronelink_transport_service_tx();
  }
}

bool dronelink_app_init(void) {
  return xTaskCreateStatic(dronelink_task, "DroneLink", DRONELINK_TASK_STACK_WORDS, NULL,
                           DRONELINK_TASK_PRIORITY, dronelink_stack, &dronelink_tcb) != NULL;
}
