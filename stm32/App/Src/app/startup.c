/** @file startup.c @brief Composição dos serviços da bancada. */
#include "app/startup.h"

#include "actuators/esc_telem.h"
#include "app/bench_link.h"
#include "app/control_task.h"
#include "dronelink/dronelink_app.h"
#include "motors.h"
#include "power.h"
#include "sensors.h"
#include "stm32f4xx_hal.h"
#include "telemetry.h"

static const char *reset_cause = "boot: causa desconhecida";

/** @brief Lê e limpa os flags de reset do RCC (o POR também liga BOR e PIN). */
static void read_reset_cause(void) {
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != 0U) {
    reset_cause = "boot: reset de baixo consumo";
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != 0U) {
    reset_cause = "boot: watchdog de janela";
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U) {
    reset_cause = "boot: watchdog (firmware travou)";
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0U) {
    reset_cause = "boot: reinicio por software ou gravacao";
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != 0U) {
    reset_cause = "boot: ligou (energia aplicada)";
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != 0U) {
    reset_cause = "boot: QUEDA DE TENSAO (brown-out)";
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0U) {
    reset_cause = "boot: botao RESET ou pino NRST";
  } else {
    /* sem flag */
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

const char *app_reset_cause(void) {
  return reset_cause;
}

bool app_services_start(void) {
  read_reset_cause();
  /* Telemetria antes dos produtores; ESC antes dos motores (agenda KISS). */
  /* DroneLink por último: a identidade de boot lê a flash SPI antes de atender a Pi. */
  return telemetry_start() && sensors_init() && power_start() && esc_telem_start() &&
         motors_start() && control_task_start() && bench_link_start() &&
         ((ESC_TELEM_UART != 0) || dronelink_app_init());
}
