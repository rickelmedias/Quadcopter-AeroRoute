/**
 * @file flight_command_stub.c
 * @brief Implementações provisórias do contrato de flight_command_api.h.
 *
 * São funções fracas (weak): quando o controle de voo real definir funções
 * com o mesmo nome, estas deixam de ser usadas, sem precisar apagar este
 * arquivo. Enquanto isso, tudo é negado com FLIGHT_REASON_NOT_IMPLEMENTED,
 * exceto desarmar.
 */
#include "dronelink/flight_command_api.h"

#include <string.h>

/** Marca uma função como fraca (pode ser substituída na ligação). */
#define FLIGHT_WEAK __attribute__((weak))

/**
 * @brief Resposta padrão das implementações provisórias.
 * @return Pedido negado com ::FLIGHT_REASON_NOT_IMPLEMENTED.
 */
static flight_request_result_t rejected(void) {
  const flight_request_result_t result = {
      .status = FLIGHT_REQUEST_DENIED,
      .reason = FLIGHT_REASON_NOT_IMPLEMENTED,
  };
  return result;
}

/** @brief Provisória: aceita desarmar e nega armar (ver flight_command_api.h). */
FLIGHT_WEAK flight_request_result_t flight_request_arm(bool arm) {
  if (!arm) {
    const flight_request_result_t result = {
        .status = FLIGHT_REQUEST_ACCEPTED,
        .reason = FLIGHT_REASON_NONE,
    };
    return result;
  }
  return rejected();
}

/** @brief Provisória: nega (ver flight_command_api.h). */
FLIGHT_WEAK flight_request_result_t flight_request_leg(const flight_leg_t *leg) {
  (void)leg;
  return rejected();
}

/** @brief Provisória: nega (ver flight_command_api.h). */
FLIGHT_WEAK flight_request_result_t flight_request_cancel(void) {
  return rejected();
}

/** @brief Provisória: nega (ver flight_command_api.h). */
FLIGHT_WEAK flight_request_result_t flight_request_hold(flight_reason_t reason) {
  (void)reason;
  return rejected();
}

/** @brief Provisória: nega (ver flight_command_api.h). */
FLIGHT_WEAK flight_request_result_t flight_request_land(flight_reason_t reason) {
  (void)reason;
  return rejected();
}

/** @brief Provisória: estado STANDBY, nada válido (ver flight_command_api.h). */
FLIGHT_WEAK void flight_get_snapshot(flight_snapshot_t *snapshot) {
  if (snapshot == NULL) {
    return;
  }
  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->state = FLIGHT_STATE_STANDBY;
  snapshot->q[0] = 1.0F;
}

/** @brief Provisória: sem acionamento (ver flight_command_api.h). */
FLIGHT_WEAK bool flight_actuation_is_available(void) {
  return false;
}
