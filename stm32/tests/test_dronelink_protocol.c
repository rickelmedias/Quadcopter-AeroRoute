/**
 * @file test_dronelink_protocol.c
 * @brief Pilha DroneLink do STM32 com frames MAVLink 2 reais: protocolo, transações,
 *        telemetria e executor sobre o controle e a planta do FLT_SIM.
 *
 * O teste faz o papel da Raspberry Pi (sistema 1, componente 191): empacota com os
 * mesmos headers gerados, entrega a dronelink_protocol_consume() e decodifica o
 * que o firmware enfileira no transporte (trocado por uma captura). Cobre a
 * autorização para armar, trechos idempotentes, recusas (boot, sequência,
 * colisão), telemetria e a supervisão do enlace (HOLD e pouso sem a Pi).
 */
#include "dronelink/dronelink_mavlink.h"
#include "dronelink/dronelink_protocol.h"
#include "support/fake_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI_SYS  1U
#define PI_COMP 191U
#define FC_COMP 1U
#define BOOT_ID 7U
#define INBOX   64U

/** Mensagens que o firmware enviou (as últimas ::INBOX). */
static mavlink_message_t inbox[INBOX];
static uint32_t inbox_n;
static mavlink_status_t out_status;
static mavlink_message_t out_msg;

bool dronelink_transport_send(const uint8_t *data, uint16_t length,
                              dronelink_tx_priority_t priority) {
  (void)priority;
  for (uint16_t i = 0U; i < length; ++i) {
    if (mavlink_frame_char_buffer(&out_msg, &out_status, data[i], &out_msg, &out_status) ==
        MAVLINK_FRAMING_OK) {
      inbox[inbox_n % INBOX] = out_msg;
      ++inbox_n;
    }
  }
  return true;
}
uint32_t dronelink_transport_rx_dropped(void) {
  return 0U;
}
uint32_t dronelink_transport_tx_dropped(void) {
  return 0U;
}

/** @brief Entrega uma mensagem "da Pi" ao firmware. */
static void from_pi(mavlink_message_t *m) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(buf, m);
  dronelink_protocol_consume(buf, n);
}

static void pi_heartbeat(void) {
  mavlink_message_t m;
  mavlink_msg_heartbeat_pack(PI_SYS, PI_COMP, &m, MAV_TYPE_ONBOARD_CONTROLLER,
                             MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  from_pi(&m);
}

static void pi_lease(void) {
  mavlink_message_t m;
  mavlink_msg_drone_control_lease_pack(PI_SYS, PI_COMP, &m, BOOT_ID, 5U, 99U, 1000U,
                                       DRONE_LEASE_FLAG_ACTIVE, PI_SYS, FC_COMP);
  from_pi(&m);
}

static void pi_arm(bool arm) {
  mavlink_message_t m;
  mavlink_msg_command_long_pack(PI_SYS, PI_COMP, &m, PI_SYS, FC_COMP,
                                MAV_CMD_COMPONENT_ARM_DISARM, 0, arm ? 1.0F : 0.0F, 0, 0, 0, 0,
                                0, 0);
  from_pi(&m);
}

static void pi_leg(uint32_t boot, uint32_t cmd, uint16_t index, uint8_t action, uint8_t frame,
                   float x, float y, float z, float yaw) {
  mavlink_message_t m;
  mavlink_msg_drone_leg_command_pack(PI_SYS, PI_COMP, &m, boot, 5U, cmd, 60000U, x, y, z, yaw,
                                     0.0F, 0.0F, 0.0F, 0.0F, index, 0U, action, frame, PI_SYS,
                                     FC_COMP);
  from_pi(&m);
}

/** @brief Última mensagem do tipo pedido recebida depois de @p since (NULL se nenhuma). */
static const mavlink_message_t *last_of(uint32_t msgid, uint32_t since) {
  const mavlink_message_t *found = NULL;
  for (uint32_t i = since; i < inbox_n; ++i) {
    if (inbox[i % INBOX].msgid == msgid) {
      found = &inbox[i % INBOX];
    }
  }
  return found;
}

static mavlink_drone_leg_ack_t leg_ack(uint32_t since, uint32_t cmd) {
  mavlink_drone_leg_ack_t a;
  memset(&a, 0, sizeof(a));
  a.result = 255U;
  for (uint32_t i = since; i < inbox_n; ++i) {
    if (inbox[i % INBOX].msgid == MAVLINK_MSG_ID_DRONE_LEG_ACK) {
      mavlink_drone_leg_ack_t t;
      mavlink_msg_drone_leg_ack_decode(&inbox[i % INBOX], &t);
      if (t.command_id == cmd) {
        a = t;
      }
    }
  }
  return a;
}

/** @brief Avança @p ms: controle, protocolo e, se @p pi, HEARTBEAT e autorização a cada 200 ms. */
static void run(uint32_t ms, bool pi) {
  for (uint32_t t = 0U; t < ms; t += 10U) {
    if (pi && ((now_ms % 200U) == 0U)) {
      pi_heartbeat();
      pi_lease();
    }
    fake_step();
    dronelink_protocol_tick(now_ms);
  }
}

static uint8_t hb_state(uint32_t since) {
  const mavlink_message_t *m = last_of(MAVLINK_MSG_ID_HEARTBEAT, since);
  assert(m != NULL);
  return (uint8_t)mavlink_msg_heartbeat_get_custom_mode(m);
}

int main(void) {
  fake_runs_executor = false;
  lqg_ctrl_init(&ctrl);
  dronelink_protocol_init(BOOT_ID, true);
  run(1100U, true);
  const mavlink_message_t *info = last_of(MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO, 0U);
  assert(info != NULL);
  assert(mavlink_msg_drone_protocol_info_get_boot_id(info) == BOOT_ID);
  assert(info->sysid == PI_SYS && info->compid == FC_COMP);
  assert(hb_state(0U) == DRONE_FLIGHT_STATE_STANDBY);

  /* Mensagem de outra origem é ignorada. */
  uint32_t mark = inbox_n;
  mavlink_message_t m;
  mavlink_msg_command_long_pack(PI_SYS, 42U, &m, PI_SYS, FC_COMP, MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1.0F, 0, 0, 0, 0, 0, 0);
  from_pi(&m);
  assert(last_of(MAVLINK_MSG_ID_COMMAND_ACK, mark) == NULL && owner == MOTORS_OWNER_NONE);

  /* Armar sem autorização do operador: recusado. */
  dronelink_protocol_init(BOOT_ID, true);
  mark = inbox_n;
  pi_arm(true);
  const mavlink_message_t *ack = last_of(MAVLINK_MSG_ID_COMMAND_ACK, mark);
  assert(ack != NULL && mavlink_msg_command_ack_get_result(ack) == MAV_RESULT_DENIED);
  assert(mavlink_msg_command_ack_get_result_param2(ack) == DRONE_FAILURE_OPERATOR_LEASE_LOST);

  /* Com HEARTBEAT e autorização: arma e decola a 30 cm. */
  run(400U, true);
  mark = inbox_n;
  pi_arm(true);
  ack = last_of(MAVLINK_MSG_ID_COMMAND_ACK, mark);
  assert(ack != NULL && mavlink_msg_command_ack_get_result(ack) == MAV_RESULT_ACCEPTED);
  assert(owner == MOTORS_OWNER_CONTROL);
  mark = inbox_n;
  pi_leg(BOOT_ID, 1U, 1U, DRONE_ACTION_TAKEOFF_RELATIVE, MAV_FRAME_LOCAL_FRD, 0, 0, -0.30F, 0);
  assert(leg_ack(mark, 1U).result == MAV_RESULT_IN_PROGRESS);
  /* Repetido (a Pi não viu a resposta): mesma resposta, não executa de novo. */
  mark = inbox_n;
  pi_leg(BOOT_ID, 1U, 1U, DRONE_ACTION_TAKEOFF_RELATIVE, MAV_FRAME_LOCAL_FRD, 0, 0, -0.30F, 0);
  assert(leg_ack(mark, 1U).result == MAV_RESULT_IN_PROGRESS);
  /* Mesmos IDs, outro conteúdo: colisão. */
  mark = inbox_n;
  pi_leg(BOOT_ID, 1U, 1U, DRONE_ACTION_TAKEOFF_RELATIVE, MAV_FRAME_LOCAL_FRD, 0, 0, -0.50F, 0);
  const mavlink_drone_leg_ack_t col = leg_ack(mark, 1U);
  assert(col.result == MAV_RESULT_DENIED && col.reason == DRONE_FAILURE_ID_COLLISION);

  mark = inbox_n;
  bool done = false;
  bool saw_text = false;
  for (uint32_t i = 0U; (i < 200U) && !done; ++i) {
    run(100U, true);
    done = leg_ack(mark, 1U).result == MAV_RESULT_ACCEPTED;
    saw_text = saw_text || (last_of(MAVLINK_MSG_ID_STATUSTEXT, mark) != NULL);
  }
  assert(done && saw_text);
  assert(leg_ack(mark, 1U).progress == 100U);
  mark = inbox_n;
  run(600U, true); /* HEARTBEAT sai a 2 Hz */
  assert(hb_state(mark) == DRONE_FLIGHT_STATE_HOLD);
  const mavlink_message_t *pos = last_of(MAVLINK_MSG_ID_LOCAL_POSITION_NED, mark);
  assert(pos != NULL && fabsf(mavlink_msg_local_position_ned_get_z(pos) + 0.30F) < 0.06F);
  assert(last_of(MAVLINK_MSG_ID_DRONE_CTRL_DEBUG, mark) != NULL);

  /* Boot antigo e sequência errada: recusados. */
  mark = inbox_n;
  pi_leg(BOOT_ID + 1U, 2U, 2U, DRONE_ACTION_HOLD, MAV_FRAME_LOCAL_FRD, 0, 0, 0, 0);
  assert(leg_ack(mark, 2U).reason == DRONE_FAILURE_STALE_BOOT);
  mark = inbox_n;
  pi_leg(BOOT_ID, 3U, 5U, DRONE_ACTION_ROTATE_RELATIVE, MAV_FRAME_BODY_FRD, 0, 0, 0, 0.5F);
  assert(leg_ack(mark, 3U).reason == DRONE_FAILURE_SEQUENCE_ERROR);

  /* Andar 20 cm para a frente. */
  mark = inbox_n;
  pi_leg(BOOT_ID, 4U, 2U, DRONE_ACTION_TRANSLATE_RELATIVE, MAV_FRAME_BODY_FRD, 0.2F, 0, 0, 0);
  assert(leg_ack(mark, 4U).result == MAV_RESULT_IN_PROGRESS);
  done = false;
  for (uint32_t i = 0U; (i < 200U) && !done; ++i) {
    run(100U, true);
    done = leg_ack(mark, 4U).result == MAV_RESULT_ACCEPTED;
  }
  assert(done);

  /* A Pi some: HOLD em 1 s, pouso em 5 s, e o drone desarma sozinho no chão. */
  run(1500U, false);
  assert(lqg_ctrl_state(&ctrl) == LQG_STATE_HOVER || lqg_ctrl_state(&ctrl) == LQG_STATE_CLIMB);
  for (uint32_t i = 0U; (i < 300U) && (owner != MOTORS_OWNER_NONE); ++i) {
    run(100U, false);
  }
  assert(owner == MOTORS_OWNER_NONE);
  assert(-(float)sim.x[2] < 0.06F);
  puts("test_dronelink_protocol ok");
  return 0;
}
