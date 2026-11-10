#pragma once
// MESSAGE DRONE_LEG_ACK PACKING

#define MAVLINK_MSG_ID_DRONE_LEG_ACK 51002


typedef struct __mavlink_drone_leg_ack_t {
 uint32_t boot_id; /*<  STM32 boot identity echoed from the command.*/
 uint32_t mission_id; /*<  Mission identity echoed from the command.*/
 uint32_t command_id; /*<  Command identity echoed from the command.*/
 uint32_t eta_ms; /*< [ms] Estimated remaining execution time, UINT32_MAX if unknown.*/
 float remaining_distance; /*< [m] Estimated remaining distance, NaN if unknown.*/
 uint16_t leg_index; /*<  Leg index echoed from the command.*/
 uint16_t reason; /*<  Failure or rejection reason.*/
 uint8_t result; /*<  Command result.*/
 uint8_t progress; /*< [%] Progress from 0 to 100, 255 if unknown.*/
 uint8_t flight_state; /*<  Current flight state.*/
 uint8_t target_system; /*<  Intended recipient system.*/
 uint8_t target_component; /*<  Intended recipient component.*/
} mavlink_drone_leg_ack_t;

#define MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN 29
#define MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN 29
#define MAVLINK_MSG_ID_51002_LEN 29
#define MAVLINK_MSG_ID_51002_MIN_LEN 29

#define MAVLINK_MSG_ID_DRONE_LEG_ACK_CRC 114
#define MAVLINK_MSG_ID_51002_CRC 114



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_LEG_ACK { \
    51002, \
    "DRONE_LEG_ACK", \
    12, \
    {  { "boot_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_leg_ack_t, boot_id) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_leg_ack_t, mission_id) }, \
         { "command_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_leg_ack_t, command_id) }, \
         { "eta_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_drone_leg_ack_t, eta_ms) }, \
         { "remaining_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_drone_leg_ack_t, remaining_distance) }, \
         { "leg_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_drone_leg_ack_t, leg_index) }, \
         { "reason", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_drone_leg_ack_t, reason) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_drone_leg_ack_t, result) }, \
         { "progress", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_drone_leg_ack_t, progress) }, \
         { "flight_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_drone_leg_ack_t, flight_state) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_drone_leg_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_drone_leg_ack_t, target_component) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_LEG_ACK { \
    "DRONE_LEG_ACK", \
    12, \
    {  { "boot_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_leg_ack_t, boot_id) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_leg_ack_t, mission_id) }, \
         { "command_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_leg_ack_t, command_id) }, \
         { "eta_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_drone_leg_ack_t, eta_ms) }, \
         { "remaining_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_drone_leg_ack_t, remaining_distance) }, \
         { "leg_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_drone_leg_ack_t, leg_index) }, \
         { "reason", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_drone_leg_ack_t, reason) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_drone_leg_ack_t, result) }, \
         { "progress", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_drone_leg_ack_t, progress) }, \
         { "flight_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_drone_leg_ack_t, flight_state) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_drone_leg_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_drone_leg_ack_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_leg_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param boot_id  STM32 boot identity echoed from the command.
 * @param mission_id  Mission identity echoed from the command.
 * @param command_id  Command identity echoed from the command.
 * @param eta_ms [ms] Estimated remaining execution time, UINT32_MAX if unknown.
 * @param remaining_distance [m] Estimated remaining distance, NaN if unknown.
 * @param leg_index  Leg index echoed from the command.
 * @param reason  Failure or rejection reason.
 * @param result  Command result.
 * @param progress [%] Progress from 0 to 100, 255 if unknown.
 * @param flight_state  Current flight state.
 * @param target_system  Intended recipient system.
 * @param target_component  Intended recipient component.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_leg_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint32_t eta_ms, float remaining_distance, uint16_t leg_index, uint16_t reason, uint8_t result, uint8_t progress, uint8_t flight_state, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint32_t(buf, 12, eta_ms);
    _mav_put_float(buf, 16, remaining_distance);
    _mav_put_uint16_t(buf, 20, leg_index);
    _mav_put_uint16_t(buf, 22, reason);
    _mav_put_uint8_t(buf, 24, result);
    _mav_put_uint8_t(buf, 25, progress);
    _mav_put_uint8_t(buf, 26, flight_state);
    _mav_put_uint8_t(buf, 27, target_system);
    _mav_put_uint8_t(buf, 28, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN);
#else
    mavlink_drone_leg_ack_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.eta_ms = eta_ms;
    packet.remaining_distance = remaining_distance;
    packet.leg_index = leg_index;
    packet.reason = reason;
    packet.result = result;
    packet.progress = progress;
    packet.flight_state = flight_state;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_LEG_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_CRC);
}

/**
 * @brief Pack a drone_leg_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param boot_id  STM32 boot identity echoed from the command.
 * @param mission_id  Mission identity echoed from the command.
 * @param command_id  Command identity echoed from the command.
 * @param eta_ms [ms] Estimated remaining execution time, UINT32_MAX if unknown.
 * @param remaining_distance [m] Estimated remaining distance, NaN if unknown.
 * @param leg_index  Leg index echoed from the command.
 * @param reason  Failure or rejection reason.
 * @param result  Command result.
 * @param progress [%] Progress from 0 to 100, 255 if unknown.
 * @param flight_state  Current flight state.
 * @param target_system  Intended recipient system.
 * @param target_component  Intended recipient component.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_leg_ack_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint32_t eta_ms, float remaining_distance, uint16_t leg_index, uint16_t reason, uint8_t result, uint8_t progress, uint8_t flight_state, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint32_t(buf, 12, eta_ms);
    _mav_put_float(buf, 16, remaining_distance);
    _mav_put_uint16_t(buf, 20, leg_index);
    _mav_put_uint16_t(buf, 22, reason);
    _mav_put_uint8_t(buf, 24, result);
    _mav_put_uint8_t(buf, 25, progress);
    _mav_put_uint8_t(buf, 26, flight_state);
    _mav_put_uint8_t(buf, 27, target_system);
    _mav_put_uint8_t(buf, 28, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN);
#else
    mavlink_drone_leg_ack_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.eta_ms = eta_ms;
    packet.remaining_distance = remaining_distance;
    packet.leg_index = leg_index;
    packet.reason = reason;
    packet.result = result;
    packet.progress = progress;
    packet.flight_state = flight_state;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_LEG_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN);
#endif
}

/**
 * @brief Pack a drone_leg_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boot_id  STM32 boot identity echoed from the command.
 * @param mission_id  Mission identity echoed from the command.
 * @param command_id  Command identity echoed from the command.
 * @param eta_ms [ms] Estimated remaining execution time, UINT32_MAX if unknown.
 * @param remaining_distance [m] Estimated remaining distance, NaN if unknown.
 * @param leg_index  Leg index echoed from the command.
 * @param reason  Failure or rejection reason.
 * @param result  Command result.
 * @param progress [%] Progress from 0 to 100, 255 if unknown.
 * @param flight_state  Current flight state.
 * @param target_system  Intended recipient system.
 * @param target_component  Intended recipient component.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_leg_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t boot_id,uint32_t mission_id,uint32_t command_id,uint32_t eta_ms,float remaining_distance,uint16_t leg_index,uint16_t reason,uint8_t result,uint8_t progress,uint8_t flight_state,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint32_t(buf, 12, eta_ms);
    _mav_put_float(buf, 16, remaining_distance);
    _mav_put_uint16_t(buf, 20, leg_index);
    _mav_put_uint16_t(buf, 22, reason);
    _mav_put_uint8_t(buf, 24, result);
    _mav_put_uint8_t(buf, 25, progress);
    _mav_put_uint8_t(buf, 26, flight_state);
    _mav_put_uint8_t(buf, 27, target_system);
    _mav_put_uint8_t(buf, 28, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN);
#else
    mavlink_drone_leg_ack_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.eta_ms = eta_ms;
    packet.remaining_distance = remaining_distance;
    packet.leg_index = leg_index;
    packet.reason = reason;
    packet.result = result;
    packet.progress = progress;
    packet.flight_state = flight_state;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_LEG_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_CRC);
}

/**
 * @brief Encode a drone_leg_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_leg_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_leg_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_leg_ack_t* drone_leg_ack)
{
    return mavlink_msg_drone_leg_ack_pack(system_id, component_id, msg, drone_leg_ack->boot_id, drone_leg_ack->mission_id, drone_leg_ack->command_id, drone_leg_ack->eta_ms, drone_leg_ack->remaining_distance, drone_leg_ack->leg_index, drone_leg_ack->reason, drone_leg_ack->result, drone_leg_ack->progress, drone_leg_ack->flight_state, drone_leg_ack->target_system, drone_leg_ack->target_component);
}

/**
 * @brief Encode a drone_leg_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_leg_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_leg_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_leg_ack_t* drone_leg_ack)
{
    return mavlink_msg_drone_leg_ack_pack_chan(system_id, component_id, chan, msg, drone_leg_ack->boot_id, drone_leg_ack->mission_id, drone_leg_ack->command_id, drone_leg_ack->eta_ms, drone_leg_ack->remaining_distance, drone_leg_ack->leg_index, drone_leg_ack->reason, drone_leg_ack->result, drone_leg_ack->progress, drone_leg_ack->flight_state, drone_leg_ack->target_system, drone_leg_ack->target_component);
}

/**
 * @brief Encode a drone_leg_ack struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param drone_leg_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_leg_ack_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_drone_leg_ack_t* drone_leg_ack)
{
    return mavlink_msg_drone_leg_ack_pack_status(system_id, component_id, _status, msg,  drone_leg_ack->boot_id, drone_leg_ack->mission_id, drone_leg_ack->command_id, drone_leg_ack->eta_ms, drone_leg_ack->remaining_distance, drone_leg_ack->leg_index, drone_leg_ack->reason, drone_leg_ack->result, drone_leg_ack->progress, drone_leg_ack->flight_state, drone_leg_ack->target_system, drone_leg_ack->target_component);
}

/**
 * @brief Send a drone_leg_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param boot_id  STM32 boot identity echoed from the command.
 * @param mission_id  Mission identity echoed from the command.
 * @param command_id  Command identity echoed from the command.
 * @param eta_ms [ms] Estimated remaining execution time, UINT32_MAX if unknown.
 * @param remaining_distance [m] Estimated remaining distance, NaN if unknown.
 * @param leg_index  Leg index echoed from the command.
 * @param reason  Failure or rejection reason.
 * @param result  Command result.
 * @param progress [%] Progress from 0 to 100, 255 if unknown.
 * @param flight_state  Current flight state.
 * @param target_system  Intended recipient system.
 * @param target_component  Intended recipient component.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_leg_ack_send(mavlink_channel_t chan, uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint32_t eta_ms, float remaining_distance, uint16_t leg_index, uint16_t reason, uint8_t result, uint8_t progress, uint8_t flight_state, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint32_t(buf, 12, eta_ms);
    _mav_put_float(buf, 16, remaining_distance);
    _mav_put_uint16_t(buf, 20, leg_index);
    _mav_put_uint16_t(buf, 22, reason);
    _mav_put_uint8_t(buf, 24, result);
    _mav_put_uint8_t(buf, 25, progress);
    _mav_put_uint8_t(buf, 26, flight_state);
    _mav_put_uint8_t(buf, 27, target_system);
    _mav_put_uint8_t(buf, 28, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_ACK, buf, MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_CRC);
#else
    mavlink_drone_leg_ack_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.eta_ms = eta_ms;
    packet.remaining_distance = remaining_distance;
    packet.leg_index = leg_index;
    packet.reason = reason;
    packet.result = result;
    packet.progress = progress;
    packet.flight_state = flight_state;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_ACK, (const char *)&packet, MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_CRC);
#endif
}

/**
 * @brief Send a drone_leg_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_leg_ack_send_struct(mavlink_channel_t chan, const mavlink_drone_leg_ack_t* drone_leg_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_leg_ack_send(chan, drone_leg_ack->boot_id, drone_leg_ack->mission_id, drone_leg_ack->command_id, drone_leg_ack->eta_ms, drone_leg_ack->remaining_distance, drone_leg_ack->leg_index, drone_leg_ack->reason, drone_leg_ack->result, drone_leg_ack->progress, drone_leg_ack->flight_state, drone_leg_ack->target_system, drone_leg_ack->target_component);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_ACK, (const char *)drone_leg_ack, MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_leg_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint32_t eta_ms, float remaining_distance, uint16_t leg_index, uint16_t reason, uint8_t result, uint8_t progress, uint8_t flight_state, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint32_t(buf, 12, eta_ms);
    _mav_put_float(buf, 16, remaining_distance);
    _mav_put_uint16_t(buf, 20, leg_index);
    _mav_put_uint16_t(buf, 22, reason);
    _mav_put_uint8_t(buf, 24, result);
    _mav_put_uint8_t(buf, 25, progress);
    _mav_put_uint8_t(buf, 26, flight_state);
    _mav_put_uint8_t(buf, 27, target_system);
    _mav_put_uint8_t(buf, 28, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_ACK, buf, MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_CRC);
#else
    mavlink_drone_leg_ack_t *packet = (mavlink_drone_leg_ack_t *)msgbuf;
    packet->boot_id = boot_id;
    packet->mission_id = mission_id;
    packet->command_id = command_id;
    packet->eta_ms = eta_ms;
    packet->remaining_distance = remaining_distance;
    packet->leg_index = leg_index;
    packet->reason = reason;
    packet->result = result;
    packet->progress = progress;
    packet->flight_state = flight_state;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_ACK, (const char *)packet, MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN, MAVLINK_MSG_ID_DRONE_LEG_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_LEG_ACK UNPACKING


/**
 * @brief Get field boot_id from drone_leg_ack message
 *
 * @return  STM32 boot identity echoed from the command.
 */
static inline uint32_t mavlink_msg_drone_leg_ack_get_boot_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field mission_id from drone_leg_ack message
 *
 * @return  Mission identity echoed from the command.
 */
static inline uint32_t mavlink_msg_drone_leg_ack_get_mission_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field command_id from drone_leg_ack message
 *
 * @return  Command identity echoed from the command.
 */
static inline uint32_t mavlink_msg_drone_leg_ack_get_command_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field eta_ms from drone_leg_ack message
 *
 * @return [ms] Estimated remaining execution time, UINT32_MAX if unknown.
 */
static inline uint32_t mavlink_msg_drone_leg_ack_get_eta_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field remaining_distance from drone_leg_ack message
 *
 * @return [m] Estimated remaining distance, NaN if unknown.
 */
static inline float mavlink_msg_drone_leg_ack_get_remaining_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field leg_index from drone_leg_ack message
 *
 * @return  Leg index echoed from the command.
 */
static inline uint16_t mavlink_msg_drone_leg_ack_get_leg_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field reason from drone_leg_ack message
 *
 * @return  Failure or rejection reason.
 */
static inline uint16_t mavlink_msg_drone_leg_ack_get_reason(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field result from drone_leg_ack message
 *
 * @return  Command result.
 */
static inline uint8_t mavlink_msg_drone_leg_ack_get_result(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field progress from drone_leg_ack message
 *
 * @return [%] Progress from 0 to 100, 255 if unknown.
 */
static inline uint8_t mavlink_msg_drone_leg_ack_get_progress(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field flight_state from drone_leg_ack message
 *
 * @return  Current flight state.
 */
static inline uint8_t mavlink_msg_drone_leg_ack_get_flight_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field target_system from drone_leg_ack message
 *
 * @return  Intended recipient system.
 */
static inline uint8_t mavlink_msg_drone_leg_ack_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field target_component from drone_leg_ack message
 *
 * @return  Intended recipient component.
 */
static inline uint8_t mavlink_msg_drone_leg_ack_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Decode a drone_leg_ack message into a struct
 *
 * @param msg The message to decode
 * @param drone_leg_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_leg_ack_decode(const mavlink_message_t* msg, mavlink_drone_leg_ack_t* drone_leg_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_leg_ack->boot_id = mavlink_msg_drone_leg_ack_get_boot_id(msg);
    drone_leg_ack->mission_id = mavlink_msg_drone_leg_ack_get_mission_id(msg);
    drone_leg_ack->command_id = mavlink_msg_drone_leg_ack_get_command_id(msg);
    drone_leg_ack->eta_ms = mavlink_msg_drone_leg_ack_get_eta_ms(msg);
    drone_leg_ack->remaining_distance = mavlink_msg_drone_leg_ack_get_remaining_distance(msg);
    drone_leg_ack->leg_index = mavlink_msg_drone_leg_ack_get_leg_index(msg);
    drone_leg_ack->reason = mavlink_msg_drone_leg_ack_get_reason(msg);
    drone_leg_ack->result = mavlink_msg_drone_leg_ack_get_result(msg);
    drone_leg_ack->progress = mavlink_msg_drone_leg_ack_get_progress(msg);
    drone_leg_ack->flight_state = mavlink_msg_drone_leg_ack_get_flight_state(msg);
    drone_leg_ack->target_system = mavlink_msg_drone_leg_ack_get_target_system(msg);
    drone_leg_ack->target_component = mavlink_msg_drone_leg_ack_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN? msg->len : MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN;
        memset(drone_leg_ack, 0, MAVLINK_MSG_ID_DRONE_LEG_ACK_LEN);
    memcpy(drone_leg_ack, _MAV_PAYLOAD(msg), len);
#endif
}
