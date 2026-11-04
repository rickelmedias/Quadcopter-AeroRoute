#pragma once
// MESSAGE DRONE_LEG_CANCEL PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_DRONE_LEG_CANCEL 51003


typedef struct __mavlink_drone_leg_cancel_t {
 uint32_t boot_id; /*<  Expected STM32 boot identity.*/
 uint32_t mission_id; /*<  Mission identity.*/
 uint32_t command_id; /*<  Command identity being cancelled.*/
 uint16_t leg_index; /*<  Leg index being cancelled.*/
 uint8_t target_system; /*<  System that must process the cancellation.*/
 uint8_t target_component; /*<  Component that must process the cancellation.*/
} mavlink_drone_leg_cancel_t;

#define MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN 16
#define MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN 16
#define MAVLINK_MSG_ID_51003_LEN 16
#define MAVLINK_MSG_ID_51003_MIN_LEN 16

#define MAVLINK_MSG_ID_DRONE_LEG_CANCEL_CRC 39
#define MAVLINK_MSG_ID_51003_CRC 39



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_LEG_CANCEL { \
    51003, \
    "DRONE_LEG_CANCEL", \
    6, \
    {  { "boot_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_leg_cancel_t, boot_id) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_leg_cancel_t, mission_id) }, \
         { "command_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_leg_cancel_t, command_id) }, \
         { "leg_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_drone_leg_cancel_t, leg_index) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_drone_leg_cancel_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_drone_leg_cancel_t, target_component) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_LEG_CANCEL { \
    "DRONE_LEG_CANCEL", \
    6, \
    {  { "boot_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_leg_cancel_t, boot_id) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_leg_cancel_t, mission_id) }, \
         { "command_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_leg_cancel_t, command_id) }, \
         { "leg_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_drone_leg_cancel_t, leg_index) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_drone_leg_cancel_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_drone_leg_cancel_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_leg_cancel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param boot_id  Expected STM32 boot identity.
 * @param mission_id  Mission identity.
 * @param command_id  Command identity being cancelled.
 * @param leg_index  Leg index being cancelled.
 * @param target_system  System that must process the cancellation.
 * @param target_component  Component that must process the cancellation.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_leg_cancel_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint16_t leg_index, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint16_t(buf, 12, leg_index);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN);
#else
    mavlink_drone_leg_cancel_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.leg_index = leg_index;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_LEG_CANCEL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_CRC);
}

/**
 * @brief Pack a drone_leg_cancel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param boot_id  Expected STM32 boot identity.
 * @param mission_id  Mission identity.
 * @param command_id  Command identity being cancelled.
 * @param leg_index  Leg index being cancelled.
 * @param target_system  System that must process the cancellation.
 * @param target_component  Component that must process the cancellation.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_leg_cancel_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint16_t leg_index, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint16_t(buf, 12, leg_index);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN);
#else
    mavlink_drone_leg_cancel_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.leg_index = leg_index;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_LEG_CANCEL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN);
#endif
}

/**
 * @brief Pack a drone_leg_cancel message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boot_id  Expected STM32 boot identity.
 * @param mission_id  Mission identity.
 * @param command_id  Command identity being cancelled.
 * @param leg_index  Leg index being cancelled.
 * @param target_system  System that must process the cancellation.
 * @param target_component  Component that must process the cancellation.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_leg_cancel_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t boot_id,uint32_t mission_id,uint32_t command_id,uint16_t leg_index,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint16_t(buf, 12, leg_index);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN);
#else
    mavlink_drone_leg_cancel_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.leg_index = leg_index;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_LEG_CANCEL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_CRC);
}

/**
 * @brief Encode a drone_leg_cancel struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_leg_cancel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_leg_cancel_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_leg_cancel_t* drone_leg_cancel)
{
    return mavlink_msg_drone_leg_cancel_pack(system_id, component_id, msg, drone_leg_cancel->boot_id, drone_leg_cancel->mission_id, drone_leg_cancel->command_id, drone_leg_cancel->leg_index, drone_leg_cancel->target_system, drone_leg_cancel->target_component);
}

/**
 * @brief Encode a drone_leg_cancel struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_leg_cancel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_leg_cancel_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_leg_cancel_t* drone_leg_cancel)
{
    return mavlink_msg_drone_leg_cancel_pack_chan(system_id, component_id, chan, msg, drone_leg_cancel->boot_id, drone_leg_cancel->mission_id, drone_leg_cancel->command_id, drone_leg_cancel->leg_index, drone_leg_cancel->target_system, drone_leg_cancel->target_component);
}

/**
 * @brief Encode a drone_leg_cancel struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param drone_leg_cancel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_leg_cancel_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_drone_leg_cancel_t* drone_leg_cancel)
{
    return mavlink_msg_drone_leg_cancel_pack_status(system_id, component_id, _status, msg,  drone_leg_cancel->boot_id, drone_leg_cancel->mission_id, drone_leg_cancel->command_id, drone_leg_cancel->leg_index, drone_leg_cancel->target_system, drone_leg_cancel->target_component);
}

/**
 * @brief Send a drone_leg_cancel message
 * @param chan MAVLink channel to send the message
 *
 * @param boot_id  Expected STM32 boot identity.
 * @param mission_id  Mission identity.
 * @param command_id  Command identity being cancelled.
 * @param leg_index  Leg index being cancelled.
 * @param target_system  System that must process the cancellation.
 * @param target_component  Component that must process the cancellation.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_leg_cancel_send(mavlink_channel_t chan, uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint16_t leg_index, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint16_t(buf, 12, leg_index);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_CANCEL, buf, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_CRC);
#else
    mavlink_drone_leg_cancel_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.leg_index = leg_index;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_CANCEL, (const char *)&packet, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_CRC);
#endif
}

/**
 * @brief Send a drone_leg_cancel message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_leg_cancel_send_struct(mavlink_channel_t chan, const mavlink_drone_leg_cancel_t* drone_leg_cancel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_leg_cancel_send(chan, drone_leg_cancel->boot_id, drone_leg_cancel->mission_id, drone_leg_cancel->command_id, drone_leg_cancel->leg_index, drone_leg_cancel->target_system, drone_leg_cancel->target_component);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_CANCEL, (const char *)drone_leg_cancel, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_leg_cancel_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint16_t leg_index, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint16_t(buf, 12, leg_index);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_CANCEL, buf, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_CRC);
#else
    mavlink_drone_leg_cancel_t *packet = (mavlink_drone_leg_cancel_t *)msgbuf;
    packet->boot_id = boot_id;
    packet->mission_id = mission_id;
    packet->command_id = command_id;
    packet->leg_index = leg_index;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_CANCEL, (const char *)packet, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_LEG_CANCEL UNPACKING


/**
 * @brief Get field boot_id from drone_leg_cancel message
 *
 * @return  Expected STM32 boot identity.
 */
static inline uint32_t mavlink_msg_drone_leg_cancel_get_boot_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field mission_id from drone_leg_cancel message
 *
 * @return  Mission identity.
 */
static inline uint32_t mavlink_msg_drone_leg_cancel_get_mission_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field command_id from drone_leg_cancel message
 *
 * @return  Command identity being cancelled.
 */
static inline uint32_t mavlink_msg_drone_leg_cancel_get_command_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field leg_index from drone_leg_cancel message
 *
 * @return  Leg index being cancelled.
 */
static inline uint16_t mavlink_msg_drone_leg_cancel_get_leg_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field target_system from drone_leg_cancel message
 *
 * @return  System that must process the cancellation.
 */
static inline uint8_t mavlink_msg_drone_leg_cancel_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field target_component from drone_leg_cancel message
 *
 * @return  Component that must process the cancellation.
 */
static inline uint8_t mavlink_msg_drone_leg_cancel_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Decode a drone_leg_cancel message into a struct
 *
 * @param msg The message to decode
 * @param drone_leg_cancel C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_leg_cancel_decode(const mavlink_message_t* msg, mavlink_drone_leg_cancel_t* drone_leg_cancel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_leg_cancel->boot_id = mavlink_msg_drone_leg_cancel_get_boot_id(msg);
    drone_leg_cancel->mission_id = mavlink_msg_drone_leg_cancel_get_mission_id(msg);
    drone_leg_cancel->command_id = mavlink_msg_drone_leg_cancel_get_command_id(msg);
    drone_leg_cancel->leg_index = mavlink_msg_drone_leg_cancel_get_leg_index(msg);
    drone_leg_cancel->target_system = mavlink_msg_drone_leg_cancel_get_target_system(msg);
    drone_leg_cancel->target_component = mavlink_msg_drone_leg_cancel_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN? msg->len : MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN;
        memset(drone_leg_cancel, 0, MAVLINK_MSG_ID_DRONE_LEG_CANCEL_LEN);
    memcpy(drone_leg_cancel, _MAV_PAYLOAD(msg), len);
#endif
}
