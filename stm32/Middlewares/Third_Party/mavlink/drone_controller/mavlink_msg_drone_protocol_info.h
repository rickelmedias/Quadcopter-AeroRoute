#pragma once
// MESSAGE DRONE_PROTOCOL_INFO PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO 51000


typedef struct __mavlink_drone_protocol_info_t {
 uint32_t boot_id; /*<  Monotonic STM32 boot identity. Zero is invalid for movement.*/
 uint32_t capabilities; /*<  Implemented capabilities.*/
 uint32_t max_leg_timeout_ms; /*< [ms] Largest accepted leg timeout.*/
 uint8_t protocol_major; /*<  DroneLink major version.*/
 uint8_t protocol_minor; /*<  DroneLink minor version.*/
 uint8_t flight_state; /*<  Current flight state.*/
 uint8_t target_system; /*<  Intended recipient system, zero for broadcast.*/
 uint8_t target_component; /*<  Intended recipient component, zero for broadcast.*/
} mavlink_drone_protocol_info_t;

#define MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN 17
#define MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN 17
#define MAVLINK_MSG_ID_51000_LEN 17
#define MAVLINK_MSG_ID_51000_MIN_LEN 17

#define MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_CRC 177
#define MAVLINK_MSG_ID_51000_CRC 177



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_PROTOCOL_INFO { \
    51000, \
    "DRONE_PROTOCOL_INFO", \
    8, \
    {  { "boot_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_protocol_info_t, boot_id) }, \
         { "capabilities", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_protocol_info_t, capabilities) }, \
         { "max_leg_timeout_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_protocol_info_t, max_leg_timeout_ms) }, \
         { "protocol_major", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_drone_protocol_info_t, protocol_major) }, \
         { "protocol_minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_drone_protocol_info_t, protocol_minor) }, \
         { "flight_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_drone_protocol_info_t, flight_state) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_drone_protocol_info_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_drone_protocol_info_t, target_component) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_PROTOCOL_INFO { \
    "DRONE_PROTOCOL_INFO", \
    8, \
    {  { "boot_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_protocol_info_t, boot_id) }, \
         { "capabilities", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_protocol_info_t, capabilities) }, \
         { "max_leg_timeout_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_protocol_info_t, max_leg_timeout_ms) }, \
         { "protocol_major", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_drone_protocol_info_t, protocol_major) }, \
         { "protocol_minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_drone_protocol_info_t, protocol_minor) }, \
         { "flight_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_drone_protocol_info_t, flight_state) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_drone_protocol_info_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_drone_protocol_info_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_protocol_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param boot_id  Monotonic STM32 boot identity. Zero is invalid for movement.
 * @param capabilities  Implemented capabilities.
 * @param max_leg_timeout_ms [ms] Largest accepted leg timeout.
 * @param protocol_major  DroneLink major version.
 * @param protocol_minor  DroneLink minor version.
 * @param flight_state  Current flight state.
 * @param target_system  Intended recipient system, zero for broadcast.
 * @param target_component  Intended recipient component, zero for broadcast.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_protocol_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t boot_id, uint32_t capabilities, uint32_t max_leg_timeout_ms, uint8_t protocol_major, uint8_t protocol_minor, uint8_t flight_state, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, capabilities);
    _mav_put_uint32_t(buf, 8, max_leg_timeout_ms);
    _mav_put_uint8_t(buf, 12, protocol_major);
    _mav_put_uint8_t(buf, 13, protocol_minor);
    _mav_put_uint8_t(buf, 14, flight_state);
    _mav_put_uint8_t(buf, 15, target_system);
    _mav_put_uint8_t(buf, 16, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN);
#else
    mavlink_drone_protocol_info_t packet;
    packet.boot_id = boot_id;
    packet.capabilities = capabilities;
    packet.max_leg_timeout_ms = max_leg_timeout_ms;
    packet.protocol_major = protocol_major;
    packet.protocol_minor = protocol_minor;
    packet.flight_state = flight_state;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_CRC);
}

/**
 * @brief Pack a drone_protocol_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param boot_id  Monotonic STM32 boot identity. Zero is invalid for movement.
 * @param capabilities  Implemented capabilities.
 * @param max_leg_timeout_ms [ms] Largest accepted leg timeout.
 * @param protocol_major  DroneLink major version.
 * @param protocol_minor  DroneLink minor version.
 * @param flight_state  Current flight state.
 * @param target_system  Intended recipient system, zero for broadcast.
 * @param target_component  Intended recipient component, zero for broadcast.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_protocol_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t boot_id, uint32_t capabilities, uint32_t max_leg_timeout_ms, uint8_t protocol_major, uint8_t protocol_minor, uint8_t flight_state, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, capabilities);
    _mav_put_uint32_t(buf, 8, max_leg_timeout_ms);
    _mav_put_uint8_t(buf, 12, protocol_major);
    _mav_put_uint8_t(buf, 13, protocol_minor);
    _mav_put_uint8_t(buf, 14, flight_state);
    _mav_put_uint8_t(buf, 15, target_system);
    _mav_put_uint8_t(buf, 16, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN);
#else
    mavlink_drone_protocol_info_t packet;
    packet.boot_id = boot_id;
    packet.capabilities = capabilities;
    packet.max_leg_timeout_ms = max_leg_timeout_ms;
    packet.protocol_major = protocol_major;
    packet.protocol_minor = protocol_minor;
    packet.flight_state = flight_state;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN);
#endif
}

/**
 * @brief Pack a drone_protocol_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boot_id  Monotonic STM32 boot identity. Zero is invalid for movement.
 * @param capabilities  Implemented capabilities.
 * @param max_leg_timeout_ms [ms] Largest accepted leg timeout.
 * @param protocol_major  DroneLink major version.
 * @param protocol_minor  DroneLink minor version.
 * @param flight_state  Current flight state.
 * @param target_system  Intended recipient system, zero for broadcast.
 * @param target_component  Intended recipient component, zero for broadcast.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_protocol_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t boot_id,uint32_t capabilities,uint32_t max_leg_timeout_ms,uint8_t protocol_major,uint8_t protocol_minor,uint8_t flight_state,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, capabilities);
    _mav_put_uint32_t(buf, 8, max_leg_timeout_ms);
    _mav_put_uint8_t(buf, 12, protocol_major);
    _mav_put_uint8_t(buf, 13, protocol_minor);
    _mav_put_uint8_t(buf, 14, flight_state);
    _mav_put_uint8_t(buf, 15, target_system);
    _mav_put_uint8_t(buf, 16, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN);
#else
    mavlink_drone_protocol_info_t packet;
    packet.boot_id = boot_id;
    packet.capabilities = capabilities;
    packet.max_leg_timeout_ms = max_leg_timeout_ms;
    packet.protocol_major = protocol_major;
    packet.protocol_minor = protocol_minor;
    packet.flight_state = flight_state;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_CRC);
}

/**
 * @brief Encode a drone_protocol_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_protocol_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_protocol_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_protocol_info_t* drone_protocol_info)
{
    return mavlink_msg_drone_protocol_info_pack(system_id, component_id, msg, drone_protocol_info->boot_id, drone_protocol_info->capabilities, drone_protocol_info->max_leg_timeout_ms, drone_protocol_info->protocol_major, drone_protocol_info->protocol_minor, drone_protocol_info->flight_state, drone_protocol_info->target_system, drone_protocol_info->target_component);
}

/**
 * @brief Encode a drone_protocol_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_protocol_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_protocol_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_protocol_info_t* drone_protocol_info)
{
    return mavlink_msg_drone_protocol_info_pack_chan(system_id, component_id, chan, msg, drone_protocol_info->boot_id, drone_protocol_info->capabilities, drone_protocol_info->max_leg_timeout_ms, drone_protocol_info->protocol_major, drone_protocol_info->protocol_minor, drone_protocol_info->flight_state, drone_protocol_info->target_system, drone_protocol_info->target_component);
}

/**
 * @brief Encode a drone_protocol_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param drone_protocol_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_protocol_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_drone_protocol_info_t* drone_protocol_info)
{
    return mavlink_msg_drone_protocol_info_pack_status(system_id, component_id, _status, msg,  drone_protocol_info->boot_id, drone_protocol_info->capabilities, drone_protocol_info->max_leg_timeout_ms, drone_protocol_info->protocol_major, drone_protocol_info->protocol_minor, drone_protocol_info->flight_state, drone_protocol_info->target_system, drone_protocol_info->target_component);
}

/**
 * @brief Send a drone_protocol_info message
 * @param chan MAVLink channel to send the message
 *
 * @param boot_id  Monotonic STM32 boot identity. Zero is invalid for movement.
 * @param capabilities  Implemented capabilities.
 * @param max_leg_timeout_ms [ms] Largest accepted leg timeout.
 * @param protocol_major  DroneLink major version.
 * @param protocol_minor  DroneLink minor version.
 * @param flight_state  Current flight state.
 * @param target_system  Intended recipient system, zero for broadcast.
 * @param target_component  Intended recipient component, zero for broadcast.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_protocol_info_send(mavlink_channel_t chan, uint32_t boot_id, uint32_t capabilities, uint32_t max_leg_timeout_ms, uint8_t protocol_major, uint8_t protocol_minor, uint8_t flight_state, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, capabilities);
    _mav_put_uint32_t(buf, 8, max_leg_timeout_ms);
    _mav_put_uint8_t(buf, 12, protocol_major);
    _mav_put_uint8_t(buf, 13, protocol_minor);
    _mav_put_uint8_t(buf, 14, flight_state);
    _mav_put_uint8_t(buf, 15, target_system);
    _mav_put_uint8_t(buf, 16, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO, buf, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_CRC);
#else
    mavlink_drone_protocol_info_t packet;
    packet.boot_id = boot_id;
    packet.capabilities = capabilities;
    packet.max_leg_timeout_ms = max_leg_timeout_ms;
    packet.protocol_major = protocol_major;
    packet.protocol_minor = protocol_minor;
    packet.flight_state = flight_state;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO, (const char *)&packet, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_CRC);
#endif
}

/**
 * @brief Send a drone_protocol_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_protocol_info_send_struct(mavlink_channel_t chan, const mavlink_drone_protocol_info_t* drone_protocol_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_protocol_info_send(chan, drone_protocol_info->boot_id, drone_protocol_info->capabilities, drone_protocol_info->max_leg_timeout_ms, drone_protocol_info->protocol_major, drone_protocol_info->protocol_minor, drone_protocol_info->flight_state, drone_protocol_info->target_system, drone_protocol_info->target_component);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO, (const char *)drone_protocol_info, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_protocol_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t boot_id, uint32_t capabilities, uint32_t max_leg_timeout_ms, uint8_t protocol_major, uint8_t protocol_minor, uint8_t flight_state, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, capabilities);
    _mav_put_uint32_t(buf, 8, max_leg_timeout_ms);
    _mav_put_uint8_t(buf, 12, protocol_major);
    _mav_put_uint8_t(buf, 13, protocol_minor);
    _mav_put_uint8_t(buf, 14, flight_state);
    _mav_put_uint8_t(buf, 15, target_system);
    _mav_put_uint8_t(buf, 16, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO, buf, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_CRC);
#else
    mavlink_drone_protocol_info_t *packet = (mavlink_drone_protocol_info_t *)msgbuf;
    packet->boot_id = boot_id;
    packet->capabilities = capabilities;
    packet->max_leg_timeout_ms = max_leg_timeout_ms;
    packet->protocol_major = protocol_major;
    packet->protocol_minor = protocol_minor;
    packet->flight_state = flight_state;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO, (const char *)packet, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_PROTOCOL_INFO UNPACKING


/**
 * @brief Get field boot_id from drone_protocol_info message
 *
 * @return  Monotonic STM32 boot identity. Zero is invalid for movement.
 */
static inline uint32_t mavlink_msg_drone_protocol_info_get_boot_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field capabilities from drone_protocol_info message
 *
 * @return  Implemented capabilities.
 */
static inline uint32_t mavlink_msg_drone_protocol_info_get_capabilities(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field max_leg_timeout_ms from drone_protocol_info message
 *
 * @return [ms] Largest accepted leg timeout.
 */
static inline uint32_t mavlink_msg_drone_protocol_info_get_max_leg_timeout_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field protocol_major from drone_protocol_info message
 *
 * @return  DroneLink major version.
 */
static inline uint8_t mavlink_msg_drone_protocol_info_get_protocol_major(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field protocol_minor from drone_protocol_info message
 *
 * @return  DroneLink minor version.
 */
static inline uint8_t mavlink_msg_drone_protocol_info_get_protocol_minor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field flight_state from drone_protocol_info message
 *
 * @return  Current flight state.
 */
static inline uint8_t mavlink_msg_drone_protocol_info_get_flight_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field target_system from drone_protocol_info message
 *
 * @return  Intended recipient system, zero for broadcast.
 */
static inline uint8_t mavlink_msg_drone_protocol_info_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field target_component from drone_protocol_info message
 *
 * @return  Intended recipient component, zero for broadcast.
 */
static inline uint8_t mavlink_msg_drone_protocol_info_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a drone_protocol_info message into a struct
 *
 * @param msg The message to decode
 * @param drone_protocol_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_protocol_info_decode(const mavlink_message_t* msg, mavlink_drone_protocol_info_t* drone_protocol_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_protocol_info->boot_id = mavlink_msg_drone_protocol_info_get_boot_id(msg);
    drone_protocol_info->capabilities = mavlink_msg_drone_protocol_info_get_capabilities(msg);
    drone_protocol_info->max_leg_timeout_ms = mavlink_msg_drone_protocol_info_get_max_leg_timeout_ms(msg);
    drone_protocol_info->protocol_major = mavlink_msg_drone_protocol_info_get_protocol_major(msg);
    drone_protocol_info->protocol_minor = mavlink_msg_drone_protocol_info_get_protocol_minor(msg);
    drone_protocol_info->flight_state = mavlink_msg_drone_protocol_info_get_flight_state(msg);
    drone_protocol_info->target_system = mavlink_msg_drone_protocol_info_get_target_system(msg);
    drone_protocol_info->target_component = mavlink_msg_drone_protocol_info_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN? msg->len : MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN;
        memset(drone_protocol_info, 0, MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_LEN);
    memcpy(drone_protocol_info, _MAV_PAYLOAD(msg), len);
#endif
}
