#pragma once
// MESSAGE DRONE_GAIN_ACK PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_DRONE_GAIN_ACK 51012


typedef struct __mavlink_drone_gain_ack_t {
 uint32_t session_id; /*<  Session echoed from the request.*/
 uint32_t staged_crc32; /*<  CRC-32 computed over the staged values.*/
 uint32_t active_crc32; /*<  CRC-32 of the gains in use after the operation.*/
 uint32_t rows_mask; /*<  Staged rows, bit i = row i in the order Kd, Kfd, L, M.*/
 uint8_t action; /*<  Action echoed from the request.*/
 uint8_t status; /*<  Outcome.*/
 uint8_t custom; /*<  1 if the active gains are not the Simulink defaults.*/
 uint8_t target_system; /*<  Intended recipient system.*/
 uint8_t target_component; /*<  Intended recipient component.*/
} mavlink_drone_gain_ack_t;

#define MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN 21
#define MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN 21
#define MAVLINK_MSG_ID_51012_LEN 21
#define MAVLINK_MSG_ID_51012_MIN_LEN 21

#define MAVLINK_MSG_ID_DRONE_GAIN_ACK_CRC 195
#define MAVLINK_MSG_ID_51012_CRC 195



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_GAIN_ACK { \
    51012, \
    "DRONE_GAIN_ACK", \
    9, \
    {  { "session_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_gain_ack_t, session_id) }, \
         { "staged_crc32", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_gain_ack_t, staged_crc32) }, \
         { "active_crc32", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_gain_ack_t, active_crc32) }, \
         { "rows_mask", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_drone_gain_ack_t, rows_mask) }, \
         { "action", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_drone_gain_ack_t, action) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_drone_gain_ack_t, status) }, \
         { "custom", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_drone_gain_ack_t, custom) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_drone_gain_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_drone_gain_ack_t, target_component) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_GAIN_ACK { \
    "DRONE_GAIN_ACK", \
    9, \
    {  { "session_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_gain_ack_t, session_id) }, \
         { "staged_crc32", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_gain_ack_t, staged_crc32) }, \
         { "active_crc32", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_gain_ack_t, active_crc32) }, \
         { "rows_mask", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_drone_gain_ack_t, rows_mask) }, \
         { "action", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_drone_gain_ack_t, action) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_drone_gain_ack_t, status) }, \
         { "custom", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_drone_gain_ack_t, custom) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_drone_gain_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_drone_gain_ack_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_gain_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param session_id  Session echoed from the request.
 * @param staged_crc32  CRC-32 computed over the staged values.
 * @param active_crc32  CRC-32 of the gains in use after the operation.
 * @param rows_mask  Staged rows, bit i = row i in the order Kd, Kfd, L, M.
 * @param action  Action echoed from the request.
 * @param status  Outcome.
 * @param custom  1 if the active gains are not the Simulink defaults.
 * @param target_system  Intended recipient system.
 * @param target_component  Intended recipient component.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_gain_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t session_id, uint32_t staged_crc32, uint32_t active_crc32, uint32_t rows_mask, uint8_t action, uint8_t status, uint8_t custom, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint32_t(buf, 4, staged_crc32);
    _mav_put_uint32_t(buf, 8, active_crc32);
    _mav_put_uint32_t(buf, 12, rows_mask);
    _mav_put_uint8_t(buf, 16, action);
    _mav_put_uint8_t(buf, 17, status);
    _mav_put_uint8_t(buf, 18, custom);
    _mav_put_uint8_t(buf, 19, target_system);
    _mav_put_uint8_t(buf, 20, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN);
#else
    mavlink_drone_gain_ack_t packet;
    packet.session_id = session_id;
    packet.staged_crc32 = staged_crc32;
    packet.active_crc32 = active_crc32;
    packet.rows_mask = rows_mask;
    packet.action = action;
    packet.status = status;
    packet.custom = custom;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_GAIN_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_CRC);
}

/**
 * @brief Pack a drone_gain_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param session_id  Session echoed from the request.
 * @param staged_crc32  CRC-32 computed over the staged values.
 * @param active_crc32  CRC-32 of the gains in use after the operation.
 * @param rows_mask  Staged rows, bit i = row i in the order Kd, Kfd, L, M.
 * @param action  Action echoed from the request.
 * @param status  Outcome.
 * @param custom  1 if the active gains are not the Simulink defaults.
 * @param target_system  Intended recipient system.
 * @param target_component  Intended recipient component.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_gain_ack_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t session_id, uint32_t staged_crc32, uint32_t active_crc32, uint32_t rows_mask, uint8_t action, uint8_t status, uint8_t custom, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint32_t(buf, 4, staged_crc32);
    _mav_put_uint32_t(buf, 8, active_crc32);
    _mav_put_uint32_t(buf, 12, rows_mask);
    _mav_put_uint8_t(buf, 16, action);
    _mav_put_uint8_t(buf, 17, status);
    _mav_put_uint8_t(buf, 18, custom);
    _mav_put_uint8_t(buf, 19, target_system);
    _mav_put_uint8_t(buf, 20, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN);
#else
    mavlink_drone_gain_ack_t packet;
    packet.session_id = session_id;
    packet.staged_crc32 = staged_crc32;
    packet.active_crc32 = active_crc32;
    packet.rows_mask = rows_mask;
    packet.action = action;
    packet.status = status;
    packet.custom = custom;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_GAIN_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN);
#endif
}

/**
 * @brief Pack a drone_gain_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param session_id  Session echoed from the request.
 * @param staged_crc32  CRC-32 computed over the staged values.
 * @param active_crc32  CRC-32 of the gains in use after the operation.
 * @param rows_mask  Staged rows, bit i = row i in the order Kd, Kfd, L, M.
 * @param action  Action echoed from the request.
 * @param status  Outcome.
 * @param custom  1 if the active gains are not the Simulink defaults.
 * @param target_system  Intended recipient system.
 * @param target_component  Intended recipient component.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_gain_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t session_id,uint32_t staged_crc32,uint32_t active_crc32,uint32_t rows_mask,uint8_t action,uint8_t status,uint8_t custom,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint32_t(buf, 4, staged_crc32);
    _mav_put_uint32_t(buf, 8, active_crc32);
    _mav_put_uint32_t(buf, 12, rows_mask);
    _mav_put_uint8_t(buf, 16, action);
    _mav_put_uint8_t(buf, 17, status);
    _mav_put_uint8_t(buf, 18, custom);
    _mav_put_uint8_t(buf, 19, target_system);
    _mav_put_uint8_t(buf, 20, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN);
#else
    mavlink_drone_gain_ack_t packet;
    packet.session_id = session_id;
    packet.staged_crc32 = staged_crc32;
    packet.active_crc32 = active_crc32;
    packet.rows_mask = rows_mask;
    packet.action = action;
    packet.status = status;
    packet.custom = custom;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_GAIN_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_CRC);
}

/**
 * @brief Encode a drone_gain_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_gain_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_gain_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_gain_ack_t* drone_gain_ack)
{
    return mavlink_msg_drone_gain_ack_pack(system_id, component_id, msg, drone_gain_ack->session_id, drone_gain_ack->staged_crc32, drone_gain_ack->active_crc32, drone_gain_ack->rows_mask, drone_gain_ack->action, drone_gain_ack->status, drone_gain_ack->custom, drone_gain_ack->target_system, drone_gain_ack->target_component);
}

/**
 * @brief Encode a drone_gain_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_gain_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_gain_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_gain_ack_t* drone_gain_ack)
{
    return mavlink_msg_drone_gain_ack_pack_chan(system_id, component_id, chan, msg, drone_gain_ack->session_id, drone_gain_ack->staged_crc32, drone_gain_ack->active_crc32, drone_gain_ack->rows_mask, drone_gain_ack->action, drone_gain_ack->status, drone_gain_ack->custom, drone_gain_ack->target_system, drone_gain_ack->target_component);
}

/**
 * @brief Encode a drone_gain_ack struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param drone_gain_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_gain_ack_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_drone_gain_ack_t* drone_gain_ack)
{
    return mavlink_msg_drone_gain_ack_pack_status(system_id, component_id, _status, msg,  drone_gain_ack->session_id, drone_gain_ack->staged_crc32, drone_gain_ack->active_crc32, drone_gain_ack->rows_mask, drone_gain_ack->action, drone_gain_ack->status, drone_gain_ack->custom, drone_gain_ack->target_system, drone_gain_ack->target_component);
}

/**
 * @brief Send a drone_gain_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param session_id  Session echoed from the request.
 * @param staged_crc32  CRC-32 computed over the staged values.
 * @param active_crc32  CRC-32 of the gains in use after the operation.
 * @param rows_mask  Staged rows, bit i = row i in the order Kd, Kfd, L, M.
 * @param action  Action echoed from the request.
 * @param status  Outcome.
 * @param custom  1 if the active gains are not the Simulink defaults.
 * @param target_system  Intended recipient system.
 * @param target_component  Intended recipient component.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_gain_ack_send(mavlink_channel_t chan, uint32_t session_id, uint32_t staged_crc32, uint32_t active_crc32, uint32_t rows_mask, uint8_t action, uint8_t status, uint8_t custom, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint32_t(buf, 4, staged_crc32);
    _mav_put_uint32_t(buf, 8, active_crc32);
    _mav_put_uint32_t(buf, 12, rows_mask);
    _mav_put_uint8_t(buf, 16, action);
    _mav_put_uint8_t(buf, 17, status);
    _mav_put_uint8_t(buf, 18, custom);
    _mav_put_uint8_t(buf, 19, target_system);
    _mav_put_uint8_t(buf, 20, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_ACK, buf, MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_CRC);
#else
    mavlink_drone_gain_ack_t packet;
    packet.session_id = session_id;
    packet.staged_crc32 = staged_crc32;
    packet.active_crc32 = active_crc32;
    packet.rows_mask = rows_mask;
    packet.action = action;
    packet.status = status;
    packet.custom = custom;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_ACK, (const char *)&packet, MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_CRC);
#endif
}

/**
 * @brief Send a drone_gain_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_gain_ack_send_struct(mavlink_channel_t chan, const mavlink_drone_gain_ack_t* drone_gain_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_gain_ack_send(chan, drone_gain_ack->session_id, drone_gain_ack->staged_crc32, drone_gain_ack->active_crc32, drone_gain_ack->rows_mask, drone_gain_ack->action, drone_gain_ack->status, drone_gain_ack->custom, drone_gain_ack->target_system, drone_gain_ack->target_component);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_ACK, (const char *)drone_gain_ack, MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_gain_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t session_id, uint32_t staged_crc32, uint32_t active_crc32, uint32_t rows_mask, uint8_t action, uint8_t status, uint8_t custom, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint32_t(buf, 4, staged_crc32);
    _mav_put_uint32_t(buf, 8, active_crc32);
    _mav_put_uint32_t(buf, 12, rows_mask);
    _mav_put_uint8_t(buf, 16, action);
    _mav_put_uint8_t(buf, 17, status);
    _mav_put_uint8_t(buf, 18, custom);
    _mav_put_uint8_t(buf, 19, target_system);
    _mav_put_uint8_t(buf, 20, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_ACK, buf, MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_CRC);
#else
    mavlink_drone_gain_ack_t *packet = (mavlink_drone_gain_ack_t *)msgbuf;
    packet->session_id = session_id;
    packet->staged_crc32 = staged_crc32;
    packet->active_crc32 = active_crc32;
    packet->rows_mask = rows_mask;
    packet->action = action;
    packet->status = status;
    packet->custom = custom;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_ACK, (const char *)packet, MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_GAIN_ACK UNPACKING


/**
 * @brief Get field session_id from drone_gain_ack message
 *
 * @return  Session echoed from the request.
 */
static inline uint32_t mavlink_msg_drone_gain_ack_get_session_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field staged_crc32 from drone_gain_ack message
 *
 * @return  CRC-32 computed over the staged values.
 */
static inline uint32_t mavlink_msg_drone_gain_ack_get_staged_crc32(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field active_crc32 from drone_gain_ack message
 *
 * @return  CRC-32 of the gains in use after the operation.
 */
static inline uint32_t mavlink_msg_drone_gain_ack_get_active_crc32(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field rows_mask from drone_gain_ack message
 *
 * @return  Staged rows, bit i = row i in the order Kd, Kfd, L, M.
 */
static inline uint32_t mavlink_msg_drone_gain_ack_get_rows_mask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field action from drone_gain_ack message
 *
 * @return  Action echoed from the request.
 */
static inline uint8_t mavlink_msg_drone_gain_ack_get_action(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field status from drone_gain_ack message
 *
 * @return  Outcome.
 */
static inline uint8_t mavlink_msg_drone_gain_ack_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field custom from drone_gain_ack message
 *
 * @return  1 if the active gains are not the Simulink defaults.
 */
static inline uint8_t mavlink_msg_drone_gain_ack_get_custom(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field target_system from drone_gain_ack message
 *
 * @return  Intended recipient system.
 */
static inline uint8_t mavlink_msg_drone_gain_ack_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field target_component from drone_gain_ack message
 *
 * @return  Intended recipient component.
 */
static inline uint8_t mavlink_msg_drone_gain_ack_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a drone_gain_ack message into a struct
 *
 * @param msg The message to decode
 * @param drone_gain_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_gain_ack_decode(const mavlink_message_t* msg, mavlink_drone_gain_ack_t* drone_gain_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_gain_ack->session_id = mavlink_msg_drone_gain_ack_get_session_id(msg);
    drone_gain_ack->staged_crc32 = mavlink_msg_drone_gain_ack_get_staged_crc32(msg);
    drone_gain_ack->active_crc32 = mavlink_msg_drone_gain_ack_get_active_crc32(msg);
    drone_gain_ack->rows_mask = mavlink_msg_drone_gain_ack_get_rows_mask(msg);
    drone_gain_ack->action = mavlink_msg_drone_gain_ack_get_action(msg);
    drone_gain_ack->status = mavlink_msg_drone_gain_ack_get_status(msg);
    drone_gain_ack->custom = mavlink_msg_drone_gain_ack_get_custom(msg);
    drone_gain_ack->target_system = mavlink_msg_drone_gain_ack_get_target_system(msg);
    drone_gain_ack->target_component = mavlink_msg_drone_gain_ack_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN? msg->len : MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN;
        memset(drone_gain_ack, 0, MAVLINK_MSG_ID_DRONE_GAIN_ACK_LEN);
    memcpy(drone_gain_ack, _MAV_PAYLOAD(msg), len);
#endif
}
