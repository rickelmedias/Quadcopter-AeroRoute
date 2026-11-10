#pragma once
// MESSAGE DRONE_GAIN_COMMIT PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_DRONE_GAIN_COMMIT 51011


typedef struct __mavlink_drone_gain_commit_t {
 uint32_t session_id; /*<  Upload session to commit or abort.*/
 uint32_t crc32; /*<  CRC-32 (zlib) of all 280 values as float32 little-endian, in the order Kd, Kfd, L, M, row by row.*/
 uint8_t action; /*<  Requested action.*/
 uint8_t target_system; /*<  System that must process the request.*/
 uint8_t target_component; /*<  Component that must process the request.*/
} mavlink_drone_gain_commit_t;

#define MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN 11
#define MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN 11
#define MAVLINK_MSG_ID_51011_LEN 11
#define MAVLINK_MSG_ID_51011_MIN_LEN 11

#define MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_CRC 130
#define MAVLINK_MSG_ID_51011_CRC 130



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_GAIN_COMMIT { \
    51011, \
    "DRONE_GAIN_COMMIT", \
    5, \
    {  { "session_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_gain_commit_t, session_id) }, \
         { "crc32", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_gain_commit_t, crc32) }, \
         { "action", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_drone_gain_commit_t, action) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_drone_gain_commit_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_drone_gain_commit_t, target_component) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_GAIN_COMMIT { \
    "DRONE_GAIN_COMMIT", \
    5, \
    {  { "session_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_gain_commit_t, session_id) }, \
         { "crc32", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_gain_commit_t, crc32) }, \
         { "action", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_drone_gain_commit_t, action) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_drone_gain_commit_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_drone_gain_commit_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_gain_commit message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param session_id  Upload session to commit or abort.
 * @param crc32  CRC-32 (zlib) of all 280 values as float32 little-endian, in the order Kd, Kfd, L, M, row by row.
 * @param action  Requested action.
 * @param target_system  System that must process the request.
 * @param target_component  Component that must process the request.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_gain_commit_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t session_id, uint32_t crc32, uint8_t action, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint32_t(buf, 4, crc32);
    _mav_put_uint8_t(buf, 8, action);
    _mav_put_uint8_t(buf, 9, target_system);
    _mav_put_uint8_t(buf, 10, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN);
#else
    mavlink_drone_gain_commit_t packet;
    packet.session_id = session_id;
    packet.crc32 = crc32;
    packet.action = action;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_GAIN_COMMIT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_CRC);
}

/**
 * @brief Pack a drone_gain_commit message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param session_id  Upload session to commit or abort.
 * @param crc32  CRC-32 (zlib) of all 280 values as float32 little-endian, in the order Kd, Kfd, L, M, row by row.
 * @param action  Requested action.
 * @param target_system  System that must process the request.
 * @param target_component  Component that must process the request.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_gain_commit_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t session_id, uint32_t crc32, uint8_t action, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint32_t(buf, 4, crc32);
    _mav_put_uint8_t(buf, 8, action);
    _mav_put_uint8_t(buf, 9, target_system);
    _mav_put_uint8_t(buf, 10, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN);
#else
    mavlink_drone_gain_commit_t packet;
    packet.session_id = session_id;
    packet.crc32 = crc32;
    packet.action = action;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_GAIN_COMMIT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN);
#endif
}

/**
 * @brief Pack a drone_gain_commit message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param session_id  Upload session to commit or abort.
 * @param crc32  CRC-32 (zlib) of all 280 values as float32 little-endian, in the order Kd, Kfd, L, M, row by row.
 * @param action  Requested action.
 * @param target_system  System that must process the request.
 * @param target_component  Component that must process the request.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_gain_commit_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t session_id,uint32_t crc32,uint8_t action,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint32_t(buf, 4, crc32);
    _mav_put_uint8_t(buf, 8, action);
    _mav_put_uint8_t(buf, 9, target_system);
    _mav_put_uint8_t(buf, 10, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN);
#else
    mavlink_drone_gain_commit_t packet;
    packet.session_id = session_id;
    packet.crc32 = crc32;
    packet.action = action;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_GAIN_COMMIT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_CRC);
}

/**
 * @brief Encode a drone_gain_commit struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_gain_commit C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_gain_commit_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_gain_commit_t* drone_gain_commit)
{
    return mavlink_msg_drone_gain_commit_pack(system_id, component_id, msg, drone_gain_commit->session_id, drone_gain_commit->crc32, drone_gain_commit->action, drone_gain_commit->target_system, drone_gain_commit->target_component);
}

/**
 * @brief Encode a drone_gain_commit struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_gain_commit C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_gain_commit_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_gain_commit_t* drone_gain_commit)
{
    return mavlink_msg_drone_gain_commit_pack_chan(system_id, component_id, chan, msg, drone_gain_commit->session_id, drone_gain_commit->crc32, drone_gain_commit->action, drone_gain_commit->target_system, drone_gain_commit->target_component);
}

/**
 * @brief Encode a drone_gain_commit struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param drone_gain_commit C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_gain_commit_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_drone_gain_commit_t* drone_gain_commit)
{
    return mavlink_msg_drone_gain_commit_pack_status(system_id, component_id, _status, msg,  drone_gain_commit->session_id, drone_gain_commit->crc32, drone_gain_commit->action, drone_gain_commit->target_system, drone_gain_commit->target_component);
}

/**
 * @brief Send a drone_gain_commit message
 * @param chan MAVLink channel to send the message
 *
 * @param session_id  Upload session to commit or abort.
 * @param crc32  CRC-32 (zlib) of all 280 values as float32 little-endian, in the order Kd, Kfd, L, M, row by row.
 * @param action  Requested action.
 * @param target_system  System that must process the request.
 * @param target_component  Component that must process the request.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_gain_commit_send(mavlink_channel_t chan, uint32_t session_id, uint32_t crc32, uint8_t action, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint32_t(buf, 4, crc32);
    _mav_put_uint8_t(buf, 8, action);
    _mav_put_uint8_t(buf, 9, target_system);
    _mav_put_uint8_t(buf, 10, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT, buf, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_CRC);
#else
    mavlink_drone_gain_commit_t packet;
    packet.session_id = session_id;
    packet.crc32 = crc32;
    packet.action = action;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT, (const char *)&packet, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_CRC);
#endif
}

/**
 * @brief Send a drone_gain_commit message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_gain_commit_send_struct(mavlink_channel_t chan, const mavlink_drone_gain_commit_t* drone_gain_commit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_gain_commit_send(chan, drone_gain_commit->session_id, drone_gain_commit->crc32, drone_gain_commit->action, drone_gain_commit->target_system, drone_gain_commit->target_component);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT, (const char *)drone_gain_commit, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_gain_commit_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t session_id, uint32_t crc32, uint8_t action, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint32_t(buf, 4, crc32);
    _mav_put_uint8_t(buf, 8, action);
    _mav_put_uint8_t(buf, 9, target_system);
    _mav_put_uint8_t(buf, 10, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT, buf, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_CRC);
#else
    mavlink_drone_gain_commit_t *packet = (mavlink_drone_gain_commit_t *)msgbuf;
    packet->session_id = session_id;
    packet->crc32 = crc32;
    packet->action = action;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT, (const char *)packet, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_GAIN_COMMIT UNPACKING


/**
 * @brief Get field session_id from drone_gain_commit message
 *
 * @return  Upload session to commit or abort.
 */
static inline uint32_t mavlink_msg_drone_gain_commit_get_session_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field crc32 from drone_gain_commit message
 *
 * @return  CRC-32 (zlib) of all 280 values as float32 little-endian, in the order Kd, Kfd, L, M, row by row.
 */
static inline uint32_t mavlink_msg_drone_gain_commit_get_crc32(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field action from drone_gain_commit message
 *
 * @return  Requested action.
 */
static inline uint8_t mavlink_msg_drone_gain_commit_get_action(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field target_system from drone_gain_commit message
 *
 * @return  System that must process the request.
 */
static inline uint8_t mavlink_msg_drone_gain_commit_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field target_component from drone_gain_commit message
 *
 * @return  Component that must process the request.
 */
static inline uint8_t mavlink_msg_drone_gain_commit_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Decode a drone_gain_commit message into a struct
 *
 * @param msg The message to decode
 * @param drone_gain_commit C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_gain_commit_decode(const mavlink_message_t* msg, mavlink_drone_gain_commit_t* drone_gain_commit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_gain_commit->session_id = mavlink_msg_drone_gain_commit_get_session_id(msg);
    drone_gain_commit->crc32 = mavlink_msg_drone_gain_commit_get_crc32(msg);
    drone_gain_commit->action = mavlink_msg_drone_gain_commit_get_action(msg);
    drone_gain_commit->target_system = mavlink_msg_drone_gain_commit_get_target_system(msg);
    drone_gain_commit->target_component = mavlink_msg_drone_gain_commit_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN? msg->len : MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN;
        memset(drone_gain_commit, 0, MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_LEN);
    memcpy(drone_gain_commit, _MAV_PAYLOAD(msg), len);
#endif
}
