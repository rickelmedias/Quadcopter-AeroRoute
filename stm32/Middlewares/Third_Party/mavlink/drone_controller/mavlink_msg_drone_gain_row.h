#pragma once
// MESSAGE DRONE_GAIN_ROW PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_DRONE_GAIN_ROW 51010


typedef struct __mavlink_drone_gain_row_t {
 uint32_t session_id; /*<  Upload session chosen by the Pi, non-zero. A new session discards the previous staging.*/
 float values[12]; /*<  Row values; only the first count are used.*/
 uint8_t matrix; /*<  Matrix of this row.*/
 uint8_t row; /*<  Row index inside the matrix.*/
 uint8_t count; /*<  Number of columns, must match the matrix.*/
 uint8_t target_system; /*<  System that must stage the row.*/
 uint8_t target_component; /*<  Component that must stage the row.*/
} mavlink_drone_gain_row_t;

#define MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN 57
#define MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN 57
#define MAVLINK_MSG_ID_51010_LEN 57
#define MAVLINK_MSG_ID_51010_MIN_LEN 57

#define MAVLINK_MSG_ID_DRONE_GAIN_ROW_CRC 39
#define MAVLINK_MSG_ID_51010_CRC 39

#define MAVLINK_MSG_DRONE_GAIN_ROW_FIELD_VALUES_LEN 12

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_GAIN_ROW { \
    51010, \
    "DRONE_GAIN_ROW", \
    7, \
    {  { "session_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_gain_row_t, session_id) }, \
         { "values", NULL, MAVLINK_TYPE_FLOAT, 12, 4, offsetof(mavlink_drone_gain_row_t, values) }, \
         { "matrix", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_drone_gain_row_t, matrix) }, \
         { "row", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_drone_gain_row_t, row) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_drone_gain_row_t, count) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_drone_gain_row_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_drone_gain_row_t, target_component) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_GAIN_ROW { \
    "DRONE_GAIN_ROW", \
    7, \
    {  { "session_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_gain_row_t, session_id) }, \
         { "values", NULL, MAVLINK_TYPE_FLOAT, 12, 4, offsetof(mavlink_drone_gain_row_t, values) }, \
         { "matrix", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_drone_gain_row_t, matrix) }, \
         { "row", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_drone_gain_row_t, row) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_drone_gain_row_t, count) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_drone_gain_row_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_drone_gain_row_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_gain_row message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param session_id  Upload session chosen by the Pi, non-zero. A new session discards the previous staging.
 * @param values  Row values; only the first count are used.
 * @param matrix  Matrix of this row.
 * @param row  Row index inside the matrix.
 * @param count  Number of columns, must match the matrix.
 * @param target_system  System that must stage the row.
 * @param target_component  Component that must stage the row.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_gain_row_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t session_id, const float *values, uint8_t matrix, uint8_t row, uint8_t count, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint8_t(buf, 52, matrix);
    _mav_put_uint8_t(buf, 53, row);
    _mav_put_uint8_t(buf, 54, count);
    _mav_put_uint8_t(buf, 55, target_system);
    _mav_put_uint8_t(buf, 56, target_component);
    _mav_put_float_array(buf, 4, values, 12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN);
#else
    mavlink_drone_gain_row_t packet;
    packet.session_id = session_id;
    packet.matrix = matrix;
    packet.row = row;
    packet.count = count;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.values, values, sizeof(float)*12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_GAIN_ROW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_CRC);
}

/**
 * @brief Pack a drone_gain_row message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param session_id  Upload session chosen by the Pi, non-zero. A new session discards the previous staging.
 * @param values  Row values; only the first count are used.
 * @param matrix  Matrix of this row.
 * @param row  Row index inside the matrix.
 * @param count  Number of columns, must match the matrix.
 * @param target_system  System that must stage the row.
 * @param target_component  Component that must stage the row.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_gain_row_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t session_id, const float *values, uint8_t matrix, uint8_t row, uint8_t count, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint8_t(buf, 52, matrix);
    _mav_put_uint8_t(buf, 53, row);
    _mav_put_uint8_t(buf, 54, count);
    _mav_put_uint8_t(buf, 55, target_system);
    _mav_put_uint8_t(buf, 56, target_component);
    _mav_put_float_array(buf, 4, values, 12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN);
#else
    mavlink_drone_gain_row_t packet;
    packet.session_id = session_id;
    packet.matrix = matrix;
    packet.row = row;
    packet.count = count;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.values, values, sizeof(float)*12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_GAIN_ROW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN);
#endif
}

/**
 * @brief Pack a drone_gain_row message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param session_id  Upload session chosen by the Pi, non-zero. A new session discards the previous staging.
 * @param values  Row values; only the first count are used.
 * @param matrix  Matrix of this row.
 * @param row  Row index inside the matrix.
 * @param count  Number of columns, must match the matrix.
 * @param target_system  System that must stage the row.
 * @param target_component  Component that must stage the row.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_gain_row_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t session_id,const float *values,uint8_t matrix,uint8_t row,uint8_t count,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint8_t(buf, 52, matrix);
    _mav_put_uint8_t(buf, 53, row);
    _mav_put_uint8_t(buf, 54, count);
    _mav_put_uint8_t(buf, 55, target_system);
    _mav_put_uint8_t(buf, 56, target_component);
    _mav_put_float_array(buf, 4, values, 12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN);
#else
    mavlink_drone_gain_row_t packet;
    packet.session_id = session_id;
    packet.matrix = matrix;
    packet.row = row;
    packet.count = count;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.values, values, sizeof(float)*12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_GAIN_ROW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_CRC);
}

/**
 * @brief Encode a drone_gain_row struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_gain_row C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_gain_row_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_gain_row_t* drone_gain_row)
{
    return mavlink_msg_drone_gain_row_pack(system_id, component_id, msg, drone_gain_row->session_id, drone_gain_row->values, drone_gain_row->matrix, drone_gain_row->row, drone_gain_row->count, drone_gain_row->target_system, drone_gain_row->target_component);
}

/**
 * @brief Encode a drone_gain_row struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_gain_row C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_gain_row_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_gain_row_t* drone_gain_row)
{
    return mavlink_msg_drone_gain_row_pack_chan(system_id, component_id, chan, msg, drone_gain_row->session_id, drone_gain_row->values, drone_gain_row->matrix, drone_gain_row->row, drone_gain_row->count, drone_gain_row->target_system, drone_gain_row->target_component);
}

/**
 * @brief Encode a drone_gain_row struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param drone_gain_row C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_gain_row_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_drone_gain_row_t* drone_gain_row)
{
    return mavlink_msg_drone_gain_row_pack_status(system_id, component_id, _status, msg,  drone_gain_row->session_id, drone_gain_row->values, drone_gain_row->matrix, drone_gain_row->row, drone_gain_row->count, drone_gain_row->target_system, drone_gain_row->target_component);
}

/**
 * @brief Send a drone_gain_row message
 * @param chan MAVLink channel to send the message
 *
 * @param session_id  Upload session chosen by the Pi, non-zero. A new session discards the previous staging.
 * @param values  Row values; only the first count are used.
 * @param matrix  Matrix of this row.
 * @param row  Row index inside the matrix.
 * @param count  Number of columns, must match the matrix.
 * @param target_system  System that must stage the row.
 * @param target_component  Component that must stage the row.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_gain_row_send(mavlink_channel_t chan, uint32_t session_id, const float *values, uint8_t matrix, uint8_t row, uint8_t count, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN];
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint8_t(buf, 52, matrix);
    _mav_put_uint8_t(buf, 53, row);
    _mav_put_uint8_t(buf, 54, count);
    _mav_put_uint8_t(buf, 55, target_system);
    _mav_put_uint8_t(buf, 56, target_component);
    _mav_put_float_array(buf, 4, values, 12);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_ROW, buf, MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_CRC);
#else
    mavlink_drone_gain_row_t packet;
    packet.session_id = session_id;
    packet.matrix = matrix;
    packet.row = row;
    packet.count = count;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.values, values, sizeof(float)*12);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_ROW, (const char *)&packet, MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_CRC);
#endif
}

/**
 * @brief Send a drone_gain_row message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_gain_row_send_struct(mavlink_channel_t chan, const mavlink_drone_gain_row_t* drone_gain_row)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_gain_row_send(chan, drone_gain_row->session_id, drone_gain_row->values, drone_gain_row->matrix, drone_gain_row->row, drone_gain_row->count, drone_gain_row->target_system, drone_gain_row->target_component);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_ROW, (const char *)drone_gain_row, MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_gain_row_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t session_id, const float *values, uint8_t matrix, uint8_t row, uint8_t count, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, session_id);
    _mav_put_uint8_t(buf, 52, matrix);
    _mav_put_uint8_t(buf, 53, row);
    _mav_put_uint8_t(buf, 54, count);
    _mav_put_uint8_t(buf, 55, target_system);
    _mav_put_uint8_t(buf, 56, target_component);
    _mav_put_float_array(buf, 4, values, 12);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_ROW, buf, MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_CRC);
#else
    mavlink_drone_gain_row_t *packet = (mavlink_drone_gain_row_t *)msgbuf;
    packet->session_id = session_id;
    packet->matrix = matrix;
    packet->row = row;
    packet->count = count;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mav_array_memcpy(packet->values, values, sizeof(float)*12);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_GAIN_ROW, (const char *)packet, MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN, MAVLINK_MSG_ID_DRONE_GAIN_ROW_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_GAIN_ROW UNPACKING


/**
 * @brief Get field session_id from drone_gain_row message
 *
 * @return  Upload session chosen by the Pi, non-zero. A new session discards the previous staging.
 */
static inline uint32_t mavlink_msg_drone_gain_row_get_session_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field values from drone_gain_row message
 *
 * @return  Row values; only the first count are used.
 */
static inline uint16_t mavlink_msg_drone_gain_row_get_values(const mavlink_message_t* msg, float *values)
{
    return _MAV_RETURN_float_array(msg, values, 12,  4);
}

/**
 * @brief Get field matrix from drone_gain_row message
 *
 * @return  Matrix of this row.
 */
static inline uint8_t mavlink_msg_drone_gain_row_get_matrix(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field row from drone_gain_row message
 *
 * @return  Row index inside the matrix.
 */
static inline uint8_t mavlink_msg_drone_gain_row_get_row(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Get field count from drone_gain_row message
 *
 * @return  Number of columns, must match the matrix.
 */
static inline uint8_t mavlink_msg_drone_gain_row_get_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  54);
}

/**
 * @brief Get field target_system from drone_gain_row message
 *
 * @return  System that must stage the row.
 */
static inline uint8_t mavlink_msg_drone_gain_row_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  55);
}

/**
 * @brief Get field target_component from drone_gain_row message
 *
 * @return  Component that must stage the row.
 */
static inline uint8_t mavlink_msg_drone_gain_row_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  56);
}

/**
 * @brief Decode a drone_gain_row message into a struct
 *
 * @param msg The message to decode
 * @param drone_gain_row C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_gain_row_decode(const mavlink_message_t* msg, mavlink_drone_gain_row_t* drone_gain_row)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_gain_row->session_id = mavlink_msg_drone_gain_row_get_session_id(msg);
    mavlink_msg_drone_gain_row_get_values(msg, drone_gain_row->values);
    drone_gain_row->matrix = mavlink_msg_drone_gain_row_get_matrix(msg);
    drone_gain_row->row = mavlink_msg_drone_gain_row_get_row(msg);
    drone_gain_row->count = mavlink_msg_drone_gain_row_get_count(msg);
    drone_gain_row->target_system = mavlink_msg_drone_gain_row_get_target_system(msg);
    drone_gain_row->target_component = mavlink_msg_drone_gain_row_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN? msg->len : MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN;
        memset(drone_gain_row, 0, MAVLINK_MSG_ID_DRONE_GAIN_ROW_LEN);
    memcpy(drone_gain_row, _MAV_PAYLOAD(msg), len);
#endif
}
