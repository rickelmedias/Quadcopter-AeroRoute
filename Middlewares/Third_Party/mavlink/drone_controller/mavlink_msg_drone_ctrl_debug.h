#pragma once
// MESSAGE DRONE_CTRL_DEBUG PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_DRONE_CTRL_DEBUG 51020


typedef struct __mavlink_drone_ctrl_debug_t {
 uint32_t time_boot_ms; /*< [ms] STM32 tick of the step.*/
 uint32_t step; /*<  Control steps since boot.*/
 float y[9]; /*<  Measurements z, phi, theta, psi, vx, vy, p, q, r (SI, NED).*/
 float x_hat[12]; /*<  Kalman estimate x, y, z, phi, theta, psi, vx, vy, vz, p, q, r (SI, NED).*/
 float du[4]; /*< [rad/s] LQG output: motor speed deviation from hover.*/
 float ref[4]; /*<  Reference x, y, z, psi (m, m, m, rad).*/
 uint16_t throttle[4]; /*<  Throttle of each motor, 0 to 10000 (also computed in dry run).*/
 uint8_t flight_state; /*<  Flight state.*/
 uint8_t meas_flags; /*<  Measurements refreshed in this step.*/
 uint8_t ctrl_flags; /*<  Controller condition.*/
} mavlink_drone_ctrl_debug_t;

#define MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN 135
#define MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN 135
#define MAVLINK_MSG_ID_51020_LEN 135
#define MAVLINK_MSG_ID_51020_MIN_LEN 135

#define MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_CRC 28
#define MAVLINK_MSG_ID_51020_CRC 28

#define MAVLINK_MSG_DRONE_CTRL_DEBUG_FIELD_Y_LEN 9
#define MAVLINK_MSG_DRONE_CTRL_DEBUG_FIELD_X_HAT_LEN 12
#define MAVLINK_MSG_DRONE_CTRL_DEBUG_FIELD_DU_LEN 4
#define MAVLINK_MSG_DRONE_CTRL_DEBUG_FIELD_REF_LEN 4
#define MAVLINK_MSG_DRONE_CTRL_DEBUG_FIELD_THROTTLE_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_CTRL_DEBUG { \
    51020, \
    "DRONE_CTRL_DEBUG", \
    10, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_ctrl_debug_t, time_boot_ms) }, \
         { "step", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_ctrl_debug_t, step) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 9, 8, offsetof(mavlink_drone_ctrl_debug_t, y) }, \
         { "x_hat", NULL, MAVLINK_TYPE_FLOAT, 12, 44, offsetof(mavlink_drone_ctrl_debug_t, x_hat) }, \
         { "du", NULL, MAVLINK_TYPE_FLOAT, 4, 92, offsetof(mavlink_drone_ctrl_debug_t, du) }, \
         { "ref", NULL, MAVLINK_TYPE_FLOAT, 4, 108, offsetof(mavlink_drone_ctrl_debug_t, ref) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 4, 124, offsetof(mavlink_drone_ctrl_debug_t, throttle) }, \
         { "flight_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 132, offsetof(mavlink_drone_ctrl_debug_t, flight_state) }, \
         { "meas_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 133, offsetof(mavlink_drone_ctrl_debug_t, meas_flags) }, \
         { "ctrl_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 134, offsetof(mavlink_drone_ctrl_debug_t, ctrl_flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_CTRL_DEBUG { \
    "DRONE_CTRL_DEBUG", \
    10, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_ctrl_debug_t, time_boot_ms) }, \
         { "step", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_ctrl_debug_t, step) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 9, 8, offsetof(mavlink_drone_ctrl_debug_t, y) }, \
         { "x_hat", NULL, MAVLINK_TYPE_FLOAT, 12, 44, offsetof(mavlink_drone_ctrl_debug_t, x_hat) }, \
         { "du", NULL, MAVLINK_TYPE_FLOAT, 4, 92, offsetof(mavlink_drone_ctrl_debug_t, du) }, \
         { "ref", NULL, MAVLINK_TYPE_FLOAT, 4, 108, offsetof(mavlink_drone_ctrl_debug_t, ref) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 4, 124, offsetof(mavlink_drone_ctrl_debug_t, throttle) }, \
         { "flight_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 132, offsetof(mavlink_drone_ctrl_debug_t, flight_state) }, \
         { "meas_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 133, offsetof(mavlink_drone_ctrl_debug_t, meas_flags) }, \
         { "ctrl_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 134, offsetof(mavlink_drone_ctrl_debug_t, ctrl_flags) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_ctrl_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] STM32 tick of the step.
 * @param step  Control steps since boot.
 * @param y  Measurements z, phi, theta, psi, vx, vy, p, q, r (SI, NED).
 * @param x_hat  Kalman estimate x, y, z, phi, theta, psi, vx, vy, vz, p, q, r (SI, NED).
 * @param du [rad/s] LQG output: motor speed deviation from hover.
 * @param ref  Reference x, y, z, psi (m, m, m, rad).
 * @param throttle  Throttle of each motor, 0 to 10000 (also computed in dry run).
 * @param flight_state  Flight state.
 * @param meas_flags  Measurements refreshed in this step.
 * @param ctrl_flags  Controller condition.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_ctrl_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t step, const float *y, const float *x_hat, const float *du, const float *ref, const uint16_t *throttle, uint8_t flight_state, uint8_t meas_flags, uint8_t ctrl_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, step);
    _mav_put_uint8_t(buf, 132, flight_state);
    _mav_put_uint8_t(buf, 133, meas_flags);
    _mav_put_uint8_t(buf, 134, ctrl_flags);
    _mav_put_float_array(buf, 8, y, 9);
    _mav_put_float_array(buf, 44, x_hat, 12);
    _mav_put_float_array(buf, 92, du, 4);
    _mav_put_float_array(buf, 108, ref, 4);
    _mav_put_uint16_t_array(buf, 124, throttle, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN);
#else
    mavlink_drone_ctrl_debug_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.step = step;
    packet.flight_state = flight_state;
    packet.meas_flags = meas_flags;
    packet.ctrl_flags = ctrl_flags;
    mav_array_memcpy(packet.y, y, sizeof(float)*9);
    mav_array_memcpy(packet.x_hat, x_hat, sizeof(float)*12);
    mav_array_memcpy(packet.du, du, sizeof(float)*4);
    mav_array_memcpy(packet.ref, ref, sizeof(float)*4);
    mav_array_memcpy(packet.throttle, throttle, sizeof(uint16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_CTRL_DEBUG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_CRC);
}

/**
 * @brief Pack a drone_ctrl_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] STM32 tick of the step.
 * @param step  Control steps since boot.
 * @param y  Measurements z, phi, theta, psi, vx, vy, p, q, r (SI, NED).
 * @param x_hat  Kalman estimate x, y, z, phi, theta, psi, vx, vy, vz, p, q, r (SI, NED).
 * @param du [rad/s] LQG output: motor speed deviation from hover.
 * @param ref  Reference x, y, z, psi (m, m, m, rad).
 * @param throttle  Throttle of each motor, 0 to 10000 (also computed in dry run).
 * @param flight_state  Flight state.
 * @param meas_flags  Measurements refreshed in this step.
 * @param ctrl_flags  Controller condition.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_ctrl_debug_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t step, const float *y, const float *x_hat, const float *du, const float *ref, const uint16_t *throttle, uint8_t flight_state, uint8_t meas_flags, uint8_t ctrl_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, step);
    _mav_put_uint8_t(buf, 132, flight_state);
    _mav_put_uint8_t(buf, 133, meas_flags);
    _mav_put_uint8_t(buf, 134, ctrl_flags);
    _mav_put_float_array(buf, 8, y, 9);
    _mav_put_float_array(buf, 44, x_hat, 12);
    _mav_put_float_array(buf, 92, du, 4);
    _mav_put_float_array(buf, 108, ref, 4);
    _mav_put_uint16_t_array(buf, 124, throttle, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN);
#else
    mavlink_drone_ctrl_debug_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.step = step;
    packet.flight_state = flight_state;
    packet.meas_flags = meas_flags;
    packet.ctrl_flags = ctrl_flags;
    mav_array_memcpy(packet.y, y, sizeof(float)*9);
    mav_array_memcpy(packet.x_hat, x_hat, sizeof(float)*12);
    mav_array_memcpy(packet.du, du, sizeof(float)*4);
    mav_array_memcpy(packet.ref, ref, sizeof(float)*4);
    mav_array_memcpy(packet.throttle, throttle, sizeof(uint16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_CTRL_DEBUG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN);
#endif
}

/**
 * @brief Pack a drone_ctrl_debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] STM32 tick of the step.
 * @param step  Control steps since boot.
 * @param y  Measurements z, phi, theta, psi, vx, vy, p, q, r (SI, NED).
 * @param x_hat  Kalman estimate x, y, z, phi, theta, psi, vx, vy, vz, p, q, r (SI, NED).
 * @param du [rad/s] LQG output: motor speed deviation from hover.
 * @param ref  Reference x, y, z, psi (m, m, m, rad).
 * @param throttle  Throttle of each motor, 0 to 10000 (also computed in dry run).
 * @param flight_state  Flight state.
 * @param meas_flags  Measurements refreshed in this step.
 * @param ctrl_flags  Controller condition.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_ctrl_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint32_t step,const float *y,const float *x_hat,const float *du,const float *ref,const uint16_t *throttle,uint8_t flight_state,uint8_t meas_flags,uint8_t ctrl_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, step);
    _mav_put_uint8_t(buf, 132, flight_state);
    _mav_put_uint8_t(buf, 133, meas_flags);
    _mav_put_uint8_t(buf, 134, ctrl_flags);
    _mav_put_float_array(buf, 8, y, 9);
    _mav_put_float_array(buf, 44, x_hat, 12);
    _mav_put_float_array(buf, 92, du, 4);
    _mav_put_float_array(buf, 108, ref, 4);
    _mav_put_uint16_t_array(buf, 124, throttle, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN);
#else
    mavlink_drone_ctrl_debug_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.step = step;
    packet.flight_state = flight_state;
    packet.meas_flags = meas_flags;
    packet.ctrl_flags = ctrl_flags;
    mav_array_memcpy(packet.y, y, sizeof(float)*9);
    mav_array_memcpy(packet.x_hat, x_hat, sizeof(float)*12);
    mav_array_memcpy(packet.du, du, sizeof(float)*4);
    mav_array_memcpy(packet.ref, ref, sizeof(float)*4);
    mav_array_memcpy(packet.throttle, throttle, sizeof(uint16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_CTRL_DEBUG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_CRC);
}

/**
 * @brief Encode a drone_ctrl_debug struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_ctrl_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_ctrl_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_ctrl_debug_t* drone_ctrl_debug)
{
    return mavlink_msg_drone_ctrl_debug_pack(system_id, component_id, msg, drone_ctrl_debug->time_boot_ms, drone_ctrl_debug->step, drone_ctrl_debug->y, drone_ctrl_debug->x_hat, drone_ctrl_debug->du, drone_ctrl_debug->ref, drone_ctrl_debug->throttle, drone_ctrl_debug->flight_state, drone_ctrl_debug->meas_flags, drone_ctrl_debug->ctrl_flags);
}

/**
 * @brief Encode a drone_ctrl_debug struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_ctrl_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_ctrl_debug_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_ctrl_debug_t* drone_ctrl_debug)
{
    return mavlink_msg_drone_ctrl_debug_pack_chan(system_id, component_id, chan, msg, drone_ctrl_debug->time_boot_ms, drone_ctrl_debug->step, drone_ctrl_debug->y, drone_ctrl_debug->x_hat, drone_ctrl_debug->du, drone_ctrl_debug->ref, drone_ctrl_debug->throttle, drone_ctrl_debug->flight_state, drone_ctrl_debug->meas_flags, drone_ctrl_debug->ctrl_flags);
}

/**
 * @brief Encode a drone_ctrl_debug struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param drone_ctrl_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_ctrl_debug_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_drone_ctrl_debug_t* drone_ctrl_debug)
{
    return mavlink_msg_drone_ctrl_debug_pack_status(system_id, component_id, _status, msg,  drone_ctrl_debug->time_boot_ms, drone_ctrl_debug->step, drone_ctrl_debug->y, drone_ctrl_debug->x_hat, drone_ctrl_debug->du, drone_ctrl_debug->ref, drone_ctrl_debug->throttle, drone_ctrl_debug->flight_state, drone_ctrl_debug->meas_flags, drone_ctrl_debug->ctrl_flags);
}

/**
 * @brief Send a drone_ctrl_debug message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] STM32 tick of the step.
 * @param step  Control steps since boot.
 * @param y  Measurements z, phi, theta, psi, vx, vy, p, q, r (SI, NED).
 * @param x_hat  Kalman estimate x, y, z, phi, theta, psi, vx, vy, vz, p, q, r (SI, NED).
 * @param du [rad/s] LQG output: motor speed deviation from hover.
 * @param ref  Reference x, y, z, psi (m, m, m, rad).
 * @param throttle  Throttle of each motor, 0 to 10000 (also computed in dry run).
 * @param flight_state  Flight state.
 * @param meas_flags  Measurements refreshed in this step.
 * @param ctrl_flags  Controller condition.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_ctrl_debug_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint32_t step, const float *y, const float *x_hat, const float *du, const float *ref, const uint16_t *throttle, uint8_t flight_state, uint8_t meas_flags, uint8_t ctrl_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, step);
    _mav_put_uint8_t(buf, 132, flight_state);
    _mav_put_uint8_t(buf, 133, meas_flags);
    _mav_put_uint8_t(buf, 134, ctrl_flags);
    _mav_put_float_array(buf, 8, y, 9);
    _mav_put_float_array(buf, 44, x_hat, 12);
    _mav_put_float_array(buf, 92, du, 4);
    _mav_put_float_array(buf, 108, ref, 4);
    _mav_put_uint16_t_array(buf, 124, throttle, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG, buf, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_CRC);
#else
    mavlink_drone_ctrl_debug_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.step = step;
    packet.flight_state = flight_state;
    packet.meas_flags = meas_flags;
    packet.ctrl_flags = ctrl_flags;
    mav_array_memcpy(packet.y, y, sizeof(float)*9);
    mav_array_memcpy(packet.x_hat, x_hat, sizeof(float)*12);
    mav_array_memcpy(packet.du, du, sizeof(float)*4);
    mav_array_memcpy(packet.ref, ref, sizeof(float)*4);
    mav_array_memcpy(packet.throttle, throttle, sizeof(uint16_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_CRC);
#endif
}

/**
 * @brief Send a drone_ctrl_debug message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_ctrl_debug_send_struct(mavlink_channel_t chan, const mavlink_drone_ctrl_debug_t* drone_ctrl_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_ctrl_debug_send(chan, drone_ctrl_debug->time_boot_ms, drone_ctrl_debug->step, drone_ctrl_debug->y, drone_ctrl_debug->x_hat, drone_ctrl_debug->du, drone_ctrl_debug->ref, drone_ctrl_debug->throttle, drone_ctrl_debug->flight_state, drone_ctrl_debug->meas_flags, drone_ctrl_debug->ctrl_flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG, (const char *)drone_ctrl_debug, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_ctrl_debug_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint32_t step, const float *y, const float *x_hat, const float *du, const float *ref, const uint16_t *throttle, uint8_t flight_state, uint8_t meas_flags, uint8_t ctrl_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, step);
    _mav_put_uint8_t(buf, 132, flight_state);
    _mav_put_uint8_t(buf, 133, meas_flags);
    _mav_put_uint8_t(buf, 134, ctrl_flags);
    _mav_put_float_array(buf, 8, y, 9);
    _mav_put_float_array(buf, 44, x_hat, 12);
    _mav_put_float_array(buf, 92, du, 4);
    _mav_put_float_array(buf, 108, ref, 4);
    _mav_put_uint16_t_array(buf, 124, throttle, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG, buf, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_CRC);
#else
    mavlink_drone_ctrl_debug_t *packet = (mavlink_drone_ctrl_debug_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->step = step;
    packet->flight_state = flight_state;
    packet->meas_flags = meas_flags;
    packet->ctrl_flags = ctrl_flags;
    mav_array_memcpy(packet->y, y, sizeof(float)*9);
    mav_array_memcpy(packet->x_hat, x_hat, sizeof(float)*12);
    mav_array_memcpy(packet->du, du, sizeof(float)*4);
    mav_array_memcpy(packet->ref, ref, sizeof(float)*4);
    mav_array_memcpy(packet->throttle, throttle, sizeof(uint16_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG, (const char *)packet, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_CTRL_DEBUG UNPACKING


/**
 * @brief Get field time_boot_ms from drone_ctrl_debug message
 *
 * @return [ms] STM32 tick of the step.
 */
static inline uint32_t mavlink_msg_drone_ctrl_debug_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field step from drone_ctrl_debug message
 *
 * @return  Control steps since boot.
 */
static inline uint32_t mavlink_msg_drone_ctrl_debug_get_step(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field y from drone_ctrl_debug message
 *
 * @return  Measurements z, phi, theta, psi, vx, vy, p, q, r (SI, NED).
 */
static inline uint16_t mavlink_msg_drone_ctrl_debug_get_y(const mavlink_message_t* msg, float *y)
{
    return _MAV_RETURN_float_array(msg, y, 9,  8);
}

/**
 * @brief Get field x_hat from drone_ctrl_debug message
 *
 * @return  Kalman estimate x, y, z, phi, theta, psi, vx, vy, vz, p, q, r (SI, NED).
 */
static inline uint16_t mavlink_msg_drone_ctrl_debug_get_x_hat(const mavlink_message_t* msg, float *x_hat)
{
    return _MAV_RETURN_float_array(msg, x_hat, 12,  44);
}

/**
 * @brief Get field du from drone_ctrl_debug message
 *
 * @return [rad/s] LQG output: motor speed deviation from hover.
 */
static inline uint16_t mavlink_msg_drone_ctrl_debug_get_du(const mavlink_message_t* msg, float *du)
{
    return _MAV_RETURN_float_array(msg, du, 4,  92);
}

/**
 * @brief Get field ref from drone_ctrl_debug message
 *
 * @return  Reference x, y, z, psi (m, m, m, rad).
 */
static inline uint16_t mavlink_msg_drone_ctrl_debug_get_ref(const mavlink_message_t* msg, float *ref)
{
    return _MAV_RETURN_float_array(msg, ref, 4,  108);
}

/**
 * @brief Get field throttle from drone_ctrl_debug message
 *
 * @return  Throttle of each motor, 0 to 10000 (also computed in dry run).
 */
static inline uint16_t mavlink_msg_drone_ctrl_debug_get_throttle(const mavlink_message_t* msg, uint16_t *throttle)
{
    return _MAV_RETURN_uint16_t_array(msg, throttle, 4,  124);
}

/**
 * @brief Get field flight_state from drone_ctrl_debug message
 *
 * @return  Flight state.
 */
static inline uint8_t mavlink_msg_drone_ctrl_debug_get_flight_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  132);
}

/**
 * @brief Get field meas_flags from drone_ctrl_debug message
 *
 * @return  Measurements refreshed in this step.
 */
static inline uint8_t mavlink_msg_drone_ctrl_debug_get_meas_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  133);
}

/**
 * @brief Get field ctrl_flags from drone_ctrl_debug message
 *
 * @return  Controller condition.
 */
static inline uint8_t mavlink_msg_drone_ctrl_debug_get_ctrl_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  134);
}

/**
 * @brief Decode a drone_ctrl_debug message into a struct
 *
 * @param msg The message to decode
 * @param drone_ctrl_debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_ctrl_debug_decode(const mavlink_message_t* msg, mavlink_drone_ctrl_debug_t* drone_ctrl_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_ctrl_debug->time_boot_ms = mavlink_msg_drone_ctrl_debug_get_time_boot_ms(msg);
    drone_ctrl_debug->step = mavlink_msg_drone_ctrl_debug_get_step(msg);
    mavlink_msg_drone_ctrl_debug_get_y(msg, drone_ctrl_debug->y);
    mavlink_msg_drone_ctrl_debug_get_x_hat(msg, drone_ctrl_debug->x_hat);
    mavlink_msg_drone_ctrl_debug_get_du(msg, drone_ctrl_debug->du);
    mavlink_msg_drone_ctrl_debug_get_ref(msg, drone_ctrl_debug->ref);
    mavlink_msg_drone_ctrl_debug_get_throttle(msg, drone_ctrl_debug->throttle);
    drone_ctrl_debug->flight_state = mavlink_msg_drone_ctrl_debug_get_flight_state(msg);
    drone_ctrl_debug->meas_flags = mavlink_msg_drone_ctrl_debug_get_meas_flags(msg);
    drone_ctrl_debug->ctrl_flags = mavlink_msg_drone_ctrl_debug_get_ctrl_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN? msg->len : MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN;
        memset(drone_ctrl_debug, 0, MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_LEN);
    memcpy(drone_ctrl_debug, _MAV_PAYLOAD(msg), len);
#endif
}
