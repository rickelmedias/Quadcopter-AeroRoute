#pragma once
#include <stdint.h>

// MESSAGE DRONE_NAV_DEBUG PACKING

#define MAVLINK_MSG_ID_DRONE_NAV_DEBUG 51021


typedef struct __mavlink_drone_nav_debug_t {
 uint32_t time_boot_ms; /*< [ms] STM32 tick of the step.*/
 float flow_raw[2]; /*< [rad/s] MTF-01 flow, sensor x and y.*/
 float flow_body[2]; /*< [m/s] Flow velocity in body axes (forward, right), after scale and gyro compensation, before fusion.*/
 float range_raw; /*< [m] Range finder reading.*/
 float range_filt; /*< [m] Range after the jump gate and low-pass filter.*/
 float pos[3]; /*< [m] EKF position x, y (local) and z (NED, negative above ground).*/
 float vel[3]; /*< [m/s] EKF velocity vx, vy (local) and vz (NED, negative = climbing).*/
 float bias[3]; /*< [m/s/s] EKF accelerometer bias x, y, z.*/
 float accel[3]; /*< [m/s/s] Linear acceleration used in the prediction: x, y (local) and down.*/
 float flow_ratio; /*<  Last innovation test ratio of the x/y pair (above 1 = rejected).*/
 float z_ratio; /*<  Last innovation test ratio of z (above 1 = rejected).*/
 uint16_t flow_rejected; /*<  Flow corrections rejected by the gate (wraps).*/
 uint16_t z_rejected; /*<  Range corrections rejected by the EKF gate (wraps).*/
 uint16_t range_rejected; /*<  Readings rejected by the range filter (wraps).*/
 uint16_t flags; /*<  Bits: 1 new MTF frame, 2 flow fused, 4 flow rejected, 8 xy usable, 16 xy sent to the LQG, 32 range rejected, 64 z rejected, 128 z usable, 256 z sent to the LQG.*/
 uint8_t quality; /*<  MTF-01 flow quality, 0 to 255.*/
} mavlink_drone_nav_debug_t;

#define MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN 93
#define MAVLINK_MSG_ID_DRONE_NAV_DEBUG_MIN_LEN 93
#define MAVLINK_MSG_ID_51021_LEN 93
#define MAVLINK_MSG_ID_51021_MIN_LEN 93

#define MAVLINK_MSG_ID_DRONE_NAV_DEBUG_CRC 110
#define MAVLINK_MSG_ID_51021_CRC 110

#define MAVLINK_MSG_DRONE_NAV_DEBUG_FIELD_FLOW_RAW_LEN 2
#define MAVLINK_MSG_DRONE_NAV_DEBUG_FIELD_FLOW_BODY_LEN 2
#define MAVLINK_MSG_DRONE_NAV_DEBUG_FIELD_POS_LEN 3
#define MAVLINK_MSG_DRONE_NAV_DEBUG_FIELD_VEL_LEN 3
#define MAVLINK_MSG_DRONE_NAV_DEBUG_FIELD_BIAS_LEN 3
#define MAVLINK_MSG_DRONE_NAV_DEBUG_FIELD_ACCEL_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_NAV_DEBUG { \
    51021, \
    "DRONE_NAV_DEBUG", \
    16, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_nav_debug_t, time_boot_ms) }, \
         { "flow_raw", NULL, MAVLINK_TYPE_FLOAT, 2, 4, offsetof(mavlink_drone_nav_debug_t, flow_raw) }, \
         { "flow_body", NULL, MAVLINK_TYPE_FLOAT, 2, 12, offsetof(mavlink_drone_nav_debug_t, flow_body) }, \
         { "range_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_drone_nav_debug_t, range_raw) }, \
         { "range_filt", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_drone_nav_debug_t, range_filt) }, \
         { "pos", NULL, MAVLINK_TYPE_FLOAT, 3, 28, offsetof(mavlink_drone_nav_debug_t, pos) }, \
         { "vel", NULL, MAVLINK_TYPE_FLOAT, 3, 40, offsetof(mavlink_drone_nav_debug_t, vel) }, \
         { "bias", NULL, MAVLINK_TYPE_FLOAT, 3, 52, offsetof(mavlink_drone_nav_debug_t, bias) }, \
         { "accel", NULL, MAVLINK_TYPE_FLOAT, 3, 64, offsetof(mavlink_drone_nav_debug_t, accel) }, \
         { "flow_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_drone_nav_debug_t, flow_ratio) }, \
         { "z_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_drone_nav_debug_t, z_ratio) }, \
         { "flow_rejected", NULL, MAVLINK_TYPE_UINT16_T, 0, 84, offsetof(mavlink_drone_nav_debug_t, flow_rejected) }, \
         { "z_rejected", NULL, MAVLINK_TYPE_UINT16_T, 0, 86, offsetof(mavlink_drone_nav_debug_t, z_rejected) }, \
         { "range_rejected", NULL, MAVLINK_TYPE_UINT16_T, 0, 88, offsetof(mavlink_drone_nav_debug_t, range_rejected) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 90, offsetof(mavlink_drone_nav_debug_t, flags) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 92, offsetof(mavlink_drone_nav_debug_t, quality) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_NAV_DEBUG { \
    "DRONE_NAV_DEBUG", \
    16, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_nav_debug_t, time_boot_ms) }, \
         { "flow_raw", NULL, MAVLINK_TYPE_FLOAT, 2, 4, offsetof(mavlink_drone_nav_debug_t, flow_raw) }, \
         { "flow_body", NULL, MAVLINK_TYPE_FLOAT, 2, 12, offsetof(mavlink_drone_nav_debug_t, flow_body) }, \
         { "range_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_drone_nav_debug_t, range_raw) }, \
         { "range_filt", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_drone_nav_debug_t, range_filt) }, \
         { "pos", NULL, MAVLINK_TYPE_FLOAT, 3, 28, offsetof(mavlink_drone_nav_debug_t, pos) }, \
         { "vel", NULL, MAVLINK_TYPE_FLOAT, 3, 40, offsetof(mavlink_drone_nav_debug_t, vel) }, \
         { "bias", NULL, MAVLINK_TYPE_FLOAT, 3, 52, offsetof(mavlink_drone_nav_debug_t, bias) }, \
         { "accel", NULL, MAVLINK_TYPE_FLOAT, 3, 64, offsetof(mavlink_drone_nav_debug_t, accel) }, \
         { "flow_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_drone_nav_debug_t, flow_ratio) }, \
         { "z_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_drone_nav_debug_t, z_ratio) }, \
         { "flow_rejected", NULL, MAVLINK_TYPE_UINT16_T, 0, 84, offsetof(mavlink_drone_nav_debug_t, flow_rejected) }, \
         { "z_rejected", NULL, MAVLINK_TYPE_UINT16_T, 0, 86, offsetof(mavlink_drone_nav_debug_t, z_rejected) }, \
         { "range_rejected", NULL, MAVLINK_TYPE_UINT16_T, 0, 88, offsetof(mavlink_drone_nav_debug_t, range_rejected) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 90, offsetof(mavlink_drone_nav_debug_t, flags) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 92, offsetof(mavlink_drone_nav_debug_t, quality) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_nav_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] STM32 tick of the step.
 * @param flow_raw [rad/s] MTF-01 flow, sensor x and y.
 * @param flow_body [m/s] Flow velocity in body axes (forward, right), after scale and gyro compensation, before fusion.
 * @param range_raw [m] Range finder reading.
 * @param range_filt [m] Range after the jump gate and low-pass filter.
 * @param pos [m] EKF position x, y (local) and z (NED, negative above ground).
 * @param vel [m/s] EKF velocity vx, vy (local) and vz (NED, negative = climbing).
 * @param bias [m/s/s] EKF accelerometer bias x, y, z.
 * @param accel [m/s/s] Linear acceleration used in the prediction: x, y (local) and down.
 * @param flow_ratio  Last innovation test ratio of the x/y pair (above 1 = rejected).
 * @param z_ratio  Last innovation test ratio of z (above 1 = rejected).
 * @param flow_rejected  Flow corrections rejected by the gate (wraps).
 * @param z_rejected  Range corrections rejected by the EKF gate (wraps).
 * @param range_rejected  Readings rejected by the range filter (wraps).
 * @param flags  Bits: 1 new MTF frame, 2 flow fused, 4 flow rejected, 8 xy usable, 16 xy sent to the LQG, 32 range rejected, 64 z rejected, 128 z usable, 256 z sent to the LQG.
 * @param quality  MTF-01 flow quality, 0 to 255.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_nav_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const float *flow_raw, const float *flow_body, float range_raw, float range_filt, const float *pos, const float *vel, const float *bias, const float *accel, float flow_ratio, float z_ratio, uint16_t flow_rejected, uint16_t z_rejected, uint16_t range_rejected, uint16_t flags, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, range_raw);
    _mav_put_float(buf, 24, range_filt);
    _mav_put_float(buf, 76, flow_ratio);
    _mav_put_float(buf, 80, z_ratio);
    _mav_put_uint16_t(buf, 84, flow_rejected);
    _mav_put_uint16_t(buf, 86, z_rejected);
    _mav_put_uint16_t(buf, 88, range_rejected);
    _mav_put_uint16_t(buf, 90, flags);
    _mav_put_uint8_t(buf, 92, quality);
    _mav_put_float_array(buf, 4, flow_raw, 2);
    _mav_put_float_array(buf, 12, flow_body, 2);
    _mav_put_float_array(buf, 28, pos, 3);
    _mav_put_float_array(buf, 40, vel, 3);
    _mav_put_float_array(buf, 52, bias, 3);
    _mav_put_float_array(buf, 64, accel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN);
#else
    mavlink_drone_nav_debug_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.range_raw = range_raw;
    packet.range_filt = range_filt;
    packet.flow_ratio = flow_ratio;
    packet.z_ratio = z_ratio;
    packet.flow_rejected = flow_rejected;
    packet.z_rejected = z_rejected;
    packet.range_rejected = range_rejected;
    packet.flags = flags;
    packet.quality = quality;
    mav_array_memcpy(packet.flow_raw, flow_raw, sizeof(float)*2);
    mav_array_memcpy(packet.flow_body, flow_body, sizeof(float)*2);
    mav_array_memcpy(packet.pos, pos, sizeof(float)*3);
    mav_array_memcpy(packet.vel, vel, sizeof(float)*3);
    mav_array_memcpy(packet.bias, bias, sizeof(float)*3);
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_NAV_DEBUG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_CRC);
}

/**
 * @brief Pack a drone_nav_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] STM32 tick of the step.
 * @param flow_raw [rad/s] MTF-01 flow, sensor x and y.
 * @param flow_body [m/s] Flow velocity in body axes (forward, right), after scale and gyro compensation, before fusion.
 * @param range_raw [m] Range finder reading.
 * @param range_filt [m] Range after the jump gate and low-pass filter.
 * @param pos [m] EKF position x, y (local) and z (NED, negative above ground).
 * @param vel [m/s] EKF velocity vx, vy (local) and vz (NED, negative = climbing).
 * @param bias [m/s/s] EKF accelerometer bias x, y, z.
 * @param accel [m/s/s] Linear acceleration used in the prediction: x, y (local) and down.
 * @param flow_ratio  Last innovation test ratio of the x/y pair (above 1 = rejected).
 * @param z_ratio  Last innovation test ratio of z (above 1 = rejected).
 * @param flow_rejected  Flow corrections rejected by the gate (wraps).
 * @param z_rejected  Range corrections rejected by the EKF gate (wraps).
 * @param range_rejected  Readings rejected by the range filter (wraps).
 * @param flags  Bits: 1 new MTF frame, 2 flow fused, 4 flow rejected, 8 xy usable, 16 xy sent to the LQG, 32 range rejected, 64 z rejected, 128 z usable, 256 z sent to the LQG.
 * @param quality  MTF-01 flow quality, 0 to 255.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_nav_debug_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const float *flow_raw, const float *flow_body, float range_raw, float range_filt, const float *pos, const float *vel, const float *bias, const float *accel, float flow_ratio, float z_ratio, uint16_t flow_rejected, uint16_t z_rejected, uint16_t range_rejected, uint16_t flags, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, range_raw);
    _mav_put_float(buf, 24, range_filt);
    _mav_put_float(buf, 76, flow_ratio);
    _mav_put_float(buf, 80, z_ratio);
    _mav_put_uint16_t(buf, 84, flow_rejected);
    _mav_put_uint16_t(buf, 86, z_rejected);
    _mav_put_uint16_t(buf, 88, range_rejected);
    _mav_put_uint16_t(buf, 90, flags);
    _mav_put_uint8_t(buf, 92, quality);
    _mav_put_float_array(buf, 4, flow_raw, 2);
    _mav_put_float_array(buf, 12, flow_body, 2);
    _mav_put_float_array(buf, 28, pos, 3);
    _mav_put_float_array(buf, 40, vel, 3);
    _mav_put_float_array(buf, 52, bias, 3);
    _mav_put_float_array(buf, 64, accel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN);
#else
    mavlink_drone_nav_debug_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.range_raw = range_raw;
    packet.range_filt = range_filt;
    packet.flow_ratio = flow_ratio;
    packet.z_ratio = z_ratio;
    packet.flow_rejected = flow_rejected;
    packet.z_rejected = z_rejected;
    packet.range_rejected = range_rejected;
    packet.flags = flags;
    packet.quality = quality;
    mav_array_memcpy(packet.flow_raw, flow_raw, sizeof(float)*2);
    mav_array_memcpy(packet.flow_body, flow_body, sizeof(float)*2);
    mav_array_memcpy(packet.pos, pos, sizeof(float)*3);
    mav_array_memcpy(packet.vel, vel, sizeof(float)*3);
    mav_array_memcpy(packet.bias, bias, sizeof(float)*3);
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_NAV_DEBUG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN);
#endif
}

/**
 * @brief Pack a drone_nav_debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] STM32 tick of the step.
 * @param flow_raw [rad/s] MTF-01 flow, sensor x and y.
 * @param flow_body [m/s] Flow velocity in body axes (forward, right), after scale and gyro compensation, before fusion.
 * @param range_raw [m] Range finder reading.
 * @param range_filt [m] Range after the jump gate and low-pass filter.
 * @param pos [m] EKF position x, y (local) and z (NED, negative above ground).
 * @param vel [m/s] EKF velocity vx, vy (local) and vz (NED, negative = climbing).
 * @param bias [m/s/s] EKF accelerometer bias x, y, z.
 * @param accel [m/s/s] Linear acceleration used in the prediction: x, y (local) and down.
 * @param flow_ratio  Last innovation test ratio of the x/y pair (above 1 = rejected).
 * @param z_ratio  Last innovation test ratio of z (above 1 = rejected).
 * @param flow_rejected  Flow corrections rejected by the gate (wraps).
 * @param z_rejected  Range corrections rejected by the EKF gate (wraps).
 * @param range_rejected  Readings rejected by the range filter (wraps).
 * @param flags  Bits: 1 new MTF frame, 2 flow fused, 4 flow rejected, 8 xy usable, 16 xy sent to the LQG, 32 range rejected, 64 z rejected, 128 z usable, 256 z sent to the LQG.
 * @param quality  MTF-01 flow quality, 0 to 255.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_nav_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,const float *flow_raw,const float *flow_body,float range_raw,float range_filt,const float *pos,const float *vel,const float *bias,const float *accel,float flow_ratio,float z_ratio,uint16_t flow_rejected,uint16_t z_rejected,uint16_t range_rejected,uint16_t flags,uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, range_raw);
    _mav_put_float(buf, 24, range_filt);
    _mav_put_float(buf, 76, flow_ratio);
    _mav_put_float(buf, 80, z_ratio);
    _mav_put_uint16_t(buf, 84, flow_rejected);
    _mav_put_uint16_t(buf, 86, z_rejected);
    _mav_put_uint16_t(buf, 88, range_rejected);
    _mav_put_uint16_t(buf, 90, flags);
    _mav_put_uint8_t(buf, 92, quality);
    _mav_put_float_array(buf, 4, flow_raw, 2);
    _mav_put_float_array(buf, 12, flow_body, 2);
    _mav_put_float_array(buf, 28, pos, 3);
    _mav_put_float_array(buf, 40, vel, 3);
    _mav_put_float_array(buf, 52, bias, 3);
    _mav_put_float_array(buf, 64, accel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN);
#else
    mavlink_drone_nav_debug_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.range_raw = range_raw;
    packet.range_filt = range_filt;
    packet.flow_ratio = flow_ratio;
    packet.z_ratio = z_ratio;
    packet.flow_rejected = flow_rejected;
    packet.z_rejected = z_rejected;
    packet.range_rejected = range_rejected;
    packet.flags = flags;
    packet.quality = quality;
    mav_array_memcpy(packet.flow_raw, flow_raw, sizeof(float)*2);
    mav_array_memcpy(packet.flow_body, flow_body, sizeof(float)*2);
    mav_array_memcpy(packet.pos, pos, sizeof(float)*3);
    mav_array_memcpy(packet.vel, vel, sizeof(float)*3);
    mav_array_memcpy(packet.bias, bias, sizeof(float)*3);
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_NAV_DEBUG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_CRC);
}

/**
 * @brief Encode a drone_nav_debug struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_nav_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_nav_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_nav_debug_t* drone_nav_debug)
{
    return mavlink_msg_drone_nav_debug_pack(system_id, component_id, msg, drone_nav_debug->time_boot_ms, drone_nav_debug->flow_raw, drone_nav_debug->flow_body, drone_nav_debug->range_raw, drone_nav_debug->range_filt, drone_nav_debug->pos, drone_nav_debug->vel, drone_nav_debug->bias, drone_nav_debug->accel, drone_nav_debug->flow_ratio, drone_nav_debug->z_ratio, drone_nav_debug->flow_rejected, drone_nav_debug->z_rejected, drone_nav_debug->range_rejected, drone_nav_debug->flags, drone_nav_debug->quality);
}

/**
 * @brief Encode a drone_nav_debug struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_nav_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_nav_debug_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_nav_debug_t* drone_nav_debug)
{
    return mavlink_msg_drone_nav_debug_pack_chan(system_id, component_id, chan, msg, drone_nav_debug->time_boot_ms, drone_nav_debug->flow_raw, drone_nav_debug->flow_body, drone_nav_debug->range_raw, drone_nav_debug->range_filt, drone_nav_debug->pos, drone_nav_debug->vel, drone_nav_debug->bias, drone_nav_debug->accel, drone_nav_debug->flow_ratio, drone_nav_debug->z_ratio, drone_nav_debug->flow_rejected, drone_nav_debug->z_rejected, drone_nav_debug->range_rejected, drone_nav_debug->flags, drone_nav_debug->quality);
}

/**
 * @brief Encode a drone_nav_debug struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param drone_nav_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_nav_debug_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_drone_nav_debug_t* drone_nav_debug)
{
    return mavlink_msg_drone_nav_debug_pack_status(system_id, component_id, _status, msg,  drone_nav_debug->time_boot_ms, drone_nav_debug->flow_raw, drone_nav_debug->flow_body, drone_nav_debug->range_raw, drone_nav_debug->range_filt, drone_nav_debug->pos, drone_nav_debug->vel, drone_nav_debug->bias, drone_nav_debug->accel, drone_nav_debug->flow_ratio, drone_nav_debug->z_ratio, drone_nav_debug->flow_rejected, drone_nav_debug->z_rejected, drone_nav_debug->range_rejected, drone_nav_debug->flags, drone_nav_debug->quality);
}

/**
 * @brief Send a drone_nav_debug message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] STM32 tick of the step.
 * @param flow_raw [rad/s] MTF-01 flow, sensor x and y.
 * @param flow_body [m/s] Flow velocity in body axes (forward, right), after scale and gyro compensation, before fusion.
 * @param range_raw [m] Range finder reading.
 * @param range_filt [m] Range after the jump gate and low-pass filter.
 * @param pos [m] EKF position x, y (local) and z (NED, negative above ground).
 * @param vel [m/s] EKF velocity vx, vy (local) and vz (NED, negative = climbing).
 * @param bias [m/s/s] EKF accelerometer bias x, y, z.
 * @param accel [m/s/s] Linear acceleration used in the prediction: x, y (local) and down.
 * @param flow_ratio  Last innovation test ratio of the x/y pair (above 1 = rejected).
 * @param z_ratio  Last innovation test ratio of z (above 1 = rejected).
 * @param flow_rejected  Flow corrections rejected by the gate (wraps).
 * @param z_rejected  Range corrections rejected by the EKF gate (wraps).
 * @param range_rejected  Readings rejected by the range filter (wraps).
 * @param flags  Bits: 1 new MTF frame, 2 flow fused, 4 flow rejected, 8 xy usable, 16 xy sent to the LQG, 32 range rejected, 64 z rejected, 128 z usable, 256 z sent to the LQG.
 * @param quality  MTF-01 flow quality, 0 to 255.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_nav_debug_send(mavlink_channel_t chan, uint32_t time_boot_ms, const float *flow_raw, const float *flow_body, float range_raw, float range_filt, const float *pos, const float *vel, const float *bias, const float *accel, float flow_ratio, float z_ratio, uint16_t flow_rejected, uint16_t z_rejected, uint16_t range_rejected, uint16_t flags, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, range_raw);
    _mav_put_float(buf, 24, range_filt);
    _mav_put_float(buf, 76, flow_ratio);
    _mav_put_float(buf, 80, z_ratio);
    _mav_put_uint16_t(buf, 84, flow_rejected);
    _mav_put_uint16_t(buf, 86, z_rejected);
    _mav_put_uint16_t(buf, 88, range_rejected);
    _mav_put_uint16_t(buf, 90, flags);
    _mav_put_uint8_t(buf, 92, quality);
    _mav_put_float_array(buf, 4, flow_raw, 2);
    _mav_put_float_array(buf, 12, flow_body, 2);
    _mav_put_float_array(buf, 28, pos, 3);
    _mav_put_float_array(buf, 40, vel, 3);
    _mav_put_float_array(buf, 52, bias, 3);
    _mav_put_float_array(buf, 64, accel, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_NAV_DEBUG, buf, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_CRC);
#else
    mavlink_drone_nav_debug_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.range_raw = range_raw;
    packet.range_filt = range_filt;
    packet.flow_ratio = flow_ratio;
    packet.z_ratio = z_ratio;
    packet.flow_rejected = flow_rejected;
    packet.z_rejected = z_rejected;
    packet.range_rejected = range_rejected;
    packet.flags = flags;
    packet.quality = quality;
    mav_array_memcpy(packet.flow_raw, flow_raw, sizeof(float)*2);
    mav_array_memcpy(packet.flow_body, flow_body, sizeof(float)*2);
    mav_array_memcpy(packet.pos, pos, sizeof(float)*3);
    mav_array_memcpy(packet.vel, vel, sizeof(float)*3);
    mav_array_memcpy(packet.bias, bias, sizeof(float)*3);
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_NAV_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_CRC);
#endif
}

/**
 * @brief Send a drone_nav_debug message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_nav_debug_send_struct(mavlink_channel_t chan, const mavlink_drone_nav_debug_t* drone_nav_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_nav_debug_send(chan, drone_nav_debug->time_boot_ms, drone_nav_debug->flow_raw, drone_nav_debug->flow_body, drone_nav_debug->range_raw, drone_nav_debug->range_filt, drone_nav_debug->pos, drone_nav_debug->vel, drone_nav_debug->bias, drone_nav_debug->accel, drone_nav_debug->flow_ratio, drone_nav_debug->z_ratio, drone_nav_debug->flow_rejected, drone_nav_debug->z_rejected, drone_nav_debug->range_rejected, drone_nav_debug->flags, drone_nav_debug->quality);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_NAV_DEBUG, (const char *)drone_nav_debug, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_nav_debug_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, const float *flow_raw, const float *flow_body, float range_raw, float range_filt, const float *pos, const float *vel, const float *bias, const float *accel, float flow_ratio, float z_ratio, uint16_t flow_rejected, uint16_t z_rejected, uint16_t range_rejected, uint16_t flags, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, range_raw);
    _mav_put_float(buf, 24, range_filt);
    _mav_put_float(buf, 76, flow_ratio);
    _mav_put_float(buf, 80, z_ratio);
    _mav_put_uint16_t(buf, 84, flow_rejected);
    _mav_put_uint16_t(buf, 86, z_rejected);
    _mav_put_uint16_t(buf, 88, range_rejected);
    _mav_put_uint16_t(buf, 90, flags);
    _mav_put_uint8_t(buf, 92, quality);
    _mav_put_float_array(buf, 4, flow_raw, 2);
    _mav_put_float_array(buf, 12, flow_body, 2);
    _mav_put_float_array(buf, 28, pos, 3);
    _mav_put_float_array(buf, 40, vel, 3);
    _mav_put_float_array(buf, 52, bias, 3);
    _mav_put_float_array(buf, 64, accel, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_NAV_DEBUG, buf, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_CRC);
#else
    mavlink_drone_nav_debug_t *packet = (mavlink_drone_nav_debug_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->range_raw = range_raw;
    packet->range_filt = range_filt;
    packet->flow_ratio = flow_ratio;
    packet->z_ratio = z_ratio;
    packet->flow_rejected = flow_rejected;
    packet->z_rejected = z_rejected;
    packet->range_rejected = range_rejected;
    packet->flags = flags;
    packet->quality = quality;
    mav_array_memcpy(packet->flow_raw, flow_raw, sizeof(float)*2);
    mav_array_memcpy(packet->flow_body, flow_body, sizeof(float)*2);
    mav_array_memcpy(packet->pos, pos, sizeof(float)*3);
    mav_array_memcpy(packet->vel, vel, sizeof(float)*3);
    mav_array_memcpy(packet->bias, bias, sizeof(float)*3);
    mav_array_memcpy(packet->accel, accel, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_NAV_DEBUG, (const char *)packet, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_MIN_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_NAV_DEBUG UNPACKING


/**
 * @brief Get field time_boot_ms from drone_nav_debug message
 *
 * @return [ms] STM32 tick of the step.
 */
static inline uint32_t mavlink_msg_drone_nav_debug_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field flow_raw from drone_nav_debug message
 *
 * @return [rad/s] MTF-01 flow, sensor x and y.
 */
static inline uint16_t mavlink_msg_drone_nav_debug_get_flow_raw(const mavlink_message_t* msg, float *flow_raw)
{
    return _MAV_RETURN_float_array(msg, flow_raw, 2,  4);
}

/**
 * @brief Get field flow_body from drone_nav_debug message
 *
 * @return [m/s] Flow velocity in body axes (forward, right), after scale and gyro compensation, before fusion.
 */
static inline uint16_t mavlink_msg_drone_nav_debug_get_flow_body(const mavlink_message_t* msg, float *flow_body)
{
    return _MAV_RETURN_float_array(msg, flow_body, 2,  12);
}

/**
 * @brief Get field range_raw from drone_nav_debug message
 *
 * @return [m] Range finder reading.
 */
static inline float mavlink_msg_drone_nav_debug_get_range_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field range_filt from drone_nav_debug message
 *
 * @return [m] Range after the jump gate and low-pass filter.
 */
static inline float mavlink_msg_drone_nav_debug_get_range_filt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pos from drone_nav_debug message
 *
 * @return [m] EKF position x, y (local) and z (NED, negative above ground).
 */
static inline uint16_t mavlink_msg_drone_nav_debug_get_pos(const mavlink_message_t* msg, float *pos)
{
    return _MAV_RETURN_float_array(msg, pos, 3,  28);
}

/**
 * @brief Get field vel from drone_nav_debug message
 *
 * @return [m/s] EKF velocity vx, vy (local) and vz (NED, negative = climbing).
 */
static inline uint16_t mavlink_msg_drone_nav_debug_get_vel(const mavlink_message_t* msg, float *vel)
{
    return _MAV_RETURN_float_array(msg, vel, 3,  40);
}

/**
 * @brief Get field bias from drone_nav_debug message
 *
 * @return [m/s/s] EKF accelerometer bias x, y, z.
 */
static inline uint16_t mavlink_msg_drone_nav_debug_get_bias(const mavlink_message_t* msg, float *bias)
{
    return _MAV_RETURN_float_array(msg, bias, 3,  52);
}

/**
 * @brief Get field accel from drone_nav_debug message
 *
 * @return [m/s/s] Linear acceleration used in the prediction: x, y (local) and down.
 */
static inline uint16_t mavlink_msg_drone_nav_debug_get_accel(const mavlink_message_t* msg, float *accel)
{
    return _MAV_RETURN_float_array(msg, accel, 3,  64);
}

/**
 * @brief Get field flow_ratio from drone_nav_debug message
 *
 * @return  Last innovation test ratio of the x/y pair (above 1 = rejected).
 */
static inline float mavlink_msg_drone_nav_debug_get_flow_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field z_ratio from drone_nav_debug message
 *
 * @return  Last innovation test ratio of z (above 1 = rejected).
 */
static inline float mavlink_msg_drone_nav_debug_get_z_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field flow_rejected from drone_nav_debug message
 *
 * @return  Flow corrections rejected by the gate (wraps).
 */
static inline uint16_t mavlink_msg_drone_nav_debug_get_flow_rejected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  84);
}

/**
 * @brief Get field z_rejected from drone_nav_debug message
 *
 * @return  Range corrections rejected by the EKF gate (wraps).
 */
static inline uint16_t mavlink_msg_drone_nav_debug_get_z_rejected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  86);
}

/**
 * @brief Get field range_rejected from drone_nav_debug message
 *
 * @return  Readings rejected by the range filter (wraps).
 */
static inline uint16_t mavlink_msg_drone_nav_debug_get_range_rejected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  88);
}

/**
 * @brief Get field flags from drone_nav_debug message
 *
 * @return  Bits: 1 new MTF frame, 2 flow fused, 4 flow rejected, 8 xy usable, 16 xy sent to the LQG, 32 range rejected, 64 z rejected, 128 z usable, 256 z sent to the LQG.
 */
static inline uint16_t mavlink_msg_drone_nav_debug_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  90);
}

/**
 * @brief Get field quality from drone_nav_debug message
 *
 * @return  MTF-01 flow quality, 0 to 255.
 */
static inline uint8_t mavlink_msg_drone_nav_debug_get_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  92);
}

/**
 * @brief Decode a drone_nav_debug message into a struct
 *
 * @param msg The message to decode
 * @param drone_nav_debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_nav_debug_decode(const mavlink_message_t* msg, mavlink_drone_nav_debug_t* drone_nav_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_nav_debug->time_boot_ms = mavlink_msg_drone_nav_debug_get_time_boot_ms(msg);
    mavlink_msg_drone_nav_debug_get_flow_raw(msg, drone_nav_debug->flow_raw);
    mavlink_msg_drone_nav_debug_get_flow_body(msg, drone_nav_debug->flow_body);
    drone_nav_debug->range_raw = mavlink_msg_drone_nav_debug_get_range_raw(msg);
    drone_nav_debug->range_filt = mavlink_msg_drone_nav_debug_get_range_filt(msg);
    mavlink_msg_drone_nav_debug_get_pos(msg, drone_nav_debug->pos);
    mavlink_msg_drone_nav_debug_get_vel(msg, drone_nav_debug->vel);
    mavlink_msg_drone_nav_debug_get_bias(msg, drone_nav_debug->bias);
    mavlink_msg_drone_nav_debug_get_accel(msg, drone_nav_debug->accel);
    drone_nav_debug->flow_ratio = mavlink_msg_drone_nav_debug_get_flow_ratio(msg);
    drone_nav_debug->z_ratio = mavlink_msg_drone_nav_debug_get_z_ratio(msg);
    drone_nav_debug->flow_rejected = mavlink_msg_drone_nav_debug_get_flow_rejected(msg);
    drone_nav_debug->z_rejected = mavlink_msg_drone_nav_debug_get_z_rejected(msg);
    drone_nav_debug->range_rejected = mavlink_msg_drone_nav_debug_get_range_rejected(msg);
    drone_nav_debug->flags = mavlink_msg_drone_nav_debug_get_flags(msg);
    drone_nav_debug->quality = mavlink_msg_drone_nav_debug_get_quality(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN? msg->len : MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN;
        memset(drone_nav_debug, 0, MAVLINK_MSG_ID_DRONE_NAV_DEBUG_LEN);
    memcpy(drone_nav_debug, _MAV_PAYLOAD(msg), len);
#endif
}
