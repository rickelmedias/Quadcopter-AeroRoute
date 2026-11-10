#pragma once
// MESSAGE DRONE_LEG_COMMAND PACKING

#define MAVLINK_MSG_ID_DRONE_LEG_COMMAND 51001


typedef struct __mavlink_drone_leg_command_t {
 uint32_t boot_id; /*<  Expected STM32 boot identity.*/
 uint32_t mission_id; /*<  Persistent mission identity.*/
 uint32_t command_id; /*<  Monotonic command identity within the mission.*/
 uint32_t timeout_ms; /*< [ms] Absolute execution timeout.*/
 float x; /*< [m] X objective in the selected frame.*/
 float y; /*< [m] Y objective in the selected frame.*/
 float z; /*< [m] Z objective in the selected frame.*/
 float yaw; /*< [rad] Yaw objective or relative delta.*/
 float speed; /*< [m/s] Maximum translational speed.*/
 float acceleration; /*< [m/s/s] Maximum translational acceleration.*/
 float position_tolerance; /*< [m] Position completion tolerance.*/
 float yaw_tolerance; /*< [rad] Yaw completion tolerance.*/
 uint16_t leg_index; /*<  Monotonic leg index in the compiled route.*/
 uint16_t flags; /*<  Requested behavior flags.*/
 uint8_t action; /*<  Guidance action.*/
 uint8_t frame; /*<  Coordinate frame.*/
 uint8_t target_system; /*<  System that must execute the command.*/
 uint8_t target_component; /*<  Component that must execute the command.*/
} mavlink_drone_leg_command_t;

#define MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN 56
#define MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN 56
#define MAVLINK_MSG_ID_51001_LEN 56
#define MAVLINK_MSG_ID_51001_MIN_LEN 56

#define MAVLINK_MSG_ID_DRONE_LEG_COMMAND_CRC 65
#define MAVLINK_MSG_ID_51001_CRC 65



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_LEG_COMMAND { \
    51001, \
    "DRONE_LEG_COMMAND", \
    18, \
    {  { "boot_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_leg_command_t, boot_id) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_leg_command_t, mission_id) }, \
         { "command_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_leg_command_t, command_id) }, \
         { "timeout_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_drone_leg_command_t, timeout_ms) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_drone_leg_command_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_drone_leg_command_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_drone_leg_command_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_drone_leg_command_t, yaw) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_drone_leg_command_t, speed) }, \
         { "acceleration", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_drone_leg_command_t, acceleration) }, \
         { "position_tolerance", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_drone_leg_command_t, position_tolerance) }, \
         { "yaw_tolerance", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_drone_leg_command_t, yaw_tolerance) }, \
         { "leg_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_drone_leg_command_t, leg_index) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 50, offsetof(mavlink_drone_leg_command_t, flags) }, \
         { "action", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_drone_leg_command_t, action) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_drone_leg_command_t, frame) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_drone_leg_command_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_drone_leg_command_t, target_component) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_LEG_COMMAND { \
    "DRONE_LEG_COMMAND", \
    18, \
    {  { "boot_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_leg_command_t, boot_id) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_leg_command_t, mission_id) }, \
         { "command_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_leg_command_t, command_id) }, \
         { "timeout_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_drone_leg_command_t, timeout_ms) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_drone_leg_command_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_drone_leg_command_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_drone_leg_command_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_drone_leg_command_t, yaw) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_drone_leg_command_t, speed) }, \
         { "acceleration", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_drone_leg_command_t, acceleration) }, \
         { "position_tolerance", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_drone_leg_command_t, position_tolerance) }, \
         { "yaw_tolerance", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_drone_leg_command_t, yaw_tolerance) }, \
         { "leg_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_drone_leg_command_t, leg_index) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 50, offsetof(mavlink_drone_leg_command_t, flags) }, \
         { "action", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_drone_leg_command_t, action) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_drone_leg_command_t, frame) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_drone_leg_command_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_drone_leg_command_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_leg_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param boot_id  Expected STM32 boot identity.
 * @param mission_id  Persistent mission identity.
 * @param command_id  Monotonic command identity within the mission.
 * @param timeout_ms [ms] Absolute execution timeout.
 * @param x [m] X objective in the selected frame.
 * @param y [m] Y objective in the selected frame.
 * @param z [m] Z objective in the selected frame.
 * @param yaw [rad] Yaw objective or relative delta.
 * @param speed [m/s] Maximum translational speed.
 * @param acceleration [m/s/s] Maximum translational acceleration.
 * @param position_tolerance [m] Position completion tolerance.
 * @param yaw_tolerance [rad] Yaw completion tolerance.
 * @param leg_index  Monotonic leg index in the compiled route.
 * @param flags  Requested behavior flags.
 * @param action  Guidance action.
 * @param frame  Coordinate frame.
 * @param target_system  System that must execute the command.
 * @param target_component  Component that must execute the command.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_leg_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint32_t timeout_ms, float x, float y, float z, float yaw, float speed, float acceleration, float position_tolerance, float yaw_tolerance, uint16_t leg_index, uint16_t flags, uint8_t action, uint8_t frame, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint32_t(buf, 12, timeout_ms);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, yaw);
    _mav_put_float(buf, 32, speed);
    _mav_put_float(buf, 36, acceleration);
    _mav_put_float(buf, 40, position_tolerance);
    _mav_put_float(buf, 44, yaw_tolerance);
    _mav_put_uint16_t(buf, 48, leg_index);
    _mav_put_uint16_t(buf, 50, flags);
    _mav_put_uint8_t(buf, 52, action);
    _mav_put_uint8_t(buf, 53, frame);
    _mav_put_uint8_t(buf, 54, target_system);
    _mav_put_uint8_t(buf, 55, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN);
#else
    mavlink_drone_leg_command_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.timeout_ms = timeout_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.speed = speed;
    packet.acceleration = acceleration;
    packet.position_tolerance = position_tolerance;
    packet.yaw_tolerance = yaw_tolerance;
    packet.leg_index = leg_index;
    packet.flags = flags;
    packet.action = action;
    packet.frame = frame;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_LEG_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_CRC);
}

/**
 * @brief Pack a drone_leg_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param boot_id  Expected STM32 boot identity.
 * @param mission_id  Persistent mission identity.
 * @param command_id  Monotonic command identity within the mission.
 * @param timeout_ms [ms] Absolute execution timeout.
 * @param x [m] X objective in the selected frame.
 * @param y [m] Y objective in the selected frame.
 * @param z [m] Z objective in the selected frame.
 * @param yaw [rad] Yaw objective or relative delta.
 * @param speed [m/s] Maximum translational speed.
 * @param acceleration [m/s/s] Maximum translational acceleration.
 * @param position_tolerance [m] Position completion tolerance.
 * @param yaw_tolerance [rad] Yaw completion tolerance.
 * @param leg_index  Monotonic leg index in the compiled route.
 * @param flags  Requested behavior flags.
 * @param action  Guidance action.
 * @param frame  Coordinate frame.
 * @param target_system  System that must execute the command.
 * @param target_component  Component that must execute the command.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_leg_command_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint32_t timeout_ms, float x, float y, float z, float yaw, float speed, float acceleration, float position_tolerance, float yaw_tolerance, uint16_t leg_index, uint16_t flags, uint8_t action, uint8_t frame, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint32_t(buf, 12, timeout_ms);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, yaw);
    _mav_put_float(buf, 32, speed);
    _mav_put_float(buf, 36, acceleration);
    _mav_put_float(buf, 40, position_tolerance);
    _mav_put_float(buf, 44, yaw_tolerance);
    _mav_put_uint16_t(buf, 48, leg_index);
    _mav_put_uint16_t(buf, 50, flags);
    _mav_put_uint8_t(buf, 52, action);
    _mav_put_uint8_t(buf, 53, frame);
    _mav_put_uint8_t(buf, 54, target_system);
    _mav_put_uint8_t(buf, 55, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN);
#else
    mavlink_drone_leg_command_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.timeout_ms = timeout_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.speed = speed;
    packet.acceleration = acceleration;
    packet.position_tolerance = position_tolerance;
    packet.yaw_tolerance = yaw_tolerance;
    packet.leg_index = leg_index;
    packet.flags = flags;
    packet.action = action;
    packet.frame = frame;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_LEG_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN);
#endif
}

/**
 * @brief Pack a drone_leg_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boot_id  Expected STM32 boot identity.
 * @param mission_id  Persistent mission identity.
 * @param command_id  Monotonic command identity within the mission.
 * @param timeout_ms [ms] Absolute execution timeout.
 * @param x [m] X objective in the selected frame.
 * @param y [m] Y objective in the selected frame.
 * @param z [m] Z objective in the selected frame.
 * @param yaw [rad] Yaw objective or relative delta.
 * @param speed [m/s] Maximum translational speed.
 * @param acceleration [m/s/s] Maximum translational acceleration.
 * @param position_tolerance [m] Position completion tolerance.
 * @param yaw_tolerance [rad] Yaw completion tolerance.
 * @param leg_index  Monotonic leg index in the compiled route.
 * @param flags  Requested behavior flags.
 * @param action  Guidance action.
 * @param frame  Coordinate frame.
 * @param target_system  System that must execute the command.
 * @param target_component  Component that must execute the command.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_leg_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t boot_id,uint32_t mission_id,uint32_t command_id,uint32_t timeout_ms,float x,float y,float z,float yaw,float speed,float acceleration,float position_tolerance,float yaw_tolerance,uint16_t leg_index,uint16_t flags,uint8_t action,uint8_t frame,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint32_t(buf, 12, timeout_ms);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, yaw);
    _mav_put_float(buf, 32, speed);
    _mav_put_float(buf, 36, acceleration);
    _mav_put_float(buf, 40, position_tolerance);
    _mav_put_float(buf, 44, yaw_tolerance);
    _mav_put_uint16_t(buf, 48, leg_index);
    _mav_put_uint16_t(buf, 50, flags);
    _mav_put_uint8_t(buf, 52, action);
    _mav_put_uint8_t(buf, 53, frame);
    _mav_put_uint8_t(buf, 54, target_system);
    _mav_put_uint8_t(buf, 55, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN);
#else
    mavlink_drone_leg_command_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.timeout_ms = timeout_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.speed = speed;
    packet.acceleration = acceleration;
    packet.position_tolerance = position_tolerance;
    packet.yaw_tolerance = yaw_tolerance;
    packet.leg_index = leg_index;
    packet.flags = flags;
    packet.action = action;
    packet.frame = frame;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_LEG_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_CRC);
}

/**
 * @brief Encode a drone_leg_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_leg_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_leg_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_leg_command_t* drone_leg_command)
{
    return mavlink_msg_drone_leg_command_pack(system_id, component_id, msg, drone_leg_command->boot_id, drone_leg_command->mission_id, drone_leg_command->command_id, drone_leg_command->timeout_ms, drone_leg_command->x, drone_leg_command->y, drone_leg_command->z, drone_leg_command->yaw, drone_leg_command->speed, drone_leg_command->acceleration, drone_leg_command->position_tolerance, drone_leg_command->yaw_tolerance, drone_leg_command->leg_index, drone_leg_command->flags, drone_leg_command->action, drone_leg_command->frame, drone_leg_command->target_system, drone_leg_command->target_component);
}

/**
 * @brief Encode a drone_leg_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_leg_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_leg_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_leg_command_t* drone_leg_command)
{
    return mavlink_msg_drone_leg_command_pack_chan(system_id, component_id, chan, msg, drone_leg_command->boot_id, drone_leg_command->mission_id, drone_leg_command->command_id, drone_leg_command->timeout_ms, drone_leg_command->x, drone_leg_command->y, drone_leg_command->z, drone_leg_command->yaw, drone_leg_command->speed, drone_leg_command->acceleration, drone_leg_command->position_tolerance, drone_leg_command->yaw_tolerance, drone_leg_command->leg_index, drone_leg_command->flags, drone_leg_command->action, drone_leg_command->frame, drone_leg_command->target_system, drone_leg_command->target_component);
}

/**
 * @brief Encode a drone_leg_command struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param drone_leg_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_leg_command_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_drone_leg_command_t* drone_leg_command)
{
    return mavlink_msg_drone_leg_command_pack_status(system_id, component_id, _status, msg,  drone_leg_command->boot_id, drone_leg_command->mission_id, drone_leg_command->command_id, drone_leg_command->timeout_ms, drone_leg_command->x, drone_leg_command->y, drone_leg_command->z, drone_leg_command->yaw, drone_leg_command->speed, drone_leg_command->acceleration, drone_leg_command->position_tolerance, drone_leg_command->yaw_tolerance, drone_leg_command->leg_index, drone_leg_command->flags, drone_leg_command->action, drone_leg_command->frame, drone_leg_command->target_system, drone_leg_command->target_component);
}

/**
 * @brief Send a drone_leg_command message
 * @param chan MAVLink channel to send the message
 *
 * @param boot_id  Expected STM32 boot identity.
 * @param mission_id  Persistent mission identity.
 * @param command_id  Monotonic command identity within the mission.
 * @param timeout_ms [ms] Absolute execution timeout.
 * @param x [m] X objective in the selected frame.
 * @param y [m] Y objective in the selected frame.
 * @param z [m] Z objective in the selected frame.
 * @param yaw [rad] Yaw objective or relative delta.
 * @param speed [m/s] Maximum translational speed.
 * @param acceleration [m/s/s] Maximum translational acceleration.
 * @param position_tolerance [m] Position completion tolerance.
 * @param yaw_tolerance [rad] Yaw completion tolerance.
 * @param leg_index  Monotonic leg index in the compiled route.
 * @param flags  Requested behavior flags.
 * @param action  Guidance action.
 * @param frame  Coordinate frame.
 * @param target_system  System that must execute the command.
 * @param target_component  Component that must execute the command.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_leg_command_send(mavlink_channel_t chan, uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint32_t timeout_ms, float x, float y, float z, float yaw, float speed, float acceleration, float position_tolerance, float yaw_tolerance, uint16_t leg_index, uint16_t flags, uint8_t action, uint8_t frame, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint32_t(buf, 12, timeout_ms);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, yaw);
    _mav_put_float(buf, 32, speed);
    _mav_put_float(buf, 36, acceleration);
    _mav_put_float(buf, 40, position_tolerance);
    _mav_put_float(buf, 44, yaw_tolerance);
    _mav_put_uint16_t(buf, 48, leg_index);
    _mav_put_uint16_t(buf, 50, flags);
    _mav_put_uint8_t(buf, 52, action);
    _mav_put_uint8_t(buf, 53, frame);
    _mav_put_uint8_t(buf, 54, target_system);
    _mav_put_uint8_t(buf, 55, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_COMMAND, buf, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_CRC);
#else
    mavlink_drone_leg_command_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.command_id = command_id;
    packet.timeout_ms = timeout_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.speed = speed;
    packet.acceleration = acceleration;
    packet.position_tolerance = position_tolerance;
    packet.yaw_tolerance = yaw_tolerance;
    packet.leg_index = leg_index;
    packet.flags = flags;
    packet.action = action;
    packet.frame = frame;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_CRC);
#endif
}

/**
 * @brief Send a drone_leg_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_leg_command_send_struct(mavlink_channel_t chan, const mavlink_drone_leg_command_t* drone_leg_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_leg_command_send(chan, drone_leg_command->boot_id, drone_leg_command->mission_id, drone_leg_command->command_id, drone_leg_command->timeout_ms, drone_leg_command->x, drone_leg_command->y, drone_leg_command->z, drone_leg_command->yaw, drone_leg_command->speed, drone_leg_command->acceleration, drone_leg_command->position_tolerance, drone_leg_command->yaw_tolerance, drone_leg_command->leg_index, drone_leg_command->flags, drone_leg_command->action, drone_leg_command->frame, drone_leg_command->target_system, drone_leg_command->target_component);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_COMMAND, (const char *)drone_leg_command, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_leg_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t boot_id, uint32_t mission_id, uint32_t command_id, uint32_t timeout_ms, float x, float y, float z, float yaw, float speed, float acceleration, float position_tolerance, float yaw_tolerance, uint16_t leg_index, uint16_t flags, uint8_t action, uint8_t frame, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, command_id);
    _mav_put_uint32_t(buf, 12, timeout_ms);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, yaw);
    _mav_put_float(buf, 32, speed);
    _mav_put_float(buf, 36, acceleration);
    _mav_put_float(buf, 40, position_tolerance);
    _mav_put_float(buf, 44, yaw_tolerance);
    _mav_put_uint16_t(buf, 48, leg_index);
    _mav_put_uint16_t(buf, 50, flags);
    _mav_put_uint8_t(buf, 52, action);
    _mav_put_uint8_t(buf, 53, frame);
    _mav_put_uint8_t(buf, 54, target_system);
    _mav_put_uint8_t(buf, 55, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_COMMAND, buf, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_CRC);
#else
    mavlink_drone_leg_command_t *packet = (mavlink_drone_leg_command_t *)msgbuf;
    packet->boot_id = boot_id;
    packet->mission_id = mission_id;
    packet->command_id = command_id;
    packet->timeout_ms = timeout_ms;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->yaw = yaw;
    packet->speed = speed;
    packet->acceleration = acceleration;
    packet->position_tolerance = position_tolerance;
    packet->yaw_tolerance = yaw_tolerance;
    packet->leg_index = leg_index;
    packet->flags = flags;
    packet->action = action;
    packet->frame = frame;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_LEG_COMMAND, (const char *)packet, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_LEG_COMMAND UNPACKING


/**
 * @brief Get field boot_id from drone_leg_command message
 *
 * @return  Expected STM32 boot identity.
 */
static inline uint32_t mavlink_msg_drone_leg_command_get_boot_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field mission_id from drone_leg_command message
 *
 * @return  Persistent mission identity.
 */
static inline uint32_t mavlink_msg_drone_leg_command_get_mission_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field command_id from drone_leg_command message
 *
 * @return  Monotonic command identity within the mission.
 */
static inline uint32_t mavlink_msg_drone_leg_command_get_command_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field timeout_ms from drone_leg_command message
 *
 * @return [ms] Absolute execution timeout.
 */
static inline uint32_t mavlink_msg_drone_leg_command_get_timeout_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field x from drone_leg_command message
 *
 * @return [m] X objective in the selected frame.
 */
static inline float mavlink_msg_drone_leg_command_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field y from drone_leg_command message
 *
 * @return [m] Y objective in the selected frame.
 */
static inline float mavlink_msg_drone_leg_command_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field z from drone_leg_command message
 *
 * @return [m] Z objective in the selected frame.
 */
static inline float mavlink_msg_drone_leg_command_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yaw from drone_leg_command message
 *
 * @return [rad] Yaw objective or relative delta.
 */
static inline float mavlink_msg_drone_leg_command_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field speed from drone_leg_command message
 *
 * @return [m/s] Maximum translational speed.
 */
static inline float mavlink_msg_drone_leg_command_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field acceleration from drone_leg_command message
 *
 * @return [m/s/s] Maximum translational acceleration.
 */
static inline float mavlink_msg_drone_leg_command_get_acceleration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field position_tolerance from drone_leg_command message
 *
 * @return [m] Position completion tolerance.
 */
static inline float mavlink_msg_drone_leg_command_get_position_tolerance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field yaw_tolerance from drone_leg_command message
 *
 * @return [rad] Yaw completion tolerance.
 */
static inline float mavlink_msg_drone_leg_command_get_yaw_tolerance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field leg_index from drone_leg_command message
 *
 * @return  Monotonic leg index in the compiled route.
 */
static inline uint16_t mavlink_msg_drone_leg_command_get_leg_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  48);
}

/**
 * @brief Get field flags from drone_leg_command message
 *
 * @return  Requested behavior flags.
 */
static inline uint16_t mavlink_msg_drone_leg_command_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  50);
}

/**
 * @brief Get field action from drone_leg_command message
 *
 * @return  Guidance action.
 */
static inline uint8_t mavlink_msg_drone_leg_command_get_action(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field frame from drone_leg_command message
 *
 * @return  Coordinate frame.
 */
static inline uint8_t mavlink_msg_drone_leg_command_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Get field target_system from drone_leg_command message
 *
 * @return  System that must execute the command.
 */
static inline uint8_t mavlink_msg_drone_leg_command_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  54);
}

/**
 * @brief Get field target_component from drone_leg_command message
 *
 * @return  Component that must execute the command.
 */
static inline uint8_t mavlink_msg_drone_leg_command_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  55);
}

/**
 * @brief Decode a drone_leg_command message into a struct
 *
 * @param msg The message to decode
 * @param drone_leg_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_leg_command_decode(const mavlink_message_t* msg, mavlink_drone_leg_command_t* drone_leg_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_leg_command->boot_id = mavlink_msg_drone_leg_command_get_boot_id(msg);
    drone_leg_command->mission_id = mavlink_msg_drone_leg_command_get_mission_id(msg);
    drone_leg_command->command_id = mavlink_msg_drone_leg_command_get_command_id(msg);
    drone_leg_command->timeout_ms = mavlink_msg_drone_leg_command_get_timeout_ms(msg);
    drone_leg_command->x = mavlink_msg_drone_leg_command_get_x(msg);
    drone_leg_command->y = mavlink_msg_drone_leg_command_get_y(msg);
    drone_leg_command->z = mavlink_msg_drone_leg_command_get_z(msg);
    drone_leg_command->yaw = mavlink_msg_drone_leg_command_get_yaw(msg);
    drone_leg_command->speed = mavlink_msg_drone_leg_command_get_speed(msg);
    drone_leg_command->acceleration = mavlink_msg_drone_leg_command_get_acceleration(msg);
    drone_leg_command->position_tolerance = mavlink_msg_drone_leg_command_get_position_tolerance(msg);
    drone_leg_command->yaw_tolerance = mavlink_msg_drone_leg_command_get_yaw_tolerance(msg);
    drone_leg_command->leg_index = mavlink_msg_drone_leg_command_get_leg_index(msg);
    drone_leg_command->flags = mavlink_msg_drone_leg_command_get_flags(msg);
    drone_leg_command->action = mavlink_msg_drone_leg_command_get_action(msg);
    drone_leg_command->frame = mavlink_msg_drone_leg_command_get_frame(msg);
    drone_leg_command->target_system = mavlink_msg_drone_leg_command_get_target_system(msg);
    drone_leg_command->target_component = mavlink_msg_drone_leg_command_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN;
        memset(drone_leg_command, 0, MAVLINK_MSG_ID_DRONE_LEG_COMMAND_LEN);
    memcpy(drone_leg_command, _MAV_PAYLOAD(msg), len);
#endif
}
