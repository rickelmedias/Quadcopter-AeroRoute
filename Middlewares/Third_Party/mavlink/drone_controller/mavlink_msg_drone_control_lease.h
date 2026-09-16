#pragma once
// MESSAGE DRONE_CONTROL_LEASE PACKING

#define MAVLINK_MSG_ID_DRONE_CONTROL_LEASE 51004


typedef struct __mavlink_drone_control_lease_t {
 uint32_t boot_id; /*<  STM32 boot identity observed by the Pi.*/
 uint32_t mission_id; /*<  Mission currently supervised, zero when none.*/
 uint32_t lease_id; /*<  Identity changed whenever a new operator lease is acquired.*/
 uint16_t timeout_ms; /*< [ms] Validity measured from local receipt time.*/
 uint8_t flags; /*<  Lease state flags.*/
 uint8_t target_system; /*<  System that validates the lease.*/
 uint8_t target_component; /*<  Component that validates the lease.*/
} mavlink_drone_control_lease_t;

#define MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN 17
#define MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN 17
#define MAVLINK_MSG_ID_51004_LEN 17
#define MAVLINK_MSG_ID_51004_MIN_LEN 17

#define MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_CRC 110
#define MAVLINK_MSG_ID_51004_CRC 110



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_CONTROL_LEASE { \
    51004, \
    "DRONE_CONTROL_LEASE", \
    7, \
    {  { "boot_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_control_lease_t, boot_id) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_control_lease_t, mission_id) }, \
         { "lease_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_control_lease_t, lease_id) }, \
         { "timeout_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_drone_control_lease_t, timeout_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_drone_control_lease_t, flags) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_drone_control_lease_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_drone_control_lease_t, target_component) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_CONTROL_LEASE { \
    "DRONE_CONTROL_LEASE", \
    7, \
    {  { "boot_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_control_lease_t, boot_id) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_drone_control_lease_t, mission_id) }, \
         { "lease_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_drone_control_lease_t, lease_id) }, \
         { "timeout_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_drone_control_lease_t, timeout_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_drone_control_lease_t, flags) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_drone_control_lease_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_drone_control_lease_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_control_lease message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param boot_id  STM32 boot identity observed by the Pi.
 * @param mission_id  Mission currently supervised, zero when none.
 * @param lease_id  Identity changed whenever a new operator lease is acquired.
 * @param timeout_ms [ms] Validity measured from local receipt time.
 * @param flags  Lease state flags.
 * @param target_system  System that validates the lease.
 * @param target_component  Component that validates the lease.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_control_lease_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t boot_id, uint32_t mission_id, uint32_t lease_id, uint16_t timeout_ms, uint8_t flags, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, lease_id);
    _mav_put_uint16_t(buf, 12, timeout_ms);
    _mav_put_uint8_t(buf, 14, flags);
    _mav_put_uint8_t(buf, 15, target_system);
    _mav_put_uint8_t(buf, 16, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN);
#else
    mavlink_drone_control_lease_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.lease_id = lease_id;
    packet.timeout_ms = timeout_ms;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_CONTROL_LEASE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_CRC);
}

/**
 * @brief Pack a drone_control_lease message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param boot_id  STM32 boot identity observed by the Pi.
 * @param mission_id  Mission currently supervised, zero when none.
 * @param lease_id  Identity changed whenever a new operator lease is acquired.
 * @param timeout_ms [ms] Validity measured from local receipt time.
 * @param flags  Lease state flags.
 * @param target_system  System that validates the lease.
 * @param target_component  Component that validates the lease.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_control_lease_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t boot_id, uint32_t mission_id, uint32_t lease_id, uint16_t timeout_ms, uint8_t flags, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, lease_id);
    _mav_put_uint16_t(buf, 12, timeout_ms);
    _mav_put_uint8_t(buf, 14, flags);
    _mav_put_uint8_t(buf, 15, target_system);
    _mav_put_uint8_t(buf, 16, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN);
#else
    mavlink_drone_control_lease_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.lease_id = lease_id;
    packet.timeout_ms = timeout_ms;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_CONTROL_LEASE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN);
#endif
}

/**
 * @brief Pack a drone_control_lease message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boot_id  STM32 boot identity observed by the Pi.
 * @param mission_id  Mission currently supervised, zero when none.
 * @param lease_id  Identity changed whenever a new operator lease is acquired.
 * @param timeout_ms [ms] Validity measured from local receipt time.
 * @param flags  Lease state flags.
 * @param target_system  System that validates the lease.
 * @param target_component  Component that validates the lease.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_control_lease_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t boot_id,uint32_t mission_id,uint32_t lease_id,uint16_t timeout_ms,uint8_t flags,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, lease_id);
    _mav_put_uint16_t(buf, 12, timeout_ms);
    _mav_put_uint8_t(buf, 14, flags);
    _mav_put_uint8_t(buf, 15, target_system);
    _mav_put_uint8_t(buf, 16, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN);
#else
    mavlink_drone_control_lease_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.lease_id = lease_id;
    packet.timeout_ms = timeout_ms;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_CONTROL_LEASE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_CRC);
}

/**
 * @brief Encode a drone_control_lease struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_control_lease C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_control_lease_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_control_lease_t* drone_control_lease)
{
    return mavlink_msg_drone_control_lease_pack(system_id, component_id, msg, drone_control_lease->boot_id, drone_control_lease->mission_id, drone_control_lease->lease_id, drone_control_lease->timeout_ms, drone_control_lease->flags, drone_control_lease->target_system, drone_control_lease->target_component);
}

/**
 * @brief Encode a drone_control_lease struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_control_lease C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_control_lease_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_control_lease_t* drone_control_lease)
{
    return mavlink_msg_drone_control_lease_pack_chan(system_id, component_id, chan, msg, drone_control_lease->boot_id, drone_control_lease->mission_id, drone_control_lease->lease_id, drone_control_lease->timeout_ms, drone_control_lease->flags, drone_control_lease->target_system, drone_control_lease->target_component);
}

/**
 * @brief Encode a drone_control_lease struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param drone_control_lease C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_control_lease_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_drone_control_lease_t* drone_control_lease)
{
    return mavlink_msg_drone_control_lease_pack_status(system_id, component_id, _status, msg,  drone_control_lease->boot_id, drone_control_lease->mission_id, drone_control_lease->lease_id, drone_control_lease->timeout_ms, drone_control_lease->flags, drone_control_lease->target_system, drone_control_lease->target_component);
}

/**
 * @brief Send a drone_control_lease message
 * @param chan MAVLink channel to send the message
 *
 * @param boot_id  STM32 boot identity observed by the Pi.
 * @param mission_id  Mission currently supervised, zero when none.
 * @param lease_id  Identity changed whenever a new operator lease is acquired.
 * @param timeout_ms [ms] Validity measured from local receipt time.
 * @param flags  Lease state flags.
 * @param target_system  System that validates the lease.
 * @param target_component  Component that validates the lease.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_control_lease_send(mavlink_channel_t chan, uint32_t boot_id, uint32_t mission_id, uint32_t lease_id, uint16_t timeout_ms, uint8_t flags, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN];
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, lease_id);
    _mav_put_uint16_t(buf, 12, timeout_ms);
    _mav_put_uint8_t(buf, 14, flags);
    _mav_put_uint8_t(buf, 15, target_system);
    _mav_put_uint8_t(buf, 16, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE, buf, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_CRC);
#else
    mavlink_drone_control_lease_t packet;
    packet.boot_id = boot_id;
    packet.mission_id = mission_id;
    packet.lease_id = lease_id;
    packet.timeout_ms = timeout_ms;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE, (const char *)&packet, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_CRC);
#endif
}

/**
 * @brief Send a drone_control_lease message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_control_lease_send_struct(mavlink_channel_t chan, const mavlink_drone_control_lease_t* drone_control_lease)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_control_lease_send(chan, drone_control_lease->boot_id, drone_control_lease->mission_id, drone_control_lease->lease_id, drone_control_lease->timeout_ms, drone_control_lease->flags, drone_control_lease->target_system, drone_control_lease->target_component);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE, (const char *)drone_control_lease, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_control_lease_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t boot_id, uint32_t mission_id, uint32_t lease_id, uint16_t timeout_ms, uint8_t flags, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, boot_id);
    _mav_put_uint32_t(buf, 4, mission_id);
    _mav_put_uint32_t(buf, 8, lease_id);
    _mav_put_uint16_t(buf, 12, timeout_ms);
    _mav_put_uint8_t(buf, 14, flags);
    _mav_put_uint8_t(buf, 15, target_system);
    _mav_put_uint8_t(buf, 16, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE, buf, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_CRC);
#else
    mavlink_drone_control_lease_t *packet = (mavlink_drone_control_lease_t *)msgbuf;
    packet->boot_id = boot_id;
    packet->mission_id = mission_id;
    packet->lease_id = lease_id;
    packet->timeout_ms = timeout_ms;
    packet->flags = flags;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE, (const char *)packet, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_CONTROL_LEASE UNPACKING


/**
 * @brief Get field boot_id from drone_control_lease message
 *
 * @return  STM32 boot identity observed by the Pi.
 */
static inline uint32_t mavlink_msg_drone_control_lease_get_boot_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field mission_id from drone_control_lease message
 *
 * @return  Mission currently supervised, zero when none.
 */
static inline uint32_t mavlink_msg_drone_control_lease_get_mission_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field lease_id from drone_control_lease message
 *
 * @return  Identity changed whenever a new operator lease is acquired.
 */
static inline uint32_t mavlink_msg_drone_control_lease_get_lease_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field timeout_ms from drone_control_lease message
 *
 * @return [ms] Validity measured from local receipt time.
 */
static inline uint16_t mavlink_msg_drone_control_lease_get_timeout_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field flags from drone_control_lease message
 *
 * @return  Lease state flags.
 */
static inline uint8_t mavlink_msg_drone_control_lease_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field target_system from drone_control_lease message
 *
 * @return  System that validates the lease.
 */
static inline uint8_t mavlink_msg_drone_control_lease_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field target_component from drone_control_lease message
 *
 * @return  Component that validates the lease.
 */
static inline uint8_t mavlink_msg_drone_control_lease_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a drone_control_lease message into a struct
 *
 * @param msg The message to decode
 * @param drone_control_lease C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_control_lease_decode(const mavlink_message_t* msg, mavlink_drone_control_lease_t* drone_control_lease)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_control_lease->boot_id = mavlink_msg_drone_control_lease_get_boot_id(msg);
    drone_control_lease->mission_id = mavlink_msg_drone_control_lease_get_mission_id(msg);
    drone_control_lease->lease_id = mavlink_msg_drone_control_lease_get_lease_id(msg);
    drone_control_lease->timeout_ms = mavlink_msg_drone_control_lease_get_timeout_ms(msg);
    drone_control_lease->flags = mavlink_msg_drone_control_lease_get_flags(msg);
    drone_control_lease->target_system = mavlink_msg_drone_control_lease_get_target_system(msg);
    drone_control_lease->target_component = mavlink_msg_drone_control_lease_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN? msg->len : MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN;
        memset(drone_control_lease, 0, MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_LEN);
    memcpy(drone_control_lease, _MAV_PAYLOAD(msg), len);
#endif
}
