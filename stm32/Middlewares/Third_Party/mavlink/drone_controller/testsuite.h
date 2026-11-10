/** @file
 *    @brief MAVLink comm protocol testsuite generated from drone_controller.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef DRONE_CONTROLLER_TESTSUITE_H
#define DRONE_CONTROLLER_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_drone_controller(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_drone_controller(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_drone_protocol_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_drone_protocol_info_t packet_in = {
        963497464,963497672,963497880,41,108,175,242,53
    };
    mavlink_drone_protocol_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.boot_id = packet_in.boot_id;
        packet1.capabilities = packet_in.capabilities;
        packet1.max_leg_timeout_ms = packet_in.max_leg_timeout_ms;
        packet1.protocol_major = packet_in.protocol_major;
        packet1.protocol_minor = packet_in.protocol_minor;
        packet1.flight_state = packet_in.flight_state;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_protocol_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_drone_protocol_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_protocol_info_pack(system_id, component_id, &msg , packet1.boot_id , packet1.capabilities , packet1.max_leg_timeout_ms , packet1.protocol_major , packet1.protocol_minor , packet1.flight_state , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_protocol_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_protocol_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.boot_id , packet1.capabilities , packet1.max_leg_timeout_ms , packet1.protocol_major , packet1.protocol_minor , packet1.flight_state , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_protocol_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_drone_protocol_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_protocol_info_send(MAVLINK_COMM_1 , packet1.boot_id , packet1.capabilities , packet1.max_leg_timeout_ms , packet1.protocol_major , packet1.protocol_minor , packet1.flight_state , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_protocol_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DRONE_PROTOCOL_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DRONE_PROTOCOL_INFO) != NULL);
#endif
}

static void mavlink_test_drone_leg_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DRONE_LEG_COMMAND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_drone_leg_command_t packet_in = {
        963497464,963497672,963497880,963498088,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,19731,19835,161,228,39,106
    };
    mavlink_drone_leg_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.boot_id = packet_in.boot_id;
        packet1.mission_id = packet_in.mission_id;
        packet1.command_id = packet_in.command_id;
        packet1.timeout_ms = packet_in.timeout_ms;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.yaw = packet_in.yaw;
        packet1.speed = packet_in.speed;
        packet1.acceleration = packet_in.acceleration;
        packet1.position_tolerance = packet_in.position_tolerance;
        packet1.yaw_tolerance = packet_in.yaw_tolerance;
        packet1.leg_index = packet_in.leg_index;
        packet1.flags = packet_in.flags;
        packet1.action = packet_in.action;
        packet1.frame = packet_in.frame;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DRONE_LEG_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_command_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_drone_leg_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_command_pack(system_id, component_id, &msg , packet1.boot_id , packet1.mission_id , packet1.command_id , packet1.timeout_ms , packet1.x , packet1.y , packet1.z , packet1.yaw , packet1.speed , packet1.acceleration , packet1.position_tolerance , packet1.yaw_tolerance , packet1.leg_index , packet1.flags , packet1.action , packet1.frame , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_leg_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.boot_id , packet1.mission_id , packet1.command_id , packet1.timeout_ms , packet1.x , packet1.y , packet1.z , packet1.yaw , packet1.speed , packet1.acceleration , packet1.position_tolerance , packet1.yaw_tolerance , packet1.leg_index , packet1.flags , packet1.action , packet1.frame , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_leg_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_drone_leg_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_command_send(MAVLINK_COMM_1 , packet1.boot_id , packet1.mission_id , packet1.command_id , packet1.timeout_ms , packet1.x , packet1.y , packet1.z , packet1.yaw , packet1.speed , packet1.acceleration , packet1.position_tolerance , packet1.yaw_tolerance , packet1.leg_index , packet1.flags , packet1.action , packet1.frame , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_leg_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DRONE_LEG_COMMAND") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DRONE_LEG_COMMAND) != NULL);
#endif
}

static void mavlink_test_drone_leg_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DRONE_LEG_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_drone_leg_ack_t packet_in = {
        963497464,963497672,963497880,963498088,129.0,18275,18379,77,144,211,22,89
    };
    mavlink_drone_leg_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.boot_id = packet_in.boot_id;
        packet1.mission_id = packet_in.mission_id;
        packet1.command_id = packet_in.command_id;
        packet1.eta_ms = packet_in.eta_ms;
        packet1.remaining_distance = packet_in.remaining_distance;
        packet1.leg_index = packet_in.leg_index;
        packet1.reason = packet_in.reason;
        packet1.result = packet_in.result;
        packet1.progress = packet_in.progress;
        packet1.flight_state = packet_in.flight_state;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DRONE_LEG_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_drone_leg_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_ack_pack(system_id, component_id, &msg , packet1.boot_id , packet1.mission_id , packet1.command_id , packet1.eta_ms , packet1.remaining_distance , packet1.leg_index , packet1.reason , packet1.result , packet1.progress , packet1.flight_state , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_leg_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.boot_id , packet1.mission_id , packet1.command_id , packet1.eta_ms , packet1.remaining_distance , packet1.leg_index , packet1.reason , packet1.result , packet1.progress , packet1.flight_state , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_leg_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_drone_leg_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_ack_send(MAVLINK_COMM_1 , packet1.boot_id , packet1.mission_id , packet1.command_id , packet1.eta_ms , packet1.remaining_distance , packet1.leg_index , packet1.reason , packet1.result , packet1.progress , packet1.flight_state , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_leg_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DRONE_LEG_ACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DRONE_LEG_ACK) != NULL);
#endif
}

static void mavlink_test_drone_leg_cancel(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DRONE_LEG_CANCEL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_drone_leg_cancel_t packet_in = {
        963497464,963497672,963497880,17859,175,242
    };
    mavlink_drone_leg_cancel_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.boot_id = packet_in.boot_id;
        packet1.mission_id = packet_in.mission_id;
        packet1.command_id = packet_in.command_id;
        packet1.leg_index = packet_in.leg_index;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DRONE_LEG_CANCEL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_cancel_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_drone_leg_cancel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_cancel_pack(system_id, component_id, &msg , packet1.boot_id , packet1.mission_id , packet1.command_id , packet1.leg_index , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_leg_cancel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_cancel_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.boot_id , packet1.mission_id , packet1.command_id , packet1.leg_index , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_leg_cancel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_drone_leg_cancel_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_leg_cancel_send(MAVLINK_COMM_1 , packet1.boot_id , packet1.mission_id , packet1.command_id , packet1.leg_index , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_leg_cancel_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DRONE_LEG_CANCEL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DRONE_LEG_CANCEL) != NULL);
#endif
}

static void mavlink_test_drone_control_lease(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DRONE_CONTROL_LEASE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_drone_control_lease_t packet_in = {
        963497464,963497672,963497880,17859,175,242,53
    };
    mavlink_drone_control_lease_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.boot_id = packet_in.boot_id;
        packet1.mission_id = packet_in.mission_id;
        packet1.lease_id = packet_in.lease_id;
        packet1.timeout_ms = packet_in.timeout_ms;
        packet1.flags = packet_in.flags;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DRONE_CONTROL_LEASE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_control_lease_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_drone_control_lease_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_control_lease_pack(system_id, component_id, &msg , packet1.boot_id , packet1.mission_id , packet1.lease_id , packet1.timeout_ms , packet1.flags , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_control_lease_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_control_lease_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.boot_id , packet1.mission_id , packet1.lease_id , packet1.timeout_ms , packet1.flags , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_control_lease_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_drone_control_lease_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_control_lease_send(MAVLINK_COMM_1 , packet1.boot_id , packet1.mission_id , packet1.lease_id , packet1.timeout_ms , packet1.flags , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_control_lease_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DRONE_CONTROL_LEASE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DRONE_CONTROL_LEASE) != NULL);
#endif
}

static void mavlink_test_drone_controller(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_drone_protocol_info(system_id, component_id, last_msg);
    mavlink_test_drone_leg_command(system_id, component_id, last_msg);
    mavlink_test_drone_leg_ack(system_id, component_id, last_msg);
    mavlink_test_drone_leg_cancel(system_id, component_id, last_msg);
    mavlink_test_drone_control_lease(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // DRONE_CONTROLLER_TESTSUITE_H
