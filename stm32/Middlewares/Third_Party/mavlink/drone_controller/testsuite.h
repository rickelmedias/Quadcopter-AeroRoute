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

static void mavlink_test_drone_gain_row(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DRONE_GAIN_ROW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_drone_gain_row_t packet_in = {
        963497464,{ 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0 },161,228,39,106,173
    };
    mavlink_drone_gain_row_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.session_id = packet_in.session_id;
        packet1.matrix = packet_in.matrix;
        packet1.row = packet_in.row;
        packet1.count = packet_in.count;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.values, packet_in.values, sizeof(float)*12);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DRONE_GAIN_ROW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_row_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_drone_gain_row_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_row_pack(system_id, component_id, &msg , packet1.session_id , packet1.values , packet1.matrix , packet1.row , packet1.count , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_gain_row_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_row_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.session_id , packet1.values , packet1.matrix , packet1.row , packet1.count , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_gain_row_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_drone_gain_row_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_row_send(MAVLINK_COMM_1 , packet1.session_id , packet1.values , packet1.matrix , packet1.row , packet1.count , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_gain_row_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DRONE_GAIN_ROW") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DRONE_GAIN_ROW) != NULL);
#endif
}

static void mavlink_test_drone_gain_commit(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DRONE_GAIN_COMMIT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_drone_gain_commit_t packet_in = {
        963497464,963497672,29,96,163
    };
    mavlink_drone_gain_commit_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.session_id = packet_in.session_id;
        packet1.crc32 = packet_in.crc32;
        packet1.action = packet_in.action;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DRONE_GAIN_COMMIT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_commit_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_drone_gain_commit_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_commit_pack(system_id, component_id, &msg , packet1.session_id , packet1.crc32 , packet1.action , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_gain_commit_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_commit_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.session_id , packet1.crc32 , packet1.action , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_gain_commit_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_drone_gain_commit_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_commit_send(MAVLINK_COMM_1 , packet1.session_id , packet1.crc32 , packet1.action , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_gain_commit_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DRONE_GAIN_COMMIT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DRONE_GAIN_COMMIT) != NULL);
#endif
}

static void mavlink_test_drone_gain_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DRONE_GAIN_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_drone_gain_ack_t packet_in = {
        963497464,963497672,963497880,963498088,53,120,187,254,65
    };
    mavlink_drone_gain_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.session_id = packet_in.session_id;
        packet1.staged_crc32 = packet_in.staged_crc32;
        packet1.active_crc32 = packet_in.active_crc32;
        packet1.rows_mask = packet_in.rows_mask;
        packet1.action = packet_in.action;
        packet1.status = packet_in.status;
        packet1.custom = packet_in.custom;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DRONE_GAIN_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_drone_gain_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_ack_pack(system_id, component_id, &msg , packet1.session_id , packet1.staged_crc32 , packet1.active_crc32 , packet1.rows_mask , packet1.action , packet1.status , packet1.custom , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_gain_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.session_id , packet1.staged_crc32 , packet1.active_crc32 , packet1.rows_mask , packet1.action , packet1.status , packet1.custom , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_gain_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_drone_gain_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_gain_ack_send(MAVLINK_COMM_1 , packet1.session_id , packet1.staged_crc32 , packet1.active_crc32 , packet1.rows_mask , packet1.action , packet1.status , packet1.custom , packet1.target_system , packet1.target_component );
    mavlink_msg_drone_gain_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DRONE_GAIN_ACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DRONE_GAIN_ACK) != NULL);
#endif
}

static void mavlink_test_drone_ctrl_debug(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DRONE_CTRL_DEBUG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_drone_ctrl_debug_t packet_in = {
        963497464,963497672,{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0 },{ 325.0, 326.0, 327.0, 328.0, 329.0, 330.0, 331.0, 332.0, 333.0, 334.0, 335.0, 336.0 },{ 661.0, 662.0, 663.0, 664.0 },{ 773.0, 774.0, 775.0, 776.0 },{ 23683, 23684, 23685, 23686 },145,212,23
    };
    mavlink_drone_ctrl_debug_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.step = packet_in.step;
        packet1.flight_state = packet_in.flight_state;
        packet1.meas_flags = packet_in.meas_flags;
        packet1.ctrl_flags = packet_in.ctrl_flags;
        
        mav_array_memcpy(packet1.y, packet_in.y, sizeof(float)*9);
        mav_array_memcpy(packet1.x_hat, packet_in.x_hat, sizeof(float)*12);
        mav_array_memcpy(packet1.du, packet_in.du, sizeof(float)*4);
        mav_array_memcpy(packet1.ref, packet_in.ref, sizeof(float)*4);
        mav_array_memcpy(packet1.throttle, packet_in.throttle, sizeof(uint16_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DRONE_CTRL_DEBUG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_ctrl_debug_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_drone_ctrl_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_ctrl_debug_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.step , packet1.y , packet1.x_hat , packet1.du , packet1.ref , packet1.throttle , packet1.flight_state , packet1.meas_flags , packet1.ctrl_flags );
    mavlink_msg_drone_ctrl_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_ctrl_debug_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.step , packet1.y , packet1.x_hat , packet1.du , packet1.ref , packet1.throttle , packet1.flight_state , packet1.meas_flags , packet1.ctrl_flags );
    mavlink_msg_drone_ctrl_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_drone_ctrl_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_ctrl_debug_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.step , packet1.y , packet1.x_hat , packet1.du , packet1.ref , packet1.throttle , packet1.flight_state , packet1.meas_flags , packet1.ctrl_flags );
    mavlink_msg_drone_ctrl_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DRONE_CTRL_DEBUG") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DRONE_CTRL_DEBUG) != NULL);
#endif
}

static void mavlink_test_drone_controller(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_drone_protocol_info(system_id, component_id, last_msg);
    mavlink_test_drone_leg_command(system_id, component_id, last_msg);
    mavlink_test_drone_leg_ack(system_id, component_id, last_msg);
    mavlink_test_drone_leg_cancel(system_id, component_id, last_msg);
    mavlink_test_drone_control_lease(system_id, component_id, last_msg);
    mavlink_test_drone_gain_row(system_id, component_id, last_msg);
    mavlink_test_drone_gain_commit(system_id, component_id, last_msg);
    mavlink_test_drone_gain_ack(system_id, component_id, last_msg);
    mavlink_test_drone_ctrl_debug(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // DRONE_CONTROLLER_TESTSUITE_H
