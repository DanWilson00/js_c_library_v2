/** @file
 *    @brief MAVLink comm protocol testsuite generated from jetshark.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef JETSHARK_TESTSUITE_H
#define JETSHARK_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_jetshark(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_jetshark(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_ultramotion_port(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ULTRAMOTION_PORT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ultramotion_port_t packet_in = {
        17.0,45.0,73.0,101.0,53,120,187,254,65,132
    };
    mavlink_ultramotion_port_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pos_cmd = packet_in.pos_cmd;
        packet1.pos = packet_in.pos;
        packet1.current = packet_in.current;
        packet1.temp_degc = packet_in.temp_degc;
        packet1.over_torque = packet_in.over_torque;
        packet1.at_target = packet_in.at_target;
        packet1.following_error = packet_in.following_error;
        packet1.speed_below_threshold = packet_in.speed_below_threshold;
        packet1.rx_error = packet_in.rx_error;
        packet1.tx_error = packet_in.tx_error;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ULTRAMOTION_PORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ULTRAMOTION_PORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_port_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ultramotion_port_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_port_pack(system_id, component_id, &msg , packet1.pos_cmd , packet1.pos , packet1.current , packet1.temp_degc , packet1.over_torque , packet1.at_target , packet1.following_error , packet1.speed_below_threshold , packet1.rx_error , packet1.tx_error );
    mavlink_msg_ultramotion_port_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_port_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pos_cmd , packet1.pos , packet1.current , packet1.temp_degc , packet1.over_torque , packet1.at_target , packet1.following_error , packet1.speed_below_threshold , packet1.rx_error , packet1.tx_error );
    mavlink_msg_ultramotion_port_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ultramotion_port_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_port_send(MAVLINK_COMM_1 , packet1.pos_cmd , packet1.pos , packet1.current , packet1.temp_degc , packet1.over_torque , packet1.at_target , packet1.following_error , packet1.speed_below_threshold , packet1.rx_error , packet1.tx_error );
    mavlink_msg_ultramotion_port_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ULTRAMOTION_PORT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ULTRAMOTION_PORT) != NULL);
#endif
}

static void mavlink_test_ultramotion_sb(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ULTRAMOTION_SB >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ultramotion_sb_t packet_in = {
        17.0,45.0,73.0,101.0,53,120,187,254,65,132
    };
    mavlink_ultramotion_sb_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pos_cmd = packet_in.pos_cmd;
        packet1.pos = packet_in.pos;
        packet1.current = packet_in.current;
        packet1.temp_degc = packet_in.temp_degc;
        packet1.over_torque = packet_in.over_torque;
        packet1.at_target = packet_in.at_target;
        packet1.following_error = packet_in.following_error;
        packet1.speed_below_threshold = packet_in.speed_below_threshold;
        packet1.rx_error = packet_in.rx_error;
        packet1.tx_error = packet_in.tx_error;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_sb_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ultramotion_sb_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_sb_pack(system_id, component_id, &msg , packet1.pos_cmd , packet1.pos , packet1.current , packet1.temp_degc , packet1.over_torque , packet1.at_target , packet1.following_error , packet1.speed_below_threshold , packet1.rx_error , packet1.tx_error );
    mavlink_msg_ultramotion_sb_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_sb_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pos_cmd , packet1.pos , packet1.current , packet1.temp_degc , packet1.over_torque , packet1.at_target , packet1.following_error , packet1.speed_below_threshold , packet1.rx_error , packet1.tx_error );
    mavlink_msg_ultramotion_sb_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ultramotion_sb_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_sb_send(MAVLINK_COMM_1 , packet1.pos_cmd , packet1.pos , packet1.current , packet1.temp_degc , packet1.over_torque , packet1.at_target , packet1.following_error , packet1.speed_below_threshold , packet1.rx_error , packet1.tx_error );
    mavlink_msg_ultramotion_sb_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ULTRAMOTION_SB") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ULTRAMOTION_SB) != NULL);
#endif
}

static void mavlink_test_ultramotion_rear(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ULTRAMOTION_REAR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ultramotion_rear_t packet_in = {
        17.0,45.0,73.0,101.0,53,120,187,254,65,132
    };
    mavlink_ultramotion_rear_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pos_cmd = packet_in.pos_cmd;
        packet1.pos = packet_in.pos;
        packet1.current = packet_in.current;
        packet1.temp_degc = packet_in.temp_degc;
        packet1.over_torque = packet_in.over_torque;
        packet1.at_target = packet_in.at_target;
        packet1.following_error = packet_in.following_error;
        packet1.speed_below_threshold = packet_in.speed_below_threshold;
        packet1.rx_error = packet_in.rx_error;
        packet1.tx_error = packet_in.tx_error;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ULTRAMOTION_REAR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ULTRAMOTION_REAR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_rear_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ultramotion_rear_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_rear_pack(system_id, component_id, &msg , packet1.pos_cmd , packet1.pos , packet1.current , packet1.temp_degc , packet1.over_torque , packet1.at_target , packet1.following_error , packet1.speed_below_threshold , packet1.rx_error , packet1.tx_error );
    mavlink_msg_ultramotion_rear_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_rear_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pos_cmd , packet1.pos , packet1.current , packet1.temp_degc , packet1.over_torque , packet1.at_target , packet1.following_error , packet1.speed_below_threshold , packet1.rx_error , packet1.tx_error );
    mavlink_msg_ultramotion_rear_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ultramotion_rear_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ultramotion_rear_send(MAVLINK_COMM_1 , packet1.pos_cmd , packet1.pos , packet1.current , packet1.temp_degc , packet1.over_torque , packet1.at_target , packet1.following_error , packet1.speed_below_threshold , packet1.rx_error , packet1.tx_error );
    mavlink_msg_ultramotion_rear_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ULTRAMOTION_REAR") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ULTRAMOTION_REAR) != NULL);
#endif
}

static void mavlink_test_jetshark(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ultramotion_port(system_id, component_id, last_msg);
    mavlink_test_ultramotion_sb(system_id, component_id, last_msg);
    mavlink_test_ultramotion_rear(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // JETSHARK_TESTSUITE_H
