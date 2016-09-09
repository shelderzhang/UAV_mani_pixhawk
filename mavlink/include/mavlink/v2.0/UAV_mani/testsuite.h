/** @file
 *	@brief MAVLink comm protocol testsuite generated from UAV_mani.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef UAV_MANI_TESTSUITE_H
#define UAV_MANI_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_UAV_mani(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_UAV_mani(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_endeff_frame(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ENDEFF_FRAME >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_endeff_frame_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,'A',228
    };
	mavlink_endeff_frame_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.gripper_posi = packet_in.gripper_posi;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.roll_rate = packet_in.roll_rate;
        packet1.pitch_rate = packet_in.pitch_rate;
        packet1.yaw_rate = packet_in.yaw_rate;
        packet1.arm_enable = packet_in.arm_enable;
        packet1.gripper_status = packet_in.gripper_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ENDEFF_FRAME_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ENDEFF_FRAME_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_endeff_frame_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_endeff_frame_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_endeff_frame_pack(system_id, component_id, &msg , packet1.arm_enable , packet1.gripper_status , packet1.gripper_posi , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.vx , packet1.vy , packet1.vz , packet1.roll_rate , packet1.pitch_rate , packet1.yaw_rate );
	mavlink_msg_endeff_frame_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_endeff_frame_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.arm_enable , packet1.gripper_status , packet1.gripper_posi , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.vx , packet1.vy , packet1.vz , packet1.roll_rate , packet1.pitch_rate , packet1.yaw_rate );
	mavlink_msg_endeff_frame_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_endeff_frame_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_endeff_frame_send(MAVLINK_COMM_1 , packet1.arm_enable , packet1.gripper_status , packet1.gripper_posi , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.vx , packet1.vy , packet1.vz , packet1.roll_rate , packet1.pitch_rate , packet1.yaw_rate );
	mavlink_msg_endeff_frame_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_manipulator_joint_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_manipulator_joint_status_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0
    };
	mavlink_manipulator_joint_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.joint_posi_1 = packet_in.joint_posi_1;
        packet1.joint_posi_2 = packet_in.joint_posi_2;
        packet1.joint_posi_3 = packet_in.joint_posi_3;
        packet1.joint_posi_4 = packet_in.joint_posi_4;
        packet1.joint_posi_5 = packet_in.joint_posi_5;
        packet1.joint_posi_6 = packet_in.joint_posi_6;
        packet1.joint_posi_7 = packet_in.joint_posi_7;
        packet1.joint_rate_1 = packet_in.joint_rate_1;
        packet1.joint_rate_2 = packet_in.joint_rate_2;
        packet1.joint_rate_3 = packet_in.joint_rate_3;
        packet1.joint_rate_4 = packet_in.joint_rate_4;
        packet1.joint_rate_5 = packet_in.joint_rate_5;
        packet1.joint_rate_6 = packet_in.joint_rate_6;
        packet1.joint_rate_7 = packet_in.joint_rate_7;
        packet1.torque_1 = packet_in.torque_1;
        packet1.torque_2 = packet_in.torque_2;
        packet1.torque_3 = packet_in.torque_3;
        packet1.torque_4 = packet_in.torque_4;
        packet1.torque_5 = packet_in.torque_5;
        packet1.torque_6 = packet_in.torque_6;
        packet1.torque_7 = packet_in.torque_7;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_manipulator_joint_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_manipulator_joint_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_manipulator_joint_status_pack(system_id, component_id, &msg , packet1.joint_posi_1 , packet1.joint_posi_2 , packet1.joint_posi_3 , packet1.joint_posi_4 , packet1.joint_posi_5 , packet1.joint_posi_6 , packet1.joint_posi_7 , packet1.joint_rate_1 , packet1.joint_rate_2 , packet1.joint_rate_3 , packet1.joint_rate_4 , packet1.joint_rate_5 , packet1.joint_rate_6 , packet1.joint_rate_7 , packet1.torque_1 , packet1.torque_2 , packet1.torque_3 , packet1.torque_4 , packet1.torque_5 , packet1.torque_6 , packet1.torque_7 );
	mavlink_msg_manipulator_joint_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_manipulator_joint_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.joint_posi_1 , packet1.joint_posi_2 , packet1.joint_posi_3 , packet1.joint_posi_4 , packet1.joint_posi_5 , packet1.joint_posi_6 , packet1.joint_posi_7 , packet1.joint_rate_1 , packet1.joint_rate_2 , packet1.joint_rate_3 , packet1.joint_rate_4 , packet1.joint_rate_5 , packet1.joint_rate_6 , packet1.joint_rate_7 , packet1.torque_1 , packet1.torque_2 , packet1.torque_3 , packet1.torque_4 , packet1.torque_5 , packet1.torque_6 , packet1.torque_7 );
	mavlink_msg_manipulator_joint_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_manipulator_joint_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_manipulator_joint_status_send(MAVLINK_COMM_1 , packet1.joint_posi_1 , packet1.joint_posi_2 , packet1.joint_posi_3 , packet1.joint_posi_4 , packet1.joint_posi_5 , packet1.joint_posi_6 , packet1.joint_posi_7 , packet1.joint_rate_1 , packet1.joint_rate_2 , packet1.joint_rate_3 , packet1.joint_rate_4 , packet1.joint_rate_5 , packet1.joint_rate_6 , packet1.joint_rate_7 , packet1.torque_1 , packet1.torque_2 , packet1.torque_3 , packet1.torque_4 , packet1.torque_5 , packet1.torque_6 , packet1.torque_7 );
	mavlink_msg_manipulator_joint_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_UAV_mani(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_endeff_frame(system_id, component_id, last_msg);
	mavlink_test_manipulator_joint_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // UAV_MANI_TESTSUITE_H
