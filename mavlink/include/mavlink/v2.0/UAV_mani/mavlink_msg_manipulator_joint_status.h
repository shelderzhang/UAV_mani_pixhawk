// MESSAGE MANIPULATOR_JOINT_STATUS PACKING

#define MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS 182

MAVPACKED(
typedef struct __mavlink_manipulator_joint_status_t {
 float joint_posi_1; /*< Position of jiont 1 in rad*/
 float joint_posi_2; /*< Position of jiont 2 in rad*/
 float joint_posi_3; /*< Position of jiont 3 in rad*/
 float joint_posi_4; /*< Position of jiont 4 in rad*/
 float joint_posi_5; /*< Position of jiont 5 in rad*/
 float joint_posi_6; /*< Position of jiont 6 in rad*/
 float joint_posi_7; /*< Position of jiont 7 in rad*/
 float joint_rate_1; /*< Rate of jiont 1 in rad/s*/
 float joint_rate_2; /*< Rate of jiont 2 in rad/s*/
 float joint_rate_3; /*< Rate of jiont 3 in rad/s*/
 float joint_rate_4; /*< Rate of jiont 4 in rad/s*/
 float joint_rate_5; /*< Rate of jiont 5 in rad/s*/
 float joint_rate_6; /*< Rate of jiont 6 in rad/s*/
 float joint_rate_7; /*< Rate of jiont 7 in rad/s*/
 float torque_1; /*< torque of jiont 1 in N*m*/
 float torque_2; /*< torque of jiont 2 in N*m*/
 float torque_3; /*< torque of jiont 3 in N*m*/
 float torque_4; /*< torque of jiont 4 in N*m*/
 float torque_5; /*< torque of jiont 5 in N*m*/
 float torque_6; /*< torque of jiont 6 in N*m*/
 float torque_7; /*< torque of jiont 7 in N*m*/
}) mavlink_manipulator_joint_status_t;

#define MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN 84
#define MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_MIN_LEN 84
#define MAVLINK_MSG_ID_182_LEN 84
#define MAVLINK_MSG_ID_182_MIN_LEN 84

#define MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_CRC 62
#define MAVLINK_MSG_ID_182_CRC 62



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MANIPULATOR_JOINT_STATUS { \
	182, \
	"MANIPULATOR_JOINT_STATUS", \
	21, \
	{  { "joint_posi_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_manipulator_joint_status_t, joint_posi_1) }, \
         { "joint_posi_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_manipulator_joint_status_t, joint_posi_2) }, \
         { "joint_posi_3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_manipulator_joint_status_t, joint_posi_3) }, \
         { "joint_posi_4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_manipulator_joint_status_t, joint_posi_4) }, \
         { "joint_posi_5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_manipulator_joint_status_t, joint_posi_5) }, \
         { "joint_posi_6", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_manipulator_joint_status_t, joint_posi_6) }, \
         { "joint_posi_7", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_manipulator_joint_status_t, joint_posi_7) }, \
         { "joint_rate_1", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_manipulator_joint_status_t, joint_rate_1) }, \
         { "joint_rate_2", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_manipulator_joint_status_t, joint_rate_2) }, \
         { "joint_rate_3", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_manipulator_joint_status_t, joint_rate_3) }, \
         { "joint_rate_4", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_manipulator_joint_status_t, joint_rate_4) }, \
         { "joint_rate_5", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_manipulator_joint_status_t, joint_rate_5) }, \
         { "joint_rate_6", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_manipulator_joint_status_t, joint_rate_6) }, \
         { "joint_rate_7", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_manipulator_joint_status_t, joint_rate_7) }, \
         { "torque_1", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_manipulator_joint_status_t, torque_1) }, \
         { "torque_2", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_manipulator_joint_status_t, torque_2) }, \
         { "torque_3", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_manipulator_joint_status_t, torque_3) }, \
         { "torque_4", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_manipulator_joint_status_t, torque_4) }, \
         { "torque_5", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_manipulator_joint_status_t, torque_5) }, \
         { "torque_6", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_manipulator_joint_status_t, torque_6) }, \
         { "torque_7", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_manipulator_joint_status_t, torque_7) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MANIPULATOR_JOINT_STATUS { \
	"MANIPULATOR_JOINT_STATUS", \
	21, \
	{  { "joint_posi_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_manipulator_joint_status_t, joint_posi_1) }, \
         { "joint_posi_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_manipulator_joint_status_t, joint_posi_2) }, \
         { "joint_posi_3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_manipulator_joint_status_t, joint_posi_3) }, \
         { "joint_posi_4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_manipulator_joint_status_t, joint_posi_4) }, \
         { "joint_posi_5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_manipulator_joint_status_t, joint_posi_5) }, \
         { "joint_posi_6", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_manipulator_joint_status_t, joint_posi_6) }, \
         { "joint_posi_7", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_manipulator_joint_status_t, joint_posi_7) }, \
         { "joint_rate_1", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_manipulator_joint_status_t, joint_rate_1) }, \
         { "joint_rate_2", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_manipulator_joint_status_t, joint_rate_2) }, \
         { "joint_rate_3", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_manipulator_joint_status_t, joint_rate_3) }, \
         { "joint_rate_4", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_manipulator_joint_status_t, joint_rate_4) }, \
         { "joint_rate_5", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_manipulator_joint_status_t, joint_rate_5) }, \
         { "joint_rate_6", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_manipulator_joint_status_t, joint_rate_6) }, \
         { "joint_rate_7", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_manipulator_joint_status_t, joint_rate_7) }, \
         { "torque_1", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_manipulator_joint_status_t, torque_1) }, \
         { "torque_2", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_manipulator_joint_status_t, torque_2) }, \
         { "torque_3", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_manipulator_joint_status_t, torque_3) }, \
         { "torque_4", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_manipulator_joint_status_t, torque_4) }, \
         { "torque_5", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_manipulator_joint_status_t, torque_5) }, \
         { "torque_6", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_manipulator_joint_status_t, torque_6) }, \
         { "torque_7", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_manipulator_joint_status_t, torque_7) }, \
         } \
}
#endif

/**
 * @brief Pack a manipulator_joint_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param joint_posi_1 Position of jiont 1 in rad
 * @param joint_posi_2 Position of jiont 2 in rad
 * @param joint_posi_3 Position of jiont 3 in rad
 * @param joint_posi_4 Position of jiont 4 in rad
 * @param joint_posi_5 Position of jiont 5 in rad
 * @param joint_posi_6 Position of jiont 6 in rad
 * @param joint_posi_7 Position of jiont 7 in rad
 * @param joint_rate_1 Rate of jiont 1 in rad/s
 * @param joint_rate_2 Rate of jiont 2 in rad/s
 * @param joint_rate_3 Rate of jiont 3 in rad/s
 * @param joint_rate_4 Rate of jiont 4 in rad/s
 * @param joint_rate_5 Rate of jiont 5 in rad/s
 * @param joint_rate_6 Rate of jiont 6 in rad/s
 * @param joint_rate_7 Rate of jiont 7 in rad/s
 * @param torque_1 torque of jiont 1 in N*m
 * @param torque_2 torque of jiont 2 in N*m
 * @param torque_3 torque of jiont 3 in N*m
 * @param torque_4 torque of jiont 4 in N*m
 * @param torque_5 torque of jiont 5 in N*m
 * @param torque_6 torque of jiont 6 in N*m
 * @param torque_7 torque of jiont 7 in N*m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_manipulator_joint_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float joint_posi_1, float joint_posi_2, float joint_posi_3, float joint_posi_4, float joint_posi_5, float joint_posi_6, float joint_posi_7, float joint_rate_1, float joint_rate_2, float joint_rate_3, float joint_rate_4, float joint_rate_5, float joint_rate_6, float joint_rate_7, float torque_1, float torque_2, float torque_3, float torque_4, float torque_5, float torque_6, float torque_7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN];
	_mav_put_float(buf, 0, joint_posi_1);
	_mav_put_float(buf, 4, joint_posi_2);
	_mav_put_float(buf, 8, joint_posi_3);
	_mav_put_float(buf, 12, joint_posi_4);
	_mav_put_float(buf, 16, joint_posi_5);
	_mav_put_float(buf, 20, joint_posi_6);
	_mav_put_float(buf, 24, joint_posi_7);
	_mav_put_float(buf, 28, joint_rate_1);
	_mav_put_float(buf, 32, joint_rate_2);
	_mav_put_float(buf, 36, joint_rate_3);
	_mav_put_float(buf, 40, joint_rate_4);
	_mav_put_float(buf, 44, joint_rate_5);
	_mav_put_float(buf, 48, joint_rate_6);
	_mav_put_float(buf, 52, joint_rate_7);
	_mav_put_float(buf, 56, torque_1);
	_mav_put_float(buf, 60, torque_2);
	_mav_put_float(buf, 64, torque_3);
	_mav_put_float(buf, 68, torque_4);
	_mav_put_float(buf, 72, torque_5);
	_mav_put_float(buf, 76, torque_6);
	_mav_put_float(buf, 80, torque_7);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN);
#else
	mavlink_manipulator_joint_status_t packet;
	packet.joint_posi_1 = joint_posi_1;
	packet.joint_posi_2 = joint_posi_2;
	packet.joint_posi_3 = joint_posi_3;
	packet.joint_posi_4 = joint_posi_4;
	packet.joint_posi_5 = joint_posi_5;
	packet.joint_posi_6 = joint_posi_6;
	packet.joint_posi_7 = joint_posi_7;
	packet.joint_rate_1 = joint_rate_1;
	packet.joint_rate_2 = joint_rate_2;
	packet.joint_rate_3 = joint_rate_3;
	packet.joint_rate_4 = joint_rate_4;
	packet.joint_rate_5 = joint_rate_5;
	packet.joint_rate_6 = joint_rate_6;
	packet.joint_rate_7 = joint_rate_7;
	packet.torque_1 = torque_1;
	packet.torque_2 = torque_2;
	packet.torque_3 = torque_3;
	packet.torque_4 = torque_4;
	packet.torque_5 = torque_5;
	packet.torque_6 = torque_6;
	packet.torque_7 = torque_7;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_CRC);
}

/**
 * @brief Pack a manipulator_joint_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param joint_posi_1 Position of jiont 1 in rad
 * @param joint_posi_2 Position of jiont 2 in rad
 * @param joint_posi_3 Position of jiont 3 in rad
 * @param joint_posi_4 Position of jiont 4 in rad
 * @param joint_posi_5 Position of jiont 5 in rad
 * @param joint_posi_6 Position of jiont 6 in rad
 * @param joint_posi_7 Position of jiont 7 in rad
 * @param joint_rate_1 Rate of jiont 1 in rad/s
 * @param joint_rate_2 Rate of jiont 2 in rad/s
 * @param joint_rate_3 Rate of jiont 3 in rad/s
 * @param joint_rate_4 Rate of jiont 4 in rad/s
 * @param joint_rate_5 Rate of jiont 5 in rad/s
 * @param joint_rate_6 Rate of jiont 6 in rad/s
 * @param joint_rate_7 Rate of jiont 7 in rad/s
 * @param torque_1 torque of jiont 1 in N*m
 * @param torque_2 torque of jiont 2 in N*m
 * @param torque_3 torque of jiont 3 in N*m
 * @param torque_4 torque of jiont 4 in N*m
 * @param torque_5 torque of jiont 5 in N*m
 * @param torque_6 torque of jiont 6 in N*m
 * @param torque_7 torque of jiont 7 in N*m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_manipulator_joint_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float joint_posi_1,float joint_posi_2,float joint_posi_3,float joint_posi_4,float joint_posi_5,float joint_posi_6,float joint_posi_7,float joint_rate_1,float joint_rate_2,float joint_rate_3,float joint_rate_4,float joint_rate_5,float joint_rate_6,float joint_rate_7,float torque_1,float torque_2,float torque_3,float torque_4,float torque_5,float torque_6,float torque_7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN];
	_mav_put_float(buf, 0, joint_posi_1);
	_mav_put_float(buf, 4, joint_posi_2);
	_mav_put_float(buf, 8, joint_posi_3);
	_mav_put_float(buf, 12, joint_posi_4);
	_mav_put_float(buf, 16, joint_posi_5);
	_mav_put_float(buf, 20, joint_posi_6);
	_mav_put_float(buf, 24, joint_posi_7);
	_mav_put_float(buf, 28, joint_rate_1);
	_mav_put_float(buf, 32, joint_rate_2);
	_mav_put_float(buf, 36, joint_rate_3);
	_mav_put_float(buf, 40, joint_rate_4);
	_mav_put_float(buf, 44, joint_rate_5);
	_mav_put_float(buf, 48, joint_rate_6);
	_mav_put_float(buf, 52, joint_rate_7);
	_mav_put_float(buf, 56, torque_1);
	_mav_put_float(buf, 60, torque_2);
	_mav_put_float(buf, 64, torque_3);
	_mav_put_float(buf, 68, torque_4);
	_mav_put_float(buf, 72, torque_5);
	_mav_put_float(buf, 76, torque_6);
	_mav_put_float(buf, 80, torque_7);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN);
#else
	mavlink_manipulator_joint_status_t packet;
	packet.joint_posi_1 = joint_posi_1;
	packet.joint_posi_2 = joint_posi_2;
	packet.joint_posi_3 = joint_posi_3;
	packet.joint_posi_4 = joint_posi_4;
	packet.joint_posi_5 = joint_posi_5;
	packet.joint_posi_6 = joint_posi_6;
	packet.joint_posi_7 = joint_posi_7;
	packet.joint_rate_1 = joint_rate_1;
	packet.joint_rate_2 = joint_rate_2;
	packet.joint_rate_3 = joint_rate_3;
	packet.joint_rate_4 = joint_rate_4;
	packet.joint_rate_5 = joint_rate_5;
	packet.joint_rate_6 = joint_rate_6;
	packet.joint_rate_7 = joint_rate_7;
	packet.torque_1 = torque_1;
	packet.torque_2 = torque_2;
	packet.torque_3 = torque_3;
	packet.torque_4 = torque_4;
	packet.torque_5 = torque_5;
	packet.torque_6 = torque_6;
	packet.torque_7 = torque_7;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_CRC);
}

/**
 * @brief Encode a manipulator_joint_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param manipulator_joint_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_manipulator_joint_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_manipulator_joint_status_t* manipulator_joint_status)
{
	return mavlink_msg_manipulator_joint_status_pack(system_id, component_id, msg, manipulator_joint_status->joint_posi_1, manipulator_joint_status->joint_posi_2, manipulator_joint_status->joint_posi_3, manipulator_joint_status->joint_posi_4, manipulator_joint_status->joint_posi_5, manipulator_joint_status->joint_posi_6, manipulator_joint_status->joint_posi_7, manipulator_joint_status->joint_rate_1, manipulator_joint_status->joint_rate_2, manipulator_joint_status->joint_rate_3, manipulator_joint_status->joint_rate_4, manipulator_joint_status->joint_rate_5, manipulator_joint_status->joint_rate_6, manipulator_joint_status->joint_rate_7, manipulator_joint_status->torque_1, manipulator_joint_status->torque_2, manipulator_joint_status->torque_3, manipulator_joint_status->torque_4, manipulator_joint_status->torque_5, manipulator_joint_status->torque_6, manipulator_joint_status->torque_7);
}

/**
 * @brief Encode a manipulator_joint_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param manipulator_joint_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_manipulator_joint_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_manipulator_joint_status_t* manipulator_joint_status)
{
	return mavlink_msg_manipulator_joint_status_pack_chan(system_id, component_id, chan, msg, manipulator_joint_status->joint_posi_1, manipulator_joint_status->joint_posi_2, manipulator_joint_status->joint_posi_3, manipulator_joint_status->joint_posi_4, manipulator_joint_status->joint_posi_5, manipulator_joint_status->joint_posi_6, manipulator_joint_status->joint_posi_7, manipulator_joint_status->joint_rate_1, manipulator_joint_status->joint_rate_2, manipulator_joint_status->joint_rate_3, manipulator_joint_status->joint_rate_4, manipulator_joint_status->joint_rate_5, manipulator_joint_status->joint_rate_6, manipulator_joint_status->joint_rate_7, manipulator_joint_status->torque_1, manipulator_joint_status->torque_2, manipulator_joint_status->torque_3, manipulator_joint_status->torque_4, manipulator_joint_status->torque_5, manipulator_joint_status->torque_6, manipulator_joint_status->torque_7);
}

/**
 * @brief Send a manipulator_joint_status message
 * @param chan MAVLink channel to send the message
 *
 * @param joint_posi_1 Position of jiont 1 in rad
 * @param joint_posi_2 Position of jiont 2 in rad
 * @param joint_posi_3 Position of jiont 3 in rad
 * @param joint_posi_4 Position of jiont 4 in rad
 * @param joint_posi_5 Position of jiont 5 in rad
 * @param joint_posi_6 Position of jiont 6 in rad
 * @param joint_posi_7 Position of jiont 7 in rad
 * @param joint_rate_1 Rate of jiont 1 in rad/s
 * @param joint_rate_2 Rate of jiont 2 in rad/s
 * @param joint_rate_3 Rate of jiont 3 in rad/s
 * @param joint_rate_4 Rate of jiont 4 in rad/s
 * @param joint_rate_5 Rate of jiont 5 in rad/s
 * @param joint_rate_6 Rate of jiont 6 in rad/s
 * @param joint_rate_7 Rate of jiont 7 in rad/s
 * @param torque_1 torque of jiont 1 in N*m
 * @param torque_2 torque of jiont 2 in N*m
 * @param torque_3 torque of jiont 3 in N*m
 * @param torque_4 torque of jiont 4 in N*m
 * @param torque_5 torque of jiont 5 in N*m
 * @param torque_6 torque of jiont 6 in N*m
 * @param torque_7 torque of jiont 7 in N*m
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_manipulator_joint_status_send(mavlink_channel_t chan, float joint_posi_1, float joint_posi_2, float joint_posi_3, float joint_posi_4, float joint_posi_5, float joint_posi_6, float joint_posi_7, float joint_rate_1, float joint_rate_2, float joint_rate_3, float joint_rate_4, float joint_rate_5, float joint_rate_6, float joint_rate_7, float torque_1, float torque_2, float torque_3, float torque_4, float torque_5, float torque_6, float torque_7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN];
	_mav_put_float(buf, 0, joint_posi_1);
	_mav_put_float(buf, 4, joint_posi_2);
	_mav_put_float(buf, 8, joint_posi_3);
	_mav_put_float(buf, 12, joint_posi_4);
	_mav_put_float(buf, 16, joint_posi_5);
	_mav_put_float(buf, 20, joint_posi_6);
	_mav_put_float(buf, 24, joint_posi_7);
	_mav_put_float(buf, 28, joint_rate_1);
	_mav_put_float(buf, 32, joint_rate_2);
	_mav_put_float(buf, 36, joint_rate_3);
	_mav_put_float(buf, 40, joint_rate_4);
	_mav_put_float(buf, 44, joint_rate_5);
	_mav_put_float(buf, 48, joint_rate_6);
	_mav_put_float(buf, 52, joint_rate_7);
	_mav_put_float(buf, 56, torque_1);
	_mav_put_float(buf, 60, torque_2);
	_mav_put_float(buf, 64, torque_3);
	_mav_put_float(buf, 68, torque_4);
	_mav_put_float(buf, 72, torque_5);
	_mav_put_float(buf, 76, torque_6);
	_mav_put_float(buf, 80, torque_7);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS, buf, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_CRC);
#else
	mavlink_manipulator_joint_status_t packet;
	packet.joint_posi_1 = joint_posi_1;
	packet.joint_posi_2 = joint_posi_2;
	packet.joint_posi_3 = joint_posi_3;
	packet.joint_posi_4 = joint_posi_4;
	packet.joint_posi_5 = joint_posi_5;
	packet.joint_posi_6 = joint_posi_6;
	packet.joint_posi_7 = joint_posi_7;
	packet.joint_rate_1 = joint_rate_1;
	packet.joint_rate_2 = joint_rate_2;
	packet.joint_rate_3 = joint_rate_3;
	packet.joint_rate_4 = joint_rate_4;
	packet.joint_rate_5 = joint_rate_5;
	packet.joint_rate_6 = joint_rate_6;
	packet.joint_rate_7 = joint_rate_7;
	packet.torque_1 = torque_1;
	packet.torque_2 = torque_2;
	packet.torque_3 = torque_3;
	packet.torque_4 = torque_4;
	packet.torque_5 = torque_5;
	packet.torque_6 = torque_6;
	packet.torque_7 = torque_7;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_CRC);
#endif
}

/**
 * @brief Send a manipulator_joint_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_manipulator_joint_status_send_struct(mavlink_channel_t chan, const mavlink_manipulator_joint_status_t* manipulator_joint_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_manipulator_joint_status_send(chan, manipulator_joint_status->joint_posi_1, manipulator_joint_status->joint_posi_2, manipulator_joint_status->joint_posi_3, manipulator_joint_status->joint_posi_4, manipulator_joint_status->joint_posi_5, manipulator_joint_status->joint_posi_6, manipulator_joint_status->joint_posi_7, manipulator_joint_status->joint_rate_1, manipulator_joint_status->joint_rate_2, manipulator_joint_status->joint_rate_3, manipulator_joint_status->joint_rate_4, manipulator_joint_status->joint_rate_5, manipulator_joint_status->joint_rate_6, manipulator_joint_status->joint_rate_7, manipulator_joint_status->torque_1, manipulator_joint_status->torque_2, manipulator_joint_status->torque_3, manipulator_joint_status->torque_4, manipulator_joint_status->torque_5, manipulator_joint_status->torque_6, manipulator_joint_status->torque_7);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS, (const char *)manipulator_joint_status, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_manipulator_joint_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float joint_posi_1, float joint_posi_2, float joint_posi_3, float joint_posi_4, float joint_posi_5, float joint_posi_6, float joint_posi_7, float joint_rate_1, float joint_rate_2, float joint_rate_3, float joint_rate_4, float joint_rate_5, float joint_rate_6, float joint_rate_7, float torque_1, float torque_2, float torque_3, float torque_4, float torque_5, float torque_6, float torque_7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, joint_posi_1);
	_mav_put_float(buf, 4, joint_posi_2);
	_mav_put_float(buf, 8, joint_posi_3);
	_mav_put_float(buf, 12, joint_posi_4);
	_mav_put_float(buf, 16, joint_posi_5);
	_mav_put_float(buf, 20, joint_posi_6);
	_mav_put_float(buf, 24, joint_posi_7);
	_mav_put_float(buf, 28, joint_rate_1);
	_mav_put_float(buf, 32, joint_rate_2);
	_mav_put_float(buf, 36, joint_rate_3);
	_mav_put_float(buf, 40, joint_rate_4);
	_mav_put_float(buf, 44, joint_rate_5);
	_mav_put_float(buf, 48, joint_rate_6);
	_mav_put_float(buf, 52, joint_rate_7);
	_mav_put_float(buf, 56, torque_1);
	_mav_put_float(buf, 60, torque_2);
	_mav_put_float(buf, 64, torque_3);
	_mav_put_float(buf, 68, torque_4);
	_mav_put_float(buf, 72, torque_5);
	_mav_put_float(buf, 76, torque_6);
	_mav_put_float(buf, 80, torque_7);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS, buf, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_CRC);
#else
	mavlink_manipulator_joint_status_t *packet = (mavlink_manipulator_joint_status_t *)msgbuf;
	packet->joint_posi_1 = joint_posi_1;
	packet->joint_posi_2 = joint_posi_2;
	packet->joint_posi_3 = joint_posi_3;
	packet->joint_posi_4 = joint_posi_4;
	packet->joint_posi_5 = joint_posi_5;
	packet->joint_posi_6 = joint_posi_6;
	packet->joint_posi_7 = joint_posi_7;
	packet->joint_rate_1 = joint_rate_1;
	packet->joint_rate_2 = joint_rate_2;
	packet->joint_rate_3 = joint_rate_3;
	packet->joint_rate_4 = joint_rate_4;
	packet->joint_rate_5 = joint_rate_5;
	packet->joint_rate_6 = joint_rate_6;
	packet->joint_rate_7 = joint_rate_7;
	packet->torque_1 = torque_1;
	packet->torque_2 = torque_2;
	packet->torque_3 = torque_3;
	packet->torque_4 = torque_4;
	packet->torque_5 = torque_5;
	packet->torque_6 = torque_6;
	packet->torque_7 = torque_7;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS, (const char *)packet, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MANIPULATOR_JOINT_STATUS UNPACKING


/**
 * @brief Get field joint_posi_1 from manipulator_joint_status message
 *
 * @return Position of jiont 1 in rad
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_posi_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field joint_posi_2 from manipulator_joint_status message
 *
 * @return Position of jiont 2 in rad
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_posi_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field joint_posi_3 from manipulator_joint_status message
 *
 * @return Position of jiont 3 in rad
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_posi_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field joint_posi_4 from manipulator_joint_status message
 *
 * @return Position of jiont 4 in rad
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_posi_4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field joint_posi_5 from manipulator_joint_status message
 *
 * @return Position of jiont 5 in rad
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_posi_5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field joint_posi_6 from manipulator_joint_status message
 *
 * @return Position of jiont 6 in rad
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_posi_6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field joint_posi_7 from manipulator_joint_status message
 *
 * @return Position of jiont 7 in rad
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_posi_7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field joint_rate_1 from manipulator_joint_status message
 *
 * @return Rate of jiont 1 in rad/s
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_rate_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field joint_rate_2 from manipulator_joint_status message
 *
 * @return Rate of jiont 2 in rad/s
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_rate_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field joint_rate_3 from manipulator_joint_status message
 *
 * @return Rate of jiont 3 in rad/s
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_rate_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field joint_rate_4 from manipulator_joint_status message
 *
 * @return Rate of jiont 4 in rad/s
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_rate_4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field joint_rate_5 from manipulator_joint_status message
 *
 * @return Rate of jiont 5 in rad/s
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_rate_5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field joint_rate_6 from manipulator_joint_status message
 *
 * @return Rate of jiont 6 in rad/s
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_rate_6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field joint_rate_7 from manipulator_joint_status message
 *
 * @return Rate of jiont 7 in rad/s
 */
static inline float mavlink_msg_manipulator_joint_status_get_joint_rate_7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field torque_1 from manipulator_joint_status message
 *
 * @return torque of jiont 1 in N*m
 */
static inline float mavlink_msg_manipulator_joint_status_get_torque_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field torque_2 from manipulator_joint_status message
 *
 * @return torque of jiont 2 in N*m
 */
static inline float mavlink_msg_manipulator_joint_status_get_torque_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field torque_3 from manipulator_joint_status message
 *
 * @return torque of jiont 3 in N*m
 */
static inline float mavlink_msg_manipulator_joint_status_get_torque_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field torque_4 from manipulator_joint_status message
 *
 * @return torque of jiont 4 in N*m
 */
static inline float mavlink_msg_manipulator_joint_status_get_torque_4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field torque_5 from manipulator_joint_status message
 *
 * @return torque of jiont 5 in N*m
 */
static inline float mavlink_msg_manipulator_joint_status_get_torque_5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field torque_6 from manipulator_joint_status message
 *
 * @return torque of jiont 6 in N*m
 */
static inline float mavlink_msg_manipulator_joint_status_get_torque_6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field torque_7 from manipulator_joint_status message
 *
 * @return torque of jiont 7 in N*m
 */
static inline float mavlink_msg_manipulator_joint_status_get_torque_7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Decode a manipulator_joint_status message into a struct
 *
 * @param msg The message to decode
 * @param manipulator_joint_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_manipulator_joint_status_decode(const mavlink_message_t* msg, mavlink_manipulator_joint_status_t* manipulator_joint_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	manipulator_joint_status->joint_posi_1 = mavlink_msg_manipulator_joint_status_get_joint_posi_1(msg);
	manipulator_joint_status->joint_posi_2 = mavlink_msg_manipulator_joint_status_get_joint_posi_2(msg);
	manipulator_joint_status->joint_posi_3 = mavlink_msg_manipulator_joint_status_get_joint_posi_3(msg);
	manipulator_joint_status->joint_posi_4 = mavlink_msg_manipulator_joint_status_get_joint_posi_4(msg);
	manipulator_joint_status->joint_posi_5 = mavlink_msg_manipulator_joint_status_get_joint_posi_5(msg);
	manipulator_joint_status->joint_posi_6 = mavlink_msg_manipulator_joint_status_get_joint_posi_6(msg);
	manipulator_joint_status->joint_posi_7 = mavlink_msg_manipulator_joint_status_get_joint_posi_7(msg);
	manipulator_joint_status->joint_rate_1 = mavlink_msg_manipulator_joint_status_get_joint_rate_1(msg);
	manipulator_joint_status->joint_rate_2 = mavlink_msg_manipulator_joint_status_get_joint_rate_2(msg);
	manipulator_joint_status->joint_rate_3 = mavlink_msg_manipulator_joint_status_get_joint_rate_3(msg);
	manipulator_joint_status->joint_rate_4 = mavlink_msg_manipulator_joint_status_get_joint_rate_4(msg);
	manipulator_joint_status->joint_rate_5 = mavlink_msg_manipulator_joint_status_get_joint_rate_5(msg);
	manipulator_joint_status->joint_rate_6 = mavlink_msg_manipulator_joint_status_get_joint_rate_6(msg);
	manipulator_joint_status->joint_rate_7 = mavlink_msg_manipulator_joint_status_get_joint_rate_7(msg);
	manipulator_joint_status->torque_1 = mavlink_msg_manipulator_joint_status_get_torque_1(msg);
	manipulator_joint_status->torque_2 = mavlink_msg_manipulator_joint_status_get_torque_2(msg);
	manipulator_joint_status->torque_3 = mavlink_msg_manipulator_joint_status_get_torque_3(msg);
	manipulator_joint_status->torque_4 = mavlink_msg_manipulator_joint_status_get_torque_4(msg);
	manipulator_joint_status->torque_5 = mavlink_msg_manipulator_joint_status_get_torque_5(msg);
	manipulator_joint_status->torque_6 = mavlink_msg_manipulator_joint_status_get_torque_6(msg);
	manipulator_joint_status->torque_7 = mavlink_msg_manipulator_joint_status_get_torque_7(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN;
        memset(manipulator_joint_status, 0, MAVLINK_MSG_ID_MANIPULATOR_JOINT_STATUS_LEN);
	memcpy(manipulator_joint_status, _MAV_PAYLOAD(msg), len);
#endif
}
