// MESSAGE ENDEFF_FRAME_STATUS PACKING

#define MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS 181

MAVPACKED(
typedef struct __mavlink_endeff_frame_status_t {
 float gripper_posi; /*< Position of girpper in rad*/
 float x; /*< X Position in NED frame in meters*/
 float y; /*< Y Position in NED frame in meters*/
 float z; /*< Z Position in NED frame in meters*/
 float roll; /*< roll in rad*/
 float pitch; /*< pitch in rad*/
 float yaw; /*< yaw in rad*/
 float vx; /*< X velocity in NED frame in meter/s*/
 float vy; /*< Y velocity in NED frame in meter/s*/
 float vz; /*< Z velocity in NED frame in meter/s*/
 float roll_rate; /*< roll rate in rad/s*/
 float pitch_rate; /*< pitch rate in rad/s*/
 float yaw_rate; /*< yaw rate in rad/s*/
 char arm_enable; /*< enable robotic arm  0 disenable, 1 enable*/
 int8_t gripper_status; /*< status of girpper 1 open, 0 closing, -1 closed*/
}) mavlink_endeff_frame_status_t;

#define MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN 54
#define MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_MIN_LEN 54
#define MAVLINK_MSG_ID_181_LEN 54
#define MAVLINK_MSG_ID_181_MIN_LEN 54

#define MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_CRC 170
#define MAVLINK_MSG_ID_181_CRC 170



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ENDEFF_FRAME_STATUS { \
	181, \
	"ENDEFF_FRAME_STATUS", \
	15, \
	{  { "gripper_posi", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_endeff_frame_status_t, gripper_posi) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_endeff_frame_status_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_endeff_frame_status_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_endeff_frame_status_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_endeff_frame_status_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_endeff_frame_status_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_endeff_frame_status_t, yaw) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_endeff_frame_status_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_endeff_frame_status_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_endeff_frame_status_t, vz) }, \
         { "roll_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_endeff_frame_status_t, roll_rate) }, \
         { "pitch_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_endeff_frame_status_t, pitch_rate) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_endeff_frame_status_t, yaw_rate) }, \
         { "arm_enable", NULL, MAVLINK_TYPE_CHAR, 0, 52, offsetof(mavlink_endeff_frame_status_t, arm_enable) }, \
         { "gripper_status", NULL, MAVLINK_TYPE_INT8_T, 0, 53, offsetof(mavlink_endeff_frame_status_t, gripper_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ENDEFF_FRAME_STATUS { \
	"ENDEFF_FRAME_STATUS", \
	15, \
	{  { "gripper_posi", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_endeff_frame_status_t, gripper_posi) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_endeff_frame_status_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_endeff_frame_status_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_endeff_frame_status_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_endeff_frame_status_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_endeff_frame_status_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_endeff_frame_status_t, yaw) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_endeff_frame_status_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_endeff_frame_status_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_endeff_frame_status_t, vz) }, \
         { "roll_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_endeff_frame_status_t, roll_rate) }, \
         { "pitch_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_endeff_frame_status_t, pitch_rate) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_endeff_frame_status_t, yaw_rate) }, \
         { "arm_enable", NULL, MAVLINK_TYPE_CHAR, 0, 52, offsetof(mavlink_endeff_frame_status_t, arm_enable) }, \
         { "gripper_status", NULL, MAVLINK_TYPE_INT8_T, 0, 53, offsetof(mavlink_endeff_frame_status_t, gripper_status) }, \
         } \
}
#endif

/**
 * @brief Pack a endeff_frame_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param arm_enable enable robotic arm  0 disenable, 1 enable
 * @param gripper_status status of girpper 1 open, 0 closing, -1 closed
 * @param gripper_posi Position of girpper in rad
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters
 * @param roll roll in rad
 * @param pitch pitch in rad
 * @param yaw yaw in rad
 * @param vx X velocity in NED frame in meter/s
 * @param vy Y velocity in NED frame in meter/s
 * @param vz Z velocity in NED frame in meter/s
 * @param roll_rate roll rate in rad/s
 * @param pitch_rate pitch rate in rad/s
 * @param yaw_rate yaw rate in rad/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_endeff_frame_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       char arm_enable, int8_t gripper_status, float gripper_posi, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float roll_rate, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN];
	_mav_put_float(buf, 0, gripper_posi);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, roll);
	_mav_put_float(buf, 20, pitch);
	_mav_put_float(buf, 24, yaw);
	_mav_put_float(buf, 28, vx);
	_mav_put_float(buf, 32, vy);
	_mav_put_float(buf, 36, vz);
	_mav_put_float(buf, 40, roll_rate);
	_mav_put_float(buf, 44, pitch_rate);
	_mav_put_float(buf, 48, yaw_rate);
	_mav_put_char(buf, 52, arm_enable);
	_mav_put_int8_t(buf, 53, gripper_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN);
#else
	mavlink_endeff_frame_status_t packet;
	packet.gripper_posi = gripper_posi;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.roll_rate = roll_rate;
	packet.pitch_rate = pitch_rate;
	packet.yaw_rate = yaw_rate;
	packet.arm_enable = arm_enable;
	packet.gripper_status = gripper_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_CRC);
}

/**
 * @brief Pack a endeff_frame_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arm_enable enable robotic arm  0 disenable, 1 enable
 * @param gripper_status status of girpper 1 open, 0 closing, -1 closed
 * @param gripper_posi Position of girpper in rad
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters
 * @param roll roll in rad
 * @param pitch pitch in rad
 * @param yaw yaw in rad
 * @param vx X velocity in NED frame in meter/s
 * @param vy Y velocity in NED frame in meter/s
 * @param vz Z velocity in NED frame in meter/s
 * @param roll_rate roll rate in rad/s
 * @param pitch_rate pitch rate in rad/s
 * @param yaw_rate yaw rate in rad/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_endeff_frame_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           char arm_enable,int8_t gripper_status,float gripper_posi,float x,float y,float z,float roll,float pitch,float yaw,float vx,float vy,float vz,float roll_rate,float pitch_rate,float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN];
	_mav_put_float(buf, 0, gripper_posi);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, roll);
	_mav_put_float(buf, 20, pitch);
	_mav_put_float(buf, 24, yaw);
	_mav_put_float(buf, 28, vx);
	_mav_put_float(buf, 32, vy);
	_mav_put_float(buf, 36, vz);
	_mav_put_float(buf, 40, roll_rate);
	_mav_put_float(buf, 44, pitch_rate);
	_mav_put_float(buf, 48, yaw_rate);
	_mav_put_char(buf, 52, arm_enable);
	_mav_put_int8_t(buf, 53, gripper_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN);
#else
	mavlink_endeff_frame_status_t packet;
	packet.gripper_posi = gripper_posi;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.roll_rate = roll_rate;
	packet.pitch_rate = pitch_rate;
	packet.yaw_rate = yaw_rate;
	packet.arm_enable = arm_enable;
	packet.gripper_status = gripper_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_CRC);
}

/**
 * @brief Encode a endeff_frame_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param endeff_frame_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_endeff_frame_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_endeff_frame_status_t* endeff_frame_status)
{
	return mavlink_msg_endeff_frame_status_pack(system_id, component_id, msg, endeff_frame_status->arm_enable, endeff_frame_status->gripper_status, endeff_frame_status->gripper_posi, endeff_frame_status->x, endeff_frame_status->y, endeff_frame_status->z, endeff_frame_status->roll, endeff_frame_status->pitch, endeff_frame_status->yaw, endeff_frame_status->vx, endeff_frame_status->vy, endeff_frame_status->vz, endeff_frame_status->roll_rate, endeff_frame_status->pitch_rate, endeff_frame_status->yaw_rate);
}

/**
 * @brief Encode a endeff_frame_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param endeff_frame_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_endeff_frame_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_endeff_frame_status_t* endeff_frame_status)
{
	return mavlink_msg_endeff_frame_status_pack_chan(system_id, component_id, chan, msg, endeff_frame_status->arm_enable, endeff_frame_status->gripper_status, endeff_frame_status->gripper_posi, endeff_frame_status->x, endeff_frame_status->y, endeff_frame_status->z, endeff_frame_status->roll, endeff_frame_status->pitch, endeff_frame_status->yaw, endeff_frame_status->vx, endeff_frame_status->vy, endeff_frame_status->vz, endeff_frame_status->roll_rate, endeff_frame_status->pitch_rate, endeff_frame_status->yaw_rate);
}

/**
 * @brief Send a endeff_frame_status message
 * @param chan MAVLink channel to send the message
 *
 * @param arm_enable enable robotic arm  0 disenable, 1 enable
 * @param gripper_status status of girpper 1 open, 0 closing, -1 closed
 * @param gripper_posi Position of girpper in rad
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters
 * @param roll roll in rad
 * @param pitch pitch in rad
 * @param yaw yaw in rad
 * @param vx X velocity in NED frame in meter/s
 * @param vy Y velocity in NED frame in meter/s
 * @param vz Z velocity in NED frame in meter/s
 * @param roll_rate roll rate in rad/s
 * @param pitch_rate pitch rate in rad/s
 * @param yaw_rate yaw rate in rad/s
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_endeff_frame_status_send(mavlink_channel_t chan, char arm_enable, int8_t gripper_status, float gripper_posi, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float roll_rate, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN];
	_mav_put_float(buf, 0, gripper_posi);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, roll);
	_mav_put_float(buf, 20, pitch);
	_mav_put_float(buf, 24, yaw);
	_mav_put_float(buf, 28, vx);
	_mav_put_float(buf, 32, vy);
	_mav_put_float(buf, 36, vz);
	_mav_put_float(buf, 40, roll_rate);
	_mav_put_float(buf, 44, pitch_rate);
	_mav_put_float(buf, 48, yaw_rate);
	_mav_put_char(buf, 52, arm_enable);
	_mav_put_int8_t(buf, 53, gripper_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS, buf, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_CRC);
#else
	mavlink_endeff_frame_status_t packet;
	packet.gripper_posi = gripper_posi;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.roll_rate = roll_rate;
	packet.pitch_rate = pitch_rate;
	packet.yaw_rate = yaw_rate;
	packet.arm_enable = arm_enable;
	packet.gripper_status = gripper_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_CRC);
#endif
}

/**
 * @brief Send a endeff_frame_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_endeff_frame_status_send_struct(mavlink_channel_t chan, const mavlink_endeff_frame_status_t* endeff_frame_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_endeff_frame_status_send(chan, endeff_frame_status->arm_enable, endeff_frame_status->gripper_status, endeff_frame_status->gripper_posi, endeff_frame_status->x, endeff_frame_status->y, endeff_frame_status->z, endeff_frame_status->roll, endeff_frame_status->pitch, endeff_frame_status->yaw, endeff_frame_status->vx, endeff_frame_status->vy, endeff_frame_status->vz, endeff_frame_status->roll_rate, endeff_frame_status->pitch_rate, endeff_frame_status->yaw_rate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS, (const char *)endeff_frame_status, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_endeff_frame_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  char arm_enable, int8_t gripper_status, float gripper_posi, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float roll_rate, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, gripper_posi);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, roll);
	_mav_put_float(buf, 20, pitch);
	_mav_put_float(buf, 24, yaw);
	_mav_put_float(buf, 28, vx);
	_mav_put_float(buf, 32, vy);
	_mav_put_float(buf, 36, vz);
	_mav_put_float(buf, 40, roll_rate);
	_mav_put_float(buf, 44, pitch_rate);
	_mav_put_float(buf, 48, yaw_rate);
	_mav_put_char(buf, 52, arm_enable);
	_mav_put_int8_t(buf, 53, gripper_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS, buf, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_CRC);
#else
	mavlink_endeff_frame_status_t *packet = (mavlink_endeff_frame_status_t *)msgbuf;
	packet->gripper_posi = gripper_posi;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;
	packet->roll_rate = roll_rate;
	packet->pitch_rate = pitch_rate;
	packet->yaw_rate = yaw_rate;
	packet->arm_enable = arm_enable;
	packet->gripper_status = gripper_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS, (const char *)packet, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ENDEFF_FRAME_STATUS UNPACKING


/**
 * @brief Get field arm_enable from endeff_frame_status message
 *
 * @return enable robotic arm  0 disenable, 1 enable
 */
static inline char mavlink_msg_endeff_frame_status_get_arm_enable(const mavlink_message_t* msg)
{
	return _MAV_RETURN_char(msg,  52);
}

/**
 * @brief Get field gripper_status from endeff_frame_status message
 *
 * @return status of girpper 1 open, 0 closing, -1 closed
 */
static inline int8_t mavlink_msg_endeff_frame_status_get_gripper_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  53);
}

/**
 * @brief Get field gripper_posi from endeff_frame_status message
 *
 * @return Position of girpper in rad
 */
static inline float mavlink_msg_endeff_frame_status_get_gripper_posi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field x from endeff_frame_status message
 *
 * @return X Position in NED frame in meters
 */
static inline float mavlink_msg_endeff_frame_status_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from endeff_frame_status message
 *
 * @return Y Position in NED frame in meters
 */
static inline float mavlink_msg_endeff_frame_status_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from endeff_frame_status message
 *
 * @return Z Position in NED frame in meters
 */
static inline float mavlink_msg_endeff_frame_status_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll from endeff_frame_status message
 *
 * @return roll in rad
 */
static inline float mavlink_msg_endeff_frame_status_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitch from endeff_frame_status message
 *
 * @return pitch in rad
 */
static inline float mavlink_msg_endeff_frame_status_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yaw from endeff_frame_status message
 *
 * @return yaw in rad
 */
static inline float mavlink_msg_endeff_frame_status_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vx from endeff_frame_status message
 *
 * @return X velocity in NED frame in meter/s
 */
static inline float mavlink_msg_endeff_frame_status_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vy from endeff_frame_status message
 *
 * @return Y velocity in NED frame in meter/s
 */
static inline float mavlink_msg_endeff_frame_status_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field vz from endeff_frame_status message
 *
 * @return Z velocity in NED frame in meter/s
 */
static inline float mavlink_msg_endeff_frame_status_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field roll_rate from endeff_frame_status message
 *
 * @return roll rate in rad/s
 */
static inline float mavlink_msg_endeff_frame_status_get_roll_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field pitch_rate from endeff_frame_status message
 *
 * @return pitch rate in rad/s
 */
static inline float mavlink_msg_endeff_frame_status_get_pitch_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field yaw_rate from endeff_frame_status message
 *
 * @return yaw rate in rad/s
 */
static inline float mavlink_msg_endeff_frame_status_get_yaw_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Decode a endeff_frame_status message into a struct
 *
 * @param msg The message to decode
 * @param endeff_frame_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_endeff_frame_status_decode(const mavlink_message_t* msg, mavlink_endeff_frame_status_t* endeff_frame_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	endeff_frame_status->gripper_posi = mavlink_msg_endeff_frame_status_get_gripper_posi(msg);
	endeff_frame_status->x = mavlink_msg_endeff_frame_status_get_x(msg);
	endeff_frame_status->y = mavlink_msg_endeff_frame_status_get_y(msg);
	endeff_frame_status->z = mavlink_msg_endeff_frame_status_get_z(msg);
	endeff_frame_status->roll = mavlink_msg_endeff_frame_status_get_roll(msg);
	endeff_frame_status->pitch = mavlink_msg_endeff_frame_status_get_pitch(msg);
	endeff_frame_status->yaw = mavlink_msg_endeff_frame_status_get_yaw(msg);
	endeff_frame_status->vx = mavlink_msg_endeff_frame_status_get_vx(msg);
	endeff_frame_status->vy = mavlink_msg_endeff_frame_status_get_vy(msg);
	endeff_frame_status->vz = mavlink_msg_endeff_frame_status_get_vz(msg);
	endeff_frame_status->roll_rate = mavlink_msg_endeff_frame_status_get_roll_rate(msg);
	endeff_frame_status->pitch_rate = mavlink_msg_endeff_frame_status_get_pitch_rate(msg);
	endeff_frame_status->yaw_rate = mavlink_msg_endeff_frame_status_get_yaw_rate(msg);
	endeff_frame_status->arm_enable = mavlink_msg_endeff_frame_status_get_arm_enable(msg);
	endeff_frame_status->gripper_status = mavlink_msg_endeff_frame_status_get_gripper_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN;
        memset(endeff_frame_status, 0, MAVLINK_MSG_ID_ENDEFF_FRAME_STATUS_LEN);
	memcpy(endeff_frame_status, _MAV_PAYLOAD(msg), len);
#endif
}
