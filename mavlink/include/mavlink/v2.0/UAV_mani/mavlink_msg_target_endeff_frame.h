// MESSAGE TARGET_ENDEFF_FRAME PACKING

#define MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME 180

MAVPACKED(
typedef struct __mavlink_target_endeff_frame_t {
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
}) mavlink_target_endeff_frame_t;

#define MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN 49
#define MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_MIN_LEN 49
#define MAVLINK_MSG_ID_180_LEN 49
#define MAVLINK_MSG_ID_180_MIN_LEN 49

#define MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_CRC 242
#define MAVLINK_MSG_ID_180_CRC 242



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TARGET_ENDEFF_FRAME { \
	180, \
	"TARGET_ENDEFF_FRAME", \
	13, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_target_endeff_frame_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_endeff_frame_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_endeff_frame_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_endeff_frame_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_endeff_frame_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_endeff_frame_t, yaw) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_target_endeff_frame_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_target_endeff_frame_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_target_endeff_frame_t, vz) }, \
         { "roll_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_target_endeff_frame_t, roll_rate) }, \
         { "pitch_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_target_endeff_frame_t, pitch_rate) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_target_endeff_frame_t, yaw_rate) }, \
         { "arm_enable", NULL, MAVLINK_TYPE_CHAR, 0, 48, offsetof(mavlink_target_endeff_frame_t, arm_enable) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TARGET_ENDEFF_FRAME { \
	"TARGET_ENDEFF_FRAME", \
	13, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_target_endeff_frame_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_endeff_frame_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_endeff_frame_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_endeff_frame_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_endeff_frame_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_endeff_frame_t, yaw) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_target_endeff_frame_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_target_endeff_frame_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_target_endeff_frame_t, vz) }, \
         { "roll_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_target_endeff_frame_t, roll_rate) }, \
         { "pitch_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_target_endeff_frame_t, pitch_rate) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_target_endeff_frame_t, yaw_rate) }, \
         { "arm_enable", NULL, MAVLINK_TYPE_CHAR, 0, 48, offsetof(mavlink_target_endeff_frame_t, arm_enable) }, \
         } \
}
#endif

/**
 * @brief Pack a target_endeff_frame message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param arm_enable enable robotic arm  0 disenable, 1 enable
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
static inline uint16_t mavlink_msg_target_endeff_frame_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       char arm_enable, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float roll_rate, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, roll);
	_mav_put_float(buf, 16, pitch);
	_mav_put_float(buf, 20, yaw);
	_mav_put_float(buf, 24, vx);
	_mav_put_float(buf, 28, vy);
	_mav_put_float(buf, 32, vz);
	_mav_put_float(buf, 36, roll_rate);
	_mav_put_float(buf, 40, pitch_rate);
	_mav_put_float(buf, 44, yaw_rate);
	_mav_put_char(buf, 48, arm_enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN);
#else
	mavlink_target_endeff_frame_t packet;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_MIN_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_CRC);
}

/**
 * @brief Pack a target_endeff_frame message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arm_enable enable robotic arm  0 disenable, 1 enable
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
static inline uint16_t mavlink_msg_target_endeff_frame_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           char arm_enable,float x,float y,float z,float roll,float pitch,float yaw,float vx,float vy,float vz,float roll_rate,float pitch_rate,float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, roll);
	_mav_put_float(buf, 16, pitch);
	_mav_put_float(buf, 20, yaw);
	_mav_put_float(buf, 24, vx);
	_mav_put_float(buf, 28, vy);
	_mav_put_float(buf, 32, vz);
	_mav_put_float(buf, 36, roll_rate);
	_mav_put_float(buf, 40, pitch_rate);
	_mav_put_float(buf, 44, yaw_rate);
	_mav_put_char(buf, 48, arm_enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN);
#else
	mavlink_target_endeff_frame_t packet;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_MIN_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_CRC);
}

/**
 * @brief Encode a target_endeff_frame struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param target_endeff_frame C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_endeff_frame_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_target_endeff_frame_t* target_endeff_frame)
{
	return mavlink_msg_target_endeff_frame_pack(system_id, component_id, msg, target_endeff_frame->arm_enable, target_endeff_frame->x, target_endeff_frame->y, target_endeff_frame->z, target_endeff_frame->roll, target_endeff_frame->pitch, target_endeff_frame->yaw, target_endeff_frame->vx, target_endeff_frame->vy, target_endeff_frame->vz, target_endeff_frame->roll_rate, target_endeff_frame->pitch_rate, target_endeff_frame->yaw_rate);
}

/**
 * @brief Encode a target_endeff_frame struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_endeff_frame C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_endeff_frame_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_target_endeff_frame_t* target_endeff_frame)
{
	return mavlink_msg_target_endeff_frame_pack_chan(system_id, component_id, chan, msg, target_endeff_frame->arm_enable, target_endeff_frame->x, target_endeff_frame->y, target_endeff_frame->z, target_endeff_frame->roll, target_endeff_frame->pitch, target_endeff_frame->yaw, target_endeff_frame->vx, target_endeff_frame->vy, target_endeff_frame->vz, target_endeff_frame->roll_rate, target_endeff_frame->pitch_rate, target_endeff_frame->yaw_rate);
}

/**
 * @brief Send a target_endeff_frame message
 * @param chan MAVLink channel to send the message
 *
 * @param arm_enable enable robotic arm  0 disenable, 1 enable
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

static inline void mavlink_msg_target_endeff_frame_send(mavlink_channel_t chan, char arm_enable, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float roll_rate, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, roll);
	_mav_put_float(buf, 16, pitch);
	_mav_put_float(buf, 20, yaw);
	_mav_put_float(buf, 24, vx);
	_mav_put_float(buf, 28, vy);
	_mav_put_float(buf, 32, vz);
	_mav_put_float(buf, 36, roll_rate);
	_mav_put_float(buf, 40, pitch_rate);
	_mav_put_float(buf, 44, yaw_rate);
	_mav_put_char(buf, 48, arm_enable);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME, buf, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_MIN_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_CRC);
#else
	mavlink_target_endeff_frame_t packet;
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

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME, (const char *)&packet, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_MIN_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_CRC);
#endif
}

/**
 * @brief Send a target_endeff_frame message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_target_endeff_frame_send_struct(mavlink_channel_t chan, const mavlink_target_endeff_frame_t* target_endeff_frame)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_target_endeff_frame_send(chan, target_endeff_frame->arm_enable, target_endeff_frame->x, target_endeff_frame->y, target_endeff_frame->z, target_endeff_frame->roll, target_endeff_frame->pitch, target_endeff_frame->yaw, target_endeff_frame->vx, target_endeff_frame->vy, target_endeff_frame->vz, target_endeff_frame->roll_rate, target_endeff_frame->pitch_rate, target_endeff_frame->yaw_rate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME, (const char *)target_endeff_frame, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_MIN_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_CRC);
#endif
}

#if MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_target_endeff_frame_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  char arm_enable, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float roll_rate, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, roll);
	_mav_put_float(buf, 16, pitch);
	_mav_put_float(buf, 20, yaw);
	_mav_put_float(buf, 24, vx);
	_mav_put_float(buf, 28, vy);
	_mav_put_float(buf, 32, vz);
	_mav_put_float(buf, 36, roll_rate);
	_mav_put_float(buf, 40, pitch_rate);
	_mav_put_float(buf, 44, yaw_rate);
	_mav_put_char(buf, 48, arm_enable);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME, buf, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_MIN_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_CRC);
#else
	mavlink_target_endeff_frame_t *packet = (mavlink_target_endeff_frame_t *)msgbuf;
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

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME, (const char *)packet, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_MIN_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_CRC);
#endif
}
#endif

#endif

// MESSAGE TARGET_ENDEFF_FRAME UNPACKING


/**
 * @brief Get field arm_enable from target_endeff_frame message
 *
 * @return enable robotic arm  0 disenable, 1 enable
 */
static inline char mavlink_msg_target_endeff_frame_get_arm_enable(const mavlink_message_t* msg)
{
	return _MAV_RETURN_char(msg,  48);
}

/**
 * @brief Get field x from target_endeff_frame message
 *
 * @return X Position in NED frame in meters
 */
static inline float mavlink_msg_target_endeff_frame_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from target_endeff_frame message
 *
 * @return Y Position in NED frame in meters
 */
static inline float mavlink_msg_target_endeff_frame_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from target_endeff_frame message
 *
 * @return Z Position in NED frame in meters
 */
static inline float mavlink_msg_target_endeff_frame_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field roll from target_endeff_frame message
 *
 * @return roll in rad
 */
static inline float mavlink_msg_target_endeff_frame_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitch from target_endeff_frame message
 *
 * @return pitch in rad
 */
static inline float mavlink_msg_target_endeff_frame_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yaw from target_endeff_frame message
 *
 * @return yaw in rad
 */
static inline float mavlink_msg_target_endeff_frame_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vx from target_endeff_frame message
 *
 * @return X velocity in NED frame in meter/s
 */
static inline float mavlink_msg_target_endeff_frame_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vy from target_endeff_frame message
 *
 * @return Y velocity in NED frame in meter/s
 */
static inline float mavlink_msg_target_endeff_frame_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vz from target_endeff_frame message
 *
 * @return Z velocity in NED frame in meter/s
 */
static inline float mavlink_msg_target_endeff_frame_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field roll_rate from target_endeff_frame message
 *
 * @return roll rate in rad/s
 */
static inline float mavlink_msg_target_endeff_frame_get_roll_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field pitch_rate from target_endeff_frame message
 *
 * @return pitch rate in rad/s
 */
static inline float mavlink_msg_target_endeff_frame_get_pitch_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field yaw_rate from target_endeff_frame message
 *
 * @return yaw rate in rad/s
 */
static inline float mavlink_msg_target_endeff_frame_get_yaw_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a target_endeff_frame message into a struct
 *
 * @param msg The message to decode
 * @param target_endeff_frame C-struct to decode the message contents into
 */
static inline void mavlink_msg_target_endeff_frame_decode(const mavlink_message_t* msg, mavlink_target_endeff_frame_t* target_endeff_frame)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	target_endeff_frame->x = mavlink_msg_target_endeff_frame_get_x(msg);
	target_endeff_frame->y = mavlink_msg_target_endeff_frame_get_y(msg);
	target_endeff_frame->z = mavlink_msg_target_endeff_frame_get_z(msg);
	target_endeff_frame->roll = mavlink_msg_target_endeff_frame_get_roll(msg);
	target_endeff_frame->pitch = mavlink_msg_target_endeff_frame_get_pitch(msg);
	target_endeff_frame->yaw = mavlink_msg_target_endeff_frame_get_yaw(msg);
	target_endeff_frame->vx = mavlink_msg_target_endeff_frame_get_vx(msg);
	target_endeff_frame->vy = mavlink_msg_target_endeff_frame_get_vy(msg);
	target_endeff_frame->vz = mavlink_msg_target_endeff_frame_get_vz(msg);
	target_endeff_frame->roll_rate = mavlink_msg_target_endeff_frame_get_roll_rate(msg);
	target_endeff_frame->pitch_rate = mavlink_msg_target_endeff_frame_get_pitch_rate(msg);
	target_endeff_frame->yaw_rate = mavlink_msg_target_endeff_frame_get_yaw_rate(msg);
	target_endeff_frame->arm_enable = mavlink_msg_target_endeff_frame_get_arm_enable(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN? msg->len : MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN;
        memset(target_endeff_frame, 0, MAVLINK_MSG_ID_TARGET_ENDEFF_FRAME_LEN);
	memcpy(target_endeff_frame, _MAV_PAYLOAD(msg), len);
#endif
}
