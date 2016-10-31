// MESSAGE TARGET_INFO PACKING

#define MAVLINK_MSG_ID_TARGET_INFO 210

MAVPACKED(
typedef struct __mavlink_target_info_t {
 uint64_t time_usec; /*< Timestamp*/
 float x; /*< X Position*/
 float y; /*< Y Position*/
 float z; /*< Z Position*/
 float vx; /*< X Speed*/
 float vy; /*< Y Speed*/
 float vz; /*< Z Speed*/
 float roll; /*< Roll angle (rad, -pi..+pi)*/
 float pitch; /*< Pitch angle (rad, -pi..+pi)*/
 float yaw; /*< Yaw angle (rad, -pi..+pi)*/
}) mavlink_target_info_t;

#define MAVLINK_MSG_ID_TARGET_INFO_LEN 44
#define MAVLINK_MSG_ID_TARGET_INFO_MIN_LEN 44
#define MAVLINK_MSG_ID_210_LEN 44
#define MAVLINK_MSG_ID_210_MIN_LEN 44

#define MAVLINK_MSG_ID_TARGET_INFO_CRC 20
#define MAVLINK_MSG_ID_210_CRC 20



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TARGET_INFO { \
	210, \
	"TARGET_INFO", \
	10, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_target_info_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_info_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_info_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_info_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_info_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_target_info_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_target_info_t, vz) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_target_info_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_target_info_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_target_info_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TARGET_INFO { \
	"TARGET_INFO", \
	10, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_target_info_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_info_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_info_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_info_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_info_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_target_info_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_target_info_t, vz) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_target_info_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_target_info_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_target_info_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a target_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, float x, float y, float z, float vx, float vy, float vz, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TARGET_INFO_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, x);
	_mav_put_float(buf, 12, y);
	_mav_put_float(buf, 16, z);
	_mav_put_float(buf, 20, vx);
	_mav_put_float(buf, 24, vy);
	_mav_put_float(buf, 28, vz);
	_mav_put_float(buf, 32, roll);
	_mav_put_float(buf, 36, pitch);
	_mav_put_float(buf, 40, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_INFO_LEN);
#else
	mavlink_target_info_t packet;
	packet.time_usec = time_usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_INFO_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TARGET_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARGET_INFO_MIN_LEN, MAVLINK_MSG_ID_TARGET_INFO_LEN, MAVLINK_MSG_ID_TARGET_INFO_CRC);
}

/**
 * @brief Pack a target_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,float x,float y,float z,float vx,float vy,float vz,float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TARGET_INFO_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, x);
	_mav_put_float(buf, 12, y);
	_mav_put_float(buf, 16, z);
	_mav_put_float(buf, 20, vx);
	_mav_put_float(buf, 24, vy);
	_mav_put_float(buf, 28, vz);
	_mav_put_float(buf, 32, roll);
	_mav_put_float(buf, 36, pitch);
	_mav_put_float(buf, 40, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_INFO_LEN);
#else
	mavlink_target_info_t packet;
	packet.time_usec = time_usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_INFO_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TARGET_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARGET_INFO_MIN_LEN, MAVLINK_MSG_ID_TARGET_INFO_LEN, MAVLINK_MSG_ID_TARGET_INFO_CRC);
}

/**
 * @brief Encode a target_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param target_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_target_info_t* target_info)
{
	return mavlink_msg_target_info_pack(system_id, component_id, msg, target_info->time_usec, target_info->x, target_info->y, target_info->z, target_info->vx, target_info->vy, target_info->vz, target_info->roll, target_info->pitch, target_info->yaw);
}

/**
 * @brief Encode a target_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_target_info_t* target_info)
{
	return mavlink_msg_target_info_pack_chan(system_id, component_id, chan, msg, target_info->time_usec, target_info->x, target_info->y, target_info->z, target_info->vx, target_info->vy, target_info->vz, target_info->roll, target_info->pitch, target_info->yaw);
}

/**
 * @brief Send a target_info message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_target_info_send(mavlink_channel_t chan, uint64_t time_usec, float x, float y, float z, float vx, float vy, float vz, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TARGET_INFO_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, x);
	_mav_put_float(buf, 12, y);
	_mav_put_float(buf, 16, z);
	_mav_put_float(buf, 20, vx);
	_mav_put_float(buf, 24, vy);
	_mav_put_float(buf, 28, vz);
	_mav_put_float(buf, 32, roll);
	_mav_put_float(buf, 36, pitch);
	_mav_put_float(buf, 40, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_INFO, buf, MAVLINK_MSG_ID_TARGET_INFO_MIN_LEN, MAVLINK_MSG_ID_TARGET_INFO_LEN, MAVLINK_MSG_ID_TARGET_INFO_CRC);
#else
	mavlink_target_info_t packet;
	packet.time_usec = time_usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_INFO, (const char *)&packet, MAVLINK_MSG_ID_TARGET_INFO_MIN_LEN, MAVLINK_MSG_ID_TARGET_INFO_LEN, MAVLINK_MSG_ID_TARGET_INFO_CRC);
#endif
}

/**
 * @brief Send a target_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_target_info_send_struct(mavlink_channel_t chan, const mavlink_target_info_t* target_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_target_info_send(chan, target_info->time_usec, target_info->x, target_info->y, target_info->z, target_info->vx, target_info->vy, target_info->vz, target_info->roll, target_info->pitch, target_info->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_INFO, (const char *)target_info, MAVLINK_MSG_ID_TARGET_INFO_MIN_LEN, MAVLINK_MSG_ID_TARGET_INFO_LEN, MAVLINK_MSG_ID_TARGET_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_TARGET_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_target_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float x, float y, float z, float vx, float vy, float vz, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, x);
	_mav_put_float(buf, 12, y);
	_mav_put_float(buf, 16, z);
	_mav_put_float(buf, 20, vx);
	_mav_put_float(buf, 24, vy);
	_mav_put_float(buf, 28, vz);
	_mav_put_float(buf, 32, roll);
	_mav_put_float(buf, 36, pitch);
	_mav_put_float(buf, 40, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_INFO, buf, MAVLINK_MSG_ID_TARGET_INFO_MIN_LEN, MAVLINK_MSG_ID_TARGET_INFO_LEN, MAVLINK_MSG_ID_TARGET_INFO_CRC);
#else
	mavlink_target_info_t *packet = (mavlink_target_info_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_INFO, (const char *)packet, MAVLINK_MSG_ID_TARGET_INFO_MIN_LEN, MAVLINK_MSG_ID_TARGET_INFO_LEN, MAVLINK_MSG_ID_TARGET_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE TARGET_INFO UNPACKING


/**
 * @brief Get field time_usec from target_info message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_target_info_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field x from target_info message
 *
 * @return X Position
 */
static inline float mavlink_msg_target_info_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from target_info message
 *
 * @return Y Position
 */
static inline float mavlink_msg_target_info_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from target_info message
 *
 * @return Z Position
 */
static inline float mavlink_msg_target_info_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vx from target_info message
 *
 * @return X Speed
 */
static inline float mavlink_msg_target_info_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vy from target_info message
 *
 * @return Y Speed
 */
static inline float mavlink_msg_target_info_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vz from target_info message
 *
 * @return Z Speed
 */
static inline float mavlink_msg_target_info_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field roll from target_info message
 *
 * @return Roll angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_target_info_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field pitch from target_info message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_target_info_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field yaw from target_info message
 *
 * @return Yaw angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_target_info_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a target_info message into a struct
 *
 * @param msg The message to decode
 * @param target_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_target_info_decode(const mavlink_message_t* msg, mavlink_target_info_t* target_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	target_info->time_usec = mavlink_msg_target_info_get_time_usec(msg);
	target_info->x = mavlink_msg_target_info_get_x(msg);
	target_info->y = mavlink_msg_target_info_get_y(msg);
	target_info->z = mavlink_msg_target_info_get_z(msg);
	target_info->vx = mavlink_msg_target_info_get_vx(msg);
	target_info->vy = mavlink_msg_target_info_get_vy(msg);
	target_info->vz = mavlink_msg_target_info_get_vz(msg);
	target_info->roll = mavlink_msg_target_info_get_roll(msg);
	target_info->pitch = mavlink_msg_target_info_get_pitch(msg);
	target_info->yaw = mavlink_msg_target_info_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TARGET_INFO_LEN? msg->len : MAVLINK_MSG_ID_TARGET_INFO_LEN;
        memset(target_info, 0, MAVLINK_MSG_ID_TARGET_INFO_LEN);
	memcpy(target_info, _MAV_PAYLOAD(msg), len);
#endif
}
