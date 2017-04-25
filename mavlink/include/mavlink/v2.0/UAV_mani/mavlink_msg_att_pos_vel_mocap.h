// MESSAGE ATT_POS_VEL_MOCAP PACKING

#define MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP 209

MAVPACKED(
typedef struct __mavlink_att_pos_vel_mocap_t {
 uint64_t time_usec; /*< Timestamp*/
 float q[4]; /*< Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)*/
 float x; /*< X position in meters (NED)*/
 float y; /*< Y position in meters (NED)*/
 float z; /*< Z position in meters (NED)*/
 float Vx; /*< X velocity in m/s (NED)*/
 float Vy; /*< Y velocity in m/s (NED)*/
 float Vz; /*< Z velocity in m/s (NED)*/
}) mavlink_att_pos_vel_mocap_t;

#define MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN 48
#define MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_MIN_LEN 48
#define MAVLINK_MSG_ID_209_LEN 48
#define MAVLINK_MSG_ID_209_MIN_LEN 48

#define MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_CRC 99
#define MAVLINK_MSG_ID_209_CRC 99

#define MAVLINK_MSG_ATT_POS_VEL_MOCAP_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATT_POS_VEL_MOCAP { \
	209, \
	"ATT_POS_VEL_MOCAP", \
	8, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_att_pos_vel_mocap_t, time_usec) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_att_pos_vel_mocap_t, q) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_att_pos_vel_mocap_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_att_pos_vel_mocap_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_att_pos_vel_mocap_t, z) }, \
         { "Vx", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_att_pos_vel_mocap_t, Vx) }, \
         { "Vy", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_att_pos_vel_mocap_t, Vy) }, \
         { "Vz", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_att_pos_vel_mocap_t, Vz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATT_POS_VEL_MOCAP { \
	"ATT_POS_VEL_MOCAP", \
	8, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_att_pos_vel_mocap_t, time_usec) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_att_pos_vel_mocap_t, q) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_att_pos_vel_mocap_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_att_pos_vel_mocap_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_att_pos_vel_mocap_t, z) }, \
         { "Vx", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_att_pos_vel_mocap_t, Vx) }, \
         { "Vy", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_att_pos_vel_mocap_t, Vy) }, \
         { "Vz", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_att_pos_vel_mocap_t, Vz) }, \
         } \
}
#endif

/**
 * @brief Pack a att_pos_vel_mocap message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp
 * @param q Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param x X position in meters (NED)
 * @param y Y position in meters (NED)
 * @param z Z position in meters (NED)
 * @param Vx X velocity in m/s (NED)
 * @param Vy Y velocity in m/s (NED)
 * @param Vz Z velocity in m/s (NED)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_att_pos_vel_mocap_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, const float *q, float x, float y, float z, float Vx, float Vy, float Vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 24, x);
	_mav_put_float(buf, 28, y);
	_mav_put_float(buf, 32, z);
	_mav_put_float(buf, 36, Vx);
	_mav_put_float(buf, 40, Vy);
	_mav_put_float(buf, 44, Vz);
	_mav_put_float_array(buf, 8, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN);
#else
	mavlink_att_pos_vel_mocap_t packet;
	packet.time_usec = time_usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.Vx = Vx;
	packet.Vy = Vy;
	packet.Vz = Vz;
	mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_CRC);
}

/**
 * @brief Pack a att_pos_vel_mocap message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp
 * @param q Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param x X position in meters (NED)
 * @param y Y position in meters (NED)
 * @param z Z position in meters (NED)
 * @param Vx X velocity in m/s (NED)
 * @param Vy Y velocity in m/s (NED)
 * @param Vz Z velocity in m/s (NED)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_att_pos_vel_mocap_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,const float *q,float x,float y,float z,float Vx,float Vy,float Vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 24, x);
	_mav_put_float(buf, 28, y);
	_mav_put_float(buf, 32, z);
	_mav_put_float(buf, 36, Vx);
	_mav_put_float(buf, 40, Vy);
	_mav_put_float(buf, 44, Vz);
	_mav_put_float_array(buf, 8, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN);
#else
	mavlink_att_pos_vel_mocap_t packet;
	packet.time_usec = time_usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.Vx = Vx;
	packet.Vy = Vy;
	packet.Vz = Vz;
	mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_CRC);
}

/**
 * @brief Encode a att_pos_vel_mocap struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param att_pos_vel_mocap C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_att_pos_vel_mocap_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_att_pos_vel_mocap_t* att_pos_vel_mocap)
{
	return mavlink_msg_att_pos_vel_mocap_pack(system_id, component_id, msg, att_pos_vel_mocap->time_usec, att_pos_vel_mocap->q, att_pos_vel_mocap->x, att_pos_vel_mocap->y, att_pos_vel_mocap->z, att_pos_vel_mocap->Vx, att_pos_vel_mocap->Vy, att_pos_vel_mocap->Vz);
}

/**
 * @brief Encode a att_pos_vel_mocap struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param att_pos_vel_mocap C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_att_pos_vel_mocap_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_att_pos_vel_mocap_t* att_pos_vel_mocap)
{
	return mavlink_msg_att_pos_vel_mocap_pack_chan(system_id, component_id, chan, msg, att_pos_vel_mocap->time_usec, att_pos_vel_mocap->q, att_pos_vel_mocap->x, att_pos_vel_mocap->y, att_pos_vel_mocap->z, att_pos_vel_mocap->Vx, att_pos_vel_mocap->Vy, att_pos_vel_mocap->Vz);
}

/**
 * @brief Send a att_pos_vel_mocap message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp
 * @param q Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param x X position in meters (NED)
 * @param y Y position in meters (NED)
 * @param z Z position in meters (NED)
 * @param Vx X velocity in m/s (NED)
 * @param Vy Y velocity in m/s (NED)
 * @param Vz Z velocity in m/s (NED)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_att_pos_vel_mocap_send(mavlink_channel_t chan, uint64_t time_usec, const float *q, float x, float y, float z, float Vx, float Vy, float Vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 24, x);
	_mav_put_float(buf, 28, y);
	_mav_put_float(buf, 32, z);
	_mav_put_float(buf, 36, Vx);
	_mav_put_float(buf, 40, Vy);
	_mav_put_float(buf, 44, Vz);
	_mav_put_float_array(buf, 8, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP, buf, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_CRC);
#else
	mavlink_att_pos_vel_mocap_t packet;
	packet.time_usec = time_usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.Vx = Vx;
	packet.Vy = Vy;
	packet.Vz = Vz;
	mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP, (const char *)&packet, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_CRC);
#endif
}

/**
 * @brief Send a att_pos_vel_mocap message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_att_pos_vel_mocap_send_struct(mavlink_channel_t chan, const mavlink_att_pos_vel_mocap_t* att_pos_vel_mocap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_att_pos_vel_mocap_send(chan, att_pos_vel_mocap->time_usec, att_pos_vel_mocap->q, att_pos_vel_mocap->x, att_pos_vel_mocap->y, att_pos_vel_mocap->z, att_pos_vel_mocap->Vx, att_pos_vel_mocap->Vy, att_pos_vel_mocap->Vz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP, (const char *)att_pos_vel_mocap, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_att_pos_vel_mocap_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, const float *q, float x, float y, float z, float Vx, float Vy, float Vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 24, x);
	_mav_put_float(buf, 28, y);
	_mav_put_float(buf, 32, z);
	_mav_put_float(buf, 36, Vx);
	_mav_put_float(buf, 40, Vy);
	_mav_put_float(buf, 44, Vz);
	_mav_put_float_array(buf, 8, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP, buf, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_CRC);
#else
	mavlink_att_pos_vel_mocap_t *packet = (mavlink_att_pos_vel_mocap_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->Vx = Vx;
	packet->Vy = Vy;
	packet->Vz = Vz;
	mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP, (const char *)packet, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_CRC);
#endif
}
#endif

#endif

// MESSAGE ATT_POS_VEL_MOCAP UNPACKING


/**
 * @brief Get field time_usec from att_pos_vel_mocap message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_att_pos_vel_mocap_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field q from att_pos_vel_mocap message
 *
 * @return Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 */
static inline uint16_t mavlink_msg_att_pos_vel_mocap_get_q(const mavlink_message_t* msg, float *q)
{
	return _MAV_RETURN_float_array(msg, q, 4,  8);
}

/**
 * @brief Get field x from att_pos_vel_mocap message
 *
 * @return X position in meters (NED)
 */
static inline float mavlink_msg_att_pos_vel_mocap_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field y from att_pos_vel_mocap message
 *
 * @return Y position in meters (NED)
 */
static inline float mavlink_msg_att_pos_vel_mocap_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field z from att_pos_vel_mocap message
 *
 * @return Z position in meters (NED)
 */
static inline float mavlink_msg_att_pos_vel_mocap_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field Vx from att_pos_vel_mocap message
 *
 * @return X velocity in m/s (NED)
 */
static inline float mavlink_msg_att_pos_vel_mocap_get_Vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field Vy from att_pos_vel_mocap message
 *
 * @return Y velocity in m/s (NED)
 */
static inline float mavlink_msg_att_pos_vel_mocap_get_Vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field Vz from att_pos_vel_mocap message
 *
 * @return Z velocity in m/s (NED)
 */
static inline float mavlink_msg_att_pos_vel_mocap_get_Vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a att_pos_vel_mocap message into a struct
 *
 * @param msg The message to decode
 * @param att_pos_vel_mocap C-struct to decode the message contents into
 */
static inline void mavlink_msg_att_pos_vel_mocap_decode(const mavlink_message_t* msg, mavlink_att_pos_vel_mocap_t* att_pos_vel_mocap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	att_pos_vel_mocap->time_usec = mavlink_msg_att_pos_vel_mocap_get_time_usec(msg);
	mavlink_msg_att_pos_vel_mocap_get_q(msg, att_pos_vel_mocap->q);
	att_pos_vel_mocap->x = mavlink_msg_att_pos_vel_mocap_get_x(msg);
	att_pos_vel_mocap->y = mavlink_msg_att_pos_vel_mocap_get_y(msg);
	att_pos_vel_mocap->z = mavlink_msg_att_pos_vel_mocap_get_z(msg);
	att_pos_vel_mocap->Vx = mavlink_msg_att_pos_vel_mocap_get_Vx(msg);
	att_pos_vel_mocap->Vy = mavlink_msg_att_pos_vel_mocap_get_Vy(msg);
	att_pos_vel_mocap->Vz = mavlink_msg_att_pos_vel_mocap_get_Vz(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN? msg->len : MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN;
        memset(att_pos_vel_mocap, 0, MAVLINK_MSG_ID_ATT_POS_VEL_MOCAP_LEN);
	memcpy(att_pos_vel_mocap, _MAV_PAYLOAD(msg), len);
#endif
}
