#pragma once
// MESSAGE ULTRAMOTION_SB PACKING

#define MAVLINK_MSG_ID_ULTRAMOTION_SB 181


typedef struct __mavlink_ultramotion_sb_t {
 float pos_cmd; /*<  Commanded position [-1, 1]*/
 float pos; /*<  Sensed position [-1, 1]*/
 float current; /*<  Sensed current [0, 1]*/
 float temp_degc; /*<  Actuator temperature*/
 uint8_t over_torque; /*<  Over torque*/
 uint8_t at_target; /*<  At target*/
 uint8_t following_error; /*<  Following error*/
 uint8_t speed_below_threshold; /*<  Speed is below threshold*/
 uint8_t rx_error; /*<  Rx error*/
 uint8_t tx_error; /*<  Tx error*/
} mavlink_ultramotion_sb_t;

#define MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN 22
#define MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN 22
#define MAVLINK_MSG_ID_181_LEN 22
#define MAVLINK_MSG_ID_181_MIN_LEN 22

#define MAVLINK_MSG_ID_ULTRAMOTION_SB_CRC 137
#define MAVLINK_MSG_ID_181_CRC 137



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ULTRAMOTION_SB { \
    181, \
    "ULTRAMOTION_SB", \
    10, \
    {  { "pos_cmd", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ultramotion_sb_t, pos_cmd) }, \
         { "pos", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ultramotion_sb_t, pos) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ultramotion_sb_t, current) }, \
         { "temp_degc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ultramotion_sb_t, temp_degc) }, \
         { "over_torque", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_ultramotion_sb_t, over_torque) }, \
         { "at_target", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_ultramotion_sb_t, at_target) }, \
         { "following_error", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_ultramotion_sb_t, following_error) }, \
         { "speed_below_threshold", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_ultramotion_sb_t, speed_below_threshold) }, \
         { "rx_error", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_ultramotion_sb_t, rx_error) }, \
         { "tx_error", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_ultramotion_sb_t, tx_error) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ULTRAMOTION_SB { \
    "ULTRAMOTION_SB", \
    10, \
    {  { "pos_cmd", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ultramotion_sb_t, pos_cmd) }, \
         { "pos", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ultramotion_sb_t, pos) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ultramotion_sb_t, current) }, \
         { "temp_degc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ultramotion_sb_t, temp_degc) }, \
         { "over_torque", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_ultramotion_sb_t, over_torque) }, \
         { "at_target", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_ultramotion_sb_t, at_target) }, \
         { "following_error", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_ultramotion_sb_t, following_error) }, \
         { "speed_below_threshold", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_ultramotion_sb_t, speed_below_threshold) }, \
         { "rx_error", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_ultramotion_sb_t, rx_error) }, \
         { "tx_error", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_ultramotion_sb_t, tx_error) }, \
         } \
}
#endif

/**
 * @brief Pack a ultramotion_sb message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_cmd  Commanded position [-1, 1]
 * @param pos  Sensed position [-1, 1]
 * @param current  Sensed current [0, 1]
 * @param temp_degc  Actuator temperature
 * @param over_torque  Over torque
 * @param at_target  At target
 * @param following_error  Following error
 * @param speed_below_threshold  Speed is below threshold
 * @param rx_error  Rx error
 * @param tx_error  Tx error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ultramotion_sb_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pos_cmd, float pos, float current, float temp_degc, uint8_t over_torque, uint8_t at_target, uint8_t following_error, uint8_t speed_below_threshold, uint8_t rx_error, uint8_t tx_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN];
    _mav_put_float(buf, 0, pos_cmd);
    _mav_put_float(buf, 4, pos);
    _mav_put_float(buf, 8, current);
    _mav_put_float(buf, 12, temp_degc);
    _mav_put_uint8_t(buf, 16, over_torque);
    _mav_put_uint8_t(buf, 17, at_target);
    _mav_put_uint8_t(buf, 18, following_error);
    _mav_put_uint8_t(buf, 19, speed_below_threshold);
    _mav_put_uint8_t(buf, 20, rx_error);
    _mav_put_uint8_t(buf, 21, tx_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN);
#else
    mavlink_ultramotion_sb_t packet;
    packet.pos_cmd = pos_cmd;
    packet.pos = pos;
    packet.current = current;
    packet.temp_degc = temp_degc;
    packet.over_torque = over_torque;
    packet.at_target = at_target;
    packet.following_error = following_error;
    packet.speed_below_threshold = speed_below_threshold;
    packet.rx_error = rx_error;
    packet.tx_error = tx_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ULTRAMOTION_SB;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_CRC);
}

/**
 * @brief Pack a ultramotion_sb message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_cmd  Commanded position [-1, 1]
 * @param pos  Sensed position [-1, 1]
 * @param current  Sensed current [0, 1]
 * @param temp_degc  Actuator temperature
 * @param over_torque  Over torque
 * @param at_target  At target
 * @param following_error  Following error
 * @param speed_below_threshold  Speed is below threshold
 * @param rx_error  Rx error
 * @param tx_error  Tx error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ultramotion_sb_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float pos_cmd, float pos, float current, float temp_degc, uint8_t over_torque, uint8_t at_target, uint8_t following_error, uint8_t speed_below_threshold, uint8_t rx_error, uint8_t tx_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN];
    _mav_put_float(buf, 0, pos_cmd);
    _mav_put_float(buf, 4, pos);
    _mav_put_float(buf, 8, current);
    _mav_put_float(buf, 12, temp_degc);
    _mav_put_uint8_t(buf, 16, over_torque);
    _mav_put_uint8_t(buf, 17, at_target);
    _mav_put_uint8_t(buf, 18, following_error);
    _mav_put_uint8_t(buf, 19, speed_below_threshold);
    _mav_put_uint8_t(buf, 20, rx_error);
    _mav_put_uint8_t(buf, 21, tx_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN);
#else
    mavlink_ultramotion_sb_t packet;
    packet.pos_cmd = pos_cmd;
    packet.pos = pos;
    packet.current = current;
    packet.temp_degc = temp_degc;
    packet.over_torque = over_torque;
    packet.at_target = at_target;
    packet.following_error = following_error;
    packet.speed_below_threshold = speed_below_threshold;
    packet.rx_error = rx_error;
    packet.tx_error = tx_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ULTRAMOTION_SB;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN);
#endif
}

/**
 * @brief Pack a ultramotion_sb message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pos_cmd  Commanded position [-1, 1]
 * @param pos  Sensed position [-1, 1]
 * @param current  Sensed current [0, 1]
 * @param temp_degc  Actuator temperature
 * @param over_torque  Over torque
 * @param at_target  At target
 * @param following_error  Following error
 * @param speed_below_threshold  Speed is below threshold
 * @param rx_error  Rx error
 * @param tx_error  Tx error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ultramotion_sb_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pos_cmd,float pos,float current,float temp_degc,uint8_t over_torque,uint8_t at_target,uint8_t following_error,uint8_t speed_below_threshold,uint8_t rx_error,uint8_t tx_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN];
    _mav_put_float(buf, 0, pos_cmd);
    _mav_put_float(buf, 4, pos);
    _mav_put_float(buf, 8, current);
    _mav_put_float(buf, 12, temp_degc);
    _mav_put_uint8_t(buf, 16, over_torque);
    _mav_put_uint8_t(buf, 17, at_target);
    _mav_put_uint8_t(buf, 18, following_error);
    _mav_put_uint8_t(buf, 19, speed_below_threshold);
    _mav_put_uint8_t(buf, 20, rx_error);
    _mav_put_uint8_t(buf, 21, tx_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN);
#else
    mavlink_ultramotion_sb_t packet;
    packet.pos_cmd = pos_cmd;
    packet.pos = pos;
    packet.current = current;
    packet.temp_degc = temp_degc;
    packet.over_torque = over_torque;
    packet.at_target = at_target;
    packet.following_error = following_error;
    packet.speed_below_threshold = speed_below_threshold;
    packet.rx_error = rx_error;
    packet.tx_error = tx_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ULTRAMOTION_SB;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_CRC);
}

/**
 * @brief Encode a ultramotion_sb struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ultramotion_sb C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ultramotion_sb_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ultramotion_sb_t* ultramotion_sb)
{
    return mavlink_msg_ultramotion_sb_pack(system_id, component_id, msg, ultramotion_sb->pos_cmd, ultramotion_sb->pos, ultramotion_sb->current, ultramotion_sb->temp_degc, ultramotion_sb->over_torque, ultramotion_sb->at_target, ultramotion_sb->following_error, ultramotion_sb->speed_below_threshold, ultramotion_sb->rx_error, ultramotion_sb->tx_error);
}

/**
 * @brief Encode a ultramotion_sb struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ultramotion_sb C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ultramotion_sb_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ultramotion_sb_t* ultramotion_sb)
{
    return mavlink_msg_ultramotion_sb_pack_chan(system_id, component_id, chan, msg, ultramotion_sb->pos_cmd, ultramotion_sb->pos, ultramotion_sb->current, ultramotion_sb->temp_degc, ultramotion_sb->over_torque, ultramotion_sb->at_target, ultramotion_sb->following_error, ultramotion_sb->speed_below_threshold, ultramotion_sb->rx_error, ultramotion_sb->tx_error);
}

/**
 * @brief Encode a ultramotion_sb struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ultramotion_sb C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ultramotion_sb_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_ultramotion_sb_t* ultramotion_sb)
{
    return mavlink_msg_ultramotion_sb_pack_status(system_id, component_id, _status, msg,  ultramotion_sb->pos_cmd, ultramotion_sb->pos, ultramotion_sb->current, ultramotion_sb->temp_degc, ultramotion_sb->over_torque, ultramotion_sb->at_target, ultramotion_sb->following_error, ultramotion_sb->speed_below_threshold, ultramotion_sb->rx_error, ultramotion_sb->tx_error);
}

/**
 * @brief Send a ultramotion_sb message
 * @param chan MAVLink channel to send the message
 *
 * @param pos_cmd  Commanded position [-1, 1]
 * @param pos  Sensed position [-1, 1]
 * @param current  Sensed current [0, 1]
 * @param temp_degc  Actuator temperature
 * @param over_torque  Over torque
 * @param at_target  At target
 * @param following_error  Following error
 * @param speed_below_threshold  Speed is below threshold
 * @param rx_error  Rx error
 * @param tx_error  Tx error
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ultramotion_sb_send(mavlink_channel_t chan, float pos_cmd, float pos, float current, float temp_degc, uint8_t over_torque, uint8_t at_target, uint8_t following_error, uint8_t speed_below_threshold, uint8_t rx_error, uint8_t tx_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN];
    _mav_put_float(buf, 0, pos_cmd);
    _mav_put_float(buf, 4, pos);
    _mav_put_float(buf, 8, current);
    _mav_put_float(buf, 12, temp_degc);
    _mav_put_uint8_t(buf, 16, over_torque);
    _mav_put_uint8_t(buf, 17, at_target);
    _mav_put_uint8_t(buf, 18, following_error);
    _mav_put_uint8_t(buf, 19, speed_below_threshold);
    _mav_put_uint8_t(buf, 20, rx_error);
    _mav_put_uint8_t(buf, 21, tx_error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ULTRAMOTION_SB, buf, MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_CRC);
#else
    mavlink_ultramotion_sb_t packet;
    packet.pos_cmd = pos_cmd;
    packet.pos = pos;
    packet.current = current;
    packet.temp_degc = temp_degc;
    packet.over_torque = over_torque;
    packet.at_target = at_target;
    packet.following_error = following_error;
    packet.speed_below_threshold = speed_below_threshold;
    packet.rx_error = rx_error;
    packet.tx_error = tx_error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ULTRAMOTION_SB, (const char *)&packet, MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_CRC);
#endif
}

/**
 * @brief Send a ultramotion_sb message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ultramotion_sb_send_struct(mavlink_channel_t chan, const mavlink_ultramotion_sb_t* ultramotion_sb)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ultramotion_sb_send(chan, ultramotion_sb->pos_cmd, ultramotion_sb->pos, ultramotion_sb->current, ultramotion_sb->temp_degc, ultramotion_sb->over_torque, ultramotion_sb->at_target, ultramotion_sb->following_error, ultramotion_sb->speed_below_threshold, ultramotion_sb->rx_error, ultramotion_sb->tx_error);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ULTRAMOTION_SB, (const char *)ultramotion_sb, MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_CRC);
#endif
}

#if MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ultramotion_sb_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pos_cmd, float pos, float current, float temp_degc, uint8_t over_torque, uint8_t at_target, uint8_t following_error, uint8_t speed_below_threshold, uint8_t rx_error, uint8_t tx_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pos_cmd);
    _mav_put_float(buf, 4, pos);
    _mav_put_float(buf, 8, current);
    _mav_put_float(buf, 12, temp_degc);
    _mav_put_uint8_t(buf, 16, over_torque);
    _mav_put_uint8_t(buf, 17, at_target);
    _mav_put_uint8_t(buf, 18, following_error);
    _mav_put_uint8_t(buf, 19, speed_below_threshold);
    _mav_put_uint8_t(buf, 20, rx_error);
    _mav_put_uint8_t(buf, 21, tx_error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ULTRAMOTION_SB, buf, MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_CRC);
#else
    mavlink_ultramotion_sb_t *packet = (mavlink_ultramotion_sb_t *)msgbuf;
    packet->pos_cmd = pos_cmd;
    packet->pos = pos;
    packet->current = current;
    packet->temp_degc = temp_degc;
    packet->over_torque = over_torque;
    packet->at_target = at_target;
    packet->following_error = following_error;
    packet->speed_below_threshold = speed_below_threshold;
    packet->rx_error = rx_error;
    packet->tx_error = tx_error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ULTRAMOTION_SB, (const char *)packet, MAVLINK_MSG_ID_ULTRAMOTION_SB_MIN_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN, MAVLINK_MSG_ID_ULTRAMOTION_SB_CRC);
#endif
}
#endif

#endif

// MESSAGE ULTRAMOTION_SB UNPACKING


/**
 * @brief Get field pos_cmd from ultramotion_sb message
 *
 * @return  Commanded position [-1, 1]
 */
static inline float mavlink_msg_ultramotion_sb_get_pos_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pos from ultramotion_sb message
 *
 * @return  Sensed position [-1, 1]
 */
static inline float mavlink_msg_ultramotion_sb_get_pos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field current from ultramotion_sb message
 *
 * @return  Sensed current [0, 1]
 */
static inline float mavlink_msg_ultramotion_sb_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field temp_degc from ultramotion_sb message
 *
 * @return  Actuator temperature
 */
static inline float mavlink_msg_ultramotion_sb_get_temp_degc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field over_torque from ultramotion_sb message
 *
 * @return  Over torque
 */
static inline uint8_t mavlink_msg_ultramotion_sb_get_over_torque(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field at_target from ultramotion_sb message
 *
 * @return  At target
 */
static inline uint8_t mavlink_msg_ultramotion_sb_get_at_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field following_error from ultramotion_sb message
 *
 * @return  Following error
 */
static inline uint8_t mavlink_msg_ultramotion_sb_get_following_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field speed_below_threshold from ultramotion_sb message
 *
 * @return  Speed is below threshold
 */
static inline uint8_t mavlink_msg_ultramotion_sb_get_speed_below_threshold(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field rx_error from ultramotion_sb message
 *
 * @return  Rx error
 */
static inline uint8_t mavlink_msg_ultramotion_sb_get_rx_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field tx_error from ultramotion_sb message
 *
 * @return  Tx error
 */
static inline uint8_t mavlink_msg_ultramotion_sb_get_tx_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Decode a ultramotion_sb message into a struct
 *
 * @param msg The message to decode
 * @param ultramotion_sb C-struct to decode the message contents into
 */
static inline void mavlink_msg_ultramotion_sb_decode(const mavlink_message_t* msg, mavlink_ultramotion_sb_t* ultramotion_sb)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ultramotion_sb->pos_cmd = mavlink_msg_ultramotion_sb_get_pos_cmd(msg);
    ultramotion_sb->pos = mavlink_msg_ultramotion_sb_get_pos(msg);
    ultramotion_sb->current = mavlink_msg_ultramotion_sb_get_current(msg);
    ultramotion_sb->temp_degc = mavlink_msg_ultramotion_sb_get_temp_degc(msg);
    ultramotion_sb->over_torque = mavlink_msg_ultramotion_sb_get_over_torque(msg);
    ultramotion_sb->at_target = mavlink_msg_ultramotion_sb_get_at_target(msg);
    ultramotion_sb->following_error = mavlink_msg_ultramotion_sb_get_following_error(msg);
    ultramotion_sb->speed_below_threshold = mavlink_msg_ultramotion_sb_get_speed_below_threshold(msg);
    ultramotion_sb->rx_error = mavlink_msg_ultramotion_sb_get_rx_error(msg);
    ultramotion_sb->tx_error = mavlink_msg_ultramotion_sb_get_tx_error(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN? msg->len : MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN;
        memset(ultramotion_sb, 0, MAVLINK_MSG_ID_ULTRAMOTION_SB_LEN);
    memcpy(ultramotion_sb, _MAV_PAYLOAD(msg), len);
#endif
}
