/*
 * Dynamixel.cpp
 *
 *  Created on: Feb 23, 2013
 *      Author: rbtying
 */

#include "Dynamixel.h"

const static uint8_t BUF_LEN = 24;

uint8_t _msgbuf[BUF_LEN];

#define _L_(x) (static_cast<uint8_t>(x & 0xff))
#define _H_(x) (static_cast<uint8_t>(x >> 8u))
#define _W_(x,y) (x | (static_cast<uint16_t>(y) << 8u))

Dynamixel::Dynamixel(DynamixelManager *dyn, uint8_t id, uint8_t flags) {
	m_dyn = dyn;
	m_id = id;
	m_flags = flags;
	if (m_flags & WHEEL_MODE) {
		init_cont_turn();
	} else {
		set_angle_limits(MAX_POSITION, 0);
	}
}

void Dynamixel::write_id(uint8_t id) {
	write_address(_ID, id);
	m_id = id;
}

void Dynamixel::set_baud_rate(uint8_t baud) {
	write_address(_BAUD_RATE, baud);
}

void Dynamixel::set_return_delay(uint16_t delay) {
	// 2 us per value sent
	uint8_t scaled_delay = (delay >> 1) & 0xff;
	write_address(_RETURN_DELAY_TIME, scaled_delay);
}

void Dynamixel::set_angle_limits(uint16_t cw, uint16_t ccw) {
	uint8_t data[4];
	data[0] = _L_(cw);
	data[1] = _H_(cw);
	data[2] = _L_(ccw);
	data[3] = _H_(ccw);

	if (cw != 0 && ccw != 0) {
		m_flags &= ~WHEEL_MODE;
	} else {
		m_flags |= WHEEL_MODE;
	}

	write_address(_CW_ANGLE_LIMIT_L, data, 4);
}

void Dynamixel::init_cont_turn() {
	set_angle_limits(0, 0);
	m_flags |= WHEEL_MODE;
}

void Dynamixel::set_max_temp(uint8_t temp) {
	write_address(_TEMP_LIMIT, temp);
}

void Dynamixel::set_voltage_limits(float min, float max) {
	uint8_t data[2];
	data[0] = static_cast<uint8_t>(min * 10);
	data[1] = static_cast<uint8_t>(max * 10);
	write_address(_VOLTAGE_LOW_LIMIT, data, 2);
}

void Dynamixel::set_max_torque(uint16_t tmax) {
	uint8_t data[2];
	data[0] = _L_(tmax);
	data[1] = _H_(tmax);
	write_address(_TORQUE_MAX_L, data, 2);
}

void Dynamixel::set_status_return_level(uint8_t level) {
	write_address(_STATUS_RETURN_LVL, level);
}

void Dynamixel::set_alarm_led(uint8_t errors) {
	write_address(_ALARM_LED, errors);
}
void Dynamixel::set_alarm_shutdown(uint8_t errors) {
	write_address(_ALARM_SHUTDOWN, errors);
}

void Dynamixel::enable_torque() {
	write_address(_TORQUE_ENABLE, true);
}

void Dynamixel::disable_torque() {
	write_address(_TORQUE_ENABLE, false);
}

void Dynamixel::set_led(uint8_t on) {
	write_address(_LED, on ? true : false);
}

void Dynamixel::set_pid(uint8_t p, uint8_t i, uint8_t d) {
	uint8_t data[3] = { d, i, p };
	write_address(_D_GAIN, data, 3);
}

void Dynamixel::set_goal_position(uint16_t position) {
	position = constrain(position, 0, MAX_POSITION);
	if (m_flags & FLIP_DIRECTION) {
		position = MAX_POSITION - position;
	}
	uint8_t data[2];
	data[0] = _L_(position);
	data[1] = _H_(position);
	write_address(_GOAL_POS_L, data, 2);
}

void Dynamixel::set_goal_speed(float rpm) {
	int16_t speed = rpm * 0.114;

	set_goal_speed(speed);
}

void Dynamixel::set_goal_speed(int16_t speed) {
	speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
	if (m_flags & FLIP_DIRECTION) {
		speed = -speed;
	}
	if (!(m_flags & WHEEL_MODE)) {
		speed = abs(speed);
	}
	uint8_t data[2];
	data[0] = _L_(speed);
	data[1] = _H_(speed);
	write_address(_MOVE_SPEED_L, data, 2);
}

void Dynamixel::set_torque_limit(float percent) {
	uint16_t limit = 1023 * percent;
	uint8_t data[2];
	data[0] = _L_(limit);
	data[1] = _H_(limit);
	write_address(_TORQUE_LIMIT_L, data, 2);
}

uint16_t Dynamixel::get_position() {
	read_address(_CUR_POSITION_L, 2);
	uint16_t pos = _W_(_msgbuf[0], _msgbuf[1]);
	return pos;
}

int16_t Dynamixel::get_speed() {
	read_address(_CUR_SPEED_L, 2);
	int16_t val = ((((_msgbuf[1] >> 2) & 1) == 0) ? -1 : 1)
			* (_msgbuf[0] | ((_msgbuf[1] >> 6u) << 8u));
	if (m_flags & FLIP_DIRECTION) {
		val = -val;
	}
	return val;
}

int16_t Dynamixel::get_load() {
	read_address(_CUR_LOAD_L, 2);
	int16_t val = ((((_msgbuf[1] >> 2) & 1) == 0) ? -1 : 1)
			* (_msgbuf[0] | ((_msgbuf[1] >> 6u) << 8u));
	if (m_flags & FLIP_DIRECTION) {
		val = -val;
	}
	return val;
}

float Dynamixel::get_voltage() {
	read_address(_CUR_VOLTAGE, 1);
	return _msgbuf[0] * 0.1;
}

uint8_t Dynamixel::get_temperature() {
	read_address(_CUR_TEMP, 1);
	return _msgbuf[0];
}

uint8_t Dynamixel::is_moving() {
	read_address(_MOVING, 1);
	return _msgbuf[0] != 0;
}

void Dynamixel::lock_eeprom() {
	write_address(_LOCK_EEPROM, true);
}

void Dynamixel::set_acceleration(float accel) {
	uint8_t a = accel * (1 / 8.853);
	set_acceleration(a);
}

void Dynamixel::set_acceleration(uint8_t accel) {
	write_address(_ACCELERATION, accel);
}

uint8_t Dynamixel::read_address(uint8_t addr, uint8_t len) {
	return transaction(_CMD_WRITE, addr, &len, 1);
}

uint8_t Dynamixel::write_address(uint8_t addr, uint8_t data) {
	return write_address(addr, &data, 1);
}

uint8_t Dynamixel::write_address(uint8_t addr, uint8_t *data, uint8_t datalen) {
	return transaction(_CMD_READ, addr, data, datalen);
}

uint8_t Dynamixel::transaction(uint8_t cmd, uint8_t addr, uint8_t *data,
		uint8_t datalen) {
	memset(_msgbuf, 0, BUF_LEN);
#define HEADER_LEN 6
	_msgbuf[0] = 0xff;
	_msgbuf[1] = 0xff;
	_msgbuf[2] = m_id;
	_msgbuf[3] = datalen + 3;
	_msgbuf[4] = cmd;
	_msgbuf[5] = addr;
	/* _msgbuf[6] to _msgbuf[datalen + 6 - 1] */
	memcpy(_msgbuf + HEADER_LEN, data, datalen);
	_msgbuf[HEADER_LEN + datalen] = checksum(_msgbuf + 2, _msgbuf[3] + 1);

	m_dyn->send(_msgbuf, datalen + HEADER_LEN + 1);

// clear buffer
	memset(_msgbuf, 0, BUF_LEN);

// begin read
	m_dyn->read(_msgbuf, 5);
	if (_msgbuf[0] == 0xff && _msgbuf[1] == 0xff) {
		// check start packet
		if (_msgbuf[2] == m_id) {
			uint8_t len = _msgbuf[3];
//			uint8_t error = _msgbuf[4];
			memset(_msgbuf, 0, BUF_LEN);
			return m_dyn->read(_msgbuf, len);
		} else {
			// wrong servo id
		}
	}
#undef HEADER_LEN
	return 0;
}

uint8_t Dynamixel::checksum(uint8_t *data, uint8_t len) {
	uint32_t sum = 0;
	for (uint8_t i = 0; i < len; i++) {
		sum += data[i];
	}
	return (!sum) & 0xff;
}

Dynamixel::~Dynamixel() {
// TODO Auto-generated destructor stub
}

