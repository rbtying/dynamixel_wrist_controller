/*
 * Dynamixel.h
 *
 *  Created on: Feb 23, 2013
 *      Author: rbtying
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include <Arduino.h>
#include "DynamixelManager.h"

class Dynamixel {
public:
	Dynamixel(DynamixelManager *dyn, uint8_t id, uint8_t flags);
	virtual ~Dynamixel();

	void write_id(uint8_t id);
	void set_baud_rate(uint8_t baud);
	void set_return_delay(uint16_t delay);
	void set_angle_limits(uint16_t cw, uint16_t ccw);
	void init_cont_turn();
	void set_max_temp(uint8_t temp);
	void set_voltage_limits(float min, float max);
	void set_max_torque(uint16_t tmax);
	void set_status_return_level(uint8_t level);
	void set_alarm_led(uint8_t errors);
	void set_alarm_shutdown(uint8_t errors);
	void enable_torque();
	void disable_torque();
	void set_led(uint8_t on);
	void set_pid(uint8_t p, uint8_t i, uint8_t d);
	void set_goal_position(uint16_t position);
	void set_goal_speed(float rpm);
	void set_goal_speed(int16_t spd);
	void set_acceleration(float accel);
	void set_acceleration(uint8_t accel);
	void set_torque_limit(float percent);

	uint16_t get_position();
	int16_t get_speed();
	int16_t get_load();
	float get_voltage();
	uint8_t get_temperature();
	uint8_t is_moving();
	void lock_eeprom();

private:
	DynamixelManager *m_dyn;
	uint8_t m_id;
	uint8_t m_flags;

	uint8_t transaction(uint8_t cmd, uint8_t addr, uint8_t *data,
			uint8_t datalen);
	uint8_t checksum(uint8_t *data, uint8_t datalen);
	uint8_t write_address(uint8_t addr, uint8_t *data, uint8_t datalen);
	uint8_t write_address(uint8_t addr, uint8_t data);
	uint8_t read_address(uint8_t addr, uint8_t len = 1);

public:
	static const uint8_t FLIP_DIRECTION = (1 << 0);
	static const uint8_t WHEEL_MODE = (1 << 1);

	static const uint16_t MAX_POSITION = 0xfff;
	static const int16_t MAX_SPEED = 1023;

	enum {
		BAUD_RATE_1000000 = 1,
		BAUD_RATE_500000 = 3,
		BAUD_RATE_400000 = 4,
		BAUD_RATE_250000 = 7,
		BAUD_RATE_200000 = 9,
		BAUD_RATE_115200 = 16,
		BAUD_RATE_57600 = 34,
		BAUD_RATE_19200 = 103,
		BAUD_RATE_9600 = 207
	};

	enum {
		ERROR_INSTRUCTION = 1 << 6,
		ERROR_OVERLOAD = 1 << 5,
		ERROR_CHECKSUM = 1 << 4,
		ERROR_RANGE = 1 << 3,
		ERROR_HEAT = 1 << 2,
		ERROR_ANGLE_LIMIT = 1 << 1,
		ERROR_VOLTAGE = 1 << 0,
	};

private:
	static const uint8_t _CMD_WRITE = 0x03;
	static const uint8_t _CMD_READ = 0x02;

	enum {
		_MODEL_NUMBER_H = 0x00,
		_MODEL_NUMBER_L = 0x01,
		_FIRMWARE_VERSION = 0x02,
		_ID = 0x03,
		_BAUD_RATE = 0x04,
		_RETURN_DELAY_TIME = 0x05,
		_CW_ANGLE_LIMIT_L = 0x06,
		_CW_ANGLE_LIMIT_H = 0x07,
		_CCW_ANGLE_LIMIT_L = 0x08,
		_CCW_ANGLE_LIMIT_H = 0x09,
		_TEMP_LIMIT = 0x0b,
		_VOLTAGE_LOW_LIMIT = 0x0c,
		_VOLTAGE_HIGH_LIMIT = 0x0d,
		_TORQUE_MAX_L = 0x0e,
		_TORQUE_MAX_H = 0x0f,
		_STATUS_RETURN_LVL = 0x10,
		_ALARM_LED = 0x11,
		_ALARM_SHUTDOWN = 0x12,
		_TORQUE_ENABLE = 0x18,
		_LED = 0x19,
		_D_GAIN = 0x1a,
		_I_GAIN = 0x1b,
		_P_GAIN = 0x1c,
		_GOAL_POS_L = 0x1e,
		_GOAL_POS_H = 0x1f,
		_MOVE_SPEED_L = 0x20,
		_MOVE_SPEED_H = 0x21,
		_TORQUE_LIMIT_L = 0x22,
		_TORQUE_LIMIT_H = 0x23,
		_CUR_POSITION_L = 0x24,
		_CUR_POSITION_H = 0x25,
		_CUR_SPEED_L = 0x26,
		_CUR_SPEED_H = 0x27,
		_CUR_LOAD_L = 0x28,
		_CUR_LOAD_H = 0x29,
		_CUR_VOLTAGE = 0x2a,
		_CUR_TEMP = 0x2b,
		_REGISTERED = 0x2c,
		_MOVING = 0x2e,
		_LOCK_EEPROM = 0x2f,
		_PUNCH_L = 0x30,
		_PUNCH_H = 0x31,
		_ACCELERATION = 0x49
	};
};

#endif /* DYNAMIXEL_H_ */
