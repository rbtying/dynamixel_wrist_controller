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

/*!
 * A struct to hold dynamixel config information
 */
struct dynamixel_info_t {
	uint8_t id;
	uint8_t flags;
};

/*!
 * A class to control Dynamixel servo motors
 */
class Dynamixel {

public:
	static const uint8_t FLIP_DIRECTION = (1 << 0);
	static const uint8_t WHEEL_MODE = (1 << 1);

	static const uint16_t MAX_POSITION = 0xfff;
	static const int16_t MAX_SPEED = 1023;

	enum baud_rate_t {
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

	enum error_t {
		ERROR_INSTRUCTION = 1 << 6,
		ERROR_OVERLOAD = 1 << 5,
		ERROR_CHECKSUM = 1 << 4,
		ERROR_RANGE = 1 << 3,
		ERROR_HEAT = 1 << 2,
		ERROR_ANGLE_LIMIT = 1 << 1,
		ERROR_VOLTAGE = 1 << 0,
	};

	enum status_return_level_t {
		RETURN_NONE = 0, RETURN_READ_CMDS = 1, RETURN_ALL = 2,
	};

private:
	enum command_t {
		_CMD_WRITE = 0x03, _CMD_READ = 0x02,
	};

	enum address_t {
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

public:
	/*!
	 * Constructs a dynamixel
	 *
	 * @param dyn a dynamixel manager to avoid resource contention
	 * @param id the id of the dynamixel to control
	 * @param flags initial setting for the configuration flags
	 */
	Dynamixel(DynamixelManager *dyn, uint8_t id, uint8_t flags);

	/*!
	 * Constrcuts a dynamixel
	 *
	 * @param dyn a dynamixel manager to avoid resource contention
	 * @param config a dynamixel_info_it struct with the config info
	 */
	Dynamixel(DynamixelManager *dyn, const dynamixel_info_t *config);
	virtual ~Dynamixel();

	/*!
	 * Initializes the dynamixel to match the config state
	 */
	void init();

	/*!
	 * Writes a new id to the dynamixel. Will set the internal id of this dynamixel as well.
	 * @param id the id to set
	 */
	void write_id(uint8_t id);

	/*!
	 * Writes a new baud rate to the dynamixel. Will probably cause problems,
	 * since the dynamixel manager will not be reconfigured.
	 * @param baud the baud rate
	 */
	void set_baud_rate(baud_rate_t baud);

	/*!
	 * Sets the delay between a command and its response
	 *
	 * @param delay the delay in microseconds
	 */
	void set_return_delay(uint16_t delay);

	/*!
	 * Sets the angle limits for the servo.
	 *
	 * @param cw the cw angle limit
	 * @param ccw the ccw angle limit
	 */
	void set_angle_limits(uint16_t cw, uint16_t ccw);

	/*!
	 * Initializes continuous turn (wheel) mode
	 */
	void init_cont_turn();

	/*!
	 * Sets the max temperature
	 *
	 * @param temp the temp in celsius
	 */
	void set_max_temp(uint8_t temp);

	/*!
	 * Sets the voltage limits
	 *
	 * @param min the minimum voltage (volts)
	 * @param max the maximum voltage (volts)
	 */
	void set_voltage_limits(float min, float max);

	/*!
	 * Sets the maximum torque
	 *
	 * @param tmax the max torque (0-1023)
	 */
	void set_max_torque(uint16_t tmax);

	/*!
	 * Sets the status return level
	 *
	 * @param level the level to return
	 */
	void set_status_return_level(status_return_level_t level);

	/*!
	 * Sets the cases for the alarm led to light
	 *
	 * @param errors the error cases to light
	 */
	void set_alarm_led(uint8_t errors);

	/*!
	 * Sets the cases for the servo to shutdown
	 *
	 * @param errors the error cases to shutdown
	 */
	void set_alarm_shutdown(uint8_t errors);

	/*!
	 * Enables the motor
	 */
	void enable_torque();

	/*!
	 * Disables the motor
	 */
	void disable_torque();

	/*!
	 * Sets whether the LED is on or off
	 *
	 * @param on whether the LED is on
	 */
	void set_led(uint8_t on);

	/*!
	 * Sets the PID values for the motor
	 *
	 * @param p the proportional gain
	 * @param i the integral gain
	 * @param d the differential gain
	 */
	void set_pid(uint8_t p, uint8_t i, uint8_t d);

	/*!
	 * Sets the setpoint of the position control loop
	 *
	 * @param position the position (0 to 4096)
	 */
	void set_goal_position(uint16_t position);

	/*!
	 * Sets the desired speed of the motor
	 *
	 * @param rpm the speed in rpm
	 */
	void set_goal_speed(float rpm);

	/*!
	 * Sets the desired speed of the motor
	 *
	 * @param spd the speed
	 */
	void set_goal_speed(int16_t spd);

	/*!
	 * Sets the desired accleeration of the motor
	 * @param accel the acceleration in deg/s/s
	 */
	void set_acceleration(float accel);

	/*!
	 * Sets the desired acceleration of the motor
	 * @param accel the acceleration
	 */
	void set_acceleration(uint8_t accel);

	/*!
	 * Sets the limiting torque value as a percent of max torque
	 *
	 * @parma percent the percentage
	 */
	void set_torque_limit(float percent);

	/*!
	 * Gets the current position of the servo
	 * @return position (0 to 4096)
	 */
	uint16_t get_position();

	/*!
	 * Gets the current speed of the servo
	 * @return speed (-1024 to 1024)
	 */
	int16_t get_speed();

	/*!
	 * Gets the current load of the servo
	 * @return load (-1024 to 1024)
	 */
	int16_t get_load();

	/*!
	 * Gets the measured voltage of the servo
	 * @return voltage
	 */
	float get_voltage();

	/*!
	 * Gets the measured temperature of the servo
	 * @return temperature in deg celsius
	 */
	uint8_t get_temperature();

	/*!
	 * Checks if the servo is moving
	 * @return false if not moving
	 */
	uint8_t is_moving();

	/*!
	 * Locks the servo eeprom. Cannot be unset without power cycle.
	 */
	void lock_eeprom();

	/*!
	 * Reads the error packet from the last transaction
	 */
	uint8_t read_error();

private:
	DynamixelManager *m_dyn;
	uint8_t m_id;
	uint8_t m_flags;
	uint8_t m_error;

	/*!
	 * Conducts a read/write transaction
	 *
	 * @param cmd whether to read or write
	 * @param addr the address to read or write to
	 * @param data the data to read or write
	 * @param datalen the length of the data
	 * @return the number of bytes written or read
	 */
	uint8_t transaction(command_t cmd, address_t addr, uint8_t *data,
			uint8_t datalen);

	/*!
	 * Calculates the checksum
	 *
	 * @param data the data to checksum,
	 * @param datalen the length of hte data
	 * @return the checksum
	 */
	uint8_t checksum(uint8_t *data, uint8_t datalen);

	/*!
	 * Writes to an address
	 *
	 * @param addr the address to write to
	 * @param data the data to write
	 * @param datalen the length of the data
	 * @return the number of bytes written
	 */
	uint8_t write_address(address_t addr, uint8_t *data, uint8_t datalen);

	/*!
	 * Writes one byte to an address
	 *
	 * @param addr the address to write
	 * @param data the data to write
	 * @return the number of bytes written
	 */
	uint8_t write_address(address_t addr, uint8_t data);

	/*!
	 * Reads from an address
	 *
	 * @param addr the address to read
	 * @param len the number of bytes to read
	 * @return the number of bytes read
	 */
	uint8_t read_address(address_t addr, uint8_t len = 1);

};

#endif /* DYNAMIXEL_H_ */
