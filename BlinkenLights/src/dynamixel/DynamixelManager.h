/*
 * DynamixelManager.h
 *
 *  Created on: Feb 23, 2013
 *      Author: rbtying
 */

#ifndef DYNAMIXELMANAGER_H_
#define DYNAMIXELMANAGER_H_

#include <Arduino.h>

class DynamixelManager {
public:
	DynamixelManager(HardwareSerial *serial);
	virtual ~DynamixelManager();

	void begin(uint32_t baud_rate);

	void acquireMutex();
	void releaseMutex();

	void send(uint8_t *data, size_t len);
	uint8_t read(uint8_t *data, size_t len);

private:
	HardwareSerial *m_serial;
	uint8_t m_mutex;

public:
	static const int SERIAL_TIMEOUT = 100;
};

#endif /* DYNAMIXELMANAGER_H_ */
