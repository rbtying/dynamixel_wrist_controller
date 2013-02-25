/*
 * DynamixelManager.cpp
 *
 *  Created on: Feb 23, 2013
 *      Author: rbtying
 */

#include "DynamixelManager.h"

DynamixelManager::DynamixelManager(HardwareSerial *serial) {
	m_serial = serial;
	m_mutex = false;
}

void DynamixelManager::begin(uint32_t baud_rate) {
	m_serial->begin(baud_rate);
	m_serial->setTimeout(SERIAL_TIMEOUT);
}

void DynamixelManager::acquireMutex() {
	m_mutex = true;
}

void DynamixelManager::releaseMutex() {
	m_mutex = false;
}

void DynamixelManager::send(uint8_t *data, size_t len) {
	while (m_mutex)
		;
	acquireMutex();
	m_serial->write(data, len);
	m_serial->flush();
	releaseMutex();
}

uint8_t DynamixelManager::read(uint8_t *data, size_t len) {
	while (m_mutex)
		;
	acquireMutex();
	uint8_t ret = m_serial->readBytes(reinterpret_cast<char*>(data), len);
	releaseMutex();
	return ret;
}

DynamixelManager::~DynamixelManager() {
	m_serial->end();
}
