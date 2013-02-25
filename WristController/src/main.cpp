/*
 * main.cpp
 *
 *  Created on: Feb 23, 2013
 *      Author: rbtying
 */

#include <WProgram.h>
#include <usb/usb_api.h>
#include "config/pin_config.h"
#include "config/dynamixel_config.h"
#include "dynamixel/Dynamixel.h"
#include "dynamixel/DynamixelManager.h"

HardwareSerial uart = HardwareSerial();
DynamixelManager dyn(&uart);

Dynamixel wrist_left(&dyn, &config::dynamixel::wrist_left);
Dynamixel wrist_right(&dyn, &config::dynamixel::wrist_right);

void setup() {
	pinMode(LED_PIN, OUTPUT);

	Serial.begin(9600);

	dyn.begin(config::dynamixel::baud_rate);
	wrist_left.init();
	wrist_right.init();
}

void cleanup() {
}

void loop() {
	static long p_time = 0;

	while (Serial.available()) {
		Serial.print((char) Serial.read());
		Serial.flush();
	}

	long c_time = millis();
	if (c_time - p_time > 500) {
		p_time = c_time;
		digitalWrite(LED_PIN, !digitalRead(LED_PIN));
		Serial.print("Dynamixel 1: ");
		Serial.print(wrist_left.get_voltage());
		Serial.print(" Dynamixel 2: ");
		Serial.print(wrist_right.get_voltage());
		Serial.println();
		Serial.flush();
	}
}

int main(int argc, char **argv) {
	_init_Teensyduino_internal_();
	setup();
	for (;;) {
		loop();
	}
	cleanup();
}
