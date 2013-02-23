/*
 * main.cpp
 *
 *  Created on: Feb 23, 2013
 *      Author: rbtying
 */

#include <WProgram.h>
#include <usb/usb_api.h>

HardwareSerial uart = HardwareSerial();

void setup() {
	pinMode(6, OUTPUT);
	Serial.begin(9600);
	uart.begin(115200);
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
		digitalWrite(6, !digitalRead(6));
	}
}

int main(int argc, char **argv) {
	_init_Teensyduino_internal_();
	setup();
	for (;;) {
		loop();
	}
}
