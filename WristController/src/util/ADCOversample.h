/*
 * ADCOversample.h
 *
 *  Created on: Feb 25, 2013
 *      Author: rbtying
 */

#ifndef ADCOVERSAMPLE_H_
#define ADCOVERSAMPLE_H_

#include <Arduino.h>

namespace util {

/*!
 * Uses oversampling to increase the resolution of the ADC
 *
 * @param pin the pin to read
 * @param resolution the number of bits of resolution to achieve (approx)
 * @return an ADC reading from 0 to 2^resolution
 */
uint16_t read_adc_oversampled(uint8_t pin, uint8_t resolution) {
	resolution = constrain(resolution, 10, 16) - 10;
	uint16_t samples = 1 << (resolution << 1);

	uint32_t sum = 0;
	for (uint8_t i = 0; i < samples; i++) {
		sum += analogRead(pin);
	}
	return sum >> resolution;
}

/*!
 * Uses oversampling to increase the resolution of the ADC
 *
 * @param pin the pin to read
 * @param resolution the number of bits of resolution to achieve (approx)
 * @return an ADC reading from 0 to 5.0 volts
 */
float read_adc_oversampled_f(uint8_t pin, uint8_t resolution) {
	uint16_t data = read_adc_oversampled(pin, resolution);
	return data * 5.0 / (1 << resolution);
}
}

#endif /* ADCOVERSAMPLE_H_ */
