/*
 * dynamxiel_config.h
 *
 *  Created on: Feb 25, 2013
 *      Author: rbtying
 */

#ifndef DYNAMXIEL_CONFIG_H_
#define DYNAMIXEL_CONFIG_H_

#include "../dynamixel/Dynamixel.h"

namespace config {
namespace dynamixel {

const static uint32_t baud_rate = 1000000;

const static struct dynamixel_info_t wrist_left = { 2, Dynamixel::WHEEL_MODE };
const static struct dynamixel_info_t wrist_right = { 3, Dynamixel::WHEEL_MODE
		| Dynamixel::FLIP_DIRECTION };
}
}

#endif /* DYNAMXIEL_CONFIG_H_ */
