/*
 * servo_controller_paradox.h
 *
 *  Created on: Nov 2, 2024
 *      Author: brazenparadox
 */

#ifndef INC_SERVO_DRIVER_H_
#define INC_SERVO_DRIVER_H_

#include<inttypes.h>
#include "stm32h7xx_hal.h"

typedef struct{
	uint8_t servo_pin;
	uint8_t current_position;
	TIM_HandleTypeDef *servo_pwm_timer;
} Servo;

int init_servo(Servo*);
int rotate_servo(Servo*, uint8_t);

#endif /* INC_SERVO_DRIVER_H_ */
