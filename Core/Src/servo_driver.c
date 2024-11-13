/*
 * servo_driver.cpp
 *
 *  Created on: Nov 2, 2024
 *      Author: brazenparadox
 */

#include "servo_driver.h"
int init_servo(Servo* servo){
	int status = 0; // success

	if(servo->servo_pwm_timer == 0){
		return status = 1; // servo pwm timer uninitialized
	}
	servo->current_position = 0;

	return status;
}

int rotate_servo(Servo* servo, uint8_t deg){
	int status = 0;
	servo->current_position = deg;

	// logic to rotate the servo
	// calculate the CCR1
	float pulse_width = 0.5 + ((float)deg/180) * 2;
	uint32_t CCR1 = pulse_width * (servo->servo_pwm_timer->Instance->ARR + 1)/20;
	// set the duty cycle
	servo->servo_pwm_timer->Instance->CCR1 = CCR1;
	return status;
}
