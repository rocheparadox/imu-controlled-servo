/*
 * Author : Roche Christopher
 * email  : rochextopher@gmail.com
 *
 */

/*
 *
	MIT License

	Copyright (c) 2024 Roche Christopher

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
 *
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

	/*
	 * The Servo I have, has an operating range of 0 to 180 degrees and it responds to 0.5 ms to 2.5 ms
	 * So, the operating pulse width span is 2.5 - 0.5 = 2 ms.
	 * So, the deg to pulse width calculation would be
	 *
	 * pulse width = 0.5 + (2*deg/180)
	 *
	 * */

	int status = 0;
	servo->current_position = deg;

	// logic to rotate the servo
	// calculate the CCR1
	float ticks4onems = (servo->servo_pwm_timer->Instance->ARR + 1)/20;
	float pulse_width =  (ticks4onems/2)  + (ticks4onems * 2 * ((float)deg/180));
	uint32_t CCR1 = pulse_width;
	// set the duty cycle
	servo->servo_pwm_timer->Instance->CCR1 = CCR1;
	return status;
}
