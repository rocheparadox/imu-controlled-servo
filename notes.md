# Notes

## Servo Motor Control

Servo motor is controlled by a PWM signal. The period of the PWM signal should be 20 ms. And the duty cycle should be anywhere between 5% and 10% and hence the signal should be high for anytime between 1 ms and 2 ms respectively. Technically, the servo motor should be in the position of 0 deg for 1 ms and 180 deg for 2 ms.

According to this [datasheet](https://www.friendlywire.com/projects/ne555-servo-safe/SG90-datasheet.pdf), the pulse and position would be


|Pulse (ms)|Position|
|---------|--------|
| 1       | -90     |
| 1.5     |  0      |
| 2       |  90     |



### How to use a timer for this?

Consider the following specifications. 

The timer clock is **64 MHz**.
Prescaler is 999
Auto Reload Register is 1279

The timer would set an overflow flag for ,

$64000000/(1000 + 1280) = 64000/1280 = 50$

 times in a second. So the frequency of the timer would be 50 Hz. Hence, the timer overflow flag will be set for every $1/50 = .02$ seconds = $20$ ms.

 Now this has to be turned into a PWM signal. A functionality of the timer called **Capture/Compare mode** has to be utilized here. What does this do?

 **Capture Mode**: Retrieves a timer value based on an external event. 
 **Compare Mode**: Constantly compares the timer value against a predefined value and triggers an event when the values match. 
 
 The following is an excerpt from the datasheet of STM32H723

 >Pulse width modulation mode permits to generate a signal with a frequency determined by the value of the TIMx_ARR register and a duty cycle determined by the value of the TIMx_CCRx register.

 So, from the given information, 1279 is required for a PWM of time period 20 ms. Then, to achieve a duty cycle of x%, 

 $CCR=ARR*x/100$
 
 Therefore, if the desired position of the servo motor is d, then

 $CCR = [(1 + (d/180)) * ((ARR + 1)/20)] $

 ($ARR+1)/20$ gives the ticks required for 1 ms. 

 (d/180)*((ARR+1)/20) gives the duration for which the PWM should stay high during a cycle. 1 is added to the equation since 1 ms is required for the servo motor to stay high. 

 ### Servo I have 

 The servo I have, has a wide range of pulse width operating range. 0.5 to 2.5 ms.


 
