/*
 * motor.h
 *
 *  Created on: May 5, 2026
 *      Author: alexm
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define MOTOR_PORT GPIOA
#define MOTOR__PWM_PIN 0
#define DIRECTION_PIN_1 1
#define DIRECTION_PIN_2 2

#include "stm32l4xx_hal.h"

void clk_CONFIG_48MHz( void );
void setup_TIM1_A8( void );
void set_DUTY(uint8_t iDutyCycle);



#endif /* INC_MOTOR_H_ */
