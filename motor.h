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
#define DIRECTION_PIN_1 3
#define DIRECTION_PIN_2 4

#include "stm32l4xx_hal.h"

void clk_CONFIG_48MHz( void );
void setup_TIM1_A8( void );
void set_DUTY(uint8_t iDutyCycle);
void set_Motor_Direction( float velocity );
void update_Motor_Velocity( float desired_PWM , float velocity );



#endif /* INC_MOTOR_H_ */
