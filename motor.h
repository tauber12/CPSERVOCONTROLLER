/*
 * motor.h
 *
 *  Created on: May 5, 2026
 *      Author: alexm
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32l4xx_hal.h"

#define MOTOR_PORT GPIOC
#define MOTOR__PWM_PIN 0
#define DIRECTION_PIN_1 GPIO_PIN_3
#define DIRECTION_PIN_2 GPIO_PIN_4

typedef int bool;

void GPIOC_C3_C4_Output_Init(void);
void GPIOC_C5_C6_Output_Init(void);
void clk_CONFIG_48MHz( void );
void setup_TIM1_A8( void );
void set_DUTY(uint8_t iDutyCycle);
void set_Motor_Direction( bool direction );
void update_Motor_Velocity( float desired_PWM );



#endif /* INC_MOTOR_H_ */
