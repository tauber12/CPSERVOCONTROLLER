/*
 * motor.c
 *
 *  Created on: May 5, 2026
 *      Author: alexm
 */

#include "motor.h"

// Enable clock, then configure IN1/IN2 as output

//change clock to 48MHz from default 4MHz
void clk_CONFIG_48MHz( void ){
	// flash latency first??
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;

	// configure MSI frequency range - 0b0110 for max
	RCC->CR &= ~RCC_CR_MSIRANGE;
	RCC->CR |= RCC_CR_MSIRANGE_11 | RCC_CR_MSIRGSEL;
	while (!(RCC->CR & RCC_CR_MSIRDY));

	SystemCoreClock = 48000000;
}

/*-----------------------------------------------------------------------------
 * function : setup_TIM1( )
 * INs      :
 * OUTs     :
 * action   :
 * version  : 0.1
 * date     : 260428
 * usage    : called once in main.c during initialization; iDutyCycle sets
 *            initial compare threshold in timer ticks
 *----------------------------------------------------------------------------*/

void setup_TIM1_A8( void ) {

	//initialize PA8 as AF to be driven my TIM1 CCR1
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER  &= ~(3U << 16);
	GPIOA->MODER  |=  (2U << 16);   // AF mode for PA8
   GPIOA->AFR[1] &= ~(0xF << 0);
	GPIOA->AFR[1] |=  (1U << 0);    // AF1 = TIM1 on PA8

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	// set frequency
	TIM1->PSC = 0;
	TIM1->ARR = 2399;  // 48 MHz / 2400 = 20 kHz
	TIM1->CCR1 = 1200;
	// configure output compare — PWM mode 1
	TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM1->CCMR1 |=  (6U << TIM_CCMR1_OC1M_Pos);
	TIM1->CCMR1 |=  TIM_CCMR1_OC1PE;   // preload enable
	// enable channel output
	TIM1->CCER |= TIM_CCER_CC1E;
	// MOE — mandatory for TIM1, unique to advanced timers? apparently?
	TIM1->BDTR |= TIM_BDTR_MOE;
	// start counter
	TIM1->CR1  |= TIM_CR1_CEN;

}

//set duty cycle if valid
void set_DUTY( uint8_t iDutyCycle ) {

    if (iDutyCycle <= 100) {
        uint32_t arr = TIM1->ARR;
        TIM1->CCR1 = (uint32_t)((arr + 1) * iDutyCycle / 100.0);
    }
}

void set_Motor_Direction( float velocity ){

	if( velocity > 0){
		//pc3 pc4
		GPIOA -> BSRR = (DIRECTION_PIN_1 | (DIRECTION_PIN_2 << 16));

	} else {
		GPIOA -> BSRR = (DIRECTION_PIN_2 | (DIRECTION_PIN_1 << 16));

	}

}

void update_Motor_Velocity( float desired_PWM , float velocity ){

	set_Motor_Direction( velocity );
	set_DUTY( (uint8_t) desired_PWM );

}

