/*
 * motor.c
 *
 *  Created on: May 5, 2026
 *      Author: alexm
 */

#include "motor.h"

// Enable clock, then configure IN1/IN2 as output
void GPIOC_C3_C4_Output_Init(void)
{
    // 1. Enable GPIOC peripheral clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    // 2. Set PC3 and PC4 mode to output: MODER bits = 01
    MOTOR_PORT->MODER &= ~((3U << (3 * 2)) | (3U << (4 * 2)));
    MOTOR_PORT->MODER |=  ((1U << (3 * 2)) | (1U << (4 * 2)));
    // 3. Set output type to push-pull: OTYPER bits = 0
    MOTOR_PORT->OTYPER &= ~((1U << 3) | (1U << 4));
    // 4. Set speed to low: OSPEEDR bits = 00
    MOTOR_PORT->OSPEEDR &= ~((3U << (3 * 2)) | (3U << (4 * 2)));
    // 5. Disable pull-up / pull-down: PUPDR bits = 00
    MOTOR_PORT->PUPDR &= ~((3U << (3 * 2)) | (3U << (4 * 2)));
    // 6. Set initial output low
    MOTOR_PORT->BSRR = (1U << (3 + 16)) | (1U << (4 + 16));
}

void GPIOC_C5_C6_Output_Init(void)
{
    // 1. Enable GPIOC peripheral clock (may already be enabled)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // 2. Set PC5 and PC6 to output mode (MODER = 01)
    GPIOC->MODER &= ~((3U << (5 * 2)) | (3U << (6 * 2)));
    GPIOC->MODER |=  ((1U << (5 * 2)) | (1U << (6 * 2)));

    // 3. Push-pull output type
    GPIOC->OTYPER &= ~((1U << 5) | (1U << 6));

    // 4. Low speed
    GPIOC->OSPEEDR &= ~((3U << (5 * 2)) | (3U << (6 * 2)));

    // 5. No pull-up / pull-down
    GPIOC->PUPDR &= ~((3U << (5 * 2)) | (3U << (6 * 2)));

    // 6. Initial state low
    GPIOC->BSRR = (1U << (5 + 16)) | (1U << (6 + 16));
}

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

void set_Motor_Direction( bool direction ){
	if( direction ){
		MOTOR_PORT -> BSRR = (DIRECTION_PIN_2 | (DIRECTION_PIN_1 << 16));
		//pc3 pc4

	} else {
		MOTOR_PORT -> BSRR = (DIRECTION_PIN_1 | (DIRECTION_PIN_2 << 16));

	}
}

void update_Motor_Velocity(float desired_PWM)
{
    set_Motor_Direction(desired_PWM > 0 ? 1 : 0);
    set_DUTY((uint8_t)(desired_PWM < 0 ? -desired_PWM : desired_PWM));
}
