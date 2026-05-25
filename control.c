
/*
 * control.c
 *
 *  Created on: May 14, 2026
 *      Author: alexm
 */


// tim_isr.c  — control loop ISRs
#include "control.h"

volatile float target_velocity = 0;


volatile float target_velocity1 = 0;
volatile float current_position = 0;
volatile float target_position1 = 0;

State_t state = STATE_IDLE;

void setup_LOOPTIMERS(void) {

    // enable TIM5,6 peripheral clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

    TIM5->PSC = 0x0;
    TIM5->ARR = 0x2580; // initialize as 5kHz (48,000,000 / 5000 = 0x2580)

    TIM6->PSC = 9;   // /10 → 4.8 MHz
    TIM6->ARR = 0x2580;    // initialize as 500Hz (4,800,000 / 500 = 0x2580)

    TIM5->DIER |= 0x1; // enable interrupts for tim5
    TIM6->DIER |= 0x1; // enable interrupts for tim6

    TIM5->SR &= ~( TIM_SR_UIF);
    TIM6->SR &= ~( TIM_SR_UIF);

    // enable TIM2 IRQ in NVIC
    NVIC->ISER[1] |= (1 << (TIM5_IRQn & 0x1F));
    NVIC->ISER[1] |= (1 << (TIM6_DAC_IRQn & 0x1F));

    // global IRQ enable
    __enable_irq();

    // start TIM2 CR1
    TIM5->CR1 |= TIM_CR1_CEN;
    TIM6->CR1 |= TIM_CR1_CEN;

}

void PI_Init(MotorController_t *ctx, float kp, float ki, float dt,
		float lower_limit, float upper_limit) {
    ctx->kp      = kp;
    ctx->ki      = ki;
    ctx->integrator_accum = 0.0f;
    ctx->dt           = dt;  // you'll need to add dt to the struct
    ctx->output_limit_high = upper_limit;
    ctx->output_limit_low = lower_limit;
}

float PI_Update(MotorController_t *ctx, float target, float measured)
{
	 float raw_error = target - measured;
	 float error = (raw_error > -1.5f && raw_error < 1.5f) ? 0.0f : raw_error;
    //error1 = error;

    ctx->integrator_accum += error * ctx->dt;
    //intaccum1 =  ctx->integrator_accum;

    if (ctx->integrator_accum > 250) {
   	 ctx->integrator_accum = 250;
    } else if (ctx->integrator_accum < -250) {
   	 ctx->integrator_accum = -250;
    }

    float output = (ctx->kp * error) + (ctx->ki * ctx->integrator_accum);


    // clamp — you'll need output_limit in struct too
    if      (output >  ctx->output_limit_high) output =  ctx->output_limit_high;
    else if (output < ctx->output_limit_low) output = ctx->output_limit_low;


    return output;
}

void PI_Reset(MotorController_t *ctx)
{
    ctx->integrator_accum = 0.0f;
}

/*void TIM5_IRQHandler(void) // velocity control loop
{
	 GPIOC -> ODR ^= 0x20;
    if (TIM5->SR & TIM_SR_UIF)
    {
        TIM5->SR &= ~TIM_SR_UIF;

        float velocity   = Encoder_GetVelocityRPM();
        //velocity1 = velocity;
        //float target = ((float)rawVoltageData / 4095.0f) * 400.0f - 200.0f;
        //target1 = target;

        float pwm_duty = PI_Update(&ctx_vel, target_velocity, velocity);
        //pwm_duty1 = pwm_duty;

        update_Motor_Velocity( pwm_duty ); // update motor velocity
    }
}*/

/*void TIM6_DAC_IRQHandler(void) { // position control loop

	 GPIOC->ODR ^= 0x40;
    if (TIM6->SR & TIM_SR_UIF)
    {

   	 TIM6->SR &= ~TIM_SR_UIF;

		 float position = Encoder_GetDegrees(); // measured position
		 current_position = position;

		 float target_position = ((float)rawVoltageData / 4095.0f) * 360.0f - 180.0f; // target position
		 target_position1 = target_position;

		 target_velocity = PI_Update(&ctx_pos, target_position, position);
		 target_velocity1 = target_velocity;


    }
}*/

void TIM5_IRQHandler(void) // velocity control loop
{
	 GPIOC -> ODR ^= 0x20;
    if ( TIM5->SR & TIM_SR_UIF ) {
       TIM5->SR &= ~TIM_SR_UIF;
    	 switch(state){  // behavior of loop dependent on current state
    	 	 case STATE_IDLE:
    	 	 case STATE_IDLE:
    	 	 case STATE_POSITION_CONTROL:

    	       float velocity   = Encoder_GetVelocityRPM();
    	       float pwm_duty = PI_Update(&ctx_vel, target_velocity, velocity);
    	       update_Motor_Velocity( pwm_duty ); // update motor velocity

    	 	 case STATE_VELOCITY_CONTROL:

    	       float velocity   = Encoder_GetVelocityRPM();
   			 float target_velocity = ((float)rawVoltageData / 4095.0f) * 200.0f - 100.0f;
    	       float pwm_duty = PI_Update(&ctx_vel, target_velocity, velocity);
    	       update_Motor_Velocity( pwm_duty ); // update motor velocity

    	 	 case STATE_OPEN_LOOP:
    	 	 case STATE_STEP_TEST:
    	 	 case STATE_HOLD:
    	 	 default:

    	 }

    }
}

void TIM6_DAC_IRQHandler(void) { // position control loop

	 GPIOC->ODR ^= 0x40;
    if ( TIM6->SR & TIM_SR_UIF ) {
		 TIM6->SR &= ~TIM_SR_UIF; // clear interrupt flag

   	 switch(state){  // behavior of loop dependent on current state
   	 	 case STATE_IDLE:
   	 	 case STATE_IDLE:
   	 	 case STATE_POSITION_CONTROL:

   			 float position = Encoder_GetDegrees(); // measured position
   			 current_position = position;

   			 float target_position = ((float)rawVoltageData / 4095.0f) * 360.0f - 180.0f; // target position
   			 target_position1 = target_position;

   			 target_velocity = PI_Update(&ctx_pos, target_position, position);
   			 target_velocity1 = target_velocity;

   	 	 case STATE_VELOCITY_CONTROL:

   	 		 // position loop not used for velocity control

   	 	 case STATE_OPEN_LOOP:


   	 	 case STATE_STEP_TEST:
   	 	 case STATE_HOLD:
   	 	 default:

   	 }






    }
}

/*-----------------------------------------------------------------------------
 * function : setup_PBSWEXTI( )
 * INs      : none
 * OUTs     : none
 * action   : enables GPIOA clock; configures PA3 as input with pull-down;
 *            routes EXTI3 to port A via SYSCFG; enables rising-edge trigger,
 *            disables falling-edge; unmasks EXTI3 interrupt; enables EXTI3
 *            in NVIC and globally enables IRQs
 * authors  : Alex Tauber
 * version  : 0.1
 * date     : 260424
 * usage    :
 *----------------------------------------------------------------------------*/
void setup_PBSWEXTI( void ) {

    // enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    // clear PA3 mode bits — input mode
    GPIOA->MODER &= ~(GPIO_MODER_MODE3);
    // clear PA3 pull bits then set pull-down
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD3);
    GPIOA->PUPDR |=  (GPIO_PUPDR_PUPD3_1);

    // route EXTI3 to port A via SYSCFG
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI3);
    SYSCFG->EXTICR[0] |=  (SYSCFG_EXTICR1_EXTI3_PA);

    // enable rising edge trigger
    EXTI->RTSR1 |=  (1 << 3);
    // disable falling edge trigger
    EXTI->FTSR1 &= ~(1 << 3);
    // unmask EXTI3 interrupt line
    EXTI->IMR1  |=  (1 << 3);

    // enable EXTI3 in NVIC and globally enable interrupts
    NVIC_EnableIRQ(EXTI3_IRQn);

    __enable_irq();
}

/*-----------------------------------------------------------------------------
 * function : EXTI3_IRQHandler( )
 * INs      : none
 * OUTs     : none
 * action   : clears EXTI3 pending flag; state transitions occur in this irq handler
 *
 * authors  : Alex Tauber
 * version  : 0.1
 * date     : 260424
 * usage    : ISR — called automatically by NVIC on PA3 rising edge
 *----------------------------------------------------------------------------*/
void EXTI3_IRQHandler( void ) {

    if (EXTI->PR1 & (1 << 3)) {
        // clear pending flag
        EXTI->PR1 |= (1 << 3);

        /* need to check other condition before changing state
         ( assuming mode selection happens on the TFT display with user input
          - likely need to check current position on screen
         */

    }

}


