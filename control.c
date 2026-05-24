
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


void setup_LOOPTIMERS(void) {

    // enable TIM5,6 peripheral clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    // enable CCR1 compare interrupt and update interrupt
    //TIM2->DIER |= (TIM_DIER_CC1IE | TIM_DIER_UIE);

    TIM5->PSC = 0x0;
    TIM5->ARR = 0x2580; // 5khz

    TIM6->PSC = 9;   // /10 → 4.8 MHz
    TIM6->ARR = 17700;    // 4,800,000 / 48,000 = 100 Hz

    TIM5->DIER |= 0x1;
    TIM6->DIER |= 0x1;
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

void TIM5_IRQHandler(void) // velocity control loop
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
}

void TIM6_DAC_IRQHandler(void) { // position control loop

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
}


