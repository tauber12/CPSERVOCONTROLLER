
/*
 * control.c
 *
 *  Created on: May 14, 2026
 *      Author: alexm
 */


// tim_isr.c  — control loop ISRs
#include "control.h"

volatile float pos_controller_output_velocity = 0;
volatile uint8_t tracking_toggle_request = 0;

// Variables for readout, testing only
volatile float target_velocity1 = 0;
volatile float current_position = 0;
volatile float target_position1 = 0;

Controller_State state = STATE_POSITION_CONTROL;

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

void Control_EnterTracking(void)
{
    float measured_position = Encoder_GetDegrees();

    PI_Reset(&ctx_pos);
    PI_Reset(&ctx_vel);

    current_position = measured_position;

    pos_controller_output_velocity = 0.0f;
    target_velocity1 = 0.0f;

    /*
     * Optional debug variable.
     * This does not fully hold the target unless your TIM6 ISR uses it.
     */
    target_position1 = measured_position;

    update_Motor_Velocity(0.0f);

    state = STATE_POSITION_CONTROL;
}

void Control_ExitTracking(void)
{
    state = STATE_DISABLED;

    PI_Reset(&ctx_pos);
    PI_Reset(&ctx_vel);

    pos_controller_output_velocity = 0.0f;
    target_velocity1 = 0.0f;

    update_Motor_Velocity(0.0f);
}

void Control_ToggleTracking(void)
{
    if (state == STATE_DISABLED)
    {
        Control_EnterTracking();
    }
    else if (state == STATE_POSITION_CONTROL)
    {
        Control_ExitTracking();
    }
}

void TIM5_IRQHandler(void) // velocity control loop
{
	 GPIOC -> ODR ^= 0x20;
    if ( TIM5->SR & TIM_SR_UIF ) {
       TIM5->SR &= ~TIM_SR_UIF;

       // Check if controller enabled
       if (state == STATE_DISABLED || state == STATE_FAULT)
       {
           pos_controller_output_velocity = 0.0f;
           update_Motor_Velocity(0.0f);
           return;
       }

       // Control Loop
		 float measured_velocity   = Encoder_GetVelocityRPM();
		 float target_velocity;
			 if (state == STATE_POSITION_CONTROL) {
				target_velocity = pos_controller_output_velocity;
			 } else {
				target_velocity = ((float)rawVoltageData / 4095.0f) * 200.0f - 100.0f;
			 }
		 float pwm_duty = PI_Update(&ctx_vel, target_velocity, measured_velocity);
		 update_Motor_Velocity( pwm_duty ); // update motor velocity

		 // Start conversion for reading velocity setpoint
		 ADC_StartConversion();
    }
}

void TIM6_DAC_IRQHandler(void) { // position control loop

	 GPIOC->ODR ^= 0x40;
    if ( TIM6->SR & TIM_SR_UIF ) {
		 TIM6->SR &= ~TIM_SR_UIF; // clear interrupt flag
       // Handle Tracking Toggle request
       if (tracking_toggle_request)
       {
           tracking_toggle_request = 0;
           Control_ToggleTracking();
       }
       // Control loop
		 if (state == STATE_POSITION_CONTROL) {
   			 float measured_position = Encoder_GetDegrees();
   			 current_position = measured_position;

   			 float target_position = ((float)rawVoltageData / 4095.0f) * 360.0f - 180.0f; // target position
   			 target_position1 = target_position;

   			 pos_controller_output_velocity = PI_Update(&ctx_pos, target_position, measured_position);
   			 target_velocity1 = pos_controller_output_velocity;
		 }
   }
}

