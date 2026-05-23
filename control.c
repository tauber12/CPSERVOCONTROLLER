
/*
 * control.c
 *
 *  Created on: May 14, 2026
 *      Author: alexm
 */


// tim_isr.c  — control loop ISRs
#include "control.h"
#include "stm32l4a6xx.h"


void GPIOC_C1_C2_Output_Init(void)
{
    // 1. Enable GPIOC peripheral clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // 2. Set PC1 and PC2 mode to output: MODER bits = 01
    GPIOC->MODER &= ~((3U << (1 * 2)) | (3U << (2 * 2)));
    GPIOC->MODER |=  ((1U << (1 * 2)) | (1U << (2 * 2)));

    // 3. Set output type to push-pull: OTYPER bits = 0
    GPIOC->OTYPER &= ~((1U << 1) | (1U << 2));

    // 4. Set speed to low: OSPEEDR bits = 00
    GPIOC->OSPEEDR &= ~((3U << (1 * 2)) | (3U << (2 * 2)));

    // 5. Disable pull-up / pull-down: PUPDR bits = 00
    GPIOC->PUPDR &= ~((3U << (1 * 2)) | (3U << (2 * 2)));

    // 6. Optional: set initial output low
    GPIOC->BSRR = (1U << (1 + 16)) | (1U << (2 + 16));
}

void setup_LOOPTIMERS(void) {

    // enable TIM5,6 peripheral clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    // enable CCR1 compare interrupt and update interrupt
    //TIM2->DIER |= (TIM_DIER_CC1IE | TIM_DIER_UIE);
    // set ARR for tim 5,6 for 5khz and
    TIM5->ARR = 0xBB80;
    TIM6->ARR = 0xBB80;
    TIM5->PSC = 0x0;
    TIM6->PSC = 0x0;
    TIM5->DIER |= 0x1;
    TIM6->DIER |= 0x1;
    TIM5->SR &= ~( TIM_SR_UIF);
    TIM6->SR &= ~( TIM_SR_UIF);
    // enable TIM2 IRQ in NVIC
    NVIC->ISER[1] |= (1 << (TIM5_IRQn & 0x1F));
    //NVIC->ISER[1] |= (1 << (TIM6_DAC_IRQn & 0x1F));
    // global IRQ enable
    __enable_irq();
    // start TIM2 CR1
    TIM5->CR1 |= TIM_CR1_CEN;
    TIM6->CR1 |= TIM_CR1_CEN;

}

void PI_Init(MotorController_t *ctx, float kp, float ki, float dt)
{
    ctx->kp      = kp;
    ctx->ki      = ki;
    ctx->integrator_vel = 0.0f;
    ctx->dt           = dt;  // you'll need to add dt to the struct
}

float PI_Update(MotorController_t *ctx, float target, float measured)
{
    float error = target - measured;

    ctx->integrator_vel += error * ctx->dt;

    float output = (ctx->kp * error) + (ctx->ki * ctx->integrator);


    // clamp — you'll need output_limit in struct too
    if      (output >  ctx->output_limit_high) output =  ctx->output_limit_high;
    else if (output < ctx->output_limit_low) output = ctx.output_limit_low;


    return output;
}

void PI_Reset(MotorController_t *ctx)
{
    ctx->integrator_vel = 0.0f;
}

void TIM5_IRQHandler(void) // velocity control loop
{
    if (TIM5->SR & TIM_SR_UIF)
    {
        TIM5->SR &= ~TIM_SR_UIF;

        float velocity   = Encoder_GetVelocityRPM();

        target = (uint8_t) ( ( (float) rawVoltageData / 4095 * 400.0 ) - 200.0 );

        float pwm_out = PI_Update(&ctx, target, velocity);

        set_Motor_Velocity( velocity, pwm_out ); // update motor velocity
    }
}

void TIM6_DAC_IRQHandler(void) { // position control loop
    TIM6->SR &= ~TIM_SR_UIF;
    GPIOC->ODR ^= 0x4;
    //float pos_error = ctx.position_setpoint - counts_to_degrees(ctx.encoder_count);
    //ctx.speed_setpoint = CLAMP(pid_update_pos(pos_error), -MAX_RPM, MAX_RPM);
}
