/* pi.c */
#include "pi.h"

void PI_Init(PI_Controller *pi, float kp, float ki, float output_limit, float dt)
{
    pi->kp            = kp;
    pi->ki            = ki;
    pi->integral      = 0.0f;
    pi->output_limit  = output_limit;
    pi->dt            = dt;
}

float PI_Update(PI_Controller *pi, float target, float measured)
{
    float error = target - measured;

    pi->integral += error * pi->dt;

    float output = (pi->kp * error) + (pi->ki * pi->integral);

    if      (output >  pi->output_limit) output =  pi->output_limit;
    else if (output < -pi->output_limit) output = -pi->output_limit;

    return output;
}

void PI_Reset(PI_Controller *pi)
{
    pi->integral = 0.0f;
}

PI_Controller speed_pi;
volatile float target_rpm = 0.0f;

void TIM5_IRQHandler(void)
{
    if (TIM5->SR & TIM_SR_UIF)
    {
        TIM5->SR &= ~TIM_SR_UIF;

        float velocity   = Encoder_GetVelocityRPM();
        float duty_cycle = PI_Update(&speed_pi, target_rpm, velocity);

        /* write duty_cycle to PWM */
    }
}
