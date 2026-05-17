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
