/* pi.h */
#ifndef PI_H
#define PI_H

typedef struct {
    float kp;
    float ki;
    float integral;
    float output_limit;
    float dt;
} PI_Controller;

void  PI_Init  (PI_Controller *pi, float kp, float ki, float output_limit, float dt);
float PI_Update(PI_Controller *pi, float target, float measured);
void  PI_Reset (PI_Controller *pi);

#endif
