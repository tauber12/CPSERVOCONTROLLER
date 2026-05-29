/*
 * control.h
 *
 *  Created on: May 14, 2026
 *      Author: alexm
 */

#ifndef SRC_CONTROL_H_
#define SRC_CONTROL_H_
#define OUTPUT_LIMIT 100

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "encoder.h"
#include "motor.h"
#include "ADC.h"
#include "HMI.h"

typedef enum {
    STATE_DISABLED = 0,
    STATE_POSITION_CONTROL,
     STATE_VELOCITY_CONTROL,
     STATE_FAULT
}Controller_State;

typedef struct {

     volatile float    dt;            // sample period
     volatile float    output_limit_high;  // clamp value
     volatile float    output_limit_low;  // clamp value

    // Setpoints
     volatile float target;

    // PI state
    volatile float    kp, ki;
    volatile float    integrator_accum;

    // faults
    volatile uint8_t  fault_flags;


}MotorController_t;

extern MotorController_t ctx_pos;
extern MotorController_t ctx_vel;

// Required global variables.
extern volatile float pos_controller_output_velocity;
extern volatile uint8_t tracking_toggle_request;

/*
 * current_state is the active controller state right now.
 * desired_state is the control mode that will be entered when tracking starts.
 */
extern volatile Controller_State current_state;
extern volatile Controller_State desired_state;

// For living data tracking.
extern volatile float current_position;
extern volatile float target_position1;
extern volatile float target_velocity1;
extern volatile float measured_velocity1;
extern volatile float encoder_measured_position1;
extern volatile float pwm_duty1;

void setup_LOOPTIMERS(void);
void PI_Init(MotorController_t *ctx, float kp, float ki, float dt,
        float lower_limit, float upper_limit);
float PI_Update(MotorController_t *ctx, float target, float measured);
void  PI_Reset (MotorController_t *ctx);

void Control_SetDesiredState(Controller_State requested_state);
void Control_RequestVelocityMode(void);
void Control_RequestPositionMode(void);
uint8_t Control_IsTrackingEnabled(void);
void Control_EnterTracking(void);
void Control_ExitTracking(void);
void Control_ToggleTracking(void);


#endif /* SRC_CONTROL_H_ */
