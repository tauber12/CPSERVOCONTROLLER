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

extern volatile float target_rpm;

typedef enum {
    STATE_1,
    STATE_2,
    STATE_3,
    STATE_4_
} State_t;

typedef struct {

	 volatile float    dt;            // sample period
	 volatile float    output_limit;  // clamp value
    // Setpoints
    volatile float    position_setpoint;   // degrees
    volatile float    speed_setpoint;      // RPM

    // Measurements
    volatile int32_t  encoder_count;       // raw TIM2 counts
    volatile float    velocity_rpm;        // computed in velocity ISR

    // PID state — velocity loop
    volatile float    kp_vel, ki_vel, kd_vel;
    volatile float    integrator_vel;
    volatile float    prev_measurement_vel;

    // PID state — position loop
    volatile float    kp_pos, ki_pos, kd_pos;
    volatile float    integrator_pos;
    volatile float    prev_measurement_pos;

    // Knob input
    volatile float    knob_degrees;        // AS5600 reading

    // ADC
    volatile uint16_t adc_gain;            // PC0 DMA target
    volatile uint16_t adc_velocity;        // PC1 DMA target

    // System state
    State_t   state;
    volatile uint8_t  fault_flags;

}MotorController_t;

extern MotorController_t ctx;

void PI_Init(MotorController_t *ctx, float kp, float ki, float dt);
float PI_Update(MotorController_t *ctx, float target, float measured);
void  PI_Reset (MotorController_t *ctx);

#endif /* SRC_CONTROL_H_ */
