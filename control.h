/*
 * control.h
 *
 *  Created on: May 14, 2026
 *      Author: alexm
 */

#ifndef SRC_CONTROL_H_
#define SRC_CONTROL_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define OUTPUT_LIMIT 100

#define CONTROL_PRESET_COUNT        2U
#define CONTROL_FREQ_RESPONSE_POINTS 48U

typedef enum {
    STATE_DISABLED = 0,
    STATE_POSITION_CONTROL,
    STATE_VELOCITY_CONTROL,
    STATE_FREQUENCY_RESPONSE,
    STATE_FAULT
} Controller_State;

typedef enum {
    CONTROL_INPUT_ENCODER = 0,
    CONTROL_INPUT_SINE,
    CONTROL_INPUT_COSINE,
    CONTROL_INPUT_RAMP,
    CONTROL_INPUT_SQUARE,
    CONTROL_INPUT_STEP,
    CONTROL_INPUT_COUNT
} ControlInputSource_t;

typedef struct {
    volatile float dt;                 // sample period
    volatile float output_limit_high;  // clamp value
    volatile float output_limit_low;   // clamp value

    // Setpoints
    volatile float target;

    // PI state
    volatile float kp;
    volatile float ki;
    volatile float integrator_accum;

    // faults
    volatile uint8_t fault_flags;
} MotorController_t;

typedef struct {
    float pos_kp;
    float pos_ki;
    float vel_kp;
    float vel_ki;
} ControlPreset_t;

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
extern volatile ControlInputSource_t control_input_source;

// User-editable loop rates. Use Control_Set*LoopHz() to enforce constraints.
extern volatile float control_position_loop_hz;
extern volatile float control_velocity_loop_hz;

// For live data tracking / display plotting.
extern volatile float current_position;
extern volatile float target_position1;
extern volatile float target_velocity1;
extern volatile float measured_velocity1;
extern volatile float encoder_measured_position1;
extern volatile float pwm_duty1;

// Frequency response data for Bode magnitude plot.
extern volatile uint8_t freq_response_active;
extern volatile uint8_t freq_response_done;
extern volatile uint16_t freq_response_point_count;
extern volatile uint16_t freq_response_current_index;
extern volatile float freq_response_current_hz;
extern volatile float freq_response_freq_hz[CONTROL_FREQ_RESPONSE_POINTS];
extern volatile float freq_response_mag_db[CONTROL_FREQ_RESPONSE_POINTS];
extern volatile float freq_response_output_amp_deg[CONTROL_FREQ_RESPONSE_POINTS];

void setup_LOOPTIMERS(void);
void PI_Init(MotorController_t *ctx, float kp, float ki, float dt,
        float lower_limit, float upper_limit);
float PI_Update(MotorController_t *ctx, float target, float measured);
void  PI_Reset (MotorController_t *ctx);

void Control_SetDesiredState(Controller_State requested_state);
void Control_RequestVelocityMode(void);
void Control_RequestPositionMode(void);
void Control_RequestFrequencyResponseMode(void);
uint8_t Control_IsTrackingEnabled(void);
void Control_EnterTracking(void);
void Control_ExitTracking(void);
void Control_ToggleTracking(void);

const char *Control_GetInputSourceName(ControlInputSource_t source);
void Control_SetInputSource(ControlInputSource_t source);
void Control_NextInputSource(int32_t direction);
float Control_GetInputTargetPositionDeg(void);
float Control_GetInputTargetVelocityRPM(void);

float Control_SetPositionLoopHz(float requested_hz);
float Control_SetVelocityLoopHz(float requested_hz);
float Control_GetPositionLoopHz(void);
float Control_GetVelocityLoopHz(void);

void Control_InitPresetsFromCurrent(void);
void Control_SavePreset(uint8_t slot);
void Control_LoadPreset(uint8_t slot);
ControlPreset_t Control_GetPreset(uint8_t slot);
void Control_SetPresetValue(uint8_t slot, uint8_t field, float value);

void Control_FrequencyResponseReset(void);
uint8_t Control_FrequencyResponseIsDone(void);

#endif /* SRC_CONTROL_H_ */
