/*
 * control.h
 *
 * PI control-loop configuration and public API.
 */

#ifndef SRC_CONTROL_H_
#define SRC_CONTROL_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "encoder.h"

/* -------------------------------------------------------------------------- */
/* Controller configuration                                                    */
/* -------------------------------------------------------------------------- */

#define CONTROL_PRESET_COUNT              2U

/* Keep the initial velocity-loop rate tied to encoder.c. */
#define CONTROL_DEFAULT_VEL_HZ            ENCODER_DEFAULT_SAMPLE_HZ
#define CONTROL_DEFAULT_POS_HZ            500.0f
#define CONTROL_MIN_POS_HZ                50.0f
#define CONTROL_MAX_POS_HZ                2000.0f
#define CONTROL_MIN_VEL_HZ                100.0f
#define CONTROL_MAX_VEL_HZ                10000.0f
#define CONTROL_MIN_RATE_RATIO            2.0f

#define CONTROL_DEFAULT_POS_KP            10.0f
#define CONTROL_DEFAULT_POS_KI            1.0f
#define CONTROL_DEFAULT_POS_RANGE_RPM     500.0f
#define CONTROL_DEFAULT_POS_ERR_DEG       1.5f

#define CONTROL_DEFAULT_VEL_KP            20.0f
#define CONTROL_DEFAULT_VEL_KI            0.5f
#define CONTROL_DEFAULT_VEL_RANGE_PWM     100.0f
#define CONTROL_DEFAULT_VEL_ERR_RPM       0.5f

#define CONTROL_MIN_OUTPUT_RANGE          0.0f
#define CONTROL_MAX_POS_RANGE_RPM         1000.0f
#define CONTROL_MAX_VEL_RANGE_PWM         100.0f

#define CONTROL_INPUT_DEFAULT_FREQ_HZ     0.25f
#define CONTROL_INPUT_MIN_FREQ_HZ         0.01f
#define CONTROL_INPUT_MAX_FREQ_HZ         5.0f
#define CONTROL_INPUT_DEFAULT_AMPLITUDE   80.0f
#define CONTROL_INPUT_MIN_AMPLITUDE       0.0f

#ifndef CONTROL_PI_F
#define CONTROL_PI_F                      3.14159265358979323846f
#endif

typedef enum {
    STATE_DISABLED = 0,
    STATE_POSITION_CONTROL,
    STATE_VELOCITY_CONTROL,
    STATE_FAULT
} Controller_State;

typedef enum {
    CONTROL_INPUT_USER = 0,
    CONTROL_INPUT_SINE,
    CONTROL_INPUT_COSINE,
    CONTROL_INPUT_RAMP,
    CONTROL_INPUT_SQUARE,
    CONTROL_INPUT_STEP,
    CONTROL_INPUT_COUNT
} ControlInputSource_t;

typedef struct {
    volatile float dt;              /* sample period, seconds */
    volatile float output_range;    /* symmetric output clamp: +/-range */
    volatile float error_margin;

    volatile float target;
    volatile float kp;
    volatile float ki;
    volatile float integrator_accum;

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

extern volatile float pos_controller_output_velocity;
extern volatile uint8_t tracking_toggle_request;

extern volatile Controller_State current_state;
extern volatile Controller_State desired_state;
extern volatile ControlInputSource_t control_input_source;

extern volatile float control_position_loop_hz;
extern volatile float control_velocity_loop_hz;
extern volatile float control_input_wave_freq_hz;
extern volatile float control_input_wave_amplitude;

extern volatile float current_position;
extern volatile float target_position1;
extern volatile float target_velocity1;
extern volatile float measured_velocity1;
extern volatile float encoder_measured_position1;
extern volatile float pwm_duty1;
extern volatile float integrator_accum1;

void setup_LOOPTIMERS(void);
void PI_Init(MotorController_t *ctx, float kp, float ki, float dt,
             float output_range, float error_margin);
float PI_Update(MotorController_t *ctx, float target, float measured);
void  PI_Reset(MotorController_t *ctx);

void Control_SetDesiredState(Controller_State requested_state);
void Control_RequestVelocityMode(void);
void Control_RequestPositionMode(void);
uint8_t Control_IsTrackingEnabled(void);
void Control_EnterTracking(void);
void Control_ExitTracking(void);
void Control_ToggleTracking(void);

const char *Control_GetInputSourceName(ControlInputSource_t source);
void Control_SetInputSource(ControlInputSource_t source);
void Control_NextInputSource(int32_t direction);
float Control_SetInputWaveFrequencyHz(float requested_hz);
float Control_SetInputWaveAmplitude(float requested_amplitude);
float Control_GetInputWaveFrequencyHz(void);
float Control_GetInputWaveAmplitude(void);
float Control_GetInputTargetPositionDeg(void);
float Control_GetInputTargetVelocityRPM(void);

float Control_SetPositionLoopHz(float requested_hz);
float Control_SetVelocityLoopHz(float requested_hz);
float Control_GetPositionLoopHz(void);
float Control_GetVelocityLoopHz(void);
float Control_SetPositionOutputRange(float requested_range);
float Control_SetVelocityOutputRange(float requested_range);

void Control_InitPresetsFromCurrent(void);
void Control_SavePreset(uint8_t slot);
void Control_LoadPreset(uint8_t slot);
ControlPreset_t Control_GetPreset(uint8_t slot);
void Control_SetPresetValue(uint8_t slot, uint8_t field, float value);

#endif /* SRC_CONTROL_H_ */
