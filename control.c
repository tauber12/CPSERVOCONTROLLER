/*
 * control.c
 *
 *  Created on: May 14, 2026
 *      Author: alexm
 */

// tim_isr.c -- control loop ISRs
#include "control.h"

#include <math.h>
#include <stddef.h>

#include "ADC.h"
#include "HMI.h"
#include "encoder.h"
#include "motor.h"

#ifndef CONTROL_PI_F
#define CONTROL_PI_F 3.14159265358979323846f
#endif

#define CONTROL_DEFAULT_VEL_HZ             5000.0f
#define CONTROL_DEFAULT_POS_HZ              500.0f
#define CONTROL_MIN_POS_HZ                   50.0f
#define CONTROL_MAX_POS_HZ                 2000.0f
#define CONTROL_MIN_VEL_HZ                  100.0f
#define CONTROL_MAX_VEL_HZ                10000.0f
#define CONTROL_MIN_RATE_RATIO                2.0f

#define CONTROL_WAVE_FREQ_HZ                  0.25f
#define CONTROL_WAVE_POSITION_AMP_DEG        90.0f
#define CONTROL_WAVE_VELOCITY_AMP_RPM        80.0f

#define CONTROL_FREQ_MIN_HZ                   0.10f
#define CONTROL_FREQ_MAX_HZ                  10.00f
#define CONTROL_FREQ_INPUT_AMP_DEG           45.0f
#define CONTROL_FREQ_CYCLES_PER_POINT         4.0f
#define CONTROL_FREQ_SETTLE_CYCLES            1.0f
#define CONTROL_FREQ_MIN_SAMPLES_PER_POINT   60U
#define CONTROL_FREQ_DB_FLOOR               -80.0f

volatile float pos_controller_output_velocity = 0.0f;
volatile uint8_t tracking_toggle_request = 0U;

// Variables for readout / live plotting.
volatile float target_velocity1 = 0.0f;
volatile float current_position = 0.0f;
volatile float target_position1 = 0.0f;
volatile float measured_velocity1 = 0.0f;
volatile float encoder_measured_position1 = 0.0f;
volatile float pwm_duty1 = 0.0f;

/*
 * current_state:
 *     What the controller is doing right now.
 *     STATE_DISABLED means tracking/control output is off.
 *
 * desired_state:
 *     The mode the controller should enter the next time tracking is enabled.
 *     The HMI POSITION/VELOCITY/FREQ buttons change this value while disabled.
 */
volatile Controller_State current_state = STATE_DISABLED;
volatile Controller_State desired_state = STATE_POSITION_CONTROL;
volatile ControlInputSource_t control_input_source = CONTROL_INPUT_ENCODER;

volatile float control_position_loop_hz = CONTROL_DEFAULT_POS_HZ;
volatile float control_velocity_loop_hz = CONTROL_DEFAULT_VEL_HZ;

volatile uint8_t freq_response_active = 0U;
volatile uint8_t freq_response_done = 0U;
volatile uint16_t freq_response_point_count = 0U;
volatile uint16_t freq_response_current_index = 0U;
volatile float freq_response_current_hz = CONTROL_FREQ_MIN_HZ;
volatile float freq_response_freq_hz[CONTROL_FREQ_RESPONSE_POINTS] = {0.0f};
volatile float freq_response_mag_db[CONTROL_FREQ_RESPONSE_POINTS] = {0.0f};
volatile float freq_response_output_amp_deg[CONTROL_FREQ_RESPONSE_POINTS] = {0.0f};

static ControlPreset_t control_presets[CONTROL_PRESET_COUNT];

static volatile uint32_t control_input_start_ms = 0U;

static float freq_response_baseline_deg = 0.0f;
static float freq_response_phase_rad = 0.0f;
static float freq_response_min_deg = 0.0f;
static float freq_response_max_deg = 0.0f;
static uint32_t freq_response_sample_counter = 0U;
static uint32_t freq_response_settle_samples = 0U;
static uint32_t freq_response_total_samples = 0U;

static float Control_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Control_ClampF(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint8_t Control_IsModeState(Controller_State mode)
{
    return ((mode == STATE_POSITION_CONTROL) ||
            (mode == STATE_VELOCITY_CONTROL) ||
            (mode == STATE_FREQUENCY_RESPONSE)) ? 1U : 0U;
}

uint8_t Control_IsTrackingEnabled(void)
{
    return Control_IsModeState(current_state);
}

static float Control_TimerRateHz(TIM_TypeDef *tim)
{
    uint32_t psc = tim->PSC + 1U;
    uint32_t arr = tim->ARR + 1U;

    if ((psc == 0U) || (arr == 0U))
    {
        return 0.0f;
    }

    return ((float)SystemCoreClock) / ((float)psc * (float)arr);
}

static float Control_ConfigureTimerHz(TIM_TypeDef *tim, float requested_hz, uint32_t max_arr)
{
    if (requested_hz < 1.0f)
    {
        requested_hz = 1.0f;
    }

    uint32_t was_enabled = (tim->CR1 & TIM_CR1_CEN);
    tim->CR1 &= ~TIM_CR1_CEN;

    uint32_t psc = 0U;
    float ticks = ((float)SystemCoreClock) / requested_hz;
    float max_ticks = (float)max_arr + 1.0f;

    while ((ticks > max_ticks) && (psc < 0xFFFFU))
    {
        psc++;
        ticks = ((float)SystemCoreClock) / (((float)psc + 1.0f) * requested_hz);
    }

    uint32_t arr_plus_one = (uint32_t)(ticks + 0.5f);
    if (arr_plus_one < 1U)
    {
        arr_plus_one = 1U;
    }
    if ((float)arr_plus_one > max_ticks)
    {
        arr_plus_one = max_arr;
    }

    tim->PSC = psc;
    tim->ARR = arr_plus_one - 1U;
    tim->CNT = 0U;
    tim->EGR |= TIM_EGR_UG;
    tim->SR &= ~TIM_SR_UIF;

    if (was_enabled != 0U)
    {
        tim->CR1 |= TIM_CR1_CEN;
    }

    return Control_TimerRateHz(tim);
}

float Control_GetPositionLoopHz(void)
{
    return control_position_loop_hz;
}

float Control_GetVelocityLoopHz(void)
{
    return control_velocity_loop_hz;
}

float Control_SetVelocityLoopHz(float requested_hz)
{
    requested_hz = Control_ClampF(requested_hz,
                                  CONTROL_MIN_VEL_HZ,
                                  CONTROL_MAX_VEL_HZ);

    float minimum_velocity_hz = control_position_loop_hz * CONTROL_MIN_RATE_RATIO;
    if (requested_hz < minimum_velocity_hz)
    {
        requested_hz = minimum_velocity_hz;
    }
    if (requested_hz > CONTROL_MAX_VEL_HZ)
    {
        requested_hz = CONTROL_MAX_VEL_HZ;
    }

    control_velocity_loop_hz = requested_hz;

    if ((RCC->APB1ENR1 & RCC_APB1ENR1_TIM5EN) != 0U)
    {
        control_velocity_loop_hz = Control_ConfigureTimerHz(TIM5,
                                                            requested_hz,
                                                            0xFFFFFFFFUL);
    }

    if (control_velocity_loop_hz > 0.0f)
    {
        ctx_vel.dt = 1.0f / control_velocity_loop_hz;
        Encoder_SetSampleRateHz(control_velocity_loop_hz);
    }

    return control_velocity_loop_hz;
}

float Control_SetPositionLoopHz(float requested_hz)
{
    requested_hz = Control_ClampF(requested_hz,
                                  CONTROL_MIN_POS_HZ,
                                  CONTROL_MAX_POS_HZ);

    if ((requested_hz * CONTROL_MIN_RATE_RATIO) > CONTROL_MAX_VEL_HZ)
    {
        requested_hz = CONTROL_MAX_VEL_HZ / CONTROL_MIN_RATE_RATIO;
    }

    if (control_velocity_loop_hz < (requested_hz * CONTROL_MIN_RATE_RATIO))
    {
        (void)Control_SetVelocityLoopHz(requested_hz * CONTROL_MIN_RATE_RATIO);
    }

    if (requested_hz > (control_velocity_loop_hz / CONTROL_MIN_RATE_RATIO))
    {
        requested_hz = control_velocity_loop_hz / CONTROL_MIN_RATE_RATIO;
    }

    control_position_loop_hz = requested_hz;

    if ((RCC->APB1ENR1 & RCC_APB1ENR1_TIM6EN) != 0U)
    {
        control_position_loop_hz = Control_ConfigureTimerHz(TIM6,
                                                            requested_hz,
                                                            0xFFFFUL);
    }

    if (control_position_loop_hz > 0.0f)
    {
        ctx_pos.dt = 1.0f / control_position_loop_hz;
    }

    return control_position_loop_hz;
}

void setup_LOOPTIMERS(void)
{
    // enable TIM5,6 peripheral clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

    (void)Control_SetVelocityLoopHz(control_velocity_loop_hz);
    (void)Control_SetPositionLoopHz(control_position_loop_hz);

    TIM5->DIER |= TIM_DIER_UIE; // enable interrupts for tim5
    TIM6->DIER |= TIM_DIER_UIE; // enable interrupts for tim6

    TIM5->SR &= ~(TIM_SR_UIF);
    TIM6->SR &= ~(TIM_SR_UIF);

    // enable TIM5 and TIM6 IRQ in NVIC
    NVIC->ISER[1] |= (1U << (TIM5_IRQn & 0x1FU));
    NVIC->ISER[1] |= (1U << (TIM6_DAC_IRQn & 0x1FU));

    // global IRQ enable
    __enable_irq();

    // start timers
    TIM5->CR1 |= TIM_CR1_CEN;
    TIM6->CR1 |= TIM_CR1_CEN;
}

void PI_Init(MotorController_t *ctx, float kp, float ki, float dt,
        float lower_limit, float upper_limit)
{
    ctx->kp = kp;
    ctx->ki = ki;
    ctx->integrator_accum = 0.0f;
    ctx->dt = dt;
    ctx->output_limit_high = upper_limit;
    ctx->output_limit_low = lower_limit;
}

float PI_Update(MotorController_t *ctx, float target, float measured)
{
    float raw_error = target - measured;
    float error = (raw_error > -1.5f && raw_error < 1.5f) ? 0.0f : raw_error;

    ctx->integrator_accum += error * ctx->dt;

    if (ctx->integrator_accum > 250.0f)
    {
        ctx->integrator_accum = 250.0f;
    }
    else if (ctx->integrator_accum < -250.0f)
    {
        ctx->integrator_accum = -250.0f;
    }

    float output = (ctx->kp * error) + (ctx->ki * ctx->integrator_accum);

    if (output > ctx->output_limit_high)
    {
        output = ctx->output_limit_high;
    }
    else if (output < ctx->output_limit_low)
    {
        output = ctx->output_limit_low;
    }

    return output;
}

void PI_Reset(MotorController_t *ctx)
{
    ctx->integrator_accum = 0.0f;
}

const char *Control_GetInputSourceName(ControlInputSource_t source)
{
    switch (source)
    {
        case CONTROL_INPUT_ENCODER:
            return "ENC";
        case CONTROL_INPUT_SINE:
            return "SIN";
        case CONTROL_INPUT_COSINE:
            return "COS";
        case CONTROL_INPUT_RAMP:
            return "RAMP";
        case CONTROL_INPUT_SQUARE:
            return "SQUARE";
        case CONTROL_INPUT_STEP:
            return "STEP";
        case CONTROL_INPUT_COUNT:
        default:
            return "?";
    }
}

void Control_SetInputSource(ControlInputSource_t source)
{
    if (source >= CONTROL_INPUT_COUNT)
    {
        source = CONTROL_INPUT_ENCODER;
    }

    control_input_source = source;
    control_input_start_ms = HAL_GetTick();
}

void Control_NextInputSource(int32_t direction)
{
    int32_t next = (int32_t)control_input_source;

    if (direction >= 0)
    {
        next++;
    }
    else
    {
        next--;
    }

    if (next >= (int32_t)CONTROL_INPUT_COUNT)
    {
        next = 0;
    }
    else if (next < 0)
    {
        next = (int32_t)CONTROL_INPUT_COUNT - 1;
    }

    Control_SetInputSource((ControlInputSource_t)next);
}

static float Control_InputSeconds(void)
{
    return ((float)(HAL_GetTick() - control_input_start_ms)) * 0.001f;
}

static float Control_WaveUnit(ControlInputSource_t source, float t_seconds)
{
    float phase = fmodf(t_seconds * CONTROL_WAVE_FREQ_HZ, 1.0f);
    if (phase < 0.0f)
    {
        phase += 1.0f;
    }

    switch (source)
    {
        case CONTROL_INPUT_SINE:
            return sinf(2.0f * CONTROL_PI_F * phase);

        case CONTROL_INPUT_COSINE:
            return cosf(2.0f * CONTROL_PI_F * phase);

        case CONTROL_INPUT_RAMP:
            return (2.0f * phase) - 1.0f;

        case CONTROL_INPUT_SQUARE:
            return (phase < 0.5f) ? 1.0f : -1.0f;

        case CONTROL_INPUT_STEP:
            return (t_seconds >= 0.50f) ? 1.0f : 0.0f;

        case CONTROL_INPUT_ENCODER:
        case CONTROL_INPUT_COUNT:
        default:
            return 0.0f;
    }
}

float Control_GetInputTargetPositionDeg(void)
{
    ControlInputSource_t source = control_input_source;

    if (source == CONTROL_INPUT_ENCODER)
    {
        return HMI_Encoder_GetDegrees();
    }

    return CONTROL_WAVE_POSITION_AMP_DEG * Control_WaveUnit(source, Control_InputSeconds());
}

float Control_GetInputTargetVelocityRPM(void)
{
    ControlInputSource_t source = control_input_source;

    if (source == CONTROL_INPUT_ENCODER)
    {
        /* Manual velocity target from the front-panel encoder. */
        return HMI_Encoder_GetDegrees() * 0.25f;
    }

    return CONTROL_WAVE_VELOCITY_AMP_RPM * Control_WaveUnit(source, Control_InputSeconds());
}

static void Control_ResetLoopState(void)
{
    PI_Reset(&ctx_pos);
    PI_Reset(&ctx_vel);
    pos_controller_output_velocity = 0.0f;
    target_velocity1 = 0.0f;
    pwm_duty1 = 0.0f;
}

static void Control_BeginMode(Controller_State mode)
{
    Control_ResetLoopState();

    float measured_position = Encoder_GetDegrees();
    current_position = measured_position;
    encoder_measured_position1 = measured_position;
    target_position1 = measured_position;

    if (mode == STATE_FREQUENCY_RESPONSE)
    {
        Control_FrequencyResponseReset();
    }
    else
    {
        freq_response_active = 0U;
        freq_response_done = 0U;
    }
}

void Control_SetDesiredState(Controller_State requested_state)
{
    if (Control_IsModeState(requested_state) == 0U)
    {
        return;
    }

    desired_state = requested_state;

    /*
     * If tracking is already enabled, allow live switching between position,
     * velocity, and frequency-response modes. If tracking is disabled, only
     * desired_state changes.
     */
    if (Control_IsTrackingEnabled() != 0U)
    {
        if (current_state != requested_state)
        {
            current_state = requested_state;
            Control_BeginMode(requested_state);
        }
    }
}

void Control_RequestVelocityMode(void)
{
    Control_SetDesiredState(STATE_VELOCITY_CONTROL);
}

void Control_RequestPositionMode(void)
{
    Control_SetDesiredState(STATE_POSITION_CONTROL);
}

void Control_RequestFrequencyResponseMode(void)
{
    Control_SetDesiredState(STATE_FREQUENCY_RESPONSE);
}

void Control_EnterTracking(void)
{
    float measured_position = Encoder_GetDegrees();

    current_position = measured_position;
    encoder_measured_position1 = measured_position;
    pos_controller_output_velocity = 0.0f;
    target_velocity1 = 0.0f;
    target_position1 = measured_position;
    pwm_duty1 = 0.0f;

    update_Motor_Velocity(0.0f);

    if (Control_IsModeState(desired_state) == 0U)
    {
        desired_state = STATE_POSITION_CONTROL;
    }

    current_state = desired_state;
    control_input_start_ms = HAL_GetTick();
    Control_BeginMode(current_state);
}

void Control_ExitTracking(void)
{
    current_state = STATE_DISABLED;

    Control_ResetLoopState();
    freq_response_active = 0U;
    freq_response_done = 0U;

    update_Motor_Velocity(0.0f);
}

void Control_ToggleTracking(void)
{
    if (current_state == STATE_DISABLED)
    {
        Control_EnterTracking();
    }
    else if (Control_IsTrackingEnabled() != 0U)
    {
        Control_ExitTracking();
    }
}

void Control_InitPresetsFromCurrent(void)
{
    for (uint8_t i = 0U; i < CONTROL_PRESET_COUNT; i++)
    {
        Control_SavePreset(i);
    }
}

void Control_SavePreset(uint8_t slot)
{
    if (slot >= CONTROL_PRESET_COUNT)
    {
        return;
    }

    control_presets[slot].pos_kp = ctx_pos.kp;
    control_presets[slot].pos_ki = ctx_pos.ki;
    control_presets[slot].vel_kp = ctx_vel.kp;
    control_presets[slot].vel_ki = ctx_vel.ki;
}

void Control_LoadPreset(uint8_t slot)
{
    if (slot >= CONTROL_PRESET_COUNT)
    {
        return;
    }

    ctx_pos.kp = control_presets[slot].pos_kp;
    ctx_pos.ki = control_presets[slot].pos_ki;
    ctx_vel.kp = control_presets[slot].vel_kp;
    ctx_vel.ki = control_presets[slot].vel_ki;

    Control_ResetLoopState();
}

ControlPreset_t Control_GetPreset(uint8_t slot)
{
    ControlPreset_t empty = {0.0f, 0.0f, 0.0f, 0.0f};

    if (slot >= CONTROL_PRESET_COUNT)
    {
        return empty;
    }

    return control_presets[slot];
}

void Control_SetPresetValue(uint8_t slot, uint8_t field, float value)
{
    if (slot >= CONTROL_PRESET_COUNT)
    {
        return;
    }

    switch (field)
    {
        case 0U:
            control_presets[slot].pos_kp = value;
            break;
        case 1U:
            control_presets[slot].pos_ki = value;
            break;
        case 2U:
            control_presets[slot].vel_kp = value;
            break;
        case 3U:
            control_presets[slot].vel_ki = value;
            break;
        default:
            break;
    }
}

static void Control_FrequencyResponseStartPoint(uint16_t index)
{
    if (index >= CONTROL_FREQ_RESPONSE_POINTS)
    {
        freq_response_active = 0U;
        freq_response_done = 1U;
        return;
    }

    float frac = 0.0f;
    if (CONTROL_FREQ_RESPONSE_POINTS > 1U)
    {
        frac = (float)index / (float)(CONTROL_FREQ_RESPONSE_POINTS - 1U);
    }

    freq_response_current_hz = CONTROL_FREQ_MIN_HZ *
        powf((CONTROL_FREQ_MAX_HZ / CONTROL_FREQ_MIN_HZ), frac);

    freq_response_current_index = index;
    freq_response_phase_rad = 0.0f;
    freq_response_sample_counter = 0U;
    freq_response_min_deg = 1000000.0f;
    freq_response_max_deg = -1000000.0f;

    float dt = ctx_pos.dt;
    if (dt <= 0.0f)
    {
        dt = 1.0f / CONTROL_DEFAULT_POS_HZ;
    }

    float samples_per_cycle = 1.0f / (freq_response_current_hz * dt);
    freq_response_settle_samples = (uint32_t)((CONTROL_FREQ_SETTLE_CYCLES * samples_per_cycle) + 0.5f);
    freq_response_total_samples = (uint32_t)((CONTROL_FREQ_CYCLES_PER_POINT * samples_per_cycle) + 0.5f);

    if (freq_response_total_samples < CONTROL_FREQ_MIN_SAMPLES_PER_POINT)
    {
        freq_response_total_samples = CONTROL_FREQ_MIN_SAMPLES_PER_POINT;
    }

    if (freq_response_settle_samples >= freq_response_total_samples)
    {
        freq_response_settle_samples = freq_response_total_samples / 2U;
    }

    freq_response_freq_hz[index] = freq_response_current_hz;
    freq_response_mag_db[index] = CONTROL_FREQ_DB_FLOOR;
    freq_response_output_amp_deg[index] = 0.0f;
}

void Control_FrequencyResponseReset(void)
{
    freq_response_baseline_deg = Encoder_GetDegrees();
    freq_response_point_count = 0U;
    freq_response_done = 0U;
    freq_response_active = 1U;

    for (uint16_t i = 0U; i < CONTROL_FREQ_RESPONSE_POINTS; i++)
    {
        freq_response_freq_hz[i] = 0.0f;
        freq_response_mag_db[i] = CONTROL_FREQ_DB_FLOOR;
        freq_response_output_amp_deg[i] = 0.0f;
    }

    Control_FrequencyResponseStartPoint(0U);
}

uint8_t Control_FrequencyResponseIsDone(void)
{
    return freq_response_done;
}

static float Control_FrequencyResponseTarget(float measured_position)
{
    if ((freq_response_active == 0U) || (freq_response_done != 0U))
    {
        return freq_response_baseline_deg;
    }

    float target = freq_response_baseline_deg +
        (CONTROL_FREQ_INPUT_AMP_DEG * sinf(freq_response_phase_rad));

    float dt = ctx_pos.dt;
    if (dt <= 0.0f)
    {
        dt = 1.0f / CONTROL_DEFAULT_POS_HZ;
    }

    freq_response_phase_rad += 2.0f * CONTROL_PI_F * freq_response_current_hz * dt;
    while (freq_response_phase_rad >= (2.0f * CONTROL_PI_F))
    {
        freq_response_phase_rad -= 2.0f * CONTROL_PI_F;
    }

    if (freq_response_sample_counter >= freq_response_settle_samples)
    {
        if (measured_position < freq_response_min_deg)
        {
            freq_response_min_deg = measured_position;
        }
        if (measured_position > freq_response_max_deg)
        {
            freq_response_max_deg = measured_position;
        }
    }

    freq_response_sample_counter++;

    if (freq_response_sample_counter >= freq_response_total_samples)
    {
        float output_amp = 0.0f;
        if (freq_response_max_deg > freq_response_min_deg)
        {
            output_amp = (freq_response_max_deg - freq_response_min_deg) * 0.5f;
        }

        float mag = output_amp / CONTROL_FREQ_INPUT_AMP_DEG;
        float mag_db = CONTROL_FREQ_DB_FLOOR;
        if (mag > 0.00001f)
        {
            mag_db = 20.0f * log10f(mag);
            if (mag_db < CONTROL_FREQ_DB_FLOOR)
            {
                mag_db = CONTROL_FREQ_DB_FLOOR;
            }
        }

        uint16_t index = freq_response_current_index;
        if (index < CONTROL_FREQ_RESPONSE_POINTS)
        {
            freq_response_output_amp_deg[index] = output_amp;
            freq_response_mag_db[index] = mag_db;
            freq_response_point_count = (uint16_t)(index + 1U);
        }

        index++;
        if (index >= CONTROL_FREQ_RESPONSE_POINTS)
        {
            freq_response_active = 0U;
            freq_response_done = 1U;
        }
        else
        {
            Control_FrequencyResponseStartPoint(index);
        }
    }

    return target;
}

void TIM5_IRQHandler(void) // velocity control loop
{
    GPIOC->ODR ^= 0x20U;

    if ((TIM5->SR & TIM_SR_UIF) != 0U)
    {
        TIM5->SR &= ~TIM_SR_UIF;

        // Start conversion for diagnostics / legacy analog input availability.
        ADC_StartConversion();

        // Check if controller enabled
        if ((current_state == STATE_DISABLED) || (current_state == STATE_FAULT))
        {
            pos_controller_output_velocity = 0.0f;
            pwm_duty1 = 0.0f;
            update_Motor_Velocity(0.0f);
            return;
        }

        float measured_velocity = Encoder_GetVelocityRPM();
        measured_velocity1 = measured_velocity;

        float target_velocity = 0.0f;
        if ((current_state == STATE_POSITION_CONTROL) ||
            (current_state == STATE_FREQUENCY_RESPONSE))
        {
            target_velocity = pos_controller_output_velocity;
        }
        else if (current_state == STATE_VELOCITY_CONTROL)
        {
            target_velocity = Control_GetInputTargetVelocityRPM();
        }
        else
        {
            target_velocity = 0.0f;
        }

        target_velocity1 = target_velocity;

        if ((current_state == STATE_FREQUENCY_RESPONSE) && (freq_response_done != 0U))
        {
            pos_controller_output_velocity = 0.0f;
            target_velocity1 = 0.0f;
            pwm_duty1 = 0.0f;
            update_Motor_Velocity(0.0f);
            return;
        }

        float pwm_duty = PI_Update(&ctx_vel, target_velocity, measured_velocity);
        pwm_duty1 = pwm_duty;
        update_Motor_Velocity(pwm_duty);
    }
}

void TIM6_DAC_IRQHandler(void) // position control loop
{
    GPIOC->ODR ^= 0x40U;

    if ((TIM6->SR & TIM_SR_UIF) != 0U)
    {
        TIM6->SR &= ~TIM_SR_UIF; // clear interrupt flag

        // Handle Tracking Toggle request
        if (tracking_toggle_request != 0U)
        {
            tracking_toggle_request = 0U;
            Control_ToggleTracking();
        }

        if (current_state == STATE_POSITION_CONTROL)
        {
            float measured_position = Encoder_GetDegrees();
            current_position = measured_position;
            encoder_measured_position1 = measured_position;

            float target_position = Control_GetInputTargetPositionDeg();
            target_position1 = target_position;

            pos_controller_output_velocity = PI_Update(&ctx_pos,
                                                       target_position,
                                                       measured_position);
            target_velocity1 = pos_controller_output_velocity;
        }
        else if (current_state == STATE_FREQUENCY_RESPONSE)
        {
            float measured_position = Encoder_GetDegrees();
            current_position = measured_position;
            encoder_measured_position1 = measured_position;

            float target_position = Control_FrequencyResponseTarget(measured_position);
            target_position1 = target_position;

            if (freq_response_done != 0U)
            {
                pos_controller_output_velocity = 0.0f;
                target_velocity1 = 0.0f;
            }
            else
            {
                pos_controller_output_velocity = PI_Update(&ctx_pos,
                                                           target_position,
                                                           measured_position);
                target_velocity1 = pos_controller_output_velocity;
            }
        }
    }
}
