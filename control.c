/*
 * control.c
 *
 * Cascaded position/velocity PI controller. Frequency-response code has been
 * removed; only disabled, position, velocity, and fault states remain.
 */

#include "control.h"

#include <math.h>
#include <stddef.h>

#include "ADC.h"
#include "HMI.h"
#include "encoder.h"
#include "motor.h"

volatile float pos_controller_output_velocity = 0.0f;
volatile uint8_t tracking_toggle_request = 0U;

volatile float target_velocity1 = 0.0f;
volatile float current_position = 0.0f;
volatile float target_position1 = 0.0f;
volatile float measured_velocity1 = 0.0f;
volatile float encoder_measured_position1 = 0.0f;
volatile float pwm_duty1 = 0.0f;
volatile float integrator_accum1 = 0.0f;

volatile Controller_State current_state = STATE_DISABLED;
volatile Controller_State desired_state = STATE_POSITION_CONTROL;
volatile ControlInputSource_t control_input_source = CONTROL_INPUT_USER;

volatile float control_position_loop_hz = CONTROL_DEFAULT_POS_HZ;
volatile float control_velocity_loop_hz = CONTROL_DEFAULT_VEL_HZ;
volatile float control_input_wave_freq_hz = CONTROL_INPUT_DEFAULT_FREQ_HZ;
volatile float control_input_wave_amplitude = CONTROL_INPUT_DEFAULT_AMPLITUDE;

static ControlPreset_t control_presets[CONTROL_PRESET_COUNT];
static volatile uint32_t control_input_start_ms = 0U;

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
            (mode == STATE_VELOCITY_CONTROL)) ? 1U : 0U;
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
        arr_plus_one = max_arr + 1U;
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

float Control_SetPositionOutputRange(float requested_range)
{
    ctx_pos.output_range = Control_ClampF(Control_AbsF(requested_range),
                                          1.0f,
                                          CONTROL_MAX_POS_RANGE_RPM);
    control_input_wave_amplitude = Control_SetInputWaveAmplitude(control_input_wave_amplitude);
    return ctx_pos.output_range;
}

float Control_SetVelocityOutputRange(float requested_range)
{
    ctx_vel.output_range = Control_ClampF(Control_AbsF(requested_range),
                                          1.0f,
                                          CONTROL_MAX_VEL_RANGE_PWM);
    return ctx_vel.output_range;
}

void setup_LOOPTIMERS(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

    (void)Control_SetVelocityLoopHz(control_velocity_loop_hz);
    (void)Control_SetPositionLoopHz(control_position_loop_hz);

    TIM5->DIER |= TIM_DIER_UIE;
    TIM6->DIER |= TIM_DIER_UIE;

    TIM5->SR &= ~(TIM_SR_UIF);
    TIM6->SR &= ~(TIM_SR_UIF);

    NVIC->ISER[1] |= (1U << (TIM5_IRQn & 0x1FU));
    NVIC->ISER[1] |= (1U << (TIM6_DAC_IRQn & 0x1FU));

    __enable_irq();

    TIM5->CR1 |= TIM_CR1_CEN;
    TIM6->CR1 |= TIM_CR1_CEN;
}

void PI_Init(MotorController_t *ctx, float kp, float ki, float dt,
             float output_range, float error_margin)
{
    if (ctx == NULL)
    {
        return;
    }

    ctx->kp = kp;
    ctx->ki = ki;
    ctx->integrator_accum = 0.0f;
    ctx->dt = dt;
    ctx->output_range = Control_AbsF(output_range);
    ctx->error_margin = error_margin;
    ctx->target = 0.0f;
    ctx->fault_flags = 0U;
}

float PI_Update(MotorController_t *ctx, float target, float measured)
{
    if (ctx == NULL)
    {
        return 0.0f;
    }

    ctx->target = target;

    float raw_error = target - measured;
    float error = (Control_AbsF(raw_error) < ctx->error_margin) ? 0.0f : raw_error;
    float range = Control_AbsF(ctx->output_range);

    if (range < 0.001f)
    {
        ctx->integrator_accum = 0.0f;
        integrator_accum1 = 0.0f;
        return 0.0f;
    }

    float proposed_integrator = ctx->integrator_accum + (error * ctx->dt);

    if (ctx->ki > 0.000001f)
    {
        float integrator_limit = range / ctx->ki;
        proposed_integrator = Control_ClampF(proposed_integrator,
                                             -integrator_limit,
                                             integrator_limit);
    }

    float proportional = ctx->kp * error;
    float proposed_output = proportional + (ctx->ki * proposed_integrator);

    uint8_t integrating_into_saturation = 0U;
    if (((proposed_output > range) && (error > 0.0f)) ||
        ((proposed_output < -range) && (error < 0.0f)))
    {
        integrating_into_saturation = 1U;
    }

    if (integrating_into_saturation == 0U)
    {
        ctx->integrator_accum = proposed_integrator;
    }

    integrator_accum1 = ctx->integrator_accum;

    float output = proportional + (ctx->ki * ctx->integrator_accum);
    return Control_ClampF(output, -range, range);
}

void PI_Reset(MotorController_t *ctx)
{
    if (ctx != NULL)
    {
        ctx->integrator_accum = 0.0f;
        ctx->target = 0.0f;
    }
}

const char *Control_GetInputSourceName(ControlInputSource_t source)
{
    switch (source)
    {
        case CONTROL_INPUT_USER:
            return "USER";
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
        source = CONTROL_INPUT_USER;
    }

    control_input_source = source;
    control_input_start_ms = HAL_GetTick();
}

void Control_NextInputSource(int32_t direction)
{
    int32_t next = (int32_t)control_input_source;

    next += (direction >= 0) ? 1 : -1;

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

float Control_SetInputWaveFrequencyHz(float requested_hz)
{
    control_input_wave_freq_hz = Control_ClampF(requested_hz,
                                                CONTROL_INPUT_MIN_FREQ_HZ,
                                                CONTROL_INPUT_MAX_FREQ_HZ);
    return control_input_wave_freq_hz;
}

float Control_SetInputWaveAmplitude(float requested_amplitude)
{
    float max_amplitude = ctx_pos.output_range;
    if (max_amplitude < 1.0f)
    {
        max_amplitude = CONTROL_DEFAULT_POS_RANGE_RPM;
    }

    control_input_wave_amplitude = Control_ClampF(Control_AbsF(requested_amplitude),
                                                  CONTROL_INPUT_MIN_AMPLITUDE,
                                                  max_amplitude);
    return control_input_wave_amplitude;
}

float Control_GetInputWaveFrequencyHz(void)
{
    return control_input_wave_freq_hz;
}

float Control_GetInputWaveAmplitude(void)
{
    return control_input_wave_amplitude;
}

static float Control_InputSeconds(void)
{
    return ((float)(HAL_GetTick() - control_input_start_ms)) * 0.001f;
}

static float Control_WaveUnit(ControlInputSource_t source, float t_seconds)
{
    float freq_hz = control_input_wave_freq_hz;
    if (freq_hz < CONTROL_INPUT_MIN_FREQ_HZ)
    {
        freq_hz = CONTROL_INPUT_MIN_FREQ_HZ;
    }

    float phase = fmodf(t_seconds * freq_hz, 1.0f);
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

        case CONTROL_INPUT_USER:
        case CONTROL_INPUT_COUNT:
        default:
            return 0.0f;
    }
}

static float Control_PotToSignedRange(float range)
{
    float clamped_adc = (float)rawVoltageData;
    clamped_adc = Control_ClampF(clamped_adc, 0.0f, 4095.0f);

    float normalized = (clamped_adc / 4095.0f) * 2.0f - 1.0f;
    return normalized * range;
}

float Control_GetInputTargetPositionDeg(void)
{
    ControlInputSource_t source = control_input_source;

    if (source == CONTROL_INPUT_USER)
    {
        return HMI_Encoder_GetDegrees();
    }

    return control_input_wave_amplitude * Control_WaveUnit(source, Control_InputSeconds());
}

float Control_GetInputTargetVelocityRPM(void)
{
    ControlInputSource_t source = control_input_source;
    float max_velocity = Control_AbsF(ctx_pos.output_range);

    if (max_velocity < 1.0f)
    {
        max_velocity = CONTROL_DEFAULT_POS_RANGE_RPM;
    }

    if (source == CONTROL_INPUT_USER)
    {
        return Control_PotToSignedRange(max_velocity);
    }

    float amp = Control_ClampF(control_input_wave_amplitude, 0.0f, max_velocity);
    return amp * Control_WaveUnit(source, Control_InputSeconds());
}

static void Control_ResetLoopState(void)
{
    PI_Reset(&ctx_pos);
    PI_Reset(&ctx_vel);
    pos_controller_output_velocity = 0.0f;
    target_velocity1 = 0.0f;
    target_position1 = 0.0f;
    measured_velocity1 = 0.0f;
    current_position = 0.0f;
    encoder_measured_position1 = 0.0f;
    pwm_duty1 = 0.0f;
    integrator_accum1 = 0.0f;
}

static void Control_BeginMode(Controller_State mode)
{
    Control_ResetLoopState();

    float measured_position = Encoder_GetDegrees();
    current_position = measured_position;
    encoder_measured_position1 = measured_position;
    target_position1 = measured_position;

    if (mode == STATE_VELOCITY_CONTROL)
    {
        target_velocity1 = Control_GetInputTargetVelocityRPM();
    }
}

void Control_SetDesiredState(Controller_State requested_state)
{
    if (Control_IsModeState(requested_state) == 0U)
    {
        return;
    }

    desired_state = requested_state;

    if ((Control_IsTrackingEnabled() != 0U) && (current_state != requested_state))
    {
        current_state = requested_state;
        Control_BeginMode(requested_state);
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

void Control_EnterTracking(void)
{
    Control_ResetLoopState();
    update_Motor_Velocity(0.0f);

    Encoder_ResetCount();
    HMI_Encoder_ResetCount();

    if (Control_IsModeState(desired_state) == 0U)
    {
        desired_state = STATE_POSITION_CONTROL;
    }

    current_state = desired_state;
    control_input_start_ms = HAL_GetTick();

    current_position = 0.0f;
    encoder_measured_position1 = 0.0f;
    target_position1 = 0.0f;
    target_velocity1 = 0.0f;
    measured_velocity1 = 0.0f;

    Control_BeginMode(current_state);
}

void Control_ExitTracking(void)
{
    current_state = STATE_DISABLED;
    tracking_toggle_request = 0U;
    Control_ResetLoopState();
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

void TIM5_IRQHandler(void)
{
    GPIOC->ODR ^= 0x20U;

    if ((TIM5->SR & TIM_SR_UIF) != 0U)
    {
        TIM5->SR &= ~TIM_SR_UIF;

        ADC_StartConversion();

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
        if (current_state == STATE_POSITION_CONTROL)
        {
            target_velocity = pos_controller_output_velocity;
        }
        else if (current_state == STATE_VELOCITY_CONTROL)
        {
            target_velocity = Control_GetInputTargetVelocityRPM();
        }

        target_velocity1 = target_velocity;

        float pwm_duty = PI_Update(&ctx_vel, target_velocity, measured_velocity);
        pwm_duty1 = pwm_duty;
        update_Motor_Velocity(pwm_duty);
    }
}

void TIM6_DAC_IRQHandler(void)
{
    GPIOC->ODR ^= 0x40U;

    if ((TIM6->SR & TIM_SR_UIF) != 0U)
    {
        TIM6->SR &= ~TIM_SR_UIF;

        if (tracking_toggle_request != 0U)
        {
            tracking_toggle_request = 0U;
            Control_ToggleTracking();
        }

        float measured_position = Encoder_GetDegrees();
        current_position = measured_position;
        encoder_measured_position1 = measured_position;

        if (current_state == STATE_POSITION_CONTROL)
        {
            float target_position = Control_GetInputTargetPositionDeg();
            target_position1 = target_position;

            pos_controller_output_velocity = PI_Update(&ctx_pos,
                                                       target_position,
                                                       measured_position);
            target_velocity1 = pos_controller_output_velocity;
        }
        else if (current_state == STATE_VELOCITY_CONTROL)
        {
            target_position1 = 0.0f;
            pos_controller_output_velocity = 0.0f;
        }
    }
}
