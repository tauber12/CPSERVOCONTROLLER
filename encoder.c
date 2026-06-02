/*
 *******************************************************************************
 * @file           : encoder.c
 * @brief          : Quadrature encoder driver with stable adaptive RPM output
 *******************************************************************************
 *
 * TIM2 counts PA0/PA1 in x4 encoder mode. TIM3 is a 1 MHz free-running
 * timebase extended in software. RPM is calculated from TIM2 count deltas over
 * an adaptive time window instead of from a single control-loop sample. This is
 * intentionally less noisy at low speed and prevents random positive/negative
 * sign flips when only a few counts arrive per sample.
 *******************************************************************************
 */

#include "encoder.h"

#define ENCODER_TIM3_TICK_HZ             1000000UL
#define ENCODER_TIM3_PRESCALER          ((48000000UL / ENCODER_TIM3_TICK_HZ) - 1UL)
#define ENCODER_TIM3_PERIOD_US          65536UL
#define ENCODER_TIM3_MASK               0xFFFFUL

static volatile uint32_t tim3_high_us = 0U;
static volatile float encoder_sample_rate_hz = ENCODER_DEFAULT_SAMPLE_HZ;
static volatile float encoder_low_rpm_threshold =
    (((float)ENCODER_RPM_MIN_UPDATE_COUNTS * 60.0f * 1000000.0f) /
     ((float)ENCODER_RPM_COUNTS_PER_REV * (float)ENCODER_RPM_MAX_WINDOW_US));

static int32_t rpm_window_start_count = 0;
static uint32_t rpm_window_start_us = 0U;
static int32_t rpm_last_count = 0;
static uint32_t rpm_last_motion_us = 0U;
static uint8_t rpm_estimator_ready = 0U;
static float rpm_raw_hold = 0.0f;
static float rpm_filtered = 0.0f;

static int32_t cps_prev_count = 0;
static uint32_t cps_prev_us = 0U;
static uint8_t cps_ready = 0U;

static float Encoder_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t Encoder_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static float Encoder_ClampF(float value, float min_value, float max_value)
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

static float Encoder_LowRpmThresholdFromWindow(void)
{
    return (((float)ENCODER_RPM_MIN_UPDATE_COUNTS * 60.0f * 1000000.0f) /
            ((float)ENCODER_RPM_COUNTS_PER_REV * (float)ENCODER_RPM_MAX_WINDOW_US));
}

static void TIM3_ServiceOverflow(void)
{
    if ((TIM3->SR & TIM_SR_UIF) != 0U)
    {
        TIM3->SR &= ~TIM_SR_UIF;
        tim3_high_us += ENCODER_TIM3_PERIOD_US;
    }
}

void TIM3_IRQHandler(void)
{
    TIM3_ServiceOverflow();
}

static uint32_t Encoder_Micros32(void)
{
    TIM3_ServiceOverflow();

    uint32_t high = tim3_high_us;
    uint32_t low = ((uint32_t)TIM3->CNT) & ENCODER_TIM3_MASK;

    if ((TIM3->SR & TIM_SR_UIF) != 0U)
    {
        high += ENCODER_TIM3_PERIOD_US;
        low = ((uint32_t)TIM3->CNT) & ENCODER_TIM3_MASK;
    }

    return high + low;
}

static void TIM3_Init_Micros(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    TIM3->PSC = ENCODER_TIM3_PRESCALER;
    TIM3->ARR = ENCODER_TIM3_MASK;
    TIM3->CNT = 0U;
    TIM3->EGR |= TIM_EGR_UG;
    TIM3->SR &= ~TIM_SR_UIF;
    TIM3->DIER |= TIM_DIER_UIE;

    tim3_high_us = 0U;

    NVIC->ISER[0] |= (1U << (TIM3_IRQn & 0x1FU));
    TIM3->CR1 |= TIM_CR1_CEN;
}

static void Encoder_ResetVelocityState(uint32_t now_us, int32_t count)
{
    rpm_window_start_count = count;
    rpm_window_start_us = now_us;
    rpm_last_count = count;
    rpm_last_motion_us = now_us;
    rpm_estimator_ready = 1U;
    rpm_raw_hold = 0.0f;
    rpm_filtered = 0.0f;

    cps_prev_count = count;
    cps_prev_us = now_us;
    cps_ready = 1U;
}

void Encoder_RecordEdge(void)
{
    /* Kept for compatibility with older T-method code. The current estimator
     * intentionally does not enable EXTI edge timing because the adaptive TIM2
     * count-window estimate is more stable on the noisy low-speed setup. */
}

void Encoder_SetSampleRateHz(float sample_rate_hz)
{
    if (sample_rate_hz < 1.0f)
    {
        sample_rate_hz = 1.0f;
    }

    encoder_sample_rate_hz = sample_rate_hz;
    encoder_low_rpm_threshold = Encoder_LowRpmThresholdFromWindow();
}

float Encoder_GetSampleRateHz(void)
{
    return encoder_sample_rate_hz;
}

float Encoder_GetLowRpmThreshold(void)
{
    return encoder_low_rpm_threshold;
}

void Encoder_Config(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;

    GPIOA->MODER   &= ~(GPIO_MODER_MODE0     | GPIO_MODER_MODE1);
    GPIOA->MODER   |=  (GPIO_MODER_MODE0_1   | GPIO_MODER_MODE1_1);
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1);
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD0     | GPIO_PUPDR_PUPD1);
    GPIOA->PUPDR   |=  (GPIO_PUPDR_PUPD0_0   | GPIO_PUPDR_PUPD1_0);
    GPIOA->AFR[0]  &= ~((0xFU << 0) | (0xFU << 4));
    GPIOA->AFR[0]  |=  ((0x1U << 0) | (0x1U << 4));

    TIM2->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_SMS_3);
    TIM2->SMCR |=  (3U << TIM_SMCR_SMS_Pos);

    TIM2->CCMR1 = 0U;
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;
    TIM2->CCMR1 |= (0x6U << TIM_CCMR1_IC1F_Pos);
    TIM2->CCMR1 |= (0x6U << TIM_CCMR1_IC2F_Pos);

    TIM2->CCER &= ~(TIM_CCER_CC1P  | TIM_CCER_CC2P |
                    TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM2->CCER |=  (TIM_CCER_CC1E  | TIM_CCER_CC2E);

    TIM2->ARR = 0xFFFFFFFFUL;
    TIM2->CNT = 0U;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SR  &= ~TIM_SR_UIF;
    TIM2->CR1 |= TIM_CR1_CEN;

    Encoder_SetSampleRateHz(ENCODER_DEFAULT_SAMPLE_HZ);
    TIM3_Init_Micros();
    Encoder_ResetVelocityState(Encoder_Micros32(), 0);
}

void Encoder_ResetCount(void)
{
    TIM2->CNT = 0U;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;

    Encoder_ResetVelocityState(Encoder_Micros32(), 0);
}

int32_t Encoder_GetCount(void)
{
    return (int32_t)TIM2->CNT;
}

float Encoder_GetRevolutions(void)
{
    return (float)Encoder_GetCount() / (float)COUNTS_PER_REV;
}

float Encoder_GetDegrees(void)
{
    return Encoder_GetRevolutions() * 360.0f;
}

static uint8_t Encoder_RpmWindowShouldUpdate(uint32_t elapsed_us, int32_t delta_counts)
{
    if (elapsed_us < ENCODER_RPM_MIN_WINDOW_US)
    {
        return 0U;
    }

    if (Encoder_AbsI32(delta_counts) >= ENCODER_RPM_MIN_UPDATE_COUNTS)
    {
        return 1U;
    }

    if (elapsed_us >= ENCODER_RPM_MAX_WINDOW_US)
    {
        return 1U;
    }

    return 0U;
}

static float Encoder_RpmFromWindow(int32_t delta_counts, uint32_t elapsed_us)
{
    if (elapsed_us == 0U)
    {
        return rpm_raw_hold;
    }

    float rpm = ((float)delta_counts * 60.0f * 1000000.0f) /
                ((float)ENCODER_RPM_COUNTS_PER_REV * (float)elapsed_us);

    float max_valid = ENCODER_MAX_REPORTED_RPM * ENCODER_RPM_GLITCH_MARGIN;
    return Encoder_ClampF(rpm, -max_valid, max_valid);
}

static void Encoder_FilterRpm(float rpm_raw)
{
    if (((rpm_filtered > ENCODER_ZERO_RPM_DEADBAND) &&
         (rpm_raw < -ENCODER_ZERO_RPM_DEADBAND)) ||
        ((rpm_filtered < -ENCODER_ZERO_RPM_DEADBAND) &&
         (rpm_raw > ENCODER_ZERO_RPM_DEADBAND)))
    {
        if (Encoder_AbsF(rpm_raw) < ENCODER_RPM_SIGN_HYSTERESIS)
        {
            rpm_raw = 0.0f;
        }
    }

    if (Encoder_AbsF(rpm_raw) < ENCODER_ZERO_RPM_DEADBAND)
    {
        rpm_raw = 0.0f;
    }

    rpm_raw_hold = rpm_raw;

    float alpha = (Encoder_AbsF(rpm_raw) < ENCODER_RPM_LOW_FILTER_THRESHOLD) ?
                  ENCODER_RPM_FILTER_ALPHA_LOW :
                  ENCODER_RPM_FILTER_ALPHA_HIGH;

    rpm_filtered += alpha * (rpm_raw - rpm_filtered);

    if (Encoder_AbsF(rpm_filtered) < ENCODER_ZERO_RPM_DEADBAND)
    {
        rpm_filtered = 0.0f;
    }
}

float Encoder_GetVelocityRPM(void)
{
    uint32_t now_us = Encoder_Micros32();
    int32_t current_count = Encoder_GetCount();

    if (rpm_estimator_ready == 0U)
    {
        Encoder_ResetVelocityState(now_us, current_count);
        return 0.0f;
    }

    if (current_count != rpm_last_count)
    {
        rpm_last_motion_us = now_us;
        rpm_last_count = current_count;
    }

    uint32_t elapsed_us = now_us - rpm_window_start_us;
    int32_t delta_counts = current_count - rpm_window_start_count;

    if (Encoder_RpmWindowShouldUpdate(elapsed_us, delta_counts) != 0U)
    {
        float rpm_raw = 0.0f;

        if ((delta_counts != 0) || ((now_us - rpm_last_motion_us) < ENCODER_RPM_STALL_TIMEOUT_US))
        {
            rpm_raw = Encoder_RpmFromWindow(delta_counts, elapsed_us);
        }

        Encoder_FilterRpm(rpm_raw);

        rpm_window_start_count = current_count;
        rpm_window_start_us = now_us;
    }

    return rpm_filtered;
}

float Encoder_GetVelocityCPS(void)
{
    uint32_t now_us = Encoder_Micros32();
    int32_t current_count = Encoder_GetCount();

    if (cps_ready == 0U)
    {
        cps_prev_count = current_count;
        cps_prev_us = now_us;
        cps_ready = 1U;
        return 0.0f;
    }

    uint32_t elapsed_us = now_us - cps_prev_us;
    int32_t delta = current_count - cps_prev_count;

    cps_prev_count = current_count;
    cps_prev_us = now_us;

    if (elapsed_us == 0U)
    {
        return 0.0f;
    }

    return ((float)delta * 1000000.0f) / (float)elapsed_us;
}
