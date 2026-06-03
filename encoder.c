/*
 *******************************************************************************
 * @file           : encoder.c
 * @brief          : Quadrature encoder driver with M-method velocity estimate
 *******************************************************************************
 *
 * TIM2 counts PA0/PA1 in x4 encoder mode.
 * Velocity is calculated only from count delta over a fixed sample window:
 *
 *     RPM = delta_counts * sample_hz * 60 / (counts_per_rev * samples)
 *
 * No TIM3 timebase, EXTI edge timing, or T-method logic is used.
 *******************************************************************************
 */

#include "encoder.h"

/* -------------------------------------------------------------------------- */
/* Module-private state                                                       */
/* -------------------------------------------------------------------------- */

static volatile float encoder_sample_rate_hz = ENCODER_DEFAULT_SAMPLE_HZ;

static int32_t velocity_prev_count = 0;
static int32_t velocity_accum_counts = 0;
static uint32_t velocity_accum_samples = 0U;
static float velocity_rpm_latest = 0.0f;

static int32_t cps_prev_count = 0;

/* -------------------------------------------------------------------------- */
/* Small helpers                                                              */
/* -------------------------------------------------------------------------- */

static float Encoder_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void Encoder_ResetVelocityState(void)
{
    int32_t count = Encoder_GetCount();

    velocity_prev_count = count;
    velocity_accum_counts = 0;
    velocity_accum_samples = 0U;
    velocity_rpm_latest = 0.0f;

    cps_prev_count = count;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void Encoder_SetSampleRateHz(float sample_rate_hz)
{
    if (sample_rate_hz < 1.0f)
    {
        sample_rate_hz = 1.0f;
    }

    encoder_sample_rate_hz = sample_rate_hz;
    Encoder_ResetVelocityState();
}

float Encoder_GetSampleRateHz(void)
{
    return encoder_sample_rate_hz;
}

float Encoder_GetLowRpmThreshold(void)
{
    /* Compatibility function retained for older diagnostic UI code.
     * There is no M/T crossover threshold because this driver is M-method only. */
    return 0.0f;
}

void Encoder_Config(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    /* PA0 = TIM2_CH1, PA1 = TIM2_CH2, AF1, pull-up. */
    GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
    GPIOA->MODER |= (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0);
    GPIOA->AFR[0] &= ~((0xFU << 0) | (0xFU << 4));
    GPIOA->AFR[0] |= ((0x1U << 0) | (0x1U << 4));

    /* TIM2 encoder mode 3: count on both TI1 and TI2 edges. */
    TIM2->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_SMS_3);
    TIM2->SMCR |= (3U << TIM_SMCR_SMS_Pos);

    TIM2->CCMR1 = 0U;
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;
    TIM2->CCMR1 |= (ENCODER_TIM2_INPUT_FILTER << TIM_CCMR1_IC1F_Pos);
    TIM2->CCMR1 |= (ENCODER_TIM2_INPUT_FILTER << TIM_CCMR1_IC2F_Pos);

    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P |
                    TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);

    TIM2->ARR = 0xFFFFFFFFUL;
    TIM2->CNT = 0U;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->CR1 |= TIM_CR1_CEN;

    Encoder_SetSampleRateHz(ENCODER_DEFAULT_SAMPLE_HZ);
    Encoder_ResetVelocityState();
}

void Encoder_RecordEdge(void)
{
    /* Retained as a no-op so older startup/interrupt code can still link.
     * This simplified encoder driver does not use EXTI edge timing. */
}

void Encoder_ResetCount(void)
{
    TIM2->CNT = 0U;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;

    Encoder_ResetVelocityState();
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

float Encoder_GetVelocityRPM(void)
{
    int32_t current_count = Encoder_GetCount();
    int32_t delta = current_count - velocity_prev_count;
    velocity_prev_count = current_count;

    velocity_accum_counts += delta;
    velocity_accum_samples++;

    if (velocity_accum_samples >= ENCODER_RPM_WINDOW_SAMPLES)
    {
        float rpm = ((float)velocity_accum_counts * encoder_sample_rate_hz * 60.0f) /
                    ((float)ENCODER_RPM_COUNTS_PER_REV * (float)velocity_accum_samples);

        if (Encoder_AbsF(rpm) < ENCODER_RPM_DEADBAND)
        {
            rpm = 0.0f;
        }

        velocity_rpm_latest = rpm;
        velocity_accum_counts = 0;
        velocity_accum_samples = 0U;
    }

    return velocity_rpm_latest;
}

float Encoder_GetVelocityCPS(void)
{
    int32_t current_count = Encoder_GetCount();
    int32_t delta = current_count - cps_prev_count;
    cps_prev_count = current_count;

    return (float)delta * encoder_sample_rate_hz;
}
