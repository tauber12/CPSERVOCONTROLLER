/*
 *******************************************************************************
 * @file           : encoder.c
 * @brief          : Quadrature encoder position + filtered T-method RPM driver
 *******************************************************************************
 *
 * TIM2 counts PA0/PA1 in x4 encoder mode for position.
 * TIM3 provides a 1 MHz microsecond timebase for Channel-A rising-edge timing.
 *
 * Velocity estimation is T-method only:
 *      RPM = 60,000,000 / (period_us * ENCODER_T_EDGES_PER_REV)
 *
 * The code extends the 16-bit TIM3 timer in software, rejects stationary/noisy
 * edge events, filters edge period, and dynamically zeros stale RPM after missed
 * expected edges.
 *******************************************************************************
 */

#include "encoder.h"

/* -------------------------------------------------------------------------- */
/* Module-private state                                                       */
/* -------------------------------------------------------------------------- */

static volatile uint32_t tim3_high_us = 0U;

static volatile uint8_t  edge_capture_started = 0U;
static volatile uint8_t  edge_period_valid = 0U;
static volatile uint32_t edge_timestamp_us = 0U;
static volatile uint32_t last_valid_edge_us = 0U;
static volatile int32_t  last_edge_count = 0;

static volatile uint32_t edge_period_hist[3] = {0U, 0U, 0U};
static volatile uint8_t  edge_period_hist_count = 0U;
static volatile float    filtered_period_us = 0.0f;

static volatile int8_t   reported_direction = 1;
static volatile int8_t   pending_direction = 1;
static volatile uint8_t  pending_direction_count = 0U;

static volatile float    encoder_reported_rpm = 0.0f;
static volatile uint32_t rpm_update_timestamp_us = 0U;

static volatile float encoder_sample_rate_hz = ENCODER_DEFAULT_SAMPLE_HZ;

/* -------------------------------------------------------------------------- */
/* Small helpers                                                              */
/* -------------------------------------------------------------------------- */

static float Encoder_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint32_t Encoder_AbsI32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

static uint32_t Encoder_MaxU32(uint32_t a, uint32_t b)
{
    return (a > b) ? a : b;
}

static uint32_t Encoder_MinU32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

static uint32_t Encoder_Median3(uint32_t a, uint32_t b, uint32_t c)
{
    if (a > b)
    {
        uint32_t t = a;
        a = b;
        b = t;
    }

    if (b > c)
    {
        uint32_t t = b;
        b = c;
        c = t;
    }

    if (a > b)
    {
        uint32_t t = a;
        a = b;
        b = t;
    }

    return b;
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

static uint32_t micros(void)
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

    TIM3->CR1 = 0U;
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

static void Encoder_ResetVelocityState(void)
{
    uint32_t now = micros();

    edge_capture_started = 0U;
    edge_period_valid = 0U;
    edge_timestamp_us = now;
    last_valid_edge_us = now;
    last_edge_count = Encoder_GetCount();

    edge_period_hist[0] = 0U;
    edge_period_hist[1] = 0U;
    edge_period_hist[2] = 0U;
    edge_period_hist_count = 0U;
    filtered_period_us = 0.0f;

    reported_direction = 1;
    pending_direction = 1;
    pending_direction_count = 0U;

    encoder_reported_rpm = 0.0f;
    rpm_update_timestamp_us = now;
}

static void Encoder_UpdateDirection(int32_t count_delta, uint8_t first_valid_period)
{
    int8_t new_direction = (count_delta < 0) ? -1 : 1;

    if (first_valid_period != 0U)
    {
        reported_direction = new_direction;
        pending_direction = new_direction;
        pending_direction_count = 0U;
        return;
    }

    if (new_direction == reported_direction)
    {
        pending_direction = new_direction;
        pending_direction_count = 0U;
        return;
    }

    if (new_direction == pending_direction)
    {
        if (pending_direction_count < 255U)
        {
            pending_direction_count++;
        }
    }
    else
    {
        pending_direction = new_direction;
        pending_direction_count = 1U;
    }

    if (pending_direction_count >= ENCODER_T_DIRECTION_CONFIRM_EDGES)
    {
        reported_direction = pending_direction;
        pending_direction_count = 0U;
    }
}

static uint32_t Encoder_FilterPeriod(uint32_t period_us)
{
    uint32_t period_used = period_us;

#if ENCODER_T_ENABLE_MEDIAN3
    edge_period_hist[2] = edge_period_hist[1];
    edge_period_hist[1] = edge_period_hist[0];
    edge_period_hist[0] = period_us;

    if (edge_period_hist_count < 3U)
    {
        edge_period_hist_count++;
    }

    if (edge_period_hist_count >= 3U)
    {
        period_used = Encoder_Median3(edge_period_hist[0],
                                      edge_period_hist[1],
                                      edge_period_hist[2]);
    }
#endif

    if ((filtered_period_us <= 0.0f) || (edge_period_valid == 0U))
    {
        filtered_period_us = (float)period_used;
    }
    else
    {
        filtered_period_us = ((1.0f - ENCODER_T_PERIOD_FILTER_ALPHA) * filtered_period_us) +
                             (ENCODER_T_PERIOD_FILTER_ALPHA * (float)period_used);
    }

    if (filtered_period_us < 1.0f)
    {
        filtered_period_us = 1.0f;
    }

    return (uint32_t)(filtered_period_us + 0.5f);
}

static float Encoder_ApplySlewLimit(float target_rpm, uint32_t now_us)
{
    uint32_t dt_us = now_us - rpm_update_timestamp_us;

    if ((rpm_update_timestamp_us == 0U) || (dt_us == 0U))
    {
        rpm_update_timestamp_us = now_us;
        return target_rpm;
    }

    float max_step = ENCODER_RPM_SLEW_LIMIT_RPM_PER_S * ((float)dt_us / 1000000.0f);
    float delta = target_rpm - encoder_reported_rpm;

    if (delta > max_step)
    {
        target_rpm = encoder_reported_rpm + max_step;
    }
    else if (delta < -max_step)
    {
        target_rpm = encoder_reported_rpm - max_step;
    }

    rpm_update_timestamp_us = now_us;
    return target_rpm;
}

static uint32_t Encoder_GetDynamicStationaryTimeoutUs(void)
{
    uint32_t period_us = (filtered_period_us > 1.0f) ?
                         (uint32_t)(filtered_period_us + 0.5f) :
                         ENCODER_STATIONARY_MIN_TIMEOUT_US;

    uint32_t timeout_us = (uint32_t)(((float)period_us * ENCODER_STATIONARY_TIMEOUT_PERIODS) + 0.5f);

    timeout_us = Encoder_MaxU32(timeout_us, ENCODER_STATIONARY_MIN_TIMEOUT_US);
    timeout_us = Encoder_MinU32(timeout_us, ENCODER_STALL_TIMEOUT_US);

    return timeout_us;
}

/* -------------------------------------------------------------------------- */
/* T-method edge capture on PA0 rising edge                                   */
/* -------------------------------------------------------------------------- */

void Encoder_RecordEdge(void)
{
    uint32_t now = micros();
    int32_t current_count = Encoder_GetCount();

    if (edge_capture_started == 0U)
    {
        edge_capture_started = 1U;
        edge_timestamp_us = now;
        last_valid_edge_us = now;
        last_edge_count = current_count;
        return;
    }

    uint32_t period = now - edge_timestamp_us;
    int32_t count_delta = current_count - last_edge_count;
    uint32_t abs_count_delta = Encoder_AbsI32(count_delta);

    /* Reject double-triggered EXTI events and impossible speeds.  Do not update
     * edge_timestamp_us here; otherwise a noisy edge shortens the next valid
     * measured period and creates an RPM spike. */
    if (period < ENCODER_T_MIN_PERIOD_US)
    {
        return;
    }

    /* PA0-rising to PA0-rising should be about 4 quadrature counts.  Reject tiny
     * stationary jitter and large irregular deltas. */
    if ((abs_count_delta < (uint32_t)ENCODER_T_MIN_EDGE_COUNT_DELTA) ||
        (abs_count_delta > (uint32_t)ENCODER_T_MAX_EDGE_COUNT_DELTA))
    {
        return;
    }

    uint8_t first_valid_period = (edge_period_valid == 0U) ? 1U : 0U;

    edge_timestamp_us = now;
    last_valid_edge_us = now;
    last_edge_count = current_count;

    Encoder_UpdateDirection(count_delta, first_valid_period);

    uint32_t filtered_period = Encoder_FilterPeriod(period);

    float rpm = 60000000.0f /
                ((float)filtered_period * (float)ENCODER_T_EDGES_PER_REV);
    rpm *= (float)reported_direction;

    if (Encoder_AbsF(rpm) < ENCODER_ZERO_RPM_DEADBAND)
    {
        rpm = 0.0f;
    }

    encoder_reported_rpm = Encoder_ApplySlewLimit(rpm, now);
    edge_period_valid = 1U;
}

static void EXTI0_EdgeCapture_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

    EXTI->RTSR1 |= EXTI_RTSR1_RT0;
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT0;
    EXTI->PR1 = EXTI_PR1_PIF0;
    EXTI->IMR1 |= EXTI_IMR1_IM0;

    NVIC->ISER[0] |= (1U << (EXTI0_IRQn & 0x1FU));
}

void EXTI0_IRQHandler(void)
{
    if ((EXTI->PR1 & EXTI_PR1_PIF0) != 0U)
    {
        EXTI->PR1 = EXTI_PR1_PIF0;
        Encoder_RecordEdge();
    }
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
}

float Encoder_GetSampleRateHz(void)
{
    return encoder_sample_rate_hz;
}

float Encoder_GetLowRpmThreshold(void)
{
    /* Compatibility with older M/T code.  T-method is now used across the full
     * operating range, so there is no crossover threshold. */
    return 0.0f;
}

void Encoder_Config(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
    GPIOA->MODER |= (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0);
    GPIOA->AFR[0] &= ~((0xFU << 0) | (0xFU << 4));
    GPIOA->AFR[0] |= ((0x1U << 0) | (0x1U << 4));

    TIM2->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_SMS_3);
    TIM2->SMCR |= (3U << TIM_SMCR_SMS_Pos);

    TIM2->CCMR1 = 0U;
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;
    TIM2->CCMR1 |= (0x6U << TIM_CCMR1_IC1F_Pos);
    TIM2->CCMR1 |= (0x6U << TIM_CCMR1_IC2F_Pos);

    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P |
                    TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);

    TIM2->ARR = 0xFFFFFFFFUL;
    TIM2->CNT = 0U;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->CR1 |= TIM_CR1_CEN;

    Encoder_SetSampleRateHz(ENCODER_DEFAULT_SAMPLE_HZ);
    TIM3_Init_Micros();
    Encoder_ResetVelocityState();
    EXTI0_EdgeCapture_Init();
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
    if (edge_period_valid == 0U)
    {
        return 0.0f;
    }

    uint32_t now = micros();
    uint32_t time_since_edge = now - last_valid_edge_us;
    uint32_t timeout_us = Encoder_GetDynamicStationaryTimeoutUs();

    if (time_since_edge > timeout_us)
    {
        encoder_reported_rpm = 0.0f;
        return 0.0f;
    }

    float rpm = encoder_reported_rpm;

    /* If the next expected edge is late, the motor must be slower than the last
     * measured period implied.  Decay the reported RPM instead of holding a
     * stale nonzero value until the full timeout expires. */
    if (filtered_period_us > 1.0f)
    {
        uint32_t expected_period_us = (uint32_t)(filtered_period_us + 0.5f);

        if (time_since_edge > expected_period_us)
        {
            float elapsed_rpm = 60000000.0f /
                                ((float)time_since_edge * (float)ENCODER_T_EDGES_PER_REV);
            elapsed_rpm *= (float)reported_direction;

            if (Encoder_AbsF(elapsed_rpm) < Encoder_AbsF(rpm))
            {
                rpm = elapsed_rpm;
            }
        }
    }

    if (Encoder_AbsF(rpm) < ENCODER_ZERO_RPM_DEADBAND)
    {
        rpm = 0.0f;
    }

    return rpm;
}

float Encoder_GetVelocityCPS(void)
{
    return (Encoder_GetVelocityRPM() * (float)ENCODER_RPM_COUNTS_PER_REV) / 60.0f;
}
