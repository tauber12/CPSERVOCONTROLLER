/*
 *******************************************************************************
 * @file           : encoder.c
 * @brief          : Quadrature encoder driver with robust filtered RPM output
 *******************************************************************************
 *
 * TIM2 counts PA0/PA1 in x4 encoder mode. TIM3 is a 1 MHz free-running timebase
 * extended in software so the low-speed T-method works below the 16-bit timer
 * wrap period. EXTI0 timestamps PA0 rising edges for the T-method.
 *
 * M-method: delta x4 counts over a fixed velocity-loop sample window.
 * T-method: period between PA0 rising edges. Since EXTI sees one rising edge per
 * encoder pulse, T-method divides by ENCODER_T_EDGES_PER_REV, not x4 counts.
 *******************************************************************************
 */

#include "encoder.h"

#define ENCODER_TIM3_TICK_HZ             1000000UL
#define ENCODER_TIM3_PRESCALER          ((48000000UL / ENCODER_TIM3_TICK_HZ) - 1UL)
#define ENCODER_TIM3_PERIOD_US          65536UL
#define ENCODER_TIM3_MASK               0xFFFFUL
#define ENCODER_BLEND_LOW_FACTOR        0.75f
#define ENCODER_BLEND_HIGH_FACTOR       1.25f

static volatile uint32_t tim3_high_us = 0U;

static volatile uint32_t edge_timestamp_us = 0U;
static volatile float    edge_period_filtered_us = 0.0f;
static volatile int8_t   edge_direction = 1;
static volatile uint8_t  edge_seen = 0U;
static volatile uint8_t  edge_period_valid = 0U;

static volatile float encoder_sample_rate_hz = ENCODER_DEFAULT_SAMPLE_HZ;
static volatile float encoder_low_rpm_threshold =
    (ENCODER_M_MIN_COUNTS_PER_SAMPLE * ENCODER_DEFAULT_SAMPLE_HZ * 60.0f) /
    (float)COUNTS_PER_REV;

static int32_t velocity_prev_count = 0;
static int32_t cps_prev_count = 0;
static int32_t m_delta_accum = 0;
static uint16_t m_sample_count = 0U;
static float rpm_m_window = 0.0f;
static float filtered_rpm = 0.0f;

static float Encoder_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
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

static uint32_t micros32(void)
{
    TIM3_ServiceOverflow();

    uint32_t high = tim3_high_us;
    uint32_t low = ((uint32_t)TIM3->CNT) & ENCODER_TIM3_MASK;

    /* If an overflow happened just after servicing, account for it without
     * waiting for the IRQ. This keeps period measurements monotonic even inside
     * other ISRs. */
    if ((TIM3->SR & TIM_SR_UIF) != 0U)
    {
        high += ENCODER_TIM3_PERIOD_US;
        low = ((uint32_t)TIM3->CNT) & ENCODER_TIM3_MASK;
    }

    return high + low;
}

static uint32_t Encoder_MinValidPeriodUs(void)
{
    float max_rpm = ENCODER_MAX_OUTPUT_RPM;
    if (max_rpm < 1.0f)
    {
        max_rpm = 1.0f;
    }

    float min_period = 60000000.0f / (max_rpm * (float)ENCODER_T_EDGES_PER_REV);

    /* Allow 2x rated-speed margin while still rejecting short noise spikes. */
    min_period *= 0.5f;
    if (min_period < 2.0f)
    {
        min_period = 2.0f;
    }

    return (uint32_t)(min_period + 0.5f);
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

void Encoder_RecordEdge(void)
{
    uint32_t now = micros32();
    uint32_t period = now - edge_timestamp_us;
    uint32_t min_period = Encoder_MinValidPeriodUs();

    if ((edge_seen != 0U) && (period < min_period))
    {
        return;
    }

    if (edge_seen != 0U)
    {
        if (edge_period_valid == 0U)
        {
            edge_period_filtered_us = (float)period;
        }
        else
        {
            edge_period_filtered_us += ENCODER_T_PERIOD_FILTER_ALPHA *
                ((float)period - edge_period_filtered_us);
        }
        edge_period_valid = 1U;
    }
    else
    {
        edge_period_valid = 0U;
        edge_seen = 1U;
    }

    edge_timestamp_us = now;
    edge_direction = (TIM2->CR1 & TIM_CR1_DIR) ? -1 : +1;
}

static void EXTI0_EdgeCapture_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI0_PA;

    EXTI->RTSR1 |=  EXTI_RTSR1_RT0;
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT0;

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

void Encoder_SetSampleRateHz(float sample_rate_hz)
{
    if (sample_rate_hz < 1.0f)
    {
        sample_rate_hz = 1.0f;
    }

    encoder_sample_rate_hz = sample_rate_hz;
    encoder_low_rpm_threshold = (ENCODER_M_MIN_COUNTS_PER_SAMPLE * sample_rate_hz * 60.0f) /
                                (float)COUNTS_PER_REV;
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

    GPIOA->MODER  &= ~(GPIO_MODER_MODE0    | GPIO_MODER_MODE1);
    GPIOA->MODER  |=  (GPIO_MODER_MODE0_1  | GPIO_MODER_MODE1_1);
    GPIOA->OSPEEDR|=  (GPIO_OSPEEDR_OSPEED0| GPIO_OSPEEDR_OSPEED1);
    GPIOA->PUPDR  &= ~(GPIO_PUPDR_PUPD0    | GPIO_PUPDR_PUPD1);
    GPIOA->PUPDR  |=  (GPIO_PUPDR_PUPD0_0  | GPIO_PUPDR_PUPD1_0);
    GPIOA->AFR[0] &= ~((0xFU << 0) | (0xFU << 4));
    GPIOA->AFR[0] |=  ((0x1U << 0) | (0x1U << 4));

    TIM2->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_SMS_3);
    TIM2->SMCR |=  (3U << TIM_SMCR_SMS_Pos);

    TIM2->CCMR1  = 0U;
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

    edge_timestamp_us = 0U;
    edge_period_filtered_us = 0.0f;
    edge_direction = 1;
    edge_seen = 0U;
    edge_period_valid = 0U;
    velocity_prev_count = 0;
    cps_prev_count = 0;
    m_delta_accum = 0;
    m_sample_count = 0U;
    rpm_m_window = 0.0f;
    filtered_rpm = 0.0f;

    Encoder_SetSampleRateHz(ENCODER_DEFAULT_SAMPLE_HZ);
    TIM3_Init_Micros();
    EXTI0_EdgeCapture_Init();
}

void Encoder_ResetCount(void)
{
    TIM2->CNT = 0U;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;

    velocity_prev_count = 0;
    cps_prev_count = 0;
    m_delta_accum = 0;
    m_sample_count = 0U;
    rpm_m_window = 0.0f;
    filtered_rpm = 0.0f;

    edge_timestamp_us = micros32();
    edge_period_filtered_us = 0.0f;
    edge_direction = 1;
    edge_seen = 0U;
    edge_period_valid = 0U;
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

    float loop_hz = encoder_sample_rate_hz;
    if (loop_hz < 1.0f)
    {
        loop_hz = ENCODER_DEFAULT_SAMPLE_HZ;
    }

    m_delta_accum += delta;
    m_sample_count++;
    if (m_sample_count >= ENCODER_M_WINDOW_SAMPLES)
    {
        rpm_m_window = ((float)m_delta_accum * loop_hz * 60.0f) /
                       ((float)COUNTS_PER_REV * (float)m_sample_count);
        m_delta_accum = 0;
        m_sample_count = 0U;
    }

    uint32_t now = micros32();
    uint32_t last_edge = edge_timestamp_us;
    float period_us = edge_period_filtered_us;
    int8_t direction = edge_direction;
    uint8_t period_valid = edge_period_valid;
    uint8_t have_seen_edge = edge_seen;
    uint32_t time_since_edge = now - last_edge;

    if ((have_seen_edge == 0U) || (time_since_edge > ENCODER_STALL_TIMEOUT_US))
    {
        edge_seen = 0U;
        edge_period_valid = 0U;
        edge_period_filtered_us = 0.0f;
        filtered_rpm += ENCODER_RPM_FILTER_ALPHA_HIGH * (0.0f - filtered_rpm);
        if (Encoder_AbsF(filtered_rpm) < 0.05f)
        {
            filtered_rpm = 0.0f;
        }
        return filtered_rpm;
    }

    float rpm_t = 0.0f;
    if ((period_valid != 0U) && (period_us > 0.5f))
    {
        rpm_t = (60.0f * 1000000.0f) /
                (period_us * (float)ENCODER_T_EDGES_PER_REV);

        if (delta > 0)
        {
            direction = 1;
        }
        else if (delta < 0)
        {
            direction = -1;
        }

        rpm_t *= (float)direction;
    }

    float abs_m = Encoder_AbsF(rpm_m_window);
    float threshold = encoder_low_rpm_threshold;
    float low = threshold * ENCODER_BLEND_LOW_FACTOR;
    float high = threshold * ENCODER_BLEND_HIGH_FACTOR;
    float rpm_raw;

    if ((period_valid == 0U) || (threshold <= 0.0f))
    {
        rpm_raw = rpm_m_window;
    }
    else if (abs_m <= low)
    {
        rpm_raw = rpm_t;
    }
    else if (abs_m >= high)
    {
        rpm_raw = rpm_m_window;
    }
    else
    {
        float blend = (abs_m - low) / (high - low);
        blend = Encoder_ClampF(blend, 0.0f, 1.0f);
        rpm_raw = (rpm_t * (1.0f - blend)) + (rpm_m_window * blend);
    }

    float alpha = (Encoder_AbsF(rpm_raw) < threshold) ?
                  ENCODER_RPM_FILTER_ALPHA_LOW :
                  ENCODER_RPM_FILTER_ALPHA_HIGH;

    filtered_rpm += alpha * (rpm_raw - filtered_rpm);
    return filtered_rpm;
}

float Encoder_GetVelocityCPS(void)
{
    int32_t current_count = Encoder_GetCount();
    int32_t delta = current_count - cps_prev_count;
    cps_prev_count = current_count;
    return (float)delta * encoder_sample_rate_hz;
}
