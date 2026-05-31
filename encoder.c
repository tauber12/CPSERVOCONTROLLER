/*
 *******************************************************************************
 * @file           : encoder.c
 * @brief          : Quadrature encoder driver with M/T velocity estimation
 * project         : EE 329 S'26 AX
 * authors         : alexm
 * version         : 0.3
 * date            : May 30, 2026
 * compiler        : STM32CubeIDE v.1.19.0
 * target          : NUCLEO-L4A6ZG
 * clocks          : 48 MHz MSI
 *
 *******************************************************************************
 * Velocity Estimation — M/T Method
 *
 * Two methods are blended based on speed:
 *
 *   HIGH speed (M-method): count encoder edges per fixed sample period.
 *       RPM = (delta_counts / COUNTS_PER_REV) * LOOP_HZ * 60
 *       Good when delta_counts is large (many counts per sample).
 *       Degrades below ~LOW_RPM_THRESHOLD because delta_counts rounds to 0.
 *
 *   LOW speed (T-method): measure time between successive encoder edges
 *       using TIM3 as a free-running microsecond timer.
 *       RPM = 60,000,000 / (period_us * COUNTS_PER_REV)
 *       Accurate at very low RPM; overflows/saturates at high RPM.
 *
 *   Crossover is now recalculated whenever the velocity loop sample rate is
 *   changed by Control_SetVelocityLoopHz(). The default remains 30 RPM at
 *   5 kHz, and it scales linearly with LOOP_HZ so the M/T transition happens
 *   at the same approximate encoder-counts-per-sample operating point.
 *
 *   A stall timeout (STALL_TIMEOUT_US) forces velocity to 0 if no edges arrive
 *   for longer than the equivalent period of MIN_RPM.
 *
 *******************************************************************************
 * TIM2 — Encoder (PA0=TI1, PA1=TI2), x4 quadrature, 32-bit counter
 * TIM3 — Free-running microsecond timebase for T-method (no IRQ needed)
 *******************************************************************************
 */

#include "encoder.h"

/* ---------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------*/

/* 30 RPM at the default 5 kHz velocity loop. */
#define LOW_RPM_THRESHOLD_AT_DEFAULT_HZ 30.0f

/* Stall detection: if no edge arrives within this many µs, report 0 RPM.
 * 60,000,000 / (COUNTS_PER_REV * MIN_RPM) = period at MIN_RPM
 * At 1 RPM, one count every 60,000,000/4200 approx 14,286 us -> use 20,000 us */
#define STALL_TIMEOUT_US    20000UL

/* TIM3 runs at 1 MHz (1 us per tick) using a prescaler derived from HCLK */
#define TIM3_PRESCALER      (48U - 1U)   // 48 MHz / 48 = 1 MHz

/* ---------------------------------------------------------------------------
 * Module-private state
 * -------------------------------------------------------------------------*/
static volatile uint32_t edge_timestamp_us = 0U;   // TIM3 count at last edge
static volatile uint32_t edge_period_us    = 0U;   // period between last two edges
static volatile int8_t   edge_direction    = 1;    // +1 or -1, sampled at edge

static volatile float encoder_sample_rate_hz = ENCODER_DEFAULT_SAMPLE_HZ;
static volatile float encoder_low_rpm_threshold = LOW_RPM_THRESHOLD_AT_DEFAULT_HZ;

/* ---------------------------------------------------------------------------
 * TIM3 — microsecond free-running timer (no overflow IRQ needed for <71 min)
 * -------------------------------------------------------------------------*/
static void TIM3_Init_Micros(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    TIM3->PSC  = TIM3_PRESCALER;
    TIM3->ARR  = 0xFFFFFFFFUL;   // TIM3 is 16-bit on L4; register truncates safely
    TIM3->CNT  = 0U;
    TIM3->EGR |= TIM_EGR_UG;
    TIM3->SR  &= ~TIM_SR_UIF;
    TIM3->CR1 |= TIM_CR1_CEN;
}

/* Return current microsecond timestamp (wraps every ~65.5 ms on 16-bit TIM3).
 * Wrap-safe subtraction handles overflow automatically for periods < 65 ms,
 * which covers RPM > ~1 RPM per count — fine given stall timeout. */
static inline uint32_t micros(void)
{
    return (uint32_t)TIM3->CNT;
}

/* Called from edge interrupt (EXTI or input capture) to record timing */
void Encoder_RecordEdge(void)
{
    uint32_t now = micros();
    uint32_t period = (uint32_t)(now - edge_timestamp_us);  // handles wrap
    edge_timestamp_us = now;

    /* Only accept periods that imply plausible RPM (filter glitches) */
    if (period > 100UL) {   // ignore edges < 100 us apart (>~142 RPM per count)
        edge_period_us = period;
    }

    /* Capture direction at the moment of edge */
    edge_direction = (TIM2->CR1 & TIM_CR1_DIR) ? -1 : +1;
}

/* ---------------------------------------------------------------------------
 * EXTI0 — rising edge on PA0 for T-method timing
 * Configure PA0 for EXTI in addition to TIM2 AF (both can share the pin
 * in input mode; TIM2 reads the AF while EXTI monitors the digital input).
 * -------------------------------------------------------------------------*/
static void EXTI0_EdgeCapture_Init(void)
{
    /* PA0 clock already enabled by Encoder_Config */
    /* Configure EXTI0 source = GPIOA */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI0_PA;

    /* Rising edge trigger */
    EXTI->RTSR1 |=  EXTI_RTSR1_RT0;
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT0;

    /* Unmask and enable */
    EXTI->IMR1  |=  EXTI_IMR1_IM0;
    NVIC->ISER[0] |= (1U << (EXTI0_IRQn & 0x1FU));
}

void EXTI0_IRQHandler(void)
{
    if ((EXTI->PR1 & EXTI_PR1_PIF0) != 0U) {
        EXTI->PR1 = EXTI_PR1_PIF0;   // clear pending flag
        Encoder_RecordEdge();
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

void Encoder_SetSampleRateHz(float sample_rate_hz)
{
    if (sample_rate_hz < 1.0f)
    {
        sample_rate_hz = 1.0f;
    }

    encoder_sample_rate_hz = sample_rate_hz;
    encoder_low_rpm_threshold = LOW_RPM_THRESHOLD_AT_DEFAULT_HZ *
                                (sample_rate_hz / ENCODER_DEFAULT_SAMPLE_HZ);
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
    /* 1. Enable clocks */
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;

    /* 2. PA0 (TI1), PA1 (TI2) -> AF1 (TIM2), pull-up */
    GPIOA->MODER  &= ~(GPIO_MODER_MODE0    | GPIO_MODER_MODE1);
    GPIOA->MODER  |=  (GPIO_MODER_MODE0_1  | GPIO_MODER_MODE1_1);
    GPIOA->OSPEEDR|=  (GPIO_OSPEEDR_OSPEED0| GPIO_OSPEEDR_OSPEED1);
    GPIOA->PUPDR  &= ~(GPIO_PUPDR_PUPD0    | GPIO_PUPDR_PUPD1);
    GPIOA->PUPDR  |=  (GPIO_PUPDR_PUPD0_0  | GPIO_PUPDR_PUPD1_0);
    GPIOA->AFR[0] &= ~((0xFU << 0) | (0xFU << 4));
    GPIOA->AFR[0] |=  ((0x1U << 0) | (0x1U << 4));

    /* 3. Encoder mode x4 (count on both edges of TI1 and TI2) */
    TIM2->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_SMS_3);
    TIM2->SMCR |=  (3U << TIM_SMCR_SMS_Pos);

    /* 4. TI1->IC1, TI2->IC2, light noise filter */
    TIM2->CCMR1  = 0U;
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;
    TIM2->CCMR1 |= (0x2U << TIM_CCMR1_IC1F_Pos);
    TIM2->CCMR1 |= (0x2U << TIM_CCMR1_IC2F_Pos);

    /* 5. Active-high polarity, enable capture inputs */
    TIM2->CCER &= ~(TIM_CCER_CC1P  | TIM_CCER_CC2P |
                    TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM2->CCER |=  (TIM_CCER_CC1E  | TIM_CCER_CC2E);

    /* 6. Full 32-bit range */
    TIM2->ARR = 0xFFFFFFFFUL;
    TIM2->CNT = 0U;

    /* 7. Generate update to load registers, clear spurious UIF */
    TIM2->EGR |=  TIM_EGR_UG;
    TIM2->SR  &= ~TIM_SR_UIF;

    /* 8. Start encoder counter */
    TIM2->CR1 |= TIM_CR1_CEN;

    /* 9. Start us timebase and edge-capture interrupt */
    Encoder_SetSampleRateHz(ENCODER_DEFAULT_SAMPLE_HZ);
    TIM3_Init_Micros();
    EXTI0_EdgeCapture_Init();
}

/* Raw count — signed 32-bit, wraps at +/-2^31 counts */
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

/*
 * Encoder_GetVelocityRPM — M/T blended velocity estimator
 *
 * Called from TIM5 ISR at Encoder_GetSampleRateHz().
 *
 * M-method (high RPM):
 *   velocity = (delta_counts * LOOP_HZ * 60) / COUNTS_PER_REV
 *
 * T-method (low RPM):
 *   velocity = 60,000,000 / (edge_period_us * COUNTS_PER_REV)
 *   Direction sign comes from TIM2 direction bit captured at the last edge.
 *
 * Stall detection:
 *   If no edge has arrived within STALL_TIMEOUT_US, return 0.
 */
float Encoder_GetVelocityRPM(void)
{
    /* --- M-method: count delta over one control period --- */
    static int32_t prev_count = 0;
    int32_t current_count = Encoder_GetCount();
    int32_t delta         = current_count - prev_count;
    prev_count            = current_count;

    float loop_hz = encoder_sample_rate_hz;
    if (loop_hz < 1.0f)
    {
        loop_hz = ENCODER_DEFAULT_SAMPLE_HZ;
    }

    float rpm_m = (float)delta * loop_hz * 60.0f / (float)COUNTS_PER_REV;

    /* --- Stall check: time since last edge --- */
    uint32_t now             = micros();
    uint32_t time_since_edge = (uint32_t)(now - edge_timestamp_us);

    if (time_since_edge > STALL_TIMEOUT_US) {
        prev_count = current_count;   // reset so M-method doesn't jump on restart
        edge_period_us = 0U;
        return 0.0f;
    }

    /* --- T-method: period between last two edges --- */
    uint32_t period = edge_period_us;   // snapshot volatile once

    float rpm_t = 0.0f;
    if (period > 0U) {
        rpm_t = (60.0f * 1000000.0f) / ((float)period * (float)COUNTS_PER_REV);
        rpm_t *= (float)edge_direction;
    }

    /* --- Blend: use T-method below threshold, M-method above --- */
    float abs_rpm_m = rpm_m < 0.0f ? -rpm_m : rpm_m;

    if (abs_rpm_m < encoder_low_rpm_threshold) {
        return rpm_t;   // T-method: accurate at low speed
    } else {
        return rpm_m;   // M-method: accurate at high speed
    }
}

/*
 * Encoder_GetVelocityCPS — counts per second (M-method only)
 * Useful for inner-loop diagnostics.
 */
float Encoder_GetVelocityCPS(void)
{
    static int32_t prev_count = 0;
    int32_t current_count     = Encoder_GetCount();
    int32_t delta             = current_count - prev_count;
    prev_count                = current_count;
    return (float)delta * encoder_sample_rate_hz;
}
