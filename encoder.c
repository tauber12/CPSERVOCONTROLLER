/*
 *******************************************************************************
 * @file           : encoder.c
 * @brief          : X
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.1
 * date            : May 14, 2026
 * compiler        : STM32CubeIDE v.1.19.0 Build: 14980_20230301_1550 (UTC)
 * target          : NUCLEO-L4A6ZG
 * clocks          : 4 MHz MSI to AHB2
 * @attention      : (c) 2026 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * Description: X
 *
 *******************************************************************************
 * GPIO Wiring
 * |   Component    | GPIO Identifier | Connector Location | Config
 *-----------------------------------------------------------------------------
 * | LCD - DB4 - 11 | PC0             | CN9-3              | OUT
 *******************************************************************************
 * Version History
 *  Ver.|   Date   |  Description
 *  ---------------------------------------------------------------------------
 *      |          |
 *******************************************************************************
 *
 * Header format adapted from [Code Appendix by Kevin Vo] pg 5
 */
/* encoder.c */

#include "encoder.h"

#define PPR             7
#define GEAR_RATIO      150
#define COUNTS_PER_REV  (PPR * GEAR_RATIO * 4) // 4200
#define TIM3_FREQ_HZ    48000000UL

static volatile uint32_t pulse_period      = 0;
static volatile uint32_t last_capture_time = 0;

void Encoder_Config(void)
{
    /* 1. Enable clocks */
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;

    /* 2. PA0 (TI1), PA1 (TI2) → AF1, pull-up */
    GPIOA->MODER  &= ~(GPIO_MODER_MODE0    | GPIO_MODER_MODE1);
    GPIOA->MODER  |=  (GPIO_MODER_MODE0_1  | GPIO_MODER_MODE1_1);
    GPIOA->OSPEEDR|=  (GPIO_OSPEEDR_OSPEED0| GPIO_OSPEEDR_OSPEED1);
    GPIOA->PUPDR  &= ~(GPIO_PUPDR_PUPD0    | GPIO_PUPDR_PUPD1);
    GPIOA->PUPDR  |=  (GPIO_PUPDR_PUPD0_0  | GPIO_PUPDR_PUPD1_0);
    GPIOA->AFR[0] &= ~((0xF << 0) | (0xF << 4));
    GPIOA->AFR[0] |=  ((0x1 << 0) | (0x1 << 4));

    /* 3. Encoder mode: count on both TI1 and TI2 edges (x4) */
    TIM2->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_SMS_3);
    TIM2->SMCR |=  (3 << TIM_SMCR_SMS_Pos);

    /* 4. TI1→IC1, TI2→IC2, light filter (20 MHz cutoff) */
    TIM2->CCMR1  = 0;
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;
    TIM2->CCMR1 |= (0x2 << TIM_CCMR1_IC1F_Pos);
    TIM2->CCMR1 |= (0x2 << TIM_CCMR1_IC2F_Pos);

    /* 5. Active high polarity, enable inputs */
    TIM2->CCER &= ~(TIM_CCER_CC1P  | TIM_CCER_CC2P |
                    TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM2->CCER |=  (TIM_CCER_CC1E  | TIM_CCER_CC2E);

    /* 6. Full 32-bit range, start at 0 */
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CNT = 0;

    /* 7. Configure TIM2 TRGO to pulse on each encoder count */
    TIM2->CR2 &= ~TIM_CR2_MMS;
    TIM2->CR2 |=  (0b111 << TIM_CR2_MMS_Pos);   // MMS = encoder clock out

    /* 8. Initialise registers, clear any spurious UIF */
    TIM2->EGR |=  TIM_EGR_UG;
    TIM2->SR  &= ~TIM_SR_UIF;

    /* 9. Start counter */
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM3_InputCapture_Init(void)
{
    /* 1. Enable TIM3 clock */
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

    /* 2. Free running counter, full 32-bit range, 48 MHz tick */
    TIM3->PSC = 0;
    TIM3->ARR = 0xFFFFFFFF;
    TIM3->CNT = 0;

    /* 3. CH1 capture on TRC (internal trigger source) */
    TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM3->CCMR1 |=  TIM_CCMR1_CC1S_1;           // CC1S = 10 = TRC

    /* 4. Select ITR1 as trigger (TS = 001 = TIM2 per Table 203) */
    TIM3->SMCR &= ~TIM_SMCR_TS;
    TIM3->SMCR |=  (0b001 << TIM_SMCR_TS_Pos);

    /* 5. Slave mode: trigger mode — each TIM2 count triggers a capture */
    TIM3->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_SMS_3);
    TIM3->SMCR |=  (0b0110 << TIM_SMCR_SMS_Pos);

    /* 6. Enable input capture on CH1 */
    TIM3->CCER |= TIM_CCER_CC1E;

    /* 7. Enable capture interrupt */
    TIM3->SR   &= ~TIM_SR_CC1IF;
    TIM3->DIER |=  TIM_DIER_CC1IE;

    NVIC_SetPriority(TIM3_IRQn, 3);
    NVIC_EnableIRQ(TIM3_IRQn);

    /* 8. Initialise and start */
    TIM3->EGR |=  TIM_EGR_UG;
    TIM3->SR  &= ~TIM_SR_UIF;
    TIM3->CR1 |=  TIM_CR1_CEN;
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_CC1IF)
    {
        TIM3->SR &= ~TIM_SR_CC1IF;

        static uint32_t prev_capture = 0;
        uint32_t current_capture = TIM3->CCR1;        // read clears flag too
        pulse_period             = current_capture - prev_capture;
        prev_capture             = current_capture;
        last_capture_time        = current_capture;
    }
}

/* Input capture velocity — precise at low speed */
float Encoder_GetVelocityIC_RPM(void)
{
    /* if no pulse for >100ms assume stopped */
    if ((TIM3->CNT - last_capture_time) > (TIM3_FREQ_HZ / 10)) return 0.0f;
    if (pulse_period == 0) return 0.0f;

    float period_sec = (float)pulse_period / TIM3_FREQ_HZ;
    float direction  = (TIM2->CR1 & TIM_CR1_DIR) ? -1.0f : 1.0f;
    return (1.0f / period_sec) * 60.0f / COUNTS_PER_REV * direction;
}

/* Differentiation velocity — call at fixed control loop rate */
int32_t Encoder_GetCount(void)
{
    return (int32_t)TIM2->CNT;
}

float Encoder_GetRevolutions(void)
{
    return (float)Encoder_GetCount() / COUNTS_PER_REV;
}

float Encoder_GetDegrees(void)
{
    return Encoder_GetRevolutions() * 360.0f;
}

float Encoder_GetVelocityCPS(void)
{
    static int32_t prev_count = 0;
    int32_t current_count     = Encoder_GetCount();
    int32_t delta             = current_count - prev_count;
    prev_count                = current_count;
    return (float)delta * CONTROL_LOOP_HZ;
}

float Encoder_GetVelocityRPM(void)
{
    static int32_t prev_count = 0;
    int32_t current_count     = Encoder_GetCount();
    int32_t delta             = current_count - prev_count;
    prev_count                = current_count;
    return (float)delta * CONTROL_LOOP_HZ * 60.0f / COUNTS_PER_REV;
}