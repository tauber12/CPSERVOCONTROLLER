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
#define COUNTS_PER_REV  (PPR * GEAR_RATIO * 4)

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
    // Use to configure direction of count with respect to encoder direction
    TIM2->CCER &= ~(TIM_CCER_CC1P  | TIM_CCER_CC2P |
                    TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM2->CCER |=  (TIM_CCER_CC1E  | TIM_CCER_CC2E);

    /* 6. Full 32-bit range, start at 0 */
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CNT = 0;

    /* 7. Initialise registers, clear any spurious UIF */
    TIM2->EGR |=  TIM_EGR_UG;
    TIM2->SR  &= ~TIM_SR_UIF;

    /* 8. Start counter */
    TIM2->CR1 |= TIM_CR1_CEN;
}

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
