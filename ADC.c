/*
 *******************************************************************************
 * @file           : ADC.c
 * @brief          : X
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.1
 * date            : May 20, 2026
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
#include "ADC.h"

volatile uint16_t rawVoltageData = 0;

void ADC_init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;          // turn on clock for ADC
    // power up & calibrate ADC
    ADC123_COMMON->CCR |= (1 << ADC_CCR_CKMODE_Pos); // clock source = HCLK/1
    ADC1->CR &= ~(ADC_CR_DEEPPWD);              // disable deep-power-down
    ADC1->CR |= (ADC_CR_ADVREGEN);              // enable V regulator
    HAL_Delay(1);                               // wait 1 ms for ADC to power up
    ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_2);     // PC1=ADC1_IN2, single-ended
    ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF); // disable ADC, single-end calib
    ADC1->CR |= ADC_CR_ADCAL;                   // start calibration
    while (ADC1->CR & ADC_CR_ADCAL) {;}         // wait for calib to finish
    // enable ADC
    ADC1->ISR |= (ADC_ISR_ADRDY);               // set to clr ADC Ready flag
    ADC1->CR |= ADC_CR_ADEN;                    // enable ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY)) {;}     // wait for ADC Ready flag
    ADC1->ISR |= (ADC_ISR_ADRDY);               // set to clr ADC Ready flag
    // configure ADC sampling & sequencing
    ADC1->SQR1  |= (2 << ADC_SQR1_SQ1_Pos);     // sequence = 1 conv., ch 2
    ADC1->SMPR1 |= (1 << ADC_SMPR1_SMP2_Pos);   // ch 2 sample time = 6.5 clocks
    ADC1->CFGR  &= ~( ADC_CFGR_CONT  |          // single conversion mode
                      ADC_CFGR_EXTEN |          // h/w trig disabled for s/w trig
                      ADC_CFGR_RES   );         // 12-bit resolution
    // configure & enable ADC interrupt
    ADC1->IER |= ADC_IER_EOCIE;                 // enable end-of-conv interrupt
    ADC1->ISR |= ADC_ISR_EOC;                   // set to clear EOC flag
    NVIC->ISER[0] = (1<<(ADC1_2_IRQn & 0x1F));  // enable ADC interrupt service
    __enable_irq();                             // enable global interrupts
    // configure GPIO pin PC1
    RCC->AHB2ENR  |= (RCC_AHB2ENR_GPIOCEN);     // connect clock to GPIOC
    GPIOC->MODER  |= (GPIO_MODER_MODE1);        // analog mode for PC1

    ADC1->CR |= ADC_CR_ADSTART;                 // start 1st conversion
}

void ADC1_2_IRQHandler(void) {
    if (ADC1->ISR & ADC_ISR_EOC) {
        rawVoltageData = ADC1->DR;              // read DR, clears EOC flag
        ADC1->CR |= ADC_CR_ADSTART;             // start next conversion
    }
}
