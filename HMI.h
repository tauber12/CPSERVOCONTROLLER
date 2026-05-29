/*
 *******************************************************************************
 * @file           : HMI.h
 * @brief          : Human Machine Interface - menu navigation and user input
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.2
 * date            : May 25, 2026
 * compiler        : STM32CubeIDE v.1.19.0 Build: 14980_20230301_1550 (UTC)
 * target          : NUCLEO-L4A6ZG
 * clocks          : 4 MHz MSI to AHB2
 * @attention      : (c) 2026 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * Description: Handles menu navigation, user input via button and encoder,
 *              and UI state management for TFT display rendering.
 *
 *******************************************************************************
 * GPIO Wiring
 * |   Component    | GPIO Identifier | Connector Location | Config
 *-----------------------------------------------------------------------------
 * | Button         | PC13            | CN11-23            | IN  pull-down
 * | Encoder A(TI1) | PB6             | CN10-17            | AF2 pull-up
 * | Encoder B(TI2) | PB7             | CN10-21            | AF2 pull-up
 *******************************************************************************
 * Version History
 *  Ver.|   Date   |  Description
 *  ---------------------------------------------------------------------------
 *  0.1 | 05-25-26 | Initial version
 *  0.2 | 05-25-26 | Added encoder and TIM7 declarations
 *******************************************************************************
 *
 * Header format adapted from [Code Appendix by Kevin Vo] pg 5
 */

#ifndef INC_HMI_H_
#define INC_HMI_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

/* ─── Constants ──────────────────────────────────────────────────────────── */

#define PPR             600
#define COUNTS_PER_REV  (PPR * 4)   /* 2400 counts/rev (quadrature x4) */

/* ─── UI Event Enum ──────────────────────────────────────────────────────── */

typedef enum {
    UI_EVENT_NONE = 0,
    UI_EVENT_UP,
    UI_EVENT_DOWN,
    UI_EVENT_SELECT
} UiEvent_t;

/* ─── UI Screen Enum ─────────────────────────────────────────────────────── */

typedef enum {
    UI_SCREEN_MAIN = 0,
    UI_SCREEN_MOTOR,
    UI_SCREEN_SETTINGS
} UiScreen_t;

/* ─── UI Context Struct ──────────────────────────────────────────────────── */

typedef struct {
    UiScreen_t  screen;
    uint8_t     selected_index;
    uint8_t     needs_redraw;
} UiContext_t;

/* ─── Button API ─────────────────────────────────────────────────────────── */

void    Button_Init(void);
void    Button_Update_1ms(void);
uint8_t Button_WasPressed(void);

/* ─── Timer / ISR ────────────────────────────────────────────────────────── */

void    setup_TIM7_ButtonPoll(void);
void    TIM7_IRQHandler(void);

/* ─── UI API ─────────────────────────────────────────────────────────────── */

void    UI_Init(void);
void    UI_Update(UiEvent_t event);
void    UI_Draw(void);

/* ─── Encoder API ────────────────────────────────────────────────────────── */

void    HMI_Encoder_Config(void);
int32_t HMI_Encoder_GetCount(void);
float   HMI_Encoder_GetRevolutions(void);
float   HMI_Encoder_GetDegrees(void);

#endif /* INC_HMI_H_ */
