/*
 *******************************************************************************
 * @file           : HMI.h
 * @brief          : X
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.1
 * date            : May 25, 2026
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


#ifndef INC_HMI_H_
#define INC_HMI_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

typedef enum {
    UI_EVENT_NONE = 0,
    UI_EVENT_UP,
    UI_EVENT_DOWN,
    UI_EVENT_SELECT
} UiEvent_t;

typedef enum {
    UI_SCREEN_MAIN = 0,
    UI_SCREEN_MOTOR,
    UI_SCREEN_SETTINGS
} UiScreen_t;

typedef struct {
    UiScreen_t screen;
    uint8_t selected_index;
    uint8_t needs_redraw;
} UiContext_t;


void Button_Init(void);
void Button_Update_1ms(void);
uint8_t Button_WasPressed(void);

void setup_TIM7_ButtonPoll(void);

void UI_Init(void);
void UI_Update(UiEvent_t event);
void UI_Draw(void);


#endif /* INC_HMI_H_ */
