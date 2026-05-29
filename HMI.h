/*
 *******************************************************************************
 * @file           : HMI.h
 * @brief          : HMI input and UI backend API
 *******************************************************************************
 */

#ifndef INC_HMI_H_
#define INC_HMI_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

typedef enum
{
    UI_EVENT_NONE = 0,
    UI_EVENT_UP,
    UI_EVENT_DOWN,
    UI_EVENT_SELECT
} UiEvent_t;

void Button_Init(void);
void Button_Update_1ms(void);
uint8_t Button_WasPressed(void);
void setup_TIM7_ButtonPoll(void);

void UI_Init(void);
void UI_Task(void);
void UI_Update(UiEvent_t event);
void UI_Draw(void);

void HMI_Encoder_Config(void);
int32_t HMI_Encoder_GetCount(void);
float HMI_Encoder_GetRevolutions(void);
float HMI_Encoder_GetDegrees(void);

#endif /* INC_HMI_H_ */
