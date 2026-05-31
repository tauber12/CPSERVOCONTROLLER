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

#define HMI_LOCAL_PPR                    600.0f
#define HMI_LOCAL_COUNTS_PER_REV         (HMI_LOCAL_PPR * 4.0f)
#define HMI_ENCODER_COUNTS_PER_STEP      80
#define HMI_ENCODER_MAX_STEPS_PER_TASK   8
#define HMI_ENCODER_TIMER_COUNTS         65536L

#define HMI_VALUE_REFRESH_MS             200U
#define HMI_PLOT_REFRESH_MS              50U
#define HMI_PLOT_TICK_MS                 1000U
#define HMI_PLOT_MAJOR_TICK_MS           5000U

#define HMI_PLOT_X                       28U
#define HMI_PLOT_Y                       52U
#define HMI_PLOT_W                       280U
#define HMI_PLOT_H                       158U
#define HMI_PLOT_POS_MIN_DEG             -180.0f
#define HMI_PLOT_POS_MAX_DEG             180.0f

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
void HMI_Encoder_ResetCount(void);
int32_t HMI_Encoder_GetCount(void);
float HMI_Encoder_GetRevolutions(void);
float HMI_Encoder_GetDegrees(void);

#endif /* INC_HMI_H_ */
