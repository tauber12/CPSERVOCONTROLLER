/*
 *******************************************************************************
 * @file           : encoder.h
 * @brief          : Quadrature encoder driver with M/T velocity estimation
 * project         : EE 329 S'26 AX
 * version         : 0.3
 * date            : May 30, 2026
 *******************************************************************************
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define ENCODER_DEFAULT_SAMPLE_HZ 5000.0f

#define PPR                 7
#define GEAR_RATIO          150
#define COUNTS_PER_REV      (PPR * GEAR_RATIO * 4)   // 4,200

void    Encoder_Config(void);
void    Encoder_RecordEdge(void);       // called from EXTI0 ISR (in encoder.c)

void    Encoder_SetSampleRateHz(float sample_rate_hz);
float   Encoder_GetSampleRateHz(void);
float   Encoder_GetLowRpmThreshold(void);

int32_t Encoder_GetCount(void);
float   Encoder_GetRevolutions(void);
float   Encoder_GetDegrees(void);
float   Encoder_GetVelocityRPM(void);  // M/T blended
float   Encoder_GetVelocityCPS(void);  // M-method only, for diagnostics

#endif /* INC_ENCODER_H_ */
