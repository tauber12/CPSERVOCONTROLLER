/*
 *******************************************************************************
 * @file           : encoder.h
 * @brief          : Quadrature encoder driver with filtered M/T RPM estimation
 *******************************************************************************
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

/* -------------------------------------------------------------------------- */
/* Motor encoder/mechanics configuration                                       */
/* -------------------------------------------------------------------------- */

#define ENCODER_DEFAULT_SAMPLE_HZ        5000.0f

/* New motor: 220 PPR encoder, 20:1 gearbox, TIM2 x4 quadrature decoding.
 * Returned position/RPM are gearbox output-shaft values. If your 220 PPR spec
 * is already measured at the gearbox output shaft, change ENCODER_GEAR_RATIO
 * to 1.0f. */
#define ENCODER_PPR                      220.0f
#define ENCODER_GEAR_RATIO               20.0f
#define ENCODER_X4_COUNTS_PER_PULSE      4.0f

#define ENCODER_T_EDGES_PER_REV          (ENCODER_PPR * ENCODER_GEAR_RATIO)
#define COUNTS_PER_REV                   (ENCODER_T_EDGES_PER_REV * ENCODER_X4_COUNTS_PER_PULSE)

/* Velocity-estimator tuning. */
#define ENCODER_M_MIN_COUNTS_PER_SAMPLE  4.0f
#define ENCODER_M_WINDOW_SAMPLES         4U
#define ENCODER_MAX_OUTPUT_RPM           600.0f
#define ENCODER_STALL_TIMEOUT_US         250000UL
#define ENCODER_T_PERIOD_FILTER_ALPHA    0.20f
#define ENCODER_RPM_FILTER_ALPHA_LOW     0.12f
#define ENCODER_RPM_FILTER_ALPHA_HIGH    0.25f

void    Encoder_Config(void);
void    Encoder_RecordEdge(void);
void    Encoder_ResetCount(void);

void    Encoder_SetSampleRateHz(float sample_rate_hz);
float   Encoder_GetSampleRateHz(void);
float   Encoder_GetLowRpmThreshold(void);

int32_t Encoder_GetCount(void);
float   Encoder_GetRevolutions(void);
float   Encoder_GetDegrees(void);
float   Encoder_GetVelocityRPM(void);
float   Encoder_GetVelocityCPS(void);

#endif /* INC_ENCODER_H_ */
