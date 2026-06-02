/*
 *******************************************************************************
 * @file           : encoder.h
 * @brief          : Quadrature encoder driver with adaptive-window RPM estimate
 *******************************************************************************
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

/* -------------------------------------------------------------------------- */
/* Motor encoder/mechanics configuration                                      */
/* -------------------------------------------------------------------------- */

#define ENCODER_DEFAULT_SAMPLE_HZ          5000.0f

/* Motor uses an 11 PPR motor-shaft encoder and a 20:1 gearbox.
 * TIM2 decodes quadrature x4, so this is 44 counts/motor rev and
 * 880 counts/gearbox-output rev. */
#define ENCODER_PPR                        11.0f
#define ENCODER_GEAR_RATIO                 20.0f
#define ENCODER_X4_COUNTS_PER_PULSE        4.0f

#define ENCODER_COUNTS_PER_MOTOR_REV       (ENCODER_PPR * ENCODER_X4_COUNTS_PER_PULSE)
#define ENCODER_COUNTS_PER_OUTPUT_REV      (ENCODER_COUNTS_PER_MOTOR_REV * ENCODER_GEAR_RATIO)

/* Position is still reported at the gearbox output shaft. */
#define COUNTS_PER_REV                     ENCODER_COUNTS_PER_OUTPUT_REV

/* The UI/control velocity setpoint is gearbox-output RPM by default.
 * Set this to 1 only if you intentionally want motor-shaft RPM instead. */
#define ENCODER_REPORT_MOTOR_RPM           0U
#if ENCODER_REPORT_MOTOR_RPM
#define ENCODER_RPM_COUNTS_PER_REV         ENCODER_COUNTS_PER_MOTOR_REV
#else
#define ENCODER_RPM_COUNTS_PER_REV         ENCODER_COUNTS_PER_OUTPUT_REV
#endif

/* Adaptive RPM-estimator tuning. The estimator waits for enough counts or for
 * the max window time, whichever happens first. This avoids the sign jitter and
 * +/- spikes that occur when a 1-sample delta rounds between 0 and 1 count. */
#define ENCODER_RPM_MIN_WINDOW_US          2500UL
#define ENCODER_RPM_MAX_WINDOW_US          80000UL
#define ENCODER_RPM_MIN_UPDATE_COUNTS      8L
#define ENCODER_RPM_STALL_TIMEOUT_US       160000UL
#define ENCODER_MAX_REPORTED_RPM           1200.0f
#define ENCODER_RPM_GLITCH_MARGIN          1.75f
#define ENCODER_RPM_FILTER_ALPHA_LOW       0.08f
#define ENCODER_RPM_FILTER_ALPHA_HIGH      0.22f
#define ENCODER_RPM_LOW_FILTER_THRESHOLD   60.0f
#define ENCODER_ZERO_RPM_DEADBAND          0.35f
#define ENCODER_RPM_SIGN_HYSTERESIS        5.0f

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
