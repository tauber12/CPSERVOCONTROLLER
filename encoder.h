/*
 *******************************************************************************
 * @file           : encoder.h
 * @brief          : Quadrature encoder position + filtered T-method RPM driver
 *******************************************************************************
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

/* -------------------------------------------------------------------------- */
/* Motor encoder/mechanics configuration                                      */
/* -------------------------------------------------------------------------- */

/* Kept for compatibility with the control code.  The T-method RPM estimator
 * does not depend on the control-loop sampling rate. */
#define ENCODER_DEFAULT_SAMPLE_HZ              5000.0f

/* Motor uses an 11 PPR motor-shaft encoder and a 20:1 gearbox.
 * TIM2 decodes quadrature x4, so this is 44 counts/motor rev and
 * 880 counts/gearbox-output rev. */
#define ENCODER_PPR                            11.0f
#define ENCODER_GEAR_RATIO                     20.0f
#define ENCODER_X4_COUNTS_PER_PULSE            4.0f

#define ENCODER_COUNTS_PER_MOTOR_REV           (ENCODER_PPR * ENCODER_X4_COUNTS_PER_PULSE)
#define ENCODER_COUNTS_PER_OUTPUT_REV          (ENCODER_COUNTS_PER_MOTOR_REV * ENCODER_GEAR_RATIO)

/* Position is reported at the gearbox output shaft. */
#define COUNTS_PER_REV                         ENCODER_COUNTS_PER_OUTPUT_REV

/* Velocity is reported as gearbox-output RPM by default.
 * Set this to 1 only if motor-shaft RPM is desired instead. */
#define ENCODER_REPORT_MOTOR_RPM               0U
#if ENCODER_REPORT_MOTOR_RPM
#define ENCODER_RPM_COUNTS_PER_REV             ENCODER_COUNTS_PER_MOTOR_REV
#define ENCODER_T_EDGES_PER_REV                ENCODER_PPR
#else
#define ENCODER_RPM_COUNTS_PER_REV             ENCODER_COUNTS_PER_OUTPUT_REV
#define ENCODER_T_EDGES_PER_REV                (ENCODER_PPR * ENCODER_GEAR_RATIO)
#endif

/* -------------------------------------------------------------------------- */
/* T-method velocity-estimator tuning                                         */
/* -------------------------------------------------------------------------- */

/* TIM3 is used as a 1 MHz microsecond timebase for edge-period timing. */
#define ENCODER_TIM3_TICK_HZ                   1000000UL
#define ENCODER_TIM3_PERIOD_US                 65536UL
#define ENCODER_TIM3_MASK                      0xFFFFUL
#define ENCODER_TIM3_PRESCALER                 ((48000000UL / ENCODER_TIM3_TICK_HZ) - 1UL)

/* Intended measurable gearbox-output RPM range. */
#define ENCODER_MIN_MEASURABLE_RPM             0.1f
#define ENCODER_MAX_MEASURABLE_RPM             500.0f

/* Channel-A rising-edge period bounds.  The margin allows normal acceleration
 * above the nominal max speed but rejects impossible double edges/noise. */
#define ENCODER_EDGE_PERIOD_MIN_MARGIN         0.70f
#define ENCODER_EDGE_PERIOD_MAX_MARGIN         1.50f

#define ENCODER_T_PERIOD_US_AT_RPM(rpm) \
    ((uint32_t)((60000000.0f / ((rpm) * ENCODER_T_EDGES_PER_REV)) + 0.5f))

#define ENCODER_T_MIN_PERIOD_US \
    ((uint32_t)((60000000.0f / (ENCODER_MAX_MEASURABLE_RPM * ENCODER_T_EDGES_PER_REV)) * \
                ENCODER_EDGE_PERIOD_MIN_MARGIN))

#define ENCODER_T_MAX_PERIOD_US \
    ((uint32_t)((60000000.0f / (ENCODER_MIN_MEASURABLE_RPM * ENCODER_T_EDGES_PER_REV)) * \
                ENCODER_EDGE_PERIOD_MAX_MARGIN))

/* A real PA0-rising to PA0-rising event should advance TIM2 by about 4 x4
 * counts.  This tight check rejects stationary electrical noise and tiny
 * back-and-forth jitter that otherwise appears as a nonzero RPM. */
#define ENCODER_T_MIN_EDGE_COUNT_DELTA         3L
#define ENCODER_T_MAX_EDGE_COUNT_DELTA         6L

/* A missed expected edge means the motor has slowed/stopped.  This dynamic
 * timeout prevents the RPM from holding a stale nonzero value while stationary.
 * The minimum timeout avoids zeroing during normal high-speed edge jitter. */
#define ENCODER_STATIONARY_TIMEOUT_PERIODS     2.0f
#define ENCODER_STATIONARY_MIN_TIMEOUT_US      5000UL
#define ENCODER_STALL_TIMEOUT_US               ENCODER_T_MAX_PERIOD_US

/* Period filtering.  Higher alpha = faster control feedback but noisier RPM.
 * Lower alpha = smoother display but more feedback delay. */
#define ENCODER_T_ENABLE_MEDIAN3               1U
#define ENCODER_T_PERIOD_FILTER_ALPHA          0.25f

/* Require repeated agreement before changing reported direction. */
#define ENCODER_T_DIRECTION_CONFIRM_EDGES      2U

/* Final output conditioning. */
#define ENCODER_ZERO_RPM_DEADBAND              0.05f
#define ENCODER_RPM_SLEW_LIMIT_RPM_PER_S       20000.0f

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
