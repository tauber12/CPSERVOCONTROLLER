/*
 *******************************************************************************
 * @file           : encoder.h
 * @brief          : Quadrature encoder driver with M-method velocity estimate
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

/* Position is reported at the gearbox output shaft. */
#define COUNTS_PER_REV                     ENCODER_COUNTS_PER_OUTPUT_REV

/* Velocity is reported as gearbox-output RPM by default.
 * Set this to 1 only if motor-shaft RPM is desired instead. */
#define ENCODER_REPORT_MOTOR_RPM           0U
#if ENCODER_REPORT_MOTOR_RPM
#define ENCODER_RPM_COUNTS_PER_REV         ENCODER_COUNTS_PER_MOTOR_REV
#else
#define ENCODER_RPM_COUNTS_PER_REV         ENCODER_COUNTS_PER_OUTPUT_REV
#endif

/* -------------------------------------------------------------------------- */
/* M-method velocity-estimator tuning                                         */
/* -------------------------------------------------------------------------- */

/* RPM is calculated from encoder-count delta over this many velocity-loop
 * samples. This is still pure M-method; the window only reduces low-PPR
 * quantization. At 5 kHz and 880 counts/rev, 32 samples gives about
 * 10.65 RPM/count resolution and about 6.4 ms update spacing. */
#define ENCODER_RPM_WINDOW_SAMPLES         32U

/* Small displayed/controller deadband after the windowed M-method update. */
#define ENCODER_RPM_DEADBAND               0.5f

/* TIM2 digital input filter used on both quadrature channels. */
#define ENCODER_TIM2_INPUT_FILTER          0x2U

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
