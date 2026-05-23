/*
 *******************************************************************************
 * @file           : encoder.h
 * @brief          : X
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.1
 * date            : May 14, 2026
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


#ifndef ENCODER_H
#define ENCODER_H

#include "stm32l4xx.h"
#include <stdint.h>

/* ── Configuration ────────────────────────────────────────────────────────── */
#define PPR              7
#define GEAR_RATIO       150
#define COUNTS_PER_REV   (PPR * GEAR_RATIO * 4)   // 4200
#define CONTROL_LOOP_HZ  1000

/* ── Initialisation ───────────────────────────────────────────────────────── */
void    Encoder_Config(void);
void    TIM3_InputCapture_Init(void);

/* ── Position ─────────────────────────────────────────────────────────────── */
int32_t Encoder_GetCount(void);
float   Encoder_GetRevolutions(void);
float   Encoder_GetDegrees(void);

/* ── Velocity ─────────────────────────────────────────────────────────────── */
float   Encoder_GetVelocityCPS(void);       // differentiation — counts/sec
float   Encoder_GetVelocityRPM(void);       // differentiation — RPM
float   Encoder_GetVelocityIC_RPM(void);    // input capture  — RPM (precise at low speed)

#endif /* ENCODER_H */