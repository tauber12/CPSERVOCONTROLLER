/*
 * overcurrent.h
 *
 * Active-low over-current detection input with majority-vote filtering.
 *
 * Default wiring:
 *   NPN collector / fault node -> PB12
 *   NPN emitter                 -> GND
 *   PB12 is configured as a GPIO input with pull-up.
 *
 * This version does NOT use TIM1_BKIN and does NOT touch TIM1/PWM registers.
 * The pin is sampled every 1 ms from the existing button/HMI timer.
 *
 * Fault rule, by default:
 *   PB12 low for at least 7 of the last 10 samples -> overcurrent latched
 *
 * This rejects short comparator glitches, but still trips when the comparator
 * is mostly active during noisy/oscillatory operation.
 */

#ifndef SRC_OVERCURRENT_H_
#define SRC_OVERCURRENT_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define OVERCURRENT_GPIO_PORT          GPIOB
#define OVERCURRENT_GPIO_CLK_ENABLE()  (RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN)
#define OVERCURRENT_GPIO_PIN           12U
#define OVERCURRENT_GPIO_PIN_MASK      (1UL << OVERCURRENT_GPIO_PIN)

/* Called once per 1 ms. */
#define OVERCURRENT_SAMPLE_PERIOD_MS   1U

/* Majority-vote filter window and threshold. */
#define OVERCURRENT_WINDOW_MS          100U
#define OVERCURRENT_TRIP_SAMPLES       80U

/* Backward-compatible name from the old strict-debounce version. */
#define OVERCURRENT_DEBOUNCE_MS        OVERCURRENT_WINDOW_MS

#if (OVERCURRENT_TRIP_SAMPLES > OVERCURRENT_WINDOW_MS)
#error "OVERCURRENT_TRIP_SAMPLES must be <= OVERCURRENT_WINDOW_MS"
#endif

void OverCurrent_Init(void);
void OverCurrent_Update_1ms(void);

/* Raw active-low input state: 1 = PB12 low right now, 0 = PB12 high right now. */
uint8_t OverCurrent_IsActive(void);

/* Latched software fault state. */
uint8_t OverCurrent_IsLatched(void);
uint8_t OverCurrent_FaultPending(void);

/* Returns active-low sample count in the current majority window. */
uint8_t OverCurrent_GetDebounceCount(void);
uint8_t OverCurrent_GetActiveSampleCount(void);

void OverCurrent_ClearPending(void);
void OverCurrent_Rearm(void);

#endif /* SRC_OVERCURRENT_H_ */
