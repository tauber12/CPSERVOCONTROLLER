/*
 * overcurrent.c
 *
 * Configures the active-low over-current input from the external NPN fault
 * transistor. The pin is treated as a normal GPIO input, not TIM1_BKIN.
 *
 * Fault behavior:
 *   - PB12 high: normal
 *   - PB12 low:  over-current comparator is active
 *
 * Majority-vote filter:
 *   - Sample PB12 every 1 ms.
 *   - Latch the fault when at least OVERCURRENT_TRIP_SAMPLES samples are low
 *     inside the last OVERCURRENT_WINDOW_MS samples.
 *
 * Default rule:
 *   7 active-low samples out of the last 10 ms -> latch over-current fault.
 *
 * This file intentionally does not modify TIM1, MOE, CCRx, or any PWM-related
 * register.
 */

#include "overcurrent.h"

static volatile uint8_t overcurrent_latched = 0U;

static uint8_t overcurrent_samples[OVERCURRENT_WINDOW_MS];
static volatile uint8_t overcurrent_sample_index = 0U;
static volatile uint8_t overcurrent_active_count = 0U;
static volatile uint8_t overcurrent_samples_seen = 0U;

static void OverCurrent_FilterClear(void)
{
    uint8_t i;

    for (i = 0U; i < OVERCURRENT_WINDOW_MS; i++)
    {
        overcurrent_samples[i] = 0U;
    }

    overcurrent_sample_index = 0U;
    overcurrent_active_count = 0U;
    overcurrent_samples_seen = 0U;
}

static void OverCurrent_GPIO_Init(void)
{
    uint32_t pin_shift = OVERCURRENT_GPIO_PIN * 2UL;

    OVERCURRENT_GPIO_CLK_ENABLE();

    /* PB12 as GPIO input. */
    OVERCURRENT_GPIO_PORT->MODER &= ~(3UL << pin_shift);

    /* NPN collector/open-collector style output: use pull-up. */
    OVERCURRENT_GPIO_PORT->PUPDR &= ~(3UL << pin_shift);
    OVERCURRENT_GPIO_PORT->PUPDR |=  (1UL << pin_shift);

    OVERCURRENT_GPIO_PORT->OSPEEDR &= ~(3UL << pin_shift);
}

void OverCurrent_Init(void)
{
    overcurrent_latched = 0U;
    OverCurrent_FilterClear();
    OverCurrent_GPIO_Init();
    OverCurrent_ClearPending();
}

uint8_t OverCurrent_IsActive(void)
{
    return ((OVERCURRENT_GPIO_PORT->IDR & OVERCURRENT_GPIO_PIN_MASK) == 0UL) ? 1U : 0U;
}

void OverCurrent_Update_1ms(void)
{
    uint8_t old_sample;
    uint8_t new_sample;

    new_sample = OverCurrent_IsActive();
    old_sample = overcurrent_samples[overcurrent_sample_index];

    /* Remove the sample being overwritten from the running count. */
    if (old_sample != 0U)
    {
        if (overcurrent_active_count > 0U)
        {
            overcurrent_active_count--;
        }
    }

    /* Store and count the new sample. */
    overcurrent_samples[overcurrent_sample_index] = new_sample;

    if (new_sample != 0U)
    {
        if (overcurrent_active_count < OVERCURRENT_WINDOW_MS)
        {
            overcurrent_active_count++;
        }
    }

    /* Advance circular window. */
    overcurrent_sample_index++;
    if (overcurrent_sample_index >= OVERCURRENT_WINDOW_MS)
    {
        overcurrent_sample_index = 0U;
    }

    if (overcurrent_samples_seen < OVERCURRENT_WINDOW_MS)
    {
        overcurrent_samples_seen++;
    }

    /* Evaluate only after the window has filled once. This keeps the earliest
     * trip time near OVERCURRENT_WINDOW_MS instead of allowing an immediate
     * trip after only OVERCURRENT_TRIP_SAMPLES milliseconds.
     */
    if (overcurrent_samples_seen >= OVERCURRENT_WINDOW_MS)
    {
        if (overcurrent_active_count >= OVERCURRENT_TRIP_SAMPLES)
        {
            overcurrent_latched = 1U;
        }
    }
}

uint8_t OverCurrent_IsLatched(void)
{
    return overcurrent_latched;
}

uint8_t OverCurrent_FaultPending(void)
{
    return overcurrent_latched;
}

uint8_t OverCurrent_GetDebounceCount(void)
{
    return overcurrent_active_count;
}

uint8_t OverCurrent_GetActiveSampleCount(void)
{
    return overcurrent_active_count;
}

void OverCurrent_ClearPending(void)
{
    EXTI->PR1 = OVERCURRENT_GPIO_PIN_MASK;
}

void OverCurrent_Rearm(void)
{
    OverCurrent_ClearPending();

    if (OverCurrent_IsActive() == 0U)
    {
        OverCurrent_FilterClear();
        overcurrent_latched = 0U;
    }
    else
    {
        /* Do not let RESET clear the fault while the hardware line is still
         * actively low. Keep the latch set and seed the filter as tripped so
         * the UI can continue to show the active hardware condition.
         */
        uint8_t i;

        for (i = 0U; i < OVERCURRENT_WINDOW_MS; i++)
        {
            overcurrent_samples[i] = 1U;
        }

        overcurrent_sample_index = 0U;
        overcurrent_active_count = OVERCURRENT_WINDOW_MS;
        overcurrent_samples_seen = OVERCURRENT_WINDOW_MS;
        overcurrent_latched = 1U;
    }
}

void EXTI15_10_IRQHandler(void)
{
    /* Over-current is intentionally polled/filtered from TIM7 now. Keep this
     * handler only so a stale EXTI12 pending bit cannot trap the CPU if the
     * vector is present in the startup file.
     */
    if ((EXTI->PR1 & OVERCURRENT_GPIO_PIN_MASK) != 0UL)
    {
        EXTI->PR1 = OVERCURRENT_GPIO_PIN_MASK;
    }
}
