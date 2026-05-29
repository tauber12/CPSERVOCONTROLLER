/*
 *******************************************************************************
 * @file           : HMI.c
 * @brief          : X
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.1
 * date            : May 25, 2026
 * compiler        : STM32CubeIDE v.1.19.0 Build: 14980_20230301_1550 (UTC)
 * target          : NUCLEO-L4A6ZG
 * clocks          : 4 MHz MSI to AHB2
 * @attention      : (c) 2026 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * Description: This file will handle menu navigation along with user input
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


#include "HMI.h"
#define MAIN_MENU_COUNT   3
#define MOTOR_MENU_COUNT  3

typedef enum {
    MAIN_ITEM_MOTOR = 0,
    MAIN_ITEM_SETTINGS,
    MAIN_ITEM_ABOUT
} MainMenuItem_t;

typedef enum {
    MOTOR_ITEM_ENABLE_TRACKING = 0,
    MOTOR_ITEM_DISABLE_TRACKING,
    MOTOR_ITEM_BACK
} MotorMenuItem_t;

static UiContext_t ui;

#define BUTTON_DEBOUNCE_MS 25

#define BUTTON_NOT_PRESSED 0
#define BUTTON_PRESSED     1

static uint8_t button_stable_state = BUTTON_NOT_PRESSED;
static uint8_t button_last_raw_state = BUTTON_NOT_PRESSED;
static uint8_t button_debounce_count = 0;
static uint8_t button_pressed_event = 0;

void Button_Init(void)
{
    /*
     * Configure GPIO_PIN_13 GPIOC as button input.
     * PC13 uses pull-down, so:
     *
     * pressed     = 1
     * not pressed = 0
     */

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    GPIOC->MODER &= ~(GPIO_MODER_MODE13);

    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13);
    GPIOC->PUPDR |=  GPIO_PUPDR_PUPD13_1;

    button_stable_state = BUTTON_NOT_PRESSED;
    button_last_raw_state = BUTTON_NOT_PRESSED;
    button_debounce_count = 0;
    button_pressed_event = 0;
}

static uint8_t Button_ReadRaw(void)
{
    if (GPIOC->IDR & GPIO_IDR_ID13)
    {
        return BUTTON_PRESSED;
    }
    else
    {
        return BUTTON_NOT_PRESSED;
    }
}

void Button_Update_1ms(void)
{
    uint8_t raw_state = Button_ReadRaw();

    if (raw_state != button_last_raw_state)
    {
        button_last_raw_state = raw_state;
        button_debounce_count = 0;
    }
    else
    {
        if (button_debounce_count < BUTTON_DEBOUNCE_MS)
        {
            button_debounce_count++;
        }
        else
        {
            if (button_stable_state != raw_state)
            {
                button_stable_state = raw_state;

                if (button_stable_state == BUTTON_PRESSED)
                {
                    button_pressed_event = 1;
                }
            }
        }
    }
}

uint8_t Button_WasPressed(void)
{
    if (button_pressed_event)
    {
        button_pressed_event = 0;
        return 1;
    }

    return 0;
}

void setup_TIM7_ButtonPoll(void)
{

    // Enable TIM7 peripheral clock.

    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;

    // Prescale 48 MHz -> 1 MHz
    TIM7->PSC = 47;
    // Interrupt rate = 1ms
    TIM7->ARR = 999;


    //Reset counter.

    TIM7->CNT = 0;


    //Clear update interrupt flag.

    TIM7->SR &= ~TIM_SR_UIF;


    //Enable update interrupt.

    TIM7->DIER |= TIM_DIER_UIE;

    NVIC->ISER[1] |= (1 << (TIM7_IRQn & 0x1F));

    /*
     * Start TIM7.
     */
    TIM7->CR1 |= TIM_CR1_CEN;
}

void TIM7_IRQHandler(void)
{
    if (TIM7->SR & TIM_SR_UIF)
    {
        TIM7->SR &= ~TIM_SR_UIF;

        Button_Update_1ms();
    }
}

void UI_Init(void)
{
    ui.screen = UI_SCREEN_MAIN;
    ui.selected_index = 0;
    ui.needs_redraw = 1;
}

static void UI_MoveUp(uint8_t item_count)
{
    if (ui.selected_index == 0)
    {
        ui.selected_index = item_count - 1;
    }
    else
    {
        ui.selected_index--;
    }

    ui.needs_redraw = 1;
}

static void UI_MoveDown(uint8_t item_count)
{
    ui.selected_index++;

    if (ui.selected_index >= item_count)
    {
        ui.selected_index = 0;
    }

    ui.needs_redraw = 1;
}

static void UI_UpdateMainMenu(UiEvent_t event)
{
    switch (event)
    {
        case UI_EVENT_UP:
            UI_MoveUp(MAIN_MENU_COUNT);
            break;

        case UI_EVENT_DOWN:
            UI_MoveDown(MAIN_MENU_COUNT);
            break;

        case UI_EVENT_SELECT:
            switch (ui.selected_index)
            {
                case MAIN_ITEM_MOTOR:
                    ui.screen = UI_SCREEN_MOTOR;
                    ui.selected_index = 0;
                    ui.needs_redraw = 1;
                    break;

                case MAIN_ITEM_SETTINGS:
                    ui.screen = UI_SCREEN_SETTINGS;
                    ui.selected_index = 0;
                    ui.needs_redraw = 1;
                    break;

                case MAIN_ITEM_ABOUT:
                    /*
                     * Do something simple here later.
                     * For now, stay on the main menu.
                     */
                    ui.needs_redraw = 1;
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }
}

static void UI_UpdateMotorMenu(UiEvent_t event)
{
    switch (event)
    {
        case UI_EVENT_UP:
            UI_MoveUp(MOTOR_MENU_COUNT);
            break;

        case UI_EVENT_DOWN:
            UI_MoveDown(MOTOR_MENU_COUNT);
            break;

        case UI_EVENT_SELECT:
            switch (ui.selected_index)
            {
                case MOTOR_ITEM_ENABLE_TRACKING:
                    /*
                     * Later:
                     * tracking_enable_request = 1;
                     */
                    break;

                case MOTOR_ITEM_DISABLE_TRACKING:
                    /*
                     * Later:
                     * tracking_disable_request = 1;
                     */
                    break;

                case MOTOR_ITEM_BACK:
                    ui.screen = UI_SCREEN_MAIN;
                    ui.selected_index = 0;
                    ui.needs_redraw = 1;
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }
}

static void UI_UpdateSettingsMenu(UiEvent_t event)
{
    switch (event)
    {
        case UI_EVENT_UP:
        case UI_EVENT_DOWN:
            /*
             * Add settings items later.
             */
            break;

        case UI_EVENT_SELECT:
            /*
             * For now, use select to go back.
             */
            ui.screen = UI_SCREEN_MAIN;
            ui.selected_index = 0;
            ui.needs_redraw = 1;
            break;

        default:
            break;
    }
}

void UI_Update(UiEvent_t event)
{
    if (event == UI_EVENT_NONE)
    {
        return;
    }

    switch (ui.screen)
    {
        case UI_SCREEN_MAIN:
            UI_UpdateMainMenu(event);
            break;

        case UI_SCREEN_MOTOR:
            UI_UpdateMotorMenu(event);
            break;

        case UI_SCREEN_SETTINGS:
            UI_UpdateSettingsMenu(event);
            break;

        default:
            ui.screen = UI_SCREEN_MAIN;
            ui.selected_index = 0;
            ui.needs_redraw = 1;
            break;
    }
}

void UI_Draw(void)
{
    if (!ui.needs_redraw)
    {
        return;
    }

    ui.needs_redraw = 0;

    /*
     * Replace these comments with your TFT drawing functions.
     *
     * The basic idea:
     * - Clear the screen
     * - Draw the current menu title
     * - Draw each menu item
     * - Highlight the item where index == ui.selected_index
     */

    switch (ui.screen)
    {
        case UI_SCREEN_MAIN:
            /*
             * Draw:
             *   Main Menu
             *   > Motor
             *     Settings
             *     About
             */
            break;

        case UI_SCREEN_MOTOR:
            /*
             * Draw:
             *   Motor Menu
             *   > Enable Tracking
             *     Disable Tracking
             *     Back
             */
            break;

        case UI_SCREEN_SETTINGS:
            /*
             * Draw settings screen.
             */
            break;

        default:
            break;
    }
}

#define PPR             600
#define COUNTS_PER_REV  (PPR * 4) // 2400 counts per output shaft revolution
                                                // x4 because quadrature counts both edges of A and B


void HMI_Encoder_Config(void)
{
    /* 1. Enable clocks */
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;

    /* 2. PB6 (TI1), PB7 (TI2) → AF2 (TIM4), pull-up */
    GPIOB->MODER  &= ~(GPIO_MODER_MODE6    | GPIO_MODER_MODE7);
    GPIOB->MODER  |=  (GPIO_MODER_MODE6_1  | GPIO_MODER_MODE7_1);
    GPIOB->OSPEEDR|=  (GPIO_OSPEEDR_OSPEED6| GPIO_OSPEEDR_OSPEED7);
    GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPD6    | GPIO_PUPDR_PUPD7);
    GPIOB->PUPDR  |=  (GPIO_PUPDR_PUPD6_0  | GPIO_PUPDR_PUPD7_0);
    GPIOB->AFR[0] &= ~((0xF << GPIO_AFRL_AFSEL6_Pos) | (0xF << GPIO_AFRL_AFSEL7_Pos));
    GPIOB->AFR[0] |=  ((0x2 << GPIO_AFRL_AFSEL6_Pos) | (0x2 << GPIO_AFRL_AFSEL7_Pos));

    /* 3. Encoder mode x4 (count on both edges of TI1 and TI2) */
    TIM4->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_SMS_3);
    TIM4->SMCR |=  (3 << TIM_SMCR_SMS_Pos);

    /* 4. TI1→IC1, TI2→IC2, light noise filter */
    TIM4->CCMR1  = 0;
    TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM4->CCMR1 |= TIM_CCMR1_CC2S_0;
    TIM4->CCMR1 |= (0x2 << TIM_CCMR1_IC1F_Pos);
    TIM4->CCMR1 |= (0x2 << TIM_CCMR1_IC2F_Pos);

    /* 5. Active-high polarity, enable capture inputs */
    TIM4->CCER &= ~(TIM_CCER_CC1P  | TIM_CCER_CC2P |
                    TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM4->CCER |=  (TIM_CCER_CC1E  | TIM_CCER_CC2E);

    /* 6. Full 16-bit range */
    TIM4->ARR = 0xFFFF;
    TIM4->CNT = 0;

    /* 7. Generate update to load registers, clear spurious UIF */
    TIM4->EGR |=  TIM_EGR_UG;
    TIM4->SR  &= ~TIM_SR_UIF;

    /* 8. Start encoder counter */
    TIM4->CR1 |= TIM_CR1_CEN;
}

// Returns raw encoder count as signed 16-bit
// TIM4 is 16-bit so cast to int16_t to get correct signed wraparound
int32_t HMI_Encoder_GetCount(void)
{
    return (int32_t)(int16_t)TIM4->CNT;
}

// Returns total output shaft revolutions from count
float HMI_Encoder_GetRevolutions(void)
{
    return (float)HMI_Encoder_GetCount() / COUNTS_PER_REV;
}

// Returns absolute position in degrees
float HMI_Encoder_GetDegrees(void)
{
    return HMI_Encoder_GetRevolutions() * 360.0f;
}
