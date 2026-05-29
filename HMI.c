/*
 *******************************************************************************
 * @file           : HMI.c
 * @brief          : Object-like HMI backend for TFT buttons and numeric entries
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.4
 * date            : May 29, 2026
 *******************************************************************************
 * Description:
 *   This file owns all interactive/dynamic UI items on top of the static
 *   control-loop diagram drawn by control_loop_display.c.
 *
 *   UI item model:
 *       item->render(item)
 *       item->on_press(item)
 *       item->on_encoder(item, steps)
 *
 *   Items currently rendered:
 *       - VELOCITY button
 *       - POSITION button
 *       - TRACK button
 *       - Input block source label: SRC:ENC
 *       - Position PI editable entries: Kp, Ki
 *       - Position PI readout: Fs
 *       - Velocity PI editable entries: Kp, Ki
 *       - Velocity PI readout: Fs
 *
 *   Button behavior:
 *       - VELOCITY sets desired_state to STATE_VELOCITY_CONTROL.
 *       - POSITION sets desired_state to STATE_POSITION_CONTROL.
 *       - If tracking is already enabled, velocity/position can switch live.
 *       - TRACK raises tracking_toggle_request and locks HMI encoder scrolling
 *         while the motor is tracking.
 *
 *   The static block diagram, arrows, feedback paths, and block outlines are
 *   intentionally not drawn here. Those belong to control_loop_display.c.
 *******************************************************************************
 */

#include "HMI.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "ILI9341_text.h"
#include "control.h"

/* -------------------------------------------------------------------------- */
/* Button debounce                                                            */
/* -------------------------------------------------------------------------- */

#define BUTTON_DEBOUNCE_MS      25U
#define BUTTON_NOT_PRESSED      0U
#define BUTTON_PRESSED          1U

static uint8_t button_stable_state = BUTTON_NOT_PRESSED;
static uint8_t button_last_raw_state = BUTTON_NOT_PRESSED;
static uint8_t button_debounce_count = 0U;
static uint8_t button_pressed_event = 0U;

void Button_Init(void)
{
    /* PC13 input with pull-down. Pressed = 1, not pressed = 0. */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    GPIOC->MODER &= ~(GPIO_MODER_MODE13);

    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13);
    GPIOC->PUPDR |=  GPIO_PUPDR_PUPD13_1;

    button_stable_state = BUTTON_NOT_PRESSED;
    button_last_raw_state = BUTTON_NOT_PRESSED;
    button_debounce_count = 0U;
    button_pressed_event = 0U;
}

static uint8_t Button_ReadRaw(void)
{
    if ((GPIOC->IDR & GPIO_IDR_ID13) != 0U)
    {
        return BUTTON_PRESSED;
    }

    return BUTTON_NOT_PRESSED;
}

void Button_Update_1ms(void)
{
    uint8_t raw_state = Button_ReadRaw();

    if (raw_state != button_last_raw_state)
    {
        button_last_raw_state = raw_state;
        button_debounce_count = 0U;
    }
    else
    {
        if (button_debounce_count < BUTTON_DEBOUNCE_MS)
        {
            button_debounce_count++;
        }
        else if (button_stable_state != raw_state)
        {
            button_stable_state = raw_state;

            if (button_stable_state == BUTTON_PRESSED)
            {
                button_pressed_event = 1U;
            }
        }
        else
        {
            /* Stable, no event. */
        }
    }
}

uint8_t Button_WasPressed(void)
{
    if (button_pressed_event != 0U)
    {
        button_pressed_event = 0U;
        return 1U;
    }

    return 0U;
}

void setup_TIM7_ButtonPoll(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;

    /* 48 MHz -> 1 MHz, then 1 kHz update interrupt. */
    TIM7->PSC = 47U;
    TIM7->ARR = 999U;
    TIM7->CNT = 0U;

    TIM7->SR &= ~TIM_SR_UIF;
    TIM7->DIER |= TIM_DIER_UIE;

    NVIC->ISER[1] |= (1U << (TIM7_IRQn & 0x1FU));

    TIM7->CR1 |= TIM_CR1_CEN;
}

void TIM7_IRQHandler(void)
{
    if ((TIM7->SR & TIM_SR_UIF) != 0U)
    {
        TIM7->SR &= ~TIM_SR_UIF;
        Button_Update_1ms();
    }
}

/* -------------------------------------------------------------------------- */
/* HMI encoder                                                                */
/* -------------------------------------------------------------------------- */

#define HMI_LOCAL_PPR             600
#define HMI_LOCAL_COUNTS_PER_REV  (HMI_LOCAL_PPR * 4)

void HMI_Encoder_Config(void)
{
    /* 1. Enable clocks. */
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;

    /* 2. PB6 (TI1), PB7 (TI2) -> AF2 (TIM4), pull-up. */
    GPIOB->MODER   &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOB->MODER   |=  (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
    GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
    GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
    GPIOB->PUPDR   |=  (GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0);

    GPIOB->AFR[0] &= ~((0xFU << GPIO_AFRL_AFSEL6_Pos) |
                       (0xFU << GPIO_AFRL_AFSEL7_Pos));
    GPIOB->AFR[0] |=  ((0x2U << GPIO_AFRL_AFSEL6_Pos) |
                       (0x2U << GPIO_AFRL_AFSEL7_Pos));

    /* 3. Encoder mode x4. */
    TIM4->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_SMS_3);
    TIM4->SMCR |=  (3U << TIM_SMCR_SMS_Pos);

    /* 4. TI1->IC1, TI2->IC2, light input filter. */
    TIM4->CCMR1 = 0U;
    TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM4->CCMR1 |= TIM_CCMR1_CC2S_0;
    TIM4->CCMR1 |= (0x2U << TIM_CCMR1_IC1F_Pos);
    TIM4->CCMR1 |= (0x2U << TIM_CCMR1_IC2F_Pos);

    /* 5. Active-high polarity, enable capture inputs. */
    TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P |
                    TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM4->CCER |=  (TIM_CCER_CC1E | TIM_CCER_CC2E);

    /* 6. Full 16-bit range. */
    TIM4->ARR = 0xFFFFU;
    TIM4->CNT = 0U;

    /* 7. Generate update, clear spurious UIF, start. */
    TIM4->EGR |= TIM_EGR_UG;
    TIM4->SR  &= ~TIM_SR_UIF;
    TIM4->CR1 |= TIM_CR1_CEN;
}

int32_t HMI_Encoder_GetCount(void)
{
    return (int32_t)(int16_t)TIM4->CNT;
}

float HMI_Encoder_GetRevolutions(void)
{
    return (float)HMI_Encoder_GetCount() / (float)HMI_LOCAL_COUNTS_PER_REV;
}

float HMI_Encoder_GetDegrees(void)
{
    return HMI_Encoder_GetRevolutions() * 360.0f;
}

/* -------------------------------------------------------------------------- */
/* Object-like UI item backend                                                */
/* -------------------------------------------------------------------------- */

#define HMI_ENCODER_COUNTS_PER_STEP   400
#define HMI_VALUE_REFRESH_MS          200U

#define HMI_BG                        COLOR_WHITE
#define HMI_FG                        COLOR_BLACK
#define HMI_DIM                       COLOR_GRAY
#define HMI_SELECTED                  COLOR_YELLOW
#define HMI_TEXT_SCALE                1U
#define HMI_TEXT_CHAR_W               6U
#define HMI_BUTTON_TEXT_Y_OFFSET      8U
#define HMI_BUTTON_BORDER_THICKNESS   2U
#define HMI_FIELD_BORDER_THICKNESS    1U

/* These coordinates intentionally match control_loop_display.c. */
#define MODE_BOX_Y                    19U
#define MODE_BOX_W                    74U
#define MODE_BOX_H                    24U
#define MODE_BOX_GAP                  5U
#define MODE_SLOT_0_X                 4U
#define MODE_SLOT_1_X                 (MODE_SLOT_0_X + MODE_BOX_W + MODE_BOX_GAP)
#define MODE_SLOT_2_X                 (MODE_SLOT_1_X + MODE_BOX_W + MODE_BOX_GAP)

#define BLOCK_Y                       84U
#define BLOCK_H                       70U
#define INPUT_X                       4U
#define INPUT_W                       64U
#define POS_X                         91U
#define POS_W                         68U
#define VEL_X                         183U
#define VEL_W                         68U

#define VALUE_ROW_0                   (BLOCK_Y + 27U)
#define VALUE_ROW_1                   (BLOCK_Y + 39U)
#define VALUE_ROW_2                   (BLOCK_Y + 51U)
#define VALUE_ROW_3                   (BLOCK_Y + 61U)

#define FIELD_TEXT_Y_OFFSET           2U
#define FIELD_TEXT_X_OFFSET           2U
#define FIELD_H                       11U
#define FIELD_Y(row)                  ((uint16_t)((row) - FIELD_TEXT_Y_OFFSET))
#define INPUT_FIELD_X                 (INPUT_X + 3U)
#define INPUT_FIELD_W                 (INPUT_W - 6U)
#define POS_FIELD_X                   (POS_X + 3U)
#define POS_FIELD_W                   (POS_W - 6U)
#define VEL_FIELD_X                   (VEL_X + 3U)
#define VEL_FIELD_W                   (VEL_W - 6U)

#define INPUT_TEXT_CHARS              8U
#define PI_TEXT_CHARS                 10U

#define HMI_INVALID_INDEX             0xFFU

#define HMI_INDEX_VELOCITY            0U
#define HMI_INDEX_POSITION            1U
#define HMI_INDEX_TRACK               2U

#define HMI_FLOAT_MIN_KP              0.0f
#define HMI_FLOAT_MAX_KP              200.0f
#define HMI_FLOAT_MIN_KI              0.0f
#define HMI_FLOAT_MAX_KI              50.0f
#define HMI_FLOAT_STEP_GAIN           0.10f

typedef enum
{
    UI_ITEM_BUTTON = 0,
    UI_ITEM_FLOAT_ENTRY,
    UI_ITEM_READOUT
} UiItemType_t;

typedef struct UiItem UiItem_t;

typedef void (*UiRenderFn_t)(UiItem_t *self);
typedef void (*UiPressFn_t)(UiItem_t *self);
typedef void (*UiEncoderFn_t)(UiItem_t *self, int32_t steps);
typedef void (*UiFormatFn_t)(UiItem_t *self, char *dst, size_t dst_len);

typedef struct
{
    volatile float *value;
    float min_value;
    float max_value;
    float step;
    uint8_t decimals;
} HmiFloatEntry_t;

struct UiItem
{
    UiItemType_t type;

    const char *label;

    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;

    uint8_t selectable;
    uint8_t is_selected;
    uint8_t is_on;
    uint8_t text_chars;

    uint16_t active_color;

    void *data;
    UiFormatFn_t format;

    UiRenderFn_t render;
    UiPressFn_t on_press;
    UiEncoderFn_t on_encoder;
};

static void HMI_ButtonRender(UiItem_t *self);
static void HMI_ButtonPress(UiItem_t *self);
static void HMI_RenderModeButtons(void);
static uint8_t HMI_SyncButtonStates(void);
static void HMI_SyncTrackingLock(void);
static void HMI_SelectIndex(uint8_t new_index);
static void HMI_RequestVelocityMode(void);
static void HMI_RequestPositionMode(void);
static void HMI_RequestTrackToggle(void);
static void HMI_FloatEntryRender(UiItem_t *self);
static void HMI_FloatEntryPress(UiItem_t *self);
static void HMI_FloatEntryEncoder(UiItem_t *self, int32_t steps);
static void HMI_ReadoutRender(UiItem_t *self);

static void HMI_FormatFloatEntry(UiItem_t *self, char *dst, size_t dst_len);
static void HMI_FormatInputSource(UiItem_t *self, char *dst, size_t dst_len);
static void HMI_FormatPosFs(UiItem_t *self, char *dst, size_t dst_len);
static void HMI_FormatVelFs(UiItem_t *self, char *dst, size_t dst_len);

static HmiFloatEntry_t hmi_pos_kp = { &ctx_pos.kp, HMI_FLOAT_MIN_KP, HMI_FLOAT_MAX_KP, HMI_FLOAT_STEP_GAIN, 2U };
static HmiFloatEntry_t hmi_pos_ki = { &ctx_pos.ki, HMI_FLOAT_MIN_KI, HMI_FLOAT_MAX_KI, HMI_FLOAT_STEP_GAIN, 2U };
static HmiFloatEntry_t hmi_vel_kp = { &ctx_vel.kp, HMI_FLOAT_MIN_KP, HMI_FLOAT_MAX_KP, HMI_FLOAT_STEP_GAIN, 2U };
static HmiFloatEntry_t hmi_vel_ki = { &ctx_vel.ki, HMI_FLOAT_MIN_KI, HMI_FLOAT_MAX_KI, HMI_FLOAT_STEP_GAIN, 2U };

static UiItem_t hmi_items[] =
{
    {
        .type = UI_ITEM_BUTTON,
        .label = "VELOCITY",
        .x = MODE_SLOT_0_X,
        .y = MODE_BOX_Y,
        .w = MODE_BOX_W,
        .h = MODE_BOX_H,
        .selectable = 1U,
        .is_selected = 1U,
        .is_on = 0U,
        .text_chars = 0U,
        .active_color = COLOR_BLUE,
        .data = NULL,
        .format = NULL,
        .render = HMI_ButtonRender,
        .on_press = HMI_ButtonPress,
        .on_encoder = NULL
    },
    {
        .type = UI_ITEM_BUTTON,
        .label = "POSITION",
        .x = MODE_SLOT_1_X,
        .y = MODE_BOX_Y,
        .w = MODE_BOX_W,
        .h = MODE_BOX_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = 0U,
        .active_color = COLOR_RED,
        .data = NULL,
        .format = NULL,
        .render = HMI_ButtonRender,
        .on_press = HMI_ButtonPress,
        .on_encoder = NULL
    },
    {
        .type = UI_ITEM_BUTTON,
        .label = "TRACK",
        .x = MODE_SLOT_2_X,
        .y = MODE_BOX_Y,
        .w = MODE_BOX_W,
        .h = MODE_BOX_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = 0U,
        .active_color = COLOR_GREEN,
        .data = NULL,
        .format = NULL,
        .render = HMI_ButtonRender,
        .on_press = HMI_ButtonPress,
        .on_encoder = NULL
    },

    /* Input Select block source label. */
    {
        .type = UI_ITEM_READOUT,
        .label = "SRC",
        .x = INPUT_FIELD_X,
        .y = FIELD_Y(VALUE_ROW_1),
        .w = INPUT_FIELD_W,
        .h = FIELD_H,
        .selectable = 0U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = INPUT_TEXT_CHARS,
        .active_color = HMI_FG,
        .data = NULL,
        .format = HMI_FormatInputSource,
        .render = HMI_ReadoutRender,
        .on_press = NULL,
        .on_encoder = NULL
    },

    /* Position PI items. */
    {
        .type = UI_ITEM_FLOAT_ENTRY,
        .label = "Kp",
        .x = POS_FIELD_X,
        .y = FIELD_Y(VALUE_ROW_0),
        .w = POS_FIELD_W,
        .h = FIELD_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = PI_TEXT_CHARS,
        .active_color = COLOR_RED,
        .data = &hmi_pos_kp,
        .format = HMI_FormatFloatEntry,
        .render = HMI_FloatEntryRender,
        .on_press = HMI_FloatEntryPress,
        .on_encoder = HMI_FloatEntryEncoder
    },
    {
        .type = UI_ITEM_FLOAT_ENTRY,
        .label = "Ki",
        .x = POS_FIELD_X,
        .y = FIELD_Y(VALUE_ROW_1),
        .w = POS_FIELD_W,
        .h = FIELD_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = PI_TEXT_CHARS,
        .active_color = COLOR_RED,
        .data = &hmi_pos_ki,
        .format = HMI_FormatFloatEntry,
        .render = HMI_FloatEntryRender,
        .on_press = HMI_FloatEntryPress,
        .on_encoder = HMI_FloatEntryEncoder
    },
    {
        .type = UI_ITEM_READOUT,
        .label = "Fs",
        .x = POS_FIELD_X,
        .y = FIELD_Y(VALUE_ROW_2),
        .w = POS_FIELD_W,
        .h = FIELD_H,
        .selectable = 0U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = PI_TEXT_CHARS,
        .active_color = COLOR_RED,
        .data = NULL,
        .format = HMI_FormatPosFs,
        .render = HMI_ReadoutRender,
        .on_press = NULL,
        .on_encoder = NULL
    },

    /* Velocity PI items. */
    {
        .type = UI_ITEM_FLOAT_ENTRY,
        .label = "Kp",
        .x = VEL_FIELD_X,
        .y = FIELD_Y(VALUE_ROW_0),
        .w = VEL_FIELD_W,
        .h = FIELD_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = PI_TEXT_CHARS,
        .active_color = COLOR_BLUE,
        .data = &hmi_vel_kp,
        .format = HMI_FormatFloatEntry,
        .render = HMI_FloatEntryRender,
        .on_press = HMI_FloatEntryPress,
        .on_encoder = HMI_FloatEntryEncoder
    },
    {
        .type = UI_ITEM_FLOAT_ENTRY,
        .label = "Ki",
        .x = VEL_FIELD_X,
        .y = FIELD_Y(VALUE_ROW_1),
        .w = VEL_FIELD_W,
        .h = FIELD_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = PI_TEXT_CHARS,
        .active_color = COLOR_BLUE,
        .data = &hmi_vel_ki,
        .format = HMI_FormatFloatEntry,
        .render = HMI_FloatEntryRender,
        .on_press = HMI_FloatEntryPress,
        .on_encoder = HMI_FloatEntryEncoder
    },
    {
        .type = UI_ITEM_READOUT,
        .label = "Fs",
        .x = VEL_FIELD_X,
        .y = FIELD_Y(VALUE_ROW_2),
        .w = VEL_FIELD_W,
        .h = FIELD_H,
        .selectable = 0U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = PI_TEXT_CHARS,
        .active_color = COLOR_BLUE,
        .data = NULL,
        .format = HMI_FormatVelFs,
        .render = HMI_ReadoutRender,
        .on_press = NULL,
        .on_encoder = NULL
    }
};

#define HMI_ITEM_COUNT ((uint8_t)(sizeof(hmi_items) / sizeof(hmi_items[0])))

static uint8_t hmi_selected_index = 0U;
static uint8_t hmi_editing = 0U;
static uint8_t hmi_tracking_lock = 0U;
static int32_t hmi_last_encoder_count = 0;
static int32_t hmi_encoder_accum = 0;
static uint32_t hmi_last_value_refresh_ms = 0U;

static uint16_t HMI_TextWidth(const char *s)
{
    uint16_t n = 0U;

    if (s == NULL)
    {
        return 0U;
    }

    while (*s != '\0')
    {
        n++;
        s++;
    }

    return (uint16_t)(n * HMI_TEXT_CHAR_W * HMI_TEXT_SCALE);
}

static void HMI_DrawRectBorder(uint16_t x,
                               uint16_t y,
                               uint16_t w,
                               uint16_t h,
                               uint16_t color,
                               uint16_t thickness)
{
    ILI9341_fillRect(x, y, w, thickness, color);
    ILI9341_fillRect(x, (uint16_t)(y + h - thickness), w, thickness, color);
    ILI9341_fillRect(x, y, thickness, h, color);
    ILI9341_fillRect((uint16_t)(x + w - thickness), y, thickness, h, color);
}

static void HMI_FloatToStr(char *dst, size_t dst_len, float value, uint8_t decimals)
{
    int32_t scale = 1;

    for (uint8_t i = 0U; i < decimals; i++)
    {
        scale *= 10;
    }

    int32_t scaled;
    if (value >= 0.0f)
    {
        scaled = (int32_t)((value * (float)scale) + 0.5f);
    }
    else
    {
        scaled = (int32_t)((value * (float)scale) - 0.5f);
    }

    const char *sign = "";
    if (scaled < 0)
    {
        sign = "-";
        scaled = -scaled;
    }

    int32_t whole = scaled / scale;
    int32_t frac = scaled % scale;

    if (decimals == 0U)
    {
        (void)snprintf(dst, dst_len, "%s%ld", sign, (long)whole);
    }
    else if (decimals == 1U)
    {
        (void)snprintf(dst, dst_len, "%s%ld.%01ld", sign, (long)whole, (long)frac);
    }
    else if (decimals == 2U)
    {
        (void)snprintf(dst, dst_len, "%s%ld.%02ld", sign, (long)whole, (long)frac);
    }
    else
    {
        (void)snprintf(dst, dst_len, "%s%ld.%03ld", sign, (long)whole, (long)frac);
    }
}

static void HMI_RateToStr(char *dst, size_t dst_len, float hz)
{
    char n[16];

    if (hz >= 1000.0f)
    {
        HMI_FloatToStr(n, sizeof(n), hz / 1000.0f, 1U);
        (void)snprintf(dst, dst_len, "%skHz", n);
    }
    else
    {
        HMI_FloatToStr(n, sizeof(n), hz, 0U);
        (void)snprintf(dst, dst_len, "%sHz", n);
    }
}

static float HMI_TimerRateHz(TIM_TypeDef *tim)
{
    uint32_t psc = tim->PSC + 1U;
    uint32_t arr = tim->ARR + 1U;

    if ((psc == 0U) || (arr == 0U))
    {
        return 0.0f;
    }

    return ((float)SystemCoreClock) / ((float)psc * (float)arr);
}

static void HMI_PrintFixedWidth(uint16_t x,
                                uint16_t y,
                                const char *s,
                                uint8_t chars,
                                uint16_t fg,
                                uint16_t bg)
{
    char padded[32];
    uint8_t i;

    if (chars >= sizeof(padded))
    {
        chars = (uint8_t)(sizeof(padded) - 1U);
    }

    for (i = 0U; i < chars; i++)
    {
        if ((s != NULL) && (s[i] != '\0'))
        {
            padded[i] = s[i];
        }
        else
        {
            padded[i] = ' ';
        }
    }

    padded[chars] = '\0';

    ILI9341_printString(x, y, padded, fg, bg, HMI_TEXT_SCALE);
}

static void HMI_RenderItem(uint8_t index)
{
    if (index >= HMI_ITEM_COUNT)
    {
        return;
    }

    if (hmi_items[index].render != NULL)
    {
        hmi_items[index].render(&hmi_items[index]);
    }
}

static void HMI_RenderAllItems(void)
{
    for (uint8_t i = 0U; i < HMI_ITEM_COUNT; i++)
    {
        HMI_RenderItem(i);
    }
}

static void HMI_RenderReadouts(void)
{
    for (uint8_t i = 0U; i < HMI_ITEM_COUNT; i++)
    {
        if (hmi_items[i].type == UI_ITEM_READOUT)
        {
            HMI_RenderItem(i);
        }
    }
}

static void HMI_RenderModeButtons(void)
{
    HMI_RenderItem(HMI_INDEX_VELOCITY);
    HMI_RenderItem(HMI_INDEX_POSITION);
    HMI_RenderItem(HMI_INDEX_TRACK);
}

static uint8_t HMI_SyncButtonStates(void)
{
    uint8_t changed = 0U;

    /*
     * Velocity and position show desired_state so the user can choose the
     * startup tracking mode even while current_state is STATE_DISABLED.
     * TRACK shows whether tracking is currently active or pending.
     */
    uint8_t velocity_on = (desired_state == STATE_VELOCITY_CONTROL) ? 1U : 0U;
    uint8_t position_on = (desired_state == STATE_POSITION_CONTROL) ? 1U : 0U;
    uint8_t track_on = ((Control_IsTrackingEnabled() != 0U) ||
                        (hmi_tracking_lock != 0U)) ? 1U : 0U;

    if (hmi_items[HMI_INDEX_VELOCITY].is_on != velocity_on)
    {
        hmi_items[HMI_INDEX_VELOCITY].is_on = velocity_on;
        changed = 1U;
    }

    if (hmi_items[HMI_INDEX_POSITION].is_on != position_on)
    {
        hmi_items[HMI_INDEX_POSITION].is_on = position_on;
        changed = 1U;
    }

    if (hmi_items[HMI_INDEX_TRACK].is_on != track_on)
    {
        hmi_items[HMI_INDEX_TRACK].is_on = track_on;
        changed = 1U;
    }

    return changed;
}

static void HMI_SyncTrackingLock(void)
{
    if (Control_IsTrackingEnabled() != 0U)
    {
        if (hmi_tracking_lock == 0U)
        {
            hmi_tracking_lock = 1U;
            hmi_editing = 0U;
            HMI_SelectIndex(HMI_INDEX_TRACK);
        }
    }
    else
    {
        /*
         * Do not unlock while a tracking toggle request is still pending in
         * control.c. This prevents a fast UI_Task() pass from briefly giving
         * the encoder back to HMI before TIM6 processes the request.
         */
        if (tracking_toggle_request == 0U)
        {
            hmi_tracking_lock = 0U;
        }
    }
}

static void HMI_SelectIndex(uint8_t new_index)
{
    if ((new_index >= HMI_ITEM_COUNT) || (hmi_items[new_index].selectable == 0U))
    {
        return;
    }

    uint8_t old_index = hmi_selected_index;

    if (new_index == old_index)
    {
        hmi_items[hmi_selected_index].is_selected = 1U;
        HMI_RenderItem(hmi_selected_index);
        return;
    }

    hmi_items[old_index].is_selected = 0U;
    hmi_selected_index = new_index;
    hmi_items[hmi_selected_index].is_selected = 1U;

    HMI_RenderItem(old_index);
    HMI_RenderItem(hmi_selected_index);
}

static void HMI_RequestVelocityMode(void)
{
    Control_SetDesiredState(STATE_VELOCITY_CONTROL);
}

static void HMI_RequestPositionMode(void)
{
    Control_SetDesiredState(STATE_POSITION_CONTROL);
}

static void HMI_RequestTrackToggle(void)
{
    tracking_toggle_request = 1U;
    hmi_editing = 0U;
    HMI_SelectIndex(HMI_INDEX_TRACK);

    /*
     * If this press is an enable request, lock immediately so the next encoder
     * movement goes to the position target instead of scrolling the UI. If this
     * press is a disable request, stay locked until control.c processes the
     * request and current_state returns to STATE_DISABLED.
     */
    hmi_tracking_lock = 1U;
}

static void HMI_ButtonRender(UiItem_t *self)
{
    if (self == NULL)
    {
        return;
    }

    uint16_t fill_color = self->is_on ? self->active_color : HMI_BG;
    uint16_t text_color = self->is_on ? COLOR_WHITE : COLOR_BLACK;
    uint16_t border_color = self->is_selected ? HMI_SELECTED : HMI_DIM;

    ILI9341_fillRect(self->x, self->y, self->w, self->h, fill_color);

    HMI_DrawRectBorder(self->x,
                       self->y,
                       self->w,
                       self->h,
                       border_color,
                       HMI_BUTTON_BORDER_THICKNESS);

    uint16_t text_w = HMI_TextWidth(self->label);
    uint16_t text_x = self->x;

    if (self->w > text_w)
    {
        text_x = (uint16_t)(self->x + ((self->w - text_w) / 2U));
    }

    ILI9341_printString(text_x,
                        (uint16_t)(self->y + HMI_BUTTON_TEXT_Y_OFFSET),
                        self->label,
                        text_color,
                        fill_color,
                        HMI_TEXT_SCALE);
}

static void HMI_ButtonPress(UiItem_t *self)
{
    if (self == NULL)
    {
        return;
    }

    if (self == &hmi_items[HMI_INDEX_VELOCITY])
    {
        HMI_RequestVelocityMode();
    }
    else if (self == &hmi_items[HMI_INDEX_POSITION])
    {
        HMI_RequestPositionMode();
    }
    else if (self == &hmi_items[HMI_INDEX_TRACK])
    {
        HMI_RequestTrackToggle();
    }
    else
    {
        /* Unknown button object. */
    }

    (void)HMI_SyncButtonStates();
}

static void HMI_FormatFloatEntry(UiItem_t *self, char *dst, size_t dst_len)
{
    char num[16];
    HmiFloatEntry_t *entry = (HmiFloatEntry_t *)self->data;

    if ((entry == NULL) || (entry->value == NULL))
    {
        (void)snprintf(dst, dst_len, "%s=?", self->label);
        return;
    }

    HMI_FloatToStr(num, sizeof(num), *(entry->value), entry->decimals);
    (void)snprintf(dst, dst_len, "%s=%s", self->label, num);
}

static void HMI_FloatEntryRender(UiItem_t *self)
{
    char text[24];

    if (self == NULL)
    {
        return;
    }

    if (self->format != NULL)
    {
        self->format(self, text, sizeof(text));
    }
    else
    {
        (void)snprintf(text, sizeof(text), "%s", self->label);
    }

    uint8_t selected = self->is_selected;
    uint8_t editing_this = ((selected != 0U) && (hmi_editing != 0U)) ? 1U : 0U;

    uint16_t fill = editing_this ? HMI_SELECTED : HMI_BG;
    uint16_t fg = editing_this ? HMI_FG : HMI_FG;
    uint16_t border = selected ? HMI_SELECTED : HMI_DIM;

    ILI9341_fillRect(self->x, self->y, self->w, self->h, fill);

    HMI_DrawRectBorder(self->x,
                       self->y,
                       self->w,
                       self->h,
                       border,
                       HMI_FIELD_BORDER_THICKNESS);

    HMI_PrintFixedWidth((uint16_t)(self->x + FIELD_TEXT_X_OFFSET),
                        (uint16_t)(self->y + FIELD_TEXT_Y_OFFSET),
                        text,
                        self->text_chars,
                        fg,
                        fill);
}

static void HMI_FloatEntryPress(UiItem_t *self)
{
    (void)self;
    hmi_editing = hmi_editing ? 0U : 1U;
}

static void HMI_FloatEntryEncoder(UiItem_t *self, int32_t steps)
{
    if ((self == NULL) || (self->data == NULL) || (hmi_editing == 0U))
    {
        return;
    }

    HmiFloatEntry_t *entry = (HmiFloatEntry_t *)self->data;

    if (entry->value == NULL)
    {
        return;
    }

    float value = *(entry->value);
    value += ((float)steps * entry->step);

    if (value > entry->max_value)
    {
        value = entry->max_value;
    }
    else if (value < entry->min_value)
    {
        value = entry->min_value;
    }
    else
    {
        /* In range. */
    }

    *(entry->value) = value;
}

static void HMI_ReadoutRender(UiItem_t *self)
{
    char text[24];

    if (self == NULL)
    {
        return;
    }

    if (self->format != NULL)
    {
        self->format(self, text, sizeof(text));
    }
    else
    {
        (void)snprintf(text, sizeof(text), "%s", self->label);
    }

    ILI9341_fillRect(self->x, self->y, self->w, self->h, HMI_BG);

    HMI_PrintFixedWidth((uint16_t)(self->x + FIELD_TEXT_X_OFFSET),
                        (uint16_t)(self->y + FIELD_TEXT_Y_OFFSET),
                        text,
                        self->text_chars,
                        self->active_color,
                        HMI_BG);
}

static void HMI_FormatInputSource(UiItem_t *self, char *dst, size_t dst_len)
{
    (void)self;
    (void)snprintf(dst, dst_len, "SRC:ENC");
}

static void HMI_FormatPosFs(UiItem_t *self, char *dst, size_t dst_len)
{
    (void)self;

    char rate[16];
    HMI_RateToStr(rate, sizeof(rate), HMI_TimerRateHz(TIM6));
    (void)snprintf(dst, dst_len, "Fs=%s", rate);
}

static void HMI_FormatVelFs(UiItem_t *self, char *dst, size_t dst_len)
{
    (void)self;

    char rate[16];
    HMI_RateToStr(rate, sizeof(rate), HMI_TimerRateHz(TIM5));
    (void)snprintf(dst, dst_len, "Fs=%s", rate);
}

static uint8_t HMI_FindNextSelectable(uint8_t start_index, int8_t direction)
{
    uint8_t index = start_index;

    for (uint8_t i = 0U; i < HMI_ITEM_COUNT; i++)
    {
        if (direction > 0)
        {
            index++;
            if (index >= HMI_ITEM_COUNT)
            {
                index = 0U;
            }
        }
        else
        {
            if (index == 0U)
            {
                index = (uint8_t)(HMI_ITEM_COUNT - 1U);
            }
            else
            {
                index--;
            }
        }

        if (hmi_items[index].selectable != 0U)
        {
            return index;
        }
    }

    return start_index;
}

static void HMI_MoveSelectionBySteps(int32_t steps)
{
    if (steps == 0)
    {
        return;
    }

    uint8_t new_index = hmi_selected_index;

    int8_t direction = (steps > 0) ? 1 : -1;
    int32_t count = (steps > 0) ? steps : -steps;

    while (count > 0)
    {
        new_index = HMI_FindNextSelectable(new_index, direction);
        count--;
    }

    HMI_SelectIndex(new_index);
}

static void HMI_PressSelected(void)
{
    UiItem_t *selected = &hmi_items[hmi_selected_index];

    if (selected->on_press != NULL)
    {
        selected->on_press(selected);
    }

    if (selected->type == UI_ITEM_BUTTON)
    {
        HMI_RenderModeButtons();
    }
    else
    {
        HMI_RenderItem(hmi_selected_index);
    }
}

static void HMI_HandleEncoderSteps(int32_t steps)
{
    if (steps == 0)
    {
        return;
    }

    UiItem_t *selected = &hmi_items[hmi_selected_index];

    if ((hmi_editing != 0U) && (selected->on_encoder != NULL))
    {
        selected->on_encoder(selected, steps);
        HMI_RenderItem(hmi_selected_index);
    }
    else
    {
        HMI_MoveSelectionBySteps(steps);
    }
}

static void HMI_ProcessEncoder(void)
{
    int32_t current_count = HMI_Encoder_GetCount();
    int32_t delta = current_count - hmi_last_encoder_count;

    hmi_last_encoder_count = current_count;

    if (hmi_tracking_lock != 0U)
    {
        /*
         * While TRACK is enabled, the physical HMI encoder belongs to the
         * position setpoint path in control.c. Drain movement here so the UI
         * does not build up a scroll backlog and does not leave TRACK selected.
         */
        hmi_encoder_accum = 0;
        return;
    }

    hmi_encoder_accum += delta;

    int32_t steps = hmi_encoder_accum / HMI_ENCODER_COUNTS_PER_STEP;

    if (steps != 0)
    {
        hmi_encoder_accum -= (steps * HMI_ENCODER_COUNTS_PER_STEP);
        HMI_HandleEncoderSteps(steps);
    }
}

static void HMI_ClearSelectionState(void)
{
    for (uint8_t i = 0U; i < HMI_ITEM_COUNT; i++)
    {
        hmi_items[i].is_selected = 0U;
    }
}

static uint8_t HMI_FirstSelectableIndex(void)
{
    for (uint8_t i = 0U; i < HMI_ITEM_COUNT; i++)
    {
        if (hmi_items[i].selectable != 0U)
        {
            return i;
        }
    }

    return HMI_INVALID_INDEX;
}

void UI_Init(void)
{
    /* The control-loop display uses rotation 1. Keep the same coordinate frame. */
    ILI9341_setRotation(1U);

    hmi_last_encoder_count = HMI_Encoder_GetCount();
    hmi_encoder_accum = 0;
    hmi_editing = 0U;
    hmi_tracking_lock = 0U;
    hmi_last_value_refresh_ms = HAL_GetTick();

    HMI_ClearSelectionState();

    hmi_selected_index = HMI_FirstSelectableIndex();
    if (hmi_selected_index != HMI_INVALID_INDEX)
    {
        hmi_items[hmi_selected_index].is_selected = 1U;
    }
    else
    {
        hmi_selected_index = 0U;
    }

    (void)HMI_SyncButtonStates();
    HMI_RenderAllItems();
}

void UI_Task(void)
{
    HMI_SyncTrackingLock();
    HMI_ProcessEncoder();

    if (HMI_SyncButtonStates() != 0U)
    {
        HMI_RenderModeButtons();
    }

    if (Button_WasPressed() != 0U)
    {
        HMI_PressSelected();
    }

    uint32_t now = HAL_GetTick();
    if ((now - hmi_last_value_refresh_ms) >= HMI_VALUE_REFRESH_MS)
    {
        hmi_last_value_refresh_ms = now;
        HMI_RenderReadouts();
    }
}

void UI_Update(UiEvent_t event)
{
    switch (event)
    {
        case UI_EVENT_UP:
            HMI_HandleEncoderSteps(-1);
            break;

        case UI_EVENT_DOWN:
            HMI_HandleEncoderSteps(1);
            break;

        case UI_EVENT_SELECT:
            HMI_PressSelected();
            break;

        case UI_EVENT_NONE:
        default:
            break;
    }
}

void UI_Draw(void)
{
    HMI_RenderAllItems();
}
