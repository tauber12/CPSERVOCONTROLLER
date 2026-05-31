/*
 *******************************************************************************
 * @file           : HMI.c
 * @brief          : TFT HMI, user encoder, presets, input selection, and plots
 *******************************************************************************
 */

#include "HMI.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "ILI9341_text.h"
#include "control.h"
#include "control_loop_display.h"

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
    return ((GPIOC->IDR & GPIO_IDR_ID13) != 0U) ? BUTTON_PRESSED : BUTTON_NOT_PRESSED;
}

void Button_Update_1ms(void)
{
    uint8_t raw_state = Button_ReadRaw();

    if (raw_state != button_last_raw_state)
    {
        button_last_raw_state = raw_state;
        button_debounce_count = 0U;
    }
    else if (button_debounce_count < BUTTON_DEBOUNCE_MS)
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
    TIM7->PSC = 47U;
    TIM7->ARR = 999U;
    TIM7->CNT = 0U;
    TIM7->SR &= ~TIM_SR_UIF;
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC->ISER[((uint32_t)TIM7_IRQn >> 5U)] |= (1U << ((uint32_t)TIM7_IRQn & 0x1FU));
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
/* HMI/user encoder                                                           */
/* -------------------------------------------------------------------------- */

static volatile int32_t hmi_encoder_overflow_count = 0;

static void HMI_Encoder_ServiceOverflow(void)
{
    if ((TIM4->SR & TIM_SR_UIF) != 0U)
    {
        uint16_t cnt = (uint16_t)TIM4->CNT;
        TIM4->SR &= ~TIM_SR_UIF;

        /* In encoder mode, an upward overflow leaves CNT near 0, while a
         * downward underflow leaves CNT near ARR. Use CNT rather than only DIR
         * so a direction change immediately after the wrap does not corrupt the
         * extended count. */
        if (cnt < 0x8000U)
        {
            hmi_encoder_overflow_count += HMI_ENCODER_TIMER_COUNTS;
        }
        else
        {
            hmi_encoder_overflow_count -= HMI_ENCODER_TIMER_COUNTS;
        }
    }
}

void TIM4_IRQHandler(void)
{
    HMI_Encoder_ServiceOverflow();
}

void HMI_Encoder_Config(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;

    GPIOB->MODER   &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOB->MODER   |=  (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
    GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
    GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
    GPIOB->PUPDR   |=  (GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0);

    GPIOB->AFR[0] &= ~((0xFU << GPIO_AFRL_AFSEL6_Pos) |
                       (0xFU << GPIO_AFRL_AFSEL7_Pos));
    GPIOB->AFR[0] |=  ((0x2U << GPIO_AFRL_AFSEL6_Pos) |
                       (0x2U << GPIO_AFRL_AFSEL7_Pos));

    TIM4->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_SMS_3);
    TIM4->SMCR |=  (3U << TIM_SMCR_SMS_Pos);

    TIM4->CCMR1 = 0U;
    TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM4->CCMR1 |= TIM_CCMR1_CC2S_0;
    TIM4->CCMR1 |= (0x6U << TIM_CCMR1_IC1F_Pos);
    TIM4->CCMR1 |= (0x6U << TIM_CCMR1_IC2F_Pos);

    TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P |
                    TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM4->CCER |=  (TIM_CCER_CC1E | TIM_CCER_CC2E);

    TIM4->ARR = 0xFFFFU;
    TIM4->CNT = 0U;
    TIM4->EGR |= TIM_EGR_UG;
    TIM4->SR  &= ~TIM_SR_UIF;
    TIM4->DIER |= TIM_DIER_UIE;

    hmi_encoder_overflow_count = 0;
    NVIC->ISER[((uint32_t)TIM4_IRQn >> 5U)] |= (1U << ((uint32_t)TIM4_IRQn & 0x1FU));
    TIM4->CR1 |= TIM_CR1_CEN;
}

void HMI_Encoder_ResetCount(void)
{
    TIM4->CR1 &= ~TIM_CR1_CEN;
    hmi_encoder_overflow_count = 0;
    TIM4->CNT = 0U;
    TIM4->SR &= ~TIM_SR_UIF;
    TIM4->CR1 |= TIM_CR1_CEN;
}

int32_t HMI_Encoder_GetCount(void)
{
    HMI_Encoder_ServiceOverflow();
    return hmi_encoder_overflow_count + (int32_t)((uint16_t)TIM4->CNT);
}

float HMI_Encoder_GetRevolutions(void)
{
    return (float)HMI_Encoder_GetCount() / HMI_LOCAL_COUNTS_PER_REV;
}

float HMI_Encoder_GetDegrees(void)
{
    return HMI_Encoder_GetRevolutions() * 360.0f;
}

/* -------------------------------------------------------------------------- */
/* UI constants and types                                                     */
/* -------------------------------------------------------------------------- */

#define HMI_BG                         COLOR_WHITE
#define HMI_FG                         COLOR_BLACK
#define HMI_DIM                        COLOR_GRAY
#define HMI_SELECTED                   COLOR_YELLOW
#define HMI_TEXT_SCALE                 1U
#define HMI_CHAR_W                     6U
#define HMI_BUTTON_BORDER              2U
#define HMI_FIELD_BORDER               1U

#define MODE_BOX_Y                     19U
#define MODE_BOX_W                     56U
#define MODE_BOX_H                     24U
#define MODE_GAP                       4U
#define MODE_SLOT_0_X                  4U
#define MODE_SLOT_1_X                  (MODE_SLOT_0_X + MODE_BOX_W + MODE_GAP)
#define MODE_SLOT_2_X                  (MODE_SLOT_1_X + MODE_BOX_W + MODE_GAP)

#define PRESET_BOX_Y                   MODE_BOX_Y
#define PRESET_BOX_W                   31U
#define PRESET_BOX_H                   MODE_BOX_H
#define PRESET_SLOT_0_X                252U
#define PRESET_SLOT_1_X                286U

#define BLOCK_Y                        84U
#define INPUT_X                        4U
#define INPUT_W                        64U
#define POS_X                          91U
#define POS_W                          68U
#define VEL_X                          183U
#define VEL_W                          68U

#define VALUE_ROW_0                    (BLOCK_Y + 27U)
#define VALUE_ROW_1                    (BLOCK_Y + 39U)
#define VALUE_ROW_2                    (BLOCK_Y + 51U)
#define VALUE_ROW_3                    (BLOCK_Y + 62U)
#define FIELD_H                        11U
#define FIELD_Y(row)                   ((uint16_t)((row) - 2U))
#define FIELD_X_OFFSET                 2U
#define FIELD_Y_OFFSET                 2U

#define INPUT_FIELD_X                  (INPUT_X + 3U)
#define INPUT_FIELD_W                  (INPUT_W - 6U)
#define POS_FIELD_X                    (POS_X + 3U)
#define POS_FIELD_W                    (POS_W - 6U)
#define VEL_FIELD_X                    (VEL_X + 3U)
#define VEL_FIELD_W                    (VEL_W - 6U)

#define HMI_FLOAT_MIN_KP               0.0f
#define HMI_FLOAT_MAX_KP               200.0f
#define HMI_FLOAT_MIN_KI               0.0f
#define HMI_FLOAT_MAX_KI               50.0f
#define HMI_FLOAT_STEP_GAIN            0.10f
#define HMI_FLOAT_STEP_RANGE           10.0f
#define HMI_FLOAT_STEP_POS_FS          50.0f
#define HMI_FLOAT_STEP_VEL_FS          100.0f
#define HMI_FLOAT_STEP_FREQ            0.05f
#define HMI_FLOAT_STEP_AMP             5.0f

#define HMI_INVALID_INDEX              0xFFU

typedef enum
{
    HMI_PAGE_DIAGRAM = 0,
    HMI_PAGE_PLOT,
    HMI_PAGE_PRESET
} HmiPage_t;

typedef enum
{
    UI_ITEM_BUTTON = 0,
    UI_ITEM_FLOAT_ENTRY,
    UI_ITEM_INPUT_SELECT,
    UI_ITEM_PRESET_BUTTON
} UiItemType_t;

typedef struct UiItem UiItem_t;
typedef void (*UiRenderFn_t)(UiItem_t *self);
typedef void (*UiPressFn_t)(UiItem_t *self);
typedef void (*UiEncoderFn_t)(UiItem_t *self, int32_t steps);
typedef void (*UiFormatFn_t)(UiItem_t *self, char *dst, size_t dst_len);
typedef float (*HmiFloatApplyFn_t)(float value);

typedef struct
{
    volatile float *value;
    float min_value;
    float max_value;
    float step;
    uint8_t decimals;
    HmiFloatApplyFn_t apply;
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
static void HMI_FloatEntryRender(UiItem_t *self);
static void HMI_FloatEntryPress(UiItem_t *self);
static void HMI_FloatEntryEncoder(UiItem_t *self, int32_t steps);
static void HMI_InputSelectRender(UiItem_t *self);
static void HMI_InputSelectPress(UiItem_t *self);
static void HMI_InputSelectEncoder(UiItem_t *self, int32_t steps);
static void HMI_PresetButtonPress(UiItem_t *self);
static void HMI_FormatFloatEntry(UiItem_t *self, char *dst, size_t dst_len);
static void HMI_FormatInputSource(UiItem_t *self, char *dst, size_t dst_len);
static void HMI_FormatPreset(UiItem_t *self, char *dst, size_t dst_len);

static float HMI_ApplyInputAmplitude(float value);

static HmiFloatEntry_t hmi_input_freq = { &control_input_wave_freq_hz, CONTROL_INPUT_MIN_FREQ_HZ, CONTROL_INPUT_MAX_FREQ_HZ, HMI_FLOAT_STEP_FREQ, 2U, Control_SetInputWaveFrequencyHz };
static HmiFloatEntry_t hmi_input_amp  = { &control_input_wave_amplitude, CONTROL_INPUT_MIN_AMPLITUDE, CONTROL_MAX_POS_RANGE_RPM, HMI_FLOAT_STEP_AMP, 0U, HMI_ApplyInputAmplitude };
static HmiFloatEntry_t hmi_pos_kp     = { &ctx_pos.kp, HMI_FLOAT_MIN_KP, HMI_FLOAT_MAX_KP, HMI_FLOAT_STEP_GAIN, 2U, NULL };
static HmiFloatEntry_t hmi_pos_ki     = { &ctx_pos.ki, HMI_FLOAT_MIN_KI, HMI_FLOAT_MAX_KI, HMI_FLOAT_STEP_GAIN, 2U, NULL };
static HmiFloatEntry_t hmi_pos_rng    = { &ctx_pos.output_range, 1.0f, CONTROL_MAX_POS_RANGE_RPM, HMI_FLOAT_STEP_RANGE, 0U, Control_SetPositionOutputRange };
static HmiFloatEntry_t hmi_pos_fs     = { &control_position_loop_hz, CONTROL_MIN_POS_HZ, CONTROL_MAX_POS_HZ, HMI_FLOAT_STEP_POS_FS, 0U, Control_SetPositionLoopHz };
static HmiFloatEntry_t hmi_vel_kp     = { &ctx_vel.kp, HMI_FLOAT_MIN_KP, HMI_FLOAT_MAX_KP, HMI_FLOAT_STEP_GAIN, 2U, NULL };
static HmiFloatEntry_t hmi_vel_ki     = { &ctx_vel.ki, HMI_FLOAT_MIN_KI, HMI_FLOAT_MAX_KI, HMI_FLOAT_STEP_GAIN, 2U, NULL };
static HmiFloatEntry_t hmi_vel_rng    = { &ctx_vel.output_range, 1.0f, CONTROL_MAX_VEL_RANGE_PWM, 1.0f, 0U, Control_SetVelocityOutputRange };
static HmiFloatEntry_t hmi_vel_fs     = { &control_velocity_loop_hz, CONTROL_MIN_VEL_HZ, CONTROL_MAX_VEL_HZ, HMI_FLOAT_STEP_VEL_FS, 0U, Control_SetVelocityLoopHz };

enum
{
    HMI_INDEX_VELOCITY = 0,
    HMI_INDEX_POSITION,
    HMI_INDEX_START,
    HMI_INDEX_PRESET1,
    HMI_INDEX_PRESET2,
    HMI_INDEX_INPUT_SOURCE,
    HMI_INDEX_INPUT_FREQ,
    HMI_INDEX_INPUT_AMP,
    HMI_INDEX_POS_KP,
    HMI_INDEX_POS_KI,
    HMI_INDEX_POS_RANGE,
    HMI_INDEX_POS_FS,
    HMI_INDEX_VEL_KP,
    HMI_INDEX_VEL_KI,
    HMI_INDEX_VEL_RANGE,
    HMI_INDEX_VEL_FS,
    HMI_ITEM_COUNT
};

static UiItem_t hmi_items[HMI_ITEM_COUNT] =
{
    [HMI_INDEX_VELOCITY] = { UI_ITEM_BUTTON, "VELOCITY", MODE_SLOT_0_X, MODE_BOX_Y, MODE_BOX_W, MODE_BOX_H, 1U, 1U, 0U, 0U, COLOR_BLUE, NULL, NULL, HMI_ButtonRender, HMI_ButtonPress, NULL },
    [HMI_INDEX_POSITION] = { UI_ITEM_BUTTON, "POSITION", MODE_SLOT_1_X, MODE_BOX_Y, MODE_BOX_W, MODE_BOX_H, 1U, 0U, 0U, 0U, COLOR_RED, NULL, NULL, HMI_ButtonRender, HMI_ButtonPress, NULL },
    [HMI_INDEX_START]    = { UI_ITEM_BUTTON, "START",    MODE_SLOT_2_X, MODE_BOX_Y, MODE_BOX_W, MODE_BOX_H, 1U, 0U, 0U, 0U, COLOR_GREEN, NULL, NULL, HMI_ButtonRender, HMI_ButtonPress, NULL },
    [HMI_INDEX_PRESET1]  = { UI_ITEM_PRESET_BUTTON, "P1", PRESET_SLOT_0_X, PRESET_BOX_Y, PRESET_BOX_W, PRESET_BOX_H, 1U, 0U, 0U, 0U, COLOR_CYAN, (void *)0U, HMI_FormatPreset, HMI_ButtonRender, HMI_PresetButtonPress, NULL },
    [HMI_INDEX_PRESET2]  = { UI_ITEM_PRESET_BUTTON, "P2", PRESET_SLOT_1_X, PRESET_BOX_Y, PRESET_BOX_W, PRESET_BOX_H, 1U, 0U, 0U, 0U, COLOR_CYAN, (void *)1U, HMI_FormatPreset, HMI_ButtonRender, HMI_PresetButtonPress, NULL },

    [HMI_INDEX_INPUT_SOURCE] = { UI_ITEM_INPUT_SELECT, "SRC", INPUT_FIELD_X, FIELD_Y(VALUE_ROW_1), INPUT_FIELD_W, FIELD_H, 1U, 0U, 0U, 9U, COLOR_ORANGE, NULL, HMI_FormatInputSource, HMI_InputSelectRender, HMI_InputSelectPress, HMI_InputSelectEncoder },
    [HMI_INDEX_INPUT_FREQ]   = { UI_ITEM_FLOAT_ENTRY, "Fq", INPUT_FIELD_X, FIELD_Y(VALUE_ROW_2), INPUT_FIELD_W, FIELD_H, 1U, 0U, 0U, 9U, COLOR_ORANGE, &hmi_input_freq, HMI_FormatFloatEntry, HMI_FloatEntryRender, HMI_FloatEntryPress, HMI_FloatEntryEncoder },
    [HMI_INDEX_INPUT_AMP]    = { UI_ITEM_FLOAT_ENTRY, "Amp", INPUT_FIELD_X, FIELD_Y(VALUE_ROW_3), INPUT_FIELD_W, FIELD_H, 1U, 0U, 0U, 9U, COLOR_ORANGE, &hmi_input_amp, HMI_FormatFloatEntry, HMI_FloatEntryRender, HMI_FloatEntryPress, HMI_FloatEntryEncoder },

    [HMI_INDEX_POS_KP]       = { UI_ITEM_FLOAT_ENTRY, "Kp",  POS_FIELD_X, FIELD_Y(VALUE_ROW_0), POS_FIELD_W, FIELD_H, 1U, 0U, 0U, 10U, COLOR_RED, &hmi_pos_kp, HMI_FormatFloatEntry, HMI_FloatEntryRender, HMI_FloatEntryPress, HMI_FloatEntryEncoder },
    [HMI_INDEX_POS_KI]       = { UI_ITEM_FLOAT_ENTRY, "Ki",  POS_FIELD_X, FIELD_Y(VALUE_ROW_1), POS_FIELD_W, FIELD_H, 1U, 0U, 0U, 10U, COLOR_RED, &hmi_pos_ki, HMI_FormatFloatEntry, HMI_FloatEntryRender, HMI_FloatEntryPress, HMI_FloatEntryEncoder },
    [HMI_INDEX_POS_RANGE]    = { UI_ITEM_FLOAT_ENTRY, "Rng", POS_FIELD_X, FIELD_Y(VALUE_ROW_2), POS_FIELD_W, FIELD_H, 1U, 0U, 0U, 10U, COLOR_RED, &hmi_pos_rng, HMI_FormatFloatEntry, HMI_FloatEntryRender, HMI_FloatEntryPress, HMI_FloatEntryEncoder },
    [HMI_INDEX_POS_FS]       = { UI_ITEM_FLOAT_ENTRY, "Fs",  POS_FIELD_X, FIELD_Y(VALUE_ROW_3), POS_FIELD_W, FIELD_H, 1U, 0U, 0U, 10U, COLOR_RED, &hmi_pos_fs, HMI_FormatFloatEntry, HMI_FloatEntryRender, HMI_FloatEntryPress, HMI_FloatEntryEncoder },

    [HMI_INDEX_VEL_KP]       = { UI_ITEM_FLOAT_ENTRY, "Kp",  VEL_FIELD_X, FIELD_Y(VALUE_ROW_0), VEL_FIELD_W, FIELD_H, 1U, 0U, 0U, 10U, COLOR_BLUE, &hmi_vel_kp, HMI_FormatFloatEntry, HMI_FloatEntryRender, HMI_FloatEntryPress, HMI_FloatEntryEncoder },
    [HMI_INDEX_VEL_KI]       = { UI_ITEM_FLOAT_ENTRY, "Ki",  VEL_FIELD_X, FIELD_Y(VALUE_ROW_1), VEL_FIELD_W, FIELD_H, 1U, 0U, 0U, 10U, COLOR_BLUE, &hmi_vel_ki, HMI_FormatFloatEntry, HMI_FloatEntryRender, HMI_FloatEntryPress, HMI_FloatEntryEncoder },
    [HMI_INDEX_VEL_RANGE]    = { UI_ITEM_FLOAT_ENTRY, "Rng", VEL_FIELD_X, FIELD_Y(VALUE_ROW_2), VEL_FIELD_W, FIELD_H, 1U, 0U, 0U, 10U, COLOR_BLUE, &hmi_vel_rng, HMI_FormatFloatEntry, HMI_FloatEntryRender, HMI_FloatEntryPress, HMI_FloatEntryEncoder },
    [HMI_INDEX_VEL_FS]       = { UI_ITEM_FLOAT_ENTRY, "Fs",  VEL_FIELD_X, FIELD_Y(VALUE_ROW_3), VEL_FIELD_W, FIELD_H, 1U, 0U, 0U, 10U, COLOR_BLUE, &hmi_vel_fs, HMI_FormatFloatEntry, HMI_FloatEntryRender, HMI_FloatEntryPress, HMI_FloatEntryEncoder }
};

static HmiPage_t hmi_page = HMI_PAGE_DIAGRAM;
static uint8_t hmi_selected_index = HMI_INDEX_VELOCITY;
static uint8_t hmi_editing = 0U;
static uint8_t hmi_tracking_lock = 0U;
static uint8_t hmi_layout_dirty = 0U;
static int32_t hmi_last_encoder_count = 0;
static int32_t hmi_encoder_accum = 0;
static uint32_t hmi_last_value_refresh_ms = 0U;
static uint32_t hmi_last_plot_refresh_ms = 0U;
static uint16_t hmi_plot_x = 0U;

static uint8_t hmi_preset_slot = 0U;
static uint8_t hmi_preset_selected = 0U;
static uint8_t hmi_preset_editing = 0U;

/* -------------------------------------------------------------------------- */
/* Small drawing/format helpers                                               */
/* -------------------------------------------------------------------------- */

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
    return (uint16_t)(n * HMI_CHAR_W * HMI_TEXT_SCALE);
}

static void HMI_DrawPixel(int16_t x, int16_t y, uint16_t color)
{
    if ((x < 0) || (y < 0) || (x >= 320) || (y >= 240))
    {
        return;
    }
    ILI9341_fillRect((uint16_t)x, (uint16_t)y, 1U, 1U, color);
}

static void HMI_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    int16_t dx = (x1 > x0) ? (int16_t)(x1 - x0) : (int16_t)(x0 - x1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t dy = (y1 > y0) ? (int16_t)(y0 - y1) : (int16_t)(y1 - y0);
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = (int16_t)(dx + dy);

    for (;;)
    {
        HMI_DrawPixel(x0, y0, color);
        if ((x0 == x1) && (y0 == y1))
        {
            break;
        }
        int16_t e2 = (int16_t)(2 * err);
        if (e2 >= dy)
        {
            err = (int16_t)(err + dy);
            x0 = (int16_t)(x0 + sx);
        }
        if (e2 <= dx)
        {
            err = (int16_t)(err + dx);
            y0 = (int16_t)(y0 + sy);
        }
    }
}

static void HMI_DrawRectBorder(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                               uint16_t color, uint16_t thickness)
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

    int32_t scaled = (value >= 0.0f) ?
        (int32_t)((value * (float)scale) + 0.5f) :
        (int32_t)((value * (float)scale) - 0.5f);

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
    else
    {
        (void)snprintf(dst, dst_len, "%s%ld.%02ld", sign, (long)whole, (long)frac);
    }
}

static void HMI_RateToStr(char *dst, size_t dst_len, float hz)
{
    char n[16];
    if (hz >= 1000.0f)
    {
        HMI_FloatToStr(n, sizeof(n), hz / 1000.0f, 1U);
        (void)snprintf(dst, dst_len, "%sk", n);
    }
    else
    {
        HMI_FloatToStr(n, sizeof(n), hz, 0U);
        (void)snprintf(dst, dst_len, "%s", n);
    }
}

static void HMI_PrintFixedWidth(uint16_t x, uint16_t y, const char *s,
                                uint8_t chars, uint16_t fg, uint16_t bg)
{
    char padded[32];
    if (chars >= sizeof(padded))
    {
        chars = (uint8_t)(sizeof(padded) - 1U);
    }

    for (uint8_t i = 0U; i < chars; i++)
    {
        padded[i] = ((s != NULL) && (s[i] != '\0')) ? s[i] : ' ';
    }
    padded[chars] = '\0';
    ILI9341_printString(x, y, padded, fg, bg, HMI_TEXT_SCALE);
}

static float HMI_WrapDeg180(float deg)
{
    deg = fmodf(deg, 360.0f);
    if (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    else if (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

static int16_t HMI_MapY(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        value = min_value;
    }
    else if (value > max_value)
    {
        value = max_value;
    }

    float span = max_value - min_value;
    if (span <= 0.0f)
    {
        return (int16_t)(HMI_PLOT_Y + (HMI_PLOT_H / 2U));
    }

    float frac = (value - min_value) / span;
    return (int16_t)((float)(HMI_PLOT_Y + HMI_PLOT_H - 2U) -
                     (frac * (float)(HMI_PLOT_H - 3U)));
}

/* -------------------------------------------------------------------------- */
/* UI item visibility and navigation                                          */
/* -------------------------------------------------------------------------- */

static uint8_t HMI_ItemIsVisible(uint8_t index)
{
    if (index >= HMI_ITEM_COUNT)
    {
        return 0U;
    }

    if ((index == HMI_INDEX_INPUT_FREQ) || (index == HMI_INDEX_INPUT_AMP))
    {
        return (control_input_source == CONTROL_INPUT_USER) ? 0U : 1U;
    }

    if ((index == HMI_INDEX_POS_KP) ||
        (index == HMI_INDEX_POS_KI) ||
        (index == HMI_INDEX_POS_RANGE) ||
        (index == HMI_INDEX_POS_FS))
    {
        return (desired_state == STATE_POSITION_CONTROL) ? 1U : 0U;
    }

    return 1U;
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
        if ((hmi_items[i].selectable != 0U) && (HMI_ItemIsVisible(i) != 0U))
        {
            return i;
        }
    }
    return HMI_INVALID_INDEX;
}

static void HMI_EnsureSelectedVisible(void)
{
    if ((hmi_selected_index < HMI_ITEM_COUNT) &&
        (hmi_items[hmi_selected_index].selectable != 0U) &&
        (HMI_ItemIsVisible(hmi_selected_index) != 0U))
    {
        hmi_items[hmi_selected_index].is_selected = 1U;
        return;
    }

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
}

static void HMI_ResetEncoderAccumulator(void)
{
    hmi_last_encoder_count = HMI_Encoder_GetCount();
    hmi_encoder_accum = 0;
}

static int32_t HMI_ConsumeEncoderSteps(void)
{
    int32_t count = HMI_Encoder_GetCount();
    int32_t delta = count - hmi_last_encoder_count;
    hmi_last_encoder_count = count;

    int32_t delta_limit = HMI_ENCODER_COUNTS_PER_STEP * HMI_ENCODER_MAX_STEPS_PER_TASK;
    if (delta > delta_limit)
    {
        delta = delta_limit;
    }
    else if (delta < -delta_limit)
    {
        delta = -delta_limit;
    }

    hmi_encoder_accum += delta;
    if (hmi_encoder_accum > delta_limit)
    {
        hmi_encoder_accum = delta_limit;
    }
    else if (hmi_encoder_accum < -delta_limit)
    {
        hmi_encoder_accum = -delta_limit;
    }

    int32_t steps = hmi_encoder_accum / HMI_ENCODER_COUNTS_PER_STEP;
    if (steps != 0)
    {
        hmi_encoder_accum -= steps * HMI_ENCODER_COUNTS_PER_STEP;
    }
    return steps;
}

static void HMI_RenderItem(uint8_t index)
{
    if ((index >= HMI_ITEM_COUNT) || (HMI_ItemIsVisible(index) == 0U))
    {
        return;
    }
    if (hmi_items[index].render != NULL)
    {
        hmi_items[index].render(&hmi_items[index]);
    }
}

static void HMI_RenderTopItems(void)
{
    for (uint8_t i = HMI_INDEX_VELOCITY; i <= HMI_INDEX_PRESET2; i++)
    {
        HMI_RenderItem(i);
    }
}

static void HMI_RenderDiagramItems(void)
{
    for (uint8_t i = HMI_INDEX_INPUT_SOURCE; i < HMI_ITEM_COUNT; i++)
    {
        HMI_RenderItem(i);
    }
}

static void HMI_RenderAllItems(void)
{
    HMI_RenderTopItems();
    HMI_RenderDiagramItems();
}

static uint8_t HMI_SyncButtonStates(void)
{
    uint8_t changed = 0U;
    uint8_t velocity_on = (desired_state == STATE_VELOCITY_CONTROL) ? 1U : 0U;
    uint8_t position_on = (desired_state == STATE_POSITION_CONTROL) ? 1U : 0U;
    uint8_t start_on = (Control_IsTrackingEnabled() != 0U) ? 1U : 0U;

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
    if (hmi_items[HMI_INDEX_START].is_on != start_on)
    {
        hmi_items[HMI_INDEX_START].is_on = start_on;
        changed = 1U;
    }
    return changed;
}

static void HMI_SelectIndex(uint8_t new_index)
{
    if ((new_index >= HMI_ITEM_COUNT) ||
        (hmi_items[new_index].selectable == 0U) ||
        (HMI_ItemIsVisible(new_index) == 0U))
    {
        return;
    }

    uint8_t old_index = hmi_selected_index;
    if (new_index == old_index)
    {
        hmi_items[hmi_selected_index].is_selected = 1U;
        return;
    }

    hmi_items[old_index].is_selected = 0U;
    hmi_selected_index = new_index;
    hmi_items[hmi_selected_index].is_selected = 1U;
    HMI_RenderItem(old_index);
    HMI_RenderItem(hmi_selected_index);
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
        else if (index == 0U)
        {
            index = (uint8_t)(HMI_ITEM_COUNT - 1U);
        }
        else
        {
            index--;
        }

        if ((hmi_items[index].selectable != 0U) && (HMI_ItemIsVisible(index) != 0U))
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

/* -------------------------------------------------------------------------- */
/* Page redraw helpers                                                        */
/* -------------------------------------------------------------------------- */

static void HMI_RedrawFullDiagramPage(void)
{
    hmi_page = HMI_PAGE_DIAGRAM;
    hmi_editing = 0U;
    HMI_ResetEncoderAccumulator();
    ControlLoopDisplay_DrawForMode(desired_state);
    (void)HMI_SyncButtonStates();
    HMI_EnsureSelectedVisible();
    HMI_RenderAllItems();
    hmi_last_value_refresh_ms = HAL_GetTick();
}

static void HMI_RedrawDiagramAreaOnly(void)
{
    hmi_page = HMI_PAGE_DIAGRAM;
    HMI_ResetEncoderAccumulator();
    ControlLoopDisplay_DrawDiagramOnlyForMode(desired_state);
    HMI_EnsureSelectedVisible();
    HMI_RenderDiagramItems();
    hmi_last_value_refresh_ms = HAL_GetTick();
    hmi_layout_dirty = 0U;
}

/* -------------------------------------------------------------------------- */
/* Item renderers and actions                                                 */
/* -------------------------------------------------------------------------- */

static void HMI_ButtonRender(UiItem_t *self)
{
    uint16_t fill = self->is_on ? self->active_color : HMI_BG;
    uint16_t fg = self->is_on ? COLOR_WHITE : HMI_FG;
    uint16_t border = self->is_selected ? HMI_SELECTED : HMI_DIM;

    ILI9341_fillRect(self->x, self->y, self->w, self->h, fill);
    HMI_DrawRectBorder(self->x, self->y, self->w, self->h, border, HMI_BUTTON_BORDER);

    uint16_t tw = HMI_TextWidth(self->label);
    uint16_t tx = (tw < self->w) ? (uint16_t)(self->x + ((self->w - tw) / 2U)) : self->x;
    ILI9341_printString(tx, (uint16_t)(self->y + 8U), self->label, fg, fill, HMI_TEXT_SCALE);
}

static void HMI_RequestVelocityMode(void)
{
    Control_SetDesiredState(STATE_VELOCITY_CONTROL);
    hmi_editing = 0U;
    (void)HMI_SyncButtonStates();
    HMI_RenderTopItems();
    HMI_RedrawDiagramAreaOnly();
}

static void HMI_RequestPositionMode(void)
{
    Control_SetDesiredState(STATE_POSITION_CONTROL);
    hmi_editing = 0U;
    (void)HMI_SyncButtonStates();
    HMI_RenderTopItems();
    HMI_RedrawDiagramAreaOnly();
}

static void HMI_ShowPlotPage(void);

static void HMI_RequestStartToggle(void)
{
    hmi_editing = 0U;
    HMI_SelectIndex(HMI_INDEX_START);

    if (Control_IsTrackingEnabled() == 0U)
    {
        Control_EnterTracking();
        hmi_tracking_lock = 1U;
        (void)HMI_SyncButtonStates();
        HMI_ShowPlotPage();
    }
    else
    {
        Control_ExitTracking();
        hmi_tracking_lock = 0U;
        HMI_RedrawFullDiagramPage();
    }
}

static void HMI_ButtonPress(UiItem_t *self)
{
    if (self == &hmi_items[HMI_INDEX_VELOCITY])
    {
        HMI_RequestVelocityMode();
    }
    else if (self == &hmi_items[HMI_INDEX_POSITION])
    {
        HMI_RequestPositionMode();
    }
    else if (self == &hmi_items[HMI_INDEX_START])
    {
        HMI_RequestStartToggle();
    }
}

static float HMI_ApplyInputAmplitude(float value)
{
    return Control_SetInputWaveAmplitude(value);
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

    if ((self->label[0] == 'F') && (self->label[1] == 's'))
    {
        HMI_RateToStr(num, sizeof(num), *(entry->value));
    }
    else
    {
        HMI_FloatToStr(num, sizeof(num), *(entry->value), entry->decimals);
    }

    (void)snprintf(dst, dst_len, "%s=%s", self->label, num);
}

static void HMI_FloatEntryRender(UiItem_t *self)
{
    char text[24];
    self->format(self, text, sizeof(text));

    uint8_t editing_this = ((self->is_selected != 0U) && (hmi_editing != 0U)) ? 1U : 0U;
    uint16_t fill = editing_this ? HMI_SELECTED : HMI_BG;
    uint16_t border = self->is_selected ? HMI_SELECTED : HMI_DIM;

    ILI9341_fillRect(self->x, self->y, self->w, self->h, fill);
    HMI_DrawRectBorder(self->x, self->y, self->w, self->h, border, HMI_FIELD_BORDER);
    HMI_PrintFixedWidth((uint16_t)(self->x + FIELD_X_OFFSET),
                        (uint16_t)(self->y + FIELD_Y_OFFSET),
                        text,
                        self->text_chars,
                        HMI_FG,
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

    float value = *(entry->value) + ((float)steps * entry->step);
    if (value > entry->max_value)
    {
        value = entry->max_value;
    }
    else if (value < entry->min_value)
    {
        value = entry->min_value;
    }

    if (entry->apply != NULL)
    {
        value = entry->apply(value);
    }
    else
    {
        *(entry->value) = value;
    }
}

static void HMI_FormatInputSource(UiItem_t *self, char *dst, size_t dst_len)
{
    (void)self;
    (void)snprintf(dst, dst_len, "SRC:%s", Control_GetInputSourceName(control_input_source));
}

static void HMI_InputSelectRender(UiItem_t *self)
{
    char text[24];
    self->format(self, text, sizeof(text));

    uint8_t editing_this = ((self->is_selected != 0U) && (hmi_editing != 0U)) ? 1U : 0U;
    uint16_t fill = editing_this ? HMI_SELECTED : HMI_BG;
    uint16_t border = self->is_selected ? HMI_SELECTED : HMI_DIM;

    ILI9341_fillRect(self->x, self->y, self->w, self->h, fill);
    HMI_DrawRectBorder(self->x, self->y, self->w, self->h, border, HMI_FIELD_BORDER);
    HMI_PrintFixedWidth((uint16_t)(self->x + FIELD_X_OFFSET),
                        (uint16_t)(self->y + FIELD_Y_OFFSET),
                        text,
                        self->text_chars,
                        HMI_FG,
                        fill);
}

static void HMI_InputSelectPress(UiItem_t *self)
{
    (void)self;
    hmi_editing = hmi_editing ? 0U : 1U;
}

static void HMI_InputSelectEncoder(UiItem_t *self, int32_t steps)
{
    (void)self;
    if (hmi_editing == 0U)
    {
        return;
    }

    ControlInputSource_t old_source = control_input_source;
    while (steps > 0)
    {
        Control_NextInputSource(1);
        steps--;
    }
    while (steps < 0)
    {
        Control_NextInputSource(-1);
        steps++;
    }

    if (old_source != control_input_source)
    {
        hmi_layout_dirty = 1U;
    }
}

static void HMI_FormatPreset(UiItem_t *self, char *dst, size_t dst_len)
{
    (void)snprintf(dst, dst_len, "%s", self->label);
}

static void HMI_PresetButtonPress(UiItem_t *self);

static void HMI_PressSelected(void)
{
    if (hmi_selected_index >= HMI_ITEM_COUNT)
    {
        return;
    }

    UiItem_t *selected = &hmi_items[hmi_selected_index];
    if (selected->on_press != NULL)
    {
        selected->on_press(selected);
    }

    if (hmi_page != HMI_PAGE_DIAGRAM)
    {
        return;
    }

    if ((selected->type == UI_ITEM_BUTTON) || (selected->type == UI_ITEM_PRESET_BUTTON))
    {
        HMI_RenderTopItems();
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
        if (hmi_layout_dirty == 0U)
        {
            HMI_RenderItem(hmi_selected_index);
        }
    }
    else
    {
        HMI_MoveSelectionBySteps(steps);
    }
}

static void HMI_ProcessEncoder(void)
{
    if (Control_IsTrackingEnabled() != 0U)
    {
        HMI_ResetEncoderAccumulator();
        HMI_SelectIndex(HMI_INDEX_START);
        return;
    }

    int32_t steps = HMI_ConsumeEncoderSteps();
    HMI_HandleEncoderSteps(steps);
}

/* -------------------------------------------------------------------------- */
/* Plot pages                                                                 */
/* -------------------------------------------------------------------------- */

static void HMI_DrawPlotBase(const char *title)
{
    ILI9341_setRotation(1U);
    ILI9341_fillRect(0U, 0U, 320U, 240U, HMI_BG);
    ILI9341_printString(6U, 8U, title, HMI_FG, HMI_BG, HMI_TEXT_SCALE);
    ILI9341_printString(208U, 8U, "BTN=STOP", HMI_DIM, HMI_BG, HMI_TEXT_SCALE);
    ILI9341_printString(6U, 28U, "blue=target  red=measured", HMI_DIM, HMI_BG, HMI_TEXT_SCALE);

    ILI9341_fillRect(HMI_PLOT_X, HMI_PLOT_Y, HMI_PLOT_W, HMI_PLOT_H, HMI_BG);
    HMI_DrawRectBorder(HMI_PLOT_X, HMI_PLOT_Y, HMI_PLOT_W, HMI_PLOT_H, HMI_DIM, 1U);
}

static void HMI_DrawAxisLabels(const char *x_label,
                               const char *y_label,
                               const char *top_label,
                               const char *mid_label,
                               const char *bottom_label)
{
    ILI9341_printString(6U, (uint16_t)(HMI_PLOT_Y + 2U), top_label, HMI_DIM, HMI_BG, HMI_TEXT_SCALE);
    ILI9341_printString(10U, (uint16_t)(HMI_PLOT_Y + (HMI_PLOT_H / 2U) - 4U), mid_label, HMI_DIM, HMI_BG, HMI_TEXT_SCALE);
    ILI9341_printString(2U, (uint16_t)(HMI_PLOT_Y + HMI_PLOT_H - 8U), bottom_label, HMI_DIM, HMI_BG, HMI_TEXT_SCALE);
    ILI9341_printString((uint16_t)(HMI_PLOT_X + 2U), (uint16_t)(HMI_PLOT_Y - 12U), y_label, HMI_DIM, HMI_BG, HMI_TEXT_SCALE);
    ILI9341_printString((uint16_t)(HMI_PLOT_X + 92U), (uint16_t)(HMI_PLOT_Y + HMI_PLOT_H + 18U), x_label, HMI_DIM, HMI_BG, HMI_TEXT_SCALE);
}

static uint16_t HMI_TimeTickPixels(void)
{
    uint16_t tick_px = (uint16_t)(HMI_PLOT_TICK_MS / HMI_PLOT_REFRESH_MS);
    return (tick_px == 0U) ? 1U : tick_px;
}

static uint16_t HMI_TimeMajorTickPixels(void)
{
    uint16_t major_px = (uint16_t)(HMI_PLOT_MAJOR_TICK_MS / HMI_PLOT_REFRESH_MS);
    return (major_px == 0U) ? HMI_TimeTickPixels() : major_px;
}

static void HMI_DrawTimeTickAtOffset(uint16_t xoff)
{
    uint16_t tick_px = HMI_TimeTickPixels();
    uint16_t major_px = HMI_TimeMajorTickPixels();

    if ((xoff % tick_px) != 0U)
    {
        return;
    }

    uint16_t x = (uint16_t)(HMI_PLOT_X + 1U + xoff);
    uint16_t bottom = (uint16_t)(HMI_PLOT_Y + HMI_PLOT_H - 2U);
    uint16_t tick_h = ((xoff % major_px) == 0U) ? 8U : 4U;
    HMI_DrawLine((int16_t)x, (int16_t)(bottom - tick_h), (int16_t)x, (int16_t)bottom, HMI_DIM);
}

static void HMI_DrawTimeTicks(void)
{
    uint16_t tick_px = HMI_TimeTickPixels();
    uint16_t major_px = HMI_TimeMajorTickPixels();
    uint16_t seconds = 0U;

    for (uint16_t xoff = 0U; xoff < (HMI_PLOT_W - 2U); xoff = (uint16_t)(xoff + tick_px))
    {
        HMI_DrawTimeTickAtOffset(xoff);

        if ((xoff % major_px) == 0U)
        {
            char label[8];
            uint16_t x = (uint16_t)(HMI_PLOT_X + 1U + xoff);
            (void)snprintf(label, sizeof(label), "%us", (unsigned)seconds);
            ILI9341_printString((uint16_t)(x - 4U),
                                (uint16_t)(HMI_PLOT_Y + HMI_PLOT_H + 5U),
                                label,
                                HMI_DIM,
                                HMI_BG,
                                HMI_TEXT_SCALE);
        }
        seconds = (uint16_t)(seconds + (HMI_PLOT_TICK_MS / 1000U));
    }
}

static void HMI_ClearTimePlotGraphArea(float min_value, float max_value)
{
    ILI9341_fillRect((uint16_t)(HMI_PLOT_X + 1U),
                     (uint16_t)(HMI_PLOT_Y + 1U),
                     (uint16_t)(HMI_PLOT_W - 2U),
                     (uint16_t)(HMI_PLOT_H - 2U),
                     HMI_BG);

    int16_t y_mid = HMI_MapY(0.0f, min_value, max_value);
    HMI_DrawLine((int16_t)(HMI_PLOT_X + 1U), y_mid,
                 (int16_t)(HMI_PLOT_X + HMI_PLOT_W - 2U), y_mid, HMI_DIM);
    HMI_DrawTimeTicks();
    hmi_plot_x = 0U;
}

static void HMI_DrawTimePlotFrame(void)
{
    char top[16];
    char bottom[16];
    float min_value;
    float max_value;

    if (desired_state == STATE_VELOCITY_CONTROL)
    {
        max_value = ctx_pos.output_range;
        if (max_value < 1.0f)
        {
            max_value = CONTROL_DEFAULT_POS_RANGE_RPM;
        }
        min_value = -max_value;
        HMI_FloatToStr(top, sizeof(top), max_value, 0U);
        HMI_FloatToStr(bottom, sizeof(bottom), min_value, 0U);
        HMI_DrawPlotBase("VELOCITY LIVE TIME PLOT");
        HMI_DrawAxisLabels("Time (s)", "Velocity (RPM)", top, "0", bottom);
    }
    else
    {
        min_value = HMI_PLOT_POS_MIN_DEG;
        max_value = HMI_PLOT_POS_MAX_DEG;
        HMI_DrawPlotBase("POSITION LIVE TIME PLOT");
        HMI_DrawAxisLabels("Time (s)", "Position (deg)", "180", "0", "-180");
    }

    HMI_ClearTimePlotGraphArea(min_value, max_value);
}

static void HMI_UpdateTimePlot(void)
{
    float min_value;
    float max_value;
    float target;
    float measured;

    if (desired_state == STATE_VELOCITY_CONTROL)
    {
        max_value = ctx_pos.output_range;
        if (max_value < 1.0f)
        {
            max_value = CONTROL_DEFAULT_POS_RANGE_RPM;
        }
        min_value = -max_value;
        target = target_velocity1;
        measured = measured_velocity1;
    }
    else
    {
        min_value = HMI_PLOT_POS_MIN_DEG;
        max_value = HMI_PLOT_POS_MAX_DEG;
        target = HMI_WrapDeg180(target_position1);
        measured = HMI_WrapDeg180(encoder_measured_position1);
    }

    if (hmi_plot_x >= (HMI_PLOT_W - 2U))
    {
        HMI_ClearTimePlotGraphArea(min_value, max_value);
    }

    uint16_t xoff = hmi_plot_x;
    uint16_t x = (uint16_t)(HMI_PLOT_X + 1U + xoff);
    ILI9341_fillRect(x, (uint16_t)(HMI_PLOT_Y + 1U), 2U, (uint16_t)(HMI_PLOT_H - 2U), HMI_BG);
    HMI_DrawTimeTickAtOffset(xoff);

    int16_t y_mid = HMI_MapY(0.0f, min_value, max_value);
    HMI_DrawPixel((int16_t)x, y_mid, HMI_DIM);

    int16_t y_target = HMI_MapY(target, min_value, max_value);
    int16_t y_meas = HMI_MapY(measured, min_value, max_value);
    ILI9341_fillRect(x, (uint16_t)y_target, 2U, 2U, COLOR_BLUE);
    ILI9341_fillRect(x, (uint16_t)y_meas, 2U, 2U, COLOR_RED);

    hmi_plot_x++;
}

static void HMI_ShowPlotPage(void)
{
    hmi_page = HMI_PAGE_PLOT;
    HMI_ResetEncoderAccumulator();
    hmi_last_plot_refresh_ms = HAL_GetTick();
    HMI_DrawTimePlotFrame();
}

static void HMI_StopAndReturnFromPlot(void)
{
    Control_ExitTracking();
    hmi_tracking_lock = 0U;
    HMI_RedrawFullDiagramPage();
}

static void HMI_PlotTask(void)
{
    if (Button_WasPressed() != 0U)
    {
        HMI_StopAndReturnFromPlot();
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - hmi_last_plot_refresh_ms) >= HMI_PLOT_REFRESH_MS)
    {
        hmi_last_plot_refresh_ms = now;
        HMI_UpdateTimePlot();
    }
}

/* -------------------------------------------------------------------------- */
/* Preset mini menu                                                           */
/* -------------------------------------------------------------------------- */

#define PRESET_MENU_COUNT             7U
#define PRESET_ITEM_POS_KP            0U
#define PRESET_ITEM_POS_KI            1U
#define PRESET_ITEM_VEL_KP            2U
#define PRESET_ITEM_VEL_KI            3U
#define PRESET_ITEM_SAVE_CUR          4U
#define PRESET_ITEM_APPLY             5U
#define PRESET_ITEM_BACK              6U
#define PRESET_MENU_X                 34U
#define PRESET_MENU_Y                 42U
#define PRESET_MENU_W                 252U
#define PRESET_ROW_H                  20U
#define PRESET_ROW_GAP                3U

static float HMI_PresetClamp(uint8_t field, float value)
{
    float max_value = ((field == PRESET_ITEM_POS_KP) || (field == PRESET_ITEM_VEL_KP)) ? HMI_FLOAT_MAX_KP : HMI_FLOAT_MAX_KI;
    if (value < 0.0f)
    {
        value = 0.0f;
    }
    else if (value > max_value)
    {
        value = max_value;
    }
    return value;
}

static float HMI_GetPresetField(ControlPreset_t preset, uint8_t field)
{
    switch (field)
    {
        case PRESET_ITEM_POS_KP: return preset.pos_kp;
        case PRESET_ITEM_POS_KI: return preset.pos_ki;
        case PRESET_ITEM_VEL_KP: return preset.vel_kp;
        case PRESET_ITEM_VEL_KI: return preset.vel_ki;
        default: return 0.0f;
    }
}

static const char *HMI_GetPresetFieldName(uint8_t field)
{
    switch (field)
    {
        case PRESET_ITEM_POS_KP: return "POS Kp";
        case PRESET_ITEM_POS_KI: return "POS Ki";
        case PRESET_ITEM_VEL_KP: return "VEL Kp";
        case PRESET_ITEM_VEL_KI: return "VEL Ki";
        case PRESET_ITEM_SAVE_CUR: return "SAVE CUR";
        case PRESET_ITEM_APPLY: return "APPLY";
        case PRESET_ITEM_BACK: return "BACK";
        default: return "?";
    }
}

static void HMI_RenderPresetRow(uint8_t row)
{
    char value_text[16];
    char row_text[32];
    uint16_t y = (uint16_t)(PRESET_MENU_Y + ((uint16_t)row * (PRESET_ROW_H + PRESET_ROW_GAP)));
    uint8_t selected = (row == hmi_preset_selected) ? 1U : 0U;
    uint8_t editing = ((selected != 0U) && (hmi_preset_editing != 0U)) ? 1U : 0U;
    uint16_t fill = editing ? HMI_SELECTED : HMI_BG;
    uint16_t border = selected ? HMI_SELECTED : HMI_DIM;

    ILI9341_fillRect(PRESET_MENU_X, y, PRESET_MENU_W, PRESET_ROW_H, fill);
    HMI_DrawRectBorder(PRESET_MENU_X, y, PRESET_MENU_W, PRESET_ROW_H, border, 1U);

    if (row <= PRESET_ITEM_VEL_KI)
    {
        ControlPreset_t preset = Control_GetPreset(hmi_preset_slot);
        HMI_FloatToStr(value_text, sizeof(value_text), HMI_GetPresetField(preset, row), 2U);
        (void)snprintf(row_text, sizeof(row_text), "%s = %s", HMI_GetPresetFieldName(row), value_text);
    }
    else
    {
        (void)snprintf(row_text, sizeof(row_text), "%s", HMI_GetPresetFieldName(row));
    }

    ILI9341_printString((uint16_t)(PRESET_MENU_X + 7U),
                        (uint16_t)(y + 6U),
                        row_text,
                        HMI_FG,
                        fill,
                        HMI_TEXT_SCALE);
}

static void HMI_DrawPresetMenu(void)
{
    char title[32];
    ILI9341_setRotation(1U);
    ILI9341_fillRect(0U, 0U, 320U, 240U, HMI_BG);
    (void)snprintf(title, sizeof(title), "PRESET %u CONFIG", (unsigned)(hmi_preset_slot + 1U));
    ILI9341_printString(6U, 8U, title, HMI_FG, HMI_BG, HMI_TEXT_SCALE);
    ILI9341_printString(6U, 24U, "Edit fields, SAVE CUR, APPLY, or BACK", HMI_DIM, HMI_BG, HMI_TEXT_SCALE);

    for (uint8_t i = 0U; i < PRESET_MENU_COUNT; i++)
    {
        HMI_RenderPresetRow(i);
    }
}

static void HMI_OpenPresetMenu(uint8_t slot)
{
    if (slot >= CONTROL_PRESET_COUNT)
    {
        return;
    }

    hmi_page = HMI_PAGE_PRESET;
    hmi_preset_slot = slot;
    hmi_preset_selected = 0U;
    hmi_preset_editing = 0U;
    hmi_editing = 0U;
    HMI_ResetEncoderAccumulator();
    HMI_DrawPresetMenu();
}

static void HMI_PresetButtonPress(UiItem_t *self)
{
    uint8_t slot = (self == &hmi_items[HMI_INDEX_PRESET2]) ? 1U : 0U;
    HMI_OpenPresetMenu(slot);
}

static void HMI_ProcessPresetEncoder(void)
{
    int32_t steps = HMI_ConsumeEncoderSteps();
    if (steps == 0)
    {
        return;
    }

    if ((hmi_preset_editing != 0U) && (hmi_preset_selected <= PRESET_ITEM_VEL_KI))
    {
        ControlPreset_t preset = Control_GetPreset(hmi_preset_slot);
        float value = HMI_GetPresetField(preset, hmi_preset_selected);
        value += ((float)steps * HMI_FLOAT_STEP_GAIN);
        value = HMI_PresetClamp(hmi_preset_selected, value);
        Control_SetPresetValue(hmi_preset_slot, hmi_preset_selected, value);
        HMI_RenderPresetRow(hmi_preset_selected);
        return;
    }

    uint8_t old = hmi_preset_selected;
    while (steps > 0)
    {
        hmi_preset_selected++;
        if (hmi_preset_selected >= PRESET_MENU_COUNT)
        {
            hmi_preset_selected = 0U;
        }
        steps--;
    }
    while (steps < 0)
    {
        if (hmi_preset_selected == 0U)
        {
            hmi_preset_selected = PRESET_MENU_COUNT - 1U;
        }
        else
        {
            hmi_preset_selected--;
        }
        steps++;
    }

    HMI_RenderPresetRow(old);
    HMI_RenderPresetRow(hmi_preset_selected);
}

static void HMI_PresetPressSelected(void)
{
    if (hmi_preset_selected <= PRESET_ITEM_VEL_KI)
    {
        hmi_preset_editing = hmi_preset_editing ? 0U : 1U;
        HMI_RenderPresetRow(hmi_preset_selected);
    }
    else if (hmi_preset_selected == PRESET_ITEM_SAVE_CUR)
    {
        Control_SavePreset(hmi_preset_slot);
        HMI_DrawPresetMenu();
    }
    else if (hmi_preset_selected == PRESET_ITEM_APPLY)
    {
        Control_LoadPreset(hmi_preset_slot);
        HMI_DrawPresetMenu();
    }
    else if (hmi_preset_selected == PRESET_ITEM_BACK)
    {
        HMI_RedrawFullDiagramPage();
    }
}

static void HMI_PresetTask(void)
{
    HMI_ProcessPresetEncoder();
    if (Button_WasPressed() != 0U)
    {
        HMI_PresetPressSelected();
    }
}

/* -------------------------------------------------------------------------- */
/* Public UI API                                                              */
/* -------------------------------------------------------------------------- */

void UI_Init(void)
{
    ILI9341_setRotation(1U);
    hmi_page = HMI_PAGE_DIAGRAM;
    hmi_editing = 0U;
    hmi_tracking_lock = 0U;
    hmi_layout_dirty = 0U;
    hmi_last_value_refresh_ms = HAL_GetTick();
    hmi_last_plot_refresh_ms = HAL_GetTick();

    HMI_ResetEncoderAccumulator();
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
    if (hmi_page == HMI_PAGE_PLOT)
    {
        HMI_PlotTask();
        return;
    }

    if (hmi_page == HMI_PAGE_PRESET)
    {
        HMI_PresetTask();
        return;
    }

    if (HMI_SyncButtonStates() != 0U)
    {
        HMI_RenderTopItems();
    }

    HMI_ProcessEncoder();

    if (hmi_layout_dirty != 0U)
    {
        HMI_RedrawDiagramAreaOnly();
    }

    if (Button_WasPressed() != 0U)
    {
        HMI_PressSelected();
        if (hmi_page != HMI_PAGE_DIAGRAM)
        {
            return;
        }
    }

    uint32_t now = HAL_GetTick();
    if ((now - hmi_last_value_refresh_ms) >= HMI_VALUE_REFRESH_MS)
    {
        hmi_last_value_refresh_ms = now;
        HMI_RenderDiagramItems();
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
    if (hmi_page == HMI_PAGE_DIAGRAM)
    {
        HMI_RenderAllItems();
    }
    else if (hmi_page == HMI_PAGE_PLOT)
    {
        HMI_DrawTimePlotFrame();
    }
    else
    {
        HMI_DrawPresetMenu();
    }
}
