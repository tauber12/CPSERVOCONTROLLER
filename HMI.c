/*
 *******************************************************************************
 * @file           : HMI.c
 * @brief          : Object-like HMI backend for TFT buttons, numeric entries,
 *                   presets, input mux selection, and plot pages
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.5
 * date            : May 30, 2026
 *******************************************************************************
 * Description:
 *   This file owns all interactive/dynamic UI items on top of the static
 *   control-loop diagram drawn by control_loop_display.c.
 *
 *   Main diagram page items:
 *       - VELOCITY / POSITION / FREQ mode buttons
 *       - START button (formerly TRACK)
 *       - P1 / P2 preset boxes in the upper-right corner
 *       - INPUT MUX source selector: ENC, SIN, COS, RAMP, SQUARE, STEP
 *       - Position PI editable entries: Kp, Ki, Fs
 *       - Velocity PI editable entries: Kp, Ki, Fs
 *
 *   START behavior:
 *       - Starts the selected mode and immediately switches to a live plot.
 *       - Position/velocity modes show a live time plot.
 *       - Frequency-response mode shows a Bode magnitude plot as points finish.
 *       - Press the physical button again from the plot page to return to
 *         the control-loop diagram; press START on the diagram to stop.
 *
 *   Preset behavior:
 *       - Select P1 or P2 to open a mini menu.
 *       - SAVE CUR copies the current active gains into that preset.
 *       - APPLY loads the preset into the active controllers.
 *       - Four manual fields allow editing Pos Kp, Pos Ki, Vel Kp, Vel Ki.
 *******************************************************************************
 */

#include "HMI.h"

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

    /* 4. TI1->IC1, TI2->IC2, encoder input filter.
     * The front-panel encoder is a human interface, so a moderate digital
     * filter is preferred over raw edge sensitivity. This rejects contact
     * bounce/noise without making menu motion feel sluggish.
     */
    TIM4->CCMR1 = 0U;
    TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM4->CCMR1 |= TIM_CCMR1_CC2S_0;
    TIM4->CCMR1 |= (0x6U << TIM_CCMR1_IC1F_Pos);
    TIM4->CCMR1 |= (0x6U << TIM_CCMR1_IC2F_Pos);

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

#define HMI_ENCODER_COUNTS_PER_STEP    80
#define HMI_ENCODER_MAX_STEPS_PER_TASK 8
#define HMI_ENCODER_DELTA_LIMIT       (HMI_ENCODER_COUNTS_PER_STEP * HMI_ENCODER_MAX_STEPS_PER_TASK)
#define HMI_ENCODER_ACCUM_LIMIT       (HMI_ENCODER_COUNTS_PER_STEP * HMI_ENCODER_MAX_STEPS_PER_TASK)
#define HMI_VALUE_REFRESH_MS          200U
#define HMI_PLOT_REFRESH_MS           50U
#define HMI_BODE_REFRESH_MS           250U

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
#define MODE_BOX_W                    56U
#define MODE_BOX_H                    24U
#define MODE_BOX_GAP                  4U
#define MODE_SLOT_0_X                 4U
#define MODE_SLOT_1_X                 (MODE_SLOT_0_X + MODE_BOX_W + MODE_BOX_GAP)
#define MODE_SLOT_2_X                 (MODE_SLOT_1_X + MODE_BOX_W + MODE_BOX_GAP)
#define MODE_SLOT_3_X                 (MODE_SLOT_2_X + MODE_BOX_W + MODE_BOX_GAP)

#define PRESET_BOX_Y                  MODE_BOX_Y
#define PRESET_BOX_W                  31U
#define PRESET_BOX_H                  MODE_BOX_H
#define PRESET_SLOT_0_X               252U
#define PRESET_SLOT_1_X               286U

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

#define INPUT_TEXT_CHARS              9U
#define PI_TEXT_CHARS                 10U

#define HMI_INVALID_INDEX             0xFFU

#define HMI_INDEX_VELOCITY            0U
#define HMI_INDEX_POSITION            1U
#define HMI_INDEX_FREQUENCY           2U
#define HMI_INDEX_START               3U
#define HMI_INDEX_PRESET1             4U
#define HMI_INDEX_PRESET2             5U
#define HMI_INDEX_INPUT_SOURCE        6U
#define HMI_INDEX_POS_KP              7U
#define HMI_INDEX_POS_KI              8U
#define HMI_INDEX_POS_FS              9U
#define HMI_INDEX_VEL_KP              10U
#define HMI_INDEX_VEL_KI              11U
#define HMI_INDEX_VEL_FS              12U

#define HMI_FLOAT_MIN_KP              0.0f
#define HMI_FLOAT_MAX_KP              200.0f
#define HMI_FLOAT_MIN_KI              0.0f
#define HMI_FLOAT_MAX_KI              50.0f
#define HMI_FLOAT_STEP_GAIN           0.10f
#define HMI_FLOAT_MIN_POS_FS          50.0f
#define HMI_FLOAT_MAX_POS_FS          2000.0f
#define HMI_FLOAT_STEP_POS_FS         50.0f
#define HMI_FLOAT_MIN_VEL_FS          100.0f
#define HMI_FLOAT_MAX_VEL_FS          10000.0f
#define HMI_FLOAT_STEP_VEL_FS         100.0f

#define PLOT_X                        28U
#define PLOT_Y                        52U
#define PLOT_W                        280U
#define PLOT_H                        158U
#define PLOT_TOP_TEXT_Y               8U
#define PLOT_HINT_Y                   222U
#define PLOT_POS_MIN_DEG              -180.0f
#define PLOT_POS_MAX_DEG               180.0f
#define PLOT_VEL_MIN_RPM              -120.0f
#define PLOT_VEL_MAX_RPM               120.0f
#define PLOT_BODE_MIN_DB              -60.0f
#define PLOT_BODE_MAX_DB               20.0f

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
    UI_ITEM_READOUT,
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
static uint8_t HMI_ItemIsVisible(uint8_t index);
static void HMI_EnsureSelectedVisible(void);
static void HMI_ResetEncoderAccumulator(void);
static int32_t HMI_ConsumeEncoderSteps(void);
static void HMI_RenderModeButtons(void);
static uint8_t HMI_SyncButtonStates(void);
static void HMI_SyncTrackingLock(void);
static void HMI_SelectIndex(uint8_t new_index);
static void HMI_RequestVelocityMode(void);
static void HMI_RequestPositionMode(void);
static void HMI_RequestFrequencyMode(void);
static void HMI_RequestStartToggle(void);
static void HMI_OpenPresetMenu(uint8_t slot);
static void HMI_FloatEntryRender(UiItem_t *self);
static void HMI_FloatEntryPress(UiItem_t *self);
static void HMI_FloatEntryEncoder(UiItem_t *self, int32_t steps);
static void HMI_ReadoutRender(UiItem_t *self);
static void HMI_InputSelectRender(UiItem_t *self);
static void HMI_InputSelectPress(UiItem_t *self);
static void HMI_InputSelectEncoder(UiItem_t *self, int32_t steps);
static void HMI_PresetButtonPress(UiItem_t *self);

static void HMI_FormatFloatEntry(UiItem_t *self, char *dst, size_t dst_len);
static void HMI_FormatInputSource(UiItem_t *self, char *dst, size_t dst_len);
static void HMI_FormatPreset(UiItem_t *self, char *dst, size_t dst_len);

static void HMI_RedrawDiagramPage(void);
static void HMI_ShowPlotPage(void);
static void HMI_PlotTask(void);
static void HMI_ReturnFromPlotToDiagram(void);
static void HMI_DrawAxisLabels(const char *x_label,
                               const char *y_label,
                               const char *top_label,
                               const char *mid_label,
                               const char *bottom_label);
static void HMI_DrawTimePlotFrame(void);
static void HMI_UpdateTimePlot(void);
static void HMI_DrawBodeFrame(void);
static void HMI_UpdateBodePlot(void);

static void HMI_PresetTask(void);
static void HMI_DrawPresetMenu(void);
static void HMI_ProcessPresetEncoder(void);
static void HMI_ClearSelectionState(void);
static uint8_t HMI_FirstSelectableIndex(void);

static HmiFloatEntry_t hmi_pos_kp = { &ctx_pos.kp, HMI_FLOAT_MIN_KP, HMI_FLOAT_MAX_KP, HMI_FLOAT_STEP_GAIN, 2U, NULL };
static HmiFloatEntry_t hmi_pos_ki = { &ctx_pos.ki, HMI_FLOAT_MIN_KI, HMI_FLOAT_MAX_KI, HMI_FLOAT_STEP_GAIN, 2U, NULL };
static HmiFloatEntry_t hmi_pos_fs = { &control_position_loop_hz, HMI_FLOAT_MIN_POS_FS, HMI_FLOAT_MAX_POS_FS, HMI_FLOAT_STEP_POS_FS, 0U, Control_SetPositionLoopHz };
static HmiFloatEntry_t hmi_vel_kp = { &ctx_vel.kp, HMI_FLOAT_MIN_KP, HMI_FLOAT_MAX_KP, HMI_FLOAT_STEP_GAIN, 2U, NULL };
static HmiFloatEntry_t hmi_vel_ki = { &ctx_vel.ki, HMI_FLOAT_MIN_KI, HMI_FLOAT_MAX_KI, HMI_FLOAT_STEP_GAIN, 2U, NULL };
static HmiFloatEntry_t hmi_vel_fs = { &control_velocity_loop_hz, HMI_FLOAT_MIN_VEL_FS, HMI_FLOAT_MAX_VEL_FS, HMI_FLOAT_STEP_VEL_FS, 0U, Control_SetVelocityLoopHz };

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
        .label = "FREQ",
        .x = MODE_SLOT_2_X,
        .y = MODE_BOX_Y,
        .w = MODE_BOX_W,
        .h = MODE_BOX_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = 0U,
        .active_color = COLOR_MAGENTA,
        .data = NULL,
        .format = NULL,
        .render = HMI_ButtonRender,
        .on_press = HMI_ButtonPress,
        .on_encoder = NULL
    },
    {
        .type = UI_ITEM_BUTTON,
        .label = "START",
        .x = MODE_SLOT_3_X,
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
    {
        .type = UI_ITEM_PRESET_BUTTON,
        .label = "P1",
        .x = PRESET_SLOT_0_X,
        .y = PRESET_BOX_Y,
        .w = PRESET_BOX_W,
        .h = PRESET_BOX_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = 0U,
        .active_color = COLOR_CYAN,
        .data = (void *)0U,
        .format = HMI_FormatPreset,
        .render = HMI_ButtonRender,
        .on_press = HMI_PresetButtonPress,
        .on_encoder = NULL
    },
    {
        .type = UI_ITEM_PRESET_BUTTON,
        .label = "P2",
        .x = PRESET_SLOT_1_X,
        .y = PRESET_BOX_Y,
        .w = PRESET_BOX_W,
        .h = PRESET_BOX_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = 0U,
        .active_color = COLOR_CYAN,
        .data = (void *)1U,
        .format = HMI_FormatPreset,
        .render = HMI_ButtonRender,
        .on_press = HMI_PresetButtonPress,
        .on_encoder = NULL
    },

    /* Input MUX source selector. */
    {
        .type = UI_ITEM_INPUT_SELECT,
        .label = "SRC",
        .x = INPUT_FIELD_X,
        .y = FIELD_Y(VALUE_ROW_1),
        .w = INPUT_FIELD_W,
        .h = FIELD_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = INPUT_TEXT_CHARS,
        .active_color = COLOR_ORANGE,
        .data = NULL,
        .format = HMI_FormatInputSource,
        .render = HMI_InputSelectRender,
        .on_press = HMI_InputSelectPress,
        .on_encoder = HMI_InputSelectEncoder
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
        .type = UI_ITEM_FLOAT_ENTRY,
        .label = "Fs",
        .x = POS_FIELD_X,
        .y = FIELD_Y(VALUE_ROW_2),
        .w = POS_FIELD_W,
        .h = FIELD_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = PI_TEXT_CHARS,
        .active_color = COLOR_RED,
        .data = &hmi_pos_fs,
        .format = HMI_FormatFloatEntry,
        .render = HMI_FloatEntryRender,
        .on_press = HMI_FloatEntryPress,
        .on_encoder = HMI_FloatEntryEncoder
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
        .type = UI_ITEM_FLOAT_ENTRY,
        .label = "Fs",
        .x = VEL_FIELD_X,
        .y = FIELD_Y(VALUE_ROW_2),
        .w = VEL_FIELD_W,
        .h = FIELD_H,
        .selectable = 1U,
        .is_selected = 0U,
        .is_on = 0U,
        .text_chars = PI_TEXT_CHARS,
        .active_color = COLOR_BLUE,
        .data = &hmi_vel_fs,
        .format = HMI_FormatFloatEntry,
        .render = HMI_FloatEntryRender,
        .on_press = HMI_FloatEntryPress,
        .on_encoder = HMI_FloatEntryEncoder
    }
};

#define HMI_ITEM_COUNT ((uint8_t)(sizeof(hmi_items) / sizeof(hmi_items[0])))

static HmiPage_t hmi_page = HMI_PAGE_DIAGRAM;
static uint8_t hmi_selected_index = 0U;
static uint8_t hmi_editing = 0U;
static uint8_t hmi_tracking_lock = 0U;
static uint16_t hmi_last_encoder_raw = 0U;
static int32_t hmi_encoder_accum = 0;
static uint32_t hmi_last_value_refresh_ms = 0U;
static uint32_t hmi_last_plot_refresh_ms = 0U;
static uint16_t hmi_plot_x = 0U;

static uint8_t hmi_preset_slot = 0U;
static uint8_t hmi_preset_selected = 0U;
static uint8_t hmi_preset_editing = 0U;

static uint8_t HMI_ItemIsVisible(uint8_t index)
{
    if (index >= HMI_ITEM_COUNT)
    {
        return 0U;
    }

    if (index == HMI_INDEX_INPUT_SOURCE)
    {
        return (desired_state == STATE_FREQUENCY_RESPONSE) ? 0U : 1U;
    }

    if ((index == HMI_INDEX_POS_KP) ||
        (index == HMI_INDEX_POS_KI) ||
        (index == HMI_INDEX_POS_FS))
    {
        return ((desired_state == STATE_POSITION_CONTROL) ||
                (desired_state == STATE_FREQUENCY_RESPONSE)) ? 1U : 0U;
    }

    if ((index == HMI_INDEX_VEL_KP) ||
        (index == HMI_INDEX_VEL_KI) ||
        (index == HMI_INDEX_VEL_FS))
    {
        return ((desired_state == STATE_POSITION_CONTROL) ||
                (desired_state == STATE_VELOCITY_CONTROL) ||
                (desired_state == STATE_FREQUENCY_RESPONSE)) ? 1U : 0U;
    }

    return 1U;
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
    hmi_last_encoder_raw = (uint16_t)TIM4->CNT;
    hmi_encoder_accum = 0;
}

static int32_t HMI_ConsumeEncoderSteps(void)
{
    uint16_t raw = (uint16_t)TIM4->CNT;
    uint16_t raw_delta = (uint16_t)(raw - hmi_last_encoder_raw);
    int32_t delta = (int32_t)((int16_t)raw_delta);

    hmi_last_encoder_raw = raw;

    if (delta > HMI_ENCODER_DELTA_LIMIT)
    {
        delta = HMI_ENCODER_DELTA_LIMIT;
    }
    else if (delta < -HMI_ENCODER_DELTA_LIMIT)
    {
        delta = -HMI_ENCODER_DELTA_LIMIT;
    }

    hmi_encoder_accum += delta;

    if (hmi_encoder_accum > HMI_ENCODER_ACCUM_LIMIT)
    {
        hmi_encoder_accum = HMI_ENCODER_ACCUM_LIMIT;
    }
    else if (hmi_encoder_accum < -HMI_ENCODER_ACCUM_LIMIT)
    {
        hmi_encoder_accum = -HMI_ENCODER_ACCUM_LIMIT;
    }

    int32_t steps = hmi_encoder_accum / HMI_ENCODER_COUNTS_PER_STEP;
    if (steps != 0)
    {
        hmi_encoder_accum -= (steps * HMI_ENCODER_COUNTS_PER_STEP);
    }

    return steps;
}

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
        (void)snprintf(dst, dst_len, "%sk", n);
    }
    else
    {
        HMI_FloatToStr(n, sizeof(n), hz, 0U);
        (void)snprintf(dst, dst_len, "%s", n);
    }
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
        return (int16_t)(PLOT_Y + (PLOT_H / 2U));
    }

    float frac = (value - min_value) / span;
    return (int16_t)((float)(PLOT_Y + PLOT_H - 2U) - (frac * (float)(PLOT_H - 3U)));
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
    if ((index >= HMI_ITEM_COUNT) || (HMI_ItemIsVisible(index) == 0U))
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

static void HMI_RenderDynamicItems(void)
{
    for (uint8_t i = 0U; i < HMI_ITEM_COUNT; i++)
    {
        if (((hmi_items[i].type == UI_ITEM_READOUT) ||
             (hmi_items[i].type == UI_ITEM_FLOAT_ENTRY) ||
             (hmi_items[i].type == UI_ITEM_INPUT_SELECT)) &&
            (HMI_ItemIsVisible(i) != 0U))
        {
            HMI_RenderItem(i);
        }
    }
}

static void HMI_RenderModeButtons(void)
{
    HMI_RenderItem(HMI_INDEX_VELOCITY);
    HMI_RenderItem(HMI_INDEX_POSITION);
    HMI_RenderItem(HMI_INDEX_FREQUENCY);
    HMI_RenderItem(HMI_INDEX_START);
    HMI_RenderItem(HMI_INDEX_PRESET1);
    HMI_RenderItem(HMI_INDEX_PRESET2);
}

static uint8_t HMI_SyncButtonStates(void)
{
    uint8_t changed = 0U;

    /*
     * Mode buttons show desired_state so the user can choose the startup mode
     * even while current_state is STATE_DISABLED. START shows whether tracking
     * is currently active or pending.
     */
    uint8_t velocity_on = (desired_state == STATE_VELOCITY_CONTROL) ? 1U : 0U;
    uint8_t position_on = (desired_state == STATE_POSITION_CONTROL) ? 1U : 0U;
    uint8_t frequency_on = (desired_state == STATE_FREQUENCY_RESPONSE) ? 1U : 0U;
    uint8_t start_on = ((Control_IsTrackingEnabled() != 0U) ||
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

    if (hmi_items[HMI_INDEX_FREQUENCY].is_on != frequency_on)
    {
        hmi_items[HMI_INDEX_FREQUENCY].is_on = frequency_on;
        changed = 1U;
    }

    if (hmi_items[HMI_INDEX_START].is_on != start_on)
    {
        hmi_items[HMI_INDEX_START].is_on = start_on;
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
            if (hmi_page == HMI_PAGE_DIAGRAM)
            {
                HMI_SelectIndex(HMI_INDEX_START);
            }
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
        HMI_RenderItem(hmi_selected_index);
        return;
    }

    hmi_items[old_index].is_selected = 0U;
    hmi_selected_index = new_index;
    hmi_items[hmi_selected_index].is_selected = 1U;

    HMI_RenderItem(old_index);
    HMI_RenderItem(hmi_selected_index);
}

static void HMI_RedrawDiagramPage(void)
{
    hmi_page = HMI_PAGE_DIAGRAM;
    HMI_ResetEncoderAccumulator();
    ControlLoopDisplay_DrawForMode(desired_state);
    (void)HMI_SyncButtonStates();
    HMI_EnsureSelectedVisible();
    HMI_RenderAllItems();
    hmi_last_value_refresh_ms = HAL_GetTick();
}

static void HMI_RequestVelocityMode(void)
{
    Control_SetDesiredState(STATE_VELOCITY_CONTROL);
    if (hmi_page == HMI_PAGE_DIAGRAM)
    {
        HMI_RedrawDiagramPage();
    }
}

static void HMI_RequestPositionMode(void)
{
    Control_SetDesiredState(STATE_POSITION_CONTROL);
    if (hmi_page == HMI_PAGE_DIAGRAM)
    {
        HMI_RedrawDiagramPage();
    }
}

static void HMI_RequestFrequencyMode(void)
{
    Control_SetDesiredState(STATE_FREQUENCY_RESPONSE);
    if (hmi_page == HMI_PAGE_DIAGRAM)
    {
        HMI_RedrawDiagramPage();
    }
}

static void HMI_RequestStartToggle(void)
{
    hmi_editing = 0U;
    HMI_SelectIndex(HMI_INDEX_START);

    if ((Control_IsTrackingEnabled() == 0U) && (tracking_toggle_request != 0U))
    {
        /* START was pressed again before TIM6 consumed the pending start.
         * Treat this as a cancel instead of queuing another start. */
        tracking_toggle_request = 0U;
        hmi_tracking_lock = 0U;
        HMI_RedrawDiagramPage();
    }
    else if (Control_IsTrackingEnabled() == 0U)
    {
        tracking_toggle_request = 1U;
        hmi_tracking_lock = 1U;
        HMI_ShowPlotPage();
    }
    else
    {
        tracking_toggle_request = 1U;
        hmi_tracking_lock = 1U;
        HMI_RedrawDiagramPage();
    }
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
    else if (self == &hmi_items[HMI_INDEX_FREQUENCY])
    {
        HMI_RequestFrequencyMode();
    }
    else if (self == &hmi_items[HMI_INDEX_START])
    {
        HMI_RequestStartToggle();
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

    if (self->label[0] == 'F')
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
    uint16_t fg = HMI_FG;
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

    if (entry->apply != NULL)
    {
        value = entry->apply(value);
    }
    else
    {
        *(entry->value) = value;
    }
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
    (void)snprintf(dst, dst_len, "SRC:%s", Control_GetInputSourceName(control_input_source));
}

static void HMI_InputSelectRender(UiItem_t *self)
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

    if (steps > 0)
    {
        while (steps > 0)
        {
            Control_NextInputSource(1);
            steps--;
        }
    }
    else
    {
        while (steps < 0)
        {
            Control_NextInputSource(-1);
            steps++;
        }
    }
}

static void HMI_FormatPreset(UiItem_t *self, char *dst, size_t dst_len)
{
    (void)self;
    (void)snprintf(dst, dst_len, "%s", self->label);
}

static void HMI_PresetButtonPress(UiItem_t *self)
{
    if (self == NULL)
    {
        return;
    }

    uint8_t slot = (self == &hmi_items[HMI_INDEX_PRESET2]) ? 1U : 0U;
    HMI_OpenPresetMenu(slot);
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

        if ((hmi_items[index].selectable != 0U) &&
            (HMI_ItemIsVisible(index) != 0U))
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

    if (hmi_page != HMI_PAGE_DIAGRAM)
    {
        return;
    }

    if ((selected->type == UI_ITEM_BUTTON) ||
        (selected->type == UI_ITEM_PRESET_BUTTON))
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
    int32_t steps = HMI_ConsumeEncoderSteps();

    if (steps == 0)
    {
        return;
    }

    HMI_HandleEncoderSteps(steps);
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
        if ((hmi_items[i].selectable != 0U) &&
            (HMI_ItemIsVisible(i) != 0U))
        {
            return i;
        }
    }

    return HMI_INVALID_INDEX;
}

/* -------------------------------------------------------------------------- */
/* Plot pages                                                                 */
/* -------------------------------------------------------------------------- */

static void HMI_DrawPlotBase(const char *title)
{
    ILI9341_setRotation(1U);
    ILI9341_fillRect(0U, 0U, 320U, 240U, HMI_BG);
    ILI9341_printString(6U, PLOT_TOP_TEXT_Y, title, HMI_FG, HMI_BG, HMI_TEXT_SCALE);
    ILI9341_printString(210U, PLOT_TOP_TEXT_Y, "BTN=DIAGRAM", HMI_DIM, HMI_BG, HMI_TEXT_SCALE);

    ILI9341_fillRect(PLOT_X, PLOT_Y, PLOT_W, PLOT_H, HMI_BG);
    HMI_DrawRectBorder(PLOT_X, PLOT_Y, PLOT_W, PLOT_H, HMI_DIM, 1U);
}

static void HMI_DrawAxisLabels(const char *x_label,
                               const char *y_label,
                               const char *top_label,
                               const char *mid_label,
                               const char *bottom_label)
{
    ILI9341_printString(6U,
                        (uint16_t)(PLOT_Y + 2U),
                        top_label,
                        HMI_DIM,
                        HMI_BG,
                        HMI_TEXT_SCALE);

    ILI9341_printString(10U,
                        (uint16_t)(PLOT_Y + (PLOT_H / 2U) - 4U),
                        mid_label,
                        HMI_DIM,
                        HMI_BG,
                        HMI_TEXT_SCALE);

    ILI9341_printString(2U,
                        (uint16_t)(PLOT_Y + PLOT_H - 8U),
                        bottom_label,
                        HMI_DIM,
                        HMI_BG,
                        HMI_TEXT_SCALE);

    ILI9341_printString((uint16_t)(PLOT_X + 2U),
                        (uint16_t)(PLOT_Y - 12U),
                        y_label,
                        HMI_DIM,
                        HMI_BG,
                        HMI_TEXT_SCALE);

    ILI9341_printString((uint16_t)(PLOT_X + 82U),
                        (uint16_t)(PLOT_Y + PLOT_H + 6U),
                        x_label,
                        HMI_DIM,
                        HMI_BG,
                        HMI_TEXT_SCALE);
}

static void HMI_DrawTimePlotFrame(void)
{
    if (desired_state == STATE_VELOCITY_CONTROL)
    {
        HMI_DrawPlotBase("VELOCITY LIVE TIME PLOT");
        ILI9341_printString(6U, 28U, "blue=target  red=measured", HMI_DIM, HMI_BG, HMI_TEXT_SCALE);
        HMI_DrawAxisLabels("X: Time (scroll)",
                           "Y: Velocity (RPM)",
                           "120",
                           "0",
                           "-120");
    }
    else
    {
        HMI_DrawPlotBase("POSITION LIVE TIME PLOT");
        ILI9341_printString(6U, 28U, "blue=target  red=measured", HMI_DIM, HMI_BG, HMI_TEXT_SCALE);
        HMI_DrawAxisLabels("X: Time (scroll)",
                           "Y: Position (deg)",
                           "180",
                           "0",
                           "-180");
    }

    int16_t y_mid = HMI_MapY(0.0f,
                             (desired_state == STATE_VELOCITY_CONTROL) ? PLOT_VEL_MIN_RPM : PLOT_POS_MIN_DEG,
                             (desired_state == STATE_VELOCITY_CONTROL) ? PLOT_VEL_MAX_RPM : PLOT_POS_MAX_DEG);
    HMI_DrawLine((int16_t)(PLOT_X + 1U), y_mid, (int16_t)(PLOT_X + PLOT_W - 2U), y_mid, HMI_DIM);

    hmi_plot_x = 0U;
}

static void HMI_UpdateTimePlot(void)
{
    float min_value;
    float max_value;
    float target;
    float measured;

    if (desired_state == STATE_VELOCITY_CONTROL)
    {
        min_value = PLOT_VEL_MIN_RPM;
        max_value = PLOT_VEL_MAX_RPM;
        target = target_velocity1;
        measured = measured_velocity1;
    }
    else
    {
        min_value = PLOT_POS_MIN_DEG;
        max_value = PLOT_POS_MAX_DEG;
        target = target_position1;
        measured = encoder_measured_position1;
    }

    if (hmi_plot_x >= (PLOT_W - 2U))
    {
        HMI_DrawTimePlotFrame();
    }

    uint16_t x = (uint16_t)(PLOT_X + 1U + hmi_plot_x);
    ILI9341_fillRect(x, (uint16_t)(PLOT_Y + 1U), 2U, (uint16_t)(PLOT_H - 2U), HMI_BG);

    int16_t y_mid = HMI_MapY(0.0f, min_value, max_value);
    HMI_DrawPixel((int16_t)x, y_mid, HMI_DIM);

    int16_t y_target = HMI_MapY(target, min_value, max_value);
    int16_t y_meas = HMI_MapY(measured, min_value, max_value);

    ILI9341_fillRect(x, (uint16_t)y_target, 2U, 2U, COLOR_BLUE);
    ILI9341_fillRect(x, (uint16_t)y_meas, 2U, 2U, COLOR_RED);

    hmi_plot_x++;
}

static void HMI_DrawBodeFrame(void)
{
    HMI_DrawPlotBase("FREQUENCY RESPONSE - BODE MAG");
    ILI9341_printString(6U, 28U, "mag=output/input", HMI_DIM, HMI_BG, HMI_TEXT_SCALE);
    HMI_DrawAxisLabels("X: Frequency (Hz, log)",
                       "Y: Magnitude (dB)",
                       "20",
                       "0",
                       "-60");
    ILI9341_printString((uint16_t)(PLOT_X + 2U),
                        (uint16_t)(PLOT_Y + PLOT_H - 10U),
                        "0.1Hz",
                        HMI_DIM,
                        HMI_BG,
                        HMI_TEXT_SCALE);
    ILI9341_printString((uint16_t)(PLOT_X + PLOT_W - 34U),
                        (uint16_t)(PLOT_Y + PLOT_H - 10U),
                        "10Hz",
                        HMI_DIM,
                        HMI_BG,
                        HMI_TEXT_SCALE);

    int16_t y0 = HMI_MapY(0.0f, PLOT_BODE_MIN_DB, PLOT_BODE_MAX_DB);
    HMI_DrawLine((int16_t)(PLOT_X + 1U), y0, (int16_t)(PLOT_X + PLOT_W - 2U), y0, HMI_DIM);
}

static void HMI_UpdateBodePlot(void)
{
    char f_text[16];
    char status[48];

    ILI9341_fillRect((uint16_t)(PLOT_X + 1U),
                     (uint16_t)(PLOT_Y + 1U),
                     (uint16_t)(PLOT_W - 2U),
                     (uint16_t)(PLOT_H - 2U),
                     HMI_BG);

    int16_t y0 = HMI_MapY(0.0f, PLOT_BODE_MIN_DB, PLOT_BODE_MAX_DB);
    HMI_DrawLine((int16_t)(PLOT_X + 1U), y0, (int16_t)(PLOT_X + PLOT_W - 2U), y0, HMI_DIM);

    uint16_t count = freq_response_point_count;
    if (count > CONTROL_FREQ_RESPONSE_POINTS)
    {
        count = CONTROL_FREQ_RESPONSE_POINTS;
    }

    int16_t prev_x = 0;
    int16_t prev_y = 0;

    for (uint16_t i = 0U; i < count; i++)
    {
        int16_t x = (int16_t)(PLOT_X + 1U +
                      ((uint32_t)i * (uint32_t)(PLOT_W - 3U)) /
                      (uint32_t)(CONTROL_FREQ_RESPONSE_POINTS - 1U));
        int16_t y = HMI_MapY(freq_response_mag_db[i], PLOT_BODE_MIN_DB, PLOT_BODE_MAX_DB);

        ILI9341_fillRect((uint16_t)x, (uint16_t)y, 3U, 3U, COLOR_MAGENTA);
        if (i > 0U)
        {
            HMI_DrawLine(prev_x, prev_y, x, y, COLOR_MAGENTA);
        }
        prev_x = x;
        prev_y = y;
    }

    HMI_FloatToStr(f_text, sizeof(f_text), freq_response_current_hz, 2U);
    (void)snprintf(status,
                   sizeof(status),
                   "F=%sHz  Pts=%u/%u  %s",
                   f_text,
                   (unsigned)count,
                   (unsigned)CONTROL_FREQ_RESPONSE_POINTS,
                   (Control_FrequencyResponseIsDone() != 0U) ? "DONE" : "RUN");

    ILI9341_fillRect(6U, PLOT_HINT_Y, 300U, 12U, HMI_BG);
    ILI9341_printString(6U, PLOT_HINT_Y, status, HMI_FG, HMI_BG, HMI_TEXT_SCALE);
}

static void HMI_ShowPlotPage(void)
{
    hmi_page = HMI_PAGE_PLOT;
    HMI_ResetEncoderAccumulator();
    hmi_last_plot_refresh_ms = HAL_GetTick();

    if (desired_state == STATE_FREQUENCY_RESPONSE)
    {
        HMI_DrawBodeFrame();
        HMI_UpdateBodePlot();
    }
    else
    {
        HMI_DrawTimePlotFrame();
    }
}

static void HMI_ReturnFromPlotToDiagram(void)
{
    /* Returning from the plot is a display navigation action only.
     * The controller keeps running; press START again from the diagram to stop. */
    HMI_RedrawDiagramPage();
}

static void HMI_PlotTask(void)
{
    HMI_SyncTrackingLock();

    if (Button_WasPressed() != 0U)
    {
        HMI_ReturnFromPlotToDiagram();
        return;
    }

    uint32_t now = HAL_GetTick();
    uint32_t period = (desired_state == STATE_FREQUENCY_RESPONSE) ? HMI_BODE_REFRESH_MS : HMI_PLOT_REFRESH_MS;

    if ((now - hmi_last_plot_refresh_ms) >= period)
    {
        hmi_last_plot_refresh_ms = now;
        if (desired_state == STATE_FREQUENCY_RESPONSE)
        {
            HMI_UpdateBodePlot();
        }
        else
        {
            HMI_UpdateTimePlot();
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Preset mini menu                                                           */
/* -------------------------------------------------------------------------- */

#define PRESET_MENU_COUNT            7U
#define PRESET_ITEM_POS_KP           0U
#define PRESET_ITEM_POS_KI           1U
#define PRESET_ITEM_VEL_KP           2U
#define PRESET_ITEM_VEL_KI           3U
#define PRESET_ITEM_SAVE_CUR         4U
#define PRESET_ITEM_APPLY            5U
#define PRESET_ITEM_BACK             6U

#define PRESET_MENU_X                34U
#define PRESET_MENU_Y                42U
#define PRESET_MENU_W                252U
#define PRESET_ROW_H                 20U
#define PRESET_ROW_GAP               3U

static float HMI_PresetClamp(uint8_t field, float value)
{
    float min_value = (field == PRESET_ITEM_POS_KP || field == PRESET_ITEM_VEL_KP) ? HMI_FLOAT_MIN_KP : HMI_FLOAT_MIN_KI;
    float max_value = (field == PRESET_ITEM_POS_KP || field == PRESET_ITEM_VEL_KP) ? HMI_FLOAT_MAX_KP : HMI_FLOAT_MAX_KI;

    if (value < min_value)
    {
        value = min_value;
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
        case PRESET_ITEM_POS_KP:
            return preset.pos_kp;
        case PRESET_ITEM_POS_KI:
            return preset.pos_ki;
        case PRESET_ITEM_VEL_KP:
            return preset.vel_kp;
        case PRESET_ITEM_VEL_KI:
            return preset.vel_ki;
        default:
            return 0.0f;
    }
}

static const char *HMI_GetPresetFieldName(uint8_t field)
{
    switch (field)
    {
        case PRESET_ITEM_POS_KP:
            return "POS Kp";
        case PRESET_ITEM_POS_KI:
            return "POS Ki";
        case PRESET_ITEM_VEL_KP:
            return "VEL Kp";
        case PRESET_ITEM_VEL_KI:
            return "VEL Ki";
        case PRESET_ITEM_SAVE_CUR:
            return "SAVE CUR";
        case PRESET_ITEM_APPLY:
            return "APPLY";
        case PRESET_ITEM_BACK:
            return "BACK";
        default:
            return "?";
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
    if (steps > 0)
    {
        while (steps > 0)
        {
            hmi_preset_selected++;
            if (hmi_preset_selected >= PRESET_MENU_COUNT)
            {
                hmi_preset_selected = 0U;
            }
            steps--;
        }
    }
    else
    {
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
        HMI_RedrawDiagramPage();
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
    /* The control-loop display uses rotation 1. Keep the same coordinate frame. */
    ILI9341_setRotation(1U);

    hmi_page = HMI_PAGE_DIAGRAM;
    HMI_ResetEncoderAccumulator();
    hmi_editing = 0U;
    hmi_tracking_lock = 0U;
    hmi_last_value_refresh_ms = HAL_GetTick();
    hmi_last_plot_refresh_ms = HAL_GetTick();

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

    HMI_SyncTrackingLock();
    HMI_ProcessEncoder();

    if (HMI_SyncButtonStates() != 0U)
    {
        HMI_RenderModeButtons();
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
        HMI_RenderDynamicItems();
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
        if (desired_state == STATE_FREQUENCY_RESPONSE)
        {
            HMI_DrawBodeFrame();
            HMI_UpdateBodePlot();
        }
        else
        {
            HMI_DrawTimePlotFrame();
        }
    }
    else
    {
        HMI_DrawPresetMenu();
    }
}
