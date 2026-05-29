/*
 * control_loop_display.c
 *
 * Landscape ILI9341 screen for the current controller architecture:
 *   INPUT SELECT -> position PI -> velocity PI -> PWM/motor/load
 * with velocity-mode bypass, position feedback, and velocity feedback.
 *
 * The display is intentionally organized so each live value is shown inside
 * the block it belongs to:
 *   - Input Select: source, raw ADC count, input voltage, mapped command
 *   - Position PI: Kp, Ki, actual TIM6 sample rate
 *   - Velocity PI: Kp, Ki, actual TIM5 sample rate
 *   - Mode Select: separate boxes for Disabled, Position, Velocity, Fault
 *
 * Call ControlLoopDisplay_Draw() once after TFT_SPI_init() and ILI9341_init().
 * Call ControlLoopDisplay_Update() slowly from the foreground loop, for example
 * every 200 ms. Do not call it from TIM5/TIM6 control-loop interrupts.
 */

#include "control_loop_display.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "stm32l4xx_hal.h"
#include "ILI9341_text.h"
#include "control.h"
#include "ADC.h"

/* Provided by the rotation-aware ILI9341_text.c. The original header may not
 * declare it yet, so keep the extern here for compatibility.
 */
extern void ILI9341_setRotation(uint8_t rotation);

#define CLD_W                  320U
#define CLD_H                  240U

#define CLD_BG                 COLOR_WHITE
#define CLD_FG                 COLOR_BLACK
#define CLD_DIM                COLOR_GRAY
#define CLD_POS                COLOR_RED
#define CLD_VEL                COLOR_BLUE
#define CLD_IN                 COLOR_ORANGE
#define CLD_PLANT              COLOR_GREEN
#define CLD_FAULT              COLOR_MAGENTA

#define CLD_TEXT_SCALE         1U
#define CLD_CHAR_W             6U
#define CLD_LINE_THICK         2U
#define CLD_SUM_R              8U

/* Landscape 320 x 240 layout. */
#define MODE_TITLE_X           5U
#define MODE_TITLE_Y           3U
#define MODE_BOX_Y             19U
#define MODE_BOX_W             74U
#define MODE_BOX_H             24U
#define MODE_BOX_GAP           5U
#define MODE_DISABLED_X        4U
#define MODE_POSITION_X        (MODE_DISABLED_X + MODE_BOX_W + MODE_BOX_GAP)
#define MODE_VELOCITY_X        (MODE_POSITION_X + MODE_BOX_W + MODE_BOX_GAP)
#define MODE_FAULT_X           (MODE_VELOCITY_X + MODE_BOX_W + MODE_BOX_GAP)

#define BYPASS_Y               62U
#define BYPASS_TEXT_X          73U
#define BYPASS_TEXT_Y          51U
#define BYPASS_CLEAR_X         66U
#define BYPASS_CLEAR_Y         49U
#define BYPASS_CLEAR_W         115U
#define BYPASS_CLEAR_H         32U

#define BLOCK_Y                84U
#define BLOCK_H                70U
#define PATH_Y                 119U

#define INPUT_X                4U
#define INPUT_W                64U
#define SUM_POS_X              79U
#define POS_X                  91U
#define POS_W                  68U
#define SUM_VEL_X              171U
#define VEL_X                  183U
#define VEL_W                  68U
#define PLANT_X                263U
#define PLANT_W                53U

#define ENC_X                  PLANT_X
#define ENC_Y                  166U
#define ENC_W                  PLANT_W
#define ENC_H                  25U
#define VEL_FB_Y               178U
#define POS_FB_Y               216U

#define INPUT_VALUE_X          (INPUT_X + 5U)
#define POS_VALUE_X            (POS_X + 5U)
#define VEL_VALUE_X            (VEL_X + 5U)
#define VALUE_ROW_0            (BLOCK_Y + 27U)
#define VALUE_ROW_1            (BLOCK_Y + 39U)
#define VALUE_ROW_2            (BLOCK_Y + 51U)
#define VALUE_ROW_3            (BLOCK_Y + 61U)

#define INPUT_SOURCE_TEXT      "ADC"

static void CLD_SwapU16(uint16_t *a, uint16_t *b)
{
    uint16_t t = *a;
    *a = *b;
    *b = t;
}

static uint16_t CLD_TextWidth(const char *s)
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

    return (uint16_t)(n * CLD_CHAR_W * CLD_TEXT_SCALE);
}

static void CLD_HLine(uint16_t x0, uint16_t x1, uint16_t y, uint16_t color)
{
    if (x1 < x0)
    {
        CLD_SwapU16(&x0, &x1);
    }

    ILI9341_fillRect(x0, y, (uint16_t)(x1 - x0 + 1U), CLD_LINE_THICK, color);
}

static void CLD_VLine(uint16_t x, uint16_t y0, uint16_t y1, uint16_t color)
{
    if (y1 < y0)
    {
        CLD_SwapU16(&y0, &y1);
    }

    ILI9341_fillRect(x, y0, CLD_LINE_THICK, (uint16_t)(y1 - y0 + 1U), color);
}

static void CLD_DrawPixel(int16_t x, int16_t y, uint16_t color)
{
    if ((x < 0) || (y < 0) || (x >= (int16_t)CLD_W) || (y >= (int16_t)CLD_H))
    {
        return;
    }

    ILI9341_fillRect((uint16_t)x, (uint16_t)y, 1U, 1U, color);
}

static void CLD_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    if (y0 == y1)
    {
        CLD_HLine((uint16_t)x0, (uint16_t)x1, (uint16_t)y0, color);
        return;
    }

    if (x0 == x1)
    {
        CLD_VLine((uint16_t)x0, (uint16_t)y0, (uint16_t)y1, color);
        return;
    }

    int16_t dx = (x1 > x0) ? (int16_t)(x1 - x0) : (int16_t)(x0 - x1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t dy = (y1 > y0) ? (int16_t)(y0 - y1) : (int16_t)(y1 - y0);
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = (int16_t)(dx + dy);

    for (;;)
    {
        CLD_DrawPixel(x0, y0, color);
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

static void CLD_Rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ILI9341_fillRect(x, y, w, CLD_LINE_THICK, color);
    ILI9341_fillRect(x, (uint16_t)(y + h - CLD_LINE_THICK), w, CLD_LINE_THICK, color);
    ILI9341_fillRect(x, y, CLD_LINE_THICK, h, color);
    ILI9341_fillRect((uint16_t)(x + w - CLD_LINE_THICK), y, CLD_LINE_THICK, h, color);
}

static void CLD_FillAndBorder(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                              uint16_t fill, uint16_t border)
{
    ILI9341_fillRect(x, y, w, h, fill);
    CLD_Rect(x, y, w, h, border);
}

static void CLD_PrintCentered(uint16_t x, uint16_t y, uint16_t w,
                              const char *text, uint16_t fg, uint16_t bg)
{
    uint16_t tw = CLD_TextWidth(text);
    uint16_t tx = x;

    if (tw < w)
    {
        tx = (uint16_t)(x + ((w - tw) / 2U));
    }

    ILI9341_printString(tx, y, text, fg, bg, CLD_TEXT_SCALE);
}

static void CLD_PrintFixedWidth(uint16_t x, uint16_t y, const char *s,
                                uint16_t chars, uint16_t fg, uint16_t bg)
{
    char padded[40];
    uint16_t i;

    if (chars >= sizeof(padded))
    {
        chars = (uint16_t)(sizeof(padded) - 1U);
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

    ILI9341_printString(x, y, padded, fg, bg, CLD_TEXT_SCALE);
}

static void CLD_Block(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      uint16_t border, const char *line0, const char *line1)
{
    CLD_FillAndBorder(x, y, w, h, CLD_BG, border);
    CLD_PrintCentered(x, (uint16_t)(y + 5U), w, line0, border, CLD_BG);
    if (line1 != NULL)
    {
        CLD_PrintCentered(x, (uint16_t)(y + 16U), w, line1, border, CLD_BG);
    }
}

static void CLD_Circle(int16_t xc, int16_t yc, int16_t r, uint16_t color)
{
    int16_t x = 0;
    int16_t y = r;
    int16_t d = (int16_t)(3 - 2 * r);

    while (y >= x)
    {
        CLD_DrawPixel((int16_t)(xc + x), (int16_t)(yc + y), color);
        CLD_DrawPixel((int16_t)(xc - x), (int16_t)(yc + y), color);
        CLD_DrawPixel((int16_t)(xc + x), (int16_t)(yc - y), color);
        CLD_DrawPixel((int16_t)(xc - x), (int16_t)(yc - y), color);
        CLD_DrawPixel((int16_t)(xc + y), (int16_t)(yc + x), color);
        CLD_DrawPixel((int16_t)(xc - y), (int16_t)(yc + x), color);
        CLD_DrawPixel((int16_t)(xc + y), (int16_t)(yc - x), color);
        CLD_DrawPixel((int16_t)(xc - y), (int16_t)(yc - x), color);

        x++;
        if (d > 0)
        {
            y--;
            d = (int16_t)(d + 4 * (x - y) + 10);
        }
        else
        {
            d = (int16_t)(d + 4 * x + 6);
        }
    }
}

static void CLD_ArrowHeadRight(uint16_t x, uint16_t y, uint16_t color)
{
    CLD_DrawLine((int16_t)x, (int16_t)y, (int16_t)(x - 7U), (int16_t)(y - 5U), color);
    CLD_DrawLine((int16_t)x, (int16_t)y, (int16_t)(x - 7U), (int16_t)(y + 5U), color);
}

static void CLD_ArrowHeadUp(uint16_t x, uint16_t y, uint16_t color)
{
    CLD_DrawLine((int16_t)x, (int16_t)y, (int16_t)(x - 5U), (int16_t)(y + 7U), color);
    CLD_DrawLine((int16_t)x, (int16_t)y, (int16_t)(x + 5U), (int16_t)(y + 7U), color);
}

static void CLD_ArrowHeadDown(uint16_t x, uint16_t y, uint16_t color)
{
    CLD_DrawLine((int16_t)x, (int16_t)y, (int16_t)(x - 5U), (int16_t)(y - 7U), color);
    CLD_DrawLine((int16_t)x, (int16_t)y, (int16_t)(x + 5U), (int16_t)(y - 7U), color);
}

static void CLD_ArrowH(uint16_t x0, uint16_t x1, uint16_t y, uint16_t color)
{
    if (x1 <= x0)
    {
        return;
    }

    CLD_HLine(x0, (uint16_t)(x1 - 1U), y, color);
    CLD_ArrowHeadRight(x1, y, color);
}

static void CLD_ArrowUp(uint16_t x, uint16_t y0, uint16_t y1, uint16_t color)
{
    CLD_VLine(x, y0, (uint16_t)(y1 + 1U), color);
    CLD_ArrowHeadUp(x, y1, color);
}

static void CLD_ArrowDown(uint16_t x, uint16_t y0, uint16_t y1, uint16_t color)
{
    CLD_VLine(x, y0, (uint16_t)(y1 - 1U), color);
    CLD_ArrowHeadDown(x, y1, color);
}

static void CLD_FloatToStr(char *dst, size_t dst_len, float value, uint8_t decimals)
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

static void CLD_RateToStr(char *dst, size_t dst_len, float hz)
{
    char n[16];

    if (hz >= 1000.0f)
    {
        CLD_FloatToStr(n, sizeof(n), hz / 1000.0f, 1U);
        (void)snprintf(dst, dst_len, "%skHz", n);
    }
    else
    {
        CLD_FloatToStr(n, sizeof(n), hz, 0U);
        (void)snprintf(dst, dst_len, "%sHz", n);
    }
}

static float CLD_TimerRateHz(TIM_TypeDef *tim)
{
    uint32_t psc = tim->PSC + 1U;
    uint32_t arr = tim->ARR + 1U;

    if ((psc == 0U) || (arr == 0U))
    {
        return 0.0f;
    }

    return ((float)SystemCoreClock) / ((float)psc * (float)arr);
}

static void CLD_DrawSum(uint16_t cx, uint16_t cy, uint16_t color)
{
    CLD_Circle((int16_t)cx, (int16_t)cy, (int16_t)CLD_SUM_R, color);
    ILI9341_printString((uint16_t)(cx - 3U), (uint16_t)(cy - 6U), "+", color, CLD_BG, CLD_TEXT_SCALE);
    ILI9341_printString((uint16_t)(cx - 3U), (uint16_t)(cy + 3U), "-", color, CLD_BG, CLD_TEXT_SCALE);
}

static void CLD_DrawStaticModeArea(void)
{
    CLD_FillAndBorder(0U, 0U, CLD_W, 47U, CLD_BG, CLD_DIM);
    ILI9341_printString(MODE_TITLE_X, MODE_TITLE_Y,
                        "MODE SELECT", CLD_FG, CLD_BG, CLD_TEXT_SCALE);
}

static void CLD_DrawModeBox(uint16_t x, const char *label, Controller_State mode,
                            uint16_t color, Controller_State active_mode)
{
    uint8_t active = (active_mode == mode) ? 1U : 0U;
    uint16_t fill = active ? color : CLD_BG;
    uint16_t fg = active ? CLD_BG : color;
    uint16_t border = active ? CLD_FG : color;

    if ((active != 0U) && (mode == STATE_DISABLED))
    {
        fg = CLD_FG;
    }

    CLD_FillAndBorder(x, MODE_BOX_Y, MODE_BOX_W, MODE_BOX_H, fill, border);
    CLD_PrintCentered(x, (uint16_t)(MODE_BOX_Y + 8U), MODE_BOX_W, label, fg, fill);
}

static void CLD_DrawModeSelect(Controller_State active_mode)
{
    CLD_DrawModeBox(MODE_DISABLED_X, "DISABLED", STATE_DISABLED, CLD_DIM, active_mode);
    CLD_DrawModeBox(MODE_POSITION_X, "POSITION", STATE_POSITION_CONTROL, CLD_POS, active_mode);
    CLD_DrawModeBox(MODE_VELOCITY_X, "VELOCITY", STATE_VELOCITY_CONTROL, CLD_VEL, active_mode);
    CLD_DrawModeBox(MODE_FAULT_X, "FAULT", STATE_FAULT, CLD_FAULT, active_mode);
}

static void CLD_DrawBypass(Controller_State active_mode)
{
    uint16_t c = (active_mode == STATE_VELOCITY_CONTROL) ? CLD_IN : CLD_DIM;
    const uint16_t bypass_end_y = (uint16_t)(PATH_Y - CLD_SUM_R - 2U);

    ILI9341_fillRect(BYPASS_CLEAR_X, BYPASS_CLEAR_Y,
                     BYPASS_CLEAR_W, BYPASS_CLEAR_H, CLD_BG);

    ILI9341_printString(BYPASS_TEXT_X, BYPASS_TEXT_Y,
                        "VEL MODE BYPASS", c, CLD_BG, CLD_TEXT_SCALE);
    CLD_HLine((uint16_t)(INPUT_X + INPUT_W), SUM_VEL_X, BYPASS_Y, c);
    CLD_ArrowDown(SUM_VEL_X, BYPASS_Y, bypass_end_y, c);
}

static void CLD_DrawStaticDiagram(void)
{
    const uint16_t plant_center_x = (uint16_t)(PLANT_X + (PLANT_W / 2U));
    const uint16_t enc_center_x = (uint16_t)(ENC_X + (ENC_W / 2U));

    CLD_Block(INPUT_X, BLOCK_Y, INPUT_W, BLOCK_H, CLD_IN, "INPUT", "SELECT");
    CLD_Block(POS_X, BLOCK_Y, POS_W, BLOCK_H, CLD_POS, "POSITION", "PI");
    CLD_Block(VEL_X, BLOCK_Y, VEL_W, BLOCK_H, CLD_VEL, "VELOCITY", "PI");
    CLD_Block(PLANT_X, BLOCK_Y, PLANT_W, BLOCK_H, CLD_PLANT, "PWM", "MOTOR");
    ILI9341_printString((uint16_t)(PLANT_X + 15U), (uint16_t)(BLOCK_Y + 27U),
                        "LOAD", CLD_FG, CLD_BG, CLD_TEXT_SCALE);
    ILI9341_printString((uint16_t)(PLANT_X + 7U), (uint16_t)(BLOCK_Y + 41U),
                        "TIM1", CLD_FG, CLD_BG, CLD_TEXT_SCALE);
    ILI9341_printString((uint16_t)(PLANT_X + 6U), (uint16_t)(BLOCK_Y + 53U),
                        "20kHz", CLD_FG, CLD_BG, CLD_TEXT_SCALE);

    CLD_ArrowH((uint16_t)(INPUT_X + INPUT_W), (uint16_t)(SUM_POS_X - CLD_SUM_R), PATH_Y, CLD_IN);
    CLD_ArrowH((uint16_t)(SUM_POS_X + CLD_SUM_R), POS_X, PATH_Y, CLD_POS);
    CLD_ArrowH((uint16_t)(POS_X + POS_W), (uint16_t)(SUM_VEL_X - CLD_SUM_R), PATH_Y, CLD_POS);
    CLD_ArrowH((uint16_t)(SUM_VEL_X + CLD_SUM_R), VEL_X, PATH_Y, CLD_VEL);
    CLD_ArrowH((uint16_t)(VEL_X + VEL_W), PLANT_X, PATH_Y, CLD_PLANT);

    CLD_DrawSum(SUM_POS_X, PATH_Y, CLD_POS);
    CLD_DrawSum(SUM_VEL_X, PATH_Y, CLD_VEL);

    CLD_FillAndBorder(ENC_X, ENC_Y, ENC_W, ENC_H, CLD_BG, CLD_DIM);
    CLD_PrintCentered(ENC_X, (uint16_t)(ENC_Y + 4U), ENC_W, "ENC", CLD_FG, CLD_BG);
    CLD_PrintCentered(ENC_X, (uint16_t)(ENC_Y + 15U), ENC_W, "POS/VEL", CLD_FG, CLD_BG);

    CLD_ArrowDown(plant_center_x, (uint16_t)(BLOCK_Y + BLOCK_H), (uint16_t)(ENC_Y - 1U), CLD_FG);

    /* Velocity feedback to velocity summing junction. */
    CLD_HLine(SUM_VEL_X, ENC_X, VEL_FB_Y, CLD_VEL);
    CLD_ArrowUp(SUM_VEL_X, VEL_FB_Y, (uint16_t)(PATH_Y + CLD_SUM_R + 2U), CLD_VEL);
    ILI9341_printString(190U, (uint16_t)(VEL_FB_Y + 5U),
                        "velocity fb", CLD_VEL, CLD_BG, CLD_TEXT_SCALE);

    /* Position feedback to position summing junction. */
    CLD_VLine(enc_center_x, (uint16_t)(ENC_Y + ENC_H), POS_FB_Y, CLD_POS);
    CLD_HLine(SUM_POS_X, enc_center_x, POS_FB_Y, CLD_POS);
    CLD_ArrowUp(SUM_POS_X, POS_FB_Y, (uint16_t)(PATH_Y + CLD_SUM_R + 2U), CLD_POS);
    ILI9341_printString(111U, (uint16_t)(POS_FB_Y + 5U),
                        "position fb", CLD_POS, CLD_BG, CLD_TEXT_SCALE);
}

static void CLD_UpdateInputBlock(Controller_State active_mode)
{
    char line[18];
    char num[16];

    uint16_t raw = rawVoltageData;
    float input_v = ((float)raw * 3.3f) / 4095.0f;
    float cmd_value = 0.0f;
    const char *cmd_suffix = "off";

    ILI9341_fillRect((uint16_t)(INPUT_X + 3U), (uint16_t)(BLOCK_Y + 26U),
                     (uint16_t)(INPUT_W - 6U), (uint16_t)(BLOCK_H - 29U), CLD_BG);

    CLD_PrintFixedWidth(INPUT_VALUE_X, VALUE_ROW_0, "SRC:" INPUT_SOURCE_TEXT, 9U, CLD_FG, CLD_BG);

    (void)snprintf(line, sizeof(line), "raw=%u", (unsigned int)raw);
    CLD_PrintFixedWidth(INPUT_VALUE_X, VALUE_ROW_1, line, 9U, CLD_FG, CLD_BG);

    CLD_FloatToStr(num, sizeof(num), input_v, 2U);
    (void)snprintf(line, sizeof(line), "V=%s", num);
    CLD_PrintFixedWidth(INPUT_VALUE_X, VALUE_ROW_2, line, 9U, CLD_FG, CLD_BG);

    if (active_mode == STATE_POSITION_CONTROL)
    {
        cmd_value = ((float)raw / 4095.0f) * 360.0f - 180.0f;
        cmd_suffix = "d";
    }
    else if (active_mode == STATE_VELOCITY_CONTROL)
    {
        cmd_value = ((float)raw / 4095.0f) * 200.0f - 100.0f;
        cmd_suffix = "r";
    }

    if ((active_mode == STATE_POSITION_CONTROL) || (active_mode == STATE_VELOCITY_CONTROL))
    {
        CLD_FloatToStr(num, sizeof(num), cmd_value, 0U);
        (void)snprintf(line, sizeof(line), "cmd=%s%s", num, cmd_suffix);
    }
    else
    {
        (void)snprintf(line, sizeof(line), "cmd=%s", cmd_suffix);
    }
    CLD_PrintFixedWidth(INPUT_VALUE_X, VALUE_ROW_3, line, 9U, CLD_IN, CLD_BG);
}

static void CLD_UpdatePIBlock(uint16_t x, uint16_t w, uint16_t color,
                              MotorController_t *ctx, TIM_TypeDef *tim)
{
    char line[18];
    char num[16];
    char rate[16];

    ILI9341_fillRect((uint16_t)(x + 3U), (uint16_t)(BLOCK_Y + 27U),
                     (uint16_t)(w - 6U), (uint16_t)(BLOCK_H - 30U), CLD_BG);

    CLD_FloatToStr(num, sizeof(num), ctx->kp, 2U);
    (void)snprintf(line, sizeof(line), "Kp=%s", num);
    CLD_PrintFixedWidth((uint16_t)(x + 5U), VALUE_ROW_0, line, 10U, CLD_FG, CLD_BG);

    CLD_FloatToStr(num, sizeof(num), ctx->ki, 2U);
    (void)snprintf(line, sizeof(line), "Ki=%s", num);
    CLD_PrintFixedWidth((uint16_t)(x + 5U), VALUE_ROW_1, line, 10U, CLD_FG, CLD_BG);

    CLD_RateToStr(rate, sizeof(rate), CLD_TimerRateHz(tim));
    (void)snprintf(line, sizeof(line), "Fs=%s", rate);
    CLD_PrintFixedWidth((uint16_t)(x + 5U), VALUE_ROW_2, line, 10U, color, CLD_BG);
}

void ControlLoopDisplay_Init(void)
{
    /* 1 = landscape, 320 x 240, in the rotation-aware text driver. */
    ILI9341_setRotation(1U);
    ILI9341_fillScreen(CLD_BG);
    ControlLoopDisplay_DrawStatic();
}

void ControlLoopDisplay_Draw(void)
{
    ControlLoopDisplay_Init();
}

void ControlLoopDisplay_DrawStatic(void)
{
    ILI9341_fillScreen(CLD_BG);

    CLD_DrawStaticModeArea();
    CLD_DrawStaticDiagram();

    ControlLoopDisplay_Update();
}

void ControlLoopDisplay_Update(void)
{
    Controller_State active_mode = state;

    CLD_DrawModeSelect(active_mode);
    CLD_DrawBypass(active_mode);
    CLD_UpdateInputBlock(active_mode);
    CLD_UpdatePIBlock(POS_X, POS_W, CLD_POS, &ctx_pos, TIM6);
    CLD_UpdatePIBlock(VEL_X, VEL_W, CLD_VEL, &ctx_vel, TIM5);
}
