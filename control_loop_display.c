/*
 * control_loop_display.c
 *
 * Static landscape ILI9341 control-loop diagram.
 *
 * This file owns only the non-interactive drawing:
 *   - screen rotation/background
 *   - mode-select title/top band
 *   - mode-specific control-loop diagrams
 *   - arrows, summing junctions, feedback paths
 *
 * Interactive objects are intentionally not drawn here anymore. Buttons,
 * editable gains, presets, and live numeric readouts are rendered by HMI.c.
 */

#include "control_loop_display.h"

#include <stdint.h>

#include "stm32l4xx_hal.h"
#include "ILI9341_text.h"

#define CLD_W                  320U
#define CLD_H                  240U

#define CLD_BG                 COLOR_WHITE
#define CLD_FG                 COLOR_BLACK
#define CLD_DIM                COLOR_GRAY
#define CLD_POS                COLOR_RED
#define CLD_VEL                COLOR_BLUE
#define CLD_IN                 COLOR_ORANGE
#define CLD_PLANT              COLOR_GREEN
#define CLD_FREQ               COLOR_MAGENTA

#define CLD_TEXT_SCALE         1U
#define CLD_CHAR_W             6U
#define CLD_LINE_THICK         2U
#define CLD_SUM_R              8U

/* Landscape 320 x 240 layout. These dimensions match the HMI item layout. */
#define MODE_TITLE_X           5U
#define MODE_TITLE_Y           3U

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

static Controller_State cld_last_mode = STATE_POSITION_CONTROL;

static void CLD_SwapU16(uint16_t *a, uint16_t *b)
{
    uint16_t t = *a;
    *a = *b;
    *b = t;
}

static uint16_t CLD_TextWidth(const char *s)
{
    uint16_t n = 0U;

    if (s == 0)
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

static void CLD_Block(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      uint16_t border, const char *line0, const char *line1)
{
    CLD_FillAndBorder(x, y, w, h, CLD_BG, border);
    CLD_PrintCentered(x, (uint16_t)(y + 5U), w, line0, border, CLD_BG);

    if (line1 != 0)
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

static void CLD_DrawSum(uint16_t cx, uint16_t cy, uint16_t color)
{
    CLD_Circle((int16_t)cx, (int16_t)cy, (int16_t)CLD_SUM_R, color);
    ILI9341_printString((uint16_t)(cx - 3U), (uint16_t)(cy - 6U), "+", color, CLD_BG, CLD_TEXT_SCALE);
    ILI9341_printString((uint16_t)(cx - 3U), (uint16_t)(cy + 3U), "-", color, CLD_BG, CLD_TEXT_SCALE);
}

static void CLD_DrawStaticModeArea(void)
{
    /* This band is only the frame/title. The actual buttons are HMI items. */
    CLD_FillAndBorder(0U, 0U, CLD_W, 47U, CLD_BG, CLD_DIM);
    ILI9341_printString(MODE_TITLE_X,
                        MODE_TITLE_Y,
                        "MODE / PRESETS",
                        CLD_FG,
                        CLD_BG,
                        CLD_TEXT_SCALE);
}

static void CLD_DrawPlantAndEncoder(void)
{
    const uint16_t plant_center_x = (uint16_t)(PLANT_X + (PLANT_W / 2U));

    CLD_Block(PLANT_X, BLOCK_Y, PLANT_W, BLOCK_H, CLD_PLANT, "PWM", "MOTOR");

    ILI9341_printString((uint16_t)(PLANT_X + 15U),
                        (uint16_t)(BLOCK_Y + 27U),
                        "LOAD",
                        CLD_FG,
                        CLD_BG,
                        CLD_TEXT_SCALE);
    ILI9341_printString((uint16_t)(PLANT_X + 7U),
                        (uint16_t)(BLOCK_Y + 41U),
                        "TIM1",
                        CLD_FG,
                        CLD_BG,
                        CLD_TEXT_SCALE);
    ILI9341_printString((uint16_t)(PLANT_X + 6U),
                        (uint16_t)(BLOCK_Y + 53U),
                        "20kHz",
                        CLD_FG,
                        CLD_BG,
                        CLD_TEXT_SCALE);

    CLD_FillAndBorder(ENC_X, ENC_Y, ENC_W, ENC_H, CLD_BG, CLD_DIM);
    CLD_PrintCentered(ENC_X, (uint16_t)(ENC_Y + 4U), ENC_W, "ENC", CLD_FG, CLD_BG);
    CLD_PrintCentered(ENC_X, (uint16_t)(ENC_Y + 15U), ENC_W, "POS/VEL", CLD_FG, CLD_BG);

    CLD_ArrowDown(plant_center_x, (uint16_t)(BLOCK_Y + BLOCK_H), (uint16_t)(ENC_Y - 1U), CLD_FG);
}

static void CLD_DrawStaticDiagramForMode(Controller_State mode)
{
    const uint16_t enc_center_x = (uint16_t)(ENC_X + (ENC_W / 2U));
    uint8_t frequency_mode = (mode == STATE_FREQUENCY_RESPONSE) ? 1U : 0U;
    uint16_t input_color = (frequency_mode != 0U) ? CLD_FREQ : CLD_IN;

    if (frequency_mode != 0U)
    {
        CLD_Block(INPUT_X, BLOCK_Y, INPUT_W, BLOCK_H, input_color, "SINE", "SWEEP");
        ILI9341_printString((uint16_t)(INPUT_X + 8U),
                            (uint16_t)(BLOCK_Y + 53U),
                            "BODE",
                            input_color,
                            CLD_BG,
                            CLD_TEXT_SCALE);
    }
    else
    {
        CLD_Block(INPUT_X, BLOCK_Y, INPUT_W, BLOCK_H, input_color, "INPUT", "MUX");
        ILI9341_printString((uint16_t)(INPUT_X + 10U),
                            (uint16_t)(BLOCK_Y + 53U),
                            "SRC",
                            input_color,
                            CLD_BG,
                            CLD_TEXT_SCALE);
    }

    if (mode == STATE_VELOCITY_CONTROL)
    {
        /* Velocity mode is drawn as a true direct velocity loop. The inactive
         * position controller, its summing junction, and its feedback path are
         * not drawn at all, so stale blocks cannot remain visible.
         */
        CLD_Block(VEL_X, BLOCK_Y, VEL_W, BLOCK_H, CLD_VEL, "VELOCITY", "PI");
        CLD_DrawPlantAndEncoder();

        CLD_ArrowH((uint16_t)(INPUT_X + INPUT_W), (uint16_t)(SUM_VEL_X - CLD_SUM_R), PATH_Y, input_color);
        CLD_ArrowH((uint16_t)(SUM_VEL_X + CLD_SUM_R), VEL_X, PATH_Y, CLD_VEL);
        CLD_ArrowH((uint16_t)(VEL_X + VEL_W), PLANT_X, PATH_Y, CLD_PLANT);
        CLD_DrawSum(SUM_VEL_X, PATH_Y, CLD_VEL);

        CLD_HLine(SUM_VEL_X, ENC_X, VEL_FB_Y, CLD_VEL);
        CLD_ArrowUp(SUM_VEL_X, VEL_FB_Y, (uint16_t)(PATH_Y + CLD_SUM_R + 2U), CLD_VEL);
        ILI9341_printString(190U,
                            (uint16_t)(VEL_FB_Y + 5U),
                            "velocity fb",
                            CLD_VEL,
                            CLD_BG,
                            CLD_TEXT_SCALE);
        return;
    }

    /* Position and frequency-response modes use the cascaded position-velocity
     * loop. Frequency response replaces the normal input source with a sine
     * sweep and relabels the position feedback as the measured amplitude path.
     */
    CLD_Block(POS_X, BLOCK_Y, POS_W, BLOCK_H, CLD_POS, "POSITION", "PI");
    CLD_Block(VEL_X, BLOCK_Y, VEL_W, BLOCK_H, CLD_VEL, "VELOCITY", "PI");
    CLD_DrawPlantAndEncoder();

    CLD_ArrowH((uint16_t)(INPUT_X + INPUT_W), (uint16_t)(SUM_POS_X - CLD_SUM_R), PATH_Y, input_color);
    CLD_ArrowH((uint16_t)(SUM_POS_X + CLD_SUM_R), POS_X, PATH_Y, CLD_POS);
    CLD_ArrowH((uint16_t)(POS_X + POS_W), (uint16_t)(SUM_VEL_X - CLD_SUM_R), PATH_Y, CLD_POS);
    CLD_ArrowH((uint16_t)(SUM_VEL_X + CLD_SUM_R), VEL_X, PATH_Y, CLD_VEL);
    CLD_ArrowH((uint16_t)(VEL_X + VEL_W), PLANT_X, PATH_Y, CLD_PLANT);

    CLD_DrawSum(SUM_POS_X, PATH_Y, CLD_POS);
    CLD_DrawSum(SUM_VEL_X, PATH_Y, CLD_VEL);

    CLD_HLine(SUM_VEL_X, ENC_X, VEL_FB_Y, CLD_VEL);
    CLD_ArrowUp(SUM_VEL_X, VEL_FB_Y, (uint16_t)(PATH_Y + CLD_SUM_R + 2U), CLD_VEL);
    ILI9341_printString(190U,
                        (uint16_t)(VEL_FB_Y + 5U),
                        "velocity fb",
                        CLD_VEL,
                        CLD_BG,
                        CLD_TEXT_SCALE);

    CLD_VLine(enc_center_x, (uint16_t)(ENC_Y + ENC_H), POS_FB_Y, CLD_POS);
    CLD_HLine(SUM_POS_X, enc_center_x, POS_FB_Y, CLD_POS);
    CLD_ArrowUp(SUM_POS_X, POS_FB_Y, (uint16_t)(PATH_Y + CLD_SUM_R + 2U), CLD_POS);
    ILI9341_printString(111U,
                        (uint16_t)(POS_FB_Y + 5U),
                        (frequency_mode != 0U) ? "position amp" : "position fb",
                        CLD_POS,
                        CLD_BG,
                        CLD_TEXT_SCALE);

    if (frequency_mode != 0U)
    {
        ILI9341_printString(6U,
                            160U,
                            "FRF: sweep sinusoids, log output amplitude",
                            CLD_FREQ,
                            CLD_BG,
                            CLD_TEXT_SCALE);
    }
}

void ControlLoopDisplay_Init(void)
{
    /* 1 = landscape, 320 x 240, in the rotation-aware text driver. */
    ILI9341_setRotation(1U);
    ControlLoopDisplay_DrawForMode(cld_last_mode);
}

void ControlLoopDisplay_Draw(void)
{
    ControlLoopDisplay_Init();
}

void ControlLoopDisplay_DrawStatic(void)
{
    ControlLoopDisplay_DrawForMode(cld_last_mode);
}

void ControlLoopDisplay_DrawForMode(Controller_State mode)
{
    if ((mode != STATE_POSITION_CONTROL) &&
        (mode != STATE_VELOCITY_CONTROL) &&
        (mode != STATE_FREQUENCY_RESPONSE))
    {
        mode = STATE_POSITION_CONTROL;
    }

    cld_last_mode = mode;

    ILI9341_setRotation(1U);
    ILI9341_fillRect(0U, 0U, CLD_W, CLD_H, CLD_BG);
    CLD_DrawStaticModeArea();
    CLD_DrawStaticDiagramForMode(mode);
}

void ControlLoopDisplay_Update(void)
{
    /*
     * No dynamic UI is drawn here anymore.
     * HMI.c owns all buttons, editable entries, presets, plots, and readouts.
     */
}
