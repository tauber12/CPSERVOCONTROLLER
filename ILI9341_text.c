/*
 *******************************************************************************
 * @file           : ILI9341_text.c
 * @brief          : Text rendering functions for ILI9341 TFT display
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.1
 * date            : May 25, 2026
 * compiler        : STM32CubeIDE v.1.19.0 Build: 14980_20230301_1550 (UTC)
 * target          : NUCLEO-L4A6ZG
 * clocks          : 48 MHz MSI to AHB2
 * @attention      : (c) 2026 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * Description:
 *   Provides character and string rendering on top of the ILI9341 SPI driver.
 *   Uses a 5x7 pixel font with a 1-pixel column gap between characters.
 *
 *   Coordinate system:
 *     (0,0) = top-left corner of display
 *     x increases rightward, y increases downward
 *     Display is 240 (wide) x 320 (tall) in default portrait orientation
 *
 *   Color format:
 *     RGB565 — 16 bits per pixel
 *     | R4 R3 R2 R1 R0 | G5 G4 G3 G2 G1 G0 | B4 B3 B2 B1 B0 |
 *     Use the ILI9341_COLOR() macro or the predefined color constants.
 *
 * Usage example:
 *   ILI9341_fillScreen(COLOR_BLACK);
 *   ILI9341_printString(10, 10, "Hello, World!", COLOR_WHITE, COLOR_BLACK, 2);
 *
 *******************************************************************************
 * Version History
 *  Ver.|   Date   |  Description
 *  ---------------------------------------------------------------------------
 *  0.1 | 05-25-26 | Initial implementation
 *******************************************************************************
 */

#include "ILI9341_text.h"
#include "font5x7.h"

/* -----------------------------------------------------------------------------
 * function : ILI9341_setAddressWindow()
 * INs      : x0, y0 — top-left pixel of window
 *            x1, y1 — bottom-right pixel of window (inclusive)
 * OUTs     : none
 * action   : Sends column address set (0x2A) and row address set (0x2B)
 *            commands to define the rectangular region that subsequent
 *            memory write (0x2C) pixel data will fill.
 * -------------------------------------------------------------------------- */
void ILI9341_setAddressWindow(uint16_t x0, uint16_t y0,
                               uint16_t x1, uint16_t y1)
{
    // column address set
    ILI9341_writeCommand(0x2A);
    ILI9341_writeData(x0 >> 8);
    ILI9341_writeData(x0 & 0xFF);
    ILI9341_writeData(x1 >> 8);
    ILI9341_writeData(x1 & 0xFF);

    // row address set
    ILI9341_writeCommand(0x2B);
    ILI9341_writeData(y0 >> 8);
    ILI9341_writeData(y0 & 0xFF);
    ILI9341_writeData(y1 >> 8);
    ILI9341_writeData(y1 & 0xFF);

    // memory write — pixel data follows
    ILI9341_writeCommand(0x2C);
}

/* -----------------------------------------------------------------------------
 * function : ILI9341_fillRect()
 * INs      : x, y   — top-left corner of rectangle
 *            w, h   — width and height in pixels
 *            color  — RGB565 fill color
 * OUTs     : none
 * action   : Fills a solid rectangle by setting the address window and
 *            pushing (w * h) pixels of the same color.
 * -------------------------------------------------------------------------- */
void ILI9341_fillRect(uint16_t x, uint16_t y,
                      uint16_t w, uint16_t h,
                      uint16_t color)
{
    ILI9341_setAddressWindow(x, y, x + w - 1, y + h - 1);

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    TFT_DC_DATA();
    TFT_CS_LOW();

    uint32_t total = (uint32_t)w * h;
    for (uint32_t i = 0; i < total; i++)
    {
        SPI1_write8(hi);
        SPI1_write8(lo);
    }

    TFT_CS_HIGH();
}

/* -----------------------------------------------------------------------------
 * function : ILI9341_fillScreen()
 * INs      : color — RGB565 color
 * OUTs     : none
 * action   : Fills the entire 240x320 display with a solid color.
 * -------------------------------------------------------------------------- */
void ILI9341_fillScreen(uint16_t color)
{
    ILI9341_fillRect(0, 0, ILI9341_WIDTH, ILI9341_HEIGHT, color);
}

/* -----------------------------------------------------------------------------
 * function : ILI9341_drawChar()
 * INs      : x, y      — top-left pixel of character cell
 *            c         — ASCII character to draw
 *            fg        — foreground (text) color, RGB565
 *            bg        — background color, RGB565
 *            scale     — integer scale factor (1 = 5x7, 2 = 10x14, etc.)
 * OUTs     : none
 * action   : Looks up the character bitmap in font5x7, then pushes pixel
 *            data column-by-column into the address window. Each font bit
 *            produces a (scale x scale) block of pixels. A 1-pixel-wide
 *            (scaled) gap column of background color is added after the
 *            character for spacing.
 *
 * Notes:
 *   - Characters outside [FONT_START, FONT_END] are drawn as a space.
 *   - Total cell width  = (FONT_WIDTH + 1) * scale pixels
 *   - Total cell height = FONT_HEIGHT * scale pixels
 * -------------------------------------------------------------------------- */
void ILI9341_drawChar(uint16_t x, uint16_t y,
                      char c,
                      uint16_t fg, uint16_t bg,
                      uint8_t scale)
{
    // clamp out-of-range characters to space
    if (c < FONT_START || c > FONT_END)
    {
        c = ' ';
    }

    const uint8_t *bitmap = font5x7[c - FONT_START];

    uint16_t cell_w = (FONT_WIDTH + 1) * scale;  // +1 column gap
    uint16_t cell_h = FONT_HEIGHT * scale;

    ILI9341_setAddressWindow(x, y, x + cell_w - 1, y + cell_h - 1);

    uint8_t fg_hi = fg >> 8;
    uint8_t fg_lo = fg & 0xFF;
    uint8_t bg_hi = bg >> 8;
    uint8_t bg_lo = bg & 0xFF;

    TFT_DC_DATA();
    TFT_CS_LOW();

    // iterate rows top to bottom
    for (uint8_t row = 0; row < FONT_HEIGHT; row++)
    {
        // each row may be scaled vertically
        for (uint8_t sy = 0; sy < scale; sy++)
        {
            // iterate columns left to right (5 font columns + 1 gap)
            for (uint8_t col = 0; col < FONT_WIDTH + 1; col++)
            {
                // determine if this pixel is foreground or background
                uint8_t lit = 0;
                if (col < FONT_WIDTH)
                {
                    // bit 'row' of bitmap column 'col'
                    lit = (bitmap[col] >> row) & 0x01;
                }
                // col == FONT_WIDTH is the gap column, always background

                uint8_t hi = lit ? fg_hi : bg_hi;
                uint8_t lo = lit ? fg_lo : bg_lo;

                // each pixel may be scaled horizontally
                for (uint8_t sx = 0; sx < scale; sx++)
                {
                    SPI1_write8(hi);
                    SPI1_write8(lo);
                }
            }
        }
    }

    TFT_CS_HIGH();
}

/* -----------------------------------------------------------------------------
 * function : ILI9341_printString()
 * INs      : x, y   — top-left pixel of first character
 *            str    — null-terminated ASCII string
 *            fg     — foreground (text) color, RGB565
 *            bg     — background color, RGB565
 *            scale  — integer scale factor passed through to drawChar()
 * OUTs     : none
 * action   : Calls ILI9341_drawChar() for each character in str, advancing
 *            x by one character cell width after each character. Does not
 *            wrap lines — the caller is responsible for positioning.
 * -------------------------------------------------------------------------- */
void ILI9341_printString(uint16_t x, uint16_t y,
                         const char *str,
                         uint16_t fg, uint16_t bg,
                         uint8_t scale)
{
    uint16_t char_w = (FONT_WIDTH + 1) * scale;

    while (*str)
    {
        ILI9341_drawChar(x, y, *str, fg, bg, scale);
        x += char_w;
        str++;
    }
}
