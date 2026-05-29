/*
 *******************************************************************************
 * @file           : ILI9341_text.c
 * @brief          : Basic rectangle/text helpers for ILI9341 RGB565 display
 *******************************************************************************
 */

#include "ILI9341_text.h"
#include "font5x7.h"

#include <stdint.h>

/* Low-level driver functions from ILI9341.c. They may already be declared in
 * ILI9341.h; these externs are harmless if the prototypes match.
 */
extern void ILI9341_writeCommand(uint8_t command);
extern void ILI9341_writeData(uint8_t data);
extern void ILI9341_writeDataBuffer(uint8_t *buffer, uint32_t length);

static uint16_t ili9341_width = ILI9341_WIDTH;
static uint16_t ili9341_height = ILI9341_HEIGHT;

void ILI9341_setRotation(uint8_t rotation)
{
    rotation &= 0x03U;

    ILI9341_writeCommand(0x36);  /* MADCTL */

    switch (rotation)
    {
        default:
        case 0: /* portrait, 240 x 320 */
            ILI9341_writeData(0x48);
            ili9341_width = 240U;
            ili9341_height = 320U;
            break;

        case 1: /* landscape, 320 x 240 */
            ILI9341_writeData(0x28);
            ili9341_width = 320U;
            ili9341_height = 240U;
            break;

        case 2: /* inverted portrait, 240 x 320 */
            ILI9341_writeData(0x88);
            ili9341_width = 240U;
            ili9341_height = 320U;
            break;

        case 3: /* inverted landscape, 320 x 240 */
            ILI9341_writeData(0xE8);
            ili9341_width = 320U;
            ili9341_height = 240U;
            break;
    }
}

uint16_t ILI9341_getWidth(void)
{
    return ili9341_width;
}

uint16_t ILI9341_getHeight(void)
{
    return ili9341_height;
}

void ILI9341_setAddressWindow(uint16_t x0, uint16_t y0,
                              uint16_t x1, uint16_t y1)
{
    ILI9341_writeCommand(0x2A); /* column address set */
    ILI9341_writeData((uint8_t)(x0 >> 8));
    ILI9341_writeData((uint8_t)(x0 & 0xFFU));
    ILI9341_writeData((uint8_t)(x1 >> 8));
    ILI9341_writeData((uint8_t)(x1 & 0xFFU));

    ILI9341_writeCommand(0x2B); /* page address set */
    ILI9341_writeData((uint8_t)(y0 >> 8));
    ILI9341_writeData((uint8_t)(y0 & 0xFFU));
    ILI9341_writeData((uint8_t)(y1 >> 8));
    ILI9341_writeData((uint8_t)(y1 & 0xFFU));

    ILI9341_writeCommand(0x2C); /* memory write */
}

void ILI9341_fillRect(uint16_t x, uint16_t y,
                      uint16_t w, uint16_t h,
                      uint16_t color)
{
    if ((w == 0U) || (h == 0U) || (x >= ili9341_width) || (y >= ili9341_height))
    {
        return;
    }

    if ((x + w) > ili9341_width)
    {
        w = (uint16_t)(ili9341_width - x);
    }
    if ((y + h) > ili9341_height)
    {
        h = (uint16_t)(ili9341_height - y);
    }

    ILI9341_setAddressWindow(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));

    uint8_t pixel[64U * 2U];
    for (uint32_t i = 0U; i < sizeof(pixel); i += 2U)
    {
        pixel[i] = (uint8_t)(color >> 8);
        pixel[i + 1U] = (uint8_t)(color & 0xFFU);
    }

    uint32_t pixels_left = (uint32_t)w * (uint32_t)h;
    while (pixels_left > 0U)
    {
        uint32_t chunk_pixels = (pixels_left > 64U) ? 64U : pixels_left;
        ILI9341_writeDataBuffer(pixel, chunk_pixels * 2U);
        pixels_left -= chunk_pixels;
    }
}

void ILI9341_fillScreen(uint16_t color)
{
    ILI9341_fillRect(0U, 0U, ili9341_width, ili9341_height, color);
}

void ILI9341_drawChar(uint16_t x, uint16_t y,
                      char c,
                      uint16_t fg, uint16_t bg,
                      uint8_t scale)
{
    if (scale == 0U)
    {
        scale = 1U;
    }

    if ((c < FONT_START) || (c > FONT_END))
    {
        c = '?';
    }

    const uint8_t *glyph = font5x7[(uint8_t)c - FONT_START];

    for (uint8_t col = 0U; col < FONT_WIDTH; col++)
    {
        uint8_t bits = glyph[col];
        for (uint8_t row = 0U; row < FONT_HEIGHT; row++)
        {
            uint16_t color = (bits & (1U << row)) ? fg : bg;
            ILI9341_fillRect((uint16_t)(x + col * scale),
                             (uint16_t)(y + row * scale),
                             scale, scale, color);
        }
    }

    /* One-column space between characters. */
    ILI9341_fillRect((uint16_t)(x + FONT_WIDTH * scale), y,
                     scale, (uint16_t)(FONT_HEIGHT * scale), bg);
}

void ILI9341_printString(uint16_t x, uint16_t y,
                         const char *str,
                         uint16_t fg, uint16_t bg,
                         uint8_t scale)
{
    uint16_t cursor_x = x;

    if (str == 0)
    {
        return;
    }

    while (*str != '\0')
    {
        if (*str == '\n')
        {
            cursor_x = x;
            y = (uint16_t)(y + (FONT_HEIGHT + 1U) * scale);
        }
        else
        {
            ILI9341_drawChar(cursor_x, y, *str, fg, bg, scale);
            cursor_x = (uint16_t)(cursor_x + (FONT_WIDTH + 1U) * scale);
        }
        str++;
    }
}
