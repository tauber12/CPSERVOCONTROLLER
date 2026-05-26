/*
 *******************************************************************************
 * @file           : ILI9341_text.h
 * @brief          : Text rendering functions for ILI9341 TFT display
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.1
 * date            : May 25, 2026
 * compiler        : STM32CubeIDE v.1.19.0 Build: 14980_20230301_1550 (UTC)
 * target          : NUCLEO-L4A6ZG
 *******************************************************************************
 */

#ifndef ILI9341_TEXT_H
#define ILI9341_TEXT_H

#include <stdint.h>
#include "ILI9341.h"

/* -----------------------------------------------------------------------------
 * Display dimensions (portrait orientation, matches 0x36 MADCTL = 0x48)
 * -------------------------------------------------------------------------- */
#define ILI9341_WIDTH   240
#define ILI9341_HEIGHT  320

/* -----------------------------------------------------------------------------
 * RGB565 color macro
 *   r : 0-31
 *   g : 0-63
 *   b : 0-31
 * -------------------------------------------------------------------------- */
#define ILI9341_COLOR(r, g, b) \
    ((uint16_t)(((r) & 0x1F) << 11) | (((g) & 0x3F) << 5) | ((b) & 0x1F))

/* -----------------------------------------------------------------------------
 * Common predefined colors (RGB565)
 * -------------------------------------------------------------------------- */
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F
#define COLOR_ORANGE    0xFD20
#define COLOR_GRAY      0x8410

/* -----------------------------------------------------------------------------
 * Function prototypes
 * -------------------------------------------------------------------------- */

// Set the active pixel write window on the display
void ILI9341_setAddressWindow(uint16_t x0, uint16_t y0,
                               uint16_t x1, uint16_t y1);

// Fill a rectangle with a solid color
void ILI9341_fillRect(uint16_t x, uint16_t y,
                      uint16_t w, uint16_t h,
                      uint16_t color);

// Fill the entire screen with a solid color
void ILI9341_fillScreen(uint16_t color);

// Draw a single scaled ASCII character
// scale=1: 6x7 pixels per char (5 wide + 1 gap)
// scale=2: 12x14 pixels per char, etc.
void ILI9341_drawChar(uint16_t x, uint16_t y,
                      char c,
                      uint16_t fg, uint16_t bg,
                      uint8_t scale);

// Draw a null-terminated string starting at (x, y)
void ILI9341_printString(uint16_t x, uint16_t y,
                         const char *str,
                         uint16_t fg, uint16_t bg,
                         uint8_t scale);

#endif // ILI9341_TEXT_H
