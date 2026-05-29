/*
 *******************************************************************************
 * @file           : ILI9341.h
 * @brief          : SPI driver for ILI9341 TFT display
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.2
 * date            : May 28, 2026
 * compiler        : STM32CubeIDE v.1.19.0 Build: 14980_20230301_1550 (UTC)
 * target          : NUCLEO-L4A6ZG
 * clocks          : 48 MHz MSI to AHB2
 * @attention      : (c) 2026 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * Description:
 *   Register-level SPI1 driver for the ILI9341 240x320 TFT display.
 *
 * GPIO Wiring
 * | Signal      | Pin | Connector | Config          |
 * |-------------|-----|-----------|-----------------|
 * | SPI1_SCK    | PA5 | CN10-11   | AF5, push-pull  |
 * | SPI1_MISO   | PA6 | CN10-13   | AF5, push-pull  |
 * | SPI1_MOSI   | PA7 | CN10-15   | AF5, push-pull  |
 * | TFT_CS      | PB0 | CN10-31   | GPIO OUT        |
 * | TFT_DC      | PB1 | CN10-7    | GPIO OUT        |
 * | TFT_RST     | PB2 | CN10-9    | GPIO OUT        |
 *
 *******************************************************************************
 * Version History
 *  Ver.|   Date   |  Description
 *  ---------------------------------------------------------------------------
 *  0.1 | 05-25-26 | Initial implementation
 *  0.2 | 05-28-26 | Corrected clock note (48 MHz), updated GPIO table,
 *                 | added FRXTH note
 *******************************************************************************
 */

#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

/* -----------------------------------------------------------------------------
 * Control pin macros — all three lines are on GPIOB
 * CS  = PB0 (active low)
 * DC  = PB1 (low = command, high = data)
 * RST = PB2 (active low)
 * -------------------------------------------------------------------------- */
#define TFT_CS_LOW()       (GPIOB->ODR &= ~(GPIO_ODR_OD0))
#define TFT_CS_HIGH()      (GPIOB->ODR |=  (GPIO_ODR_OD0))

#define TFT_DC_COMMAND()   (GPIOB->ODR &= ~(GPIO_ODR_OD1))
#define TFT_DC_DATA()      (GPIOB->ODR |=  (GPIO_ODR_OD1))

#define TFT_RST_LOW()      (GPIOB->ODR &= ~(GPIO_ODR_OD2))
#define TFT_RST_HIGH()     (GPIOB->ODR |=  (GPIO_ODR_OD2))

/* -----------------------------------------------------------------------------
 * Function prototypes
 * -------------------------------------------------------------------------- */

// Initialise SPI1 peripheral and GPIO pins
void TFT_SPI_init(void);

// Hardware reset pulse then ILI9341 command sequence (calls ILI9341_reset)
void ILI9341_init(void);

// Pulse RST low to hardware-reset the controller
void ILI9341_reset(void);

// Send one byte over SPI, flush RX FIFO, clear OVR flag
void SPI1_write8(uint8_t data);

// Assert DC=command, send one command byte
void ILI9341_writeCommand(uint8_t command);

// Assert DC=data, send one data byte
void ILI9341_writeData(uint8_t data);

// Assert DC=data, send a block of bytes (future DMA integration point)
void ILI9341_writeDataBuffer(uint8_t *buffer, uint32_t length);

#endif /* INC_ILI9341_H_ */
