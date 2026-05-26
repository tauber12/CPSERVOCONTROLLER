/*
 *******************************************************************************
 * @file           : ILI9341.h
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
 * Description: X
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

#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define TFT_CS_LOW()       (GPIOB->ODR &= ~(GPIO_ODR_OD0))
#define TFT_CS_HIGH()      (GPIOB->ODR |=  (GPIO_ODR_OD0))

#define TFT_DC_COMMAND()   (GPIOB->ODR &= ~(GPIO_ODR_OD1))
#define TFT_DC_DATA()      (GPIOB->ODR |=  (GPIO_ODR_OD1))

#define TFT_RST_LOW()      (GPIOB->ODR &= ~(GPIO_ODR_OD2))
#define TFT_RST_HIGH()     (GPIOB->ODR |=  (GPIO_ODR_OD2))

void TFT_SPI_init(void);
void ILI9341_init(void);
void ILI9341_reset(void);
void SPI1_write8(uint8_t data);
void ILI9341_writeCommand(uint8_t command);
void ILI9341_writeData(uint8_t data);
void ILI9341_writeDataBuffer(uint8_t *buffer, uint32_t length);

#endif /* INC_ILI9341_H_ */
