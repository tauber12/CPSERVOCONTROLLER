/*
 *******************************************************************************
 * @file           : ILI9341.c
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
 *   SPI clock = 48 MHz / 4 = 12 MHz (BR[2:0] = 0b001).
 *
 * Pins:
 *   PA5 = SPI1_SCK
 *   PA6 = SPI1_MISO  (optional / unused for write-only operation)
 *   PA7 = SPI1_MOSI
 *   PB0 = TFT_CS     (active low, software controlled)
 *   PB1 = TFT_DC     (low = command, high = data)
 *   PB2 = TFT_RST    (active low hardware reset)
 *
 *******************************************************************************
 * Version History
 *  Ver.|   Date   |  Description
 *  ---------------------------------------------------------------------------
 *  0.1 | 05-25-26 | Initial implementation
 *  0.2 | 05-28-26 | Fix SPI RX FIFO / OVR flag accumulation in SPI1_write8
 *                 | Call ILI9341_reset() at top of ILI9341_init()
 *                 | Clean up stale comments
 *******************************************************************************
 */

#include "ILI9341.h"
#include "ILI9341_text.h"

/* -----------------------------------------------------------------------------
 * function : TFT_SPI_init()
 * INs      : none
 * OUTs     : none
 * action   : Sets up pins for SPI1 and configures SPI1 control registers
 *            for communication with the ILI9341 TFT display.
 *
 * Pins:
 *            PA5 = SPI1_SCK
 *            PA6 = SPI1_MISO, optional / unused
 *            PA7 = SPI1_MOSI
 *            PB0 = TFT_CS
 *            PB1 = TFT_DC
 *            PB2 = TFT_RST
 * -------------------------------------------------------------------------- */
void TFT_SPI_init(void)
{
    // enable clock for GPIOA, GPIOB, and SPI1
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* -------------------------------------------------------------------------
     * Configure PA5, PA6, PA7 for SPI1 alternate function
     * ---------------------------------------------------------------------- */

    // PA5=SCK, PA6=MISO, PA7=MOSI: alternate function mode
    GPIOA->MODER &= ~(GPIO_MODER_MODE5 |
                      GPIO_MODER_MODE6 |
                      GPIO_MODER_MODE7);

    GPIOA->MODER |=  (GPIO_MODER_MODE5_1 |
                      GPIO_MODER_MODE6_1 |
                      GPIO_MODER_MODE7_1);

    // PA5 SPI1_SCK AF5
    GPIOA->AFR[0] &= ~(0x000F << GPIO_AFRL_AFSEL5_Pos);
    GPIOA->AFR[0] |=  (0x0005 << GPIO_AFRL_AFSEL5_Pos);

    // PA6 SPI1_MISO AF5
    GPIOA->AFR[0] &= ~(0x000F << GPIO_AFRL_AFSEL6_Pos);
    GPIOA->AFR[0] |=  (0x0005 << GPIO_AFRL_AFSEL6_Pos);

    // PA7 SPI1_MOSI AF5
    GPIOA->AFR[0] &= ~(0x000F << GPIO_AFRL_AFSEL7_Pos);
    GPIOA->AFR[0] |=  (0x0005 << GPIO_AFRL_AFSEL7_Pos);

    // SPI pins as push-pull
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5 |
                       GPIO_OTYPER_OT6 |
                       GPIO_OTYPER_OT7);

    // SPI pins very high speed
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED5 |
                       GPIO_OSPEEDR_OSPEED6 |
                       GPIO_OSPEEDR_OSPEED7);

    // no pull-up / pull-down on SPI pins
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5 |
                      GPIO_PUPDR_PUPD6 |
                      GPIO_PUPDR_PUPD7);

    /* -------------------------------------------------------------------------
     * Configure PB0, PB1, PB2 as regular GPIO outputs
     * ---------------------------------------------------------------------- */

    // PB0=CS, PB1=DC, PB2=RST: output mode
    GPIOB->MODER &= ~(GPIO_MODER_MODE0 |
                      GPIO_MODER_MODE1 |
                      GPIO_MODER_MODE2);

    GPIOB->MODER |=  (GPIO_MODER_MODE0_0 |
                      GPIO_MODER_MODE1_0 |
                      GPIO_MODER_MODE2_0);

    // push-pull outputs
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT0 |
                       GPIO_OTYPER_OT1 |
                       GPIO_OTYPER_OT2);

    // high speed outputs
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0 |
                       GPIO_OSPEEDR_OSPEED1 |
                       GPIO_OSPEEDR_OSPEED2);

    // no pull-up / pull-down
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD0 |
                      GPIO_PUPDR_PUPD1 |
                      GPIO_PUPDR_PUPD2);

    // default TFT control pin states
    GPIOB->ODR |=  GPIO_ODR_OD0;   // CS  high (inactive)
    GPIOB->ODR |=  GPIO_ODR_OD1;   // DC  high (data mode)
    GPIOB->ODR |=  GPIO_ODR_OD2;   // RST high (not in reset)

    /* -------------------------------------------------------------------------
     * Configure SPI1
     * ---------------------------------------------------------------------- */

    // disable SPI for configuration
    SPI1->CR1 &= ~(SPI_CR1_SPE);

    // CR1 configuration
    SPI1->CR1 &= ~(SPI_CR1_RXONLY);              // full-duplex
    SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);            // MSB first
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA); // SPI mode 0
    SPI1->CR1 |=  SPI_CR1_MSTR;                  // master mode

    // software slave management — CS driven manually on PB0
    SPI1->CR1 |= SPI_CR1_SSM;
    SPI1->CR1 |= SPI_CR1_SSI;

    // baud rate: 48 MHz / 4 = 12 MHz  (BR[2:0] = 0b001)
    SPI1->CR1 &= ~(SPI_CR1_BR);
    SPI1->CR1 |=  SPI_CR1_BR_0;

    // CR2 configuration
    SPI1->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE); // no SPI interrupts
    SPI1->CR2 &= ~(SPI_CR2_FRF);                    // Motorola frame format
    SPI1->CR2 &= ~(SPI_CR2_NSSP);                   // no hardware NSS pulse
    SPI1->CR2 &= ~(SPI_CR2_SSOE);                   // no hardware SS output

    // 8-bit data size
    SPI1->CR2 &= ~(SPI_CR2_DS);
    SPI1->CR2 |=  (0x7 << SPI_CR2_DS_Pos);

    // FIFO threshold: assert RXNE when 8 bits (1 byte) received
    // Required for correct OVR/RXNE behaviour at 8-bit data size
    SPI1->CR2 |= SPI_CR2_FRXTH;

    // enable SPI
    SPI1->CR1 |= SPI_CR1_SPE;
}

/* -----------------------------------------------------------------------------
 * function : ILI9341_init()
 * INs      : none
 * OUTs     : none
 * action   : Issues a hardware reset then sends the minimum command sequence
 *            to bring the ILI9341 out of sleep, configure the pixel format,
 *            fill the screen black, and turn the display on.
 * -------------------------------------------------------------------------- */
void ILI9341_init(void)
{
    // hardware reset before sending any commands
    // guarantees a known register state regardless of power-on conditions
    ILI9341_reset();

    // Sleep Out — starts oscillator, DC-DC converter, and panel scanning
    // datasheet requires 120 ms before next command after SLPOUT
    ILI9341_writeCommand(0x11);
    HAL_Delay(120);

    // pixel format = 16-bit RGB565
    ILI9341_writeCommand(0x3A);
    ILI9341_writeData(0x55);

    // memory access control — portrait orientation, RGB order
    ILI9341_writeCommand(0x36);
    ILI9341_writeData(0x48);

    // fill black while display is still off to avoid flash of garbage
    ILI9341_fillScreen(COLOR_WHITE);

    // display on
    ILI9341_writeCommand(0x29);
    HAL_Delay(120);
}

/* -----------------------------------------------------------------------------
 * function : SPI1_write8()
 * INs      : data — byte to transmit
 * OUTs     : none
 * action   : Blocks until the TX FIFO has room, writes the byte, then waits
 *            for BSY to clear (transaction complete). Flushes the RX FIFO
 *            and clears the OVR flag afterward to prevent flag accumulation
 *            across successive calls, which can lock the peripheral.
 *
 * Note: SPI_CR2_FRXTH must be set (done in TFT_SPI_init) so that RXNE
 *       asserts on a single received byte rather than two.
 * -------------------------------------------------------------------------- */
void SPI1_write8(uint8_t data)
{
    // wait until TX FIFO has room for a byte
    while (!(SPI1->SR & SPI_SR_TXE));

    // write byte — use 8-bit pointer access to avoid pushing a 16/32-bit frame
    *(volatile uint8_t *)&SPI1->DR = data;

    // wait until the bus is idle (shift register empty)
    while (SPI1->SR & SPI_SR_BSY);

    // flush any bytes that arrived in the RX FIFO during the transmission
    // (full-duplex SPI clocks in a byte for every byte sent)
    while (SPI1->SR & SPI_SR_RXNE)
    {
        (void)(*(volatile uint8_t *)&SPI1->DR);
    }

    // clear overrun flag: read DR then read SR
    // (harmless if OVR is not set)
    (void)(*(volatile uint8_t *)&SPI1->DR);
    (void)SPI1->SR;
}

/* -----------------------------------------------------------------------------
 * function : ILI9341_writeCommand()
 * INs      : command — ILI9341 command byte
 * OUTs     : none
 * action   : Asserts DC low (command mode), drives CS low, sends the byte,
 *            then releases CS.
 * -------------------------------------------------------------------------- */
void ILI9341_writeCommand(uint8_t command)
{
    TFT_DC_COMMAND();
    TFT_CS_LOW();

    SPI1_write8(command);

    TFT_CS_HIGH();
}

/* -----------------------------------------------------------------------------
 * function : ILI9341_writeData()
 * INs      : data — data byte to send
 * OUTs     : none
 * action   : Asserts DC high (data mode), drives CS low, sends the byte,
 *            then releases CS.
 * -------------------------------------------------------------------------- */
void ILI9341_writeData(uint8_t data)
{
    TFT_DC_DATA();
    TFT_CS_LOW();

    SPI1_write8(data);

    TFT_CS_HIGH();
}

/* -----------------------------------------------------------------------------
 * function : ILI9341_writeDataBuffer()
 * INs      : buffer — pointer to byte array
 *            length — number of bytes to send
 * OUTs     : none
 * action   : Sends a contiguous block of data bytes with a single CS
 *            assertion. More efficient than repeated ILI9341_writeData()
 *            calls for pixel data. Future DMA integration point.
 * -------------------------------------------------------------------------- */
void ILI9341_writeDataBuffer(uint8_t *buffer, uint32_t length)
{
    TFT_DC_DATA();
    TFT_CS_LOW();

    for (uint32_t i = 0; i < length; i++)
    {
        SPI1_write8(buffer[i]);
    }

    TFT_CS_HIGH();
}

/* -----------------------------------------------------------------------------
 * function : ILI9341_reset()
 * INs      : none
 * OUTs     : none
 * action   : Pulses RST low for 20 ms then releases it and waits 150 ms for
 *            the display controller to complete its internal reset sequence.
 *            Call before ILI9341_init() to guarantee a known register state.
 * -------------------------------------------------------------------------- */
void ILI9341_reset(void)
{
    TFT_RST_LOW();
    HAL_Delay(20);

    TFT_RST_HIGH();
    HAL_Delay(150);
}
