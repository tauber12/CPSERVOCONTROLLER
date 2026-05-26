/*
 *******************************************************************************
 * @file           : ILI9341.c
 * @brief          : X
 * project         : EE 329 S'26 AX
 * authors         : joeym
 * version         : 0.1
 * date            : May 25, 2026
 * compiler        : STM32CubeIDE v.1.19.0 Build: 14980_20230301_1550 (UTC)
 * target          : NUCLEO-L4A6ZG
 * clocks          : 48 MHz MSI to AHB2
 * @attention      : (c) 2026 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * Description: X
 *
 *******************************************************************************
 * Version History
 *  Ver.|   Date   |  Description
 *  ---------------------------------------------------------------------------
 *      |          | 
 *******************************************************************************
 *
 * Header format adapted from [Code Appendix by Kevin Vo] pg 5
 */

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
    GPIOB->ODR |=  GPIO_ODR_OD0;   // CS high, inactive
    GPIOB->ODR |=  GPIO_ODR_OD1;   // DC high, data mode
    GPIOB->ODR |=  GPIO_ODR_OD2;   // RST high, not reset

    /* -------------------------------------------------------------------------
     * Configure SPI1
     * ---------------------------------------------------------------------- */

    // disable SPI for configuration
    SPI1->CR1 &= ~(SPI_CR1_SPE);

    // CR1 configuration
    SPI1->CR1 &= ~(SPI_CR1_RXONLY);              // transmit/receive mode
    SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);            // MSB first
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA); // SPI mode 0
    SPI1->CR1 |=  SPI_CR1_MSTR;                  // MCU is SPI master

    // software slave management because CS is controlled manually on PB0
    SPI1->CR1 |= SPI_CR1_SSM;
    SPI1->CR1 |= SPI_CR1_SSI;

    // baud rate
    // 48 MHz / 4 = 12 MHz SPI clock
    SPI1->CR1 &= ~(SPI_CR1_BR);
    SPI1->CR1 |=  SPI_CR1_BR_0;

    // CR2 configuration
    SPI1->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE); // disable SPI interrupts
    SPI1->CR2 &= ~(SPI_CR2_FRF);                    // Motorola frame format
    SPI1->CR2 &= ~(SPI_CR2_NSSP);                   // no hardware NSS pulse
    SPI1->CR2 &= ~(SPI_CR2_SSOE);                   // no hardware SS output

    // 8-bit data size for ILI9341 commands/data
    SPI1->CR2 &= ~(SPI_CR2_DS);
    SPI1->CR2 |=  (0x7 << SPI_CR2_DS_Pos);

    // enable SPI
    SPI1->CR1 |= SPI_CR1_SPE;
}

void ILI9341_init(void)
{
    ILI9341_reset();

    // software reset
    ILI9341_writeCommand(0x01);
    HAL_Delay(150);

    // sleep out
    ILI9341_writeCommand(0x11);
    HAL_Delay(150);

    // pixel format = 16-bit RGB565
    ILI9341_writeCommand(0x3A);
    ILI9341_writeData(0x55);

    // memory access control
    // 0x48 is common portrait RGB setting
    ILI9341_writeCommand(0x36);
    ILI9341_writeData(0x48);

    // display on
    ILI9341_writeCommand(0x29);
    HAL_Delay(50);
}

void SPI1_write8(uint8_t data)
{
    // wait until transmit buffer is empty
    while (!(SPI1->SR & SPI_SR_TXE))
    {
        ;
    }

    // write data
    SPI1->DR = data;

    // wait until SPI is done transmitting
    while (SPI1->SR & SPI_SR_BSY)
    {
        ;
    }
}

void ILI9341_writeCommand(uint8_t command)
{
    TFT_DC_COMMAND();
    TFT_CS_LOW();

    SPI1_write8(command);

    TFT_CS_HIGH();
}

void ILI9341_writeData(uint8_t data)
{
    TFT_DC_DATA();
    TFT_CS_LOW();

    SPI1_write8(data);

    TFT_CS_HIGH();
}

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

void ILI9341_reset(void)
{
    TFT_RST_LOW();
    HAL_Delay(20);

    TFT_RST_HIGH();
    HAL_Delay(150);
}

