#include "main.h"

void SystemClock_Config(void);

MotorController_t ctx_pos = {0};
MotorController_t ctx_vel = {0};

void interrupt_Priorities( void ){


	NVIC_SetPriority(EXTI0_IRQn,       0);  // encoder edge timestamp (T-method)

	NVIC_SetPriority(TIM5_IRQn,        1);  // velocity loop  — 5 kHz
	NVIC_SetPriority(TIM6_DAC_IRQn,    2);  // position loop  — 500 Hz

	NVIC_SetPriority(TIM3_IRQn,        3);  // extended us timebase
	NVIC_SetPriority(ADC1_2_IRQn,      4);  // ADC conversion complete
	NVIC_SetPriority(TIM4_IRQn,        5);  // HMI encoder overflow
	NVIC_SetPriority(TIM7_IRQn,        6);  // Button Poll

}

typedef enum {
    CLR_BLACK = 0,
    CLR_WHITE,
    CLR_RED,
    CLR_GREEN,
    CLR_BLUE,
    CLR_YELLOW,
    CLR_CYAN,
    CLR_MAGENTA,
    CLR_ORANGE,
    CLR_GRAY,
    CLR_COUNT
} ColorIndex_t;

const uint16_t colors[CLR_COUNT] = {
    [CLR_BLACK]   = COLOR_BLACK,
    [CLR_WHITE]   = COLOR_WHITE,
    [CLR_RED]     = COLOR_RED,
    [CLR_GREEN]   = COLOR_GREEN,
    [CLR_BLUE]    = COLOR_BLUE,
    [CLR_YELLOW]  = COLOR_YELLOW,
    [CLR_CYAN]    = COLOR_CYAN,
    [CLR_MAGENTA] = COLOR_MAGENTA,
    [CLR_ORANGE]  = COLOR_ORANGE,
    [CLR_GRAY]    = COLOR_GRAY,
};


int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Core controller state first */
    PI_Init(&ctx_vel,
            CONTROL_DEFAULT_VEL_KP,
            CONTROL_DEFAULT_VEL_KI,
            1.0f / CONTROL_DEFAULT_VEL_HZ,
            CONTROL_DEFAULT_VEL_RANGE_PWM,
            CONTROL_DEFAULT_VEL_ERR_RPM);

    PI_Init(&ctx_pos,
            CONTROL_DEFAULT_POS_KP,
            CONTROL_DEFAULT_POS_KI,
            1.0f / CONTROL_DEFAULT_POS_HZ,
            CONTROL_DEFAULT_POS_RANGE_RPM,
            CONTROL_DEFAULT_POS_ERR_DEG);
    Control_InitPresetsFromCurrent();                     // seed P1/P2

    /* GPIO / motor output hardware */
    GPIOC_C3_C4_Output_Init();
    GPIOC_C5_C6_Output_Init();
    setup_TIM1_A8();

    /* Sensor / input hardware */
    Encoder_Config();          // motor encoder
    ADC_init();

    /* HMI input hardware */
    Button_Init();
    setup_TIM7_ButtonPoll();
    HMI_Encoder_Config();      // UI knob encoder

    /* Display hardware */
    TFT_SPI_init();
    ILI9341_init();

    /*
     * Draw static display first.
     * This clears and draws the block diagram background.
     */
    ControlLoopDisplay_Draw();

    /*
     * Draw HMI UI items on top of the static display.
     * This renders buttons and numeric entries.
     */
    UI_Init();

    /*
     * Configure interrupt priorities after peripherals exist,
     * but before enabling the control-loop timers.
     */
    interrupt_Priorities();

    /*
     * Enable real-time control-loop interrupts last.
     * After this, TIM5/TIM6 start running motor-control code.
     */
    setup_LOOPTIMERS();

    while (1)
    {
        UI_Task();
    }
}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
