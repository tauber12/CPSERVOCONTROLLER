#include "main.h"

void SystemClock_Config(void);

MotorController_t ctx_pos = {0};
MotorController_t ctx_vel = {0};

void interrupt_Priorities( void ){


	NVIC_SetPriority(EXTI0_IRQn,       0);  // encoder edge timestamp (T-method)

	NVIC_SetPriority(TIM5_IRQn,        1);  // velocity loop  — 5 kHz
	NVIC_SetPriority(TIM6_DAC_IRQn,    2);  // position loop  — 500 Hz

	NVIC_SetPriority(TIM3_IRQn,        3);  // µs timebase
	NVIC_SetPriority(ADC1_2_IRQn,      4);  // ADC conversion complete


   NVIC_SetPriority(TIM7_IRQn,        5);  // Button Poll

}

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  PI_Init(&ctx_vel, 20.0f, 0.5f, 0.0002f, -100, 100);  // 5kHz
  PI_Init(&ctx_pos, 10.0f, 1.0f, 0.002f, -200, 200);  // 500Hz

  GPIOC_C3_C4_Output_Init();
  GPIOC_C5_C6_Output_Init();
  setup_TIM1_A8();
  Button_Init();
  setup_TIM7_ButtonPoll();

  Encoder_Config();        // EXTI0 configured here
  ADC_init();
  interrupt_Priorities();  // set priorities after all peripherals initialized
  setup_LOOPTIMERS();      // enable interrupts last

  while (1)
  {
	  // Menu Navigation handled here
     if (Button_WasPressed())
     {
         tracking_toggle_request = 1;
     }

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
