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
  TFT_SPI_init();
  ILI9341_init();
  HAL_Delay(200);


/*#define NUM_RECTS 5
#define RECT_W    30
#define RECT_H    20

typedef struct {
    int16_t x, y;      // current position
    int16_t dx, dy;    // velocity (+1 or -1)
    uint16_t color;
} Rect_t;

void rectangleAnimation(void)
{
    Rect_t rects[NUM_RECTS] = {
        { 10,  20,  2,  1, COLOR_RED     },
        { 80,  60, -2,  2, COLOR_CYAN    },
        { 150, 100,  1, -2, COLOR_YELLOW  },
        { 50,  200, -1,  1, COLOR_MAGENTA },
        { 200, 280,  2, -1, COLOR_ORANGE  },
    };

    // erase positions — track previous x/y to clear only dirty region
    int16_t prev_x[NUM_RECTS];
    int16_t prev_y[NUM_RECTS];

    // init previous positions
    for (uint8_t i = 0; i < NUM_RECTS; i++) {
        prev_x[i] = rects[i].x;
        prev_y[i] = rects[i].y;
    }

    ILI9341_fillScreen(COLOR_BLACK);

    for (uint16_t frame = 0; frame < 2000; frame++)
    {
        for (uint8_t i = 0; i < NUM_RECTS; i++)
        {
            // erase previous position
            ILI9341_fillRect(prev_x[i], prev_y[i], RECT_W, RECT_H, COLOR_BLACK);

            // update position
            rects[i].x += rects[i].dx;
            rects[i].y += rects[i].dy;

            // bounce off walls — change color on bounce like DVD logo
            if (rects[i].x <= 0) {
                rects[i].x  = 0;
                rects[i].dx = -rects[i].dx;
                rects[i].color = colors[(i + frame) % CLR_COUNT];
            }
            if (rects[i].x >= ILI9341_WIDTH - RECT_W) {
                rects[i].x  = ILI9341_WIDTH - RECT_W;
                rects[i].dx = -rects[i].dx;
                rects[i].color = colors[(i + frame + 1) % CLR_COUNT];
            }
            if (rects[i].y <= 0) {
                rects[i].y  = 0;
                rects[i].dy = -rects[i].dy;
                rects[i].color = colors[(i + frame + 2) % CLR_COUNT];
            }
            if (rects[i].y >= ILI9341_HEIGHT - RECT_H) {
                rects[i].y  = ILI9341_HEIGHT - RECT_H;
                rects[i].dy = -rects[i].dy;
                rects[i].color = colors[(i + frame + 3) % CLR_COUNT];
            }

            // draw at new position
            ILI9341_fillRect(rects[i].x, rects[i].y, RECT_W, RECT_H, rects[i].color);

            // save for next erase
            prev_x[i] = rects[i].x;
            prev_y[i] = rects[i].y;
        }

        HAL_Delay(8);  // ~120 frames/sec effective, tune to taste
    }
}*/

  ControlLoopDisplay_Draw();
  while (1)
  {
	  //rectangleAnimation();

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
