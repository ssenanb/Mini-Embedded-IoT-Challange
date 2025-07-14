#include "main.h"
#include "i2c_lcd.h"
#include "stdio.h"

#define RTC_WRITE_ADDRESS 0xD0
#define RTC_READ_ADDRESS 0xD1


I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2C_LCD_HandleTypeDef lcd;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);

char lcdBuffer[32];
int mode = 1;
volatile int setting_mode = 0; //default mode
int alarm_minute = 0;
int alarm_hour = 0;

typedef struct{
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
}RTC_Time;

RTC_Time time;
RTC_Time previous_time = {255, 255, 255}; // invalid value

uint8_t DEC_To_BCD(uint8_t value){
	return (uint8_t)((value / 10 * 16) + (value % 10));
}

uint8_t BCD_To_DEC(uint8_t value){
	return (uint8_t)((value / 16 * 10) + (value % 16));
}

RTC_Time RTC_Get_Time(){
	RTC_Time time = {0};
	uint8_t time_data[3] = {0};

	if(HAL_I2C_Mem_Read(&hi2c1, RTC_READ_ADDRESS, 0x00, 1, time_data, 3, HAL_MAX_DELAY) == HAL_OK){
		time.second = BCD_To_DEC(time_data[0]);
		time.minute = BCD_To_DEC(time_data[1]);
		time.hour = BCD_To_DEC(time_data[2]);
	}

	return time;
}

void RTC_Set_Time(uint8_t second, uint8_t minute, uint8_t hour){
	uint8_t time[3] = {0};

	time[0] = DEC_To_BCD(second);
	time[1] = DEC_To_BCD(minute);
	time[2] = DEC_To_BCD(hour);

	HAL_I2C_Mem_Write(&hi2c1, RTC_WRITE_ADDRESS, 0x00, 1, time, 3, HAL_MAX_DELAY);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){ // change to the alarm mode

	if(GPIO_Pin == GPIO_PIN_0)
		setting_mode = 1;
}

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  
  RTC_Set_Time(0, 25, 15);

  lcd.hi2c = &hi2c2;
  lcd.address = 0x4E;
  lcd_init(&lcd);

  while (1)
  {
	if(setting_mode == 0){
	  time = RTC_Get_Time();

	  if (time.second != previous_time.second)
	              previous_time = time;

	  sprintf(lcdBuffer, "    %02d:%02d:%02d  ", (int)time.hour, (int)time.minute, (int)time.second);

	  lcd_gotoxy(&lcd, 0, 0);
	  lcd_puts(&lcd, lcdBuffer);
	  }else if(setting_mode == 1){
		  if(mode == 1){
			  lcd_clear(&lcd);
			  lcd_gotoxy(&lcd, 0, 0);
			  lcd_puts(&lcd, "ALARM MODE ON");
			  mode = 2;
		  }

		 if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET){ // hour set
			 HAL_Delay(200); // debounce
			 alarm_hour = (alarm_hour + 1) % 24;

			 sprintf(lcdBuffer,"Hour : %d", alarm_hour);

			 lcd_clear(&lcd);
			 lcd_gotoxy(&lcd, 0, 0);
			 lcd_puts(&lcd, lcdBuffer);
		 }

		 if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET){ // minute set
			 HAL_Delay(200); // debounce
			 alarm_minute = (alarm_minute + 1) % 60;

			 sprintf(lcdBuffer,"Minute : %d", alarm_minute);

			 lcd_clear(&lcd);
			 lcd_gotoxy(&lcd, 0, 0);
			 lcd_puts(&lcd, lcdBuffer);
		 }

		 if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET){
		  	 HAL_Delay(200); // debounce
			 setting_mode = 0;
			 mode = 1;
		 }

		 if(time.minute == alarm_minute && time.hour == alarm_hour){
          		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		 }
	  }
   }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10805D88;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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

#ifdef  USE_FULL_ASSERT
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
