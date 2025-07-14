#include "main.h"
#include "DHT.h"
#include "i2c_lcd.h"
#include "stdio.h"
#include "string.h"

ADC_HandleTypeDef hadc;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;


I2C_LCD_HandleTypeDef lcd;

uint16_t ir_value;

DHT_sensor dht11_sensor;
DHT_data sensor_data;
char temp_data_on[16];
char temp_data_off[16];

int systemMode = 0; // auto mode
int fanMode = 0; // 0 : off, 1 : on
int ledMode = 0; // 0 : off, 1 : on

char rxBuffer[64];
volatile char rxChar;
uint8_t rxIndex = 0;

uint16_t readIR()
{
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(&hadc);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance == USART1)
	{
		if ((rxChar != '\r' && rxChar != '\n') && rxIndex < sizeof(rxBuffer) - 1){
		    rxBuffer[rxIndex++] = rxChar;
		}
		else{

			if (rxIndex > 0) {
			rxBuffer[rxIndex] = '\0';
			rxIndex = 0;

			char *newline = strchr(rxBuffer, '\r');
			if (newline) *newline = '\0';
			newline = strchr(rxBuffer, '\n');
			if (newline) *newline = '\0';


		if(strcmp(rxBuffer, "AUTO") == 0){
			systemMode = 0;

			HAL_UART_Transmit(&huart1, (uint8_t *)"Auto Mode\r\n", strlen("Auto Mode\r\n"), HAL_MAX_DELAY);
			}
		else if(strcmp(rxBuffer, "MANUEL") == 0){
			systemMode = 1; //manuel mode

			HAL_UART_Transmit(&huart1, (uint8_t *)"Manuel Mode\r\n", strlen("Manuel Mode\r\n"), HAL_MAX_DELAY);
			}
		else if(strcmp(rxBuffer, "FAN OFF") == 0){
			fanMode = 0; // fan off
			}
		else if(strcmp(rxBuffer, "FAN ON") == 0){
			fanMode = 1; // fan on
			}
		else if(strcmp(rxBuffer, "LED OFF") == 0){
			ledMode = 0; // led off
			}
		else if(strcmp(rxBuffer, "LED ON") == 0){
			ledMode = 1; // led on
			}
		else{
			HAL_UART_Transmit(&huart1, (uint8_t *)"Invalid Command\r\n", strlen("Invalid Command\r\n"), HAL_MAX_DELAY);
			}
		memset(rxBuffer, 0, sizeof(rxBuffer));
		}
	    }
	}
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxChar, 1);
}

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxChar, 1);

  dht11_sensor.DHT_Port = GPIOA;
  dht11_sensor.DHT_Pin = GPIO_PIN_1;
  dht11_sensor.type = DHT11;
  dht11_sensor.pullUp = GPIO_NOPULL;

  lcd.hi2c = &hi2c1;
  lcd.address = 0x4E;
  lcd_init(&lcd);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // IN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // IN2

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  while (1)
  {
	 ir_value = readIR();
	 sensor_data = DHT_getData(&dht11_sensor);
			
	 sprintf(temp_data_on, "ON - TEMP: %d C", (int)sensor_data.temp);
	 sprintf(temp_data_off, "OFF - TEMP: %d C", (int)sensor_data.temp);
			
			
		  if(systemMode == 0){
			if(sensor_data.temp >= 27){
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
			  lcd_clear(&lcd);
			  lcd_puts(&lcd, temp_data_on);
		        }else{
 			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
			  lcd_clear(&lcd);
			  lcd_puts(&lcd, temp_data_off);
		 	}
			
			 if(ir_value <= 1500){
			   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
			   lcd_gotoxy(&lcd, 0, 1);
			   lcd_puts(&lcd, "ON - MOTION: YES");
			  }else{
			    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
			    lcd_gotoxy(&lcd, 0, 1);
			    lcd_puts(&lcd, "OFF - MOTION: NO");
				}
		  	}

	  else if(systemMode == 1){

		  if(fanMode == 1 && ledMode == 1){

			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 300);

			  lcd_gotoxy(&lcd, 0, 0);
			  lcd_puts(&lcd, "FAN ON          ");
			  lcd_gotoxy(&lcd, 0, 1);
			  lcd_puts(&lcd, "LED ON          ");

		  }else if(fanMode == 0 && ledMode == 0){

			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

			  lcd_gotoxy(&lcd, 0, 0);
			  lcd_puts(&lcd, "FAN OFF         ");
			  lcd_gotoxy(&lcd, 0, 1);
			  lcd_puts(&lcd, "LED OFF         ");

		  }else if(fanMode == 1 && ledMode == 0){

			 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 300);

			 lcd_gotoxy(&lcd, 0, 0);
			 lcd_puts(&lcd, "FAN ON           ");
			 lcd_gotoxy(&lcd, 0, 1);
			 lcd_puts(&lcd, "LED OFF          ");
		  }else if(fanMode == 0 && ledMode == 1){

			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

			  lcd_gotoxy(&lcd, 0, 0);
			  lcd_puts(&lcd, "FAN OFF          ");
			  lcd_gotoxy(&lcd, 0, 1);
			  lcd_puts(&lcd, "LED ON           ");
		  }
	     }
	 }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{

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
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
void Error_Handler(void)
{
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
