#include "main.h"
#include<stdio.h>
#include"string.h"

#define MPU6050_ADDR  (0x68 << 1) // sensor address
#define GYRO_CNFG_REG 0x1B // gyro address
#define ACC_CNFG_REG  0x1C // acc address
#define REG_USR_CTRL  0x6B // exit sleep mode
#define ACC_REG_DATA  0x3B // initial address
#define GYRO_REG_DATA 0x43 // initial addres
#define SAMPLES		  200
#define MAX_LEN 	  50 // array size

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

uint8_t data;
uint8_t acc_buffer[6], gyro_buffer[6]; // read raw data
int16_t x_acc, y_acc, z_acc; // raw value
int16_t x_gyro, y_gyro, z_gyro; // raw value
float x_acc_g, y_acc_g, z_acc_g; // real value
float x_gyro_dps, y_gyro_dps, z_gyro_dps; // real value
int32_t x_acc_offset = 0, y_acc_offset = 0, z_acc_offset = 0;
int32_t x_gyro_offset = 0, y_gyro_offset = 0, z_gyro_offset = 0;
float gyro_array_x[5], gyro_array_y[5],gyro_array_z[5];
float acc_array_x[5], acc_array_y[5], acc_array_z[5];
volatile char rxChar;
uint8_t rxIndex = 0;
uint8_t rxBuffer[MAX_LEN];
uint8_t accBuffer[MAX_LEN], gyroBuffer[MAX_LEN];
uint8_t data_index = 0;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart -> Instance == USART1){

		if(rxChar != '\r' && rxChar != '\n'){ // to understand the end of the array
			rxBuffer[rxIndex++] = rxChar;

			if(rxIndex > MAX_LEN) // to prevent buffer overflow
				rxIndex = 0;
		}else{
			 rxBuffer[rxIndex] = '\0'; // null-terminator (to make a string to the message in the array)

		if(strcmp(rxBuffer, "help") == 0){
			HAL_UART_Transmit(&huart1, (uint8_t *)">gyro : gyro data\r\n",
							strlen(">gyro : gyro data\r\n"), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t*)">acc : acc data\r\n",
							strlen(">acc : acc data\r\n"), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t*)">last_acc : acc data (last 5 datas)\r\n",
							strlen(">last_acc : acc data (last 5 datas)\r\n"), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t*)">last_gyro : gyro data (last 5 datas)\r\n",
							strlen(">last_gyro : gyro data (last 5 datas)\r\n"), HAL_MAX_DELAY);
		}
		if(strcmp(rxBuffer, "gyro") == 0){
			gyroRead();
			HAL_UART_Transmit(&huart1, (uint8_t*)gyroBuffer, strlen(gyroBuffer), HAL_MAX_DELAY);
		}
		if(strcmp(rxBuffer, "acc") == 0){
			accRead();
			HAL_UART_Transmit(&huart1, (uint8_t*)accBuffer, strlen(accBuffer), HAL_MAX_DELAY);
		}
		if(strcmp(rxBuffer, "last_acc") == 0){
			lastAcc();
		}
		if(strcmp(rxBuffer, "last_gyro") == 0){
			lastGyro();
		}
		rxIndex = 0; // write again to array head
		memset(rxBuffer, 0, MAX_LEN); // delete all of the messages
	  }
		  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxChar, 1);
	}
}
void accRead(){
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACC_REG_DATA, 1, acc_buffer, 6, HAL_MAX_DELAY);

	x_acc = ((int16_t)acc_buffer[0] << 8) | acc_buffer[1]; // sum bytes
	y_acc = ((int16_t)acc_buffer[2] << 8) | acc_buffer[3];
	z_acc = ((int16_t)acc_buffer[4] << 8) | acc_buffer[5];


	// conversion the real value
	x_acc_g = (x_acc - x_acc_offset) / 8192.0f;
	y_acc_g = (y_acc - y_acc_offset) / 8192.0f;
	z_acc_g = (z_acc - z_acc_offset) / 8192.0f;

	acc_array_x[data_index] = x_acc_g;
	acc_array_y[data_index] = y_acc_g;
	acc_array_z[data_index] = z_acc_g;

	data_index++;
	if(data_index >= 5) data_index = 0;

	sprintf(accBuffer, "Ax: %.2f Ay: %.2f Az: %.2f\r\n", x_acc_g, y_acc_g, z_acc_g);
}

void gyroRead(){
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_REG_DATA, 1, gyro_buffer, 6, HAL_MAX_DELAY);

	x_gyro = ((int16_t)gyro_buffer[0] << 8) | gyro_buffer[1]; // sum bytes
   	y_gyro = ((int16_t)gyro_buffer[2] << 8) | gyro_buffer[3];
	z_gyro = ((int16_t)gyro_buffer[4] << 8) | gyro_buffer[5];


	// conversion the real value
	x_gyro_dps = (x_gyro - x_gyro_offset) / 65.5f;
	y_gyro_dps = (y_gyro - y_gyro_offset) / 65.5f;
	z_gyro_dps = (z_gyro - z_gyro_offset) / 65.5f;

	gyro_array_x[data_index] = x_gyro_dps;
	gyro_array_y[data_index] = y_gyro_dps;
	gyro_array_z[data_index] = z_gyro_dps;

	data_index++;
	if(data_index >= 5) data_index = 0;

	sprintf(gyroBuffer,"Gx: %.2f Gy: %.2f Gz: %.2f\r\n",x_gyro_dps, y_gyro_dps, z_gyro_dps);
}

void calibrateIMU(){
	 int16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;

	for(int i = 0; i < SAMPLES; i++){
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACC_REG_DATA, 1, acc_buffer, 6, HAL_MAX_DELAY);

		acc_x = ((int16_t)acc_buffer[0] << 8) | acc_buffer[1]; // sum bytes
		acc_y = ((int16_t)acc_buffer[2] << 8) | acc_buffer[3];
		acc_z = ((int16_t)acc_buffer[4] << 8) | acc_buffer[5];

		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_REG_DATA, 1, gyro_buffer, 6, HAL_MAX_DELAY);

		gyro_x = ((int16_t)gyro_buffer[0] << 8) | gyro_buffer[1]; // sum bytes
		gyro_y = ((int16_t)gyro_buffer[2] << 8) | gyro_buffer[3];
		gyro_z = ((int16_t)gyro_buffer[4] << 8) | gyro_buffer[5];


		x_acc_offset += acc_x;
		y_acc_offset += acc_y;
		z_acc_offset += acc_z;

		x_gyro_offset += gyro_x;
		y_gyro_offset += gyro_y;
		z_gyro_offset += gyro_z;

		HAL_Delay(5); // reading range
	}

	    x_acc_offset /= SAMPLES;
	    y_acc_offset /= SAMPLES;
	    z_acc_offset /= SAMPLES;


	    x_gyro_offset /= SAMPLES;
	    y_gyro_offset /= SAMPLES;
	    z_gyro_offset /= SAMPLES;

}

void lastAcc(){
	for(int i = 0; i < 4; i++){
		sprintf(accBuffer, "Ax: %.2f Ay: %.2f Az: %.2f\r\n",acc_array_x[i], acc_array_y[i], acc_array_z[i]);
		HAL_UART_Transmit(&huart1, (uint8_t*)accBuffer, strlen(accBuffer), HAL_MAX_DELAY);
	}
}

void lastGyro(){
	for(int i = 0; i < 4; i++){
		sprintf(gyroBuffer, "Gx: %.2f Gy: %.2f Gz: %.2f\r\n",gyro_array_x[i], gyro_array_y[i], gyro_array_z[i]);
		HAL_UART_Transmit(&huart1, (uint8_t*)gyroBuffer, strlen(gyroBuffer), HAL_MAX_DELAY);
	}
}


int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

data = 0x00; // to exit sleep mode and activate the mpu6050 sensor
HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_USR_CTRL, 1, &data, 1, HAL_MAX_DELAY);

data = 0x08; // sensitivity -> +/- 4g and +/- 500 deg/s
HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);
HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACC_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);

calibrateIMU();

HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxChar, 1);
HAL_UART_Transmit(&huart1, (uint8_t*)"CLI Console 1.1.1\r\n", strlen("CLI Console 1.1.1\r\n"), HAL_MAX_DELAY);

  while (1)
  {
	  accRead();
	  gyroRead();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
