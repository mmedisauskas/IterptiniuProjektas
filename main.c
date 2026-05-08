/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "mpu6050.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define NumSamples 25
#define SendSamples 5
#define Alpha 0.05 //Accel koeficientas
#define Cal_samples 2000
#define dt 0.04 //time between each sample, in seconds
#define deadzone 0.2 //ekrane vaizduojamu skaiciu riba, iki kurios rodoma 0

//Flags
int Timer_Flag = 0;
int Cal_request = 0;

//Counters
int Sample = 0;
int counter = 0;

//Buffers
char buffer[20];
float Gyro_Gx_Buffer[NumSamples], Gyro_Gy_Buffer[NumSamples], Gyro_Gz_Buffer[NumSamples];
uint8_t TxBuffer[35];
uint8_t RxBuffer[20];

char *msg;

//Raw data
float Gx = 0, Gy = 0, Gz = 0; //raw gyro data
float Ax = 0, Ay = 0, Az = 0; //raw accel data

//Angles
float angle_x = 0, angle_y = 0, angle_z = 0; //angle of each axis
float acc_angle_x, acc_angle_y, acc_angle_z;
float avg02_x = 0, avg02_y = 0, avg02_z = 0;
float avg_x = 0, avg_y = 0, avg_z = 0;

//Calibration
float offset_x, offset_y, offset_z;
float bias_x = 0, bias_y = 0, bias_z = 0; //calibrated offset

//For UART
uint8_t rxByte;
uint8_t idx = 0;


void HandleError(){
	uint32_t uart_err;
	uart_err=HAL_UART_GetError(&huart2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2){
		if (rxByte != '\n' && rxByte != '\r' && idx < sizeof(RxBuffer) - 1){
			RxBuffer[idx++] = rxByte;
    }
  if(idx == 3){
      idx = 0;
      if (strcmp((char*)RxBuffer, "CAL") == 0){
				Cal_request = 1; 
      }
    }
	HAL_UART_Receive_IT(&huart2, &rxByte, 1);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2)
	{
		Timer_Flag = 1;
	}
}

void Calibrate(){
	msg = "Calibrating...\r\n";
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, strlen(msg));
	SSD1306_Clear();
	SSD1306_GotoXY(0,32);
	SSD1306_Puts("Calibrating...", &Font_7x10, 1);
	SSD1306_UpdateScreen();
	//Nunuliname visas reiksmes
	Sample = 0;
	angle_x = 0, angle_y = 0, angle_z = 0;
	bias_x = 0, bias_y = 0, bias_z = 0;
	offset_x = 0, offset_y = 0;
	
	for(int i = 0; i < Cal_samples; i++) {
    MPU6050_Read_Accel(&Ax, &Ay, &Az);
		MPU6050_Read_Gyro(&Gx, &Gy, &Gz);
		
		offset_x += atan2(Ay,Az)*57.2958; 
		offset_y += atan2(-Ax, sqrt(Ay*Ay + Az*Az))*57.2958;
    bias_x += Gx;
    bias_y += Gy;
    bias_z += Gz;
		HAL_Delay(5);
	}
	
	offset_x /= Cal_samples;
	offset_y /= Cal_samples;
	
	bias_x /= Cal_samples;
	bias_y /= Cal_samples;
	bias_z /= Cal_samples;
	msg = "Done!\r\n";
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, strlen(msg));
}

void UpdateScreen(){
	//Fonts: 7x10, 11x18, 16x26
	//Screen size 128x64
	snprintf(buffer, sizeof(buffer), "%d", counter);
	SSD1306_GotoXY(0,0);
	SSD1306_Puts(buffer, &Font_7x10, 1);
	//X below
	if(fabs(avg_x) < deadzone) avg_x=0;
	snprintf(buffer, sizeof(buffer), "X: %.1f   ", avg_x);
	SSD1306_GotoXY(0,9);
	SSD1306_Puts(buffer, &Font_11x18, 1);
	//Y below
	if(fabs(avg_y) < deadzone) avg_y=0;
	snprintf(buffer, sizeof(buffer), "Y: %.1f   ", avg_y);
	SSD1306_GotoXY(0,27);
	SSD1306_Puts(buffer, &Font_11x18, 1);
	//Z below
	if(fabs(avg_z) < deadzone) avg_z=0;
	snprintf(buffer, sizeof(buffer), "Z: %.1f   ", avg_z);
	SSD1306_GotoXY(0,45);
	SSD1306_Puts(buffer, &Font_11x18, 1);
	SSD1306_UpdateScreen();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	SSD1306_Init();
	mpu6050_init();
	
	msg = "Program started\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
	//Fonts: 7x10, 11x18, 16x26
	
	SSD1306_GotoXY(0,0);
	SSD1306_Puts("HELLO", &Font_11x18, 1);
	SSD1306_GotoXY(10,30);
	SSD1306_Puts(" WORLD :)", &Font_11x18, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(2000);
	Calibrate();
	SSD1306_Clear();
	UpdateScreen();
	HAL_UART_Receive_IT(&huart2, &rxByte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if(Timer_Flag == 1){
			MPU6050_Read_Gyro(&Gx, &Gy, &Gz);
			MPU6050_Read_Accel(&Ax, &Ay, &Az);
			
			if(fabs(Az)<0.05)
				Az = 0.05;
			//Angle from accel
			acc_angle_x = atan2(Ay,Az) * 57.2958; // 180/PI
			acc_angle_y = atan2(-Ax, sqrt(Ay*Ay + Az*Az))* 57.2958;
			//Angle from gyro
			angle_x = (1-Alpha)*(angle_x+(Gx-bias_x)*dt)+Alpha*(acc_angle_x-offset_x);
			angle_y = (1-Alpha)*(angle_y+(Gy-bias_y)*dt)+Alpha*(acc_angle_y-offset_y);
			angle_z = (angle_z+(Gz-bias_z)*dt);
			//Placing into buffer
			Gyro_Gx_Buffer[Sample]=angle_x;
			Gyro_Gy_Buffer[Sample]=-angle_y;
			Gyro_Gz_Buffer[Sample]=-angle_z;
			
			Sample++;
			
			if(Sample>= SendSamples && Sample%SendSamples == 0){
				avg02_x = 0, avg02_y = 0, avg02_z = 0;
				for(int i = Sample-SendSamples; i<Sample; i++){
					avg02_x += Gyro_Gx_Buffer[i];
					avg02_y += Gyro_Gy_Buffer[i];
					avg02_z += Gyro_Gz_Buffer[i];
				}
				avg02_x /= SendSamples, avg02_y /= SendSamples, avg02_z /= SendSamples;
				
				
				sprintf((char *)TxBuffer,"[%d] X: %.2f, Y: %.2f, Z: %.2f\r\n",Sample, avg02_x, avg02_y, avg02_z);
				HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
			}
			
			Timer_Flag = 0;
		}
		if(Sample >= NumSamples){
			avg_x = 0, avg_y = 0, avg_z = 0;
			
			for(int i=0; i<NumSamples; i++){
				avg_x += Gyro_Gx_Buffer[i];
				avg_y += Gyro_Gy_Buffer[i];
				avg_z += Gyro_Gz_Buffer[i];
			}
			avg_x /= NumSamples, avg_y /= NumSamples, avg_z /= NumSamples;
			
			counter++;
			UpdateScreen();
			Sample = 0;
		}
		if(Cal_request == 1){
			Calibrate();
			Cal_request = 0;
		}
  }
  /* USER CODE END 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
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
  hi2c1.Init.Timing = 0x00300617;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 320-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
