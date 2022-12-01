/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include  <stdio.h>
#include "modbus.h"
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
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
INA219_t ina219;
uint16_t vbus, vshunt, current;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void const * argument);

uint8_t receive_buffer[20];
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

#include "stm32f1xx_hal_flash.h"
uint16_t uartRecieCount=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//osMessagePut(ModBusInHandle,buffer[iii],0);
	uartRecieCount++;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT_485(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size){
	HAL_GPIO_WritePin(DE_485_GPIO_Port, DE_485_Pin, GPIO_PIN_SET);
	HAL_StatusTypeDef  ret = HAL_UART_Transmit_IT(huart, pData, Size);
	return ret;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_StatusTypeDef	st;
	UNUSED(st);

	if(huart == (UART_HandleTypeDef *)&huart1)
	{
		HAL_GPIO_WritePin(DE_485_GPIO_Port, DE_485_Pin, GPIO_PIN_RESET);
	}
}
HAL_StatusTypeDef FLASH_Program_485(uint32_t Address)
{
  uint32_t status = HAL_ERROR;

  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
  EraseInitStruct.NbPages     = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
  HAL_FLASHEx_Erase(&EraseInitStruct, &status);    //return HAL_OK if success
  HAL_FLASH_Lock();

  HAL_FLASH_Unlock();
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,FLASH_USER_START_ADDR,Address);
  HAL_FLASH_Lock();
  status=  HAL_FLASH_GetError();

  return status;
}
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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  // Read 485 Address from flash memory

  int16_t address_485=  * ((uint16_t *) FLASH_USER_START_ADDR);
  if( address_485 >= 0x01 && address_485 < 0x128){
	  //유효한 데이타이다.
  }
  else{

	  FLASH_Program_485(1);
  }
  address_485=  * ((uint16_t *) FLASH_USER_START_ADDR);



  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  ModBus_Init();
  ModBus_SetAddress(address_485);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIOC13_GPIO_Port, LED_GPIOC13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DE_485_GPIO_Port, DE_485_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10 , GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11 , GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GPIOC13_Pin */
  GPIO_InitStruct.Pin = LED_GPIOC13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIOC13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DE_485_Pin */
  GPIO_InitStruct.Pin = DE_485_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DE_485_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DE_485_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DE_485_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
//extern osMessageQId ModBusInHandle;
void ReceiveErrorFromUsart(){
	while(HAL_OK !=  HAL_UART_Receive(&huart1, receive_buffer,1, 33));
}
void ReadUsartAndMessagePut(){
	uint16_t iPos=0;
	iPos++;  //We Receive 1 Byte from Rx interrupt

	//int16_t address_485=  * ((uint16_t *) FLASH_USER_START_ADDR);
//	osMessagePut(ModBusInHandle,receive_buffer[iPos++],0);
//	if(HAL_OK !=  HAL_UART_Receive(&huart1, receive_buffer,1, 33)){
//		ReceiveErrorFromUsart();
//		return;
//	}
	// Check ID
//	if(receive_buffer[0] != address_485 ){
//		ReceiveErrorFromUsart();
//		return;
//	}
	// Now We Receive data until timeout reached
	while(HAL_OK == HAL_UART_Receive(&huart1, (uint8_t *)&receive_buffer[iPos],1, 5)){ // 5 miliscecon
			iPos++;
	}

	for(int iii=0;iii<iPos;iii++){
		osMessagePut(ModBusInHandle,receive_buffer[iii],0);
	}

}
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t buffer[256];
  /* Infinite loop */


		// 0x01 0x04 0x00 0x00 0x00 0x04 0xF1 0xC9
		//for test code

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
    // INA219_setPowerMode(&ina219, INA219_CONFIG_MODE_ADCOFF);

    while (!INA219_Init(&ina219, &hi2c2, INA219_ADDRESS)){
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      osDelay(50);

      /*  Test for 4 command
		uint8_t iii=0;
      	osDelay(1000);
		buffer[iii++]=0x01;
		buffer[iii++]=0x04;
		buffer[iii++]=0x00;
		buffer[iii++]=0x00;
		buffer[iii++]=0x00;
		buffer[iii++]=0x04;
		buffer[iii++]=0xF1;
		buffer[iii++]=0xC9;
		for(iii=0;iii<8;iii++){
			osMessagePut(ModBusInHandle,buffer[iii],0);
		}
		*/
      /*  Test for 5 command change 485 ID */
//		uint8_t iii=0;
//      	osDelay(1000);
//		buffer[iii++]=0x01;
//		buffer[iii++]=0x05;
//		buffer[iii++]=0x00;
//		buffer[iii++]=0x00;
//		buffer[iii++]=0x00;
//		buffer[iii++]=0x02;  // 2로 바꿔라.
//		buffer[iii++]=0x4c;
//		buffer[iii++]=0x0b;
//		for(iii=0;iii<8;iii++){
//			osMessagePut(ModBusInHandle,buffer[iii],0);
//		}
//	  HAL_UART_Transmit_IT_485(&huart1, buffer, iii );
    }

    HAL_UART_Receive_IT(&huart1, receive_buffer, 1);
    for (;;)
    {
    	if( uartRecieCount>0){ // We are received Data from USART
    		ReadUsartAndMessagePut();
    	}

      vbus = INA219_ReadBusVoltage(&ina219);
      vshunt = INA219_ReadShuntVolage(&ina219);
      current = INA219_ReadCurrent(&ina219);

	  int16_t address_485=0x00;
	  address_485 =  * ((uint16_t *) FLASH_USER_START_ADDR);
	  sprintf((char *)buffer,"485Address is  %d\r\n",address_485);

	  HAL_GPIO_WritePin(DE_485_GPIO_Port, DE_485_Pin, GPIO_PIN_SET);
	  //HAL_UART_Transmit(&huart1, buffer, sizeof(buffer), HAL_MAX_DELAY);
	  HAL_UART_Transmit_IT_485(&huart1, buffer, sizeof(buffer) );

      sprintf((char *)buffer,"vbus %d, vshunt %d, current %d\r\n",vbus,vshunt,current);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      osDelay(50);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	  HAL_GPIO_WritePin(DE_485_GPIO_Port, DE_485_Pin, GPIO_PIN_SET);
      //HAL_UART_Transmit(&huart1, buffer, sizeof(buffer), HAL_MAX_DELAY);
      HAL_UART_Transmit_IT(&huart1, buffer, sizeof(buffer) );
      osDelay(1000);
    }
  /* USER CODE END 5 */
}

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
