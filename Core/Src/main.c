/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "CLCD_I2C.h"
#include "dht11.h"
#include<stdio.h>


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

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* Definitions for sendCloud */
osThreadId_t sendCloudHandle;
const osThreadAttr_t sendCloud_attributes = {
  .name = "sendCloud",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ReadDataTaskHan */
osThreadId_t ReadDataTaskHanHandle;
const osThreadAttr_t ReadDataTaskHan_attributes = {
  .name = "ReadDataTaskHan",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for DisplayTaskHand */
osThreadId_t DisplayTaskHandHandle;
const osThreadAttr_t DisplayTaskHand_attributes = {
  .name = "DisplayTaskHand",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for HandleInterupt */
osThreadId_t HandleInteruptHandle;
const osThreadAttr_t HandleInterupt_attributes = {
  .name = "HandleInterupt",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for pumpControlHand */
osThreadId_t pumpControlHandHandle;
const osThreadAttr_t pumpControlHand_attributes = {
  .name = "pumpControlHand",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for TempHumid */
osSemaphoreId_t TempHumidHandle;
const osSemaphoreAttr_t TempHumid_attributes = {
  .name = "TempHumid"
};
/* Definitions for DataSem */
osSemaphoreId_t DataSemHandle;
const osSemaphoreAttr_t DataSem_attributes = {
  .name = "DataSem"
};
/* Definitions for ComSem */
osSemaphoreId_t ComSemHandle;
const osSemaphoreAttr_t ComSem_attributes = {
  .name = "ComSem"
};
/* Definitions for Interrupt */
osEventFlagsId_t InterruptHandle;
const osEventFlagsAttr_t Interrupt_attributes = {
  .name = "Interrupt"
};
/* Definitions for pumpEvent */
osEventFlagsId_t pumpEventHandle;
const osEventFlagsAttr_t pumpEvent_attributes = {
  .name = "pumpEvent"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
void sendCloudHandler(void *argument);
void ReadDataTask(void *argument);
void DisplayTask(void *argument);
void HandleISP(void *argument);
void pumpControl(void *argument);

/* USER CODE BEGIN PFP */
uint8_t readbutton1();
uint8_t readbutton2();
uint8_t readbutton3();
void sendTerminal(char* mess,uint8_t length);

CLCD_I2C_Name LCD1;

//HT_Name DHT1;
DHT11_Sensor dht;


float temp  = 0.00;
float humid = 0.00;
uint32_t T1 = 1000;
uint32_t T2 = 500;
uint32_t T3 = 200;

#define 	PERIOD_1 			1000
#define 	PERIOD_2 			5000
#define 	PERIOD_3 			10000

#define 	timeInterrupt  		3000

#define 	HUMIDITY_SET		80.0
#define		HUMIDITY_MIN		15.0

#define 	waterWaitingPeriod	10000
//uint32_t pressBTtime = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  if(GPIO_Pin == C1_Pin){
	  for(int i=0;i<1000000;i++);
	  __HAL_GPIO_EXTI_GET_IT(C1_Pin);
	  HAL_NVIC_GetPendingIRQ(EXTI15_10_IRQn);
	  osEventFlagsSet(InterruptHandle, 1);
  }
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
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
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  DHT11_Init(&dht,DHT11_GPIO_Port, DHT11_Pin, &htim4);
  DHT11_Init(&dht, DHT11_GPIO_Port,DHT11_Pin,&htim4);
  CLCD_I2C_Init(&LCD1, &hi2c2, 0x4e,20,4);



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of TempHumid */
  TempHumidHandle = osSemaphoreNew(1, 0, &TempHumid_attributes);

  /* creation of DataSem */
  DataSemHandle = osSemaphoreNew(2, 2, &DataSem_attributes);

  /* creation of ComSem */
  ComSemHandle = osSemaphoreNew(2, 2, &ComSem_attributes);

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
  /* creation of sendCloud */
  sendCloudHandle = osThreadNew(sendCloudHandler, NULL, &sendCloud_attributes);

  /* creation of ReadDataTaskHan */
  ReadDataTaskHanHandle = osThreadNew(ReadDataTask, NULL, &ReadDataTaskHan_attributes);

  /* creation of DisplayTaskHand */
  DisplayTaskHandHandle = osThreadNew(DisplayTask, NULL, &DisplayTaskHand_attributes);

  /* creation of HandleInterupt */
  HandleInteruptHandle = osThreadNew(HandleISP, NULL, &HandleInterupt_attributes);

  /* creation of pumpControlHand */
  pumpControlHandHandle = osThreadNew(pumpControl, NULL, &pumpControlHand_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Interrupt */
  InterruptHandle = osEventFlagsNew(&Interrupt_attributes);

  /* creation of pumpEvent */
  pumpEventHandle = osEventFlagsNew(&pumpEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  osSemaphoreRelease(TempHumidHandle);
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Pump_Pin|DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R2_Pin|R3_Pin|R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Pump_Pin DHT11_Pin */
  GPIO_InitStruct.Pin = Pump_Pin|DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : C1_Pin */
  GPIO_InitStruct.Pin = C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(C1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C2_Pin C3_Pin C4_Pin */
  GPIO_InitStruct.Pin = C2_Pin|C3_Pin|C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t readbutton1()
{
  return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
}
uint8_t readbutton2()
{
  return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);
}
uint8_t readbutton3()
{
  return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8);
}

void sendTerminal(char* mess,uint8_t length){
	HAL_UART_Transmit(&huart2, (uint8_t*)mess, length, 1000);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_sendCloudHandler */
/**
  * @brief  Function implementing the sendCloud thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_sendCloudHandler */
void sendCloudHandler(void *argument)
{
  /* USER CODE BEGIN 5 */
	char Temp[10];
  /* Infinite loop */
  for(;;)
  {
//	  sendTerminal("Task sendCloud acquire semaphore\n",strlen("Task sendCloud acquire semaphore\n"));
	  osSemaphoreAcquire(TempHumidHandle, osWaitForever);
//	  sendTerminal("Task sendCloud take semaphore\n",strlen("Task sendCloud take semaphore\n"));
	  sprintf(Temp, "%.2f,%.2f\n",temp,humid);
	  HAL_UART_Transmit(&huart2, (uint8_t*)Temp, strlen(Temp), 1000);
//	  sendTerminal("Task sendCloud complete\n",strlen("Task sendCloud complete\n"));
	  osSemaphoreRelease(TempHumidHandle);
//	  sendTerminal("Task sendCloud release semaphore\n",strlen("Task sendCloud release semaphore\n"));
	  osDelay(T1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ReadDataTask */
/**
* @brief Function implementing the ReadDataTaskHan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadDataTask */
void ReadDataTask(void *argument)
{
  /* USER CODE BEGIN ReadDataTask */
	DHT11_Status dhtStatus;
  /* Infinite loop */
  for(;;)
  {
//	  sendTerminal("Task readData acquire semaphore\n",strlen("Task readData acquire semaphore\n"));
	  osSemaphoreAcquire(TempHumidHandle, osWaitForever);
//	  sendTerminal("Task readData take semaphore\n",strlen("Task readData take semaphore\n"));
	  dhtStatus = DHT11_GetData(&dht);
	  temp = dht.Temp;
	  humid = dht.Humi;
	  if(humid < HUMIDITY_MIN){
		  osEventFlagsSet(pumpEventHandle, 1);
	  }
//	  sendTerminal("Task readData complete\n",strlen("Task readData complete\n"));
	  osSemaphoreRelease(TempHumidHandle);
//	  sendTerminal("Task readData release semaphore\n",strlen("Task readData release semaphore\n"));
	  osDelay(T1);
  }
  /* USER CODE END ReadDataTask */
}

/* USER CODE BEGIN Header_DisplayTask */
/**
* @brief Function implementing the DisplayTaskHand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DisplayTask */
void DisplayTask(void *argument)
{
  /* USER CODE BEGIN DisplayTask */

	char temperature[16], humidity[16];
  /* Infinite loop */
  for(;;)
  {
//	  sendTerminal("Task display acquire semaphore\n",strlen("Task display acquire semaphore\n"));
	  osSemaphoreAcquire(TempHumidHandle, osWaitForever);
//	  sendTerminal("Task display take semaphore\n",strlen("Task display take semaphore\n"));
	  sprintf(temperature, "Temp: %.2f",temp);
	  sprintf(humidity, "Humid: %.2f", humid);
	  CLCD_I2C_Clear(&LCD1);
	  CLCD_I2C_SetCursor(&LCD1,0,0);
	  CLCD_I2C_WriteString(&LCD1,temperature);
	  CLCD_I2C_SetCursor(&LCD1,0,1);
	  CLCD_I2C_WriteString(&LCD1,humidity);
//	  sendTerminal("Task display complete\n",strlen("Task display complete\n"));
	  osSemaphoreRelease(TempHumidHandle);
//	  sendTerminal("Task display release semaphore\n",strlen("Task display release semaphore\n"));
	  osDelay(T1);
  }
  /* USER CODE END DisplayTask */
}

/* USER CODE BEGIN Header_HandleISP */
/**
* @brief Function implementing the HandleInterupt thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HandleISP */
void HandleISP(void *argument)
{
  /* USER CODE BEGIN HandleISP */

  /* Infinite loop */
	for(;;)
	{
		osEventFlagsWait(InterruptHandle,1,osFlagsWaitAll,osWaitForever);
//		sendTerminal("Start Interrupt\n",strlen("Start Interrupt\n"));
		osThreadSuspend(ReadDataTaskHanHandle);
		CLCD_I2C_Clear(&LCD1);
		CLCD_I2C_SetCursor(&LCD1,0,0);
		CLCD_I2C_WriteString(&LCD1,"Select Mode");
		CLCD_I2C_SetCursor(&LCD1,0,1);
		CLCD_I2C_WriteString(&LCD1,"Sampling cycle");
		uint32_t tickstart = osKernelGetTickCount();
		uint8_t haveValue = 0;

		while(((osKernelGetTickCount() - tickstart) < timeInterrupt)&&haveValue==0)
		{
	        if(readbutton1())
	        {
	          T1 = PERIOD_1;
//	          sendTerminal("Change the period of read data to 5s\n",strlen("Change the period of read data to 5s\n"));
	          CLCD_I2C_Clear(&LCD1);
	          CLCD_I2C_SetCursor(&LCD1,0,0);
	          CLCD_I2C_WriteString(&LCD1,"Cycle  1s");
	          haveValue = 1;
	        }
	        else if(readbutton2())
	        {
	          T1 = PERIOD_2;
//	          sendTerminal("Change the period of read data to 5s\n",strlen("Change the period of read data to 5s\n"));
	          CLCD_I2C_Clear(&LCD1);
	          CLCD_I2C_SetCursor(&LCD1,0,0);
	          CLCD_I2C_WriteString(&LCD1,"Cycle  5s");
	          haveValue = 1;
	        }
	        else if(readbutton3())
	        {
	          T1 = PERIOD_3;
//	          sendTerminal("Change the period of read data to 10s\n",strlen("Change the period of read data to 10s\n"));
	          CLCD_I2C_Clear(&LCD1);
	          CLCD_I2C_SetCursor(&LCD1,0,0);
	          CLCD_I2C_WriteString(&LCD1,"Cycle 10s");
	          haveValue = 1;
	        }
		}
		osThreadResume(ReadDataTaskHanHandle);
	    osDelay(100);

  }
  /* USER CODE END HandleISP */
}

/* USER CODE BEGIN Header_pumpControl */
/**
* @brief Function implementing the pumpControlHand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pumpControl */
void pumpControl(void *argument)
{
  /* USER CODE BEGIN pumpControl */
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(pumpEventHandle,1,osFlagsWaitAll,waterWaitingPeriod);
//	  sendTerminal("Start water\n",strlen("Start water\n"));
	  HAL_GPIO_WritePin(Pump_GPIO_Port, Pump_Pin, 1);
	  osDelay(5000);
//	  sendTerminal("Stop water\n",strlen("Stop water\n"));
	  HAL_GPIO_WritePin(Pump_GPIO_Port, Pump_Pin, 0);
	  osEventFlagsClear(pumpControlHandHandle, 1);

  }
  /* USER CODE END pumpControl */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
