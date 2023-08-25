/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "LCD_driver.h"
#include "st7789.h"
#include "fonts.h"
#include "LCD_lib.h"
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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8] = {0};
uint8_t RxData[8];

uint32_t TxMailbox;

CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader2;

uint8_t TxData2[8] = {0};
uint8_t RxData2[8];

uint32_t TxMailbox2;

int dataflag = 0;
int dataflag2 = 0;
int timer0_flag = 0;
int timer0_counter = 0;
int timer1_flag = 0;
int timer1_counter = 0;
int data2counter = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void setTimer0(int time_ms);
void setTimer1(int time_ms);
void timerRun();
int calc_SAE_J1850(uint8_t data[], int crc_len);

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
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  if(HAL_CAN_Start(&hcan1)==HAL_ERROR){
	  while(1){
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		  HAL_Delay(500);
	  }
  }
  if(HAL_CAN_Start(&hcan2)==HAL_ERROR){
  	  while(1){
  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  		  HAL_Delay(500);
  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
  		  HAL_Delay(500);
  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
  		  HAL_Delay(500);
  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
  		  HAL_Delay(500);
  	  }
    }
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_TIM_Base_Start_IT(&htim2);

  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x012;

  TxHeader2.DLC = 8;
  TxHeader2.IDE = CAN_ID_STD;
  TxHeader2.RTR = CAN_RTR_DATA;
  TxHeader2.StdId = 0x0A2;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setTimer0(50);
  setTimer1(20);


  lcd_init();
  ST7789_Init();
  char buffer[50];

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/* Node 1 */
	if(dataflag){
		dataflag = 0;

		TxData[0] = RxData[0];
		TxData[1] = RxData[1];
		TxData[2] = TxData[0] + TxData[1];
		TxData[7] = calc_SAE_J1850(TxData, 7);

		sprintf(buffer, "Node 1: 0x0A2 value [0] = %#04x",RxData[0]);
		ST7789_WriteString(10, 10, buffer, Font_7x10, WHITE, BLACK);
		sprintf(buffer, "Node 1: 0x0A2 value [1] = %#04x",RxData[1]);
		ST7789_WriteString(10, 25, buffer, Font_7x10, WHITE, BLACK);
		sprintf(buffer, "Node 1: 0x0A2 value [6] = %#04x",RxData[6]);
		ST7789_WriteString(10, 40, buffer, Font_7x10, WHITE, BLACK);
	}

	if(timer0_flag){
		setTimer0(50);
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0){
			if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)!=HAL_OK){
				Error_Handler();
			}
		}
	}

	/* Node 2 */
	if(dataflag2){
		dataflag2 = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		// TO DO: DISPLAY ON LCD
		sprintf(buffer, "Node 2: CRC value = %#04x, VALID!",RxData2[7]);
		ST7789_WriteString(10, 60, buffer, Font_7x10, WHITE, BLACK);
	}


	if(timer1_flag){
		setTimer1(20);
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)>0){
			TxData2[0] = 0x01;
			TxData2[1] = 0x04;
			TxData2[6] = data2counter++;
			if(HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2)!=HAL_OK){
				Error_Handler();
			}
		}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
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
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig1;

  canfilterconfig1.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig1.FilterBank = 0;  // which filter bank to use from the assigned ones
  canfilterconfig1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig1.FilterIdHigh = 0x0A2<<5;
  canfilterconfig1.FilterIdLow = 0;
  canfilterconfig1.FilterMaskIdHigh = 0x0A2;
  canfilterconfig1.FilterMaskIdLow = 0;
  canfilterconfig1.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig1.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig1.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig1);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 8;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef canfilterconfig2;

    canfilterconfig2.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig2.FilterBank = 14;  // which filter bank to use from the assigned ones
    canfilterconfig2.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig2.FilterIdHigh = 0x012<<5;
    canfilterconfig2.FilterIdLow = 0;
    canfilterconfig2.FilterMaskIdHigh = 0x0A12;
    canfilterconfig2.FilterMaskIdLow = 0;
    canfilterconfig2.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig2.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig2.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

    HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig2);

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : JOYSTICK_CTR_Pin JOYSTICK_A_Pin JOYSTICK_B_Pin JOYSTICK_C_Pin
                           JOYSTICK_D_Pin */
  GPIO_InitStruct.Pin = JOYSTICK_CTR_Pin|JOYSTICK_A_Pin|JOYSTICK_B_Pin|JOYSTICK_C_Pin
                          |JOYSTICK_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int calc_SAE_J1850(uint8_t data[], int length){
	uint8_t polynomial = 0x1D; // Generator polynomial for CRC-8 SAE J1850
	uint8_t remainder = 0xFF;

	for (int i = 0; i < length; i++) {
		remainder ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			if (remainder & 0x80) {
				remainder = (remainder << 1) ^ polynomial;
			} else {
				remainder <<= 1;
			}
		}
	}

	return remainder ^ 0xFF; // Invert the bits
}

void setTimer0(int time_ms){
	timer0_counter = time_ms/10;
	timer0_flag = 0;
}

void setTimer1(int time_ms){
	timer1_counter = time_ms/10;
	timer1_flag = 0;
}

void timerRun(){
	if(timer0_counter > 0){
		timer0_counter--;
	}else{
		timer0_flag = 1;
	}
	if(timer1_counter > 0){
		timer1_counter--;
	}else{
		timer1_flag = 1;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if(hcan->Instance == CAN1){
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData)==HAL_OK){
			dataflag = 1;
		}else{
			Error_Handler();
		}
	}
	if(hcan->Instance == CAN2){
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader2, RxData2)==HAL_OK){
			dataflag2 = 1;
		}else{
			Error_Handler();
		}
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	timerRun();
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
