/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
   int32_t i;
   q31_t q;
} Sample;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_RATE 			44  	// kHz
#define WINDOW_DURATION		50		// ms
#define SAMPLE_WINDOW 		(SAMPLE_RATE * WINDOW_DURATION)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t rxBuffer[8];
float32_t lSampleBuf[SAMPLE_WINDOW];
float32_t rSampleBuf[SAMPLE_WINDOW];
uint16_t sampleCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void float2buf(uint8_t *buf, float num, char *chn);
static bool storeSample();
static void setLeds(uint32_t rmsValue);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_I2S1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2S_Receive_DMA(&hi2s1, rxBuffer, 4);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|p3Pin_Pin|p2Pin_Pin|n2Pin_Pin|n1Pin_Pin|n10Pin_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, n20Pin_Pin|pn0Pin_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, p1Pin_Pin|n3Pin_Pin|n7Pin_Pin|n5Pin_Pin, GPIO_PIN_SET);

  HAL_Delay(1000);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|p3Pin_Pin|p2Pin_Pin|n2Pin_Pin|n1Pin_Pin|n10Pin_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, n20Pin_Pin|pn0Pin_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, p1Pin_Pin|n3Pin_Pin|n7Pin_Pin|n5Pin_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if (sampleCounter == SAMPLE_WINDOW) {
  		// Calculate RMS value for each channel
			float32_t lRmsF;
			float32_t rRmsF;
			arm_rms_f32(lSampleBuf, SAMPLE_WINDOW, &lRmsF);
			arm_rms_f32(rSampleBuf, SAMPLE_WINDOW, &rRmsF);

			sampleCounter = 0;	// Release sample buffer lock

			// 0.33244130

			uint32_t lRms = (uint32_t) (lRmsF * 1E8);
			uint32_t rRms = (uint32_t) (lRmsF * 1E8);

			setLeds(lRms);

			// Send data via UART2
			uint8_t buffer[12] = {"0"};
      float2buf(buffer, lRmsF, "L");
      HAL_UART_Transmit(&huart2, buffer, sizeof(buffer)/sizeof(*buffer) - 1, 0xFF);
      float2buf(buffer, rRmsF, "R");
      HAL_UART_Transmit(&huart2, buffer, sizeof(buffer)/sizeof(*buffer) - 1, 0xFF);
  	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2S1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2s1ClockSelection = RCC_I2S1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_SLAVE_RX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|p3Pin_Pin|p2Pin_Pin|n2Pin_Pin 
                          |n1Pin_Pin|n10Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, n20Pin_Pin|pn0Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, p1Pin_Pin|n3Pin_Pin|n7Pin_Pin|n5Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 p3Pin_Pin p2Pin_Pin n2Pin_Pin 
                           n1Pin_Pin n10Pin_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|p3Pin_Pin|p2Pin_Pin|n2Pin_Pin 
                          |n1Pin_Pin|n10Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : n20Pin_Pin pn0Pin_Pin */
  GPIO_InitStruct.Pin = n20Pin_Pin|pn0Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : p1Pin_Pin n3Pin_Pin n7Pin_Pin n5Pin_Pin */
  GPIO_InitStruct.Pin = p1Pin_Pin|n3Pin_Pin|n7Pin_Pin|n5Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static inline void float2buf(uint8_t *buf, float num, char *chn) {
  // Set channel indicator
  buf[0] = *chn;
  sprintf(buf+1, "%.8f", fabs(num));
}

static bool storeSample() {
  Sample lSample;
  Sample rSample;

  lSample.i = (int32_t) abs((rxBuffer[0] << 16) | rxBuffer[1]);
  rSample.i = (int32_t) abs((rxBuffer[2] << 16) | rxBuffer[3]);

  float32_t lSampleF;
  float32_t rSampleF;
  arm_q31_to_float(&lSample.q, &lSampleF, 1);
  arm_q31_to_float(&rSample.q, &rSampleF, 1);

  lSampleBuf[sampleCounter] = lSampleF;
  rSampleBuf[sampleCounter] = rSampleF;
  sampleCounter++;

  if (lSample.i != 0 || rSample.i != 0) {
    return true;
  } else {
  	return false;
  }
}

static void setLeds(uint32_t rmsValue) {
  HAL_GPIO_WritePin(GPIOA, p3Pin_Pin|p2Pin_Pin|n2Pin_Pin|n1Pin_Pin|n10Pin_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, n20Pin_Pin|pn0Pin_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, p1Pin_Pin|n3Pin_Pin|n7Pin_Pin|n5Pin_Pin, GPIO_PIN_RESET);

  if (rmsValue > 30253 && rmsValue < 60794) {
    HAL_GPIO_WritePin(GPIOC, n20Pin_Pin, GPIO_PIN_SET);
  } else if (rmsValue >= 60794 && rmsValue < 132061) {
    HAL_GPIO_WritePin(GPIOA, n10Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, n20Pin_Pin, GPIO_PIN_SET);
  } else if (rmsValue >= 132061 && rmsValue < 264829) {
    HAL_GPIO_WritePin(GPIOA, n10Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, n20Pin_Pin|pn0Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, n7Pin_Pin, GPIO_PIN_SET);
  } else if (rmsValue >= 264829 && rmsValue < 439503) {
    HAL_GPIO_WritePin(GPIOA, n10Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, n20Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, n7Pin_Pin|n5Pin_Pin, GPIO_PIN_SET);
  } else if (rmsValue >= 439503 && rmsValue < 1048978) {
    HAL_GPIO_WritePin(GPIOA, n10Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, n20Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, n3Pin_Pin|n7Pin_Pin|n5Pin_Pin, GPIO_PIN_SET);
  } else if (rmsValue >= 1048978 && rmsValue < 1742188) {
    HAL_GPIO_WritePin(GPIOA, n2Pin_Pin|n10Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, n20Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, n3Pin_Pin|n7Pin_Pin|n5Pin_Pin, GPIO_PIN_SET);
  } else if (rmsValue >= 1742188 && rmsValue < 3792372) {
    HAL_GPIO_WritePin(GPIOA, n2Pin_Pin|n1Pin_Pin|n10Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, n20Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, n3Pin_Pin|n7Pin_Pin|n5Pin_Pin, GPIO_PIN_SET);
  } else if (rmsValue >= 3792372 && rmsValue < 8292617) {
    HAL_GPIO_WritePin(GPIOA, n2Pin_Pin|n1Pin_Pin|n10Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, n20Pin_Pin|pn0Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, n3Pin_Pin|n7Pin_Pin|n5Pin_Pin, GPIO_PIN_SET);
  } else if (rmsValue >= 8292617 && rmsValue < 16105936) {
    HAL_GPIO_WritePin(GPIOA, n2Pin_Pin|n1Pin_Pin|n10Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, n20Pin_Pin|pn0Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, p1Pin_Pin|n3Pin_Pin|n7Pin_Pin|n5Pin_Pin, GPIO_PIN_SET);
  } else if (rmsValue >= 16105936 && rmsValue < 33961411) {
    HAL_GPIO_WritePin(GPIOA, p2Pin_Pin|n2Pin_Pin|n1Pin_Pin|n10Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, n20Pin_Pin|pn0Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, p1Pin_Pin|n3Pin_Pin|n7Pin_Pin|n5Pin_Pin, GPIO_PIN_SET);
  } else if (rmsValue >= 33961411) {
    HAL_GPIO_WritePin(GPIOA, p3Pin_Pin|p2Pin_Pin|n2Pin_Pin|n1Pin_Pin|n10Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, n20Pin_Pin|pn0Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, p1Pin_Pin|n3Pin_Pin|n7Pin_Pin|n5Pin_Pin, GPIO_PIN_SET);
  }
}

void HAL_I2S_RxHalfCpltCallback (I2S_HandleTypeDef *hi2s) {
  if (sampleCounter == SAMPLE_WINDOW) { return; }	// Fail safe

  bool led = storeSample();

  if (led) {
    GPIOA->ODR |= GPIO_PIN_5;
  }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (sampleCounter == SAMPLE_WINDOW) { return; }	// Fail safe

  bool led = storeSample();

  if (led) {
    GPIOA->ODR &= ~GPIO_PIN_5;
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
