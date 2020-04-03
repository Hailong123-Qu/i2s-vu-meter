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
#define SAMPLE_WINDOW 2200  // 50 ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
__IO ITStatus i2sReady = SET;
uint16_t rxBuffer[8];
q31_t lSampleBuf[SAMPLE_WINDOW];
q31_t rSampleBuf[SAMPLE_WINDOW];
uint16_t sampleCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void int2buf(char *buf, int32_t num, char *chn);
static void float2buf(char *buf, float num, char *chn);
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if (sampleCounter == SAMPLE_WINDOW) {
//  			float32_t a[] = {0.3962, 0.4397, 0.3164, 0.2097, 0.8512, 0.2042, 0.4470, 0.8055, 0.8835, 0.773, 0.4341, 0.1206, 0.7533, 0.9478, 0.2790, 0.2339, 0.995, 0.9994, 0.6407, 0.3317, 0.3032, 0.2137, 0.5389, 0.6209, 0.6090, 0.9075, 0.3504, 0.7641, 0.2121, 0.4372, 0.8589, 0.4985, 0.5113, 0.373, 0.568, 0.7245, 0.1036, 0.6595, 0.340, 0.2594, 0.9380, 0.1393, 0.4319, 0.7338, 0.6393, 0.7850, 0.8846, 0.9670, 0.6537, 0.1028, 0.1618, 0.1748, 0.6172, 0.4288, 0.9433, 0.4758, 0.5151, 0.7948, 0.5516, 0.7346, 0.8945, 0.5484, 0.3629, 0.1376, 0.2748, 0.5068, 0.9387, 0.1021, 0.3054, 0.8747, 0.6056, 0.3818, 0.9751, 0.9874, 0.5564, 0.889, 0.3860, 0.9819, 0.4033, 0.1405, 0.2723, 0.5474, 0.490, 0.4316, 0.5342, 0.2334, 0.2784, 0.4896, 0.2391, 0.8915, 0.333, 0.9592, 0.9540, 0.7032, 0.9235, 0.9614, 0.6863, 0.3851, 0.7715, 0.649, 0.43, 0.879, 0.7755, 0.1074, 0.769, 0.7108, 0.1521, 0.4634, 0.2292, 0.7316, 0.5933, 0.9733, 0.2886, 0.1208, 0.9473, 0.5154, 0.6930, 0.4400, 0.480, 0.2882, 0.1108, 0.6043, 0.4658, 0.6354, 0.9774, 0.6677, 0.8630, 0.2561, 0.219, 0.3303, 0.4655, 0.4987, 0.1461, 0.2710, 0.7311, 0.7608, 0.8313, 0.3071, 0.7433, 0.5855, 0.1501, 0.9512, 0.3181, 0.8419, 0.7127, 0.3034, 0.1872, 0.6713, 0.9431, 0.8355, 0.8857, 0.430, 0.9870, 0.4365, 0.8105, 0.1551, 0.3620, 0.808, 0.360, 0.4379, 0.4053, 0.5787, 0.9649, 0.6697, 0.7757, 0.950, 0.7792, 0.1009, 0.484, 0.2552, 0.9871, 0.4976, 0.1743, 0.2046, 0.5981, 0.1209, 0.8731, 0.8064, 0.9237, 0.4767, 0.5270, 0.8726, 0.1182, 0.2843, 0.2633, 0.9113, 0.9630, 0.4490, 0.5694, 0.7396, 0.4898, 0.6008, 0.6250, 0.9714, 0.2825, 0.6307, 0.9284, 0.95, 0.5314, 0.3202, 0.8441, 0.5262, 0.2521, 0.6190, 0.9252, 0.7021, 0.7620, 0.8372, 0.5743, 0.6619, 0.2877, 0.8347, 0.9853, 0.8090, 0.7960, 0.3091, 0.5334, 0.1915, 0.1204, 0.2529, 0.2613, 0.9588, 0.3249, 0.5860, 0.1463, 0.2984, 0.4096, 0.5992, 0.634, 0.5021, 0.535, 0.8310, 0.9755, 0.5205, 0.7454, 0.2196, 0.9553, 0.7730, 0.6519, 0.9952, 0.1489, 0.5862, 0.711, 0.6251, 0.3146, 0.3409, 0.8432, 0.9543, 0.6768, 0.6460};
//  			float32_t e;
//  			arm_rms_f32(a, sizeof(a)/sizeof(*a), &e);
//  			char buffer[12] = {"0"};
//  			float2buf(buffer, e, "L");
//  			HAL_UART_Transmit(&huart2, buffer, sizeof(buffer)/sizeof(*buffer) - 1, 0xFF);
  		float lFBuf[SAMPLE_WINDOW];
  		float rFBuf[SAMPLE_WINDOW];
			arm_q31_to_float(lSampleBuf, lFBuf, SAMPLE_WINDOW);
			//arm_q31_to_float(rSampleBuf, rFBuf, SAMPLE_WINDOW);
			float lRms;
			//float rRms;
			arm_rms_f32(lFBuf, SAMPLE_WINDOW, &lRms);
			//arm_rms_f32(lFBuf, SAMPLE_WINDOW, &lRms);
			sampleCounter = 0;
			// arm_q31_to_float (const q31_t *pSrc, float32_t *pDst, uint32_t blockSize)
      char buffer[12] = {"0"};
      float2buf(buffer, lRms, "L");
      HAL_UART_Transmit(&huart2, buffer, sizeof(buffer)/sizeof(*buffer) - 1, 0xFF);
//      float2buf(buffer, rRmsF, "R");
//      HAL_UART_Transmit(&huart2, buffer, sizeof(buffer)/sizeof(*buffer) - 1, 0xFF);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static inline void int2buf(char *buf, int32_t num, char *chn) {
  // Set channel indicator
  buf[0] = *chn;

  // Set format according to value
  char *fmt[6];
  if (num < 0) {
    *fmt = "-%010d";
  } else {
    *fmt = "0%010d";
  }

  sprintf(buf+1, *fmt, labs(num));
}

static inline void float2buf(char *buf, float num, char *chn) {
  // Set channel indicator
  buf[0] = *chn;
  sprintf(buf+1, "%.8f", fabs(num));
}

void HAL_I2S_RxHalfCpltCallback (I2S_HandleTypeDef *hi2s) {
  if (sampleCounter == SAMPLE_WINDOW) { return; }	// Fail safe

  Sample lSample;
  Sample rSample;

  lSample.i = (int32_t) abs((rxBuffer[0] << 16) | rxBuffer[1]);
  rSample.i = (int32_t) abs((rxBuffer[2] << 16) | rxBuffer[3]);
  lSampleBuf[sampleCounter] = lSample.q;
  rSampleBuf[sampleCounter] = rSample.q;
  sampleCounter++;

  if (lSample.i != 0 || rSample.i != 0) {
    GPIOA->ODR |= GPIO_PIN_5;
  }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (sampleCounter == SAMPLE_WINDOW) { return; }	// Fail safe

  Sample lSample;
  Sample rSample;

  lSample.i = (int32_t) abs((rxBuffer[0] << 16) | rxBuffer[1]);
  rSample.i = (int32_t) abs((rxBuffer[2] << 16) | rxBuffer[3]);
  lSampleBuf[sampleCounter] = lSample.q;
  rSampleBuf[sampleCounter] = rSample.q;
  sampleCounter++;

  if (lSample.i != 0 || rSample.i != 0) {
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
