/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_system.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define leftChannel_Pin GPIO_PIN_0
#define leftChannel_GPIO_Port GPIOA
#define rightChannel_Pin GPIO_PIN_1
#define rightChannel_GPIO_Port GPIOA
#define n20Pin_Pin GPIO_PIN_2
#define n20Pin_GPIO_Port GPIOA
#define n10Pin_Pin GPIO_PIN_3
#define n10Pin_GPIO_Port GPIOA
#define n7Pin_Pin GPIO_PIN_4
#define n7Pin_GPIO_Port GPIOA
#define n5Pin_Pin GPIO_PIN_5
#define n5Pin_GPIO_Port GPIOA
#define n3Pin_Pin GPIO_PIN_6
#define n3Pin_GPIO_Port GPIOA
#define n2Pin_Pin GPIO_PIN_7
#define n2Pin_GPIO_Port GPIOA
#define n1Pin_Pin GPIO_PIN_8
#define n1Pin_GPIO_Port GPIOA
#define pn0Pin_Pin GPIO_PIN_9
#define pn0Pin_GPIO_Port GPIOA
#define p1Pin_Pin GPIO_PIN_10
#define p1Pin_GPIO_Port GPIOA
#define p2Pin_Pin GPIO_PIN_11
#define p2Pin_GPIO_Port GPIOA
#define p3Pin_Pin GPIO_PIN_12
#define p3Pin_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
