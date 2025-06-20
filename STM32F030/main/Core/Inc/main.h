/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f0xx_hal.h"

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
#define DHT11_Pin GPIO_PIN_2
#define DHT11_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOA
#define UP_Pin GPIO_PIN_4
#define UP_GPIO_Port GPIOA
#define UP_EXTI_IRQn EXTI4_15_IRQn
#define DOWN_Pin GPIO_PIN_5
#define DOWN_GPIO_Port GPIOA
#define DOWN_EXTI_IRQn EXTI4_15_IRQn
#define BACK_Pin GPIO_PIN_6
#define BACK_GPIO_Port GPIOA
#define BACK_EXTI_IRQn EXTI4_15_IRQn
#define OK_Pin GPIO_PIN_7
#define OK_GPIO_Port GPIOA
#define OK_EXTI_IRQn EXTI4_15_IRQn
#define RELAY_3_Pin GPIO_PIN_14
#define RELAY_3_GPIO_Port GPIOB
#define RELAY_1_Pin GPIO_PIN_8
#define RELAY_1_GPIO_Port GPIOA
#define RELAY_2_Pin GPIO_PIN_10
#define RELAY_2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
