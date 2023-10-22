/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	RED,
	BLUE,
	GREEN,
	YELLOW,
	WHITE = 0xFF
} Color;
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
#define SLIDE_SAMPLING_G1_Pin GPIO_PIN_0
#define SLIDE_SAMPLING_G1_GPIO_Port GPIOA
#define SLIDE_20_Pin GPIO_PIN_1
#define SLIDE_20_GPIO_Port GPIOA
#define SLIDE_0_30_Pin GPIO_PIN_3
#define SLIDE_0_30_GPIO_Port GPIOA
#define SLIDE_SAMPLING_G7_Pin GPIO_PIN_12
#define SLIDE_SAMPLING_G7_GPIO_Port GPIOB
#define SLIDE_END_Pin GPIO_PIN_13
#define SLIDE_END_GPIO_Port GPIOB
#define SLIDE_5_Pin GPIO_PIN_14
#define SLIDE_5_GPIO_Port GPIOB
#define SLIDE_10_Pin GPIO_PIN_15
#define SLIDE_10_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_8
#define LED_GREEN_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_7
#define LED_RED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
