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
#include "stm32f1xx_hal.h"

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
#define M1_dir_Pin GPIO_PIN_14
#define M1_dir_GPIO_Port GPIOC
#define M3_dir_Pin GPIO_PIN_15
#define M3_dir_GPIO_Port GPIOC
#define M2_dir_Pin GPIO_PIN_0
#define M2_dir_GPIO_Port GPIOA
#define M4_dir_Pin GPIO_PIN_1
#define M4_dir_GPIO_Port GPIOA
#define En1_C1_Pin GPIO_PIN_3
#define En1_C1_GPIO_Port GPIOA
#define En1_C1_EXTI_IRQn EXTI3_IRQn
#define En1_C2_Pin GPIO_PIN_4
#define En1_C2_GPIO_Port GPIOA
#define En1_C2_EXTI_IRQn EXTI4_IRQn
#define En2_C1_Pin GPIO_PIN_5
#define En2_C1_GPIO_Port GPIOA
#define En2_C1_EXTI_IRQn EXTI9_5_IRQn
#define En2_C2_Pin GPIO_PIN_6
#define En2_C2_GPIO_Port GPIOA
#define En2_C2_EXTI_IRQn EXTI9_5_IRQn
#define En3_C1_Pin GPIO_PIN_7
#define En3_C1_GPIO_Port GPIOA
#define En3_C1_EXTI_IRQn EXTI9_5_IRQn
#define En3_C2_Pin GPIO_PIN_0
#define En3_C2_GPIO_Port GPIOB
#define En3_C2_EXTI_IRQn EXTI0_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
