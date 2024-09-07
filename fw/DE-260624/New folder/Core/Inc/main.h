/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VENT_DIR_IN_Pin GPIO_PIN_0
#define VENT_DIR_IN_GPIO_Port GPIOF
#define VENT_DIR_OUT_Pin GPIO_PIN_1
#define VENT_DIR_OUT_GPIO_Port GPIOF
#define LED_SPEED_1_Pin GPIO_PIN_0
#define LED_SPEED_1_GPIO_Port GPIOA
#define LED_SPEED_2_Pin GPIO_PIN_1
#define LED_SPEED_2_GPIO_Port GPIOA
#define LED_SPEED_3_Pin GPIO_PIN_2
#define LED_SPEED_3_GPIO_Port GPIOA
#define LED_SPEED_4_Pin GPIO_PIN_3
#define LED_SPEED_4_GPIO_Port GPIOA
#define LED_DIR_IN_Pin GPIO_PIN_4
#define LED_DIR_IN_GPIO_Port GPIOA
#define LED_DIR_OUT_Pin GPIO_PIN_5
#define LED_DIR_OUT_GPIO_Port GPIOA
#define ALERT_Pin GPIO_PIN_7
#define ALERT_GPIO_Port GPIOA
#define ALERT_EXTI_IRQn EXTI4_15_IRQn
#define LED_ON_OFF_Pin GPIO_PIN_1
#define LED_ON_OFF_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
