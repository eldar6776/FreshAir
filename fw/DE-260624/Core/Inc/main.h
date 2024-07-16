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
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/***********************************************************************
**	 C A P A C I T I V E		S E N S O R 		S T A T E S		
***********************************************************************/

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint8_t CAP1293_ReadRegister(uint8_t register_address);
void CAP1293_WriteRegister(uint8_t register_address, uint8_t register_data);
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
#define LED_ON_OFF_Pin GPIO_PIN_1
#define LED_ON_OFF_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define CAP1293_PRODUCT_ID							((uint8_t)0x6fU)
#define CAP1293_VENDOR_ID							((uint8_t)0x5dU)
#define CAP1293_WRITE								((uint8_t)0x50U)
#define CAP1293_READ								((uint8_t)0x51U)
/***********************************************************************
**	 	C A P 1 2 9 3 	  	 R E G I S T E R		A D D E S S E 
***********************************************************************/

#define DRV_TOUT    100
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
