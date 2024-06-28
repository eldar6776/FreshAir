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
extern __IO uint32_t din_cap_sen;
/***********************************************************************
**	 C A P A C I T I V E		S E N S O R 		S T A T E S		
***********************************************************************/
#define IsHandmaidSwitchActiv()			(din_cap_sen &   (1U<<0))
#define IsDoorBellSwitchActiv()			(din_cap_sen &   (1U<<2))
#define CAP1293_SensorPresent()			(din_cap_sen |=  (1U<<7))
#define CAP1293_SensorNotPresent()		(din_cap_sen &=(~(1U<<7)))
#define IsCAP1293_Present()				(din_cap_sen &   (1U<<7))
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
#define CAP1293_MAIN_CONTROL_REG					((uint8_t)0x00U)
#define CAP1293_GENERAL_STATUS_REG					((uint8_t)0x02U)
#define CAP1293_SENSOR_INPUT_STATUS_REG				((uint8_t)0x03U)
#define CAP1293_NOISE_FLAG_STATUS_REG				((uint8_t)0x0aU)
#define CAP1293_SENSOR_1_INPUT_DELTA_COUNT_REG		((uint8_t)0x10U)
#define CAP1293_SENSOR_2_INPUT_DELTA_COUNT_REG		((uint8_t)0x11U)
#define CAP1293_SENSOR_3_INPUT_DELTA_COUNT_REG		((uint8_t)0x12U)
#define CAP1293_SENSOR_4_INPUT_DELTA_COUNT_REG		((uint8_t)0x13U)
#define CAP1293_SENSOR_5_INPUT_DELTA_COUNT_REG		((uint8_t)0x14U)
#define CAP1293_SENSOR_6_INPUT_DELTA_COUNT_REG		((uint8_t)0x15U)
#define CAP1293_SENSOR_7_INPUT_DELTA_COUNT_REG		((uint8_t)0x16U)
#define CAP1293_SENSOR_8_INPUT_DELTA_COUNT_REG		((uint8_t)0x17U)
#define CAP1293_SENSITIVITY_CONTROL_REG				((uint8_t)0x1fU)
#define CAP1293_CONFIGURATION_REG					((uint8_t)0x20U)
#define CAP1293_SENSOR_INPUT_ENABLE_REG				((uint8_t)0x21U)
#define CAP1293_SENSOR_INPUT_CONFIGURATION_REG		((uint8_t)0x22U)
#define CAP1293_SENSOR_INPUT_CONFIGURATION_2_REG	((uint8_t)0x23U)
#define CAP1293_AVERAGING_AND_SAMPLING_CONFIG_REG	((uint8_t)0x24U)
#define CAP1293_CALIBRATION_ACTIVATE_AND_STATUS_REG	((uint8_t)0x26U)
#define CAP1293_INTERRUPT_ENABLE_REG				((uint8_t)0x27U)
#define CAP1293_REPEAT_RATE_ENABLE_REG				((uint8_t)0x28U)
#define CAP1293_SINGLE_GUARD_ENABLE_REG				((uint8_t)0x29U)
#define CAP1293_MULTIPLE_TOUCH_CONFIGURATION_REG	((uint8_t)0x2aU)
#define CAP1293_MULTIPLE_TOUCH_PATTERN_CONFIG_REG	((uint8_t)0x2bU)
#define CAP1293_MULTIPLE_TOUCH_PATTERN_REG			((uint8_t)0x2dU)
#define CAP1293_BASE_COUNT_OF_LIMIT_REG				((uint8_t)0x2eU)
#define CAP1293_RECALIBRATION_CONFIGURATION_REG		((uint8_t)0x2fU)
#define CAP1293_SENSOR_INPUT_1_TRESHOLD_REG			((uint8_t)0x30U)
#define CAP1293_SENSOR_INPUT_2_TRESHOLD_REG			((uint8_t)0x31U)
#define CAP1293_SENSOR_INPUT_3_TRESHOLD_REG			((uint8_t)0x32U)
#define CAP1293_SENSOR_INPUT_4_TRESHOLD_REG			((uint8_t)0x33U)
#define CAP1293_SENSOR_INPUT_5_TRESHOLD_REG			((uint8_t)0x34U)
#define CAP1293_SENSOR_INPUT_6_TRESHOLD_REG			((uint8_t)0x35U)
#define CAP1293_SENSOR_INPUT_7_TRESHOLD_REG			((uint8_t)0x36U)
#define CAP1293_SENSOR_INPUT_8_TRESHOLD_REG			((uint8_t)0x37U)
#define CAP1293_SENSOR_INPUT_NOISE_TRESHOLD_REG		((uint8_t)0x38U)
#define CAP1293_STANDBY_CHANNEL_REG					((uint8_t)0x40U)
#define CAP1293_STANDBY_CONFIGURATION_REG			((uint8_t)0x41U)
#define CAP1293_STANDBY_SENSITIVITY_REG				((uint8_t)0x42U)
#define CAP1293_STANDBY_TRESHOLD_REG				((uint8_t)0x43U)
#define CAP1293_CONFIGURATION_2_REG					((uint8_t)0x44U)
#define CAP1293_SENSOR_INPUT_1_BASE_COUNT_REG		((uint8_t)0x50U)
#define CAP1293_SENSOR_INPUT_2_BASE_COUNT_REG		((uint8_t)0x51U)
#define CAP1293_SENSOR_INPUT_3_BASE_COUNT_REG		((uint8_t)0x52U)
#define CAP1293_SENSOR_INPUT_4_BASE_COUNT_REG		((uint8_t)0x53U)
#define CAP1293_SENSOR_INPUT_5_BASE_COUNT_REG		((uint8_t)0x54U)
#define CAP1293_SENSOR_INPUT_6_BASE_COUNT_REG		((uint8_t)0x55U)
#define CAP1293_SENSOR_INPUT_7_BASE_COUNT_REG		((uint8_t)0x56U)
#define CAP1293_SENSOR_INPUT_8_BASE_COUNT_REG		((uint8_t)0x57U)
#define CAP1293_POWER_BUTTON_REG					((uint8_t)0x60U)
#define CAP1293_POWER_BUTTON_CONFIGURATION_REG		((uint8_t)0x61U)
#define CAP1293_CALIBRATION_SENSITIVITY_CONFIG_REG	((uint8_t)0x80U)
#define CAP1293_SENSOR_INPUT_1_CALIBRATION_REG		((uint8_t)0xb1U)
#define CAP1293_SENSOR_INPUT_2_CALIBRATION_REG		((uint8_t)0xb2U)
#define CAP1293_SENSOR_INPUT_3_CALIBRATION_REG		((uint8_t)0xb3U)
#define CAP1293_SENSOR_INPUT_4_CALIBRATION_REG		((uint8_t)0xb4U)
#define CAP1293_SENSOR_INPUT_5_CALIBRATION_REG		((uint8_t)0xb5U)
#define CAP1293_SENSOR_INPUT_6_CALIBRATION_REG		((uint8_t)0xb6U)
#define CAP1293_SENSOR_INPUT_7_CALIBRATION_REG		((uint8_t)0xb7U)
#define CAP1293_SENSOR_INPUT_8_CALIBRATION_REG		((uint8_t)0xb8U)
#define CAP1293_SENSOR_INPUT_CALIBRATION_LSB_REG	((uint8_t)0xb9U)
#define CAP1293_PRODUCT_ID_REG						((uint8_t)0xfdU)
#define CAP1293_MANUFACTURER_ID_REG					((uint8_t)0xfeU)
#define CAP1293_REVISION_REG						((uint8_t)0xffU)
#define DRV_TOUT    100
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
