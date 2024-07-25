/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAP129n.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
    STATE_OFF = 0U,
    STATE_ON,
    STATE_PAUSE
}state_t;

typedef enum{
    SPEED_OFF = 0U,
    SPEED_1,
    SPEED_2,
    SPEED_3,
    SPEED_4
}speed_t;

typedef enum{
    MODE_OFF = 0U,
    MODE_HR,
    MODE_VENT
}mode_t;

typedef enum{
    PAUSE_OFF = 0U,
    PAUSE_1H,
    PAUSE_2H,
    PAUSE_4H,
    PAUSE_8H
}pause_t;

typedef struct{
    state_t state;
    speed_t speed;
    speed_t old_speed;
    mode_t mode;
    pause_t pause;
    uint32_t timer;
    uint32_t timeout;
    uint32_t touch_tmr;
    uint32_t led_tmr;
    uint32_t blink_tmr;
    uint32_t pause_tmr;
    uint32_t pause_timeout;
    uint32_t pwm;
    uint8_t cap;
    uint8_t touch_enable;
    uint32_t filter_tmr;
    uint8_t filter;
}fan_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define PWM_SPEED_1 999
#define PWM_SPEED_2 666
#define PWM_SPEED_3 333
#define PWM_SPEED_4 0
#define PWM_RATE    20
#define LED_ON_TIME 20000
#define LED_BLINK_TIME 500
#define FANSTOP_TOUT1   2000
#define FANSTOP_TOUT2   4000
#define FANSTOP_TOUT3   8000
#define FANSTOP_TOUT4   12000
#define ON_OFF_BTN_TIME 5000
#define FAN_HR_DIR_TIME 70000
#define FILTER_CLEAN    4320 // 180 days x 24 hours filter cleaning warning
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
fan_t fan;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void CAP1293_Init(void);
static void SetLed(fan_t* fan);
static void SetSpeed(fan_t* fan);
static void SetMode(fan_t* fan);
static void SetPause(fan_t* fan);
static void GetTouch(fan_t* fan);
static void SetPwm(fan_t* fan);
static void SetOut(fan_t* fan);
static void CheckTimer(fan_t* fan);
static void Save(fan_t* fan);
static void Load(fan_t* fan);
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
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  CAP1293_Init();
  HAL_Delay(100);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  fan.state = STATE_ON;
  fan.mode = MODE_VENT;
  fan.speed = SPEED_2;
  fan.old_speed = SPEED_OFF;
  fan.pause = PAUSE_OFF;
  fan.timeout = 0;
  fan.timer = 0;
  fan.touch_enable = 0;
  fan.touch_tmr = HAL_GetTick();
  fan.led_tmr = HAL_GetTick();
  fan.pwm = PWM_SPEED_1;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, fan.pwm);
  
  while (1)
  {
      
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
    GetTouch(&fan);
    SetSpeed(&fan);
    SetPause(&fan);
    SetMode(&fan); 
    SetOut(&fan);
    SetPwm(&fan);
    SetLed(&fan);  
    CheckTimer(&fan);  
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */
#ifdef USE_WATCHDOG
  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
#endif
  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, VENT_DIR_IN_Pin|VENT_DIR_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_SPEED_1_Pin|LED_SPEED_2_Pin|LED_SPEED_3_Pin|LED_SPEED_4_Pin
                          |LED_DIR_IN_Pin|LED_DIR_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VENT_DIR_IN_Pin VENT_DIR_OUT_Pin */
  GPIO_InitStruct.Pin = VENT_DIR_IN_Pin|VENT_DIR_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_SPEED_1_Pin LED_SPEED_2_Pin LED_SPEED_3_Pin LED_SPEED_4_Pin
                           LED_DIR_IN_Pin LED_DIR_OUT_Pin */
  GPIO_InitStruct.Pin = LED_SPEED_1_Pin|LED_SPEED_2_Pin|LED_SPEED_3_Pin|LED_SPEED_4_Pin
                          |LED_DIR_IN_Pin|LED_DIR_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ALERT_Pin */
  GPIO_InitStruct.Pin = ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ALERT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_ON_OFF_Pin */
  GPIO_InitStruct.Pin = LED_ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ON_OFF_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void SetLed(fan_t* fan){
    static uint8_t cnt = 0;
    
    if(fan->led_tmr){
        switch(fan->state){
            case STATE_ON:
                switch(fan->speed){
                    case SPEED_1:
                        HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_RESET);
                        break;
                    case SPEED_2:
                        HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_RESET);
                        break;
                    case SPEED_3:
                        HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_RESET);
                        break;
                    case SPEED_4:
                        HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_SET);
                        break;
                    case SPEED_OFF:
                    default:
                        HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_RESET);
                        break;
                }
                break;
            case STATE_PAUSE:
                if(cnt == 0){
                    switch(fan->pause){
                        case PAUSE_1H:
                            HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_SET);
                            HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_RESET);
                            break;
                        case PAUSE_2H:
                            HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_SET);
                            HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_RESET);
                            break;
                        case PAUSE_4H:
                            HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_SET);
                            HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_RESET);
                           break;
                        case PAUSE_8H:
                            HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_SET);
                            break;
                        case PAUSE_OFF:
                        default:
                            HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);   
                            HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_RESET);
                            break;
                    }
                    ++cnt;
                    fan->blink_tmr = HAL_GetTick();
                }else if(cnt == 1){
                    if(!fan->blink_tmr){
                        HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_RESET);
                        fan->blink_tmr = HAL_GetTick();
                        ++cnt;
                    }
                }else if(cnt == 2){
                    if(!fan->blink_tmr){
                        cnt = 0;
                    }
                }
                break;
            case STATE_OFF:
            default:
                HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_RESET);
                break;
        }
        
        switch(fan->mode){
            case MODE_HR:
                    HAL_GPIO_WritePin(LED_DIR_IN_GPIO_Port, LED_DIR_IN_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(LED_DIR_OUT_GPIO_Port, LED_DIR_OUT_Pin, GPIO_PIN_RESET);
                break;
            case MODE_VENT:
                HAL_GPIO_WritePin(LED_DIR_OUT_GPIO_Port, LED_DIR_OUT_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_DIR_IN_GPIO_Port, LED_DIR_IN_Pin, GPIO_PIN_RESET);
                break;
            case MODE_OFF:
            default:
                HAL_GPIO_WritePin(LED_DIR_IN_GPIO_Port, LED_DIR_IN_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_DIR_OUT_GPIO_Port, LED_DIR_OUT_Pin, GPIO_PIN_RESET);
                break;
        }
        if(fan->touch_enable == 1) fan->touch_enable = 2;
    }else{
        HAL_GPIO_WritePin(LED_SPEED_1_GPIO_Port, LED_SPEED_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_SPEED_2_GPIO_Port, LED_SPEED_2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_SPEED_3_GPIO_Port, LED_SPEED_3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_SPEED_4_GPIO_Port, LED_SPEED_4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_ON_OFF_GPIO_Port, LED_ON_OFF_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_DIR_IN_GPIO_Port, LED_DIR_IN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_DIR_OUT_GPIO_Port, LED_DIR_OUT_Pin, GPIO_PIN_RESET);
    }
};

static void SetSpeed(fan_t* fan){
    static uint8_t tmp = 0;
    
    if(!fan->touch_enable){
        if(!tmp && (fan->cap & 0x04U)){
            ++tmp;
            fan->speed = SPEED_1;
            fan->state = STATE_ON;
            fan->pause = PAUSE_OFF;
        }else if(!tmp && (fan->cap & 0x08U)){
            ++tmp;
            fan->speed = SPEED_2;
            fan->state = STATE_ON;
            fan->pause = PAUSE_OFF;
        }else if(!tmp && (fan->cap & 0x20U)){
            ++tmp;
            fan->speed = SPEED_3;
            fan->state = STATE_ON;
            fan->pause = PAUSE_OFF;
        }else if(!tmp && (fan->cap & 0x40U)){
            ++tmp;
            fan->speed = SPEED_4;
            fan->state = STATE_ON;
            fan->pause = PAUSE_OFF;
        } else tmp = 0;
    }
};

static void SetMode(fan_t* fan){
    static uint8_t tmp = 0;
    
    if(!fan->touch_enable){
        if(fan->cap & 0x01U){
            if(!tmp && (fan->mode == MODE_HR)){
                ++tmp;
                fan->mode = MODE_VENT;
            } else if(!tmp && (fan->mode == MODE_VENT)){
                ++tmp;
                fan->mode = MODE_HR;
            }
        } else tmp = 0;
    }
};

static void SetPause(fan_t* fan){
    static uint8_t cnt = 0, en = 0;
    static uint32_t tmr = 0;
    
    if(!fan->touch_enable){
        if(!cnt && (fan->cap & 0x02)){
            fan->state = STATE_PAUSE;
            if      (fan->pause == PAUSE_OFF) fan->pause = PAUSE_1H, fan->pause_timeout = 3600000U;
            else if (fan->pause == PAUSE_1H) fan->pause = PAUSE_2H, fan->pause_timeout = 7200000U;
            else if (fan->pause == PAUSE_2H) fan->pause = PAUSE_4H, fan->pause_timeout = 14400000U;
            else if (fan->pause == PAUSE_4H) fan->pause = PAUSE_8H, fan->pause_timeout = 28800000U;
            else if (fan->pause == PAUSE_8H) fan->pause = PAUSE_1H, fan->pause_timeout = 3600000U;
            fan->pause_tmr = HAL_GetTick();
            tmr = HAL_GetTick();
            ++cnt;
        }else if (cnt == 1){
            if (!(fan->cap & 0x02)){
                cnt = 0;
                tmr = 0;
            }else if((HAL_GetTick() - tmr) >= ON_OFF_BTN_TIME){
                fan->pause_tmr = 0;
                fan->pause_timeout = 0;
                fan->state = STATE_OFF;
                fan->pause = PAUSE_OFF;
                ++cnt;
            }
        }else if(cnt == 2){
            if (!(fan->cap & 0x02)){
                cnt = 0;
                tmr = 0;
            }
        }
    }
    
    if(fan->state == STATE_PAUSE){
        if(!fan->pause_tmr && !en){
            fan->state = STATE_ON;
            fan->pause = PAUSE_OFF;
            fan->led_tmr = HAL_GetTick();
            ++en;
        } else if (fan->pause_tmr && en){
            en = 0;
        }
    }
};
static void GetTouch(fan_t* fan){
    uint8_t tmp;
    fan->cap = CAP1293_ReadRegister(SENSOR_INPUT_STATUS);
    tmp = CAP1293_ReadRegister(MAIN_CONTROL);
    if(tmp & (1U<<0)) CAP1293_WriteRegister(MAIN_CONTROL, 0);
    if(fan->cap) fan->touch_tmr = HAL_GetTick(),  fan->led_tmr = HAL_GetTick();
    if(!fan->cap && (fan->touch_enable == 2)) fan->touch_enable = 0;
};

static void SetPwm(fan_t* fan){
    static uint32_t tmr = 0, cnt = 0, dir = 0;
    
    if(fan->old_speed != fan->speed){
        if(cnt == 0){
            if(fan->old_speed < fan->speed) dir = 2;
            else dir = 1;
            tmr = HAL_GetTick();
            ++cnt;
        }else if(cnt == 1){
            if((HAL_GetTick() - tmr) >= PWM_RATE){
                if((dir == 1) && (fan->pwm < PWM_SPEED_1)) ++fan->pwm;
                else if((dir == 2) && (fan->pwm)) --fan->pwm;
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, fan->pwm);
            }
            if((fan->speed == SPEED_1) && (fan->pwm == PWM_SPEED_1)) fan->old_speed = fan->speed;
            if((fan->speed == SPEED_2) && (fan->pwm == PWM_SPEED_2)) fan->old_speed = fan->speed;
            if((fan->speed == SPEED_3) && (fan->pwm == PWM_SPEED_3)) fan->old_speed = fan->speed;
            if((fan->speed == SPEED_4) && (fan->pwm == PWM_SPEED_4)) fan->old_speed = fan->speed;
            if(fan->old_speed == fan->speed){
                cnt = 0;
                dir = 0;
                tmr = 0;
            }
        }
    }
};

static void SetOut(fan_t* fan){
    static uint8_t cnt = 0, cnt1 = 0, cnt2 = 0;
    static uint32_t tmr = 0, timeout = 0;
    
    if(fan->state != STATE_ON){
        cnt = 0;
        cnt1 = 0;
        cnt2 = 0;
        fan->timer = 0;
        fan->timeout = 0;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_SPEED_1);
        HAL_GPIO_WritePin(VENT_DIR_IN_GPIO_Port, VENT_DIR_IN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(VENT_DIR_OUT_GPIO_Port, VENT_DIR_OUT_Pin, GPIO_PIN_RESET);
    }else if(fan->mode == MODE_HR){
        if(cnt == 0){
            if((HAL_GetTick() - fan->timer) >= fan->timeout){
                if(cnt1 == 0){
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_SPEED_1);
                    HAL_GPIO_WritePin(VENT_DIR_IN_GPIO_Port, VENT_DIR_IN_Pin, GPIO_PIN_RESET);
                    tmr = HAL_GetTick();
                    if      (fan->speed == SPEED_1) timeout = FANSTOP_TOUT1;
                    else if (fan->speed == SPEED_2) timeout = FANSTOP_TOUT2;
                    else if (fan->speed == SPEED_3) timeout = FANSTOP_TOUT3;
                    else if (fan->speed == SPEED_4) timeout = FANSTOP_TOUT4;
                    ++cnt1;
                }else if(cnt1 == 1){
                    if((HAL_GetTick() - tmr) >= timeout){
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, fan->pwm);
                        HAL_GPIO_WritePin(VENT_DIR_OUT_GPIO_Port, VENT_DIR_OUT_Pin, GPIO_PIN_SET);
                        fan->timer = HAL_GetTick();
                        fan->timeout = FAN_HR_DIR_TIME;
                        cnt1 = 0;
                        ++cnt;
                    }                    
                }
            } 
        }else if(cnt == 1){
             if((HAL_GetTick() - fan->timer) >= fan->timeout){
                if(cnt1 == 0){
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_SPEED_1);
                    HAL_GPIO_WritePin(VENT_DIR_OUT_GPIO_Port, VENT_DIR_OUT_Pin, GPIO_PIN_RESET);
                    tmr = HAL_GetTick();
                    if      (fan->speed == SPEED_1) timeout = FANSTOP_TOUT1;
                    else if (fan->speed == SPEED_2) timeout = FANSTOP_TOUT2;
                    else if (fan->speed == SPEED_3) timeout = FANSTOP_TOUT3;
                    else if (fan->speed == SPEED_4) timeout = FANSTOP_TOUT4;
                    ++cnt1;
                }else if(cnt1 == 1){
                    if((HAL_GetTick() - tmr) >= timeout){
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, fan->pwm);
                        HAL_GPIO_WritePin(VENT_DIR_IN_GPIO_Port, VENT_DIR_IN_Pin, GPIO_PIN_SET);
                        fan->timer = HAL_GetTick();
                        fan->timeout = FAN_HR_DIR_TIME;
                        cnt1 = 0;
                        ++cnt;
                    }                    
                }
            }
        }else{
            cnt1 = 0;
            cnt = 0;
        }
        cnt2 = 0;
    }else if(fan->mode == MODE_VENT){
        if(cnt2 == 0){
            HAL_GPIO_WritePin(VENT_DIR_IN_GPIO_Port, VENT_DIR_IN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(VENT_DIR_OUT_GPIO_Port, VENT_DIR_OUT_Pin, GPIO_PIN_RESET);
            tmr = HAL_GetTick();
            if      (fan->speed == SPEED_1) timeout = FANSTOP_TOUT1;
            else if (fan->speed == SPEED_2) timeout = FANSTOP_TOUT2;
            else if (fan->speed == SPEED_3) timeout = FANSTOP_TOUT3;
            else if (fan->speed == SPEED_4) timeout = FANSTOP_TOUT4;
            ++cnt2;
        }else if(cnt2 == 1){
            if((HAL_GetTick() - tmr) >= timeout){
                fan->pwm = PWM_SPEED_1;
                fan->old_speed = SPEED_OFF;
                HAL_GPIO_WritePin(VENT_DIR_OUT_GPIO_Port, VENT_DIR_OUT_Pin, GPIO_PIN_SET);
                ++cnt2;
            }     
        }
        fan->timer = 0;
        fan->timeout = 0;
        cnt = 0;
        cnt1 = 0;     
    }
};
static void CheckTimer(fan_t* fan){
    static uint32_t reload = 0;
    fan_t tmp;
    
    if((HAL_GetTick() - fan->led_tmr) >= LED_ON_TIME) fan->led_tmr = 0;
    if((HAL_GetTick() - fan->blink_tmr) >= LED_BLINK_TIME) fan->blink_tmr = 0;
    if((HAL_GetTick() - fan->touch_tmr) >= LED_ON_TIME) fan->touch_tmr = 0, fan->touch_enable = 1;
    if((HAL_GetTick() - fan->pause_tmr) >= fan->pause_timeout) fan->pause_tmr = 0;
    if((HAL_GetTick() - reload) >= 3600000U){
        reload = HAL_GetTick();
        Load(&tmp);
        if ((tmp.state != fan->state) ||
            (tmp.mode  != fan->mode)  ||
            (tmp.pause != fan->pause) ||
            (tmp.speed != fan->speed) ||
            (tmp.filter_tmr != fan->filter_tmr)){
                ++fan->filter_tmr;
                Save(fan);
                if(fan->filter_tmr >= FILTER_CLEAN) fan->filter = 1;
            }
    }
}

static void Save(fan_t* fan){
    uint8_t size = sizeof(fan_t);
    uint8_t data[sizeof(fan_t)], addr = 0;
    memcpy(data, fan, sizeof(data));
    while(size){
        HAL_I2C_Mem_Write(&hi2c1, 0xA0, addr, I2C_MEMADD_SIZE_8BIT, &data[addr], 8, 1000);
        if(size >= 8) size -= 8, addr += 8;
        else size = 0; 
    }
}

static void Load(fan_t* fan){
    uint8_t data[sizeof(fan_t)];
    
    HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0, I2C_MEMADD_SIZE_8BIT, data, sizeof(data), 1000);
    memcpy(fan, data, sizeof(data));
}

/**
  * @brief
  * @param
  * @retval
  */
static void CAP1293_Init(void)
{
	uint8_t vendor_id, product_id, reg_wr[2];
	
	vendor_id = 0U;
	product_id = 0U;
    
	reg_wr[0] = PROD_ID;
	if(HAL_I2C_Master_Transmit(&hi2c1, CAP1293_WRITE, reg_wr, 1U, DRV_TOUT) != HAL_OK)       Error_Handler();
	if(HAL_I2C_Master_Receive(&hi2c1, CAP1293_READ, &product_id, 1U, DRV_TOUT) != HAL_OK)    Error_Handler();
	reg_wr[0] = MANUFACTURE_ID;
	if(HAL_I2C_Master_Transmit(&hi2c1, CAP1293_WRITE, reg_wr, 1U, DRV_TOUT) != HAL_OK)       Error_Handler();
	if(HAL_I2C_Master_Receive(&hi2c1, CAP1293_READ, &vendor_id, 1U, DRV_TOUT) != HAL_OK)     Error_Handler();

	if((product_id == PROD_ID_VALUE_1298) && (vendor_id == CAP1293_VENDOR_ID)) 
	{
	
		CAP1293_WriteRegister(MULTIPLE_TOUCH_CONFIG, 0U);
		CAP1293_WriteRegister(SENSOR_INPUT_ENABLE, 0x7FU);
		CAP1293_WriteRegister(INTERRUPT_ENABLE, 0x7FU);
		CAP1293_WriteRegister(REPEAT_RATE_ENABLE, 0U);
		CAP1293_WriteRegister(SIGNAL_GUARD_ENABLE, 0xFFU);
		CAP1293_WriteRegister(SENSITIVITY_CONTROL, 0x2FU);
        calibrateAll();
	}
}
/**
  * @brief
  * @param
  * @retval
  */
uint8_t CAP1293_ReadRegister(uint8_t register_address)
{
	uint8_t ret_val;
	
	if(HAL_I2C_Master_Transmit(&hi2c1, CAP1293_WRITE, &register_address, 1U, DRV_TOUT) != HAL_OK)    Error_Handler();
	if(HAL_I2C_Master_Receive(&hi2c1, CAP1293_READ, &ret_val, 1U, DRV_TOUT) != HAL_OK)               Error_Handler();
	return(ret_val);
}
/**
  * @brief
  * @param
  * @retval
  */
void CAP1293_WriteRegister(uint8_t register_address, uint8_t register_data)
{
	uint8_t reg_val[2];
	
	reg_val[0] = register_address;
	reg_val[1] = register_data;
	if(HAL_I2C_Master_Transmit(&hi2c1, CAP1293_WRITE, reg_val, 2U, DRV_TOUT) != HAL_OK)              Error_Handler();
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
