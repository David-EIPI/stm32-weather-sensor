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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

#include "stm32wbxx_ll_adc.h"
#include "stm32wbxx_ll_lptim.h"
#include "stm32wbxx_ll_system.h"
#include "stm32wbxx_ll_gpio.h"
#include "stm32wbxx_ll_exti.h"
#include "stm32wbxx_ll_bus.h"
#include "stm32wbxx_ll_cortex.h"
#include "stm32wbxx_ll_rcc.h"
#include "stm32wbxx_ll_utils.h"
#include "stm32wbxx_ll_pwr.h"
#include "stm32wbxx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern I2C_HandleTypeDef hi2c1;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void startSensorUpdates(void);
void stopSensorUpdates(void);

void SystemClock_Decrease(void);
void SystemClock_Config(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BME280_Supply_Pin LL_GPIO_PIN_0
#define BME280_Supply_GPIO_Port GPIOA
#define BOARDLED_Pin LL_GPIO_PIN_4
#define BOARDLED_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
