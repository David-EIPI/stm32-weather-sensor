/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/app_zigbee.h
  * @author  MCD Application Team
  * @brief   Header for Zigbee Application.
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
#ifndef APP_ZIGBEE_H
#define APP_ZIGBEE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
#include "tl.h"
#include "tl_zigbee_hci.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/

/* Application errors                 */
/*------------------------------------*/

/*
 *  List of all errors tracked by the Zigbee application
 *  running on M4. Some of these errors may be fatal
 *  or just warnings
 */
typedef enum
{
  ERR_ZIGBE_CMD_TO_M0,
/* USER CODE BEGIN ERROR_APPLI_ENUM */

/* USER CODE END ERROR_APPLI_ENUM */
  ERR_ZIGBEE_CHECK_WIRELESS
} ErrAppliIdEnum_t;
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void APP_ZIGBEE_Init(void);
void APP_ZIGBEE_Error(uint32_t ErrId, uint32_t ErrCode);
void APP_ZIGBEE_RegisterCmdBuffer(TL_CmdPacket_t *p_buffer);
void APP_ZIGBEE_TL_INIT(void);
void Pre_ZigbeeCmdProcessing(void);
/* USER CODE BEGIN EF */

void zigbee_check_link(void);
void APP_ZIGBEE_update_BME280_outputs(int32_t temp, uint32_t press, uint32_t hum);
void APP_ZIGBEE_update_ADC_outputs(uint32_t voltage);

void ZIGBEE_attempt_rejoin(void);
enum ZbStatusCodeT ZIGBEE_JoinStatus(void);

void start_fastpoll(uint32_t timeout_us);
/* USER CODE END EF */
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* APP_ZIGBEE_H */
