/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_seq.h"
#include "stm_logging.h"
#include "dbg_trace.h"
#include "zigbee.h"
#include "app_zigbee.h"
#include "stm32_lpm.h"
#include "stm32_lpm_if.h"
#include "hw_if.h"

#include "zcl/general/zcl.press.meas.h"
#include "zcl/general/zcl.temp.meas.h"
#include "zcl/general/zcl.elec.meas.h"
#include "zcl/general/zcl.wcm.h"

#include "bme280_support.h"
#include "aht20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define HW_TS_SERVER_20S_NB_TICKS                    (SENSOR_INTERVAL *1000*1000/CFG_TS_TICK_VAL) /* 20s */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

IPCC_HandleTypeDef hipcc;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
struct bme280_data bme280_meas_data;

#if USE_AHT20

static aht20_data_t aht20_data;

#endif

uint8_t hwSensorTimerId = CFG_HW_TS_MAX_NBR_CONCURRENT_TIMER;
uint8_t hwActiveTimerId = CFG_HW_TS_MAX_NBR_CONCURRENT_TIMER;

static unsigned SysTicks_us = 0;

uint8_t adc_num_conversions = 0;
int vrefplus = 0;
int internal_tc = 0;
int adc1_voltage = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IPCC_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_RF_Init(void);
/* USER CODE BEGIN PFP */
static void setup_lptimer(void);

static void ADC_check(int idx);
static void ADC_check_print(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t dbg_counter = 0, dbg_prev = 0;

static int wait_for_flag(uint32_t (*flag_func)(const ADC_TypeDef *ADCx), uint32_t flag)
{
	uint32_t timeout = 1000; //ms
	uint32_t count = 0;
	uint32_t prev = HAL_GetTick();

	while (flag_func(ADC1) != flag)
	{
		HAL_Delay(20);
		count = HAL_GetTick() - prev;
		if (count > timeout) {
			dbg_counter = count;
			dbg_prev = prev;
			return -1;
		}
	}
	return count;
}

static void read_voltage_and_temp(void)
{
	int32_t adc_readings[adc_num_conversions];
	int32_t wait_time[adc_num_conversions];

	if (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0) { /* ADC is not ready */
		APP_DBG("ADC not ready.");
		return;
	}

	unsigned i;
	for (i = 0; i < adc_num_conversions; i++) {
		LL_ADC_REG_StartConversion(ADC1);
		wait_time[i] = wait_for_flag(LL_ADC_IsActiveFlag_EOC, 1);
		if (wait_time[i] < 0) {
			APP_DBG("ADC CH%d timeout", i);
			return;
		}
		adc_readings[i] = LL_ADC_REG_ReadConversionData12(ADC1);
	}

	int vref_adc = adc_readings[0];
	int temp_adc = adc_readings[1];
	int ch1_adc  = adc_readings[2];

	if (vref_adc > 0)
		vrefplus = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vref_adc, LL_ADC_RESOLUTION_12B);
	if(temp_adc > 0)
		internal_tc = 100* __LL_ADC_CALC_TEMPERATURE(vrefplus, temp_adc, LL_ADC_RESOLUTION_12B);

	adc1_voltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(vrefplus, ch1_adc, LL_ADC_RESOLUTION_12B);
}

/* Enable the ADC as explained in Examples_LL/ADC_SingleConversion_TriggerSW_Init */
static int adc_enable(void)
{
	/* First, turn on the power */
	LL_ADC_DisableDeepPowerDown(ADC1);
	LL_ADC_EnableInternalRegulator(ADC1);

	/* Delay for voltage regulator stabilization */
	HAL_Delay(1);

	/* Clear ADC group regular conversion flag and overrun flag               */
    /* (To ensure of no unknown state from potential previous ADC operations) */
	LL_ADC_ClearFlag_EOC(ADC1);
	LL_ADC_ClearFlag_EOS(ADC1);
	LL_ADC_ClearFlag_EOSMP(ADC1);
	LL_ADC_ClearFlag_OVR(ADC1);

	/* Run ADC self calibration and wait for completion */
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    if (wait_for_flag(LL_ADC_IsCalibrationOnGoing, 0) < 0) {
    	ADC_check(1);
    	return -1;
    }
	/* Another short delay is needed here */
	HAL_Delay(1);

	/* Now enable the ADC and wait for its Ready flag */
	LL_ADC_Enable(ADC1);
	if (wait_for_flag(LL_ADC_IsActiveFlag_ADRDY, 1) < 0) {
		return -2;
	}
//	APP_DBG("ADC regs: CR:0x%x ISR:0x%x", ADC1->CR, ADC1->ISR);
	return 0;
}

/* Disable the ADC as described in RM034*/
static void adc_disable(void)
{
	/* Stop conversions and wait for ADSTP to clear*/
	LL_ADC_REG_StopConversion(ADC1);
	while (LL_ADC_REG_IsStopConversionOngoing(ADC1)) {}

	/* Clear Ready flag */
	LL_ADC_ClearFlag_ADRDY(ADC1);

	/* Set disable bit and wait for ADEN to clear */
	LL_ADC_Disable(ADC1);
	while (LL_ADC_IsEnabled(ADC1)) {}

	/* Finally, cut the power */
	LL_ADC_DisableInternalRegulator(ADC1);
	LL_ADC_EnableDeepPowerDown(ADC1);
}


/*
 * Main application loop. Read the sensors and update Zigbee attributes.
 * */

void seq_main_loop(void)
{
	LL_GPIO_SetOutputPin(BME280_Supply_GPIO_Port, BME280_Supply_Pin);

	HAL_Delay(10);

	MX_I2C1_Init();

	int adc_res = adc_enable();
	if (0 == adc_res) {
		read_voltage_and_temp();
		if (0 == adc1_voltage) {
			APP_DBG("ADC regs: CR:0x%x ISR:0x%x", ADC1->CR, ADC1->ISR);
		}
	} else {
		APP_DBG("ADC error: %d", adc_res);
		ADC_check_print();
		APP_DBG("Timeout counters: %u, %u", dbg_counter, dbg_prev);
	}

	adc_disable();

	/*
	 * Voltage at the ADC pin is produced by the voltage divider, where R1=1M, and R2=330K.
	 * After calibration I have obtained this ratio: 8105/2048, choosing power of 2 as a scale for performance reason.
	 * */
	int vdd_divided = (adc1_voltage * 8105) / 2048;

	/* Update the ADC attributes so that C2 can possibly begin transmission while we read the I2C sensors. */
	APP_ZIGBEE_update_ADC_outputs(vdd_divided);

	int8_t bme280_status = bme280_setup();

#if USE_AHT20
	HAL_Delay(50);
	aht_status_t aht_status = aht20_setup();
#endif

	if (BME280_OK == bme280_status) {
		bme280_status = bme280_read_measurements(&bme280_meas_data);
		bme280_meas_data.humidity = (bme280_meas_data.humidity * 100 + 512) / 1024;
	}

	if (BME280_OK != bme280_status) {
		bme280_meas_data.temperature = ZCL_TEMP_MEAS_UNKNOWN;
		bme280_meas_data.pressure = ZCL_PRESS_MEAS_UNKNOWN;
		bme280_meas_data.humidity = ZCL_WC_MEAS_UNKNOWN;

	}

#if USE_AHT20
	bme280_meas_data.humidity = ZCL_WC_MEAS_UNKNOWN;
	if (AHT_NO_ERROR == aht_status) {
		HAL_Delay(50);
		aht_status = aht20_read_measurements(&aht20_data);
		if (AHT_NO_ERROR == aht_status) {
			bme280_meas_data.humidity = aht20_data.humidity;
		}

		/* Use AHT temperature measurement if BMP280 data is unavailable. */
		if (ZCL_TEMP_MEAS_UNKNOWN == bme280_meas_data.temperature) {
			bme280_meas_data.temperature = aht20_data.temperature;
		}
	}
#endif

	LL_GPIO_ResetOutputPin(BME280_Supply_GPIO_Port, BME280_Supply_Pin);
	HAL_I2C_DeInit(&hi2c1);

	/* Update the attributes with fresh data*/
	APP_ZIGBEE_update_BME280_outputs(bme280_meas_data.temperature,
		bme280_meas_data.pressure, bme280_meas_data.humidity);

	/* Check Zigbee link health and refresh IWDG if appropriate. */
	zigbee_check_link();
}


static void read_update_sensor_data(void)
{
	/* Queue the sensor readout routine to be run in the main application loop. */
	UTIL_SEQ_SetTask(1<< CFG_TASK_MAIN_LOOP, CFG_SCH_PRIO_0);
}

void startSensorUpdates(void)
{
	if (hwSensorTimerId != CFG_HW_TS_MAX_NBR_CONCURRENT_TIMER) {
//		UTIL_SEQ_SetTask(1<< CFG_TASK_MAIN_LOOP, CFG_SCH_PRIO_0);
		seq_main_loop();
		HW_TS_Start(hwSensorTimerId, HW_TS_SERVER_20S_NB_TICKS);
	}
}

void stopSensorUpdates(void)
{
	if (hwSensorTimerId != CFG_HW_TS_MAX_NBR_CONCURRENT_TIMER) {
		HW_TS_Stop(hwSensorTimerId);
	}
}

/* Setup daily restart */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	HAL_NVIC_SystemReset();
}

#if CFG_DEBUG_TRACE
/* Debugging routine to direct the log output to the USB */
int _write(int file, char *ptr, int len) {

	HAL_StatusTypeDef res = HAL_UART_Transmit(&hlpuart1, (uint8_t *)ptr, len, 100);
	if (HAL_OK != res)
		return 0;
	return len;
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
#define MX_I2C1_Init()

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */
  SysTicks_us = SystemCoreClock / 1000000U;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();
  MX_LPTIM1_Init();
  MX_LPUART1_UART_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */
  ADC_check_print();

#undef MX_I2C1_Init
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  UTIL_SEQ_RegTask(1U << CFG_TASK_MAIN_LOOP, UTIL_SEQ_RFU, seq_main_loop);

  if (hw_ts_Successful != HW_TS_Create(CFG_TIM_ZIGBEE_APP_BME280_REPEAT, &hwSensorTimerId, hw_ts_Repeated, read_update_sensor_data)) {
	  hwSensorTimerId = CFG_HW_TS_MAX_NBR_CONCURRENT_TIMER;
  }
/*
  if (hw_ts_Successful != HW_TS_Create(CFG_TIM_ZIGBEE_APP_BME280_ONESHOT, &hwActiveTimerId, hw_ts_SingleShot, enable_power_saving_mode)) {
	  hwActiveTimerId = CFG_HW_TS_MAX_NBR_CONCURRENT_TIMER;
  }
*/
  while (1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA2   ------> ADC1_IN7
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV8;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
  LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_128, LL_ADC_OVS_SHIFT_RIGHT_7);
  LL_ADC_SetOverSamplingDiscont(ADC1, LL_ADC_OVS_REG_CONT);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Internal Channel
  */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT|LL_ADC_PATH_INTERNAL_TEMPSENSOR);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_TEMPSENSOR);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_7);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */

  adc_num_conversions = ADC_REG_InitStruct.SequencerLength ? ADC_REG_InitStruct.SequencerLength + 1 : 0;
  ADC_check(0);
  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00303D5B;
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
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
  PeriphClkInitStruct.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  LL_LPTIM_SetClockSource(LPTIM1, LL_LPTIM_CLK_SOURCE_INTERNAL);
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV32);
  LL_LPTIM_SetPolarity(LPTIM1, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_IMMEDIATE);
  LL_LPTIM_SetCounterMode(LPTIM1, LL_LPTIM_COUNTER_MODE_INTERNAL);
  LL_LPTIM_TrigSw(LPTIM1);
  LL_LPTIM_SetInput1Src(LPTIM1, LL_LPTIM_INPUT1_SRC_GPIO);
  LL_LPTIM_SetInput2Src(LPTIM1, LL_LPTIM_INPUT2_SRC_GPIO);
  /* USER CODE BEGIN LPTIM1_Init 2 */
  setup_lptimer();
  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 23;
  sAlarm.AlarmTime.Minutes = 59;
  sAlarm.AlarmTime.Seconds = 59;
  sAlarm.AlarmTime.SubSeconds = 59;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_HOURS|RTC_ALARMMASK_MINUTES
                              |RTC_ALARMMASK_SECONDS;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x2000, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
//  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);

  /**/
  LL_GPIO_ResetOutputPin(BME280_Supply_GPIO_Port, BME280_Supply_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BOARDLED_GPIO_Port, BOARDLED_Pin);

  /**/
  GPIO_InitStruct.Pin = BME280_Supply_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BME280_Supply_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BOARDLED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BOARDLED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
struct adc_config_params {
	const char *desc;
	uint32_t values[2];
} adc_config_params[] = {
		{ .desc = "Clock source :", },
		{ .desc = "Clock enabled:", },
		{ .desc = "Pin mode     :", },
		{ .desc = "Common clock :", },
		{ .desc = "Low power    :", },
		{ .desc = "SW Trigger   :", },
		{ .desc = "Power down   :", },
		{ .desc = "Int Regulator:", },
		{ .desc = "Int Channels :", },
		{ .desc = "CR reg       :", },
		{ .desc = "ISR reg      :", },

		{ .desc = "Rank 1 Chan  :", },
		{ .desc = "Rank 1 Time  :", },
		{ .desc = "Rank 1 Diff  :", },
		{ .desc = "Rank 2 Chan  :", },
		{ .desc = "Rank 2 Time  :", },
		{ .desc = "Rank 2 Diff  :", },
		{ .desc = "Rank 3 Chan  :", },
		{ .desc = "Rank 3 Time  :", },
		{ .desc = "Rank 3 Diff  :", },
};


static void ADC_check(int idx)
{
	int q, p = 0;
	adc_config_params[p++].values[idx] = LL_RCC_GetADCClockSource(LL_RCC_ADC_CLKSOURCE);
	adc_config_params[p++].values[idx] = LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_ADC);
	adc_config_params[p++].values[idx] = LL_GPIO_GetPinMode(GPIOA, LL_GPIO_PIN_2);
	adc_config_params[p++].values[idx] = LL_ADC_GetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1));
	adc_config_params[p++].values[idx] = LL_ADC_GetLowPowerMode(ADC1);
	adc_config_params[p++].values[idx] = LL_ADC_REG_IsTriggerSourceSWStart(ADC1);
	adc_config_params[p++].values[idx] = LL_ADC_IsDeepPowerDownEnabled(ADC1);
	adc_config_params[p++].values[idx] = LL_ADC_IsInternalRegulatorEnabled(ADC1);
	adc_config_params[p++].values[idx] = LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1));

	adc_config_params[p++].values[idx] = ADC1->CR;
	adc_config_params[p++].values[idx] = ADC1->ISR;

	adc_config_params[p++].values[idx] = LL_ADC_REG_GetSequencerRanks(ADC1, LL_ADC_REG_RANK_1);
	q = p;
	adc_config_params[p++].values[idx] = LL_ADC_GetChannelSamplingTime(ADC1, adc_config_params[q].values[idx]);
	adc_config_params[p++].values[idx] = LL_ADC_GetChannelSingleDiff(ADC1, adc_config_params[q].values[idx]);

	adc_config_params[p++].values[idx] = LL_ADC_REG_GetSequencerRanks(ADC1, LL_ADC_REG_RANK_2);
	q = p;
	adc_config_params[p++].values[idx] = LL_ADC_GetChannelSamplingTime(ADC1, adc_config_params[q].values[idx]);
	adc_config_params[p++].values[idx] = LL_ADC_GetChannelSingleDiff(ADC1, adc_config_params[q].values[idx]);

	adc_config_params[p++].values[idx] = LL_ADC_REG_GetSequencerRanks(ADC1, LL_ADC_REG_RANK_3);
	q = p;
	adc_config_params[p++].values[idx] = LL_ADC_GetChannelSamplingTime(ADC1, adc_config_params[q].values[idx]);
	adc_config_params[p++].values[idx] = LL_ADC_GetChannelSingleDiff(ADC1, adc_config_params[q].values[idx]);

}

static void ADC_check_print(void)
{
	unsigned n = sizeof(adc_config_params) / sizeof(adc_config_params[0]);
	unsigned n2 = n / 2;
	unsigned i;
	for (i = 0; i < n - n2; i++) {
		unsigned i2 = i + n - n2;
		if (i2 < n) {
			APP_DBG("%s %08x %08x\t%s %08x %08x",
					adc_config_params[i].desc, adc_config_params[i].values[0], adc_config_params[i].values[1],
					adc_config_params[i2].desc, adc_config_params[i2].values[0], adc_config_params[i2].values[1])
		} else {
			APP_DBG("%s %08x %08x", adc_config_params[i].desc, adc_config_params[i].values[0], adc_config_params[i].values[1])
		}
	}
}

#if defined (LPTIM1)
static uint32_t lp_tick = 0;
static uint16_t lp_prev_counter = 0;

/*
 * Count ticks in Run or Stop modes.
 * LPTIM1 is set with 1/32 divider so that it counts milliseconds.
 * To prevent the counter overflow this GetTick function needs to be called
 * at least once every 65.535 seconds.
 * */
uint32_t HAL_GetTick(void)
{
	if (0 == LPTIM1->ARR) {
		lp_tick = uwTick;
	} else {
		uint16_t counter = LL_LPTIM_GetCounter(LPTIM1);
		uint16_t delta = counter - lp_prev_counter;
		lp_tick += (uint32_t)delta;
		lp_prev_counter += delta;
	}
	return lp_tick;
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
	SysTick_Config(HAL_RCC_GetHCLKFreq() / 1000U);
	uwTickFreq = 1;

	if (0 != LPTIM1->ARR) { /* Timer has already been initialized */
		LL_LPTIM_Enable(LPTIM1);
		while (!LL_LPTIM_IsEnabled(LPTIM1)) { ; }
		LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
	} else {
		lp_tick = 0;
	}
	HAL_StatusTypeDef  status = HAL_OK;
	return status;
}

static void setup_lptimer(void)
{
	LL_LPTIM_Enable(LPTIM1);
	while (!LL_LPTIM_IsEnabled(LPTIM1)) { ; }
	LL_LPTIM_SetAutoReload(LPTIM1, ~0);
	LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
}
#endif

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
