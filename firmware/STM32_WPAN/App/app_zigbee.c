
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/app_zigbee.c
  * @author  MCD Application Team
  * @brief   Zigbee Application.
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
#include "app_common.h"
#include "app_entry.h"
#include "dbg_trace.h"
#include "app_zigbee.h"
#include "zigbee_interface.h"
#include "shci.h"
#include "stm_logging.h"
#include "app_conf.h"
#include "stm32wbxx_core_interface_def.h"
#include "zigbee_types.h"
#include "stm32_seq.h"
#include "stm32_lpm.h"

/* Private includes -----------------------------------------------------------*/
#include <assert.h>
#include "zcl/zcl.h"
#include "zcl/general/zcl.press.meas.h"
#include "zcl/general/zcl.temp.meas.h"
#include "zcl/general/zcl.elec.meas.h"
#include "zcl/general/zcl.wcm.h"

/* USER CODE BEGIN Includes */
#include "utilities_conf.h"
#include "zigbee.h"
#include "zcl.basic.h"
#include "zigbee.nwk.h"
#include "main.h"
#include "hw_flash.h"
#include "ee.h"

#include "main.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define APP_ZIGBEE_STARTUP_FAIL_DELAY               500U
#define CHANNEL                                     26
#define ZED_SLEEP_TIME_30S                           1 /* 30s sleep time unit */

#define SW1_ENDPOINT                                1

/* Pressure_meas (endpoint 1) specific defines ------------------------------------------------*/
#define PRESS_MAX_1                      32766
#define PRESS_MIN_1                      -32767
/* USER CODE BEGIN Pressure_meas (endpoint 1) defines */
/* USER CODE END Pressure_meas (endpoint 1) defines */

/* Temperature_meas (endpoint 1) specific defines ------------------------------------------------*/
#define TEMP_MIN_1                      -2000
#define TEMP_MAX_1                      5000
#define TEMP_TOLERANCE_1                      200
/* USER CODE BEGIN Temperature_meas (endpoint 1) defines */
/* USER CODE END Temperature_meas (endpoint 1) defines */

/* Water_content (endpoint 1) specific defines ------------------------------------------------*/
#define HUMIDITY_MIN_1                      1
#define HUMIDITY_MAX_1                      10000
/* USER CODE BEGIN Water_content (endpoint 1) defines */
/* USER CODE END Water_content (endpoint 1) defines */

/* USER CODE BEGIN PD */
#define CFG_NVM                 1U /* use FLASH */
#define RESET_ZB_NWK            0U

#define APP_ZIGBEE_STARTUP_REJOIN_DELAY               5000U

#if USE_FASTPOLL
#define FAST_POLL_TIMEOUT_US (USE_FASTPOLL)*1000*1000 /* USE_FASTPOLL is time in seconds */
#endif

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* * * *
 *   Redefine some of the auto-generated macros here.
 *   This way we can maintain connection to IOC.
 * * * */

/*
 * When active, MCP_ASSOC_CAP_RXONIDLE flag prevents M0 from receiving packers when in idle state.
 * Here we redefine this macro to add MCP_ASSOC_CAP_PWR_SRC to the list of flags that will be removed from the network configuration.
 * When present, this power source flag indicates mains as a source, otherwise (our case) device is powered by a battery.
 * */

const static unsigned saved_value_MCP_ASSOC_CAP_RXONIDLE = MCP_ASSOC_CAP_RXONIDLE;
#undef MCP_ASSOC_CAP_RXONIDLE
#define MCP_ASSOC_CAP_RXONIDLE (saved_value_MCP_ASSOC_CAP_RXONIDLE | MCP_ASSOC_CAP_PWR_SRC)


/*
 * Expand the channel list to all 2.4GHz channels.
 *  */
#undef CHANNEL
#define CHANNEL  1 ? (((1 << (26-11+1))-1) << 11) : 0

/*
 * A weird way to inject some code into APP_ZIGBEE_NwkForm() before ZbStartup() is called.
 * The template implementation of network formation disables Stop mode until network join is successful.
 * This results in high current usage, up to ~15-17mA.
 * In order to reduce power consumption, in this code we enable Stop mode between APP_ZIGBEE_NwkForm() calls.
 * The corresponding call is placed at the end of APP_ZIGBEE_NwkForm().
 * However, ZbStartup() should be called with Stop mode disabled for correct joining to ZHA network.
 * This macro is the only way to inject the corresponding setting into the code before ZbStartup() call.
 */
const static unsigned saved_value_ZED_SLEEP_TIME_30S = ZED_SLEEP_TIME_30S;
#undef ZED_SLEEP_TIME_30S
#define ZED_SLEEP_TIME_30S saved_value_ZED_SLEEP_TIME_30S; \
		UTIL_LPM_SetStopMode(1U << CFG_LPM_APP, UTIL_LPM_DISABLE)

/* USER CODE END PM */

/* External definition -------------------------------------------------------*/
enum ZbStatusCodeT ZbStartupWait(struct ZigBeeT *zb, struct ZbStartupT *config);

/* USER CODE BEGIN ED */
/* USER CODE END ED */

/* Private function prototypes -----------------------------------------------*/
static void APP_ZIGBEE_StackLayersInit(void);
static void APP_ZIGBEE_ConfigEndpoints(void);
static void APP_ZIGBEE_NwkForm(void);

static void APP_ZIGBEE_TraceError(const char *pMess, uint32_t ErrCode);
static void APP_ZIGBEE_CheckWirelessFirmwareInfo(void);

static void Wait_Getting_Ack_From_M0(void);
static void Receive_Ack_From_M0(void);
static void Receive_Notification_From_M0(void);

static void APP_ZIGBEE_ProcessNotifyM0ToM4(void);
static void APP_ZIGBEE_ProcessRequestM0ToM4(void);

/* USER CODE BEGIN PFP */
static void APP_ZIGBEE_setup_filter(void);
static void APP_ZIGBEE_ConfigBasic(void);


static bool APP_ZIGBEE_persist_load(void);
static bool APP_ZIGBEE_persist_save(void);
static void APP_ZIGBEE_persist_delete(void);
static void APP_ZIGBEE_persist_notify_cb(struct ZigBeeT *zb, void *cbarg);
static enum ZbStatusCodeT APP_ZIGBEE_ZbStartupPersist(struct ZigBeeT *zb);
static void APP_ZIGBEE_PersistCompleted_callback(enum ZbStatusCodeT status,void *arg);

#if USE_FASTPOLL
static void stop_fastpoll(void);
static void stop_fastpoll_task(void);
#endif
void start_fastpoll(uint32_t timeout_us);

#if CFG_NVM
static void APP_ZIGBEE_NVM_Init(void);
static bool APP_ZIGBEE_NVM_Read(void);
static bool APP_ZIGBEE_NVM_Write(void);
static void APP_ZIGBEE_NVM_Erase(void);
#endif /* CFG_NVM */

void ZIGBEE_Leave(void);

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
static TL_CmdPacket_t   *p_ZIGBEE_otcmdbuffer;
static TL_EvtPacket_t   *p_ZIGBEE_notif_M0_to_M4;
static TL_EvtPacket_t   *p_ZIGBEE_request_M0_to_M4;
static __IO uint32_t    CptReceiveNotifyFromM0 = 0;
static __IO uint32_t    CptReceiveRequestFromM0 = 0;

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_ZIGBEE_Config_t ZigbeeConfigBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ZigbeeOtCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ZigbeeNotifRspEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ZigbeeNotifRequestBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];

struct zigbee_app_info
{
  bool has_init;
  struct ZigBeeT *zb;
  enum ZbStartType startupControl;
  enum ZbStatusCodeT join_status;
  uint32_t join_delay;
  bool init_after_join;

  struct ZbZclClusterT *pressure_meas_server_1;
  struct ZbZclClusterT *temperature_meas_server_1;
  struct ZbZclClusterT *electricalMeasurement_server_1;
  struct ZbZclClusterT *water_content_server_1;
};
static struct zigbee_app_info zigbee_app_info;

/* ElectricalMeasurement server 1 custom callbacks */
static enum ZclStatusCodeT electricalMeasurement_server_1_get_profile_info(struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *src_info, void *arg);
static enum ZclStatusCodeT electricalMeasurement_server_1_get_meas_profile(struct ZbZclClusterT *cluster, struct ZbZclElecMeasClientGetMeasProfileReqT *cmd_req, struct ZbZclAddrInfoT *src_info, void *arg);

static struct ZbZclElecMeasSvrCallbacksT ElecMeasServerCallbacks_1 =
{
  .get_profile_info = electricalMeasurement_server_1_get_profile_info,
  .get_meas_profile = electricalMeasurement_server_1_get_meas_profile,
};

/* USER CODE BEGIN PV */
static unsigned int persistNumWrites = 0;

/* cache in uninit RAM to store/retrieve persistent data */
union cache
{
  uint8_t  U8_data[ST_PERSIST_MAX_ALLOC_SZ];     // in bytes
  uint32_t U32_data[ST_PERSIST_MAX_ALLOC_SZ/4U]; // in U32 words
};
// __attribute__ ((section(".noinit"))) union cache cache_persistent_data;

// __attribute__ ((section(".noinit"))) union cache cache_diag_reference;

union cache cache_persistent_data;
union cache cache_diag_reference;


uint64_t zb_epanId = 0;

struct ZbMsgFilterT * msg_filter = NULL;

/*
 * ZHA integration in Home Assistant does not support DC measurements, so we will send
 * voltage as AC measurements.
 * Since STM32 ZStack does not provide AC attributes these attributes must be defined here.
*/
static struct ZbZclAttrT elecAttrList[] = {
	{
		ZCL_ELEC_MEAS_ATTR_RMS_VOLT, ZCL_DATATYPE_UNSIGNED_16BIT,
		ZCL_ATTR_FLAG_WRITABLE | ZCL_ATTR_FLAG_PERSISTABLE | ZCL_ATTR_FLAG_REPORTABLE,
		sizeof(uint16_t), NULL, {0, 0}, {0, 0},
	},
	{
			ZCL_ELEC_MEAS_ATTR_AC_VOLT_MULTIPLIER, ZCL_DATATYPE_UNSIGNED_16BIT, 0, sizeof(uint16_t), NULL, {0, 0}, {0, 0},
	},
	{
			ZCL_ELEC_MEAS_ATTR_AC_VOLT_DIVISOR, ZCL_DATATYPE_UNSIGNED_16BIT, 0, sizeof(uint16_t), NULL, {0, 0}, {0, 0},
	},
};

/*
 * Standard attribute accepts integer value in hPa, but BMP/BME280 has a higher accuracy.
 * These extended pressure attributes can transmit more accurate measurement results.
 */
static struct ZbZclAttrT pressureExtAttrList[] = {
	{
		ZCL_PRESS_MEAS_ATTR_SCALED_VAL, ZCL_DATATYPE_UNSIGNED_16BIT,
		ZCL_ATTR_FLAG_WRITABLE | ZCL_ATTR_FLAG_PERSISTABLE | ZCL_ATTR_FLAG_REPORTABLE,
		sizeof(uint16_t), NULL, {0, 0}, {0, 0},
	},
	{
			ZCL_PRESS_MEAS_ATTR_SCALE, ZCL_DATATYPE_UNSIGNED_16BIT, 0, sizeof(uint16_t), NULL, {0, 0}, {0, 0},
	},
	{
			ZCL_PRESS_MEAS_ATTR_MIN_SCALED_VAL, ZCL_DATATYPE_UNSIGNED_16BIT, 0, sizeof(uint16_t), NULL, {0, 0}, {0, 0},
	},
	{
			ZCL_PRESS_MEAS_ATTR_MAX_SCALED_VAL, ZCL_DATATYPE_UNSIGNED_16BIT, 0, sizeof(uint16_t), NULL, {0, 0}, {0, 0},
	},
};

#if USE_FASTPOLL
static struct nwk_fastpoll_entry_t * current_fast_poll = NULL;
static uint8_t hwFastPollTimerId = 0;
#endif
static uint8_t freeze_updates = 0;

/* USER CODE END PV */
/* Functions Definition ------------------------------------------------------*/

/* ElectricalMeasurement server get_profile_info 1 command callback */
static enum ZclStatusCodeT electricalMeasurement_server_1_get_profile_info(struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *src_info, void *arg)
{
  /* USER CODE BEGIN 0 ElectricalMeasurement server 1 get_profile_info 1 */
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 0 ElectricalMeasurement server 1 get_profile_info 1 */
}

/* ElectricalMeasurement server get_meas_profile 1 command callback */
static enum ZclStatusCodeT electricalMeasurement_server_1_get_meas_profile(struct ZbZclClusterT *cluster, struct ZbZclElecMeasClientGetMeasProfileReqT *cmd_req, struct ZbZclAddrInfoT *src_info, void *arg)
{
  /* USER CODE BEGIN 1 ElectricalMeasurement server 1 get_meas_profile 1 */
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 1 ElectricalMeasurement server 1 get_meas_profile 1 */
}

/**
 * @brief  Zigbee application initialization
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_Init(void)
{
  SHCI_CmdStatus_t ZigbeeInitStatus;

  APP_DBG("APP_ZIGBEE_Init");

  /* Check the compatibility with the Coprocessor Wireless Firmware loaded */
  APP_ZIGBEE_CheckWirelessFirmwareInfo();

  /* Register cmdbuffer */
  APP_ZIGBEE_RegisterCmdBuffer(&ZigbeeOtCmdBuffer);

  /* Init config buffer and call TL_ZIGBEE_Init */
  APP_ZIGBEE_TL_INIT();

  /* Register task */
  /* Create the different tasks */
  UTIL_SEQ_RegTask(1U << (uint32_t)CFG_TASK_NOTIFY_FROM_M0_TO_M4, UTIL_SEQ_RFU, APP_ZIGBEE_ProcessNotifyM0ToM4);
  UTIL_SEQ_RegTask(1U << (uint32_t)CFG_TASK_REQUEST_FROM_M0_TO_M4, UTIL_SEQ_RFU, APP_ZIGBEE_ProcessRequestM0ToM4);

  /* Task associated with network creation process */
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, UTIL_SEQ_RFU, APP_ZIGBEE_NwkForm);

  /* USER CODE BEGIN APP_ZIGBEE_INIT */
  /* USER CODE END APP_ZIGBEE_INIT */

  /* Start the Zigbee on the CPU2 side */
  ZigbeeInitStatus = SHCI_C2_ZIGBEE_Init();
  /* Prevent unused argument(s) compilation warning */
  UNUSED(ZigbeeInitStatus);

  /* Initialize Zigbee stack layers */
  APP_ZIGBEE_StackLayersInit();

}

/**
 * @brief  Initialize Zigbee stack layers
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_StackLayersInit(void)
{
  APP_DBG("APP_ZIGBEE_StackLayersInit");

  zigbee_app_info.zb = ZbInit(0U, NULL, NULL);
  assert(zigbee_app_info.zb != NULL);

  /* Create the endpoint and cluster(s) */
  APP_ZIGBEE_ConfigEndpoints();

  /* USER CODE BEGIN APP_ZIGBEE_StackLayersInit */
  zigbee_app_info.join_status = (enum ZbStatusCodeT) 0x01; /* init to error status */
  APP_ZIGBEE_setup_filter();
  startSensorUpdates();
  /* STEP 1 - TRY to START FROM PERSISTENCE */

  /* First we disable the persistent notification */
  ZbPersistNotifyRegister(zigbee_app_info.zb,NULL,NULL);

  /* Increase transmission power (to 3.7dbm). Default setting is 0. */
  ZbNwkIfSetTxPower(zigbee_app_info.zb, "wpan0", 6);

  /* Extend default persistence timeouts. Default value of 10s is too frequent and is unnecessary in this application. */
  uint32_t pers_tm = 0;
  enum ZbStatusCodeT res = ZbBdbGetIndex(zigbee_app_info.zb, ZB_BDB_PersistTimeoutMs, &pers_tm, 4, 0);
  if (ZB_STATUS_SUCCESS == res) {
	  pers_tm *= 10000;
	  ZbBdbSetIndex(zigbee_app_info.zb, ZB_BDB_PersistTimeoutMs, &pers_tm, sizeof(pers_tm), 0);
  }

  /* Remove parent link strength threshold. This application does not transmit frequent data
   * and association happens faster when more nodes are avaialable.  */
  uint32_t bdb_flags = 0;
  res = ZbBdbGetIndex(zigbee_app_info.zb, ZB_BDB_Flags, &bdb_flags, sizeof(bdb_flags), 0);
  if (ZB_STATUS_SUCCESS == res) {
	  bdb_flags |= ZB_BDB_FLAG_IGNORE_COST_DURING_JOIN;
	  ZbBdbGetIndex(zigbee_app_info.zb, ZB_BDB_Flags, &bdb_flags, sizeof(bdb_flags), 0);
  }

  /* Call a startup from persistence */
#if CFG_NVM
#if (RESET_ZB_NWK)
  APP_ZIGBEE_persist_delete();
#endif
#endif

  enum ZbStatusCodeT status;
  status = APP_ZIGBEE_ZbStartupPersist(zigbee_app_info.zb);
  if(status == ZB_STATUS_SUCCESS)
  {
     /* no fresh startup need anymore */
     APP_DBG("ZbStartupPersist: SUCCESS, restarted from persistence");

     ZbNwkGet(zigbee_app_info.zb, ZB_NWK_NIB_ID_ExtendedPanId, &zb_epanId, sizeof(zb_epanId));
     APP_DBG("Network id: 0x%x%x", zb_epanId >> 32, zb_epanId);

     zigbee_app_info.startupControl = ZbStartTypeJoin;
     zigbee_app_info.join_status = ZB_STATUS_SUCCESS;


     UTIL_LPM_SetStopMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);
     return;
  }
  else
  {
       /* Start-up form persistence failed perform a fresh ZbStartup */
       APP_DBG("ZbStartupPersist: FAILED to restart from persistence with status: 0x%02x",status);
  }

  /* USER CODE END APP_ZIGBEE_StackLayersInit */

  /* Configure the joining parameters */
  zigbee_app_info.join_status = (enum ZbStatusCodeT) 0x01; /* init to error status */
  zigbee_app_info.join_delay = HAL_GetTick(); /* now */
  zigbee_app_info.startupControl = ZbStartTypeJoin;

  /* Initialization Complete */
  zigbee_app_info.has_init = true;

  /* run the task */
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
}

/**
 * @brief  Configure Zigbee application endpoints
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ConfigEndpoints(void)
{
  struct ZbApsmeAddEndpointReqT req;
  struct ZbApsmeAddEndpointConfT conf;

  memset(&req, 0, sizeof(req));

  /* Endpoint: SW1_ENDPOINT */
  req.profileId = ZCL_PROFILE_HOME_AUTOMATION;
  req.deviceId = ZCL_DEVICE_SIMPLE_SENSOR;
  req.endpoint = SW1_ENDPOINT;
  ZbZclAddEndpoint(zigbee_app_info.zb, &req, &conf);
  assert(conf.status == ZB_STATUS_SUCCESS);

  /* Pressure meas server */
  zigbee_app_info.pressure_meas_server_1 = ZbZclPressMeasServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT,PRESS_MIN_1,PRESS_MAX_1);
  assert(zigbee_app_info.pressure_meas_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.pressure_meas_server_1);
  /* Temperature meas server */
  zigbee_app_info.temperature_meas_server_1 = ZbZclTempMeasServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, TEMP_MIN_1, TEMP_MAX_1, TEMP_TOLERANCE_1);
  assert(zigbee_app_info.temperature_meas_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.temperature_meas_server_1);
  /* ElectricalMeasurement server */
  zigbee_app_info.electricalMeasurement_server_1 = ZbZclElecMeasServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, &ElecMeasServerCallbacks_1, NULL);
  assert(zigbee_app_info.electricalMeasurement_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.electricalMeasurement_server_1);
  /* Water content server */
  zigbee_app_info.water_content_server_1 = ZbZclWaterContentMeasServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, ZCL_CLUSTER_MEAS_HUMIDITY, HUMIDITY_MIN_1, HUMIDITY_MAX_1);
  assert(zigbee_app_info.water_content_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.water_content_server_1);

  /* USER CODE BEGIN CONFIG_ENDPOINT */
  APP_ZIGBEE_ConfigBasic();

  /* Extended pressure attributes */
  unsigned press_attr_count = ZCL_ATTR_LIST_LEN(pressureExtAttrList);
  enum ZclStatusCodeT result = ZbZclAttrAppendList(zigbee_app_info.pressure_meas_server_1, pressureExtAttrList, press_attr_count);

  if (ZCL_STATUS_SUCCESS == result) {
	  (void)ZbZclAttrIntegerWrite(zigbee_app_info.pressure_meas_server_1, ZCL_PRESS_MEAS_ATTR_SCALE, 2); /* scale = 10^2 */
//	  (void)ZbZclAttrIntegerWrite(zigbee_app_info.pressure_meas_server_1, ZCL_PRESS_MEAS_ATTR_SCALED_VAL, 5000); // 10 x Pa
	  (void)ZbZclAttrIntegerWrite(zigbee_app_info.pressure_meas_server_1, ZCL_PRESS_MEAS_ATTR_MIN_SCALED_VAL, 2000); // 10 x Pa
	  (void)ZbZclAttrIntegerWrite(zigbee_app_info.pressure_meas_server_1, ZCL_PRESS_MEAS_ATTR_MAX_SCALED_VAL, 20000); // 10 x Pa

	  (void)ZbZclAttrReportConfigDefault(zigbee_app_info.pressure_meas_server_1,
			  ZCL_PRESS_MEAS_ATTR_SCALED_VAL, DEFAULT_REPORT_MIN, DEFAULT_REPORT_MAX, NULL);
  }

  /* Basic pressure attributes */
  result = ZbZclAttrIntegerWrite(zigbee_app_info.pressure_meas_server_1, ZCL_PRESS_MEAS_ATTR_MIN_MEAS_VAL, 200); // hPa
  if (ZCL_STATUS_SUCCESS == result) {
//	  (void)ZbZclAttrIntegerWrite(zigbee_app_info.pressure_meas_server_1, ZCL_PRESS_MEAS_ATTR_MEAS_VAL, 500); // hPa
	  (void)ZbZclAttrIntegerWrite(zigbee_app_info.pressure_meas_server_1, ZCL_PRESS_MEAS_ATTR_MAX_MEAS_VAL, 2000); // hPa

	  (void)ZbZclAttrReportConfigDefault(zigbee_app_info.pressure_meas_server_1,
			  ZCL_PRESS_MEAS_ATTR_MEAS_VAL, DEFAULT_REPORT_MIN, DEFAULT_REPORT_MAX, NULL);
  }

  /* Electrical measurements attributes */
  unsigned elec_attr_count = ZCL_ATTR_LIST_LEN(elecAttrList);
  result = ZbZclAttrAppendList(zigbee_app_info.electricalMeasurement_server_1, elecAttrList, elec_attr_count);

  if (ZCL_STATUS_SUCCESS == result) {
	  (void)ZbZclAttrIntegerWrite(zigbee_app_info.electricalMeasurement_server_1, ZCL_ELEC_MEAS_ATTR_MEAS_TYPE, (1<<0) /* AC measurement */);
	  (void)ZbZclAttrIntegerWrite(zigbee_app_info.electricalMeasurement_server_1, ZCL_ELEC_MEAS_ATTR_AC_VOLT_MULTIPLIER, 1);
	  (void)ZbZclAttrIntegerWrite(zigbee_app_info.electricalMeasurement_server_1, ZCL_ELEC_MEAS_ATTR_AC_VOLT_DIVISOR, 1000);
//	  (void)ZbZclAttrIntegerWrite(zigbee_app_info.electricalMeasurement_server_1, ZCL_ELEC_MEAS_ATTR_RMS_VOLT, 0);
	  (void)ZbZclAttrReportConfigDefault(zigbee_app_info.electricalMeasurement_server_1,
			  ZCL_ELEC_MEAS_ATTR_RMS_VOLT, DEFAULT_REPORT_MIN, DEFAULT_REPORT_MAX, NULL);
  }

  /* Temperature reporting */
  (void)ZbZclAttrReportConfigDefault(zigbee_app_info.temperature_meas_server_1,
		  ZCL_TEMP_MEAS_ATTR_MEAS_VAL, DEFAULT_REPORT_MIN, DEFAULT_REPORT_MAX, NULL);

  /* Humidity reporting */
  (void)ZbZclAttrReportConfigDefault(zigbee_app_info.water_content_server_1,
		  ZCL_WC_MEAS_ATTR_MEAS_VAL, DEFAULT_REPORT_MIN, DEFAULT_REPORT_MAX, NULL);
  /* USER CODE END CONFIG_ENDPOINT */
}

/**
 * @brief  Handle Zigbee network forming and joining
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_NwkForm(void)
{
  if ((zigbee_app_info.join_status != ZB_STATUS_SUCCESS) && (HAL_GetTick() >= zigbee_app_info.join_delay))
  {
    struct ZbStartupT config;
    enum ZbStatusCodeT status;

    /* Configure Zigbee Logging */
    ZbSetLogging(zigbee_app_info.zb, ZB_LOG_MASK_LEVEL_5, NULL);

    /* Attempt to join a zigbee network */
    ZbStartupConfigGetProDefaults(&config);

    /* Set the centralized network */
    APP_DBG("Network config : APP_STARTUP_CENTRALIZED_END_DEVICE");
    config.startupControl = zigbee_app_info.startupControl;

    /* Using the default HA preconfigured Link Key */
    memcpy(config.security.preconfiguredLinkKey, sec_key_ha, ZB_SEC_KEYSIZE);

    config.channelList.count = 1;
    config.channelList.list[0].page = 0;
    config.channelList.list[0].channelMask = 1 << CHANNEL; /*Channel in use */

    /* Add End device configuration */
    config.capability &= ~(MCP_ASSOC_CAP_RXONIDLE | MCP_ASSOC_CAP_DEV_TYPE | MCP_ASSOC_CAP_ALT_COORD);
    config.endDeviceTimeout=ZED_SLEEP_TIME_30S;

    /* Using ZbStartupWait (blocking) */
    status = ZbStartupWait(zigbee_app_info.zb, &config);

    APP_DBG("ZbStartup Callback (status = 0x%02x)", status);
    zigbee_app_info.join_status = status;

    if (status == ZB_STATUS_SUCCESS)
    {
      /* Enabling Stop mode */
      UTIL_LPM_SetStopMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);
      UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_DISABLE);

      zigbee_app_info.join_delay = 0U;
      zigbee_app_info.init_after_join = true;
      APP_DBG("Startup done !\n");
      /* USER CODE BEGIN 2 */

#if USE_FASTPOLL
      start_fastpoll(FAST_POLL_TIMEOUT_US);
      UTIL_LPM_SetStopMode(1U << CFG_LPM_FASTPOLL, UTIL_LPM_DISABLE);
#endif
      ZbNwkGet(zigbee_app_info.zb, ZB_NWK_NIB_ID_ExtendedPanId, &zb_epanId, sizeof(zb_epanId));
      APP_DBG("Network id: 0x%x%x", zb_epanId >> 32, zb_epanId);

#if CFG_NVM
      /* Register Persistent data change notification */
        ZbPersistNotifyRegister(zigbee_app_info.zb,APP_ZIGBEE_persist_notify_cb,NULL);
      /* Call the callback once here to save persistence data */
        APP_ZIGBEE_persist_notify_cb(zigbee_app_info.zb,NULL);
#endif

      /* USER CODE END 2 */
    }
    else
    {
      APP_DBG("Startup failed, attempting again after a short delay (%d ms)", APP_ZIGBEE_STARTUP_FAIL_DELAY);
      zigbee_app_info.join_delay = HAL_GetTick() + APP_ZIGBEE_STARTUP_FAIL_DELAY;
      /* USER CODE BEGIN 3 */
      zb_epanId = 0;
      if (status != ZB_NWK_STATUS_NO_NETWORKS) {
    	  zigbee_app_info.join_delay = HAL_GetTick() + APP_ZIGBEE_STARTUP_REJOIN_DELAY;
      }
      if (status == ZB_NWK_STATUS_INVALID_REQUEST || ZB_APS_STATUS_SECURITY_FAIL == status) {
    	  /* Some router devices return these error codes during first join and M0 firmware becomes
    	   * unable to join after that. It keeps sending same data and receives same error code
    	   * over and over again.
    	   * Restarting the joining process with ZbReset() seems to help most of the time. */
    	  ZbReset(zigbee_app_info.zb);
      }
      /* USER CODE END 3 */
    }
  }

  /* If Network forming/joining was not successful reschedule the current task to retry the process */
  if (zigbee_app_info.join_status != ZB_STATUS_SUCCESS)
  {
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
  }
  /* USER CODE BEGIN NW_FORM */

  UTIL_LPM_SetStopMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);
  /* USER CODE END NW_FORM */
}

/*************************************************************
 * ZbStartupWait Blocking Call
 *************************************************************/
struct ZbStartupWaitInfo
{
  bool active;
  enum ZbStatusCodeT status;
};

static void ZbStartupWaitCb(enum ZbStatusCodeT status, void *cb_arg)
{
  struct ZbStartupWaitInfo *info = cb_arg;

  info->status = status;
  info->active = false;
  UTIL_SEQ_SetEvt(EVENT_ZIGBEE_STARTUP_ENDED);
}

enum ZbStatusCodeT ZbStartupWait(struct ZigBeeT *zb, struct ZbStartupT *config)
{
  struct ZbStartupWaitInfo *info;
  enum ZbStatusCodeT status;

  info = malloc(sizeof(struct ZbStartupWaitInfo));
  if (info == NULL)
  {
    return ZB_STATUS_ALLOC_FAIL;
  }
  memset(info, 0, sizeof(struct ZbStartupWaitInfo));

  info->active = true;
  status = ZbStartup(zb, config, ZbStartupWaitCb, info);
  if (status != ZB_STATUS_SUCCESS)
  {
    free(info);
    return status;
  }

  UTIL_SEQ_WaitEvt(EVENT_ZIGBEE_STARTUP_ENDED);
  status = info->status;
  free(info);
  return status;
}

/**
 * @brief  Trace the error or the warning reported.
 * @param  ErrId :
 * @param  ErrCode
 * @retval None
 */
void APP_ZIGBEE_Error(uint32_t ErrId, uint32_t ErrCode)
{
  switch (ErrId)
  {
    default:
      APP_ZIGBEE_TraceError("ERROR Unknown ", 0);
      break;
  }
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief  Warn the user that an error has occurred.
 *
 * @param  pMess  : Message associated to the error.
 * @param  ErrCode: Error code associated to the module (Zigbee or other module if any)
 * @retval None
 */
static void APP_ZIGBEE_TraceError(const char *pMess, uint32_t ErrCode)
{
  APP_DBG("**** Fatal error = %s (Err = %d)", pMess, ErrCode);
  /* USER CODE BEGIN TRACE_ERROR */
  /* USER CODE END TRACE_ERROR */

}

/**
 * @brief Check if the Coprocessor Wireless Firmware loaded supports Zigbee
 *        and display associated information
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_CheckWirelessFirmwareInfo(void)
{
  WirelessFwInfo_t wireless_info_instance;
  WirelessFwInfo_t *p_wireless_info = &wireless_info_instance;

  if (SHCI_GetWirelessFwInfo(p_wireless_info) != SHCI_Success)
  {
    APP_ZIGBEE_Error((uint32_t)ERR_ZIGBEE_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
  }
  else
  {
    APP_DBG("**********************************************************");
    APP_DBG("WIRELESS COPROCESSOR FW:");
    /* Print version */
    APP_DBG("VERSION ID = %d.%d.%d", p_wireless_info->VersionMajor, p_wireless_info->VersionMinor, p_wireless_info->VersionSub);

    switch (p_wireless_info->StackType)
    {
      case INFO_STACK_TYPE_ZIGBEE_FFD:
        APP_DBG("FW Type : FFD Zigbee stack");
        break;

      case INFO_STACK_TYPE_ZIGBEE_RFD:
        APP_DBG("FW Type : RFD Zigbee stack");
        break;

      default:
        /* No Zigbee device supported ! */
        APP_ZIGBEE_Error((uint32_t)ERR_ZIGBEE_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
        break;
    }

    /* print the application name */
    char *__PathProject__ = (strstr(__FILE__, "Zigbee") ? strstr(__FILE__, "Zigbee") + 7 : __FILE__);
    char *pdel = NULL;
    if((strchr(__FILE__, '/')) == NULL)
    {
      pdel = strchr(__PathProject__, '\\');
    }
    else
    {
      pdel = strchr(__PathProject__, '/');
    }

    int index = (int)(pdel - __PathProject__);
    APP_DBG("Application flashed: %*.*s", index, index, __PathProject__);

    /* print channel */
    APP_DBG("Channel used: %d", CHANNEL);
    /* print Link Key */
    APP_DBG("Link Key: %.16s", sec_key_ha);
    /* print Link Key value hex */
    char Z09_LL_string[ZB_SEC_KEYSIZE*3+1];
    Z09_LL_string[0] = 0;
    for (int str_index = 0; str_index < ZB_SEC_KEYSIZE; str_index++)
    {
      sprintf(&Z09_LL_string[str_index*3], "%02x ", sec_key_ha[str_index]);
    }

    APP_DBG("Link Key value: %s", Z09_LL_string);
    /* print clusters allocated */
    APP_DBG("Clusters allocated are:");
    APP_DBG("pressure_meas Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("temperature_meas Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("electricalMeasurement Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("water_content Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("**********************************************************");
  }
}

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/

void APP_ZIGBEE_RegisterCmdBuffer(TL_CmdPacket_t *p_buffer)
{
  p_ZIGBEE_otcmdbuffer = p_buffer;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_OTCmdPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)p_ZIGBEE_otcmdbuffer->cmdserial.cmd.payload;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_OTCmdRspPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)((TL_EvtPacket_t *)p_ZIGBEE_otcmdbuffer)->evtserial.evt.payload;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_NotificationPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)(p_ZIGBEE_notif_M0_to_M4)->evtserial.evt.payload;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_M0RequestPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)(p_ZIGBEE_request_M0_to_M4)->evtserial.evt.payload;
}

/**
 * @brief  This function is used to transfer the commands from the M4 to the M0.
 *
 * @param   None
 * @return  None
 */
void ZIGBEE_CmdTransfer(void)
{
  Zigbee_Cmd_Request_t *cmd_req = (Zigbee_Cmd_Request_t *)p_ZIGBEE_otcmdbuffer->cmdserial.cmd.payload;

  /* Zigbee OT command cmdcode range 0x280 .. 0x3DF = 352 */
  p_ZIGBEE_otcmdbuffer->cmdserial.cmd.cmdcode = 0x280U;
  /* Size = otCmdBuffer->Size (Number of OT cmd arguments : 1 arg = 32bits so multiply by 4 to get size in bytes)
   * + ID (4 bytes) + Size (4 bytes) */
  p_ZIGBEE_otcmdbuffer->cmdserial.cmd.plen = 8U + (cmd_req->Size * 4U);

  TL_ZIGBEE_SendM4RequestToM0();

  /* Wait completion of cmd */
  Wait_Getting_Ack_From_M0();
}

/**
 * @brief  This function is called when the M0+ acknowledge the fact that it has received a Cmd
 *
 *
 * @param   Otbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_ZIGBEE_CmdEvtReceived(TL_EvtPacket_t *Otbuffer)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Otbuffer);

  Receive_Ack_From_M0();
}

/**
 * @brief  This function is called when notification from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_ZIGBEE_NotReceived(TL_EvtPacket_t *Notbuffer)
{
  p_ZIGBEE_notif_M0_to_M4 = Notbuffer;

  Receive_Notification_From_M0();
}

/**
 * @brief  This function is called before sending any ot command to the M0
 *         core. The purpose of this function is to be able to check if
 *         there are no notifications coming from the M0 core which are
 *         pending before sending a new ot command.
 * @param  None
 * @retval None
 */
void Pre_ZigbeeCmdProcessing(void)
{
  UTIL_SEQ_WaitEvt(EVENT_SYNCHRO_BYPASS_IDLE);
}

/**
 * @brief  This function waits for getting an acknowledgment from the M0.
 *
 * @param  None
 * @retval None
 */
static void Wait_Getting_Ack_From_M0(void)
{
  UTIL_SEQ_WaitEvt(EVENT_ACK_FROM_M0_EVT);
}

/**
 * @brief  Receive an acknowledgment from the M0+ core.
 *         Each command send by the M4 to the M0 are acknowledged.
 *         This function is called under interrupt.
 * @param  None
 * @retval None
 */
static void Receive_Ack_From_M0(void)
{
  UTIL_SEQ_SetEvt(EVENT_ACK_FROM_M0_EVT);
}

/**
 * @brief  Receive a notification from the M0+ through the IPCC.
 *         This function is called under interrupt.
 * @param  None
 * @retval None
 */
static void Receive_Notification_From_M0(void)
{
  CptReceiveNotifyFromM0++;
  UTIL_SEQ_SetTask(1U << (uint32_t)CFG_TASK_NOTIFY_FROM_M0_TO_M4, CFG_SCH_PRIO_0);
}

/**
 * @brief  This function is called when a request from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_ZIGBEE_M0RequestReceived(TL_EvtPacket_t *Reqbuffer)
{
  p_ZIGBEE_request_M0_to_M4 = Reqbuffer;

  CptReceiveRequestFromM0++;
  UTIL_SEQ_SetTask(1U << (uint32_t)CFG_TASK_REQUEST_FROM_M0_TO_M4, CFG_SCH_PRIO_0);
}

/**
 * @brief Perform initialization of TL for Zigbee.
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_TL_INIT(void)
{
  ZigbeeConfigBuffer.p_ZigbeeOtCmdRspBuffer = (uint8_t *)&ZigbeeOtCmdBuffer;
  ZigbeeConfigBuffer.p_ZigbeeNotAckBuffer = (uint8_t *)ZigbeeNotifRspEvtBuffer;
  ZigbeeConfigBuffer.p_ZigbeeNotifRequestBuffer = (uint8_t *)ZigbeeNotifRequestBuffer;
  TL_ZIGBEE_Init(&ZigbeeConfigBuffer);
}

/**
 * @brief Process the messages coming from the M0.
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ProcessNotifyM0ToM4(void)
{
  if (CptReceiveNotifyFromM0 != 0)
  {
    /* Reset counter */
    CptReceiveNotifyFromM0 = 0;
    Zigbee_CallBackProcessing();
  }
}

/**
 * @brief Process the requests coming from the M0.
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ProcessRequestM0ToM4(void)
{
  if (CptReceiveRequestFromM0 != 0)
  {
    CptReceiveRequestFromM0 = 0;
    Zigbee_M0RequestProcessing();
  }
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */
void Post_ZigbeeCmdProcessing(void)
{
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
}

void APP_ZIGBEE_update_BME280_outputs(int32_t temp, uint32_t press, uint32_t hum)
{
	if (freeze_updates)
		return;

	if (ZCL_TEMP_MEAS_UNKNOWN != temp) {
		(void)ZbZclAttrIntegerWrite(zigbee_app_info.temperature_meas_server_1, ZCL_TEMP_MEAS_ATTR_MEAS_VAL, temp);
#if REPORT_KICK
		ZbZclAttrReportKick(zigbee_app_info.temperature_meas_server_1, 1, NULL, NULL);
#endif
	}

	if (ZCL_PRESS_MEAS_UNKNOWN != press) {
/* Basic accuracy */
		(void)ZbZclAttrIntegerWrite(zigbee_app_info.pressure_meas_server_1, ZCL_PRESS_MEAS_ATTR_MEAS_VAL, (press + 50) / 100);
/* Higher accuracy in the extended attribute */
		(void)ZbZclAttrIntegerWrite(zigbee_app_info.pressure_meas_server_1, ZCL_PRESS_MEAS_ATTR_SCALED_VAL, (press + 5) / 10);

#if REPORT_KICK
		ZbZclAttrReportKick(zigbee_app_info.pressure_meas_server_1, 1, NULL, NULL);
#endif
	}

	if (ZCL_WC_MEAS_UNKNOWN != hum) {
		(void)ZbZclAttrIntegerWrite(zigbee_app_info.water_content_server_1, ZCL_WC_MEAS_ATTR_MEAS_VAL, hum);
#if REPORT_KICK
		ZbZclAttrReportKick(zigbee_app_info.water_content_server_1, 1, NULL, NULL);
#endif
	}
	APP_DBG("Temp: %d Press: %u Hum: %u", temp, press, hum);
}

void APP_ZIGBEE_update_ADC_outputs(uint32_t voltage)
{
	if (freeze_updates)
		return;

	(void)ZbZclAttrIntegerWrite(zigbee_app_info.electricalMeasurement_server_1, ZCL_ELEC_MEAS_ATTR_RMS_VOLT, voltage);
#if REPORT_KICK
	ZbZclAttrReportKick(zigbee_app_info.electricalMeasurement_server_1, 1, NULL, NULL);
#endif
	APP_DBG("V: %u", voltage);
}



/*
 * Periodic check for link health. This function is called every time sensor measurements are made
 * and increments the data counter. This counter is reset in the filter callback when data-related filter is called.
 * As long as the counter is reset the watchdog is refreshed. If link is lost and Zigbee data exchange does
 * not happen for some extended period of time the system will be restarted.
 * This mechanism is here because maximum IWDG interval is ~32s, which is too short.
 * This function extends watchdog period to a reasonably long time of 20 x SENSOR_INTERVAL, which should be
 * sufficient for most cases.
 * */
static uint32_t zb_data_filter_counter = 0;
void zigbee_check_link(void)
{
	if (zb_data_filter_counter < 20) {
		HAL_IWDG_Refresh(&hiwdg);
	}

	zb_data_filter_counter += 1;
}


#if USE_FASTPOLL
static void stop_fastpoll(void)
{
	UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_STOP_FASTPOLL, CFG_SCH_PRIO_0);
}

static void stop_fastpoll_task(void)
{
	int in_fast_poll = NULL != current_fast_poll;

	if (!in_fast_poll)
		return;

    ZbNwkFastPollRelease(zigbee_app_info.zb, current_fast_poll);

	current_fast_poll = NULL;

	UTIL_LPM_SetStopMode(1U << CFG_LPM_FASTPOLL, UTIL_LPM_ENABLE);
}
#endif

/* Timeout in us */
void start_fastpoll(uint32_t timeout_us)
{
#if USE_FASTPOLL

	if (NULL != current_fast_poll)
		stop_fastpoll();

    current_fast_poll = ZbNwkFastPollRequest(zigbee_app_info.zb, 0, DIVR(timeout_us, 1000));

	uint32_t timeout_ticks = DIVR(timeout_us, CFG_TS_TICK_VAL);
	if (hwFastPollTimerId != CFG_HW_TS_MAX_NBR_CONCURRENT_TIMER) {
    	HW_TS_Start(hwFastPollTimerId, timeout_ticks);
    }
    UTIL_LPM_SetStopMode(1U << CFG_LPM_FASTPOLL, UTIL_LPM_DISABLE);

#endif
}

/* callback function */
static enum zb_msg_filter_rc
app_msg_filter_callback(struct ZigBeeT *zb, uint32_t id, void *msg, void *arg)
{
	    enum zb_msg_filter_rc res = ZB_MSG_CONTINUE;

	    if (0 != (id & (ZB_MSG_FILTER_LEAVE_IND | ZB_MSG_FILTER_FACTORY_RESET | ZB_MSG_FILTER_RESET_REPORTS))) {

			if (0 != (id & ZB_MSG_FILTER_LEAVE_IND)) {
				struct ZbNlmeLeaveIndT *ind = msg;
				APP_DBG("Leave. Re-join: %d", ind->rejoin);
				if (ind->rejoin) {
					zb_data_filter_counter = 0;
				} else {
				/* Update status and erase persistence data if permanent leave is requested. */
					ZIGBEE_Leave();
					zigbee_app_info.join_status = (enum ZbStatusCodeT) 0x04; /* init to error status */
					zigbee_app_info.join_delay = HAL_GetTick() + APP_ZIGBEE_STARTUP_REJOIN_DELAY;
					UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
				}
			}
		}

		if (0 != (id & (
				//ZB_MSG_FILTER_MCPS_DATA_IND |
				ZB_MSG_FILTER_NLDE_DATA_IND   |
				ZB_MSG_FILTER_APSDE_DATA_IND  |
				ZB_MSG_FILTER_STARTUP_IND     |
				ZB_MSG_FILTER_STARTUP_IND
			))) {
			zb_data_filter_counter = 0;
			APP_DBG("Data: 0x%x", id);
		}

		/* M0 seems to become stuck if an attribute is updated immediately after the key exchange.
		 * This flag prevents attributes updates until after the following APS data exchange happens.
		 *  */
		if (id & ZB_MSG_FILTER_CONFIRM_KEY_IND) {
			freeze_updates = 1;
		}
		if (id & ZB_MSG_FILTER_APSDE_DATA_IND) {
			freeze_updates = 0;
		}

		return res;
}

static void APP_ZIGBEE_setup_filter(void)
{
	struct ZigBeeT *zb = zigbee_app_info.zb;
	msg_filter = ZbMsgFilterRegister(zb, 0xFFFF,
			ZB_MSG_DEFAULT_PRIO, app_msg_filter_callback, NULL);
}


/**
 * @brief  Setup basic cluster attributes.
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ConfigBasic(void)
{
 	uint8_t mfr_str[] = MANUFACTURER_NAME;
 	ZbZclBasicWriteDirect(zigbee_app_info.zb, SW1_ENDPOINT,
 			ZCL_BASIC_ATTR_MFR_NAME, mfr_str, sizeof(mfr_str) - 1);

 	uint8_t model_str[] = MODEL_NAME;
 	ZbZclBasicWriteDirect(zigbee_app_info.zb, SW1_ENDPOINT,
 			ZCL_BASIC_ATTR_MODEL_NAME, model_str, sizeof(model_str) - 1);

 	uint8_t location_str[] = LOCATION;
 	ZbZclBasicWriteDirect(zigbee_app_info.zb, SW1_ENDPOINT,
 			ZCL_BASIC_ATTR_LOCATION, location_str, sizeof(location_str) - 1);

 	uint8_t power_source = 0x03; // battery
 	ZbZclBasicWriteDirect(zigbee_app_info.zb, SW1_ENDPOINT,
 			ZCL_BASIC_ATTR_POWER_SOURCE, &power_source, 1);

 	uint8_t physical_env = 0x0c; // deck
 	ZbZclBasicWriteDirect(zigbee_app_info.zb, SW1_ENDPOINT,
 			ZCL_BASIC_ATTR_ENVIRONMENT, &physical_env, 1);
}


/*************************************************************
 *
 * NVM FUNCTIONS
 *
 *************************************************************/

/**
 * @brief  notify to save persistent data callback
 * @param  zb: Zigbee device object pointer, cbarg: callback arg pointer
 * @retval None
 */
static void APP_ZIGBEE_persist_notify_cb(struct ZigBeeT *zb, void *cbarg)
{
  APP_DBG("Notification to save persistent data requested from stack");
  /* Save the persistent data */
  APP_ZIGBEE_persist_save();
}

/**
 * @brief  Start Zigbee Network from persistent data
 * @param  zb: Zigbee device object pointer
 * @retval Zigbee stack Status code
 */
static enum ZbStatusCodeT APP_ZIGBEE_ZbStartupPersist(struct ZigBeeT* zb)
{
   bool read_status;
   enum ZbStatusCodeT status = ZB_STATUS_SUCCESS;

   /* Restore persistence */
   read_status = APP_ZIGBEE_persist_load();

   if (read_status)
   {
       /* Make sure the EPID is cleared, before we are allowed to restore persistence */
       uint64_t epid = 0U;
       ZbNwkSet(zb, ZB_NWK_NIB_ID_ExtendedPanId, &epid, sizeof(uint64_t));

       /* Start-up from persistence */
       APP_DBG("APP_ZIGBEE_ZbStartupPersist: restoring stack persistence");
       status = ZbStartupPersist(zb, &cache_persistent_data.U8_data[4], cache_persistent_data.U32_data[0],NULL,APP_ZIGBEE_PersistCompleted_callback,NULL);
   }
   else
   {
       /* Failed to restart from persistence */
       APP_DBG("APP_ZIGBEE_ZbStartupPersist: no persistence data to restore");
       status = ZB_STATUS_ALLOC_FAIL;
   }

   /* Only for debug purpose, depending of persistent data, following traces
      could display bytes that are irrelevants to on off cluster */
   if(status == ZB_STATUS_SUCCESS)
   {
     /* read the last bytes of data where the ZCL on off persistent data shall be*/
      uint32_t len = cache_persistent_data.U32_data[0] + 4 ;
      APP_DBG("ClusterID %02x %02x",cache_persistent_data.U8_data[len-9],cache_persistent_data.U8_data[len-10]);
      APP_DBG("Endpoint %02x %02x",cache_persistent_data.U8_data[len-7],cache_persistent_data.U8_data[len-8]);
      APP_DBG("Direction %02x",cache_persistent_data.U8_data[len-6]);
      APP_DBG("AttrID %02x %02x",cache_persistent_data.U8_data[len-4],cache_persistent_data.U8_data[len-5]);
      APP_DBG("Len %02x %02x",cache_persistent_data.U8_data[len-2],cache_persistent_data.U8_data[len-3]);
      APP_DBG("Value %02x",cache_persistent_data.U8_data[len-1]);
   }

   return status;
}/* APP_ZIGBEE_ZbStartupPersist */

/**
 * @brief  timer callback to wait end of restore cluster persistence form M0
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_PersistCompleted_callback(enum ZbStatusCodeT status,void *arg)
{
   if(status == ZB_WPAN_STATUS_SUCCESS)
   {
	   APP_DBG("Persist complete callback entered with SUCCESS");
   }
   else
   {
	   APP_DBG("Error in persist complete callback %x",status);
   }
   /* STEP3 - Activate back the persistent notification */
     /* Register Persistent data change notification */
     ZbPersistNotifyRegister(zigbee_app_info.zb,APP_ZIGBEE_persist_notify_cb,NULL);

     /* Call the callback once here to save persistence data */
     APP_ZIGBEE_persist_notify_cb(zigbee_app_info.zb,NULL);
}/* APP_ZIGBEE_PersistCompleted_callback */


/**
 * @brief  Load persistent data
 * @param  None
 * @retval true if success, false if fail
 */
static bool APP_ZIGBEE_persist_load(void)
{
#if CFG_NVM
    APP_DBG("Retrieving persistent data from FLASH");
    bool status = APP_ZIGBEE_NVM_Read();

    return status;
#else
    /* Check length range */
    if ((cache_persistent_data.U32_data[0] == 0) ||
        (cache_persistent_data.U32_data[0] > ST_PERSIST_MAX_ALLOC_SZ))
    {
        APP_DBG("No data or too large length : %d",cache_persistent_data.U32_data[0]);
        return false;
    }
    return true;
#endif /* CFG_NVM */
} /* APP_ZIGBEE_persist_load */

/**
 * @brief  Save persistent data
 * @param  None
 * @retval true if success , false if fail
 */
static bool APP_ZIGBEE_persist_save(void)
{
    uint32_t len;

    /* Clear the RAM cache before saving */
    memset(cache_persistent_data.U8_data, 0x00, ST_PERSIST_MAX_ALLOC_SZ);

    /* Call the stack API to get current persistent data */
    len = ZbPersistGet(zigbee_app_info.zb, 0, 0);
    /* Check Length range */
    if (len == 0U)
    {
        /* If the persistence length was zero then no data available. */
        APP_DBG("APP_ZIGBEE_persist_save: no persistence data to save !");
        return false;
    }
    if (len > ST_PERSIST_MAX_ALLOC_SZ)
    {
        /* if persistence length too big to store */
        APP_DBG("APP_ZIGBEE_persist_save: persist size too large for storage (%d)", len);
        return false;
    }

    /* Store in cache the persistent data */
    len = ZbPersistGet(zigbee_app_info.zb, &cache_persistent_data.U8_data[ST_PERSIST_FLASH_DATA_OFFSET], len);

    /* Store in cache the persistent data length */
    cache_persistent_data.U32_data[0] = len;

    persistNumWrites++;
    APP_DBG("APP_ZIGBEE_persist_save: Persistence written in cache RAM (num writes = %d) len=%d",
             persistNumWrites, cache_persistent_data.U32_data[0]);

#if CFG_NVM
    if(!APP_ZIGBEE_NVM_Write())
    {
    	return false;
    }

    APP_DBG("APP_ZIGBEE_persist_save: Persistent data FLASHED");
#endif /* CFG_NVM */

    return true;
} /* APP_ZIGBEE_persist_save */

/**
 * @brief  Delete persistent data
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_persist_delete(void)
{
  /* Clear RAM cache */
   memset(cache_persistent_data.U8_data, 0x00, ST_PERSIST_MAX_ALLOC_SZ);
   APP_DBG("Persistent Data RAM cache cleared");
#if CFG_NVM
   APP_DBG("FLASH ERASED");
   APP_ZIGBEE_NVM_Erase();
#endif /* CFG_NVM */
   persistNumWrites = 0;
} /* APP_ZIGBEE_persist_delete */


#if CFG_NVM
/**
 * @brief  Init the NVM
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_NVM_Init(void)
{
  int eeprom_init_status;

  APP_DBG("Flash starting address = %x",HW_FLASH_ADDRESS  + CFG_NVM_BASE_ADDRESS);
  eeprom_init_status = EE_Init( 0 , HW_FLASH_ADDRESS + CFG_NVM_BASE_ADDRESS );

  if(eeprom_init_status != EE_OK)
  {
    /* format NVM since init failed */
    eeprom_init_status= EE_Init( 1, HW_FLASH_ADDRESS + CFG_NVM_BASE_ADDRESS );
  }
  APP_DBG("EE_init status = %d",eeprom_init_status);

} /* APP_ZIGBEE_NVM_Init */

/**
*@brief  Read the persistent data from NVM
* @param  None
* @retval true if success , false if failed
*/
static bool APP_ZIGBEE_NVM_Read(void)
{
    uint16_t num_words = 0;
    bool status = true;
    int ee_status = 0;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR | FLASH_FLAG_OPTVERR);

    /* Read the data length from cache */
    ee_status = EE_Read(0, ZIGBEE_DB_START_ADDR, &cache_persistent_data.U32_data[0]);
    if (ee_status != EE_OK)
    {
        APP_DBG("Read -> persistent data length not found ERASE to be done - Read Stopped");
        status = false;
    }
      /* Check length is not too big nor zero */
    else if((cache_persistent_data.U32_data[0] == 0) ||
            (cache_persistent_data.U32_data[0]> ST_PERSIST_MAX_ALLOC_SZ))
    {
            APP_DBG("No data or too large length : %d", cache_persistent_data.U32_data[0]);
            status = false;
    }
        /* Length is within range */
    else
    {
           /* Adjust the length to be U32 aligned */
            num_words = (uint16_t) (cache_persistent_data.U32_data[0]/4) ;
            if (cache_persistent_data.U32_data[0] % 4 != 0)
            {
                num_words++;
            }

            /* copy the read data from Flash to cache including length */
            for (uint16_t local_length = 1; local_length <= num_words; local_length++)
            {
            	if (local_length >= ST_PERSIST_MAX_ALLOC_SZ/4)
            	{
                    APP_DBG("Local length exceeds the size of the cache persistent data!");
                    status = false;
                    break;
            	}

                /* read data from first data in U32 unit */
                ee_status = EE_Read(0, local_length + ZIGBEE_DB_START_ADDR, &cache_persistent_data.U32_data[local_length] );
                if (ee_status != EE_OK)
                {
                    APP_DBG("Read not found leaving");
                    status = false;
                    break;
                }
            }
    }

    HAL_FLASH_Lock();
    if(status)
    {
        APP_DBG("READ PERSISTENT DATA LEN = %d",cache_persistent_data.U32_data[0]);
    }
    return status;
} /* APP_ZIGBEE_NVM_Read */

/**
 * @brief  Write the persistent data in NVM
 * @param  None
 * @retval None
 */
static bool APP_ZIGBEE_NVM_Write(void)
{
    int ee_status = 0;

    uint16_t num_words;
    uint16_t local_current_size;


    num_words = 1U; /* 1 words for the length */
    num_words+= (uint16_t) (cache_persistent_data.U32_data[0]/4);


    /* Adjust the length to be U32 aligned */
    if (cache_persistent_data.U32_data[0] % 4 != 0)
    {
        num_words++;
    }

    //save data in flash
    for (local_current_size = 0; local_current_size < num_words; local_current_size++)
    {
        ee_status = EE_Write(0, (uint16_t)local_current_size + ZIGBEE_DB_START_ADDR, cache_persistent_data.U32_data[local_current_size]);
        if (ee_status != EE_OK)
        {
           if(ee_status == EE_CLEAN_NEEDED) /* Shall not be there if CFG_EE_AUTO_CLEAN = 1*/
           {
              APP_DBG("CLEAN NEEDED, CLEANING");
              EE_Clean(0,0);
           }
           else
           {
              /* Failed to write , an Erase shall be done */
              APP_DBG("APP_ZIGBEE_NVM_Write failed @ %d status %d", local_current_size,ee_status);
              break;
           }
        }
    }


    if(ee_status != EE_OK)
    {
       APP_DBG("WRITE STOPPED, need a FLASH ERASE");
       return false;
    }

    APP_DBG("WRITTEN PERSISTENT DATA LEN = %d",cache_persistent_data.U32_data[0]);
    return true;

} /* APP_ZIGBEE_NVM_Write */

/**
 * @brief  Erase the NVM
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_NVM_Erase(void)
{
   EE_Init(1, HW_FLASH_ADDRESS + CFG_NVM_BASE_ADDRESS); /* Erase Flash */
} /* APP_ZIGBEE_NVM_Erase */

#endif /* CFG_NVM */


static void zb_leave_cb(struct ZbNlmeLeaveConfT *conf, void *arg)
{
	/* Configure the joining parameters */
	zigbee_app_info.join_delay = HAL_GetTick(); /* now */
	zigbee_app_info.startupControl = ZbStartTypeJoin;

	/* attempt to reconnect */
	UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
}

void ZIGBEE_Leave(void)
{
	enum ZbStatusCodeT status =
			ZbLeaveReq(zigbee_app_info.zb, zb_leave_cb, &zigbee_app_info);

	if (ZB_STATUS_SUCCESS == status) {
		zb_epanId = 0;
	}

	zigbee_app_info.join_status = (enum ZbStatusCodeT) 0x02; /* init to error status */

	if (ZB_STATUS_SUCCESS == status && persistNumWrites != 0) {
		APP_ZIGBEE_persist_delete();
	}
}

enum ZbStatusCodeT ZIGBEE_JoinStatus(void)
{
		return zigbee_app_info.join_status;
}

/* USER CODE END FD_LOCAL_FUNCTIONS */
