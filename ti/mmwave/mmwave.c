#include "mmwave.h"

/******************************************************************************
* GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
******************************************************************************
*/
typedef int (*RL_P_OS_SPAWN_FUNC_PTR)(RL_P_OSI_SPAWN_ENTRY pEntry, const void* pValue, unsigned int flags);
typedef int (*RL_P_OS_DELAY_FUNC_PTR)(unsigned int delay);

/* Global Variable for Device Status */
static pthread_mutex_t rlAsyncEvent;
static unsigned char mmwl_bInitComp = 0U;
static unsigned char mmwl_bMssBootErrStatus = 0U;
static unsigned char mmwl_bStartComp = 0U;
static unsigned char mmwl_bRfInitComp = 0U;
static unsigned char mmwl_bSensorStarted = 0U;
static unsigned char mmwl_bGpadcDataRcv = 0U;
static unsigned char mmwl_bMssCpuFault = 0U;
static unsigned char mmwl_bMssEsmFault = 0U;

static unsigned int mmwl_TDA_width[4] = { 0 };
static unsigned int mmwl_TDA_height[4] = { 0 };

static unsigned char mmwl_bTDA_CaptureCardConnect = 0U;
static unsigned char mmwl_bTDA_CreateAppACK = 0U;
static unsigned char mmwl_bTDA_StartRecordACK = 0U;
static unsigned char mmwl_bTDA_StopRecordACK = 0U;
static unsigned char mmwl_bTDA_FramePeriodicityACK = 0U;
static unsigned char mmwl_bTDA_FileAllocationACK = 0U;
static unsigned char mmwl_bTDA_DataPackagingACK = 0U;
static unsigned char mmwl_bTDA_CaptureDirectoryACK = 0U;
static unsigned char mmwl_bTDA_NumFramesToCaptureACK = 0U;
static unsigned char mmwl_bTDA_ARMDone = 0U;

unsigned char gAwr2243CrcType = RL_CRC_TYPE_32BIT;

rlUInt16_t lutOffsetInNBytes = 0;

/* Global variable configurations from config file */
rlDevGlobalCfg_t rlDevGlobalCfgArgs = { 0 };


/* store frame periodicity */
unsigned int framePeriodicity = 0;
/* store frame count */
unsigned int frameCount = 0;
/* store continous streaming time for TDA to capture the data */
unsigned int gContStreamTime = 0;
/* store continous streaming sampling rate */
unsigned int gContStreamSampleRate = 0;


/* Strcture to store async event config */
rlRfDevCfg_t rfDevCfg = { 0x0 };

/* Structure to store GPADC measurement data sent by device */
rlRecvdGpAdcData_t rcvGpAdcData = {0};

/* Thread return values */
rlReturnVal_t threadRetVal[TDA_NUM_CONNECTED_DEVICES_MAX];


rlReturnVal_t rlDeviceFileDownloadWrap(rlUInt8_t deviceMap, \
      rlUInt16_t remChunks, rlFileData_t* data) {
  return (rlDeviceFileDownload(deviceMap, data, remChunks));
}


rlReturnVal_t (*funcTableTypeA[])(unsigned char, void *) = {
  rlSetAdcOutConfig,
  rlSetLowPowerModeConfig,
  rlSetChannelConfig,
  rlSetBpmChirpConfig,
  rlRfCalibDataRestore,
  rlSetFrameConfig,
  rlSetAdvChirpConfig,
  rlSetAdvFrameConfig,
  rlRfDynamicPowerSave,
  rlRfDfeRxStatisticsReport,
  rlSetContModeConfig,
  rlEnableContMode,
  rlDeviceSetDataFmtConfig,
  rlDeviceSetDataPathConfig,
  rlDeviceSetMiscConfig,
  rlDeviceSetLaneConfig,
  rlDeviceSetDataPathClkConfig,
  rlDeviceSetLvdsLaneConfig,
  rlDeviceSetContStreamingModeConfig,
  rlDeviceSetCsi2Config,
  rlDeviceSetHsiClk,
  rlRfSetLdoBypassConfig,
  rlSetGpAdcConfig,
  rlRfSetDeviceCfg,
  rlRfSetPALoopbackConfig,
  rlRfSetPSLoopbackConfig,
  rlRfSetIFLoopbackConfig,
  rlRfSetProgFiltCoeffRam,
  rlRfSetProgFiltConfig,
  rlRfSetMiscConfig,
  rlRfSetCalMonTimeUnitConfig,
  rlRfSetCalMonFreqLimitConfig,
  rlRfInitCalibConfig,
  rlRfRunTimeCalibConfig,
  rlRfDigMonEnableConfig,
  rlRfCalibDataStore,
  rlDeviceSetTestPatternConfig,
  rlRfTxFreqPwrLimitConfig,
  rlRfRxGainPhMonConfig,
  rlRfInterRxGainPhaseConfig,
  rlRfTxPhShiftMonConfig,
  rlRfAnaFaultInjConfig,
  rlRfRxIfSatMonConfig,
  rlRfRxSigImgMonConfig,
  rlTxGainTempLutSet,
  rlRfAnaMonConfig,
  rlDeviceLatentFaultTests,
  rlDeviceEnablePeriodicTests,
  rlSetLoopBckBurstCfg,
  rlSetSubFrameStart,
  rlDeviceMcuClkConfig,
  rlRfPhShiftCalibDataRestore,
  rlSetInterChirpBlkCtrl,
  rlDevicePmicClkConfig,
  rlSetDynChirpEn,
  rlRfTxGainPhaseMismatchMonConfig,
  rlRfTempMonConfig,
  rlRfExtAnaSignalsMonConfig,
  rlRfGpadcIntAnaSignalsMonConfig,
  rlRfPmClkLoIntAnaSignalsMonConfig,
  rlRfRxIntAnaSignalsMonConfig,
  rlRfTxIntAnaSignalsMonConfig,
  rlRfDualClkCompMonConfig,
  rlRfPllContrlVoltMonConfig,
  rlRfSynthFreqMonConfig,
  rlRfTxPowrMonConfig,
  rlRfRxNoiseMonConfig,
  rlRfRxMixerInPwrConfig,
  rlRfRxIfStageMonConfig,
  rlRfDigMonPeriodicConfig,
  rlRfTxBallbreakMonConfig,
  rlRfPhShiftCalibDataStore,
  rlDeviceGetRfVersion,
  rlDeviceGetMssVersion,
  rlGetAdvFrameConfig,
  rlSetTestSourceConfig,
  rlTestSourceEnable,
  rlDeviceGetVersion,
  rlGetRfDieId,
  rlMonTypeTrigConfig,
  rlRfApllSynthBwCtlConfig,
  rlDeviceSetDebugSigEnableConfig,
  rlDeviceSetHsiDelayDummyConfig,
  rlSetAdvChirpLUTConfig,
  rlFrameStartStop,
  rlGetRfBootupStatus,
  rlSetAdvChirpDynLUTAddrOffConfig,
  rlRfGetTemperatureReport

#define SET_ADC_OUT_IND                             0
#define SET_LOW_POWER_MODE_IND                      1
#define SET_CHANNEL_CONFIG_IND                      2
#define SET_BPM_CHIRP_CONFIG_IND                    3
#define RF_CALIB_DATA_RESTORE_IND                   4
#define SET_FRAME_CONFIG_IND                        5
#define SET_ADV_CHIRP_CONFIG_IND                    6
#define SET_ADV_FRAME_CONFIG_IND                    7
#define RF_DYNAMIC_POWER_SAVE_IND                   8
#define RF_DFE_RX_STATS_REPORT_IND                  9
#define SET_CONT_MODE_CONFIG_IND                    10
#define ENABLE_CONT_MODE_IND                        11
#define SET_DATA_FORMAT_CONFIG_IND                  12
#define SET_DATA_PATH_CONFIG_IND                    13
#define SET_MISC_CONFIG_IND                         14
#define SET_LANE_CONFIG_IND                         15
#define SET_DATA_PATH_CLK_CONFIG_IND                16
#define SET_LVDS_LANE_CONFIG_IND                    17
#define SET_CONT_STREAM_MODE_CONFIG_IND             18
#define SET_CSI2_CONFIG_IND                         19
#define SET_HSI_CLK_IND                             20
#define SET_LDO_BYPASS_CONFIG_IND                   21
#define SET_GPADC_CONFIG                            22
#define RF_SET_DEVICE_CONFIG_IND                    23
#define RF_SET_PA_LPBK_CONFIG_IND                   24
#define RF_SET_PS_LPBK_CONFIG_IND                   25
#define RF_SET_IF_LPBK_CONFIG_IND                   26
#define RF_SET_PROG_FILT_COEFF_RAM_IND              27
#define RF_SET_PROG_FILT_CONFIG_IND                 28
#define RF_SET_MISC_CONFIG_IND                      29
#define RF_SET_CAL_MON_TIME_CONFIG_IND              30
#define RF_SET_CAL_MON_FREQ_LIM_IND                 31
#define RF_INIT_CALIB_CONFIG_IND                    32
#define RF_RUN_TIME_CALIB_CONFIG_IND                33
#define RF_DIG_MON_ENABLE_CONFIG                    34
#define RF_CALIB_DATA_STORE_IND                     35
#define SET_TEST_PATTERN_CONFIG_IND                 36
#define RF_TX_FREQ_PWR_LIMIT_CONFIG_IND             37
#define RF_RX_GAIN_PH_MON_CONFIG_IND                38
#define RF_INTER_RX_GAIN_PHASE_CONFIG_IND           39
#define RF_TX_PH_SHIFT_MON_CONFIG_IND               40
#define RF_ANA_FAULT_INJ_CONFIG_IND                 41
#define RF_RX_IF_SAT_MON_CONFIG_IND                 42
#define RF_RX_SIG_IMG_MON_CONFIG_IND                43
#define TX_GAIN_TEMP_LUT_SET_IND                    44
#define RF_ANA_MON_CONFIG_IND                       45
#define LATENT_FAULT_TESTS_IND                      46
#define ENABLE_PERIODIC_TESTS_IND                   47
#define SET_LOOPBAK_BURST_CFG_IND                   48
#define SET_SUBFRAME_START_IND                      49
#define MCU_CLK_CONFIG_IND                          50
#define RF_PH_SHIFT_CALIB_DATA_RESTORE_IND          51
#define SET_INTER_CHIRP_BLK_CTRL_IND                52
#define PMIC_CLK_CONFIG_IND                         53
#define SET_DYN_CHIRP_EN_IND                        54
#define RF_TX_GAIN_PHASE_MISMATCH_CONFIG_IND        55
#define RF_TEMP_MON_CONFIG_IND                      56
#define RF_EXT_ANA_SIGNALS_MON_CONFIG_IND           57
#define RF_GPADC_INT_ANA_SIGNALS_MON_CONFIG_IND     58
#define RF_PMCLK_LO_INT_ANA_SIGNALS_MON_CONFIG_IND  59
#define RF_RX_INT_ANA_SIGNALS_MON_CONFIG_IND        60
#define RF_TX_INT_ANA_SIGNALS_MON_CONFIG_IND        61
#define RF_DUAL_CLK_COMP_MON_CONFIG_IND             62
#define RF_PLL_CONTRL_VOLT_MON_CONFIG_IND           63
#define RF_SYNTH_FREQ_MON_CONFIG_IND                64
#define RF_TX_POWR_MON_CONFIG_IND                   65
#define RF_RX_NOISE_MON_CONFIG_IND                  66
#define RF_RX_MIXER_IN_PWR_CONFIG_IND               67
#define RF_RX_IF_STAGE_MON_CONFIG_IND               68
#define RF_DIG_MON_PERIODIC_CONFIG_IND              69
#define RF_TX_BALL_BREAK_MON_CONFIG_IND             70
#define RF_PH_SHIFT_CALIB_DATA_STORE_IND            71
#define GET_RF_VERSION_IND                          72
#define GET_MSS_VERSION_IND                         73
#define GET_ADV_FRAME_CONFIG_IND                    74
#define SET_TEST_SOURCE_CONFIG_IND                  75
#define RF_TEST_SOURCE_ENABLE                       76
#define RF_GET_VERSION_IND                          77
#define RF_GET_DIE_ID_IND                           78
#define RF_SET_MON_TYPE_TRIGGER_CONFIG_IND          79
#define RF_SET_APLL_SYNTH_BW_CTL_CONFIG_IND         80
#define RF_SET_DEBUG_SIGNALS_CONFIG_IND             81
#define RF_SET_CSI2_DELAY_DUMMY_CONFIG_IND          82
#define SET_ADV_CHIRP_LUT_CONFIG_IND                83
#define SENSOR_START_STOP_IND                       84
#define GET_RF_BOOTUP_STATUS_IND                    85
#define SET_ADV_CHIRP_DYN_LUT_CONFIG_IND            86
#define GET_TEMP_DEVICE_IND                         87
};


rlReturnVal_t(*funcTableTypeB[])(unsigned char) = {
  rlDeviceAddDevices,
  rlDeviceRemoveDevices,
  rlDeviceRfStart,
  rlRfInit,
  rlSensorStart,
  rlSensorStop

#define ADD_DEVICE_IND        0
#define REMOVE_DEVICE_IND     1
#define RF_START_IND          2
#define RF_INIT_IND           3
#define SENSOR_START_IND      4
#define SENSOR_STOP_IND       5
};


rlReturnVal_t(*funcTableTypeC[])(unsigned char, unsigned short, void *) = {
  rlSetProfileConfig,
  rlSetChirpConfig,
  rlRfSetPhaseShiftConfig,
  rlDeviceFileDownloadWrap,
  rlSetDynChirpCfg,
  rlSetDynPerChirpPhShifterCfg,
  rlGetProfileConfig

#define SET_PROFILE_CONFIG_IND                    0
#define SET_CHIRP_CONFIG_IND                      1
#define RF_SET_PHASE_SHIFT_CONFIG_IND             2
#define FILE_DOWNLOAD_IND                         3
#define SET_DYN_CHIRP_CFG_IND                     4
#define SET_DYN_PER_PERCHIRP_PH_SHIFTER_CFG_IND   5
#define GET_PROFILE_CONFIG_IND                    6
};


/**
 * @brief Executed by threads to interact with the radar devices
 * 
 * @param lpParam Parameter passed onto the thread
 * @return Pointer to the thread's return status
 */
void* threadHandler(void* lpParam) {
  taskData *data = (taskData*)lpParam;
  unsigned int apiId = data->apiInfo & 0xFFFF;
  unsigned int apiType = data->apiInfo & 0xF0000000;

  switch (apiType) {
    case API_TYPE_A:
      threadRetVal[data->deviceIndex] = funcTableTypeA[apiId](
        (1 << data->deviceIndex), data->payLoad
      );
      break;

    case API_TYPE_B:
      threadRetVal[data->deviceIndex] = funcTableTypeB[apiId](1 << data->deviceIndex);
      break;

    case API_TYPE_C:
      threadRetVal[data->deviceIndex] = funcTableTypeC[apiId](
        (1 << data->deviceIndex), data->flag, data->payLoad
      );
      break;
    default:
      threadRetVal[data->deviceIndex] = -1;
  }

  return NULL; // &threadRetVal[data->deviceIndex];
}


int callThreadApi(unsigned int apiInfo, unsigned int deviceMap, void *apiParams, unsigned int flags) {
  int  retVal = RL_RET_CODE_OK;
  HANDLE  hThreadArray[TDA_NUM_CONNECTED_DEVICES_MAX];
  bzero(hThreadArray, TDA_NUM_CONNECTED_DEVICES_MAX * sizeof(HANDLE));

  taskData myTaskData[TDA_NUM_CONNECTED_DEVICES_MAX];
  volatile int devIndex = 0;
  unsigned int dmap = deviceMap;

  while (deviceMap != 0U) {
    if ((deviceMap & (1U << devIndex)) != 0U) {
      myTaskData[devIndex].deviceIndex = devIndex;
      myTaskData[devIndex].payLoad = apiParams;
      myTaskData[devIndex].apiInfo = apiInfo;
      myTaskData[devIndex].flag = flags;
      threadRetVal[devIndex] = -1;
      /* create a thread */
      pthread_create(&hThreadArray[devIndex], NULL, threadHandler, &myTaskData[devIndex]);
    }
    deviceMap &= ~(1U << devIndex);
    devIndex++;
  }

  for (devIndex = 0; devIndex < 4; devIndex++) {
    if (hThreadArray[devIndex] != 0U) {
      // rlReturnVal_t* pRet = &threadRetVal[devIndex];
      // rlReturnVal_t** pRetVal = NULL;
      pthread_join(hThreadArray[devIndex], NULL);
      retVal |= threadRetVal[devIndex];
    }
  }
  return retVal;
}

#define CALL_API(m,n,o,p)  callThreadApi(m, n, o, p)


/**
 * @brief TDA Async event handler
 * 
 * @param deviceMap 
 * @param cmdCode 
 * @param ackCode 
 * @param status 
 * @param data 
 */
void TDA_asyncEventHandler(rlUInt16_t deviceMap, rlUInt16_t cmdCode, rlUInt16_t ackCode,
  rlInt32_t status, rlUInt8_t *data) {
  switch (cmdCode) {
    case CAPTURE_RESPONSE_ACK:  {
      DEBUG_PRINT(
        "Device map %u : CAPTURE_RESPONSE_ACK Async event recieved with status %d \n\n",
        deviceMap, status
      );
      if (ackCode == CAPTURE_CONFIG_CONNECT) {
        mmwl_bTDA_CaptureCardConnect = 1U;
      }
      if (deviceMap == 32) {
        if (ackCode == CAPTURE_CONFIG_CREATE_APPLICATION) {
          mmwl_bTDA_CreateAppACK = 1;
        } else if (ackCode == CAPTURE_DATA_START_RECORD) {
          mmwl_bTDA_StartRecordACK = 1;
        } else if (ackCode == CAPTURE_DATA_STOP_RECORD) {
          mmwl_bTDA_StopRecordACK = 1;
        } else if (ackCode == CAPTURE_DATA_FRAME_PERIODICITY) {
          mmwl_bTDA_FramePeriodicityACK = 1;
        } else if (ackCode == CAPTURE_DATA_NUM_ALLOCATED_FILES) {
          mmwl_bTDA_FileAllocationACK = 1;
        } else if (ackCode == CAPTURE_DATA_ENABLE_DATA_PACKAGING) {
          mmwl_bTDA_DataPackagingACK = 1;
        } else if (ackCode == CAPTURE_DATA_SESSION_DIRECTORY) {
          mmwl_bTDA_CaptureDirectoryACK = 1;
        } else if (ackCode == CAPTURE_DATA_NUM_FRAMES) {
          mmwl_bTDA_NumFramesToCaptureACK = 1;
        }
      }
      break;
    }

    case CAPTURE_RESPONSE_NACK: {
      DEBUG_PRINT(
        "Device map %u : CAPTURE_RESPONSE_NACK Async event recieved with status %d \n\n",
        deviceMap, status
      );
      break;
    }

    case CAPTURE_RESPONSE_VERSION_INFO: {
      unsigned char rcvData[100] = { 0 };
      if (data != NULL) memcpy(&rcvData, data, sizeof(rcvData));
      DEBUG_PRINT(
        "Device map %u : CAPTURE_RESPONSE_VERSION_INFO Async event recieved with status %d. TDA Version : %s \n\n",
        deviceMap, status, rcvData
      );
      break;
    }

    case CAPTURE_RESPONSE_CONFIG_INFO: {
      unsigned char rcvData_1[8] = { 0 };
      if (data != NULL) memcpy(&rcvData_1, data, sizeof(rcvData_1));
      unsigned int width = rcvData_1[0] | (rcvData_1[1] << 1) | (rcvData_1[2] << 2) | (rcvData_1[3] << 3);
      unsigned int height = rcvData_1[4] | (rcvData_1[5] << 1) | (rcvData_1[6] << 2) | (rcvData_1[7] << 3);
      DEBUG_PRINT(
        "Device map %u : CAPTURE_RESPONSE_CONFIG_INFO Async event recieved with status %d. Width : %d and Height : %d \n\n",
        deviceMap, status, width, height
      );
      break;
    }

    case CAPTURE_RESPONSE_TRACE_DATA: {
      DEBUG_PRINT(
        "Device map %u : CAPTURE_RESPONSE_TRACE_DATA Async event recieved with status %d \n\n",
        deviceMap, status
      );
      break;
    }

    case CAPTURE_RESPONSE_GPIO_DATA: {
      unsigned char rcvData_2[12] = { 0 };
      if (data != NULL) memcpy(&rcvData_2, data, sizeof(rcvData_2));
      unsigned int gpioVal = rcvData_2[8] | (rcvData_2[9] << 1) | (rcvData_2[10] << 2) | (rcvData_2[11] << 3);
      DEBUG_PRINT(
        "Device map %u : CAPTURE_RESPONSE_GPIO_DATA Async event recieved with status %d. GPIO Value : %d \n\n",
        deviceMap, status, gpioVal
      );
      break;
    }

    case SENSOR_RESPONSE_SOP_INFO: {
      unsigned char rcvData_3[4] = { 0 };
      if (data != NULL)
        memcpy(&rcvData_3, data, sizeof(rcvData_3));
      unsigned int sopMode = rcvData_3[0] | (rcvData_3[1] << 1) | (rcvData_3[2] << 2) | (rcvData_3[3] << 3);
      DEBUG_PRINT(
        "Device map %u : SENSOR_RESPONSE_SOP_INFO Async event recieved with status %d. SOP Mode : %d \n\n",
        deviceMap, status, sopMode
      );
      break;
    }

    case CAPTURE_RESPONSE_NETWORK_ERROR: {
      DEBUG_PRINT(
        "CAPTURE_RESPONSE_NETWORK_ERROR Async event recieved! Connection error! Please reboot the TDA board\n\n"
      );
      break;
    }

    default: {
      DEBUG_PRINT(
        "Device map %u : Unhandled Async Event with cmdCode = 0x%x and status = %d  \n\n",
        deviceMap, cmdCode, status
      );
      break;
    }
  }
}


/** @fn void MMWL_asyncEventHandler(rlUInt8_t deviceIndex, rlUInt16_t sbId,
*    rlUInt16_t sbLen, rlUInt8_t *payload)
*
*   @brief Radar Async Event Handler callback
*   @param[in] msgId - Message Id
*   @param[in] sbId - SubBlock Id
*   @param[in] sbLen - SubBlock Length
*   @param[in] payload - Sub Block Payload
*
*   @return None
*
*   Radar Async Event Handler callback
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
void MMWL_asyncEventHandler(rlUInt8_t deviceIndex, rlUInt16_t sbId,
  rlUInt16_t sbLen, rlUInt8_t *payload) {
  rlUInt16_t msgId = sbId / RL_MAX_SB_IN_MSG;
  rlUInt16_t asyncSB = RL_GET_SBID_FROM_MSG(sbId, msgId);

  /* Host can receive Async Event from RADARSS/MSS */
  switch (msgId) {
    /* Async Event from RADARSS */
    case RL_RF_ASYNC_EVENT_MSG: {
      switch (asyncSB) {
        case RL_RF_AE_FRAME_TRIGGER_RDY_SB: {
          pthread_mutex_lock(&rlAsyncEvent);
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          mmwl_bSensorStarted |= (1 << deviceIndex);
          DEBUG_PRINT("Device map %u : Frame Start Async event\n\n", deviceMap);
          pthread_mutex_unlock(&rlAsyncEvent);
          break;
        }

        case RL_RF_AE_FRAME_END_SB: {
          pthread_mutex_lock(&rlAsyncEvent);
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          mmwl_bSensorStarted &= ~(1 << deviceIndex);
          DEBUG_PRINT("Device map %u : Frame End Async event\n\n", deviceMap);
          pthread_mutex_unlock(&rlAsyncEvent);
          break;
        }

        case RL_RF_AE_CPUFAULT_SB: {
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          DEBUG_PRINT("Device map %u : BSS CPU Fault Async event\n\n", deviceMap);
          while(1);
        }

        case RL_RF_AE_ESMFAULT_SB: {
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          DEBUG_PRINT("Device map %u : BSS ESM Fault Async event\n\n", deviceMap);
          break;
        }

        case RL_RF_AE_INITCALIBSTATUS_SB: {
          pthread_mutex_lock(&rlAsyncEvent);
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          mmwl_bRfInitComp |= (1 << deviceIndex);
          DEBUG_PRINT("Device map %u : RF-Init Async event\n\n", deviceMap);
          pthread_mutex_unlock(&rlAsyncEvent);
          break;
        }

        case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB: {
          pthread_mutex_lock(&rlAsyncEvent);
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          rlCalMonTimingErrorReportData_t *data = (rlCalMonTimingErrorReportData_t*)payload;
          DEBUG_PRINT(
            "Device map %u : Cal Mon Time Unit Fail [0x%x] Async event\n\n",
            deviceMap, data->timingFailCode
          );
          pthread_mutex_unlock(&rlAsyncEvent);
          break;
        }

        case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB: {
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          DEBUG_PRINT(
            "Device map %u : Run time Calibration Report [0x%x] Async event\n\n",
            deviceMap, ((rlRfRunTimeCalibReport_t*)payload)->calibErrorFlag
          );
          break;
        }

        case RL_RF_AE_DIG_LATENTFAULT_REPORT_SB: {
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          rlDigLatentFaultReportData_t *data = (rlDigLatentFaultReportData_t*)payload;
          DEBUG_PRINT(
            "Device map %u : Dig Latent Fault report [0x%x] Async event\n\n",
            deviceMap, data->digMonLatentFault
          );
          break;
        }

        case RL_RF_AE_MON_DIG_PERIODIC_REPORT_SB: {
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          rlDigPeriodicReportData_t *data = (rlDigPeriodicReportData_t*)payload;
          DEBUG_PRINT(
            "Device map %u : Dig periodic report [0x%x] Async event\n\n",
            deviceMap, data->digMonPeriodicStatus
          );
          break;
        }

        case RL_RF_AE_GPADC_MEAS_DATA_SB: {
          pthread_mutex_lock(&rlAsyncEvent);
          mmwl_bGpadcDataRcv |= (1 << deviceIndex);
          /* store the GPAdc Measurement data which AWR2243 will read from the analog test pins
              where user has fed the input signal */
          memcpy(&rcvGpAdcData, payload, sizeof(rlRecvdGpAdcData_t));
          pthread_mutex_unlock(&rlAsyncEvent);
          break;
        }

        default: {
          DEBUG_PRINT("Unhandled Async Event msgId: 0x%x, asyncSB:0x%x  \n\n", msgId, asyncSB);
          break;
        }
      }
      break;
    }

    /* Async Event from MSS */
    case RL_DEV_ASYNC_EVENT_MSG: {
      switch (asyncSB) {
        case RL_DEV_AE_MSSPOWERUPDONE_SB: {
          pthread_mutex_lock(&rlAsyncEvent);
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          mmwl_bInitComp |= (1 << deviceIndex);
          DEBUG_PRINT("Device map %u : MSS Power Up Async event\n\n", deviceMap);
          rlInitComplete_t *data = (rlInitComplete_t*)payload;
          DEBUG_PRINT(
            "PowerUp Time = %d, PowerUp Status 1 = 0x%x, PowerUp Status 2 = 0x%x, BootTestStatus 1 = 0x%x, BootTestStatus 2 = 0x%x\n\n",
            data->powerUpTime, data->powerUpStatus1,
            data->powerUpStatus2, data->bootTestStatus1,
            data->bootTestStatus2
          );
          pthread_mutex_unlock(&rlAsyncEvent);
          break;
        }

        case RL_DEV_AE_RFPOWERUPDONE_SB: {
          pthread_mutex_lock(&rlAsyncEvent);
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          mmwl_bStartComp |= (1 << deviceIndex);
          DEBUG_PRINT("Device map %u : BSS Power Up Async event\n\n", deviceMap);
          pthread_mutex_unlock(&rlAsyncEvent);
          break;
        }

        case RL_DEV_AE_MSS_CPUFAULT_SB: {
          pthread_mutex_lock(&rlAsyncEvent);
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          mmwl_bMssCpuFault |= (1 << deviceIndex);
          DEBUG_PRINT("Device map %u : MSS CPU Fault Async event\n\n", deviceMap);
          pthread_mutex_unlock(&rlAsyncEvent);
          break;
        }

        case RL_DEV_AE_MSS_ESMFAULT_SB: {
          pthread_mutex_lock(&rlAsyncEvent);
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          mmwl_bMssEsmFault |= (1 << deviceIndex);
          DEBUG_PRINT("Device map %u : MSS ESM Fault Async event\n\n", deviceMap);
          rlMssEsmFault_t *data = (rlMssEsmFault_t*)payload;
          DEBUG_PRINT("ESM Grp1 Error = 0x%x, ESM Grp2 Error = 0x%x\n\n", data->esmGrp1Err, data->esmGrp2Err);
          pthread_mutex_unlock(&rlAsyncEvent);
          break;
        }

        case RL_DEV_AE_MSS_BOOTERRSTATUS_SB: {
          pthread_mutex_lock(&rlAsyncEvent);
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          mmwl_bMssBootErrStatus |= (1 << deviceIndex);
          DEBUG_PRINT("Device map %u : MSS Boot Error Status Async event\n\n", deviceMap);
          pthread_mutex_unlock(&rlAsyncEvent);
          break;
        }

        case RL_DEV_AE_MSS_LATENTFLT_TEST_REPORT_SB: {
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          rlMssLatentFaultReport_t *data = (rlMssLatentFaultReport_t*)payload;
          DEBUG_PRINT(
            "Device map %u : MSS Latent fault [0x%x] [0x%x] Async event\n\n",
            deviceMap, data->testStatusFlg1, data->testStatusFlg2
          );
          break;
        }

        case RL_DEV_AE_MSS_PERIODIC_TEST_STATUS_SB: {
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          rlMssPeriodicTestStatus_t *data = (rlMssPeriodicTestStatus_t*)payload;
          DEBUG_PRINT(
            "Device map %u : MSS periodic test [0x%x] Async event\n\n",
            deviceMap, data->testStatusFlg
          );
          break;
        }

        case RL_DEV_AE_MSS_RF_ERROR_STATUS_SB: {
          unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
          rlMssRfErrStatus_t *data = (rlMssRfErrStatus_t*)payload;
          DEBUG_PRINT("Device map %u : MSS RF Error [0x%x] Status Async event\n\n", deviceMap, data->errStatusFlg);
          break;
        }

        default: {
          DEBUG_PRINT("Unhandled Async Event msgId: 0x%x, asyncSB:0x%x  \n\n", msgId, asyncSB);
          break;
        }
      }
      break;
    }

    /* Async Event from MMWL */
    case RL_MMWL_ASYNC_EVENT_MSG: {
      switch (asyncSB) {
        case RL_MMWL_AE_MISMATCH_REPORT: {
          int errTemp = *(int32_t*)payload;

          /* CRC mismatched in the received Async-Event msg */
          if (errTemp == RL_RET_CODE_CRC_FAILED) {
              unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
              DEBUG_PRINT("Device map %u : CRC mismatched in the received Async-Event msg\n\n", deviceMap);
          }

          /* Checksum mismatched in the received msg */
          else if (errTemp == RL_RET_CODE_CHKSUM_FAILED) {
              unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
              DEBUG_PRINT("Device map %u : Checksum mismatched in the received msg\n\n", deviceMap);
          }

          /* Polling to HostIRQ is timed out,
          i.e. Device didn't respond to CNYS from the Host */
          else if (errTemp == RL_RET_CODE_HOSTIRQ_TIMEOUT) {
              unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
              DEBUG_PRINT("Device map %u : HostIRQ polling timed out\n\n", deviceMap);
          }

          /* If any of OSI call-back function returns non-zero value */
          else if (errTemp == RL_RET_CODE_RADAR_OSIF_ERROR) {
              unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
              DEBUG_PRINT("Device map %u : mmWaveLink OS_IF error \n\n", deviceMap);
          }
          break;
        }
      }
      break;
    }

    default: {
      unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
      DEBUG_PRINT(
        "Device map %u : Unhandled Async Event msgId: 0x%x, asyncSB:0x%x  \n\n",
        deviceMap, msgId, asyncSB
      );
      break;
    }
  }
}


/** @fn int MMWL_computeCRC(unsigned char* data, unsigned int dataLen, unsigned char crcLen,
                        unsigned char* outCrc)
*
*   @brief Compute the CRC of given data
*
*   @param[in] data - message data buffer pointer
*    @param[in] dataLen - length of data buffer
*    @param[in] crcLen - length of crc 2/4/8 bytes
*    @param[out] outCrc - computed CRC data
*
*   @return int Success - 0, Failure - Error Code
*
*   Compute the CRC of given data
*/
int MMWL_computeCRC(unsigned char* data, unsigned int dataLen, unsigned char crcLen, unsigned char* outCrc) {
    uint64_t crcResult = computeCRC(data, dataLen, (16 << crcLen));
    memcpy(outCrc, &crcResult, (2 << crcLen));
    return 0;
}


/** @fn int MMWL_powerOnMaster(deviceMap)
*
*   @brief Power on Master API.
*
*   @param[in] deviceMap - Devic Index
*   @param[in] rlClientCbsTimeout - Timeout to use for mmwavelink client in ms
*
*   @return int Success - 0, Failure - Error Code
*
*   Power on Master API.
*/
int MMWL_powerOnMaster(unsigned char deviceMap, uint32_t rlClientCbsTimeout) {
  int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
  /*
    \subsection     porting_step1   Step 1 - Define mmWaveLink client callback structure
  The mmWaveLink framework is ported to different platforms using mmWaveLink client callbacks. These
  callbacks are grouped as different structures such as OS callbacks, Communication Interface
  callbacks and others. Application needs to define these callbacks and initialize the mmWaveLink
  framework with the structure.

    Refer to \ref rlClientCbs_t for more details
    */
  rlClientCbs_t clientCtx = { 0 };

  clientCtx.ackTimeout = rlClientCbsTimeout;  // ms
  clientCtx.crcType =  gAwr2243CrcType;

  /*
  \subsection     porting_step2   Step 2 - Implement Communication Interface Callbacks
  The mmWaveLink device support several standard communication protocol among SPI and MailBox
  Depending on device variant, one need to choose the communication channel. For e.g
  xWR1443/xWR1642 requires Mailbox interface and AWR2243 supports SPI interface.
  The interface for this communication channel should include 4 simple access functions:
  -# rlComIfOpen
  -# rlComIfClose
  -# rlComIfRead
  -# rlComIfWrite

  Refer to \ref rlComIfCbs_t for interface details
  */
  clientCtx.comIfCb.rlComIfOpen = TDACommOpen;
  clientCtx.comIfCb.rlComIfClose = TDACommClose;
  clientCtx.comIfCb.rlComIfRead = spiReadFromDevice;
  clientCtx.comIfCb.rlComIfWrite = spiWriteToDevice;

  /*   \subsection     porting_step3   Step 3 - Implement Device Control Interface
  The mmWaveLink driver internally powers on/off the mmWave device. The exact implementation of
  these interface is platform dependent, hence you need to implement below functions:
  -# rlDeviceEnable
  -# rlDeviceDisable
  -# rlRegisterInterruptHandler

  Refer to \ref rlDeviceCtrlCbs_t for interface details
  */
  clientCtx.devCtrlCb.rlDeviceDisable = TDADisableDevice;
  clientCtx.devCtrlCb.rlDeviceEnable = TDAEnableDevice;
  clientCtx.devCtrlCb.rlDeviceMaskHostIrq = TDACommIRQMask;
  clientCtx.devCtrlCb.rlDeviceUnMaskHostIrq = TDACommIRQUnMask;
  clientCtx.devCtrlCb.rlRegisterInterruptHandler = TDAregisterCallback;
  clientCtx.devCtrlCb.rlDeviceWaitIrqStatus = TDADeviceWaitIrqStatus;

  /*  \subsection     porting_step4     Step 4 - Implement Event Handlers
  The mmWaveLink driver reports asynchronous event indicating mmWave device status, exceptions
  etc. Application can register this callback to receive these notification and take appropriate
  actions

  Refer to \ref rlEventCbs_t for interface details*/
  clientCtx.eventCb.rlAsyncEvent = MMWL_asyncEventHandler;

  /*  \subsection     porting_step5     Step 5 - Implement OS Interface
  The mmWaveLink driver can work in both OS and NonOS environment. If Application prefers to use
  operating system, it needs to implement basic OS routines such as tasks, mutex and Semaphore


  Refer to \ref rlOsiCbs_t for interface details
  */
  /* Mutex */
  clientCtx.osiCb.mutex.rlOsiMutexCreate = osiLockObjCreate;
  clientCtx.osiCb.mutex.rlOsiMutexLock = osiLockObjLock;
  clientCtx.osiCb.mutex.rlOsiMutexUnLock = osiLockObjUnlock;
  clientCtx.osiCb.mutex.rlOsiMutexDelete = osiLockObjDelete;

  /* Semaphore */
  clientCtx.osiCb.sem.rlOsiSemCreate = osiSyncObjCreate;
  clientCtx.osiCb.sem.rlOsiSemWait = osiSyncObjWait;
  clientCtx.osiCb.sem.rlOsiSemSignal = osiSyncObjSignal;
  clientCtx.osiCb.sem.rlOsiSemDelete = osiSyncObjDelete;

  /* Spawn Task */
  clientCtx.osiCb.queue.rlOsiSpawn = (RL_P_OS_SPAWN_FUNC_PTR)osiExecute;

  /* Sleep/Delay Callback*/
  clientCtx.timerCb.rlDelay = (RL_P_OS_DELAY_FUNC_PTR)osiSleep;

  /*  \subsection     porting_step6     Step 6 - Implement CRC Interface
  The mmWaveLink driver uses CRC for message integrity. If Application prefers to use
  CRC, it needs to implement CRC routine.

  Refer to \ref rlCrcCbs_t for interface details
  */
  clientCtx.crcCb.rlComputeCRC = MMWL_computeCRC;

  /*  \subsection     porting_step7     Step 7 - Define Platform
  The mmWaveLink driver can be configured to run on different platform by
  passing appropriate platform and device type
  */
  clientCtx.platform = RL_PLATFORM_HOST;
  clientCtx.arDevType = RL_AR_DEVICETYPE_22XX;

  /*clear all the interupts flag*/
  mmwl_bInitComp = 0;
  mmwl_bStartComp = 0U;
  mmwl_bRfInitComp = 0U;

  pthread_mutex_init(&rlAsyncEvent, NULL);

  /*  \subsection     porting_step8     step 8 - Call Power ON API and pass client context
  The mmWaveLink driver initializes the internal components, creates Mutex/Semaphore,
  initializes buffers, register interrupts, bring mmWave front end out of reset.
  */
  retVal = rlDevicePowerOn(deviceMap, clientCtx);

  /*  \subsection     porting_step9     step 9 - Test if porting is successful
  Once configuration is complete and mmWave device is powered On, mmWaveLink driver receives
  asynchronous event from mmWave device and notifies application using
  asynchronous event callback.
  Refer to \ref MMWL_asyncEventHandler for event details
@Note: In case of ES1.0 sample application needs to wait for MSS CPU fault as well with some timeout.
  */
  while ((mmwl_bInitComp & deviceMap) != deviceMap) {
    msleep(1); /*Sleep 1 msec*/
    timeOutCnt++;
    if (timeOutCnt > MMWL_API_INIT_TIMEOUT) {
      retVal = RL_RET_CODE_RESP_TIMEOUT;
      break;
    }
  }
  mmwl_bInitComp = 0U;
  return retVal;
}


int MMWL_fileWrite(unsigned char deviceMap, unsigned short remChunks,
                   unsigned short chunkLen, unsigned char *chunk) {
  int ret_val = -1;

  rlFileData_t fileChunk = { 0 };
  fileChunk.chunkLen = chunkLen;
  memcpy(fileChunk.fData, chunk, chunkLen);

  ret_val = CALL_API(API_TYPE_C | FILE_DOWNLOAD_IND, deviceMap, &fileChunk, remChunks);
  return ret_val;
}


/** @fn int MMWL_fileDownload((unsigned char deviceMap,
                  mmwlFileType_t fileType,
                  unsigned int fileLen)
*
*   @brief Firmware Download API.
*
*   @param[in] deviceMap - Devic Index
*    @param[in] fileType - firmware/file type
*    @param[in] fileLen - firmware/file length
*
*   @return int Success - 0, Failure - Error Code
*
*   Firmware Download API.
*/
int MMWL_fileDownload(unsigned char deviceMap, unsigned int fileLen) {
  unsigned int imgLen = fileLen;
  int ret_val = -1;
  int mmwl_iRemChunks = 0;
  unsigned short usChunkLen = 0U;
  unsigned int iNumChunks = 0U;
  unsigned short usLastChunkLen = 0;
  unsigned short usFirstChunkLen = 0;
  unsigned short usProgress = 0;

  /*First Chunk*/
  unsigned char firstChunk[MMWL_FW_CHUNK_SIZE];
  unsigned char* pmmwl_imgBuffer = NULL;

  pmmwl_imgBuffer = (unsigned char*)&metaImage[0];

  if(pmmwl_imgBuffer == NULL) {
    DEBUG_PRINT(
      "Device map %u : MMWL_fileDwld Fail. File Buffer is NULL \n\n\r",
      deviceMap
    );
    return -1;
  }

  /*Download to Device*/
  usChunkLen = MMWL_FW_CHUNK_SIZE;
  iNumChunks = (imgLen + 8) / usChunkLen;
  mmwl_iRemChunks = iNumChunks;

  if (mmwl_iRemChunks > 0) {
    usLastChunkLen = (imgLen + 8) % usChunkLen;
    usFirstChunkLen = MMWL_FW_CHUNK_SIZE;
    mmwl_iRemChunks += 1;
  } else {
    usFirstChunkLen = imgLen + 8;
  }

  *((unsigned int*)&firstChunk[0]) = (unsigned int)MMWL_FILETYPE_META_IMG;
  *((unsigned int*)&firstChunk[4]) = (unsigned int)imgLen;
  memcpy((char*)&firstChunk[8], (char*)pmmwl_imgBuffer, usFirstChunkLen - 8);

  ret_val = MMWL_fileWrite(deviceMap, (mmwl_iRemChunks-1), usFirstChunkLen,
                            firstChunk);
  if (ret_val < 0) {
    DEBUG_PRINT("Failed first chunk\n!");
      DEBUG_PRINT(
        "Device map %u : MMWL_fileDwld Fail. Ftype: %d\n\n\r",
        deviceMap, MMWL_FILETYPE_META_IMG
      );
      return ret_val;
  } else DEBUG_PRINT("Firt chunk sent!\n");
  pmmwl_imgBuffer += MMWL_FW_FIRST_CHUNK_SIZE;
  mmwl_iRemChunks--;

  if(mmwl_iRemChunks > 0) {
      DEBUG_PRINT("Device map %u : Download in Progress: ", deviceMap);
  }
  /*Remaining Chunk*/
  while (mmwl_iRemChunks > 0) {
    usProgress = (((iNumChunks - mmwl_iRemChunks) * 100) / iNumChunks);
    DEBUG_PRINT("%d%%..", usProgress);

    /* Last chunk */
    if ((mmwl_iRemChunks == 1) && (usLastChunkLen > 0)) {
      ret_val = MMWL_fileWrite(deviceMap, 0, usLastChunkLen, pmmwl_imgBuffer);
      if (ret_val < 0) {
        DEBUG_PRINT(
          "Device map %u : MMWL_fileDwld last chunk Fail : Ftype: %d\n\n\r",
          deviceMap, MMWL_FILETYPE_META_IMG
        );
        return ret_val;
      }
    }
    else {
      ret_val = MMWL_fileWrite(deviceMap, (mmwl_iRemChunks - 1),
        MMWL_FW_CHUNK_SIZE, pmmwl_imgBuffer);

      if (ret_val < 0) {
        DEBUG_PRINT(
          "\n\n\r Device map %u : MMWL_fileDwld rem chunk Fail : Ftype: %d\n\n\r",
          deviceMap, MMWL_FILETYPE_META_IMG
        );
        return ret_val;
      }
      pmmwl_imgBuffer += MMWL_FW_CHUNK_SIZE;
    }

    mmwl_iRemChunks--;
  }
  DEBUG_PRINT("Done!\n\n");
  return ret_val;
}


/** @fn int MMWL_firmwareDownload(deviceMap)
*
*   @brief Firmware Download API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Firmware Download API.
*/
int MMWL_firmwareDownload(unsigned char deviceMap) {
  int retVal = RL_RET_CODE_OK, timeOutCnt = 0;

  /* Meta Image download */
  DEBUG_PRINT("Device map %u : Meta Image (size %d bytes) download started\n\n",
    deviceMap, MMWL_META_IMG_FILE_SIZE);
  retVal = MMWL_fileDownload(deviceMap, MMWL_META_IMG_FILE_SIZE);
  DEBUG_PRINT(
    "Device map %u : Meta Image download complete ret = %d\n\n",
    deviceMap, retVal
  );

  return retVal;
}


/** @fn int MMWL_rfEnable(deviceMap)
*
*   @brief RFenable API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RFenable API.
*/
int MMWL_rfEnable(unsigned char deviceMap) {
  int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
  retVal = CALL_API(API_TYPE_B | RF_START_IND, deviceMap, NULL, 0);
  while ((mmwl_bStartComp & deviceMap) != deviceMap) {
    msleep(1); /*Sleep 1 msec*/
    timeOutCnt++;
    if (timeOutCnt > MMWL_API_START_TIMEOUT) {
      DEBUG_PRINT(
        "Device map %u : Timeout! RF Enable Status = %u\n\n",
        (unsigned int)deviceMap, mmwl_bStartComp
      );
      retVal = RL_RET_CODE_RESP_TIMEOUT;
      break;
    }
  }

  mmwl_bStartComp = mmwl_bStartComp & (~deviceMap);

  if(retVal == RL_RET_CODE_OK) {
    for (int devId = 0; devId < 4; devId++) {
      if ((deviceMap & (1 << devId)) != 0) {
        unsigned char devMap = createDevMapFromDevId(devId);
        rlVersion_t verArgs = { 0 };
        rlRfDieIdCfg_t dieId = { 0 };
        retVal = CALL_API(RF_GET_VERSION_IND, devMap, &verArgs, 0);

        DEBUG_PRINT(
          "Device map %u : RF Version [%2d.%2d.%2d.%2d] \nDevice map %u : MSS version [%2d.%3d.%2d.%3d] \nDevice map %u : mmWaveLink version [%2d.%2d.%2d.%2d]\n\n",
          devMap, verArgs.rf.fwMajor, verArgs.rf.fwMinor, verArgs.rf.fwBuild, verArgs.rf.fwDebug,
          devMap, verArgs.master.fwMajor, verArgs.master.fwMinor, verArgs.master.fwBuild, verArgs.master.fwDebug,
          devMap, verArgs.mmWaveLink.major, verArgs.mmWaveLink.minor, verArgs.mmWaveLink.build, verArgs.mmWaveLink.debug
        );

        DEBUG_PRINT(
          "Device map %u : RF Patch Version [%2d.%2d.%2d.%2d] \nDevice map %u : MSS Patch version [%2d.%2d.%2d.%2d]\n\n",
          devMap, verArgs.rf.patchMajor, verArgs.rf.patchMinor,
          ((verArgs.rf.patchBuildDebug & 0xF0) >> 4), (verArgs.rf.patchBuildDebug & 0x0F),
          devMap, verArgs.master.patchMajor, verArgs.master.patchMinor,
          ((verArgs.master.patchBuildDebug & 0xF0) >> 4), (verArgs.master.patchBuildDebug & 0x0F)
        );

        retVal = CALL_API(RF_GET_DIE_ID_IND, devMap, &dieId, 0);
      }
    }
  }
  return retVal;
}


/** @fn int MMWL_dataFmtConfig(unsigned char deviceMap)
*
*   @brief Data Format Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Data Format Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_dataFmtConfig(unsigned char deviceMap, rlDevDataFmtCfg_t dataFmtCfgArgs) {
  int retVal = RL_RET_CODE_OK;
  retVal = CALL_API(SET_DATA_FORMAT_CONFIG_IND, deviceMap, &dataFmtCfgArgs, 0);
  return retVal;
}


/** @fn int MMWL_ldoBypassConfig(unsigned char deviceMap)
*
*   @brief LDO Bypass Config API
*
*   @return Success - 0, Failure - Error Code
*
*   LDO Bypass Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_ldoBypassConfig(unsigned char deviceMap, rlRfLdoBypassCfg_t rfLdoBypassCfgArgs) {
  int retVal = RL_RET_CODE_OK;
  DEBUG_PRINT(
    "Device map %u : Calling rlRfSetLdoBypassConfig With Bypass [%d] \n\n",
    deviceMap, rfLdoBypassCfgArgs.ldoBypassEnable
  );

  retVal = CALL_API(SET_LDO_BYPASS_CONFIG_IND, deviceMap, &rfLdoBypassCfgArgs, 0);
  return retVal;
}


/** @fn int MMWL_adcOutConfig(unsigned char deviceMap)
*
*   @brief ADC Configuration API
*
*   @return Success - 0, Failure - Error Code
*
*   ADC Configuration API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_adcOutConfig(unsigned char deviceMap, rlAdcOutCfg_t adcOutCfgArgs) {
  int retVal = RL_RET_CODE_OK;
  DEBUG_PRINT(
    "Device map %u : Calling rlSetAdcOutConfig With [%d]ADC Bits and [%d]ADC Format \n\n",
    deviceMap, adcOutCfgArgs.fmt.b2AdcBits, adcOutCfgArgs.fmt.b2AdcOutFmt
  );
  retVal = CALL_API(SET_ADC_OUT_IND, deviceMap, &adcOutCfgArgs, 0);
  return retVal;
}


/** @fn int MMWL_RFDeviceConfig(unsigned char deviceMap)
*
*   @brief RF Device Configuration API
*
*   @return Success - 0, Failure - Error Code
*
*   RF Device Configuration API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_RFDeviceConfig(unsigned char deviceMap) {
  int retVal = RL_RET_CODE_OK;

  rlRfDevCfg_t rfDevCfgArgs      = { 0 };
  rfDevCfgArgs.aeDirection       = 0x5;
  rfDevCfgArgs.aeControl        = 0x0;
  rfDevCfgArgs.bssAnaControl     = 0x0; /* Clear Inter burst power save */
  rfDevCfgArgs.reserved1         = 0x0;
  rfDevCfgArgs.bssDigCtrl        = 0x0; /* Disable BSS WDT */
  rfDevCfgArgs.aeCrcConfig       = gAwr2243CrcType;
  rfDevCfgArgs.reserved2         = 0x0;
  rfDevCfgArgs.reserved3         = 0x0;

  DEBUG_PRINT(
    "Device map %u : Calling rlRfSetDeviceCfg With bssAnaControl = [%d] and bssDigCtrl = [%d]\n\n",
    deviceMap, rfDevCfgArgs.bssAnaControl, rfDevCfgArgs.bssDigCtrl
  );

  retVal = CALL_API(RF_SET_DEVICE_CONFIG_IND, deviceMap, &rfDevCfgArgs, 0);
  return retVal;
}


/** @fn int MMWL_channelConfig(unsigned char deviceMap, unsigned short cascading, rlChanCfg_t rfChanCfgArgs)
*
*   @brief Channel Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Channel Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_channelConfig(unsigned char deviceMap, unsigned short cascade, rlChanCfg_t rfChanCfgArgs) {
  int retVal = RL_RET_CODE_OK;

#if (ENABLE_TX2)
  rfChanCfgArgs.txChannelEn |= (1 << 2); // Enable TX2
#endif

  if(cascade == 2) {
    rfChanCfgArgs.cascadingPinoutCfg &= ~(1U << 5U); /* Disable OSC CLK OUT for slaves */ 
  }
  DEBUG_PRINT(
    "Device map %u : Calling rlSetChannelConfig With [%d]Rx and [%d]Tx Channel Enabled \n\n",
    deviceMap, rfChanCfgArgs.rxChannelEn, rfChanCfgArgs.txChannelEn
  );

  retVal = CALL_API(SET_CHANNEL_CONFIG_IND, deviceMap, &rfChanCfgArgs, 0);
  return retVal;
}


/** @fn int MMWL_setMiscConfig(unsigned char deviceMap)
*
*   @brief Sets misc feature such as per chirp phase shifter and Advance chirp
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Sets misc feature such as per chirp phase shifter and Advance chirp
*/
int MMWL_setMiscConfig(unsigned char deviceMap, rlRfMiscConf_t miscCfg) {
  int32_t retVal;
  retVal = CALL_API(RF_SET_MISC_CONFIG_IND, deviceMap, &miscCfg, 0);
  return retVal;
}


/** @fn int MMWL_setDeviceCrcType(unsigned char deviceMap)
*
*   @brief Set CRC type of async event from AWR2243 MasterSS
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Set CRC type of async event from AWR2243 MasterSS
*/
int MMWL_setDeviceCrcType(unsigned char deviceMap) {
  int32_t retVal;
  rlDevMiscCfg_t devMiscCfg = {0};
  /* Set the CRC Type for Async Event from MSS */
  devMiscCfg.aeCrcConfig = gAwr2243CrcType;
  retVal = CALL_API(SET_MISC_CONFIG_IND, deviceMap, &devMiscCfg, 0);
  return retVal;
}


/** @fn int MMWL_basicConfiguration(unsigned char deviceMap)
*
*   @brief Channel, ADC,Data format configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Channel, ADC,Data format configuration API.
*/
int MMWL_basicConfiguration(unsigned char deviceMap,
        rlAdcOutCfg_t adcOutCfgArgs,
        rlDevDataFmtCfg_t dataFmtCfgArgs, rlRfLdoBypassCfg_t ldoCfgArgs) {
  int retVal = RL_RET_CODE_OK;

  /* ADC out data format configuration */
  retVal = MMWL_adcOutConfig(deviceMap, adcOutCfgArgs);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "Device map %u : AdcOut Config failed with error code %d\n\n",
      deviceMap, retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT("Device map %u : AdcOut Configuration success\n\n", deviceMap);
  }

  /* RF device configuration */
  retVal = MMWL_RFDeviceConfig(deviceMap);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "Device map %u : RF Device Config failed with error code %d\n\n",
      deviceMap, retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT("Device map %u : RF Device Configuration success\n\n", deviceMap);
  }

  /* LDO bypass configuration */
  retVal = MMWL_ldoBypassConfig(deviceMap, ldoCfgArgs);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "Device map %u : LDO Bypass Config failed with error code %d\n\n",
      deviceMap, retVal
    );
    return -1;
  }
  else {
      DEBUG_PRINT("Device map %u : LDO Bypass Configuration success\n\n", deviceMap);
  }

  /* Data format configuration */
  retVal = MMWL_dataFmtConfig(deviceMap, dataFmtCfgArgs);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "Device map %u : Data format Configuration failed with error code %d\n\n",
      deviceMap, retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT("Device map %u : Data format Configuration success\n\n", deviceMap);
  }

  /* low power configuration */
  // TODO: Define default value for low power config
  rlLowPowerModeCfg_t rfLpModeCfgArgs = {0};
  retVal = MMWL_lowPowerConfig(deviceMap, rfLpModeCfgArgs);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "Device map %u : Low Power Configuration failed with error %d \n\n",
      deviceMap, retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT("Device map %u : Low Power Configuration success\n\n", deviceMap);
  }

  /* APLL Synth BW configuration */
  retVal = MMWL_ApllSynthBwConfig(deviceMap);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "Device map %u : APLL Synth BW Configuration failed with error %d \n\n",
      deviceMap, retVal
    );
    return -1;
  }
  else {
      DEBUG_PRINT("Device map %u : APLL Synth BW Configuration success\n\n", deviceMap);
  }

  return retVal;
}


/** @fn int MMWL_rfInit(unsigned char deviceMap)
*
*   @brief RFinit API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RFinit API.
*/
int MMWL_rfInit(unsigned char deviceMap) {
  int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
  mmwl_bRfInitComp = mmwl_bRfInitComp & (~deviceMap);

  // if (rlDevGlobalCfgArgs.CalibEnable == TRUE) {
  rlRfInitCalConf_t rfCalibCfgArgs = { 0 };

  /* Enable only required boot-time calibrations, by default all are enabled in the device */
  rfCalibCfgArgs.calibEnMask = 0x1FF0;

  /* RF Init Calibration Configuration */
  retVal = CALL_API(RF_INIT_CALIB_CONFIG_IND, deviceMap, &rfCalibCfgArgs, 0);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT("Device map %u : RF Init Calibration Configuration failed with error %d \n\n",
      deviceMap, retVal);
    return -1;
  }
  else {
    DEBUG_PRINT("Device map %u : RF Init Calibration Configuration success \n\n", deviceMap);
  }

  /* Run boot time calibrations */
  retVal = CALL_API(API_TYPE_B | RF_INIT_IND, deviceMap, NULL, 0);
  while ((mmwl_bRfInitComp & deviceMap) != deviceMap) {
    msleep(1); /*Sleep 1 msec*/
    timeOutCnt++;
    if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT) {
      retVal = RL_RET_CODE_RESP_TIMEOUT;
      break;
    }
  }

  mmwl_bRfInitComp = mmwl_bRfInitComp & (~deviceMap);
  return retVal;
}


/** @fn int MMWL_profileConfig(unsigned char deviceMap)
*
*   @brief Profile configuration API.
*
*   @param[in] deviceMap - Devic Index
*   @param[in] profileCfgArgs - Profile configuration
*
*   @return int Success - 0, Failure - Error Code
*
*   Profile configuration API.
*/
int MMWL_profileConfig(unsigned char deviceMap, rlProfileCfg_t profileCfgArgs) {
  int retVal = RL_RET_CODE_OK, i;
  DEBUG_PRINT(
    "Device map %u : Calling rlSetProfileConfig with \nProfileId[%d]\nStart Frequency[%f] GHz\nRamp Slope[%f] MHz/uS \n\n",
    deviceMap, profileCfgArgs.profileId,
    (float)((profileCfgArgs.startFreqConst * 53.6441803) / (1000 * 1000 * 1000)),
    (float)(profileCfgArgs.freqSlopeConst * 48.2797623) / 1000.0
  );

  /* with this API we can configure 2 profiles (max 4 profiles) at a time */
  retVal = CALL_API(API_TYPE_C | SET_PROFILE_CONFIG_IND, deviceMap, &profileCfgArgs, 1U);
  return retVal;
}


/** @fn int MMWL_chirpConfig(unsigned char deviceMap)
*
*   @brief Chirp configuration API.
*
*   @param[in] deviceMap - Devic Index
*   @param[in] chirpCfgArgs - Chrip configuration
*
*   @return int Success - 0, Failure - Error Code
*
*   Chirp configuration API.
*/
int MMWL_chirpConfig(unsigned char deviceMap, rlChirpCfg_t chirpCfgArgs) {
  int retVal = RL_RET_CODE_OK;
  DEBUG_PRINT(
    "Device map %u : Calling rlSetChirpConfig with \nProfileId[%d]\nStart Idx[%d]\nEnd Idx[%d] | Tx [%d]\n\n",
    deviceMap, chirpCfgArgs.profileId,
    chirpCfgArgs.chirpStartIdx,
    chirpCfgArgs.chirpEndIdx,
    chirpCfgArgs.txEnable
  );

  /* With this API we can configure max 512 chirp in one call */
  retVal = CALL_API(API_TYPE_C | SET_CHIRP_CONFIG_IND, deviceMap, &chirpCfgArgs, 1U);
  return retVal;
}


/** @fn int MMWL_frameConfig(unsigned char deviceMap)
*
*   @brief Frame configuration API.
*
*   @param[in] deviceMap - Devic Index
*   @param[in] frameCfgArgs - Frame config
*   @param[in] rfChanCfgArgs - Channel config
*   @param[in] adcOutCfgArgs - ADC output config
*   @param[in] dataPathCfgArgs - Datapath config
*   @param[in] profileCfgArgs - Profile config
*
*   @return int Success - 0, Failure - Error Code
*
*   Frame configuration API.
*/
int MMWL_frameConfig(unsigned char deviceMap, rlFrameCfg_t frameCfgArgs, rlChanCfg_t rfChanCfgArgs,
      rlAdcOutCfg_t adcOutCfgArgs, rlDevDataPathCfg_t dataPathCfgArgs, rlProfileCfg_t profileCfgArgs) {

  int retVal = RL_RET_CODE_OK;
  unsigned char devId;
  if (deviceMap == 1) {
    frameCfgArgs.triggerSelect = 1; // Software trigger
  }
  else {
    frameCfgArgs.triggerSelect = 2; // Hardware trigger
  }

  framePeriodicity = (frameCfgArgs.framePeriodicity * 5)/(1000*1000);
  frameCount = frameCfgArgs.numFrames;

  /* In Adv chirp context, frame start and frame end index is not used, the number 
     of chirps is taken from number of loops
  if (rlDevGlobalCfgArgs.LinkAdvChirpTest == TRUE) {
    frameCfgArgs.numLoops = frameCfgArgs.numLoops * (frameCfgArgs.chirpEndIdx - frameCfgArgs.chirpStartIdx + 1);
    frameCfgArgs.chirpEndIdx = 0;
    frameCfgArgs.chirpStartIdx = 0;
  }
  */

  DEBUG_PRINT(
    "Device map %u : Calling rlSetFrameConfig with \nStart Idx[%d]\nEnd Idx[%d]\nLoops[%d]\nPeriodicity[%d]ms \n\n",
    deviceMap, frameCfgArgs.chirpStartIdx, frameCfgArgs.chirpEndIdx,
    frameCfgArgs.numLoops, (frameCfgArgs.framePeriodicity * 5)/(1000*1000)
  );

  retVal = CALL_API(SET_FRAME_CONFIG_IND, deviceMap, &frameCfgArgs, 0);

  for (devId = 0; devId < 4; devId++) {
    if ((deviceMap & (1 << devId)) != 0) {
      /* Height calculation */
      mmwl_TDA_height[devId] = frameCfgArgs.numLoops * (frameCfgArgs.chirpEndIdx - frameCfgArgs.chirpStartIdx + 1);
      DEBUG_PRINT("Device map %u : Calculated TDA Height is %d\n\n", deviceMap, mmwl_TDA_height[devId]);

      uint8_t rxChannelEn = rfChanCfgArgs.rxChannelEn;
      /* Width calculation */
      /* Count the number of Rx antenna */
      unsigned char numRxAntenna = 0;
      while (rxChannelEn != 0) {
        if ((rxChannelEn & 0x1) == 1) {
          numRxAntenna++;
        }
        rxChannelEn = (rxChannelEn >> 1);
      }

      /* ADC format (in bytes) */
      unsigned char numValPerAdcSample = 0, numAdcBits = 0;
      if (adcOutCfgArgs.fmt.b2AdcOutFmt == 1 || adcOutCfgArgs.fmt.b2AdcOutFmt == 2) {
        numValPerAdcSample = 2;
      }
      else {
        numValPerAdcSample = 1;
      }

      if (adcOutCfgArgs.fmt.b2AdcBits == 0) {
        numAdcBits = 12;
      }
      else if (adcOutCfgArgs.fmt.b2AdcBits == 1) {
        numAdcBits = 14;
      }
      else if (adcOutCfgArgs.fmt.b2AdcBits == 2) {
        numAdcBits = 16;
      }

      /* Number of ADC samples */
      unsigned int numAdcSamples = 0;
      // if (rlDevGlobalCfgArgs.LinkAdvChirpTest == FALSE) {
      numAdcSamples = profileCfgArgs.numAdcSamples;
      // }

      /* Datapath */
      /* Get CP and CQ value */
      unsigned short cp_data = 0, cq_val = 0;
      unsigned int cq_data = 0;
      cq_val = dataPathCfgArgs.cq0TransSize + dataPathCfgArgs.cq1TransSize + dataPathCfgArgs.cq2TransSize;

      if (dataPathCfgArgs.transferFmtPkt0 == 6 || dataPathCfgArgs.transferFmtPkt0 == 9) {
        cp_data = 2;
        cq_data = 0;
      }
      else if (dataPathCfgArgs.transferFmtPkt0 == 54) {
        cp_data = 2;
        cq_data = (cq_val * 16) / numAdcBits;
      }

      DEBUG_PRINT("params: Num val per samples: %d, Num ADC samples: %u, num RX: %d\n", numValPerAdcSample, numAdcSamples, numRxAntenna);
      mmwl_TDA_width[devId] = (((numValPerAdcSample * numAdcSamples) + cp_data) * numRxAntenna) + cq_data;
      DEBUG_PRINT("Device map %u : Calculated TDA Width is %d\n\n", deviceMap, mmwl_TDA_width[devId]);
    }
  }
  return retVal;
}


/** @fn int MMWL_dataPathConfig(unsigned char deviceMap)
*
*   @brief Data path configuration API. Configures CQ data size on the
*           lanes and number of samples of CQ[0-2] to be transferred.
*
*   @param[in] deviceMap - Devic Index
*   @param[in] dataPathCfgArgs - Data path configuration
*
*   @return int Success - 0, Failure - Error Code
*
*   Data path configuration API. Configures CQ data size on the
*   lanes and number of samples of CQ[0-2] to be transferred.
*/
int MMWL_dataPathConfig(unsigned char deviceMap, rlDevDataPathCfg_t dataPathCfgArgs) {
  int retVal = RL_RET_CODE_OK;

  DEBUG_PRINT(
    "Device map %u : Calling rlDeviceSetDataPathConfig with HSI Interface[%d] Selected \n\n",
    deviceMap, dataPathCfgArgs.intfSel
  );

  /* same API is used to configure CQ data size on the
    * lanes and number of samples of CQ[0-2] to be transferred.
    */
  retVal = CALL_API(SET_DATA_PATH_CONFIG_IND, deviceMap, &dataPathCfgArgs, 0);
  return retVal;
}


#if defined (LVDS_ENABLE)
/** @fn int MMWL_lvdsLaneConfig(unsigned char deviceMap)
*
*   @brief Lane Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Lane Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_lvdsLaneConfig(unsigned char deviceMap, rlDevLaneEnable_t laneEnCfgArgs) {
    int retVal = RL_RET_CODE_OK;

  retVal = CALL_API(SET_LVDS_LANE_CONFIG_IND, deviceMap, &lvdsLaneCfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_laneConfig(unsigned char deviceMap)
*
*   @brief Lane Enable API
*
*   @return Success - 0, Failure - Error Code
*
*   Lane Enable API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_laneConfig(unsigned char deviceMap, rlDevLaneEnable_t laneEnCfgArgs) {
    int retVal = RL_RET_CODE_OK;

  retVal = CALL_API(SET_LANE_CONFIG_IND, deviceMap, &laneEnCfgArgs, 0);
    return retVal;
}
#else
/** @fn int MMWL_CSI2LaneConfig(unsigned char deviceMap)
*
*   @brief CSI2 Lane Config API
*
* @param deviceMap - Devic Index
* @param CSI2LaneCfgArgs - CSI2 Lane configuration
*
*   @return Success - 0, Failure - Error Code
*
*   CSI2 Lane Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_CSI2LaneConfig(unsigned char deviceMap, rlDevCsi2Cfg_t CSI2LaneCfgArgs) {
  int retVal = RL_RET_CODE_OK;

  retVal = CALL_API(SET_CSI2_CONFIG_IND, deviceMap, &CSI2LaneCfgArgs, 0);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT("Device map %u : CSI2LaneConfig failed with error code %d\n\n",
      deviceMap, retVal);
    return -1;
  }
  else {
    DEBUG_PRINT("Device map %u : CSI2LaneConfig success\n\n", deviceMap);
  }
  return retVal;
}
#endif


/** @fn int MMWL_setHsiClock(unsigned char deviceMap)
*
*   @brief High Speed Interface Clock Config API
*
*   @return Success - 0, Failure - Error Code
*
*   HSI Clock Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_setHsiClock(unsigned char deviceMap, rlDevHsiClk_t hsiClkgs) {
  int retVal = RL_RET_CODE_OK;

  DEBUG_PRINT(
    "Device map %u : Calling rlDeviceSetHsiClk with HSI Clock[%d] \n\n",
    deviceMap, hsiClkgs.hsiClk
  );

  retVal = CALL_API(SET_HSI_CLK_IND, deviceMap, &hsiClkgs, 0);
  return retVal;
}


/** @fn int MMWL_hsiDataRateConfig(unsigned char deviceMap)
*
*   @brief LVDS/CSI2 Clock Config API
*
*   @return Success - 0, Failure - Error Code
*
*   LVDS/CSI2 Clock Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_hsiDataRateConfig(unsigned char deviceMap, rlDevDataPathClkCfg_t dataPathClkCfgArgs) {
  int retVal = RL_RET_CODE_OK;

  DEBUG_PRINT(
    "Device map %u : Calling rlDeviceSetDataPathClkConfig with HSI Data Rate[%d] Selected \n\n",
    deviceMap, dataPathClkCfgArgs.dataRate
  );

  retVal = CALL_API(SET_DATA_PATH_CLK_CONFIG_IND, deviceMap, &dataPathClkCfgArgs, 0);
  return retVal;
}


/** @fn int MMWL_hsiClockConfig(unsigned char deviceMap)
*
*   @brief Clock configuration API.
*
*   @param[in] deviceMap - Devic Index
*   @param[in] dataPathClkCfgArgs - Datapath clock config
*   @param[in] hsiClkgs - High speed clock config
*
*   @return int Success - 0, Failure - Error Code
*
*   Clock configuration API.
*/
int MMWL_hsiClockConfig(unsigned char deviceMap, rlDevDataPathClkCfg_t dataPathClkCfgArgs, rlDevHsiClk_t hsiClkgs) {
  int retVal = RL_RET_CODE_OK, readAllParams = 0;

  /*LVDS clock configuration*/
  retVal = MMWL_hsiDataRateConfig(deviceMap, dataPathClkCfgArgs);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "Device map %u : LvdsClkConfig failed with error code %d\n\n",
      deviceMap, retVal
    );
    return -1;
  }
  else {
      DEBUG_PRINT("Device map %u : MMWL_hsiDataRateConfig success\n\n", deviceMap);
  }

  /*set high speed clock configuration*/
  retVal = MMWL_setHsiClock(deviceMap, hsiClkgs);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "Device map %u : MMWL_setHsiClock failed with error code %d\n\n",
      deviceMap, retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT("Device map %u : MMWL_setHsiClock success\n\n", deviceMap);
  }

  return retVal;
}


/** @fn int MMWL_gpadcMeasConfig(unsigned char deviceMap)
*
*   @brief API to set GPADC configuration.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code.
*
*   API to set GPADC Configuration. And device will    send GPADC
*    measurement data in form of Asynchronous event over SPI to
*    Host. User needs to feed input signal on the device pins where
*    they want to read the measurement data inside the device.
*/
int MMWL_gpadcMeasConfig(unsigned char deviceMap) {
  int retVal = RL_RET_CODE_OK;
  int timeOutCnt = 0;
  rlGpAdcCfg_t gpadcCfg = {0};

  /* enable all the sensors [0-5] to read gpADC measurement data */
  gpadcCfg.enable = 0x3F;
  /* set the number of samples device needs to collect to do the measurement */
  gpadcCfg.numOfSamples[0].sampleCnt = 32;
  gpadcCfg.numOfSamples[1].sampleCnt = 32;
  gpadcCfg.numOfSamples[2].sampleCnt = 32;
  gpadcCfg.numOfSamples[3].sampleCnt = 32;
  gpadcCfg.numOfSamples[4].sampleCnt = 32;
  gpadcCfg.numOfSamples[5].sampleCnt = 32;

  retVal = CALL_API(SET_GPADC_CONFIG, deviceMap, &gpadcCfg, 0);
  if(retVal == RL_RET_CODE_OK) {
    while ((mmwl_bGpadcDataRcv & deviceMap) != deviceMap) {
      msleep(1); /*Sleep 1 msec*/
      timeOutCnt++;
      if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT) {
        retVal = RL_RET_CODE_RESP_TIMEOUT;
        break;
      }
    }
  }

  return retVal;
}


/** @fn int MMWL_sensorStart(unsigned char deviceMap)
*
*   @brief API to Start sensor.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Start sensor.
*/
int MMWL_sensorStart(unsigned char deviceMap) {
  int retVal = RL_RET_CODE_OK;
  int timeOutCnt = 0;

  rlFrameTrigger_t data = { 0 };
  /* Start the frame */
  data.startStop = 0x1;
  mmwl_bSensorStarted = mmwl_bSensorStarted & (~deviceMap);
  retVal = CALL_API(SENSOR_START_STOP_IND, deviceMap, &data, 0);
  while ((mmwl_bSensorStarted & deviceMap) != deviceMap) {
    msleep(1); /*Sleep 1 msec*/
    timeOutCnt++;
    if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT) {
      retVal = RL_RET_CODE_RESP_TIMEOUT;
      break;
    }
  }
  return retVal;
}


/**
 * @brief int MMWL_StartFrame(unsigned int deviceMap)
 *
 * @param deviceMap 
 * @return int 
 */
int MMWL_StartFrame(unsigned char deviceMap) {
  int retVal = RL_RET_CODE_OK;

  /*  \subsection     api_sequence14     Seq 14 - Start mmWave Radar Sensor
  This will trigger the mmWave Front to start transmitting FMCW signal. Raw ADC samples
  would be received from Digital front end. For AWR2243, if high speed interface is
  configured, RAW ADC data would be transmitted over CSI2/LVDS. On xWR1443/xWR1642, it can
  be processed using HW accelerator or DSP
  */
  retVal = MMWL_sensorStart(deviceMap);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "Device map %u : Sensor Start failed with error code %d \n\n",
      deviceMap, retVal
    );
    return -1;
  }

  return retVal;
}


/** @fn int MMWL_sensorStop(unsigned char deviceMap)
*
*   @brief API to Stop sensor.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Stop Sensor.
*/
int MMWL_sensorStop(unsigned char deviceMap) {
  int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
  rlFrameTrigger_t data = { 0 };
  /* Stop the frame after the current frame is over */
  data.startStop = 0;
  retVal = CALL_API(SENSOR_START_STOP_IND, deviceMap, &data, 0);
  if (retVal == RL_RET_CODE_OK) {
    while ((mmwl_bSensorStarted & deviceMap) == deviceMap) {
      msleep(1); /*Sleep 1 msec*/
      timeOutCnt++;
      if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT) {
        retVal = RL_RET_CODE_RESP_TIMEOUT;
        break;
      }
    }
  }
  return retVal;
}


/**
 * @brief int MMWL_StopFrame(unsigned int deviceMap)
 *
 * @param deviceMap 
 * @return int 
 */
int MMWL_StopFrame(unsigned char deviceMap) {
  int retVal = RL_RET_CODE_OK;

  /* Stop the frame */
  retVal = MMWL_sensorStop(deviceMap);
  if (retVal != RL_RET_CODE_OK) {
    if (retVal == RL_RET_CODE_FRAME_ALREADY_ENDED) {
      DEBUG_PRINT(
        "Device map %u : Frame is already stopped when sensorStop CMD was issued\n\n",
        deviceMap
      );
    }
    else {
      DEBUG_PRINT(
        "Device map %u : Sensor Stop failed with error code %d \n\n",
        deviceMap, retVal
      );
      return -1;
    }
  }
  else {
    DEBUG_PRINT("Device map %u : Sensor Stop successful\n\n", deviceMap);
  }

  return retVal;
}


/** @fn int MMWL_powerOff(unsigned char deviceMap)
*
*   @brief API to poweroff device.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to poweroff device.
*/
int MMWL_powerOff(unsigned char deviceMap) {
  int retVal = RL_RET_CODE_OK;

  if (deviceMap == 1) {
    retVal = rlDevicePowerOff();
    if (retVal != RL_RET_CODE_OK) {
      DEBUG_PRINT("Device map %u : Power Off API failed with error code %d \n\n",
        deviceMap, retVal);
      return -1;
    }
    else {
      DEBUG_PRINT("Device map %u : Power Off API success\n\n", deviceMap);
      mmwl_bInitComp = mmwl_bInitComp & (~deviceMap);
      mmwl_bStartComp = mmwl_bStartComp & (~deviceMap);
      mmwl_bRfInitComp = mmwl_bRfInitComp & (~deviceMap);
      pthread_mutex_destroy(&rlAsyncEvent);
    }  
  }
  else {
    retVal = CALL_API(API_TYPE_B | REMOVE_DEVICE_IND, deviceMap, NULL, 0);
    if (retVal != RL_RET_CODE_OK) {
      DEBUG_PRINT(
        "Device map %u : Power Off API failed with error code %d \n\n",
        deviceMap, retVal
      );
      return -1;
    }
    else {
      DEBUG_PRINT("Device map %u : Power Off API success\n\n", deviceMap);
      mmwl_bInitComp = mmwl_bInitComp & (~deviceMap);
      mmwl_bStartComp = mmwl_bStartComp & (~deviceMap);
      mmwl_bRfInitComp = mmwl_bRfInitComp & (~deviceMap);
    } 
  }   

  return retVal;
}


/** @fn int MMWL_lowPowerConfig(deviceMap)
*
*   @brief LowPower configuration API.
*
*   @param[in] deviceMap - Devic Index
*   @param[in] rfLpModeCfgArgs - Low power config parameters
*
*   @return int Success - 0, Failure - Error Code
*
*   LowPower configuration API.
*/
int MMWL_lowPowerConfig(unsigned char deviceMap, rlLowPowerModeCfg_t rfLpModeCfgArgs) {
  int retVal = RL_RET_CODE_OK;

  retVal = CALL_API(SET_LOW_POWER_MODE_IND, deviceMap, &rfLpModeCfgArgs, 0);
  return retVal;
}


/** @fn int MMWL_ApllSynthBwConfig(deviceMap)
*
*   @brief APLL Synth BW configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   APLL Synth BW configuration API.
*/
int MMWL_ApllSynthBwConfig(unsigned char deviceMap) {
  int retVal = RL_RET_CODE_OK;

  rlRfApllSynthBwControl_t rfApllSynthBwCfgArgs = { 0 };
  rfApllSynthBwCfgArgs.synthIcpTrim = 3;
  rfApllSynthBwCfgArgs.synthRzTrim = 8;
  rfApllSynthBwCfgArgs.apllIcpTrim = 38;
  rfApllSynthBwCfgArgs.apllRzTrimLpf = 9;
  rfApllSynthBwCfgArgs.apllRzTrimVco = 0;
    
  retVal = CALL_API(RF_SET_APLL_SYNTH_BW_CTL_CONFIG_IND, deviceMap, &rfApllSynthBwCfgArgs, 0);
  return retVal;
}


/**
 * @brief Power up device
 * 
 * @param deviceMap - Device Index
 * @param rlClientCbsTimeout - Timeout to use for mmwavelink client
 * @param sopTimeout - Timeout after setting SOP mode in ms
 * @return int 
 */
int MMWL_DevicePowerUp(unsigned char deviceMap, uint32_t rlClientCbsTimeout, uint32_t sopTimeout) {
  int retVal = RL_RET_CODE_OK;
  TDADevHandle_t TDAImpl_devHdl = NULL;
  unsigned int devId = getDevIdFromDevMap(deviceMap);
  TDAImpl_devHdl = TDAGetDeviceCtx(devId);

  int SOPmode = 4;         /* Only SOP4 is supported for cascade */

  /* Set SOP Mode for the devices */
  if (TDAImpl_devHdl != NULL) {
    retVal = setSOPMode(TDAImpl_devHdl, SOPmode);
    msleep(1); // Additional 1 msec delay
    msleep(sopTimeout); // Additional delay
  }
  else {
    DEBUG_PRINT("Device map %u : Cannot get device context\n\n", deviceMap);
    return -1;
  }

  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT("Device map %u : SOP 4 mode failed with error %d\n\n", deviceMap,
      retVal);
    return -1;
  }
  else {
    DEBUG_PRINT("Device map %u : SOP 4 mode successful\n\n", deviceMap);
  }

  /* Reset the devices */
  if (TDAImpl_devHdl != NULL) {
    retVal = resetDevice(TDAImpl_devHdl);
  }
  else {
    DEBUG_PRINT("Device map %u : Cannot get device context\n\n", deviceMap);
    return -1;
  }

  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "Device map %u : Device reset failed with error %d \n\n", deviceMap,
      retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT("Device map %u : Device reset successful\n\n", deviceMap);
  }

  /*  \subsection     api_sequence1     Seq 1 - Call Power ON API
  The mmWaveLink driver initializes the internal components, creates Mutex/Semaphore,
  initializes buffers, register interrupts, bring mmWave front end out of reset.
  */
  if (deviceMap == 1) {
    retVal = MMWL_powerOnMaster(deviceMap, rlClientCbsTimeout);
    if (retVal != RL_RET_CODE_OK) {
      DEBUG_PRINT(
        "Device map %u : mmWave Device Power on failed with error %d \n\n",
        deviceMap, retVal
      );
      return -1;
    }
    else {
      DEBUG_PRINT(
        "Device map %u : mmWave Device Power on success\n\n",
        deviceMap
      );
    }
  }

  else {
    retVal = CALL_API(API_TYPE_B | ADD_DEVICE_IND, deviceMap, NULL, 0);
    int timeoutCnt = 0;
    /* TBD - Wait for Power ON complete
      @Note: In case of ES1.0 sample application needs to wait for MSS CPU fault as well with some timeout.

      TODO: Wait for MSS CPU fault
    */
    if (RL_RET_CODE_OK == retVal) {
      while ((mmwl_bInitComp & deviceMap) != deviceMap) {
        msleep(1); //Sleep 1 msec
        timeoutCnt++;
        if (timeoutCnt > MMWL_API_INIT_TIMEOUT) {
          CALL_API(API_TYPE_B | REMOVE_DEVICE_IND, deviceMap, NULL, 0);
          retVal = RL_RET_CODE_RESP_TIMEOUT;
          break;
        }
      }
    }
    mmwl_bInitComp = mmwl_bInitComp & (~deviceMap);

    if (retVal != RL_RET_CODE_OK) {
      DEBUG_PRINT(
        "Device map %u : mmWave Device Power on failed with error %d \n\n",
        deviceMap, retVal
      );
      return -1;
    }
    else {
      DEBUG_PRINT(
        "Device map %u : mmWave Device Power on success\n\n",
        deviceMap
      );
    }
  }

  return retVal;
}


/**
 * @brief Prepare the TDA board and notify TDA about the start of recording
 * 
 * @param tdaArmCfgArgs Config setup to arm the TDA for recording 
 * @return int 
 */
int MMWL_ArmingTDA(rlTdaArmCfg_t tdaArmCfgArgs) {
  int retVal = RL_RET_CODE_OK;
  int timeOutCnt = 0U;

  /* Set width and height for all devices*/
	/* Master */
  /*
	retVal = setWidthAndHeight(1, mmwl_TDA_width[0], mmwl_TDA_height[0]);
	if (retVal != RL_RET_CODE_OK) {
		DEBUG_PRINT(
      "ERROR: Device map 1 : Setting width = %u and height = %u failed with error code %d \n\n",
      mmwl_TDA_width[0], mmwl_TDA_height[0], retVal
    );
		return -1;
	}
	else {
		DEBUG_PRINT(
      "INFO: Device map 1 : Setting width = %u and height = %u successful\n\n",
      mmwl_TDA_width[0], mmwl_TDA_height[0]
    );
	}
  */
	for (int i = 0; i < 4; i++) {
    retVal = setWidthAndHeight(1 << i, mmwl_TDA_width[i], mmwl_TDA_height[i]);
    if (retVal != RL_RET_CODE_OK) {
      DEBUG_PRINT(
        "ERROR: Device map %u : Setting width = %u and height = %u failed with error code %d \n\n",
        1 << (i), mmwl_TDA_width[i], mmwl_TDA_height[i], retVal
      );
      return -1;
    }
    else {
      DEBUG_PRINT(
        "INFO: Device map %u : Setting width = %u and height = %u successful\n\n",
        1 << i, mmwl_TDA_width[i], mmwl_TDA_height[i]
      );
    }
	}



  mmwl_bTDA_FramePeriodicityACK = 0U;
  /* Send frame periodicity for syncing the data being received at VIP ports */
  retVal = sendFramePeriodicitySync(tdaArmCfgArgs.framePeriodicity);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "ERROR: Sending framePeriodicity = %u failed with error code %d \n\n",
      tdaArmCfgArgs.framePeriodicity, retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT("INFO: Sending framePeriodicity = %u successful \n\n", tdaArmCfgArgs.framePeriodicity);
  }

  while (1) {
    if (mmwl_bTDA_FramePeriodicityACK == 0U) {
      msleep(1); /*Sleep 1 msec*/
      timeOutCnt++;
      if (timeOutCnt > MMWL_API_TDA_TIMEOUT) {
        DEBUG_PRINT("ERROR: Frame Periodicity Response from Capture Card timed out!\n\n");
        retVal = RL_RET_CODE_RESP_TIMEOUT;
        return retVal;
      }
    }
    else {
      break;
    }
  }

  timeOutCnt = 0U;
  mmwl_bTDA_CaptureDirectoryACK = 0U;
  /* Send session's capture directory to TDA */
  retVal = setSessionDirectory(tdaArmCfgArgs.captureDirectory);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "ERROR: Sending capture directory = %s failed with error code %d \n\n",
      tdaArmCfgArgs.captureDirectory, retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT("INFO: Sending capture directory = %s successful \n\n", tdaArmCfgArgs.captureDirectory);
  }

  while (1) {
    if (mmwl_bTDA_CaptureDirectoryACK == 0U) {
      msleep(1); /*Sleep 1 msec*/
      timeOutCnt++;
      if (timeOutCnt > MMWL_API_TDA_TIMEOUT) {
        DEBUG_PRINT("ERROR: Capture Directory Response from Capture Card timed out!\n\n");
        retVal = RL_RET_CODE_RESP_TIMEOUT;
        return retVal;
      }
    }
    else {
      break;
    }
  }

  timeOutCnt = 0U;
  mmwl_bTDA_FileAllocationACK = 0U;
  /* Send number of files to be pre-allocated to TDA */
  retVal = sendNumAllocatedFiles(tdaArmCfgArgs.numberOfFilesToAllocate);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "ERROR: Sending pre-allocated files = %u failed with error code %d \n\n",
      tdaArmCfgArgs.numberOfFilesToAllocate, retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT(
      "INFO: Sending pre-allocated files = %u successful \n\n",
      tdaArmCfgArgs.numberOfFilesToAllocate
    );
  }

  while (1) {
    if (mmwl_bTDA_FileAllocationACK == 0U) {
      msleep(1); /*Sleep 1 msec*/
      timeOutCnt++;
      if (timeOutCnt > MMWL_API_TDA_TIMEOUT) {
        DEBUG_PRINT("ERROR: File Allocation Response from Capture Card timed out!\n\n");
        retVal = RL_RET_CODE_RESP_TIMEOUT;
        return retVal;
      }
    }
    else {
      break;
    }
  }

  timeOutCnt = 0U;
  mmwl_bTDA_DataPackagingACK = 0U;
  /* Send enable Data packing (0 : 16-bit, 1 : 12-bit) to TDA */
  retVal = enableDataPackaging(tdaArmCfgArgs.dataPacking);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "ERROR: Sending enable data packing = %u failed with error code %d \n\n",
      tdaArmCfgArgs.dataPacking, retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT(
      "INFO: Sending enable data packing = %u successful \n\n",
      tdaArmCfgArgs.dataPacking
    );
  }

  while (1) {
    if (mmwl_bTDA_DataPackagingACK == 0U) {
      msleep(1); /*Sleep 1 msec*/
      timeOutCnt++;
      if (timeOutCnt > MMWL_API_TDA_TIMEOUT) {
        DEBUG_PRINT("ERROR: Enable Data Packaging Response from Capture Card timed out!\n\n");
        retVal = RL_RET_CODE_RESP_TIMEOUT;
        return retVal;
      }
    }
    else {
      break;
    }
  }

  timeOutCnt = 0U;
  mmwl_bTDA_NumFramesToCaptureACK = 0U;
  /* Send number of frames to be captured by TDA */
  retVal = NumFramesToCapture(tdaArmCfgArgs.numberOfFramesToCapture);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "ERROR: Sending number of frames to capture = %u failed with error code %d \n\n",
      tdaArmCfgArgs.numberOfFramesToCapture, retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT(
      "INFO: Sending number of frames to capture = %u successful \n\n",
      tdaArmCfgArgs.numberOfFramesToCapture
    );
  }

  while (1) {
    if (mmwl_bTDA_NumFramesToCaptureACK == 0U) {
      msleep(1); /*Sleep 1 msec*/
      timeOutCnt++;
      if (timeOutCnt > MMWL_API_TDA_TIMEOUT) {
        DEBUG_PRINT("ERROR: Number of frames to be captured Response from Capture Card timed out!\n\n");
        retVal = RL_RET_CODE_RESP_TIMEOUT;
        return retVal;
      }
    }
    else {
      break;
    }
  }

  timeOutCnt = 0U;
  mmwl_bTDA_CreateAppACK = 0U;
  /* Notify TDA about creating the application */
  retVal = TDACreateApplication();
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT("ERROR: Notifying TDA about creating application failed with error code %d \n\n", retVal);
    return -1;
  }
  else {
    DEBUG_PRINT("INFO: Notifying TDA about creating application successful \n\n");
  }

  while (1) {
    if (mmwl_bTDA_CreateAppACK == 0U) {
      msleep(1); /*Sleep 1 msec*/
      timeOutCnt++;
      if (timeOutCnt > MMWL_API_TDA_TIMEOUT*3) {
        DEBUG_PRINT("ERROR: Create Application Response from Capture Card timed out!\n\n");
        retVal = RL_RET_CODE_RESP_TIMEOUT;
        return retVal;
      }
    }
    else {
      break;
    }
  }

  timeOutCnt = 0U;
  mmwl_bTDA_StartRecordACK = 0U;
  /* Notify TDA about starting the frame */
  retVal = startRecord();
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT("ERROR: Notifying TDA about start frame failed with error code %d \n\n", retVal);
    return -1;
  }
  else {
    DEBUG_PRINT("INFO: Notifying TDA about start frame successful \n\n");
    mmwl_bTDA_ARMDone = 1U;
  }

  while (1) {
    if (mmwl_bTDA_StartRecordACK == 0U) {
      msleep(1); /*Sleep 1 msec*/
      timeOutCnt++;
      if (timeOutCnt > MMWL_API_TDA_TIMEOUT) {
        DEBUG_PRINT("ERROR: Start Record Response from Capture Card timed out!\n\n");
        retVal = RL_RET_CODE_RESP_TIMEOUT;
        return retVal;
      }
    }
    else {
      break;
    }
  }

  return retVal;
}


/**
 * @brief Stop recrodings
 * 
 * @return int 
 */
int MMWL_DeArmingTDA() {
  int retVal = RL_RET_CODE_OK;
  int timeOutCnt = 0;
  mmwl_bTDA_StopRecordACK = 0U;

  /* Notify TDA about stopping the frame */
  retVal = stopRecord();
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT("ERROR: Notifying TDA about stop frame failed with error code %d \n\n", retVal);
    return -1;
  }
  else {
    DEBUG_PRINT("INFO: Notifying TDA about stop frame successful \n\n");
    mmwl_bTDA_ARMDone = 0U;
  }

  while (1) {
    if (mmwl_bTDA_StopRecordACK == 0U) {
      msleep(1); /*Sleep 1 msec*/
      timeOutCnt++;
      if (timeOutCnt > MMWL_API_TDA_TIMEOUT*2) {
        DEBUG_PRINT("ERROR: TDA Stop Record ACK not received!");
        retVal = RL_RET_CODE_RESP_TIMEOUT;
        return retVal;
      }
    }
    else {
      break;
    }
  }

  return retVal;
}


/**
 * @brief 
 * 
 * @param deviceMap 
 * @return int 
 */
int MMWL_DeviceDeInit(unsigned int deviceMap) {
  int retVal = RL_RET_CODE_OK;

  /* Note- Before Calling this API user must feed in input signal to device's pins,
  else device will return garbage data in GPAdc measurement over Async event.
  Measurement data is stored in 'rcvGpAdcData' structure after this API call. */
  retVal = MMWL_gpadcMeasConfig(deviceMap);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT("Device map %u : GPAdc measurement API failed with error code %d \n\n",
      deviceMap, retVal);
    return -1;
  }
  else {
    DEBUG_PRINT("Device map %u : GPAdc measurement API success\n\n", deviceMap);
  }

  return retVal;
}


/**
 * @brief Connect to ethernet and init TDA board
 * 
 * @param ipAddr IP Address of the TDA board (default: 192.168.33.30)
 * @param port Port number to communication with the TDA (default: 5001)
 * @param deviceMap All cascaded device map
 * @return int Initialization status
 */
int MMWL_TDAInit(unsigned char *ipAddr, unsigned int port, uint8_t deviceMap) {
  int retVal = RL_RET_CODE_OK;
  int timeOutCnt = 0;

  /* Register Async event handler with TDA */
  retVal = registerTDAStatusCallback((TDA_EVENT_HANDLER)TDA_asyncEventHandler);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "ERROR: Registering Async event handler with TDA failed with error %d \n\n",
      retVal
    );
    return -1;
  }
  else {
    DEBUG_PRINT("INFO: Registered Async event handler with TDA \n\n");
  }

  mmwl_bTDA_CaptureCardConnect = 0U;
  /* Connect to the TDA Capture card */
  retVal = ethernetConnect(ipAddr, port, deviceMap);
  if (retVal != RL_RET_CODE_OK) {
    DEBUG_PRINT(
      "ERROR: Connecting to TDA failed with error %d. Check whether the capture card is connected to the network! \n\n",
      retVal
    );
    return -1;
  }
  while (1) {
    if (mmwl_bTDA_CaptureCardConnect == 0U) {
      msleep(1); /*Sleep 1 msec*/
      timeOutCnt++;
      if (timeOutCnt > MMWL_API_TDA_TIMEOUT) {
        DEBUG_PRINT("ERROR: No Acknowlegment received from the capture card! \n\n");
        retVal = RL_RET_CODE_RESP_TIMEOUT;
        return retVal;
      }
    }
    else {
      break;
    }
  }

  if (retVal == RL_RET_CODE_OK) {
    DEBUG_PRINT("INFO: Connection to TDA successful! \n\n");
  }

  return retVal;
}


/**
 * @brief Assign device map
 * 
 * @return int 
 */
int MMWL_AssignDeviceMap(unsigned char deviceMap, uint8_t* masterMap, uint8_t* slavesMap) {
  int retVal = RL_RET_CODE_OK;
  unsigned char devId = 0;
  *slavesMap = 0;

  if ((deviceMap & 1) == 0) {
    return RL_RET_CODE_INVALID_INPUT;
  }

  // ID 0: master
  *masterMap = 1;

  // ID 1-3: slaves
  for (devId = 1; devId < 4; devId++) {
    if ((deviceMap & (1 << devId)) != 0) {
      *slavesMap |= (1 << devId);
    }
  }

  return retVal;
}
