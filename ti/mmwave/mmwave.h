/**
 * @file mmwave.h
 * @author AMOUSSOU Z. Kenneth (www.gitlab.com/azinke)
 * @brief 
 * @version 0.1
 * @date 2022-07-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef MMWAVE_H
#define MMWAVE_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>

/* AWR2243 meta image file */
#include "../firmware/xwr22xx_metaImage.h"

#include "../mmwavelink/mmwavelink.h"
#include "../ethernet/src/mmwl_port_ethernet.h"
#include "rls_osi.h"


/******************************************************************************
* MACROS
*******************************************************************************
*/

/* Default device configuration */
#define MMW_IP_ADDR         "192.168.33.180"
#define MMW_PORT            5001U
#define MMW_CAPTURE_DIR     "/mnt/ssd/MMWL_Capture"

/*Maximum number of devices connected*/
#define NUM_CONNECTED_DEVICES_MAX             (4U)

/* Trasnport Types */
#define RL_IMPL_TRANSPORT_IF_SPI              (0U)
#define RL_IMPL_TRANSPORT_IF_UART             (1U)

/*Default master device*/
#define DEFAULT_MASTER_DEVICE                 (0U)

/* Firmware File Type */
#define MMWL_FILETYPE_META_IMG                (0U)

/* Slaves deviceMap */
#define RL_DEVICE_MAP_CASCADED_SLAVES		      (14U)

#define MMWL_FW_FIRST_CHUNK_SIZE (224U)
#define MMWL_FW_CHUNK_SIZE (232U)
#define MMWL_META_IMG_FILE_SIZE (sizeof(metaImage))

#define GET_BIT_VALUE(data, noOfBits, location)    \
            ((((rlUInt32_t)(data)) >> (location)) &\
            (((rlUInt32_t)((rlUInt32_t)1U << (noOfBits))) - (rlUInt32_t)1U))

/* Async Event Timeouts */
#define MMWL_API_TDA_TIMEOUT								  (3000) /* 3 Sec */
#define MMWL_API_INIT_TIMEOUT                 (2000) /* 2 Sec*/
#define MMWL_API_START_TIMEOUT                (1000) /* 1 Sec*/
#define MMWL_API_RF_INIT_TIMEOUT              (1000) /* 1 Sec*/

/* MAX unique chirp AWR2243 supports */
#define MAX_UNIQUE_CHIRP_INDEX                (512 -1)

/* MAX index to read back chirp config  */
#define MAX_GET_CHIRP_CONFIG_IDX              14

/* To enable TX2 */
#define ENABLE_TX2                             1

/* LUT Buffer size for Advanced chirp 
   Max size = 12KB (12*1024) */
#define LUT_ADVCHIRP_TABLE_SIZE                5*1024



/******************************************************************************
* GLOBAL VARIABLES
*******************************************************************************
*/
typedef void* DevHandle_t;


rlReturnVal_t rlDeviceFileDownloadWrap(rlUInt8_t deviceMap, \
      rlUInt16_t remChunks, rlFileData_t* data) {
  return(rlDeviceFileDownload(deviceMap, data, remChunks));
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

#define SET_ADC_OUT_IND								              0
#define SET_LOW_POWER_MODE_IND						          1
#define SET_CHANNEL_CONFIG_IND						          2
#define SET_BPM_CHIRP_CONFIG_IND					          3
#define RF_CALIB_DATA_RESTORE_IND					          4
#define SET_FRAME_CONFIG_IND						            5
#define SET_ADV_CHIRP_CONFIG_IND					          6
#define SET_ADV_FRAME_CONFIG_IND					          7
#define RF_DYNAMIC_POWER_SAVE_IND					          8
#define RF_DFE_RX_STATS_REPORT_IND					        9
#define SET_CONT_MODE_CONFIG_IND					          10
#define ENABLE_CONT_MODE_IND						            11
#define SET_DATA_FORMAT_CONFIG_IND					        12
#define SET_DATA_PATH_CONFIG_IND					          13
#define SET_MISC_CONFIG_IND							            14
#define SET_LANE_CONFIG_IND							            15
#define SET_DATA_PATH_CLK_CONFIG_IND				        16
#define SET_LVDS_LANE_CONFIG_IND					          17
#define SET_CONT_STREAM_MODE_CONFIG_IND				      18
#define SET_CSI2_CONFIG_IND							            19
#define SET_HSI_CLK_IND								              20
#define SET_LDO_BYPASS_CONFIG_IND					          21
#define SET_GPADC_CONFIG							              22
#define RF_SET_DEVICE_CONFIG_IND					          23
#define RF_SET_PA_LPBK_CONFIG_IND					          24
#define RF_SET_PS_LPBK_CONFIG_IND					          25
#define RF_SET_IF_LPBK_CONFIG_IND					          26
#define RF_SET_PROG_FILT_COEFF_RAM_IND				      27
#define RF_SET_PROG_FILT_CONFIG_IND					        28
#define RF_SET_MISC_CONFIG_IND						          29
#define RF_SET_CAL_MON_TIME_CONFIG_IND				      30
#define RF_SET_CAL_MON_FREQ_LIM_IND					        31
#define RF_INIT_CALIB_CONFIG_IND					          32
#define RF_RUN_TIME_CALIB_CONFIG_IND				        33
#define RF_DIG_MON_ENABLE_CONFIG					          34
#define RF_CALIB_DATA_STORE_IND						          35
#define SET_TEST_PATTERN_CONFIG_IND					        36
#define RF_TX_FREQ_PWR_LIMIT_CONFIG_IND				      37
#define RF_RX_GAIN_PH_MON_CONFIG_IND				        38
#define RF_INTER_RX_GAIN_PHASE_CONFIG_IND			      39
#define RF_TX_PH_SHIFT_MON_CONFIG_IND			          40
#define RF_ANA_FAULT_INJ_CONFIG_IND					        41
#define RF_RX_IF_SAT_MON_CONFIG_IND					        42
#define RF_RX_SIG_IMG_MON_CONFIG_IND				        43
#define TX_GAIN_TEMP_LUT_SET_IND					          44
#define RF_ANA_MON_CONFIG_IND						            45
#define LATENT_FAULT_TESTS_IND						          46
#define ENABLE_PERIODIC_TESTS_IND					          47
#define SET_LOOPBAK_BURST_CFG_IND					          48
#define SET_SUBFRAME_START_IND						          49
#define MCU_CLK_CONFIG_IND							            50
#define RF_PH_SHIFT_CALIB_DATA_RESTORE_IND			    51
#define SET_INTER_CHIRP_BLK_CTRL_IND				        52
#define PMIC_CLK_CONFIG_IND							            53
#define SET_DYN_CHIRP_EN_IND						            54
#define RF_TX_GAIN_PHASE_MISMATCH_CONFIG_IND		    55
#define RF_TEMP_MON_CONFIG_IND						          56
#define RF_EXT_ANA_SIGNALS_MON_CONFIG_IND			      57
#define RF_GPADC_INT_ANA_SIGNALS_MON_CONFIG_IND		  58
#define RF_PMCLK_LO_INT_ANA_SIGNALS_MON_CONFIG_IND	59
#define RF_RX_INT_ANA_SIGNALS_MON_CONFIG_IND		    60
#define RF_TX_INT_ANA_SIGNALS_MON_CONFIG_IND		    61
#define RF_DUAL_CLK_COMP_MON_CONFIG_IND				      62
#define RF_PLL_CONTRL_VOLT_MON_CONFIG_IND			      63
#define RF_SYNTH_FREQ_MON_CONFIG_IND				        64
#define RF_TX_POWR_MON_CONFIG_IND					          65
#define RF_RX_NOISE_MON_CONFIG_IND					        66
#define RF_RX_MIXER_IN_PWR_CONFIG_IND				        67
#define RF_RX_IF_STAGE_MON_CONFIG_IND				        68
#define RF_DIG_MON_PERIODIC_CONFIG_IND				      69
#define RF_TX_BALL_BREAK_MON_CONFIG_IND				      70
#define RF_PH_SHIFT_CALIB_DATA_STORE_IND			      71
#define GET_RF_VERSION_IND							            72
#define GET_MSS_VERSION_IND							            73
#define GET_ADV_FRAME_CONFIG_IND					          74
#define SET_TEST_SOURCE_CONFIG_IND					        75
#define RF_TEST_SOURCE_ENABLE						            76
#define RF_GET_VERSION_IND							            77
#define RF_GET_DIE_ID_IND                           78
#define RF_SET_MON_TYPE_TRIGGER_CONFIG_IND          79
#define RF_SET_APLL_SYNTH_BW_CTL_CONFIG_IND         80
#define RF_SET_DEBUG_SIGNALS_CONFIG_IND             81
#define RF_SET_CSI2_DELAY_DUMMY_CONFIG_IND          82
#define SET_ADV_CHIRP_LUT_CONFIG_IND			          83
#define SENSOR_START_STOP_IND			                  84
#define GET_RF_BOOTUP_STATUS_IND			              85
#define SET_ADV_CHIRP_DYN_LUT_CONFIG_IND			      86
#define GET_TEMP_DEVICE_IND							            87
};


rlReturnVal_t(*funcTableTypeB[])(unsigned char) = {
  rlDeviceAddDevices,
  rlDeviceRemoveDevices,
  rlDeviceRfStart,
  rlRfInit,
  rlSensorStart,
  rlSensorStop

#define ADD_DEVICE_IND					0
#define REMOVE_DEVICE_IND				1
#define RF_START_IND					  2
#define RF_INIT_IND						  3
#define SENSOR_START_IND				4
#define SENSOR_STOP_IND					5
};


rlReturnVal_t(*funcTableTypeC[])(unsigned char, unsigned short, void *) = {
  rlSetProfileConfig,
  rlSetChirpConfig,
  rlRfSetPhaseShiftConfig,
  rlDeviceFileDownloadWrap,
  rlSetDynChirpCfg,
  rlSetDynPerChirpPhShifterCfg,
  rlGetProfileConfig

#define SET_PROFILE_CONFIG_IND						        0
#define SET_CHIRP_CONFIG_IND						          1
#define RF_SET_PHASE_SHIFT_CONFIG_IND				      2
#define FILE_DOWNLOAD_IND							            3
#define SET_DYN_CHIRP_CFG_IND						          4
#define SET_DYN_PER_PERCHIRP_PH_SHIFTER_CFG_IND		5
#define GET_PROFILE_CONFIG_IND						        6
};


/******************************************************************************
* STRUCTURE DEFINATION
******************************************************************************
*/

/* Comments/Errors Id/Strings*/
typedef struct {
  int id;
  const char* idMsg;
} idMsg_t;


#define API_TYPE_A		0x00000000
#define API_TYPE_B		0x10000000
#define API_TYPE_C		0x20000000

typedef struct {
  unsigned int deviceIndex;
  unsigned int apiInfo;
  void *payLoad;
  unsigned int flag;
} taskData;


/*! \brief
* Global Configuration Structure
*/
typedef struct rlDevGlobalCfg {
  /**
   * @brief  Advanced frame test enable/disable
   *         1 - Enable; 0 - Disable
   */
  unsigned char LinkAdvanceFrameTest;
  /**
   * @brief  Continuous mode test enable/disable
   *         1 - Enable; 0 - Disable
   */
  unsigned char LinkContModeTest;
  /**
   * @brief  Dynamic chirp test enable/disable
   *         1 - Enable; 0 - Disable
   */
  unsigned char LinkDynChirpTest;
  /**
   * @brief  Dynamic profile test enable/disable
   *         1 - Enable; 0 - Disable
   */
  unsigned char LinkDynProfileTest;
  /**
   * @brief  Advanced chirp test enable/disable
   *         1 - Enable; 0 - Disable
   */
  unsigned char LinkAdvChirpTest;
  /**
   * @brief  Firmware download enable/disable
   *         1 - Enable; 0 - Disable
   */
  unsigned char EnableFwDownload;
  /**
   * @brief  mmwavelink logging enable/disable
   *         1 - Enable; 0 - Disable
   */
  unsigned char EnableMmwlLogging;
  /**
   * @brief  Calibration enable/disable
   *         To perform calibration store/restore
   *         1 - Enable; 0 - Disable
   */
  unsigned char CalibEnable;
  /**
   * @brief  Calibration Store/Restore
   *         If CalibEnable = 1, then whether to store/restore
   *         1 - Store; 0 - Restore
   */
  unsigned char CalibStoreRestore;
  /**
   * @brief  Bitmap of the Devices to be enabled
   *				1 - Master ; 2 - Slave1 ; 4 - Slave2 ; 8 - Slave3
   */
  unsigned char CascadeDeviceMap;
  /**
   * @brief  Transport mode
   *         1 - I2C; 0 - SPI
   */
  unsigned char TransferMode;

} rlDevGlobalCfg_t;


/******************************************************************************
* FUNCTION DECLARATION
*******************************************************************************
*/

/*Device poweroff*/
int MMWL_powerOff(unsigned char deviceMap);

/*sensor stop*/
int MMWL_sensorStop(unsigned char deviceMap);

/*Sensor start*/
int MMWL_sensorStart(unsigned char deviceMap);

/*Frame configuration*/
int MMWL_frameConfig(
  unsigned char deviceMap, rlFrameCfg_t frameCfgArgs, rlChanCfg_t rfChanCfgArgs,
  rlAdcOutCfg_t adcOutCfgArgs, rlDevDataPathCfg_t dataPathCfgArgs, rlProfileCfg_t profileCfgArgs);

/*Chirp configuration*/
int MMWL_chirpConfig(unsigned char deviceMap, rlChirpCfg_t chirpCfgArgs);

/*Profile configuration*/
int MMWL_profileConfig(unsigned char deviceMap, rlProfileCfg_t profileCfgArgs);

/*HSI lane configuration*/
int MMWL_hsiLaneConfig(unsigned char deviceMap);
int MMWL_laneConfig(unsigned char deviceMap, rlDevLaneEnable_t laneEnCfgArgs);
int MMWL_lvdsLaneConfig(unsigned char deviceMap, rlDevLaneEnable_t laneEnCfgArgs);

/* CSI2 Lane configuration */
int MMWL_CSI2LaneConfig(unsigned char deviceMap, rlDevCsi2Cfg_t CSI2LaneCfgArgs);

/*HSI Clock configuration*/
int MMWL_hsiClockConfig(unsigned char deviceMap,
    rlDevDataPathClkCfg_t dataPathClkCfgArgs, rlDevHsiClk_t hsiClkgs);
int MMWL_hsiDataRateConfig(unsigned char deviceMap, rlDevDataPathClkCfg_t dataPathClkCfgArgs);
int MMWL_setHsiClock(unsigned char deviceMap, rlDevHsiClk_t hsiClkgs);

/* Data path(High SPeed Interface: CSI2/LVDS) configuration*/
int MMWL_dataPathConfig(unsigned char deviceMap, rlDevDataPathCfg_t dataPathCfgArgs);

/*RFinit*/
int MMWL_rfInit(unsigned char deviceMap);

/*Lowpower configuration*/
int MMWL_lowPowerConfig(unsigned char deviceMap, rlLowPowerModeCfg_t rfLpModeCfgArgs);
/* APLL Synth BW configuration */
int MMWL_ApllSynthBwConfig(unsigned char deviceMap);

/*Channle, ADC and Dataformat configuration API's*/
int MMWL_basicConfiguration(unsigned char deviceMap,
                            rlAdcOutCfg_t adcOutCfgArgs, rlDevDataFmtCfg_t dataFmtCfgArgs);
int MMWL_channelConfig(unsigned char deviceMap, unsigned short cascade, rlChanCfg_t rfChanCfgArgs);
int MMWL_ldoBypassConfig(unsigned char deviceMap);
int MMWL_adcOutConfig(unsigned char deviceMap, rlAdcOutCfg_t adcOutCfgArgs);
int MMWL_dataFmtConfig(unsigned char deviceMap, rlDevDataFmtCfg_t dataFmtCfgArgs);
int MMWL_setMiscConfig(unsigned char deviceMap);

/*RFenable API*/
int MMWL_rfEnable(unsigned char deviceMap);

/*Download firmware API*/
int MMWL_firmwareDownload(unsigned char deviceMap);
int MMWL_fileDownload(unsigned char deviceMap, unsigned int fileLen);
int MMWL_fileWrite(unsigned char deviceMap, unsigned short remChunks,
                   unsigned short chunkLen,
                   unsigned char *chunk);

/* Save Calibration Data to a file */
// int MMWL_saveCalibDataToFile(unsigned char deviceMap);
/* Save Phase shifter Calibration Data to a file */
// int MMWL_savePhShiftCalibDataToFile(unsigned char deviceMap);
/* Load Calibration Data to a file */
// int MMWL_LoadCalibDataFromFile(unsigned char deviceMap);
/* Load Phase shifter Calibration Data from a file */
// int MMWL_LoadPhShiftCalibDataFromFile(unsigned char deviceMap);

/*Poweron Master*/
int MMWL_powerOnMaster(unsigned char deviceMap);


uint64_t computeCRC(uint8_t *p, uint32_t len, uint8_t width);

#endif
