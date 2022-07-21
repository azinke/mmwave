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

int MMWL_TDAInit();


uint64_t computeCRC(uint8_t *p, uint32_t len, uint8_t width);

#endif
