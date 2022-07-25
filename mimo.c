/**
 * @file mimo.c
 * @author AMOUSSOU Z. Kenneth (www.gitlab.com/azinke)
 * @brief MMWave Radar in MIMO setup
 *
 * The MMWCAS-RF-EVM revision E has AWR2243 radar chips
 *
 * @version 0.1
 * @date 2022-07-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "ti/mmwave/mmwave.h"

#define NUM_CHRIPS 16


/** Device configuration */
typedef struct devConfig {

  // Device Map (1: Master, 2: Slave1, 4: Slave2, 8: Slave3)
  uint8_t deviceMap;

  // Master device map (value: 1)
  uint8_t masterMap;

  // Slave devices map (value: 14)
  uint8_t slavesMap;

  // Frame config
  rlFrameCfg_t frameCfg;

  // Profile config
  rlProfileCfg_t profileCfg;

  // Chirp config
  rlChirpCfg_t chirpCfg;

  // Channel config
  rlChanCfg_t channelCfg;

  // ADC output config
  rlAdcOutCfg_t adcOutCfg;

  // LDO Bypass config
  rlRfLdoBypassCfg_t ldoCfg;

  // Low Power mode config config
  rlLowPowerModeCfg_t lpmCfg;

  // Miscellaneous config.
  rlRfMiscConf_t miscCfg;

  // Datapath config
  rlDevDataPathCfg_t datapathCfg;

  // Datapath clock config
  rlDevDataPathClkCfg_t datapathClkCfg;

  // High Speed clock config
  rlDevHsiClk_t hsClkCfg;

  // CSI2 config
  rlDevCsi2Cfg_t csi2LaneCfg;

} devConfig_t;


/******************************
 *      CONFIGURATIONS
 ******************************/

/** Profile config */
const rlProfileCfg_t profileCfgArgs = {
  .profileId = 0,
  .pfVcoSelect = 0x02,
  .startFreqConst = 1435384036,   // 77GHz | 1 LSB = 53.644 Hz
  .freqSlopeConst = 1637,         // 79.0327 Mhz/us | 1LSB = 48.279 kHz/uS
  .idleTimeConst = 500,           // 5us  | 1LSB = 10ns
  .adcStartTimeConst = 600,       // 6us  | 1LSB = 10ns
  .rampEndTime = 4000,            // 40us | 1LSB = 10ns
  .txOutPowerBackoffCode = 0x0,
  .txPhaseShifter = 0x0,
  .txStartTime = 0x0,
  .numAdcSamples = 256,           // 256 ADC samples per chirp
  .digOutSampleRate = 8000,       // 8000 ksps (8 MHz) | 1LSB = 1 ksps
  .hpfCornerFreq1 = 0x0,          // 175kHz
  .hpfCornerFreq2 = 0x0,          // 350kHz
  .rxGain = 48,                   // 48 dB | 1LSB = 1dB
};

/** Frame config */
const rlFrameCfg_t frameCfgArgs = {
  .chirpStartIdx = 0,
  .chirpEndIdx = 11,
  .numFrames = 100,               // (0 for infinite)
  .numLoops = NUM_CHRIPS,
  .numAdcSamples = 2 * 256,       // Complex samples (for I and Q siganls)
  .frameTriggerDelay = 0x0,
  .framePeriodicity = 20000000,   // 100ms | 1LSB = 5ns
};

/** Chirps config */
rlChirpCfg_t chirpCfgArgs = {
  .chirpStartIdx = 0,
  .chirpEndIdx = 0,
  .profileId = 0,
  .txEnable = 0x00,
};

/** Channel config */
rlChanCfg_t channelCfgArgs = {
  .rxChannelEn = 0x0F,      // Enable all 4 RX Channels
  .txChannelEn = 0x07,      // Enable all 3 TX Channels
  .cascading = 0x02,        // Slave
};

/** ADC output config */
rlAdcOutCfg_t adcOutCfgArgs = {
  .fmt = {
    .b2AdcBits = 2,           // 16-bit ADC
    .b2AdcOutFmt = 1,         // Complex values
    .b8FullScaleReducFctr = 0,
  }
};

/** LDO Bypass config */
rlRfLdoBypassCfg_t ldoCfgArgs = {
  .ldoBypassEnable = 3,       // RF LDO disabled, PA LDO disabled
};

/** Low Power Mode config */
rlLowPowerModeCfg_t lpmCfgArgs = {
  .lpAdcMode = 0,             // Regular ADC power mode
};

/** Miscellaneous config */
rlRfMiscConf_t miscCfgArgs = {
  .miscCtl = 1,               // Enable Per chirp phase shifter
};

/** Datapath config */
rlDevDataPathCfg_t datapathCfgArgs = {
  .intfSel = 0,               // CSI2 intrface
  .transferFmtPkt0 = 1,       // ADC data only
  .transferFmtPkt1 = 0,       // Suppress packet 1
};

/** Datapath clock config */
rlDevDataPathClkCfg_t datapathClkCfgArgs = {
  .laneClkCfg = 1,            // DDR Clock
  .dataRate = 1,              // 600Mbps
};

/** High speed clock config */
rlDevHsiClk_t hsClkCfgArgs = {
  .hsiClk = 0x09,             // DDR 600Mbps
};

/** CSI2 config */
rlDevCsi2Cfg_t csi2LaneCfgArgs = {
  .lineStartEndDis = 0,       // Enable
  .lanePosPolSel = 0x35421,   // 0b 0011 0101 0100 0010 0001,
};



/*
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|       | Dev 1 | Dev 1 | Dev 1 | Dev 2 | Dev 2 | Dev 2 | Dev 3 | Dev 3 | Dev 3 | Dev 4 | Dev 4 | Dev 4 |
| Chirp |  TX0  |  TX1  |  TX2  |  TX 0 |  TX1  |  TX2  |  TX0  |  TX1  |  TX2  |  TX0  |  TX1  |  TX2  |
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |
|     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |
|     2 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |
|     3 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |
|     4 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |
|     5 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |
|     6 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |
|     7 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|     8 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|     9 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|    10 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|    11 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
*/


/**
 * @brief Check if a value is in the table provided in argument
 *
 * @param value Value to look for in the table
 * @param table Tbale defining the search context
 * @param size Size of the table
 * @return int8_t
 *      Return the index where the match has been found. -1 if not found
 */
int8_t is_in_table(uint8_t value, uint8_t *table, uint8_t size) {
  for (uint8_t i = 0; i < size; i++) {
    if (table[i] == value) return i;
  }
  return -1;
}


/**
 * @brief MIMO Chirp configuration
 *
 * @param devId Device ID (0: master, 1: slave1, 2: slave2, 3: slave3)
 * @param chirpCfg Initital chirp configuration
 * @return uint32_t Configuration status
 */
uint32_t configureMimoChirp(uint8_t devId, rlChirpCfg_t chirpCfg) {
  static uint8_t chripTxTbale [4][3] = {
    {11, 10, 9},   // Dev1 - Master
    {8, 7, 6},     // Dev2
    {5, 4, 3},     // Dev3
    {2, 1, 0},     // Dev4
  };
  int status = -1;

  for (uint8_t i = 0; i < NUM_CHRIPS; i++) {
    int8_t txIdx = is_in_table(i, chripTxTbale[devId], 3);

    // Update chirp config
    chirpCfg.chirpStartIdx = i;
    chirpCfg.chirpEndIdx = i;
    if (txIdx < 0) chirpCfg.txEnable = 0x00;
    else chirpCfg.txEnable = (1 << txIdx);
    status = MMWL_chirpConfig(createDevMapFromDevId(devId), chirpCfg);
    if (status < 0) {
      printf("Configuration of chirp %d failed!\n", i);
      break;
    }
  }
  return status;
}


uint32_t initMaster(rlChanCfg_t channelCfg, rlAdcOutCfg_t adcOutCfg) {
  const unsigned int masterId = 0;
  const unsigned int masterMap = 1 << masterId;
  int status = 0;

  // master chip
  channelCfg.cascading = 1;

  status = MMWL_DevicePowerUp(masterMap);
  if (status == 0) {
    status = MMWL_firmwareDownload(masterMap);
    if (status == 0) {
      status = MMWL_rfEnable(masterMap);
      if (status != 0) printf("[MASTER] Failed to enable master RF\n");
      else printf("[MASTER] RF successfully enabled\n");

      status = MMWL_channelConfig(masterMap, 1, channelCfg);
      if (status != 0) printf("[MASTER] Channels configuration failed!\n");
      else printf("[MASTER] Channels successfully configured\n");

      status = MMWL_adcOutConfig(masterMap, adcOutCfg);
      if (status != 0)
        printf("[MASTER] ADC output format configuration failed!\n");
      else printf("[MASTER] ADC output format successfully configured\n");
    } else {
      printf("[MASTER] Error: Failed to upload firmware!\n");
    }
  } else {
    printf("[MASTER] Error: Failed to power up device!\n");
  }
  printf("[MASTER] Init completed\n\n");
  return status;
}


uint32_t initSlaves(rlChanCfg_t channelCfg, rlAdcOutCfg_t adcOutCfg) {
  int status = 0;
  for (uint8_t slaveId = 1; slaveId < 4; slaveId++) {
    unsigned int slaveMap = 1 << slaveId;

    // slave chip
    channelCfg.cascading = 2;

    status = MMWL_DevicePowerUp(slaveMap);
    if (status == 0) {
      status = MMWL_firmwareDownload(slaveMap);
      if (status == 0) {
        status = MMWL_rfEnable(slaveMap);
        if (status != 0)
          printf("[SLAVE %d] Failed to enable master RF\n", slaveId);
        else
          printf("[SLAVE %d] RF successfully enabled\n", slaveId);

        status = MMWL_channelConfig(slaveMap, 2, channelCfg);
        if (status != 0)
          printf("[SLAVE %d] Channels configuration failed!\n", slaveId);
        else
          printf("[SLAVE %d] Channels successfully configured\n", slaveId);

        status = MMWL_adcOutConfig(slaveMap, adcOutCfg);
        if (status != 0)
          printf("[SLAVE %d] ADC output format configuration failed!\n", slaveId);
        else
          printf("[SLAVE %d] ADC output format successfully configured\n", slaveId);
      }
    }
  }
  printf("[SLAVE] Init completed\n\n");
  return status;
}


uint32_t configure (devConfig_t config) {
  initMaster(config.channelCfg, config.adcOutCfg);
  initSlaves(config.channelCfg, config.adcOutCfg);

  MMWL_ldoBypassConfig(config.deviceMap, config.ldoCfg);
  MMWL_lowPowerConfig(config.deviceMap, config.lpmCfg);
  MMWL_setMiscConfig(config.deviceMap, config.miscCfg);
  MMWL_rfInit(config.deviceMap);

  MMWL_dataPathConfig(config.deviceMap, config.datapathCfg);
  MMWL_hsiClockConfig(config.deviceMap, config.datapathClkCfg, config.hsClkCfg);
  MMWL_CSI2LaneConfig(config.deviceMap, config.csi2LaneCfg);

  MMWL_profileConfig(config.deviceMap, config.profileCfg);

  // MIMO Chirp configuration
  for (uint8_t devId = 0; devId < 4; devId++) {
    configureMimoChirp(devId, config.chirpCfg);
  }

  // Master frame config.
  config.frameCfg.triggerSelect = 1;    // Software trigger
  MMWL_frameConfig(
    config.masterMap,
    config.frameCfg,
    config.channelCfg,
    config.adcOutCfg,
    config.datapathCfg,
    config.profileCfg
  );
  // Slaves frame config
  // Master frame config.
  config.frameCfg.triggerSelect = 2;    // Hardware trigger
  MMWL_frameConfig(
    config.slavesMap,
    config.frameCfg,
    config.channelCfg,
    config.adcOutCfg,
    config.datapathCfg,
    config.profileCfg
  );

  printf("[MIMO] Configuration complete!\n\n");
}


/**
 * @brief Routine to close trace file
 * 
 */
FILE* rls_traceF = NULL;
void CloseTraceFile() {
  if (rls_traceF != NULL) {
    fclose(rls_traceF);
    rls_traceF = NULL;
  }
}


/**
 * @brief Application entry point
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main (int argc, char *argv[]) {
  printf("MMWave MIMO Application\n");

  // Configuration
  devConfig_t config;

  /*  Device map:  master | slave 1  | slave 2  | slave 3 */
  config.deviceMap =  1   | (1 << 1) | (1 << 2) | (1 << 3);
  MMWL_AssignDeviceMap(config.deviceMap, &config.masterMap, &config.slavesMap);

  config.frameCfg = frameCfgArgs;
  config.profileCfg = profileCfgArgs;
  config.chirpCfg = chirpCfgArgs;
  config.adcOutCfg = adcOutCfgArgs;
  config.channelCfg = channelCfgArgs;
  config.csi2LaneCfg = csi2LaneCfgArgs;
  config.datapathCfg = datapathCfgArgs;
  config.datapathClkCfg = datapathClkCfgArgs;
  config.hsClkCfg = hsClkCfgArgs;
  config.ldoCfg = ldoCfgArgs;
  config.lpmCfg = lpmCfgArgs;
  config.miscCfg = miscCfgArgs;

  unsigned char ipAddr[] = "192.168.33.180";
  unsigned int port = 5001U;
  int status = 0;

  unsigned char captureDirectory[] = "/mnt/ssd/MMWL_Capture";

  // config to ARM the TDA
  rlTdaArmCfg_t tdaCfg = {
    .captureDirectory = captureDirectory,
    .framePeriodicity = config.frameCfg.framePeriodicity,
    .numberOfFilesToAllocate = 0,
    .numberOfFramesToCapture = config.frameCfg.numFrames,
    .dataPacking = 0, // 0: 16-bit | 1: 12-bit
  };

  // Connect to TDA
  status = MMWL_TDAInit(ipAddr, port, config.deviceMap);
  if (status != 0) printf("Couldn't connect to TDA board!\n");
  else printf("Connected!\n\n");

  // Start configuration
  configure(config);

  MMWL_ArmingTDA(tdaCfg);

  MMWL_StartFrame(config.slavesMap);
  MMWL_StartFrame(config.masterMap);
  sleep(3);
  MMWL_StopFrame(config.slavesMap);
  MMWL_StopFrame(config.masterMap);

  MMWL_DeArmingTDA();
  // power off device

  /*
  int status = MMWL_TDAInit();
  msleep(2000);
  ethernetDisconnect();
  */
  return 0;
}
