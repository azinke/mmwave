/**
 * @file mimo.c
 * @author AMOUSSOU Z. Kenneth (www.gitlab.com/azinke)
 * @brief MMWave Radar configuration and control tool
 *
 * @note: Only MIMO setup is supported for now
 *
 * The MMWCAS-RF-EVM revision E has AWR2243 radar chips
 *
 * Approximate default configuration (generated uing mmWave Sensing Estimator):
 *
 *  Max Detectable Range  : ~80m
 *  Range resolution      : ~31cm
 *  May Velocity          : ~6.49 km/h
 *  Velocity resolution   : ~0.4 km/h
 *
 * @version 0.1
 * @date 2022-07-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "mimo.h"
#include "toml/config.h"

/******************************
 *      CONFIGURATIONS
 ******************************/

/** Profile config */
const rlProfileCfg_t profileCfgArgs = {
  .profileId = 0,
  .pfVcoSelect = 0x02,
  .startFreqConst = 1435384036,   // 77GHz | 1 LSB = 53.644 Hz
  .freqSlopeConst = 311,          // 15.0148 Mhz/us | 1LSB = 48.279 kHz/uS
  .idleTimeConst = 500,           // 5us  | 1LSB = 10ns
  .adcStartTimeConst = 600,       // 6us  | 1LSB = 10ns
  .rampEndTime = 4000,            // 40us | 1LSB = 10ns
  .txOutPowerBackoffCode = 0x0,
  .txPhaseShifter = 0x0,
  .txStartTime = 0x0,             // 0us | 1LSB = 10ns
  .numAdcSamples = 256,           // 256 ADC samples per chirp
  .digOutSampleRate = 8000,      // 8000 ksps (8 MHz) | 1LSB = 1 ksps
  .hpfCornerFreq1 = 0x0,          // 175kHz
  .hpfCornerFreq2 = 0x0,          // 350kHz
  .rxGain = 48,                   // 48 dB | 1LSB = 1dB
};

/** Frame config */
const rlFrameCfg_t frameCfgArgs = {
  .chirpStartIdx = 0,
  .chirpEndIdx = 11,
  .numFrames = 0,                 // (0 for infinite)
  .numLoops = 16,
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
  .adcStartTimeVar = 0,
  .idleTimeVar = 0,
  .startFreqVar = 0,
  .freqSlopeVar = 0,
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

/** Data format config */
rlDevDataFmtCfg_t dataFmtCfgArgs = {
  .iqSwapSel = 0,           // I first
  .chInterleave = 0,        // Interleaved mode
  .rxChannelEn = 0xF,       // All RX antenna enabled
  .adcFmt = 1,              // Complex
  .adcBits = 2,             // 16-bit ADC
};

/** LDO Bypass config */
rlRfLdoBypassCfg_t ldoCfgArgs = {
  .ldoBypassEnable = 3,       // RF LDO disabled, PA LDO disabled
  .ioSupplyIndicator = 0,
  .supplyMonIrDrop = 0,
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
 * @param table Table defining the search context
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
  const uint8_t chripTxTable [4][3] = {
    {11, 10, 9},   // Dev1 - Master
    {8, 7, 6},     // Dev2
    {5, 4, 3},     // Dev3
    {2, 1, 0},     // Dev4
  };
  int status = 0;

  for (uint8_t i = 0; i < NUM_CHIRPS; i++) {
    int8_t txIdx = is_in_table(i, chripTxTable[devId], 3);

    // Update chirp config
    chirpCfg.chirpStartIdx = i;
    chirpCfg.chirpEndIdx = i;
    if (txIdx < 0) chirpCfg.txEnable = 0x00;
    else chirpCfg.txEnable = (1 << txIdx);
    status += MMWL_chirpConfig(createDevMapFromDevId(devId), chirpCfg);
    DEBUG_PRINT("[CHIRP CONFIG] dev %u, chirp idx %u, status: %d\n", devId, i, status);
    if (status != 0) {
      DEBUG_PRINT("Configuration of chirp %d failed!\n", i);
      break;
    }
  }
  return status;
}

/**
 * @brief Check status and print error or success message
 *
 * @param status Status value returned by a function
 * @param success_msg Success message to print when status is 0
 * @param error_msg Error message to print in case of error
 * @param deviceMap Device map the check if related to
 * @param is_required Indicates if the checking stage is required. if so,
 *                    the program exits in case of failure.
 * @return uint32_t Configuration status
 *
 * @note: Status is considered successful when the status integer is 0.
 * Any other value is considered a failure.
 */
void check(int status, const char *success_msg, const char *error_msg,
      unsigned char deviceMap, uint8_t is_required) {
#if DEV_ENV
  printf("STATUS %4d | DEV MAP: %2u | ", status, deviceMap);
#endif
  if (status == RL_RET_CODE_OK) {
#if DEV_ENV
    printf(CGREEN);
    printf(success_msg);
    printf(CRESET);
    printf("\n");
#endif
    return;
  } else {
#if DEV_ENV
    printf(CRED);
    printf(error_msg);
    printf(CRESET);
    printf("\n");
#endif
    if (is_required != 0) exit(status);
  }
}


int32_t initMaster(rlChanCfg_t channelCfg, rlAdcOutCfg_t adcOutCfg) {
  const unsigned int masterId = 0;
  const unsigned int masterMap = 1 << masterId;
  int status = 0;

  // master chip
  channelCfg.cascading = 1;

  status += MMWL_DevicePowerUp(masterMap, 1000, 1000);
  check(status,
    "[MASTER] Power up successful!",
    "[MASTER] Error: Failed to power up device!", masterMap, TRUE);

  status += MMWL_firmwareDownload(masterMap);
  check(status,
    "[MASTER] Firmware successfully uploaded!",
    "[MASTER] Error: Firmware upload failed!", masterMap, TRUE);

  status += MMWL_setDeviceCrcType(masterMap);
  check(status,
    "[MASTER] CRC type has been set!",
    "[MASTER] Error: Unable to set CRC type!", masterMap, TRUE);

  status += MMWL_rfEnable(masterMap);
  check(status,
    "[MASTER] RF successfully enabled!",
    "[MASTER] Error: Failed to enable master RF", masterMap, TRUE);

  status += MMWL_channelConfig(masterMap, channelCfg.cascading, channelCfg);
  check(status,
    "[MASTER] Channels successfully configured!",
    "[MASTER] Error: Channels configuration failed!", masterMap, TRUE);

  status += MMWL_adcOutConfig(masterMap, adcOutCfg);
  check(status,
    "[MASTER] ADC output format successfully configured!",
    "[MASTER] Error: ADC output format configuration failed!", masterMap, TRUE);

  check(status,
    "[MASTER] Init completed with sucess\n",
    "[MASTER] Init completed with error", masterMap, TRUE);
  return status;
}


int32_t initSlaves(rlChanCfg_t channelCfg, rlAdcOutCfg_t adcOutCfg) {
  int status = 0;
  uint8_t slavesMap = (1 << 1) | (1 << 2) | (1 << 3);

  // slave chip
  channelCfg.cascading = 2;

  for (uint8_t slaveId = 1; slaveId < 4; slaveId++) {
    unsigned int slaveMap = 1 << slaveId;

    status += MMWL_DevicePowerUp(slaveMap, 1000, 1000);
    check(status,
      "[SLAVE] Power up successful!",
      "[SLAVE] Error: Failed to power up device!", slaveMap, TRUE);
  }

  //Config of all slaves together
  status += MMWL_firmwareDownload(slavesMap);
  check(status,
    "[SLAVE] Firmware successfully uploaded!",
    "[SLAVE] Error: Firmware upload failed!", slavesMap, TRUE);

  status += MMWL_setDeviceCrcType(slavesMap);
  check(status,
    "[SLAVE] CRC type has been set!",
    "[SLAVE] Error: Unable to set CRC type!", slavesMap, TRUE);

  status += MMWL_rfEnable(slavesMap);
  check(status,
    "[SLAVE] RF successfully enabled!",
    "[SLAVE] Error: Failed to enable master RF", slavesMap, TRUE);

  status += MMWL_channelConfig(slavesMap, channelCfg.cascading, channelCfg);
  check(status,
    "[SLAVE] Channels successfully configured!",
    "[SLAVE] Error: Channels configuration failed!", slavesMap, TRUE);

  status += MMWL_adcOutConfig(slavesMap, adcOutCfg);
  check(status,
    "[SLAVE] ADC output format successfully configured!",
    "[SLAVE] Error: ADC output format configuration failed!", slavesMap, TRUE);

  check(status,
    "[SLAVE] Init completed with sucess\n",
    "[SLAVE] Init completed with error", slavesMap, TRUE);
  return status;
}


uint32_t configure (devConfig_t config) {
  int status = 0;
  status += initMaster(config.channelCfg, config.adcOutCfg);
  status += initSlaves(config.channelCfg, config.adcOutCfg);

  status += MMWL_RFDeviceConfig(config.deviceMap);
  check(status,
    "[ALL] RF deivce configured!",
    "[ALL] RF device configuration failed!", config.deviceMap, TRUE);

  status += MMWL_ldoBypassConfig(config.deviceMap, config.ldoCfg);
  check(status,
    "[ALL] LDO Bypass configuration successful!",
    "[ALL] LDO Bypass configuration failed!", config.deviceMap, TRUE);

  status += MMWL_dataFmtConfig(config.deviceMap, config.dataFmtCfg);
  check(status,
    "[ALL] Data format configuration successful!",
    "[ALL] Data format configuration failed!", config.deviceMap, TRUE);

  status += MMWL_lowPowerConfig(config.deviceMap, config.lpmCfg);
  check(status,
    "[ALL] Low Power Mode configuration successful!",
    "[ALL] Low Power Mode configuration failed!", config.deviceMap, TRUE);

  status += MMWL_ApllSynthBwConfig(config.deviceMap);
  status += MMWL_setMiscConfig(config.deviceMap, config.miscCfg);
  status += MMWL_rfInit(config.deviceMap);
  check(status,
    "[ALL] RF successfully initialized!",
    "[ALL] RF init failed!", config.deviceMap, TRUE);

  status += MMWL_dataPathConfig(config.deviceMap, config.datapathCfg);
  status += MMWL_hsiClockConfig(config.deviceMap, config.datapathClkCfg, config.hsClkCfg);
  status += MMWL_CSI2LaneConfig(config.deviceMap, config.csi2LaneCfg);
  check(status,
    "[ALL] Datapath configuration successful!",
    "[ALL] Datapath configuration failed!", config.deviceMap, TRUE);

  status += MMWL_profileConfig(config.deviceMap, config.profileCfg);
  check(status,
    "[ALL] Profile configuration successful!",
    "[ALL] Profile configuration failed!", config.deviceMap, TRUE);

  // MIMO Chirp configuration
  for (uint8_t devId = 0; devId < 4; devId++) {
    status += configureMimoChirp(devId, config.chirpCfg);
  }
  check(status,
    "[ALL] Chirp configuration successful!",
    "[ALL] Chirp configuration failed!", config.deviceMap, TRUE);

  // Master frame config.
  status += MMWL_frameConfig(
    config.masterMap,
    config.frameCfg,
    config.channelCfg,
    config.adcOutCfg,
    config.datapathCfg,
    config.profileCfg
  );
  check(status,
    "[MASTER] Frame configuration completed!",
    "[MASTER] Frame configuration failed!", config.masterMap, TRUE);

  // Slaves frame config
  status += MMWL_frameConfig(
    config.slavesMap,
    config.frameCfg,
    config.channelCfg,
    config.adcOutCfg,
    config.datapathCfg,
    config.profileCfg
  );
  check(status,
    "[SLAVE] Frame configuration completed!",
    "[SLAVE] Frame configuration failed!", config.slavesMap, TRUE);

  check(status,
    "[MIMO] Configuration completed!\n",
    "[MIMO] Configuration completed with error!", config.deviceMap, TRUE);
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

// Pointer to the CLI option parser
parser_t *g_parser = NULL;

/**
 * Print program version
 */
void print_version() {
  printf(PROG_NAME " version " PROG_VERSION ", " PROG_COPYRIGHT "\n");
  exit(0);
}

/**
 * @brief Print CLI options help and exit
 */
void help() {
  print_help(g_parser);
  exit(0);
}

/**
 * @brief Free the parser to cleanup any dynamically allocated memory
 */
void cleanup() {
  free_parser(g_parser);
}

/**
 * @brief Called when the user presses CTRL+C
 *
 * This aim to explicitly call the exit function so that
 * dynamically allocated memory could be freed
 */
void signal_handler () {
  exit(1);
}


/**
 * @brief Application entry point
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main (int argc, char *argv[]) {
  DEBUG_PRINT("MMWave EVM configuration and control application\n");
  unsigned char default_ip_addr[] = "192.168.33.180";
  unsigned int default_port = 5001U;
  unsigned char capture_path[128];
  strcpy(capture_path, "/mnt/ssd/");  // Root capture path
  unsigned char default_capture_directory[64];
  sprintf(default_capture_directory, "%s_%lu", "MMWL_Capture", (unsigned long int)time(NULL));
  int status = 0;
  float default_recording_duration = 1.0;   // min

  parser_t parser = init_parser(
    PROG_NAME,
    "Configuration and control tool for TI MMWave cascade Evaluation Module"
  );
  g_parser = &parser;

  atexit(cleanup);  // Call the cleanup function before exiting the program
  signal(SIGINT, signal_handler);  // Catch CTRL+C to enable memory deallocation

  option_t opt_capturedir = {
    .args = "-d",
    .argl = "--capture-dir",
    .help = "Name of the director where to store recordings on the DSP board",
    .type = OPT_STR,
    .default_value = default_capture_directory
  };
  add_arg(&parser, &opt_capturedir);

  option_t opt_port = {
    .args = "-p",
    .argl = "--port",
    .help = "Port number the DSP board server app is listening on",
    .type = OPT_INT,
    .default_value = &default_port,
  };
  add_arg(&parser, &opt_port);

  option_t opt_ipaddr = {
    .args = "-i",
    .argl = "--ip-addr",
    .help = "IP Address of the MMWCAS DSP evaluation module",
    .type = OPT_STR,
    .default_value = default_ip_addr,
  };
  add_arg(&parser, &opt_ipaddr);

  option_t opt_config = {
    .args = "-c",
    .argl = "--configure",
    .help = "Configure the MMWCAS-RF-EVM board",
    .type = OPT_BOOL,
  };
  add_arg(&parser, &opt_config);

  option_t opt_record = {
    .args = "-r",
    .argl = "--record",
    .help = "Trigger data recording. This assumes that configuration is completed.",
    .type = OPT_BOOL,
  };
  add_arg(&parser, &opt_record);

  option_t opt_record_duration = {
    .args = "-t",
    .argl = "--time",
    .help = "Indicate how long the recording should last in minutes. Default: 1 min",
    .type = OPT_FLOAT,
    .default_value = &default_recording_duration,
  };
  add_arg(&parser, &opt_record_duration);

  option_t opt_config_file = {
    .args = "-f",
    .argl = "--cfg",
    .help = "TOML Configuration file. Overwrite the default config when provided",
    .type = OPT_STR,
    .default_value = NULL,
  };
  add_arg(&parser, &opt_config_file);

  option_t opt_help = {
    .args = "-h",
    .argl = "--help",
    .help = "Print CLI option help and exit.",
    .type = OPT_BOOL,
    .default_value = NULL,
    .callback = help,
  };
  add_arg(&parser, &opt_help);

  option_t opt_version = {
    .args = "-v",
    .argl = "--version",
    .help = "Print program version and exit.",
    .type = OPT_BOOL,
    .callback = print_version,
  };
  add_arg(&parser, &opt_version);

  parse(&parser, argc, argv);

  // Print help
  if ((unsigned char*)get_option(&parser, "help") != NULL) {
    print_help(&parser);
    exit(0);
  }

  unsigned char *ip_addr = (unsigned char*)get_option(&parser, "ip-addr");
  unsigned int port = *(unsigned int*)get_option(&parser, "port");
  unsigned char *capture_directory = (unsigned char*)get_option(&parser, "capture-dir");
  strcat(capture_path, capture_directory);
  /* Record CLI option possible values are:
   *  - start: To start a recording and exit
   *  - stop: Stop a recording and exit
   *  - oneshot: Start a recording, wait for it's complemention and stop it.
   */
  unsigned char *record = (unsigned char*)get_option(&parser, "record");
  float record_duration = *(float*)get_option(&parser, "time");
  record_duration *= 60 * 1000;  // convert into milliseconds

  unsigned char *config_filename = (unsigned char*)get_option(&parser, "cfg");

  // Configuration
  devConfig_t config;

  /*  Device map:  master | slave 1  | slave 2  | slave 3 */
  config.deviceMap =  1   | (1 << 1) | (1 << 2) | (1 << 3);
  MMWL_AssignDeviceMap(config.deviceMap, &config.masterMap, &config.slavesMap);

  config.frameCfg = frameCfgArgs;
  config.profileCfg = profileCfgArgs;
  config.chirpCfg = chirpCfgArgs;
  config.adcOutCfg = adcOutCfgArgs;
  config.dataFmtCfg = dataFmtCfgArgs;
  config.channelCfg = channelCfgArgs;
  config.csi2LaneCfg = csi2LaneCfgArgs;
  config.datapathCfg = datapathCfgArgs;
  config.datapathClkCfg = datapathClkCfgArgs;
  config.hsClkCfg = hsClkCfgArgs;
  config.ldoCfg = ldoCfgArgs;
  config.lpmCfg = lpmCfgArgs;
  config.miscCfg = miscCfgArgs;

  if (config_filename != NULL) {
    // Read parameters from config file
    read_config(config_filename, &config);
  }

  /**
   * @note: The adcOutCfg is used to overwrite the dataFmtCfg
   *
   * In a unified config file, it'll make for sense to have a single
   * source of truth for the ADC data format. And therefore use the
   * same data for setting both.
   */
  config.dataFmtCfg.rxChannelEn = channelCfgArgs.rxChannelEn;
  config.dataFmtCfg.adcBits = adcOutCfgArgs.fmt.b2AdcBits;
  config.dataFmtCfg.adcFmt = adcOutCfgArgs.fmt.b2AdcOutFmt;

  // config to ARM the TDA
  rlTdaArmCfg_t tdaCfg = {
    .captureDirectory = capture_path,
    .framePeriodicity = (frameCfgArgs.framePeriodicity * 5)/(1000*1000),
    .numberOfFilesToAllocate = 0,
    .numberOfFramesToCapture = 0, // config.frameCfg.numFrames,
    .dataPacking = 0, // 0: 16-bit | 1: 12-bit
  };

  if ((unsigned char *)get_option(&parser, "configure") != NULL) {
    // Connect to TDA
    status = MMWL_TDAInit(ip_addr, port, config.deviceMap);
    check(status,
      "[MMWCAS-DSP] TDA Connected!",
      "[MMWCAS-DSP] Couldn't connect to TDA board!\n", 32, TRUE);

    // Start configuration
    configure(config);
    msleep(2000);
  }

  if ((unsigned char *)get_option(&parser, "record") != NULL) {
    // Arm TDA
    status = MMWL_ArmingTDA(tdaCfg);
    check(status,
      "[MMWCAS-DSP] Arming TDA",
      "[MMWCAS-DSP] TDA Arming failed!\n", 32, TRUE);

    msleep(2000);

    // Start framing
    for (int i = 3; i >=0; i--) {
      status += MMWL_StartFrame(1U << i);
    }
    check(status,
      "[MMWCAS-RF] Framing ...",
      "[MMWCAS-RF] Failed to initiate framing!\n", config.deviceMap, TRUE);

    msleep((unsigned long int)record_duration);

    // Stop framing
    for (int i = 3; i >= 0; i--) {
      status += MMWL_StopFrame(1U << i);
    }

    status += MMWL_DeArmingTDA();
    check(status,
      "[MMWCAS-RF] Stop recording",
      "[MMWCAS-RF] Failed to de-arm TDA board!\n", 32, TRUE);
    msleep(1000);
  }
  return 0;
}
