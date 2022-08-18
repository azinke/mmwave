/**
 * @file mimo.h
 * @author AMOUSSOU Z. Kenneth (www.gitlab.com/azinke)
 * @brief MMWave Radar configuration and control tool
 * @version 0.1
 * @date 2022-08-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MMWAVE_MIMO_H
#define MMWAVE_MIMO_H

#include <string.h>
#include <signal.h>
#include "ti/mmwave/mmwave.h"
#include "opt/opt.h"

#define PROG_NAME       "mmwave"              // Name of the program
#define PROG_VERSION    "0.1"                 // Program version
#define PROG_COPYRIGHT  "Copyright (C) 2022"

/* Enable development environment
  Status messages are printed. Set to '0' to disable the
  development environment. Hence, no status feedback will
  be printed
*/
#define DEV_ENV    1

#define NUM_CHIRPS 12

#define CRED      "\e[0;31m"    // Terminal code for regular red text
#define CGREEN    "\e[0;32m"    // Terminal code for regular greed text
#define CRESET    "\e[0m"       // Clear reset terminal color


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

  // Data format config
  rlDevDataFmtCfg_t dataFmtCfg;

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

#endif
