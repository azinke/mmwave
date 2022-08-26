/**
 * @file config.c
 * @author AMOUSSOU Z. Kenneth (www.gitlab.com/azinke)
 * @brief TOML configuration file parser
 * @version 0.1
 * @date 2022-08-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "config.h"

#define CONFIG_FIELD_ERROR_MSG "Error with config parameter "


/**
 * @brief Read MIMO related sections of the config file
 *
 * @param configfile TOML Table parsed from the config file
 * @param config Device config
 * @return int
 */
void read_mimo_config(toml_table_t* configfile, devConfig_t *config) {
    if (configfile == NULL) return;
    toml_table_t *mimo = toml_table_in(configfile, "mimo");
    if (mimo != NULL) {
        toml_datum_t data;

        // [PROFILE CONFIGURATION]
        toml_table_t *profile = toml_table_in(mimo, "profile");
        if (profile != NULL) {
            data = toml_int_in(profile, "id");
            if (data.ok) {
                config->profileCfg.profileId = (uint16_t)data.u.i;
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'id'\n");
            }

            // Chirp start frequency in GHz
            data = toml_double_in(profile, "startFrequency");
            if (data.ok) {
                config->profileCfg.startFreqConst = (uint32_t)ceil(
                    // 1LSB = 53.644 Hz
                    (data.u.d * 1e9) / 53.644);
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'startFrequency'\n");
            }

            // Frequency slope in MHz/us
            data = toml_double_in(profile, "frequencySlope");
            if (data.ok) {
                config->profileCfg.freqSlopeConst = (uint16_t)ceil(
                    // 1LSB = 48.279 kHz/us
                    (data.u.d * 1e3) / 48.279);
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'frequencySlope'\n");
            }

            // Chrip Idle time in us
            data = toml_double_in(profile, "idleTime");
            if (data.ok) {
                config->profileCfg.idleTimeConst = (uint32_t)ceil(
                    // 1LSB = 10ns
                    (data.u.d * 1e2));
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'idleTime'\n");
            }

            // ADC start time in us
            data = toml_double_in(profile, "adcStartTime");
            if (data.ok) {
                config->profileCfg.adcStartTimeConst = (uint32_t)ceil(
                    // 1LSB = 10ns
                    (data.u.d * 1e2));
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'adcStartTime'\n");
            }

            // Chirp ramp end time in us
            data = toml_double_in(profile, "rampEndTime");
            if (data.ok) {
                config->profileCfg.rampEndTime = (uint32_t)ceil(
                    // 1LSB = 10ns
                    (data.u.d * 1e2));
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'rampEndTime'\n");
            }

            // TX starttime in us
            data = toml_double_in(profile, "txStartTime");
            if (data.ok) {
                config->profileCfg.txStartTime = (uint16_t)ceil(
                    // 1LSB = 10ns
                    (data.u.d * 1e2));
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'txStartTime'\n");
            }

            // Number of ADC samples per chirp
            data = toml_int_in(profile, "numAdcSamples");
            if (data.ok) {
                config->profileCfg.numAdcSamples = (uint16_t)data.u.i;
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'numAdcSamples'\n");
            }

            // ADC sampling frequency in ksps
            data = toml_int_in(profile, "adcSamplingFrequency");
            if (data.ok) {
                config->profileCfg.digOutSampleRate = (uint16_t)data.u.i;
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'adcSamplingFrequency'\n");
            }

            // rxGain in dB
            data = toml_int_in(profile, "rxGain");
            if (data.ok) {
                config->profileCfg.rxGain = (uint16_t)data.u.i;
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'rxGain'\n");
            }

            // hpfCornerFreq1
            data = toml_int_in(profile, "hpfCornerFreq1");
            if (data.ok) {
                config->profileCfg.hpfCornerFreq1 = (uint8_t)data.u.i;
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'hpfCornerFreq1'\n");
            }

            // hpfCornerFreq2
            data = toml_int_in(profile, "hpfCornerFreq2");
            if (data.ok) {
                config->profileCfg.hpfCornerFreq2 = (uint8_t)data.u.i;
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'hpfCornerFreq2'\n");
            }
        }

        // [FRAME CONFIGURATION]
        toml_table_t *frame = toml_table_in(mimo, "frame");
        if (frame != NULL) {
            // Number of frames to record
            data = toml_int_in(frame, "numFrames");
            if (data.ok) {
                config->frameCfg.numFrames = (uint16_t)data.u.i;
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'numFrames'\n");
            }

            // Number of chirp loop per frame
            data = toml_int_in(frame, "numLoops");
            if (data.ok) {
                config->frameCfg.numLoops = (uint16_t)data.u.i;
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'numLoops'\n");
            }

            // Frame periodicity in ms
            data = toml_double_in(frame, "framePeriodicity");
            if (data.ok) {
                config->frameCfg.framePeriodicity = (uint32_t)ceil(
                    // 1LSB = 5ns
                    (data.u.d * 1e-3) / 5e-9);
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'framePeriodicity'\n");
            }
        }

        // [CHANNEL CONFIGURATION]
        toml_table_t *channel = toml_table_in(mimo, "channel");
        if (channel != NULL) {
            // RX Channel configuration
            data = toml_int_in(channel, "rxChannelEn");
            if (data.ok) {
                config->channelCfg.rxChannelEn = (uint16_t)data.u.i;
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'rxChannelEn'\n");
            }

            // TX Channel configuration
            data = toml_int_in(channel, "txChannelEn");
            if (data.ok) {
                config->channelCfg.txChannelEn = (uint16_t)data.u.i;
            } else {
                DEBUG_PRINT(CONFIG_FIELD_ERROR_MSG "'txChannelEn'\n");
            }
        }
        config->frameCfg.numAdcSamples = 2 * config->profileCfg.numAdcSamples;
        config->dataFmtCfg.rxChannelEn = config->channelCfg.rxChannelEn;
    }
}

/**
 * @brief Read a configuration file and set the device
 *        configuration
 *
 * @param filename Path to the config file
 * @param config Device configuration structure
 * @return int Status
 */
int read_config(unsigned char *filename, devConfig_t *config) {
    FILE *fp = fopen(filename, "r");
    unsigned char err[200]; // Error buffer
    if (fp == NULL) {
        DEBUG_PRINT(" Unable to read the configuration file '%s'\n\n", filename);
        exit(1);
    }
    toml_table_t * configfile = toml_parse_file(fp, err, sizeof(err));
    read_mimo_config(configfile, config);
    toml_free(configfile);
    return 0;
}
