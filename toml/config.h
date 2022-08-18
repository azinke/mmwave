/**
 * @file config.h
 * @author AMOUSSOU Z. Kenneth (www.gitlab.com/azinke)
 * @brief TOML configuration file parser
 * @version 0.1
 * @date 2022-08-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MMWAVE_CONFIG_H
#define MMWAVE_CONFIG_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "toml.h"
#include "../mimo.h"

/* Read the configuration from the TOML file  */
int read_config(unsigned char *filename, devConfig_t *config);

/* Read all the config related to MIMO configuration */
void read_mimo_config(toml_table_t* configfile, devConfig_t *config);

#endif
