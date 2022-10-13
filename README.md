# mmwave

The MMWCAS-RF-EVM and MMWCAS-DSP-EVM boards from Texas Instruments (TI) are supported
with the TI-provided software `mmwave studio`. As so, one needs a Windows OS and Matlab
runtime engine to try out the millimeter wave radars and their capabilities.

This tool is a Linux driver for the RF and DSP boards, to enable the recording of data
from a Linux OS. As so, it can be built to run on embedded Linux devices such as
Raspberry Pi or so.

`mmwave` is based on the `mmwavelink` library and build out of the example source
codes provided by TI.

**NOTE:**
- Only MIMO Configuration with all the 04 radar chips is currently supported with this driver.
- Only Ethernet connection to the board is currently supported.


## Installation

All the third-party libraries needed are already present in the repository.
`mmwave` can then be installed as follows:

```bash
    git clone <mmwave-repository-git-url>
    cd mmwave
    sudo make install # Build and install mmwave on the machine
```

## Usage

You can first check if `mmwave` is properly installed by typing the `help` command.

```bash
 mmwave -h
```

You shall the see a help menu similar to the one below.

```txt
usage: mmwave [-d] [-p] [-i] [-c] [-r] [-t] [-f] [-h] [-v]

Configuration and control tool for TI MMWave cascade Evaluation Module

options:
    -d, --capture-dir              Name of the director where to store recordings on the DSP board 
    -p, --port                     Port number the DSP board server app is listening on 
    -i, --ip-addr                  IP Address of the MMWCAS DSP evaluation module 
    -c, --configure                Configure the MMWCAS-RF-EVM board 
    -r, --record                   Trigger data recording. This assumes that configuration is completed. 
    -t, --time                     Indicate how long the recording should last in minutes. Default: 1 min 
    -f, --cfg                      TOML Configuration file. Overwrite the default config when provided 
    -h, --help                     Print CLI option help and exit. 
    -v, --version                  Print program version and exit.
```

A default configuration is already implemented (as described below) and can be used.

```yaml
    ip: 192.168.33.180      # MMWCAS-DSP-EVM IP Address
    port: 5001              # MMWCAS-DSP-EVM server port
    time: 1                 # Recording time in min
```

If the DSP board has been reconfigured with another IP address, you can provide the new
IP address in argument with the `--ip-addr` CLI option.

## Recording data

### Default config

To record data, the typical command is:

```bash
mmwave -d <directory> --configure --record --time <duriation-in-minute>

# Exmaple
mmwave -d outdoor0 --configure --record --time 10
```

With this command, the radar chips are configured with the default configuration
implemented in the source code.

```yaml
# Default MIMO Configuration
#
#   - Max Range             : 80 m
#   - Range resolution      : 30 cm
#   - Max velocity          : 6.49 km/h
#   - Range resolution      : 0.4 km/h
#

profile:
    id: 0
    startFrequency: 77                      # GHz
    frequencySlope: 15.0148                 # MHz/us
    idleTime: 5                             # us
    adcStartTime: 6                         # us
    rampEndTime: 40                         # us
    numAdcSamples: 256                      # ADC Samples per chirp
    rxGain: 48                              # dB

frame:
    numLoops: 16                            # Number of chirp loops per frame
    numFrames: 0                            # Number of frames to record. 0: Infinte framing
    framePeriodicity: 100                   # ms

channel:
    rxChannelEn: 0xF                        # RX Channels enabled
    rxChannelEn: 0x7                        # TX Channel enabled

dataFmt:
    iqSwapSel: 0                            # I first
```

If the capture directory is not indicated (with the `-d` option), the capture folder is
automatically created as `MMW_Capture_<timestamp>`; with `<timesamp>` is placeholder for
the Unix timestamp at which the command has been issued.

### Self-defined configuration

It's possible to define custom configurations suitable for a given recording setup
with TOML config files. Some examples of config files are present on the `config`
folder of this repository.

With a config file, one can use the command below:

```bash
mmwave -f <path-to-config-file> --configure --record --time <duration-in-min>

# Example
mmwave -f config/short-range-cfg.toml --configure --record --time 2
```

### Check and copy recorded data

With the MMWCAS-DSP-EVM board, recordings are saved on its embedded Solid State
Drive (SSD). To check that the recording has proceeded and is successful, you can
log over `ssh` onto the DSP board as follows:

```bash
# Use the IP address of the DSP board
ssh root@192.168.33.180

cd /mnt/ssd

ls -l
```

In the list of folders, you can then check for the name of the folder holding the
recorded data and make sure that it contains some data.

To copy the files over Ethernet, one can use a secure copy program such as `scp`.

```bash
scp root@192.168.33.180:/mnt/ssd/<recording-directory> <path-to-local-storage>

# Example
scp root@192.168.33.180:/mnt/ssd/outdoor0 /home/user/rwu-radar
```

## Developer note

The structure of the repository is as follows:

```txt
.
├── config
│   └── short-range-cfg.toml
├── makefile
├── mimo.c
├── mimo.h
├── mmwave
├── opt
│   ├── opt.c
│   └── opt.h
├── README.md
├── ti
│   ├── ethernet
│   │   ├── docs
│   │   └── src
│   ├── firmware
│   │   ├── masterss
│   │   ├── radarss
│   │   ├── xwr22xx_metaImage.bin
│   │   └── xwr22xx_metaImage.h
│   ├── mmwave
│   │   ├── crc_compute.c
│   │   ├── mmwave.c
│   │   ├── mmwave.h
│   │   ├── rls_osi.c
│   │   └── rls_osi.h
│   └── mmwavelink
│       ├── docs
│       ├── include
│       ├── lib
│       ├── makefile
│       ├── makefiles
│       ├── mmwavelink.h
│       └── src
└── toml
    ├── config.c
    ├── config.h
    ├── toml.c
    └── toml.h
```

The content of the folders `ti/mmwavelink` and `ti/firmware` must not be modified. Those
are respectively libraries and firmware provided by Texas Instruments and should not be
modified unless one knows what it's all about. Any update to those can be directly obtained
from TI or by copying them from the installation folder of `mmwave studio` and
`mmwave dfp`.

The folders `ti/ethernet` and `ti/mmwave` are based on examples source files provided by
TI. Since the original sources were only compatible with windows, these have been modified
to operate on Linux OS. One can update these modules to extend the capabilities of this driver.

- The folder `opt` holds the source handling the CLI option parsing
- The `toml` folder handles the parsing of configuration files.
- The entry point of the program is in the `mimo.c` file.

**NOTE**: the files `toml/toml.c` and `toml/toml.h` have been authored by 
[cktan](https://github.com/cktan) and released with an MIT license on Github at
https://github.com/cktan/tomlc99. Therefore, this reference is the perfect place to
seek for new updates of the `TOML` parser library.
