/****************************************************************************************
 * FileName     : mmwl_port_ethernet.h
 *
 * Description  : This file defines the functions to configure mmwave radar device through
 * 				  the TDA2xx capture card
 *
 ****************************************************************************************
 * (C) Copyright 2014, Texas Instruments Incorporated. - TI web address www.ti.com
 *---------------------------------------------------------------------------------------
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *    Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR CONTRIBUTORS
 *  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/****************************************************************************************
* FILE INCLUSION PROTECTION
****************************************************************************************
*/
#ifndef MMWL_PORT_ETHERNET_H
#define MMWL_PORT_ETHERNET_H

 /*!
 \mainpage mmWaveLink Etherent Framework

 \section intro_sec Introduction

*  As mentioned in the User Guide, the mmWaveStudio uses underlying C DLLs to communicate
*  with the mmWave Devices.These DLL's contains a generic component which is referred
*  to as Mmwavelink. \n
*  Mmwavelink handles the communication protocol between the Host (in this case a PC)
*  and the mmWave devices. Refer the mmWave Interface Control Document (ICD) to understand
*  the protocol in detail. \n
*  - Although Mmwavelink handles the protocol, it still needs to send and receive data over
*  Ethernet cable to the TDA2XX. \n
*  - An Ethernet port layer is responsible for sending the message over Ethernet cable to
*  the TDA2XX. \n
*  - The TDA2XX then converts this data into SPI signals to communicate
*  with the device. \n
*  For more detailed information, refer \ref mmwl_port_ethernet

*  The following diagram explains the whole flow and the place of the port layer in the
*  whole sequence flow.
*
*  @image html portLayerEthernet.jpg

 @note 1: The source code for mmWavelink is shared as part of the DFP release. \n
 */

/******************************************************************************
 * INCLUDE FILES
 ******************************************************************************
 */

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include "mtime.h"

#define getError() (errno)
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR (-1)
#define closesocket(s) close(s)
/* these are not needed for Linux */
#define socketsShutdown()
#define socketsStartup()

typedef int SOCKET;
typedef pthread_t HANDLE;
typedef uint8_t BOOLEAN;

#define Sleep(x) msleep(x)
#define TRUE 1
#define FALSE 0
#define EXPORT // __attribute__((visibility("default")))


/****************************************************************************************
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

#define NETWORK_INVALID_SOCKET							0xFFFFFFFFU
#define NETWORK_ERROR												(-(int32_t)1)
#define NETWORK_SUCCESS											(0)
#define NETWORK_TDA_SERVER_PORT							(5001)
#define TDA_NUM_CONNECTED_DEVICES_MAX				(4U)
/* Avoid Warnings for Unused variables */
#define IGNORE_WARNING_UNUSED_VAR(x)				(x==0)

/*! \brief
* Command Codes
*/

/*! \brief
* TDA related Macros
*/

#define CAPTURE_CONFIG_CONNECT                      (0x10U)
#define CAPTURE_CONFIG_DISCONNECT                   (0x11U)
#define CAPTURE_CONFIG_PING                         (0x12U)
#define CAPTURE_CONFIG_VERSION_GET                  (0x13U)
#define CAPTURE_CONFIG_CONFIG_SET                   (0x14U)
#define CAPTURE_CONFIG_CONFIG_GET                   (0x15U)
#define CAPTURE_CONFIG_TRACE_START                  (0x16U)
#define CAPTURE_CONFIG_TRACE_RETREIVE               (0x17U)
#define CAPTURE_CONFIG_CREATE_APPLICATION           (0x18U)
#define CAPTURE_CONFIG_START_LOGGING_STATS          (0x19U)
#define CAPTURE_CONFIG_DEVICE_MAP                   (0x1AU)


/*! \brief
* Sensor related Macros
*/

#define SENSOR_CONFIG_DEVICE_RESET                  (0x20U)
#define SENSOR_CONFIG_SET_SOP                       (0x21U)
#define SENSOR_CONFIG_GET_SOP                       (0x22U)
#define SENSOR_CONFIG_SPI_WRITE                     (0x23U)
#define SENSOR_CONFIG_SPI_READ                      (0x24U)
#define SENSOR_CONFIG_GPIO_CONFIG                   (0x25U)
#define SENSOR_CONFIG_GPIO_SET                      (0x26U)
#define SENSOR_CONFIG_GPIO_GET                      (0x27U)


/*! \brief
* Capture related Macros
*/

#define CAPTURE_DATA_START_RECORD                   (0x31U)
#define CAPTURE_DATA_STOP_RECORD                    (0x32U)
#define CAPTURE_DATA_FRAME_PERIODICITY              (0x35U)
#define CAPTURE_DATA_NUM_ALLOCATED_FILES            (0x36U)
#define CAPTURE_DATA_ENABLE_DATA_PACKAGING          (0x37U)
#define CAPTURE_DATA_SESSION_DIRECTORY              (0x38U)
#define CAPTURE_DATA_NUM_FRAMES                     (0x39U)


/*! \brief
* Command Response Codes
*/

#define CAPTURE_RESPONSE_ACK                        (0x81U)
#define CAPTURE_RESPONSE_NACK                       (0x82U)
#define CAPTURE_RESPONSE_VERSION_INFO               (0x83U)
#define CAPTURE_RESPONSE_CONFIG_INFO                (0x84U)
#define CAPTURE_RESPONSE_TRACE_DATA                 (0x85U)
#define CAPTURE_RESPONSE_GPIO_DATA                  (0x86U)
#define CAPTURE_RESPONSE_PLAYBACK_DATA              (0x87U)
#define CAPTURE_RESPONSE_HOST_IRQ                   (0x88U)
#define SENSOR_RESPONSE_SPI_DATA                    (0x89U)
#define SENSOR_RESPONSE_SOP_INFO                    (0x8AU)
#define CAPTURE_RESPONSE_NETWORK_ERROR              (0x8DU)


/*! \brief
* Packet Formation Codes
*/

#define TX_SYNC_BYTE                                (0xA5C8U)
#define RX_SYNC_BYTE                                (0x8C5AU)
#define MAX_DATA_LENGTH                             (498U)
#define DATA_HEADER_LENGTH                          (12U)
#define HEADER_AND_CRC_LENGTH                       (4U)


/*! \brief
* ACK Codes
*/

#define ACK_ON_PROCESS                              (0x01U)
#define ACK_ON_RECEIVE                              (0x10U)
#define ACK_NOT_REQUIRED                            (0x11U)


/*! \brief
* Status Codes
*/

#define SYSTEM_LINK_STATUS_SOK                      (0U)
#define SYSTEM_LINK_STATUS_EFAIL                    (1U)
#define STATUS                                      int32_t


/*! \brief
* SOP Modes
*/

#define SOP_MODE_SCAN_APTG                          (1U)
#define SOP_MODE_DEVELOPMENT                        (2U)
#define SOP_MODE_THB                                (3U)
#define SOP_MODE_FUNCTIONAL                         (4U)
#define SOP_MODE_FLASH_DOWNLOAD                     (5U)
#define SOP_MODE_FUNCTIONAL_CPHA1                   (6U)
#define SOP_MODE_FUNCTIONAL_I2C                     (7U)


/*! \brief
* CRC related Macros
*/

#define RL_CRC_TYPE_16BIT_CCITT                     (0U)

#define BSPDRV_AR12XX_CRC_M_CRC_TABLE_SIZE          (256U)
#define BSPDRV_AR12XX_CRC_INITIAL_REMAINDER         (0xFFFFU)

#define BSPDRV_AR12XX_CRC_M_ZERO                    (0U)
#define BSPDRV_AR12XX_CRC_M_ONE                     (1U)
#define BSPDRV_AR12XX_CRC_M_TWO                     (2U)
#define BSPDRV_AR12XX_CRC_M_THREE                   (3U)
#define BSPDRV_AR12XX_CRC_M_FOUR                    (4U)
#define BSPDRV_AR12XX_CRC_M_EIGHT                   (8U)

#define BSPDRV_AR12XX_CRC_M_REG_READ8(w_addr)       \
    ((uint8_t)(*((uint8_t *)(w_addr))))
#define BSPDRV_AR12XX_CRC_M_REG_READ32(w_addr)      \
    ((uint32_t)(*((uint32_t *)(w_addr))))

#define BSP_SOK                                     ((int32_t) 0)
#define BSP_EBADARGS                                (-((int32_t) 2))

/*! \brief
* Return Codes
*/
#define RLS_RET_CODE_OK                             (0)
#define RLS_RET_CODE_EFAIL                          (-1)

#ifndef DEBUG
#define DEBUG 0
#endif

/*! \brief
* Function used for logging the messages
*/
#if defined(DEBUG) && DEBUG > 0
  #define DEBUG_PRINT(fmt, args...)\
    fprintf(stderr, "DEBUG [%s:%d]: " fmt, __FILE__, __LINE__, ##args)
#else
 #define DEBUG_PRINT(fmt, args...) /* Don't do anything in release builds */
#endif

/******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 ******************************************************************************
 */

/*! \brief
 * Network Socket Structure
 */
typedef struct {

  /**
   * @ brief  Client Socket ID
   */
  SOCKET clientSocketId;

} Network_SockObj;


/*! \brief
 * Network Command Header Structure
 */
typedef struct {

    /**
     * @brief  Size of input parameters in units of bytes. \n
      *				 Can be 0 if no parameters need to send for a command \n
     */
  unsigned int prmSize;

} NetworkTDA_CmdHeader;


/*! \brief
  * Network Connection Structure
  */
typedef struct {

  /**
   * @brief  Server Port
   */
  uint16_t serverPort;
  /**
   * @brief  IP Address of the capture card
   */
  char ipAddr[32];

} NetworkTDA_Obj;


/*! \brief
 * H/W version Structure
 */
typedef struct HWVersion_t {

  /**
   * @brief  Major version
   */
  uint8_t hwMajorVer;
  /**
   * @brief  Minor version
   */
  uint8_t hwMinorVer;
  /**
   * @brief  Debug version
   */
  uint8_t hwDebugVer;
  /**
   * @brief  Build version
   */
  uint8_t hwBuildVer;

} s_HWVersion_t;


/*! \brief
 * DLL version Structure
 */
typedef struct dllVersion_t {

  /**
   * @brief  Major version
   */
  uint8_t dllMajorVer;
  /**
   * @brief  Minor version
   */
  uint8_t dllMinorVer;
  /**
   * @brief  Debug version
   */
  uint8_t dllDebugVer;
  /**
   * @brief  Build version
   */
  uint8_t dllBuildVer;

} s_DLLVersion_t;


/*! \brief
 * Data Capture Config Structure
 */
typedef struct dataCaptureConfig_t {

  /**
   * @brief  HSI Packet Configuration
   */
  uint8_t hsiPktConfig;
  /**
   * @brief  Data format mode
   */
  uint8_t dataFormatMode;
  /**
   * @brief  Lanes
   */
  uint8_t lanePosition[4];

} s_dataCaptureConfig_t;


/*! \brief
 *  GPIO Config Structure
 */
typedef struct gpioConfig_t {

  /**
   * @brief  GPIO Pad
   */
  uint32_t gpioBase;
  /**
   * @brief  GPIO Pin
   */
  uint32_t gpioPin;
  /**
   * @brief  GPIO Value
   */
  uint32_t gpioValue;

} s_gpioConfig_t;


/*! \brief
 *  Control Messages Structure
 */
typedef struct {

  /**
   * @brief  Request Parameters
   */
  uint8_t * reqParam;
  /**
   * @brief  Response Parameters
   */
  uint8_t respParam[512];
  /**
   * @brief  Request Parameter size
   */
  uint32_t  reqParamSize;
  /**
   * @brief  Response Parameter size
   */
  uint32_t  respParamSize;

} DevComm_NetworkCtrlReqPrms;


/*! \brief
 *  Ethernet Packet Structure
 */
typedef struct {

  /**
   * @brief  Sync Byte of Header
   */
  uint16_t syncByte;
  /**
   * @brief  Command Inside the packet
   */
  uint16_t opcode;
  /**
   * @brief  ACK code for the packet
   */
  uint16_t ackCode;
  /**
   * @brief  length of the packet except sync byte and CRC
   */
  uint16_t dataLength;
  /**
   * @brief  Device selection
   */
  uint8_t devSelection;
  /**
   * @brief  01 - ACK on process; 10 - ACK on receive; 11 - No ACK required
   */
  uint8_t ackType;
  /**
   * @brief  Reserved for future use
   */
  uint8_t reserved[4];
  /**
   * @brief  data inside the packet
   */
  uint8_t data[MAX_DATA_LENGTH];

} Radar_EthDataPacketPrms;


/*! \brief
 *  Callback handler with mmWaveLink
 */
typedef void(*EVENT_HANDLER)
(
  /**
   * @brief  device map
   */
  uint8_t devSelection,
  /**
   * @brief  pointer to the data buffer/value
   */
  void* pValue
);


/*! \brief
 *  Callback handler with mmWaveStudio
 */
typedef void (*TDA_EVENT_HANDLER) (
  /**
   * @brief  device map
   */
  uint16_t devSelection,
  /**
   * @brief  command code
   */
  uint16_t u16CmdCode,
  /**
   * @brief  ACK code
   */
  uint16_t u16ACKCode,
  /**
   * @brief  status
   */
  int32_t u16Status,
  /**
   * @brief  pointer to the data buffer
   */
  void* data
);


/*! \brief
*  Capture Configuration Structure
*/
typedef struct CaptureConfig {
  /**
   * @brief  width of the capture application
   */
  uint32_t width;
  /**
   * @brief  height of the capture application
   */
  uint32_t height;

} captureConfig_t;


/*! \brief
 *  Host Interrupt Thread Structure
 */
typedef struct TDAhostIntrThread {
  /**
   * @brief  thread handle
   */
  HANDLE threadHdl;
  /**
   * @brief  ID of the thread
   */
  uint32_t threadID;
  /**
     * @brief  callback handler with mmWaveLink
     */
  EVENT_HANDLER handler;
  /**
   * @brief  pointer to the data buffer/value
   */
  void* pValue;
  /**
   * @brief  Sync Object for this thread
   */
  sem_t	eventHandle;

} TDAThreadParam_t;


/*! \brief
 *  Device Context Structure
 */
typedef struct TDADevCtx {
  /**
   * @brief device index
   */
  uint8_t deviceIndex;
  /**
   * @brief host IRQ status
   */
  volatile uint32_t hostIrqRxHigh;
  /**
   * @brief device enable/disable
   */
  BOOLEAN deviceEnabled;
  /**
   * @brief communication interface open/close
   */
  BOOLEAN interfaceOpened;
  /**
   * @brief  irq mask/unmask
   */
  BOOLEAN irqMasked;
  /**
   * @brief  critical section
   */
  pthread_mutex_t cs;
  /**
   * @brief  handle to host interrupt thread
   */
  TDAThreadParam_t hostIntrThread;

} TDADevCtx_t;


/*! \brief
 *  Device Handle
 */
typedef void* TDADevHandle_t;

/*! \brief
*  Device Map Structure
*/
typedef struct {
  /**
   * @brief  Master Enable
   */
  uint8_t isMasterEnable;
  /**
   * @brief  Slave1 Enable
   */
  uint8_t isSlave1Enable;
  /**
   * @brief  Slave2 Enable
   */
  uint8_t isSlave2Enable;
  /**
   * @brief  Slave3 Enable
   */
  uint8_t isSlave3Enable;
  /**
   * @brief  Number of device
   */
  uint32_t numDevice;

} NetworkRadarDeviceMap_param;


/**
*  @defgroup mmwl_port_ethernet mmwl_port_ethernet
*  @brief mmWaveLink Ethernet Library
*
*    Related Files
*   - mmwl_port_ethernet.c
*  @addtogroup mmwl_port_ethernet
*  @{
*/

/******************************************************************************
 * FUNCTION DECLARATIONS
 ******************************************************************************
 */

STATUS Radar_formEthDataPacket(Radar_EthDataPacketPrms *pDataPacket,
                               DevComm_NetworkCtrlReqPrms *pDevCtrlPrms,
                               uint16_t responseCode,
                               uint16_t dataLength,
                               uint8_t* data);

STATUS Radar_processData(Radar_EthDataPacketPrms *pDataPacket_ptr,
                         uint32_t prmSize);

/********************** Capture Card Configuration API's *********************/

EXPORT STATUS ethernetConnect(unsigned char *ipAddr,
                              uint32_t configPort,
                              uint32_t deviceMap);

EXPORT STATUS ethernetDisconnect();

EXPORT STATUS IsConnected();

EXPORT STATUS ConfigureDeviceMap(uint32_t deviceMap);

EXPORT STATUS TDACreateApplication();

EXPORT STATUS readHWVersion();

EXPORT STATUS readDLLVersion();

EXPORT STATUS setWidthAndHeight(uint8_t devSelection,
                                uint32_t width,
                                uint32_t height);

EXPORT STATUS getWidthAndHeight(uint8_t devSelection);

EXPORT STATUS TDAregisterCallback(uint8_t devSelection,
                                  EVENT_HANDLER RF_EventCallback,
                                  void* pValue);

EXPORT STATUS registerTDAStatusCallback(TDA_EVENT_HANDLER TDACard_EventCallback);

EXPORT STATUS startTrace(unsigned char *filename);

EXPORT STATUS stopTrace();

EXPORT STATUS showCPUStats();

EXPORT STATUS sendFramePeriodicitySync(unsigned int framePeriodicity);

EXPORT STATUS sendNumAllocatedFiles(unsigned int numAllocatedFiles);

EXPORT STATUS enableDataPackaging(unsigned int enablePackaging);

EXPORT STATUS setSessionDirectory(unsigned char *sessionDirectory);

EXPORT STATUS NumFramesToCapture(unsigned int numFrames);

/********************** Sensor Configuration API's ***************************/

EXPORT STATUS spiWriteToDevice(TDADevHandle_t hdl,
                               unsigned char *data,
                               unsigned short ByteCount);

EXPORT STATUS spiReadFromDevice(TDADevHandle_t hdl,
                                unsigned char *data,
                                unsigned short ByteCount);

EXPORT STATUS resetDevice(TDADevHandle_t hdl);

EXPORT STATUS setSOPMode(TDADevHandle_t hdl,
                         uint32_t SOPmode);

EXPORT STATUS getSOPMode(TDADevHandle_t hdl);

EXPORT STATUS gpioGetValue(unsigned int DeviceMap,
                           unsigned int gpioBase,
                           unsigned int gpioPin);

EXPORT STATUS gpioSetValue(unsigned int DeviceMap,
                           unsigned int gpioBase,
                           unsigned int gpioPin,
                           unsigned int gpioVal);

/********************** Capture Configuration API's **************************/

EXPORT STATUS startRecord();

EXPORT STATUS stopRecord();

/********************** mmWaveLink Hooks related functions *******************/

EXPORT TDADevCtx_t* TDAInitDeviceCtx(unsigned char ucDevId);

EXPORT int TDAClearDeviceCtx(TDADevCtx_t* pDevCtx);

EXPORT TDADevHandle_t TDAGetDeviceCtx(unsigned char ucDevId);

EXPORT TDADevHandle_t TDACommOpen(unsigned char deviceIndex,
                         unsigned int flags);

EXPORT int TDACommClose(TDADevHandle_t hdl);

EXPORT int TDACloseDevice(TDADevCtx_t* pDevCtx);

EXPORT int TDADisableDeviceThread(unsigned char deviceIndex);

EXPORT int TDAEnableDeviceThread(unsigned char deviceIndex);

EXPORT int TDADisableDevice(unsigned char deviceIndex);

EXPORT int TDAEnableDevice(unsigned char deviceIndex);

EXPORT int TDALockDevice(TDADevCtx_t*  pDevCtx);

EXPORT int TDAUnlockDevice(TDADevCtx_t*  pDevCtx);

EXPORT void TDACommIRQMask(TDADevHandle_t hdl);

EXPORT void TDACommIRQUnMask(TDADevHandle_t hdl);

EXPORT int TDADeviceWaitIrqStatus(TDADevHandle_t hdl,
                           unsigned char level);

EXPORT int TDAWaitForIrq(TDADevCtx_t* pDevCtx);

EXPORT int TDAStartIrqPollingThread(TDADevCtx_t* pDevCtx);

EXPORT int TDAStopIrqPollingThread(TDADevCtx_t* pDevCtx);

EXPORT unsigned int getDevIdFromDevMap(unsigned int deviceMap);

EXPORT unsigned int createDevMapFromDevId(unsigned int deviceId);

/********************** Network related functions ****************************/

EXPORT int Network_init();

EXPORT int Network_deInit();

EXPORT int Network_connect(Network_SockObj *pObj,
                      char *ipAddr,
                      uint32_t port);

EXPORT int Network_close(Network_SockObj *pObj);

EXPORT int Network_read(Network_SockObj *pObj,
                     uint8_t *dataBuf,
                     uint32_t *dataSize);

EXPORT int Network_readString(Network_SockObj *pObj,
                       uint8_t *dataBuf,
                       uint32_t maxDataSize);

EXPORT int Network_write(Network_SockObj *pObj,
                    uint8_t *dataBuf,
                    uint32_t dataSize);

EXPORT int32_t Network_waitRead(Network_SockObj *pObj);

EXPORT int ConnectToServer();

EXPORT int CloseConnection();

EXPORT int SendCommand(void *params,
                    int prmSize);

EXPORT int RecvResponseParams(uint8_t *pPrm,
                       uint32_t prmSize);

EXPORT int RecvResponse(uint32_t *prmSize);

EXPORT void handleDevCtrl(uint8_t *pDataBuf, uint32_t size);

/********************** Computation of CRC *********************/

int32_t Bsp_ar12xxComputeCrc(uint8_t* wMbDataBaseAdd,
                             uint32_t hNBytes,
                             uint8_t crcLen,
                             uint8_t* outCrc);

/*!
 Close the Doxygen group.
 @}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
/*
 * END OF MMWL_PORT_ETHERNET_H FILE
 */
