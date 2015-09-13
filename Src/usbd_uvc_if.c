/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          :
  ******************************************************************************
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_uvc_if.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_UVC 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_UVC_Private_TypesDefinitions
  * @{
  */ 
  /* USER CODE BEGIN 0 */ 
  /* USER CODE END 0 */ 
/**
  * @}
  */ 

/** @defgroup USBD_UVC_Private_Defines
  * @{
  */ 
  /* USER CODE BEGIN 1 */
/* Define size for the receive and transmit buffer over UVC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  4
#define APP_TX_DATA_SIZE  4
  /* USER CODE END 1 */  
/**
  * @}
  */ 

/** @defgroup USBD_UVC_Private_Macros
  * @{
  */ 
  /* USER CODE BEGIN 2 */ 
  /* USER CODE END 2 */
/**
  * @}
  */ 
  
/** @defgroup USBD_UVC_Private_Variables
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Send Data over USB UVC are stored in this buffer       */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USB handler declaration */
/* Handle for USB Full Speed IP */
USBD_HandleTypeDef  *hUsbDevice_0;

extern USBD_HandleTypeDef hUsbDeviceFS;

//data array for Video Probe and Commit
__ALIGN_BEGIN USBD_UVC_VideoControlTypeDef videoCommitControl __ALIGN_END =
{
  {0x00,0x00},                      // bmHint
  {0x01},                           // bFormatIndex
  {0x01},                           // bFrameIndex
  {DBVAL(0),},          // dwFrameInterval
  {0x00,0x00,},                     // wKeyFrameRate
  {0x00,0x00,},                     // wPFrameRate
  {0x00,0x00,},                     // wCompQuality
  {0x00,0x00,},                     // wCompWindowSize
  {0x00,0x00,},                      // wDelay
  {DBVAL(MAX_FRAME_SIZE(80,60,12)),},    // dwMaxVideoFrameSize
  {DBVAL(VIDEO_PACKET_SIZE),},         // dwMaxPayloadTransferSize
  {0x00, 0x00, 0x00, 0x00},         // dwClockFrequency
  {0x00},                           // bmFramingInfo
  {0x00},                           // bPreferedVersion
  {0x00},                           // bMinVersion
  {0x00},                           // bMaxVersion
};

__ALIGN_BEGIN USBD_UVC_VideoControlTypeDef videoProbeControl __ALIGN_END =
{
  {0x00,0x00},                      // bmHint
  {0x01},                           // bFormatIndex
  {0x01},                           // bFrameIndex
  {DBVAL(0),},          // dwFrameInterval
  {0x00,0x00,},                     // wKeyFrameRate
  {0x00,0x00,},                     // wPFrameRate
  {0x00,0x00,},                     // wCompQuality
  {0x00,0x00,},                     // wCompWindowSize
  {0x00,0x00,},                      // wDelay
  {DBVAL(MAX_FRAME_SIZE(80,60,12)),},    // dwMaxVideoFrameSize
  {DBVAL(VIDEO_PACKET_SIZE),},         // dwMaxPayloadTransferSize
  {0x00, 0x00, 0x00, 0x00},         // dwClockFrequency
  {0x00},                           // bmFramingInfo
  {0x00},                           // bPreferedVersion
  {0x00},                           // bMinVersion
  {0x00},                           // bMaxVersion
};

void print_vc(USBD_UVC_VideoControlTypeDef* vc)
{
  printf("bmHint %x %x\r\n", vc->bmHint[0], vc->bmHint[1]);                      // 2
  printf("bFormatIndex %x\r\n", vc->bFormatIndex[0]);                // 3
  printf("bFrameIndex %x\r\n", vc->bFrameIndex[0]);                 // 4
  printf("dwFrameInterval %x %x %x %x\r\n", vc->dwFrameInterval[0],
                                            vc->dwFrameInterval[1],
                                            vc->dwFrameInterval[2],
                                            vc->dwFrameInterval[3]);             // 8
  printf("wKeyFrameRate %x %x\r\n", vc->wKeyFrameRate[0], vc->wKeyFrameRate[1]);               // 10
  printf("wPFrameRate %x %x\r\n", vc->wPFrameRate[0], vc->wPFrameRate[1]);                 // 12
  printf("wCompQuality %x %x\r\n", vc->wCompQuality[0], vc->wCompQuality[1]);                // 14
  printf("wCompWindowSize %x %x\r\n", vc->wCompWindowSize[0], vc->wCompWindowSize[1]);             // 16
  printf("wDelay %x %x\r\n", vc->wDelay[0], vc->wDelay[1]);                      // 18
  printf("dwMaxVideoFrameSize %x %x %x %x\r\n", vc->dwMaxVideoFrameSize[0],
                                                 vc->dwMaxVideoFrameSize[1],
                                                 vc->dwMaxVideoFrameSize[2],
                                                 vc->dwMaxVideoFrameSize[3]);// 22
  printf("dwMaxPayloadTransferSize %x %x %x %x\r\n", vc->dwMaxPayloadTransferSize[0],
                                                     vc->dwMaxPayloadTransferSize[1],
                                                     vc->dwMaxPayloadTransferSize[2],
                                                     vc->dwMaxPayloadTransferSize[3]);    // 26
#ifdef UVC_1_1
  printf("dwClockFrequency %x %x %x %x\r\n", vc->dwClockFrequency[0],
                                              vc->dwClockFrequency[1],
                                              vc->dwClockFrequency[2],
                                              vc->dwClockFrequency[3]);
  printf("bmFramingInfo %x\r\n", vc->bmFramingInfo[0]);
  printf("bPreferedVersion %x\r\n", vc->bPreferedVersion[0]);
  printf("bMinVersion %x\r\n", vc->bMinVersion[0]);
  printf("bMaxVersion %x\r\n", vc->bMaxVersion[0]);
#endif
}

/**
  * @}
  */ 
  
/** @defgroup USBD_UVC_Private_FunctionPrototypes
  * @{
  */
static int8_t UVC_Init_FS     (void);
static int8_t UVC_DeInit_FS   (void);
static int8_t UVC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t index, uint16_t value);
static int8_t UVC_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

USBD_UVC_ItfTypeDef USBD_Interface_fops_FS = 
{
  UVC_Init_FS,
  UVC_DeInit_FS,
  UVC_Control_FS,  
  UVC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  UVC_Init_FS
  *         Initializes the UVC media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t UVC_Init_FS(void)
{
  hUsbDevice_0 = &hUsbDeviceFS;
  /* USER CODE BEGIN 3 */ 
  /* Set Application Buffers */
  USBD_UVC_SetTxBuffer(hUsbDevice_0, UserTxBufferFS, 0);
  USBD_UVC_SetRxBuffer(hUsbDevice_0, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */ 
}

/**
  * @brief  UVC_DeInit_FS
  *         DeInitializes the UVC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t UVC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */ 
  return (USBD_OK);
  /* USER CODE END 4 */ 
}

/**
  * @brief  UVC_Control_FS
  *         Manage the UVC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t UVC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t idx, uint16_t value)
{ 
  int i;
  /* USER CODE BEGIN 5 */
  printf("UVC_Control_FS(cmd=%x,pbuf=%p,length=%x,index=%x,value=%x)\r\n", cmd, pbuf, length, idx, value);

  switch (cmd)
  {
  case GET_DEF:
  case GET_CUR:
  case GET_MIN:
  case GET_MAX:

    printf("UVC_Control_FS(): ");
    switch (cmd) {
      case GET_DEF: printf("GET_DEF "); break;
      case GET_CUR: printf("GET_CUR "); break;
      case GET_MIN: printf("GET_MIN "); break;
      case GET_MAX: printf("GET_MAX "); break;
      default: printf("UNKNOWN "); break;
    }

    if(/*idx == 1 &&*/ value == 256)
    {
      USBD_UVC_VideoControlTypeDef *rtnBuf = (USBD_UVC_VideoControlTypeDef*)pbuf;
      printf("probe\r\n");
      printf("from host:\r\n");
      print_vc(rtnBuf);

      if (cmd == GET_DEF)
      {
        memcpy(rtnBuf,
          &(USBD_UVC_VideoControlTypeDef) {
            {0x00,0x00},                      // bmHint
            {0x01},                           // bFormatIndex
            {0x01},                           // bFrameIndex
            {DBVAL(INTERVAL),},          // dwFrameInterval
            {0x00,0x00,},                     // wKeyFrameRate
            {0x00,0x00,},                     // wPFrameRate
            {0x00,0x00,},                     // wCompQuality
            {0x00,0x00,},                     // wCompWindowSize
            {0x00,0x00,},                      // wDelay
            {DBVAL(MAX_FRAME_SIZE(80,60,8)),},    // dwMaxVideoFrameSize
            {DBVAL(VIDEO_PACKET_SIZE),},         // dwMaxPayloadTransferSize
            {0x00, 0x00, 0x00, 0x00},         // dwClockFrequency
            {0x00},                           // bmFramingInfo
            {0x00},                           // bPreferedVersion
            {0x00},                           // bMinVersion
            {0x00},                           // bMaxVersion
          }, length);
      }
      else if (cmd == GET_CUR)
      {
        memcpy(rtnBuf, &videoProbeControl, MIN(sizeof(USBD_UVC_VideoControlTypeDef), length));
      }
      else
      {
        memcpy(rtnBuf->dwMaxPayloadTransferSize, (uint8_t[]){ DBVAL(VIDEO_PACKET_SIZE) }, 4);
        memcpy(rtnBuf->dwFrameInterval, (uint8_t[]){ DBVAL(INTERVAL) }, 4);
        memcpy(rtnBuf->wKeyFrameRate, (uint8_t[]){ WBVAL(0) }, 2);
        memcpy(rtnBuf->wPFrameRate, (uint8_t[]){ WBVAL(0) }, 2);
        memcpy(rtnBuf->wCompQuality, (uint8_t[]){ WBVAL(0) }, 2);
        memcpy(rtnBuf->wCompWindowSize, (uint8_t[]){ WBVAL(0) }, 2);
        memcpy(rtnBuf->wDelay, (uint8_t[]){ WBVAL(0) }, 2);

        switch(rtnBuf->bFormatIndex[0]) {
        case FMT_INDEX_NV12:
          memcpy(rtnBuf->dwMaxVideoFrameSize, (uint8_t[]){ DBVAL(MAX_FRAME_SIZE(80,60,12)) }, 4);
          break;
        case FMT_INDEX_GREY:
          memcpy(rtnBuf->dwMaxVideoFrameSize, (uint8_t[]){ DBVAL(MAX_FRAME_SIZE(80,60,8)) }, 4);
          break;
        default:
          memcpy(rtnBuf->dwMaxVideoFrameSize, (uint8_t[]){ DBVAL(MAX_FRAME_SIZE(80,60,16)) }, 4);
          break;
        }
      }

      printf("returning:\r\n");
      print_vc(rtnBuf);
    }
    else if (/*idx == 1 &&*/ value == 512)
    {
      printf("commit\r\n");
      print_vc(&videoCommitControl);

  	  //Commit Request
      memcpy(pbuf, &videoCommitControl, MIN(sizeof(USBD_UVC_VideoControlTypeDef), length));
    }
    else
    {
      printf("FAIL\r\n");
      return USBD_FAIL;
    }
    break;

  case SET_CUR:
  {
    printf("UVC_Control_FS(): SET_CUR ");
    USBD_UVC_VideoControlTypeDef *rtnBuf = (USBD_UVC_VideoControlTypeDef*)pbuf;

    if (rtnBuf->bFormatIndex[0] > 3)
    {
      printf("\r\nBogus bFormatIndex value, ignoring\r\n");
      break;
    }

    memcpy(rtnBuf->dwMaxPayloadTransferSize, (uint8_t[]){ DBVAL(VIDEO_PACKET_SIZE) }, 4);
    memcpy(rtnBuf->dwFrameInterval, (uint8_t[]){ DBVAL(INTERVAL) }, 4);
    memcpy(rtnBuf->wKeyFrameRate, (uint8_t[]){ WBVAL(0) }, 2);
    memcpy(rtnBuf->wPFrameRate, (uint8_t[]){ WBVAL(0) }, 2);
    memcpy(rtnBuf->wCompQuality, (uint8_t[]){ WBVAL(0) }, 2);
    memcpy(rtnBuf->wCompWindowSize, (uint8_t[]){ WBVAL(0) }, 2);
    memcpy(rtnBuf->wDelay, (uint8_t[]){ WBVAL(0) }, 2);

    switch(rtnBuf->bFormatIndex[0]) {
    case FMT_INDEX_NV12:
      memcpy(rtnBuf->dwMaxVideoFrameSize, (uint8_t[]){ DBVAL(MAX_FRAME_SIZE(80,60,12)) }, 4);
      break;
    case FMT_INDEX_GREY:
      memcpy(rtnBuf->dwMaxVideoFrameSize, (uint8_t[]){ DBVAL(MAX_FRAME_SIZE(80,60,8)) }, 4);
      break;
    default:
      memcpy(rtnBuf->dwMaxVideoFrameSize, (uint8_t[]){ DBVAL(MAX_FRAME_SIZE(80,60,16)) }, 4);
      break;
    }

    if(/*idx == 1 &&*/ value == 256)
    {
      printf("probe\r\n");
      print_vc((USBD_UVC_VideoControlTypeDef*)pbuf);

      memcpy(&videoProbeControl, pbuf, MIN(sizeof(USBD_UVC_VideoControlTypeDef), length));
    }
    else if (/*idx == 1 &&*/ value == 512)
    {
      printf("commit\r\n");
      print_vc((USBD_UVC_VideoControlTypeDef*)pbuf);

      memcpy(&videoCommitControl, pbuf, MIN(sizeof(USBD_UVC_VideoControlTypeDef), length));
    }
    else
    {
      printf("FAIL\r\n");

      return USBD_FAIL;
    }
    break;
  }
  case GET_INFO:
    printf("UVC_Control_FS(): get_info, this is probably wrong...\r\n", cmd);
    pbuf[0] = 0x3;
    break;
  default:
    printf("FAIL: UVC_Control_FS() unknown %x\r\n", cmd);
    return USBD_FAIL;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  UVC_Receive_FS
  *         Data received over USB OUT endpoint are sent over UVC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on UVC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t UVC_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  return (USBD_OK);
  /* USER CODE END 6 */ 
}

/**
  * @brief  UVC_Transmit_FS
  *         Data send over USB IN endpoint are sent over UVC interface 
  *         through this function.           
  *         @note
  *         
  *                 
  * @param  Buf: Buffer of data to be send
  * @param  Len: Number of data to be send (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t UVC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_UVC_SetTxBuffer(hUsbDevice_0, Buf, Len);   
  result = USBD_UVC_TransmitPacket(hUsbDevice_0);
  /* USER CODE END 7 */ 
  return result;
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

