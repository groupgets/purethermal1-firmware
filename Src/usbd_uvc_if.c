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
#include "usbd_uvc_lepton_xu.h"

#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

// #define UVC_VC_DEBUG
// #define UVC_VS_DEBUG

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
__ALIGN_BEGIN struct uvc_streaming_control videoCommitControl __ALIGN_END =
{
  .bmHint = 0x00,
  .bFormatIndex = VS_FMT_INDEX(YUYV),
  .bFrameIndex = 0x01,
  .dwFrameInterval = 0, 
  .wKeyFrameRate = 0,
  .wPFrameRate = 0,
  .wCompQuality = 0,
  .wCompWindowSize = 0,
  .wDelay = 0,
  .dwMaxVideoFrameSize = MAX_FRAME_SIZE(80,60,VS_FMT_SIZE(YUYV)),
  .dwMaxPayloadTransferSize = VIDEO_PACKET_SIZE,
  .dwClockFrequency = 0,
  .bmFramingInfo = 0,
  .bPreferedVersion = 0,
  .bMinVersion = 0,
  .bMaxVersion = 0,
};

__ALIGN_BEGIN struct uvc_streaming_control videoProbeControl __ALIGN_END =
{
  .bmHint = 0x00,
  .bFormatIndex = VS_FMT_INDEX(YUYV),
  .bFrameIndex = 0x01,
  .dwFrameInterval = 0, 
  .wKeyFrameRate = 0,
  .wPFrameRate = 0,
  .wCompQuality = 0,
  .wCompWindowSize = 0,
  .wDelay = 0,
  .dwMaxVideoFrameSize = MAX_FRAME_SIZE(80,60,VS_FMT_SIZE(YUYV)),
  .dwMaxPayloadTransferSize = VIDEO_PACKET_SIZE,
  .dwClockFrequency = 0,
  .bmFramingInfo = 0,
  .bPreferedVersion = 0,
  .bMinVersion = 0,
  .bMaxVersion = 0,
};

#ifdef UVC_VS_DEBUG
void print_vc(struct uvc_streaming_control* vc)
{
  DEBUG_PRINTF("bmHint 0x%x\r\n", vc->bmHint);                      // 2
  DEBUG_PRINTF("bFormatIndex %d\r\n", vc->bFormatIndex);                // 3
  DEBUG_PRINTF("bFrameIndex %d\r\n", vc->bFrameIndex);                 // 4
  DEBUG_PRINTF("dwFrameInterval %lu\r\n", vc->dwFrameInterval);             // 8
  DEBUG_PRINTF("wKeyFrameRate %d\r\n", vc->wKeyFrameRate);               // 10
  DEBUG_PRINTF("wPFrameRate %d\r\n", vc->wPFrameRate);                 // 12
  DEBUG_PRINTF("wCompQuality %d\r\n", vc->wCompQuality);                // 14
  DEBUG_PRINTF("wCompWindowSize %d\r\n", vc->wCompWindowSize);             // 16
  DEBUG_PRINTF("wDelay %d\r\n", vc->wDelay);                      // 18
  DEBUG_PRINTF("dwMaxVideoFrameSize %lu\r\n", vc->dwMaxVideoFrameSize);// 22
  DEBUG_PRINTF("dwMaxPayloadTransferSize %lu\r\n", vc->dwMaxPayloadTransferSize);    // 26
#ifdef UVC_1_1
  DEBUG_PRINTF("dwClockFrequency %d\r\n", vc->dwClockFrequency);
  DEBUG_PRINTF("bmFramingInfo 0x%x\r\n", vc->bmFramingInfo);
  DEBUG_PRINTF("bPreferedVersion %d\r\n", vc->bPreferedVersion);
  DEBUG_PRINTF("bMinVersion %d\r\n", vc->bMinVersion);
  DEBUG_PRINTF("bMaxVersion %d\r\n", vc->bMaxVersion);
#endif
}
#endif

/**
  * @}
  */ 
  
/** @defgroup USBD_UVC_Private_FunctionPrototypes
  * @{
  */
static int8_t UVC_Init_FS     (void);
static int8_t UVC_DeInit_FS   (void);
static int8_t UVC_Control        (uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t index, uint16_t value);
static int8_t UVC_VC_ControlGet  (VC_TERMINAL_ID entity_id, uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t index, uint16_t value);
static int8_t UVC_VC_ControlSet  (VC_TERMINAL_ID entity_id, uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t index, uint16_t value);
static int8_t UVC_VS_ControlGet  (uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t index, uint16_t value);
static int8_t UVC_VS_ControlSet  (uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t index, uint16_t value);
static int8_t UVC_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

USBD_UVC_ItfTypeDef USBD_Interface_fops_FS = 
{
  .Init = UVC_Init_FS,
  .DeInit = UVC_DeInit_FS,
  .Control = UVC_Control,
  .VS_CtrlGet = UVC_VS_ControlGet,
  .VS_CtrlSet = UVC_VS_ControlSet,
  .ControlGet = UVC_VC_ControlGet,
  .ControlSet = UVC_VC_ControlSet,
  .Receive = UVC_Receive_FS,
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

static int8_t UVC_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t idx, uint16_t value)
{
  DEBUG_PRINTF("UVC_Control(cmd=%x,pbuf=%p,length=%x,index=%x,value=%x)\r\n", cmd, pbuf, length, idx, value);
  return USBD_OK;
}

/**
  * @brief  UVC_Control_FS
  *         Manage the UVC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t UVC_VC_ControlGet  (VC_TERMINAL_ID entity_id, uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t idx, uint16_t value)
{ 
  /* USER CODE BEGIN 5 */
  uint8_t cs_value = (value >> 8) & 0xFF;

#ifdef UVC_VC_DEBUG
  DEBUG_PRINTF("UVC_VC_ControlGet(entity_id=%d,cmd=%x,pbuf=%p,length=%x, index=%x,value=%x)\r\n", entity_id, cmd, pbuf, length, idx, value);

  DEBUG_PRINTF("UVC_VC_ControlGet(cs=%d): ", cs_value);
  switch (cmd) {
    case UVC_GET_DEF: DEBUG_PRINTF("UVC_GET_DEF "); break;
    case UVC_GET_CUR: DEBUG_PRINTF("UVC_GET_CUR "); break;
    case UVC_GET_MIN: DEBUG_PRINTF("UVC_GET_MIN "); break;
    case UVC_GET_MAX: DEBUG_PRINTF("UVC_GET_MAX "); break;
    case UVC_GET_RES: DEBUG_PRINTF("UVC_GET_RES "); break;
    case UVC_GET_LEN: DEBUG_PRINTF("UVC_GET_LEN "); break;
    case UVC_GET_INFO: DEBUG_PRINTF("UVC_GET_INFO "); break;
    default: DEBUG_PRINTF("UNKNOWN "); break;
  }
#endif

  switch (entity_id)
  {
  case VC_CONTROL_XU_LEP_AGC_ID:
  case VC_CONTROL_XU_LEP_OEM_ID:
  case VC_CONTROL_XU_LEP_RAD_ID:
  case VC_CONTROL_XU_LEP_SYS_ID:
  case VC_CONTROL_XU_LEP_VID_ID:
#ifdef UVC_VC_DEBUG
    DEBUG_PRINTF("UVC_VC_CONTROL_XU(%d)\r\n", entity_id);
#endif

    memset(pbuf, 0, length);
    switch (cmd)
    {
    case UVC_GET_DEF:
    case UVC_GET_MIN:
      break;
    case UVC_GET_CUR:
      if (length > 1)
        VC_LEP_GetAttribute(entity_id, (cs_value - 1) << 2, pbuf, length);
      break;
    case UVC_GET_MAX:
      VC_LEP_GetMaxValue(entity_id, (cs_value - 1) << 2, pbuf, length);
      break;
    case UVC_GET_RES:
      pbuf[0] = 1;
      break;
    case UVC_GET_LEN:
      VC_LEP_GetAttributeLen(entity_id, (cs_value - 1) << 2, (uint16_t*)pbuf);
      break;
    case UVC_GET_INFO:
      pbuf[0] = UVC_CONTROL_CAP_GET | UVC_CONTROL_CAP_SET;
      break;
    default:
      DEBUG_PRINTF("FAIL: UVC_VC_ControlGet() unknown %x\r\n", cmd);
      return USBD_FAIL;
    }

    break;

  case VC_CONTROL_PU_ID:
#ifdef UVC_VC_DEBUG
    DEBUG_PRINTF("UVC_VC_CONTROL_PU_ID\r\n");
#endif

    switch (cmd)
    {
    case UVC_GET_MIN:
      pbuf[0] = 0;
      break;
    case UVC_GET_DEF:
    case UVC_GET_CUR:
      pbuf[0] = 128;
      break;
    case UVC_GET_MAX:
      pbuf[0] = 255;
      break;
    case UVC_GET_RES:
      pbuf[0] = 1;
      break;
    case UVC_GET_LEN:
      pbuf[0] = 2;
      break;
    case UVC_GET_INFO:
      pbuf[0] = UVC_CONTROL_CAP_GET | UVC_CONTROL_CAP_DISABLED;
      break;
    default:
      DEBUG_PRINTF("FAIL: UVC_VC_ControlGet() unknown %x\r\n", cmd);
      return USBD_FAIL;
    }

    break;
  default:
    return USBD_FAIL;
  }

#ifdef UVC_VC_DEBUG
  DEBUG_PRINTF("\r\n");
#endif

  return (USBD_OK);
  /* USER CODE END 5 */
}

static int8_t UVC_VC_ControlSet  (VC_TERMINAL_ID entity_id, uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t idx, uint16_t value)
{ 
  /* USER CODE BEGIN 5 */
  uint8_t cs_value = (value >> 8) & 0xFF;

#ifdef UVC_VC_DEBUG
  DEBUG_PRINTF("UVC_VC_ControlSet(entity_id=%d, cmd=%x,pbuf=%p,length=%x,index=%x,value=%x)\r\n", entity_id, cmd, pbuf, length, idx, value);
#endif

  switch (cmd)
  {
  case UVC_SET_CUR:
#ifdef UVC_VC_DEBUG
    DEBUG_PRINTF("UVC_VC_ControlSet(cs=%d): ", cs_value);
#endif
    break;
  default:
    DEBUG_PRINTF("FAIL: UVC_VC_ControlSet() unknown %x\r\n", cmd);
    return USBD_FAIL;
  }

#ifdef UVC_VC_DEBUG
  DEBUG_PRINTF("UVC_VC_CONTROL_XU(%d)\r\n", entity_id);
#endif

  switch (entity_id)
  {
  case VC_CONTROL_XU_LEP_AGC_ID:
  case VC_CONTROL_XU_LEP_OEM_ID:
  case VC_CONTROL_XU_LEP_RAD_ID:
  case VC_CONTROL_XU_LEP_SYS_ID:
  case VC_CONTROL_XU_LEP_VID_ID:
    if (length == 1)
      VC_LEP_RunCommand(entity_id, (cs_value - 1) << 2);
    else
      VC_LEP_SetAttribute(entity_id, (cs_value - 1) << 2, pbuf, length);
    break;
  case VC_CONTROL_PU_ID:
    break;
  default:
    return USBD_FAIL;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}


/**
  * @brief  UVC_Control_FS
  *         Manage the UVC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t UVC_VS_ControlGet  (uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t idx, uint16_t value)
{ 
  /* USER CODE BEGIN 5 */
  uint8_t cs_value = (value >> 8) & 0xFF;

#ifdef UVC_VS_DEBUG
  DEBUG_PRINTF("UVC_VS_ControlGet(cmd=%x,pbuf=%p,length=%x,index=%x,value=%x)\r\n", cmd, pbuf, length, idx, value);
#endif

  switch (cmd)
  {
  case UVC_GET_DEF:
  case UVC_GET_CUR:
  case UVC_GET_MIN:
  case UVC_GET_MAX:

#ifdef UVC_VS_DEBUG
    DEBUG_PRINTF("UVC_VS_ControlGet(): ");
    switch (cmd) {
      case UVC_GET_DEF: DEBUG_PRINTF("UVC_GET_DEF "); break;
      case UVC_GET_CUR: DEBUG_PRINTF("UVC_GET_CUR "); break;
      case UVC_GET_MIN: DEBUG_PRINTF("UVC_GET_MIN "); break;
      case UVC_GET_MAX: DEBUG_PRINTF("UVC_GET_MAX "); break;
      default: DEBUG_PRINTF("UNKNOWN "); break;
    }
#endif

    if(cs_value == UVC_VS_PROBE_CONTROL || cs_value == UVC_VS_STILL_PROBE_CONTROL)
    {
      struct uvc_streaming_control *rtnBuf = (struct uvc_streaming_control*)pbuf;

#ifdef UVC_VS_DEBUG
      DEBUG_PRINTF("probe\r\n");
#endif

      memset(rtnBuf, 0, length);

      rtnBuf->bFormatIndex = videoProbeControl.bFormatIndex;
      rtnBuf->bFrameIndex = videoProbeControl.bFrameIndex;
      rtnBuf->dwMaxPayloadTransferSize = VIDEO_PACKET_SIZE;
      rtnBuf->dwFrameInterval = INTERVAL;

      if (cmd == UVC_GET_DEF ||
          cmd == UVC_GET_MIN ||
          cmd == UVC_GET_MAX ||
          cmd == UVC_GET_CUR)
      {
        switch(videoProbeControl.bFormatIndex) {
        case VS_FMT_INDEX(NV12):
        case VS_FMT_INDEX(YU12):
          rtnBuf->dwMaxVideoFrameSize = MAX_FRAME_SIZE(80,60,VS_FMT_SIZE(NV12));
          break;
        case VS_FMT_INDEX(GREY):
          rtnBuf->dwMaxVideoFrameSize = MAX_FRAME_SIZE(80,60,VS_FMT_SIZE(GREY));
          break;
        case VS_FMT_INDEX(Y16):
          rtnBuf->dwMaxVideoFrameSize = MAX_FRAME_SIZE(80,60,VS_FMT_SIZE(Y16));
          break;
        case VS_FMT_INDEX(BGR3):
          rtnBuf->dwMaxVideoFrameSize = MAX_FRAME_SIZE(80,60,VS_FMT_SIZE(BGR3));
          break;
        case VS_FMT_INDEX(RGB565):
          rtnBuf->dwMaxVideoFrameSize = MAX_FRAME_SIZE(80,60,VS_FMT_SIZE(RGB565));
          break;
        case VS_FMT_INDEX(YUYV):
        default:
          rtnBuf->dwMaxVideoFrameSize = MAX_FRAME_SIZE(80,60,VS_FMT_SIZE(YUYV));
          break;
        }
      }
      else
      {
        return USBD_FAIL;
      }
    }
    else if (cs_value == UVC_VS_COMMIT_CONTROL || cs_value == UVC_VS_STILL_COMMIT_CONTROL)
    {
#ifdef UVC_VS_DEBUG
      DEBUG_PRINTF("commit\r\n");
#endif

      // Commit Request
      memcpy(pbuf, &videoCommitControl, MIN(sizeof(struct uvc_streaming_control), length));
    }
    else
    {
      DEBUG_PRINTF("FAIL? (cs_value = %d)\r\n", cs_value);
      return USBD_FAIL;
    }

#ifdef UVC_VS_DEBUG
    DEBUG_PRINTF("returning:\r\n");
    print_vc((struct uvc_streaming_control*)pbuf);
#endif
    break;

  default:
    DEBUG_PRINTF("FAIL: UVC_VS_ControlGet() unknown %x\r\n", cmd);
    return USBD_FAIL;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

static int8_t UVC_VS_ControlSet  (uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t idx, uint16_t value)
{ 
  /* USER CODE BEGIN 5 */
  uint8_t cs_value = (value >> 8) & 0xFF;

#ifdef UVC_VS_DEBUG
  DEBUG_PRINTF("UVC_VS_ControlSet(cmd=%x,pbuf=%p,length=%x,index=%x,value=%x)\r\n", cmd, pbuf, length, idx, value);
#endif

  switch (cmd)
  {
  case UVC_SET_CUR:
  {
#ifdef UVC_VS_DEBUG
    DEBUG_PRINTF("UVC_VS_ControlSet(): UVC_SET_CUR ");
#endif
    struct uvc_streaming_control *rtnBuf = (struct uvc_streaming_control*)pbuf;

    if (rtnBuf->bFormatIndex > VS_NUM_FORMATS)
    {
      DEBUG_PRINTF("\r\nBogus bFormatIndex value, ignoring\r\n");
      return USBD_FAIL;
    }

    if(cs_value == UVC_VS_PROBE_CONTROL || cs_value == UVC_VS_STILL_PROBE_CONTROL)
    {
#ifdef UVC_VS_DEBUG
      DEBUG_PRINTF("probe\r\n");
#endif
      memcpy(&videoProbeControl, pbuf, MIN(sizeof(struct uvc_streaming_control), length));
    }
    else if (cs_value == UVC_VS_COMMIT_CONTROL || cs_value == UVC_VS_STILL_COMMIT_CONTROL)
    {
#ifdef UVC_VS_DEBUG
      DEBUG_PRINTF("commit\r\n");
#endif
      memcpy(&videoCommitControl, pbuf, MIN(sizeof(struct uvc_streaming_control), length));
    }
    else
    {
      DEBUG_PRINTF("FAIL? value = %d\r\n", value);
      return USBD_FAIL;
    }

#ifdef UVC_VS_DEBUG
    DEBUG_PRINTF("received:\r\n");
    print_vc((struct uvc_streaming_control*)pbuf);
#endif
    break;
  }

  default:
    DEBUG_PRINTF("FAIL: UVC_VS_ControlSet() unknown %x\r\n", cmd);
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
  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  USBD_UVC_SetTxBuffer(hUsbDevice_0, Buf, Len);   
  result = USBD_UVC_TransmitPacket(hUsbDevice_0);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
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

