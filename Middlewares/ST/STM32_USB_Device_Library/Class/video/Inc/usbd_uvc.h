/**
  ******************************************************************************
  * @file    usbd_cdc.h
  * @author  MCD Application Team
  * @version V2.4.1
  * @date    19-June-2015
  * @brief   header file for the usbd_cdc.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_UVC_H
#define __USB_UVC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_ioreq.h"
#include "uvc.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup usbd_cdc
  * @brief This file is the Header file for usbd_cdc.c
  * @{
  */ 


/** @defgroup usbd_cdc_Exported_Defines
  * @{
  */ 
#define UVC_IN_EP                                     0x81  /* EP1 for data IN */
#define UVC_OUT_EP                                    0x01  /* EP1 for data OUT */
// #define UVC_CMD_EP                                    0x82  /* EP2 for UVC commands */

/* UVC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */

#define WIDTH                                         ((unsigned int)80)
#define HEIGHT                                        ((unsigned int)60)
#define VIDEO_PACKET_SIZE                             ((unsigned int)(482))
#define BITS_PER_PIXEL                                ((unsigned int)8)

#define GUID_VS_FORMAT \
    'Y',  '8',  '0',  '0', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71

/*
#define GUID_VS_FORMAT \
    'Y',  'U',  'Y',  '2', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
*/

#define BYTES_PER_PIXEL                               (BITS_PER_PIXEL>>3)
#define MAX_FRAME_SIZE                                ((unsigned long)((WIDTH*HEIGHT*BITS_PER_PIXEL)>>3))
// #define CAM_FPS                                       ((VIDEO_PACKET_SIZE*1000)/MAX_FRAME_SIZE)
#define CAM_FPS                                       26
#define MIN_BIT_RATE                                  ((unsigned long)(WIDTH*HEIGHT*BITS_PER_PIXEL*CAM_FPS))
#define MAX_BIT_RATE                                  ((unsigned long)(WIDTH*HEIGHT*BITS_PER_PIXEL*CAM_FPS))
#define INTERVAL                                      ((unsigned long)(10000000/CAM_FPS))

#define USB_UVC_VCIF_NUM                              0
#define USB_UVC_VSIF_NUM                              (char)1

#define VIDEO_TOTAL_IF_NUM                            2

/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */


typedef struct _USBD_UVC_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t, uint16_t, uint16_t);   
  int8_t (* Receive)       (uint8_t *, uint32_t *);  

}USBD_UVC_ItfTypeDef;


typedef struct
{
  uint32_t data[VIDEO_PACKET_SIZE/4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint16_t CmdLength;    
  uint16_t CmdIndex;
  uint16_t CmdValue;
  uint8_t  *RxBuffer;  
  uint8_t  *TxBuffer;   
  uint32_t RxLength;
  uint32_t TxLength;    
  
  __IO uint32_t TxState;     
  __IO uint32_t RxState;    
}
USBD_UVC_HandleTypeDef; 

//UVC 1.0 uses only 26 first bytes
typedef struct  _USBD_UVC_VideoControlTypeDef{
  uint8_t    bmHint[2];                      // 2
  uint8_t    bFormatIndex[1];                // 3
  uint8_t    bFrameIndex[1];                 // 4
  uint8_t    dwFrameInterval[4];             // 8
  uint8_t    wKeyFrameRate[2];               // 10
  uint8_t    wPFrameRate[2];                 // 12
  uint8_t    wCompQuality[2];                // 14
  uint8_t    wCompWindowSize[2];             // 16
  uint8_t    wDelay[2];                      // 18
  uint8_t    dwMaxVideoFrameSize[4];         // 22
  uint8_t    dwMaxPayloadTransferSize[4];    // 26
  uint8_t    dwClockFrequency[4];
  uint8_t    bmFramingInfo[1];
  uint8_t    bPreferedVersion[1];
  uint8_t    bMinVersion[1];
  uint8_t    bMaxVersion[1];
}USBD_UVC_VideoControlTypeDef;


/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 

#define WBVAL(x) (x & 0xFF),((x >> 8) & 0xFF)
#define DBVAL(x) (x & 0xFF),((x >> 8) & 0xFF),((x >> 16) & 0xFF),((x >> 24) & 0xFF)

#define UVC_DATA_HS_IN_PACKET_SIZE                    VIDEO_PACKET_SIZE
#define UVC_DATA_HS_OUT_PACKET_SIZE                   VIDEO_PACKET_SIZE

#define UVC_DATA_FS_IN_PACKET_SIZE                    VIDEO_PACKET_SIZE
#define UVC_DATA_FS_OUT_PACKET_SIZE                   VIDEO_PACKET_SIZE

#define UVC_LEN_IF_ASSOCIATION_DESC                   (char)8

#define UVC_VC_INTERFACE_HEADER_DESC_SIZE(n)          (char)(12+n)
#define UVC_CAMERA_TERMINAL_DESC_SIZE(n)              (char)(15+n)
#define UVC_OUTPUT_TERMINAL_DESC_SIZE(n)              (char)(9+n)
#define UVC_VS_INTERFACE_INPUT_HEADER_DESC_SIZE(a,b)  (char) (13+a*b)

#define VS_FORMAT_UNCOMPRESSED_DESC_SIZE              (char)(27)
#define VS_FRAME_UNCOMPRESSED_DESC_SIZE               (char)(30)
#define VS_COLOR_MATCHING_DESC_SIZE                   (char)(6)

#define USB_INTERFACE_POWER_DESCRIPTOR_TYPE           8
#define USB_OTG_DESCRIPTOR_TYPE                       9
#define USB_DEBUG_DESCRIPTOR_TYPE                     10
#define USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE     11

/* bmAttributes in Configuration Descriptor */
#define USB_CONFIG_POWERED_MASK                       0xC0
#define USB_CONFIG_BUS_POWERED                        0x80

#define USB_VIDEO_DESC_SIZ (unsigned long)(\
  			  USB_LEN_CFG_DESC +\
  		    UVC_LEN_IF_ASSOCIATION_DESC +\
  		    USB_LEN_IF_DESC +  \
  		    UVC_VC_INTERFACE_HEADER_DESC_SIZE(1) + \
  		    UVC_CAMERA_TERMINAL_DESC_SIZE(2) + \
  		    UVC_OUTPUT_TERMINAL_DESC_SIZE(0) + \
  		    USB_LEN_IF_DESC +   \
  		    UVC_VS_INTERFACE_INPUT_HEADER_DESC_SIZE(1,1) + \
  		    VS_FORMAT_UNCOMPRESSED_DESC_SIZE +  \
  		    VS_FRAME_UNCOMPRESSED_DESC_SIZE  +  \
  		    VS_COLOR_MATCHING_DESC_SIZE  +\
  		    USB_LEN_IF_DESC +  \
  		    USB_LEN_EP_DESC)

#define VC_TERMINAL_SIZ (unsigned int)(UVC_VC_INTERFACE_HEADER_DESC_SIZE(1) + UVC_CAMERA_TERMINAL_DESC_SIZE(2) + UVC_OUTPUT_TERMINAL_DESC_SIZE(0))
#define VC_HEADER_SIZ (unsigned int)(UVC_VS_INTERFACE_INPUT_HEADER_DESC_SIZE(1,1) + VS_FORMAT_UNCOMPRESSED_DESC_SIZE + VS_FRAME_UNCOMPRESSED_DESC_SIZE + VS_COLOR_MATCHING_DESC_SIZE)

/* bMaxPower in Configuration Descriptor */
#define USB_CONFIG_POWER_MA(mA)                       ((mA)/2)

/* bEndpointAddress in Endpoint Descriptor */
#define USB_ENDPOINT_DIRECTION_MASK                   0x80
#define USB_ENDPOINT_OUT(addr)                        ((addr) | 0x00)
#define USB_ENDPOINT_IN(addr)                         ((addr) | 0x80)

/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_ClassTypeDef  USBD_UVC;
#define USBD_UVC_CLASS    &USBD_UVC
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_UVC_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                      USBD_UVC_ItfTypeDef *fops);

uint8_t  USBD_UVC_SetTxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff,
                                      uint16_t length);

uint8_t  USBD_UVC_SetRxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff);
  
uint8_t  USBD_UVC_ReceivePacket      (USBD_HandleTypeDef *pdev);

uint8_t  USBD_UVC_TransmitPacket     (USBD_HandleTypeDef *pdev);
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_UVC_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
