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
#define VIDEO_PACKET_SIZE                             ((unsigned int)(482))

#define CAM_FPS                                       9

enum _vs_fmt_indexes {
  VS_FMT_INDEX_NONE = 0,
  VS_FMT_INDEX_YUYV,
  VS_FMT_INDEX_Y16,
  VS_FMT_INDEX_NV12,
  VS_FMT_INDEX_YU12,
  VS_FMT_INDEX_GREY,
  VS_FMT_INDEX_MAX,
};

enum _vs_fmt_size {
  VS_FMT_SIZE_NONE =  0,
  VS_FMT_SIZE_YUYV = 16,
  VS_FMT_SIZE_Y16  = 16,
  VS_FMT_SIZE_NV12 = 12,
  VS_FMT_SIZE_YU12 = 12,
  VS_FMT_SIZE_GREY =  8,
};

#define VS_FMT_GUID_NONE \
    '0',  '0',  '0',  '0', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

#define VS_FMT_GUID_GREY \
    'Y',  '8',  '0',  '0', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71

#define VS_FMT_GUID_Y16 \
    'Y',  '1',  '6',  ' ', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71

#define VS_FMT_GUID_YUYV \
    'Y',  'U',  'Y',  '2', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71

#define VS_FMT_GUID_NV12 \
    'N',  'V',  '1',  '2', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71

#define VS_FMT_GUID_YU12 \
    'I',  '4',  '2',  '0', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71

#define VS_FMT_INDEX(NAME) VS_FMT_INDEX_ ## NAME
#define VS_FMT_GUID(NAME) { VS_FMT_GUID_ ## NAME }
#define VS_FMT_SIZE(NAME) VS_FMT_SIZE_ ## NAME

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

struct uvc_format_uncompressed {
  uint8_t bLength; /* 27*/
  uint8_t bDescriptorType; /* : CS_INTERFACE */
  uint8_t bDescriptorSubType; /* : VS_FORMAT_UNCOMPRESSED subtype */
  uint8_t bFormatIndex; /* : First format descriptor */
  uint8_t bNumFrameDescriptors; /* : One frame descriptor for this format follows. */
  uint8_t sGuidFormat[16];
  uint8_t bBitsPerPixel; /* : Number of bits per pixel used to specify color in the decoded video frame - 16 for yuy2, 12 for nv12... */
  uint8_t bDefaultFrameIndex; /* : Default frame index is 1. */
  uint8_t bAspectRatioX; /* : Non-interlaced stream not required. */
  uint8_t bAspectRatioY; /* : Non-interlaced stream not required. */
  uint8_t bmInterlaceFlags; /* : Non-interlaced stream */
  uint8_t bCopyProtect; /* : No restrictions imposed on the duplication of this video stream. */

};

struct uvc_frame_uncompressed {
  uint8_t bLength; /* 30*/
  uint8_t bDescriptorType; /* : CS_INTERFACE */
  uint8_t bDescriptorSubType; /* : VS_FRAME_UNCOMPRESSED */
  uint8_t bFrameIndex; /* : First (and only) frame descriptor */
  uint8_t bmCapabilities; /* : Still images using capture method 0 are supported at this frame setting.D1: Fixed frame-rate. */
  uint8_t wWidth[2]; /* (2bytes): Width of frame is 128 pixels. */
  uint8_t wHeight[2]; /* (2bytes): Height of frame is 64 pixels. */
  uint8_t dwMinBitRate[4]; /* (4bytes): Min bit rate in bits/s  */
  uint8_t dwMaxBitRate[4]; /* (4bytes): Max bit rate in bits/s  */
  uint8_t dwMaxVideoFrameBufSize[4]; /* (4bytes): Maximum video or still frame size, in bytes. */
  uint8_t dwDefaultFrameInterval[4]; /* : 1,000,000 * 100ns -> 10 FPS */
  uint8_t bFrameIntervalType; /* : Continuous frame interval */
  uint8_t dwMinFrameInterval[4]; /* : 1,000,000 ns  *100ns -> 10 FPS */

};

struct uvc_cs_color_matching {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubType;
  uint8_t bColorPrimary;
  uint8_t bTransferCharacteristics;
  uint8_t bMatrixCoefficients;
};

/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 

#define WBVAL(x) (x & 0xFF),((x >> 8) & 0xFF)
#define DBVAL(x) (x & 0xFF),((x >> 8) & 0xFF),((x >> 16) & 0xFF),((x >> 24) & 0xFF)

#define VALWB(x) ((x[0])|(x[1] << 8))
#define VALDB(x) ((x[0])|(x[1] << 8)|(x[2] << 16)|(x[3] << 24))

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
  		    VC_TERMINAL_SIZ + \
  		    USB_LEN_IF_DESC +   \
  		    VC_HEADER_SIZ + \
  		    USB_LEN_IF_DESC +  \
  		    USB_LEN_EP_DESC)

#define VC_TERMINAL_SIZ (unsigned int)(\
          UVC_VC_INTERFACE_HEADER_DESC_SIZE(1) + \
          UVC_CAMERA_TERMINAL_DESC_SIZE(2) + \
          0x0C + 0x1C + \
          UVC_OUTPUT_TERMINAL_DESC_SIZE(0))

#define VC_HEADER_SIZ (unsigned int)(\
          UVC_VS_INTERFACE_INPUT_HEADER_DESC_SIZE((VS_FMT_INDEX_MAX-1),1) + \
          VS_FORMAT_UNCOMPRESSED_DESC_SIZE + \
          VS_FRAME_UNCOMPRESSED_DESC_SIZE + \
          VS_COLOR_MATCHING_DESC_SIZE + \
          VS_FORMAT_UNCOMPRESSED_DESC_SIZE + \
          VS_FRAME_UNCOMPRESSED_DESC_SIZE + \
          VS_COLOR_MATCHING_DESC_SIZE + \
          VS_FORMAT_UNCOMPRESSED_DESC_SIZE + \
          VS_FRAME_UNCOMPRESSED_DESC_SIZE + \
          VS_COLOR_MATCHING_DESC_SIZE + \
          VS_FORMAT_UNCOMPRESSED_DESC_SIZE + \
          VS_FRAME_UNCOMPRESSED_DESC_SIZE + \
          VS_COLOR_MATCHING_DESC_SIZE + \
          VS_FORMAT_UNCOMPRESSED_DESC_SIZE + \
          VS_FRAME_UNCOMPRESSED_DESC_SIZE + \
          VS_COLOR_MATCHING_DESC_SIZE)

/* bMaxPower in Configuration Descriptor */
#define USB_CONFIG_POWER_MA(mA)                       ((mA)/2)
#define MAX_FRAME_SIZE(width, height, bits_per_pixel) ((unsigned long)((width*height*bits_per_pixel)>>3))
#define MIN_BIT_RATE(width, height, bits_per_pixel)   ((unsigned long)(width*height*bits_per_pixel*CAM_FPS))
#define MAX_BIT_RATE(width, height, bits_per_pixel)   ((unsigned long)(width*height*bits_per_pixel*CAM_FPS))
#define INTERVAL                                      ((unsigned long)(10000000/CAM_FPS))

/* bEndpointAddress in Endpoint Descriptor */
#define USB_ENDPOINT_DIRECTION_MASK                   0x80
#define USB_ENDPOINT_OUT(addr)                        ((addr) | 0x00)
#define USB_ENDPOINT_IN(addr)                         ((addr) | 0x80)

#define UVC_FORMAT_UNCOMPRESSED_DESCRIPTOR(FMT_NAME, NUM_FRAME_DESCS) { \
  VS_FORMAT_UNCOMPRESSED_DESC_SIZE,     /* bLength 27*/ \
  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */ \
  VS_FORMAT_UNCOMPRESSED,               /* bDescriptorSubType : VS_FORMAT_UNCOMPRESSED subtype */ \
  VS_FMT_INDEX(FMT_NAME),               /* bFormatIndex : First format descriptor */ \
  NUM_FRAME_DESCS,                      /* bNumFrameDescriptors : One frame descriptor for this format follows. */ \
  VS_FMT_GUID(FMT_NAME),                /* */ \
  VS_FMT_SIZE(FMT_NAME),                /* bBitsPerPixel : Number of bits per pixel used to specify color in the decoded video frame - 16 for yuy2, 12 for nv12... */ \
  0x01,                                 /* bDefaultFrameIndex : Default frame index is 1. */ \
  0x00,                                 /* bAspectRatioX : Non-interlaced stream not required. */ \
  0x00,                                 /* bAspectRatioY : Non-interlaced stream not required. */ \
  0x00,                                 /* bmInterlaceFlags : Non-interlaced stream */ \
  0x00,                                 /* bCopyProtect : No restrictions imposed on the duplication of this video stream. */ \
}

#define UVC_FRAME_FORMAT(FRAME_INDEX, FMT_NAME, WIDTH, HEIGHT) { \
  VS_FRAME_UNCOMPRESSED_DESC_SIZE,      /* bLength 30*/ \
  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */ \
  VS_FRAME_UNCOMPRESSED,                /* bDescriptorSubType : VS_FRAME_UNCOMPRESSED */ \
  FRAME_INDEX,                          /* bFrameIndex : First (and only) frame descriptor */ \
  0x02,                                 /* bmCapabilities : Still images using capture method 0 are supported at this frame setting.D1: Fixed frame-rate. */ \
  {WBVAL(WIDTH)},                       /* wWidth (2bytes): Width of frame is 128 pixels. */ \
  {WBVAL(HEIGHT)},                      /* wHeight (2bytes): Height of frame is 64 pixels. */ \
  {DBVAL(MIN_BIT_RATE(WIDTH,HEIGHT,VS_FMT_SIZE(FMT_NAME)))}, /* dwMinBitRate (4bytes): Min bit rate in bits/s  */ \
  {DBVAL(MAX_BIT_RATE(WIDTH,HEIGHT,VS_FMT_SIZE(FMT_NAME)))}, /* dwMaxBitRate (4bytes): Max bit rate in bits/s  */ \
  {DBVAL(MAX_FRAME_SIZE(WIDTH,HEIGHT,VS_FMT_SIZE(FMT_NAME)))}, /* dwMaxVideoFrameBufSize (4bytes): Maximum video or still frame size, in bytes. */ \
  {DBVAL(INTERVAL)},                      /* dwDefaultFrameInterval : */ \
  0x01,                                 /* bFrameIntervalType : Continuous frame interval */ \
  {DBVAL(INTERVAL)},                    /* dwMinFrameInterval : 1,000,000 ns  *100ns -> 10 FPS */ \
}

#define UVC_COLOR_MATCHING_DESCRIPTOR() { \
  VS_COLOR_MATCHING_DESC_SIZE,          /* bLength */ \
  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */ \
  0x0D,                                 /* bDescriptorSubType : VS_COLORFORMAT */ \
  0x01,                                 /* bColorPrimarie : 1: BT.709, sRGB (default) */ \
  0x01,                                 /* bTransferCharacteristics : 1: BT.709 (default) */ \
  0x04,                                 /* bMatrixCoefficients : 1: BT. 709. */ \
}

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
