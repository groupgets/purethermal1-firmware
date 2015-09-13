/**
  ******************************************************************************
  * @file    usbd_cdc.c
  * @author  MCD Application Team
  * @version V2.4.1
  * @date    19-June-2015
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB UVC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as UVC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                UVC Class Driver Description
  *          =================================================================== 
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as UVC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *      
  *  @endverbatim
  *                                  
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_uvc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

volatile uint8_t uvc_stream_status = 0;

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_UVC 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_UVC_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_UVC_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_UVC_Private_Macros
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_UVC_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_UVC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_UVC_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_UVC_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  USBD_UVC_DataIn (USBD_HandleTypeDef *pdev, 
                                 uint8_t epnum);

static uint8_t  USBD_UVC_DataOut (USBD_HandleTypeDef *pdev, 
                                uint8_t epnum);

static uint8_t  USBD_UVC_SOF (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_UVC_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  *USBD_UVC_GetFSCfgDesc (uint16_t *length);

uint8_t  *USBD_UVC_GetDeviceQualifierDescriptor (uint16_t *length);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_UVC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */ 

/** @defgroup USBD_UVC_Private_Variables
  * @{
  */ 


/* UVC interface class callbacks structure */
USBD_ClassTypeDef  USBD_UVC = 
{
  USBD_UVC_Init,
  USBD_UVC_DeInit,
  USBD_UVC_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_UVC_EP0_RxReady,
  USBD_UVC_DataIn,
  USBD_UVC_DataOut,
  USBD_UVC_SOF,
  NULL,
  NULL,     
  USBD_UVC_GetFSCfgDesc,  
  USBD_UVC_GetFSCfgDesc,    
  USBD_UVC_GetFSCfgDesc, 
  USBD_UVC_GetDeviceQualifierDescriptor,
};


/* USB UVC device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_UVC_CfgFSDesc[] __ALIGN_END =
{
  /* Configuration 1 */
  USB_LEN_CFG_DESC,               // bLength                  9
  USB_DESC_TYPE_CONFIGURATION,         // bDescriptorType          2
  WBVAL(USB_VIDEO_DESC_SIZ),
  0x02,                                      // bNumInterfaces           2
  0x01,                                      // bConfigurationValue      1 ID of this configuration
  0x00,                                      // iConfiguration           0 no description available
  USB_CONFIG_BUS_POWERED ,                   // bmAttributes          0x80 Bus Powered
  USB_CONFIG_POWER_MA(100),                  // bMaxPower              100 mA
  
  
  /* Interface Association Descriptor */
  UVC_LEN_IF_ASSOCIATION_DESC,       // bLength                  8
  USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE, // bDescriptorType         11
  0x00,                                      // bFirstInterface          0
  0x02,                                      // bInterfaceCount          2
  CC_VIDEO,                                  // bFunctionClass          14 Video
  SC_VIDEO_INTERFACE_COLLECTION,             // bFunctionSubClass        3 Video Interface Collection
  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
  0x02,                                      // iFunction                2
  
  
  /* VideoControl Interface Descriptor */
  
  
  /* Standard VC Interface Descriptor  = interface 0 */
  USB_LEN_IF_DESC,                   // bLength                  9
  USB_DESC_TYPE_INTERFACE,             // bDescriptorType          4
  USB_UVC_VCIF_NUM,                          // bInterfaceNumber         0 index of this interface (VC)
  0x00,                                      // bAlternateSetting        0 index of this setting
  0x00,                                      // bNumEndpoints            0 no endpoints
  CC_VIDEO,                                  // bInterfaceClass         14 Video
  SC_VIDEOCONTROL,                           // bInterfaceSubClass       1 Video Control
  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
  0x02,                                      // iFunction                2
  
  
  /* Class-specific VC Interface Descriptor */
  UVC_VC_INTERFACE_HEADER_DESC_SIZE(1),      // bLength                 13 12 + 1 (header + 1*interface
  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
  VC_HEADER,                                 // bDescriptorSubtype      1 (HEADER)
  WBVAL(UVC_VERSION),                        // bcdUVC                  1.10 or 1.00
  WBVAL(VC_TERMINAL_SIZ),                    // wTotalLength            header+units+terminals
  DBVAL(0x005B8D80),                         // dwClockFrequency  6.000000 MHz
  0x01,                                      // bInCollection            1 one streaming interface
  0x01,                                      // baInterfaceNr( 0)        1 VS interface 1 belongs to this VC interface
  
  
  /* Input Terminal Descriptor (Camera) */
  UVC_CAMERA_TERMINAL_DESC_SIZE(2),          // bLength                 17 15 + 2 controls
  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
  VC_INPUT_TERMINAL,                         // bDescriptorSubtype       2 (INPUT_TERMINAL)
  0x01,                                      // bTerminalID              1 ID of this Terminal
  WBVAL(ITT_CAMERA),                         // wTerminalType       0x0201 Camera Sensor
  0x00,                                      // bAssocTerminal           0 no Terminal associated
  0x00,                                      // iTerminal                0 no description available
  WBVAL(0x0000),                             // wObjectiveFocalLengthMin 0
  WBVAL(0x0000),                             // wObjectiveFocalLengthMax 0
  WBVAL(0x0000),                             // wOcularFocalLength       0
  0x02,                                      // bControlSize             2
  0x00, 0x00,                                // bmControls          0x0000 no controls supported
  
  /* Output Terminal Descriptor */
  UVC_OUTPUT_TERMINAL_DESC_SIZE(0),          // bLength                  9
  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
  VC_OUTPUT_TERMINAL,                        // bDescriptorSubtype       3 (OUTPUT_TERMINAL)
  0x02,                                      // bTerminalID              2 ID of this Terminal
  WBVAL(TT_STREAMING),                       // wTerminalType       0x0101 USB streaming terminal
  0x00,                                      // bAssocTerminal           0 no Terminal assiciated
  0x01,                                      // bSourceID                1 input pin connected to output pin unit 1
  0x00,                                      // iTerminal                0 no description available
  
  
  /* Video Streaming (VS) Interface Descriptor */
  
  
  /* Standard VS Interface Descriptor  = interface 1 */
  // alternate setting 0 = Zero Bandwidth
  USB_LEN_IF_DESC,                   // bLength                  9
  USB_DESC_TYPE_INTERFACE,             // bDescriptorType          4
  USB_UVC_VSIF_NUM,                          // bInterfaceNumber         1 index of this interface
  0x00,                                      // bAlternateSetting        0 index of this setting
  0x00,                                      // bNumEndpoints            0 no EP used
  CC_VIDEO,                                  // bInterfaceClass         14 Video
  SC_VIDEOSTREAMING,                         // bInterfaceSubClass       2 Video Streaming
  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
  0x00,                                      // iInterface               0 no description available
  
  
  /* Class-specific VS Header Descriptor (Input) */
  UVC_VS_INTERFACE_INPUT_HEADER_DESC_SIZE(1,1),// bLength               14 13 + (1*1) (no specific controls used)
  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
  VS_INPUT_HEADER,                           // bDescriptorSubtype       1 (INPUT_HEADER)
  0x01,                                      // bNumFormats              1 one format descriptor follows
  WBVAL(VC_HEADER_SIZ),
  USB_ENDPOINT_IN(1),                        // bEndPointAddress      0x83 EP 3 IN
  0x00,                                      // bmInfo                   0 no dynamic format change supported
  0x02,                                      // bTerminalLink            2 supplies terminal ID 2 (Output terminal)
  0x00,                                      // bStillCaptureMethod      0 NO supports still image capture
  0x01,                                      // bTriggerSupport          0 HW trigger supported for still image capture
  0x00,                                      // bTriggerUsage            0 HW trigger initiate a still image capture
  0x01,                                      // bControlSize             1 one byte bmaControls field size
  0x00,                                      // bmaControls(0)           0 no VS specific controls
  
  
  /* Class-specific VS Format Descriptor  */
  VS_FORMAT_UNCOMPRESSED_DESC_SIZE,     /* bLength 27*/
  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */
  VS_FORMAT_UNCOMPRESSED,               /* bDescriptorSubType : VS_FORMAT_UNCOMPRESSED subtype */
  0x01,                                 /* bFormatIndex : First (and only) format descriptor */
  0x01,                                 /* bNumFrameDescriptors : One frame descriptor for this format follows. */
  GUID_VS_FORMAT,
  BITS_PER_PIXEL,                       /* bBitsPerPixel : Number of bits per pixel used to specify color in the decoded video frame - 16 for yuy2, 12 for nv12... */
  0x01,                                 /* bDefaultFrameIndex : Default frame index is 1. */
  0x00,                                 /* bAspectRatioX : Non-interlaced stream not required. */
  0x00,                                 /* bAspectRatioY : Non-interlaced stream not required. */
  0x00,                                 /* bmInterlaceFlags : Non-interlaced stream */
  0x00,                                 /* bCopyProtect : No restrictions imposed on the duplication of this video stream. */
  
  /* Class-specific VS Frame Descriptor */
  VS_FRAME_UNCOMPRESSED_DESC_SIZE,      /* bLength 30*/
  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */
  VS_FRAME_UNCOMPRESSED,                /* bDescriptorSubType : VS_FRAME_UNCOMPRESSED */
  0x01,                                 /* bFrameIndex : First (and only) frame descriptor */
  0x02,                                 /* bmCapabilities : Still images using capture method 0 are supported at this frame setting.D1: Fixed frame-rate. */
  WBVAL(WIDTH),                         /* wWidth (2bytes): Width of frame is 128 pixels. */
  WBVAL(HEIGHT),                        /* wHeight (2bytes): Height of frame is 64 pixels. */
  DBVAL(MIN_BIT_RATE),                  /* dwMinBitRate (4bytes): Min bit rate in bits/s  */ // 128*64*16*5 = 655360 = 0x000A0000 //5fps
  DBVAL(MAX_BIT_RATE),                  /* dwMaxBitRate (4bytes): Max bit rate in bits/s  */ // 128*64*16*5 = 655360 = 0x000A0000
  DBVAL(MAX_FRAME_SIZE),                /* dwMaxVideoFrameBufSize (4bytes): Maximum video or still frame size, in bytes. */ // 128*64*2 = 16384 = 0x00004000
  DBVAL(INTERVAL),				        /* dwDefaultFrameInterval : 1,000,000 * 100ns -> 10 FPS */ // 5 FPS -> 200ms -> 200,000 us -> 2,000,000 X 100ns = 0x001e8480
  0x01,                                 /* bFrameIntervalType : Continuous frame interval */
  DBVAL(INTERVAL),                      /* dwMinFrameInterval : 1,000,000 ns  *100ns -> 10 FPS */
  
  /* Color Matching Descriptor */
  VS_COLOR_MATCHING_DESC_SIZE,          /* bLength */
  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */
  0x0D,                                 /* bDescriptorSubType : VS_COLORFORMAT */
  0x01,                                 /* bColorPrimarie : 1: BT.709, sRGB (default) */
  0x01,                                 /* bTransferCharacteristics : 1: BT.709 (default) */
  0x04,                                 /* bMatrixCoefficients : 1: BT. 709. */
  
  
  /* Standard VS Interface Descriptor  = interface 1 */
  // alternate setting 1 = operational setting
  USB_LEN_IF_DESC,                   // bLength                  9
  USB_DESC_TYPE_INTERFACE,             // bDescriptorType          4
  USB_UVC_VSIF_NUM,                          // bInterfaceNumber         1 index of this interface
  0x01,                                      // bAlternateSetting        1 index of this setting
  0x01,                                      // bNumEndpoints            1 one EP used
  CC_VIDEO,                                  // bInterfaceClass         14 Video
  SC_VIDEOSTREAMING,                         // bInterfaceSubClass       2 Video Streaming
  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
  0x00,                                      // iInterface               0 no description available
  
  
  
  /* Standard VS Isochronous Video data Endpoint Descriptor */
  USB_LEN_EP_DESC,                   // bLength                  7
  USB_DESC_TYPE_ENDPOINT,             // bDescriptorType          5 (ENDPOINT)
  USB_ENDPOINT_IN(1),                       // bEndpointAddress      0x83 EP 3 IN
  USBD_EP_TYPE_ISOC,            // bmAttributes             1 isochronous transfer type
  WBVAL(VIDEO_PACKET_SIZE),                 // wMaxPacketSize
  0x01                                      // bInterval                1 one frame interval
};

/**
  * @}
  */ 

/** @defgroup USBD_UVC_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_UVC_Init
  *         Initialize the UVC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_UVC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  uint8_t ret = 0;
  USBD_UVC_HandleTypeDef   *hcdc;
  
  if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
  {  
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
                   UVC_IN_EP,
                   USBD_EP_TYPE_ISOC,
                   UVC_DATA_HS_IN_PACKET_SIZE);
  }
  else
  {
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
                   UVC_IN_EP,
                   USBD_EP_TYPE_ISOC,
                   UVC_DATA_FS_IN_PACKET_SIZE);
  }
    
  pdev->pClassData = USBD_malloc(sizeof (USBD_UVC_HandleTypeDef));
  
  if(pdev->pClassData == NULL)
  {
    ret = 1; 
  }
  else
  {
    hcdc = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
    
    /* Init  physical Interface components */
    ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->Init();
    
    /* Init Xfer states */
    hcdc->TxState =0;
    hcdc->RxState =0;
  }
  return ret;
}

/**
  * @brief  USBD_UVC_Init
  *         DeInitialize the UVC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_UVC_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  uint8_t ret = 0;
  
  /* Open EP IN */
  USBD_LL_CloseEP(pdev,
              UVC_IN_EP);

  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  
  return ret;
}

/**
  * @brief  USBD_UVC_Setup
  *         Handle the UVC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_UVC_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  USBD_UVC_HandleTypeDef   *hcdc = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  static uint8_t ifalt = 0;
  uint8_t ret = USBD_OK;
    
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    if (req->wLength)
    {
      if (req->bmRequest & 0x80)
      {
        USBD_LL_FlushEP (pdev,USB_ENDPOINT_OUT(0));

        ret = ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                          (uint8_t *)hcdc->data,
                                                          req->wLength,
                                                          req->wIndex,
                                                          req->wValue);

        if (ret == USBD_OK)
          USBD_CtlSendData (pdev, 
                            (uint8_t *)hcdc->data,
                            req->wLength);
        else
          USBD_CtlError (pdev, req);
      }
      else
      {
        hcdc->CmdOpCode = req->bRequest;
        hcdc->CmdLength = req->wLength;
        hcdc->CmdIndex = req->wIndex;
        hcdc->CmdValue = req->wValue;
        
        USBD_CtlPrepareRx (pdev, 
                           (uint8_t *)hcdc->data,
                           req->wLength);
      }
      
    }
    else
    {
      ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                        (uint8_t*)req,
                                                        0, 0, 0);
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( (req->wValue >> 8) == CS_DEVICE)
      {
        printf("USB_REQ_GET_DESCRIPTOR(CS_DEVICE)\r\n");
        USBD_CtlSendData (pdev, (uint8_t *)USBD_UVC_CfgFSDesc + 18, MIN(USB_VIDEO_DESC_SIZ , req->wLength));
      }
      else
      {
        printf("USB_REQ_GET_DESCRIPTOR\r\n");
        USBD_CtlSendData (pdev, (uint8_t *)hcdc->data, req->wLength);
      }
      break;

    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        &ifalt,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      if ((uint8_t)(req->wValue) < VIDEO_TOTAL_IF_NUM)
      {
        ifalt = (uint8_t)(req->wValue);

        // TODO: refactor this to callback user code instead of doing this here

        if (ifalt == 1) {
          printf("USB_REQ_SET_INTERFACE: 1\r\n");
        	uvc_stream_status = 1;
        } else {
          printf("USB_REQ_SET_INTERFACE: %d\r\n", req->wValue);
        	uvc_stream_status = 0;
        }
      }
      else
      {
        /* Call the error management function (command will be nacked */
        USBD_CtlError (pdev, req);
        return USBD_FAIL;
      }
      break;
    default:
      printf("Unhandled standard request %x\r\n", req->bRequest);
    }
    break;
 
  default: 
    printf("Unknown setup request: %x\r\n", req->bmRequest);
    return USBD_FAIL;
    break;
  }
  return ret;
}

/**
  * @brief  USBD_UVC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_UVC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_UVC_HandleTypeDef   *hcdc = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    
    hcdc->TxState = 0;

    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_UVC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_UVC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
  USBD_UVC_HandleTypeDef   *hcdc = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  /* Get the received data length */
  hcdc->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
  
  /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the application Xfer */
  if(pdev->pClassData != NULL)
  {
    ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->Receive(hcdc->RxBuffer, &hcdc->RxLength);

    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

static uint8_t  USBD_UVC_SOF (USBD_HandleTypeDef *pdev)
{     
  printf("USBD_UVC_SOF()\n");

  // This is being done in our main loop, but this callback doesn't seem to get called right now anyhow

  // if (uvc_stream_status == 1)
  // {
  //     USBD_LL_FlushEP(pdev,USB_ENDPOINT_IN(1));
  //     USBD_LL_Transmit(pdev,USB_ENDPOINT_IN(1), (uint8_t*)0x0002, 2);//header
  //     uvc_stream_status = 2;
  // }

  return USBD_OK;
}


/**
  * @brief  USBD_UVC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_UVC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{ 
  uint8_t ret = USBD_OK;
  USBD_UVC_HandleTypeDef   *hcdc = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  if((pdev->pUserData != NULL) && (hcdc->CmdOpCode != 0xFF))
  {
    ret = ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->Control(hcdc->CmdOpCode,
                                                      (uint8_t *)hcdc->data,
                                                      hcdc->CmdLength, hcdc->CmdIndex, hcdc->CmdValue);
      hcdc->CmdOpCode = 0xFF; 
      if (ret == USBD_FAIL)
        USBD_CtlError (pdev, 0);
  }
  return ret;
}

/**
  * @brief  USBD_UVC_GetFSCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_UVC_GetFSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_UVC_CfgFSDesc);
  return USBD_UVC_CfgFSDesc;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_UVC_GetDeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_UVC_DeviceQualifierDesc);
  return USBD_UVC_DeviceQualifierDesc;
}

/**
* @brief  USBD_UVC_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t  USBD_UVC_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                      USBD_UVC_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;
  
  if(fops != NULL)
  {
    pdev->pUserData= fops;
    ret = USBD_OK;    
  }
  
  return ret;
}

/**
  * @brief  USBD_UVC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_UVC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length)
{
  USBD_UVC_HandleTypeDef   *hcdc = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;  
  
  return USBD_OK;  
}


/**
  * @brief  USBD_UVC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_UVC_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                   uint8_t  *pbuff)
{
  USBD_UVC_HandleTypeDef   *hcdc = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  hcdc->RxBuffer = pbuff;
  
  return USBD_OK;
}

/**
  * @brief  USBD_UVC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_UVC_TransmitPacket(USBD_HandleTypeDef *pdev)
{      
  USBD_UVC_HandleTypeDef   *hcdc = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    if(hcdc->TxState == 0)
    {
      /* Tx Transfer in progress */
      hcdc->TxState = 1;
      
      /* Transmit next packet */
      USBD_LL_Transmit(pdev,
                       UVC_IN_EP,
                       hcdc->TxBuffer,
                       hcdc->TxLength);
      
      return USBD_OK;
    }
    else
    {
      return USBD_BUSY;
    }
  }
  else
  {
    return USBD_FAIL;
  }
}


/**
  * @brief  USBD_UVC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_UVC_ReceivePacket(USBD_HandleTypeDef *pdev)
{      
  USBD_UVC_HandleTypeDef   *hcdc = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  /* Suspend or Resume USB Out process */
  if(pdev->pClassData != NULL)
  {
    if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
    {      
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             UVC_OUT_EP,
                             hcdc->RxBuffer,
                             UVC_DATA_HS_OUT_PACKET_SIZE);
    }
    else
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             UVC_OUT_EP,
                             hcdc->RxBuffer,
                             UVC_DATA_FS_OUT_PACKET_SIZE);
    }
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
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
