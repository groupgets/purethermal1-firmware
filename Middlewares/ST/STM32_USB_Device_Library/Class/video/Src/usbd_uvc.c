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
#include "usbd_types.h"
#include "uvc_desc.h"

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

// #define UVC_SETUP_REQ_DEBUG

volatile uint8_t g_uvc_stream_status = 0;
volatile uint16_t g_uvc_stream_packet_size = 0;
volatile uint8_t g_lepton_type_3 = 0;
volatile uint8_t g_telemetry_num_lines = 0;
volatile uint8_t g_format_y16 = 0;

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
__ALIGN_BEGIN struct usbd_uvc_cfg USBD_UVC_CfgFSDesc_L2 __ALIGN_END =
{
  #include "uvc_desc_conf.h"
  #include "uvc_desc_va.h"
  #include "uvc_desc_vc.h"
  #include "uvc_desc_vs_l2.h"
  #include "uvc_desc_vs_if.h"
};

__ALIGN_BEGIN struct usbd_uvc_cfg USBD_UVC_CfgFSDesc_L3 __ALIGN_END =
{
  #include "uvc_desc_conf.h"
  #include "uvc_desc_va.h"
  #include "uvc_desc_vc.h"
  #include "uvc_desc_vs_l3.h"
  #include "uvc_desc_vs_if.h"
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
  
  /* Open EP IN */
  USBD_LL_OpenEP(pdev,
                 UVC_IN_EP,
                 USBD_EP_TYPE_ISOC,
                 VIDEO_PACKET_SIZE_MAX);

  USBD_LL_OpenEP(pdev,
                 UVC_CMD_EP,
                 USBD_EP_TYPE_INTR,
                 CMD_PACKET_SIZE);


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

  USBD_LL_CloseEP(pdev,
              UVC_CMD_EP);

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
      // TODO: what to do with recipient as endpoint?

      uint8_t address = (req->wIndex >> 0) & 0xff;
      VC_TERMINAL_ID entity_id = (VC_TERMINAL_ID)((req->wIndex >> 8) & 0xff);

#ifdef UVC_SETUP_REQ_DEBUG
      DEBUG_PRINTF("Setup USB_REQ_TYPE_CLASS read=%d recipient=%d (0=dev,1=intf,2=ep)\r\n",
        ((req->bmRequest & USB_REQ_READ_MASK) == USB_REQ_READ_MASK),
        req->bmRequest & USB_REQ_RECIPIENT_MASK);

      DEBUG_PRINTF(" wValue=%x: address=%d entity_id=%d (0=ep)\r\n",
        req->wIndex, address, entity_id);
#endif

      if (req->bmRequest & USB_REQ_READ_MASK)
      {
        USBD_LL_FlushEP (pdev,USB_ENDPOINT_OUT(0));

        if (address == USB_UVC_VCIF_NUM)
        {
          ret = ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->ControlGet(entity_id,
                                                                      req->bRequest,
                                                                      (uint8_t *)hcdc->data,
                                                                      req->wLength,
                                                                      req->wIndex,
                                                                      req->wValue);
        }
        else
        {
          ret = ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->VS_CtrlGet(req->bRequest,
                                                                      (uint8_t *)hcdc->data,
                                                                      req->wLength,
                                                                      req->wIndex,
                                                                      req->wValue);
        }

        if (ret == USBD_OK)
          USBD_CtlSendData (pdev, 
                            (uint8_t *)hcdc->data,
                            req->wLength);
        else if (ret == USBD_BUSY)
          ret = USBD_OK; // response was dispatched, nothing more to do here
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
      if( (req->wValue >> 8) == UVC_CS_DEVICE)
      {
        DEBUG_PRINTF("USB_REQ_GET_DESCRIPTOR(UVC_CS_DEVICE)\r\n");
        if (g_lepton_type_3)
        {
          USBD_CtlSendData (pdev, (uint8_t *)&USBD_UVC_CfgFSDesc_L3.uvc_vc_if_desc, MIN(sizeof(USBD_UVC_CfgFSDesc_L3) , req->wLength));
        }
        else
        {
          USBD_CtlSendData (pdev, (uint8_t *)&USBD_UVC_CfgFSDesc_L2.uvc_vc_if_desc, MIN(sizeof(USBD_UVC_CfgFSDesc_L2) , req->wLength));
        }
      }
      else
      {
        DEBUG_PRINTF("USB_REQ_GET_DESCRIPTOR\r\n");
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

        if (ifalt > 0) {
          struct uvc_vs_alt_setting *alt;
          // DEBUG_PRINTF("USB_REQ_SET_INTERFACE: 1\r\n");
          g_uvc_stream_status = 1;
          alt = &(g_lepton_type_3 ? USBD_UVC_CfgFSDesc_L3 : USBD_UVC_CfgFSDesc_L2).uvc_vs_alt[ifalt - USB_UVC_VSIF_ALT_START];
          g_uvc_stream_packet_size = alt->ep.wMaxPacketSize;

        } else {
          DEBUG_PRINTF("USB_REQ_SET_INTERFACE: %d\r\n", req->wValue);
          g_uvc_stream_status = 0;
          g_uvc_stream_packet_size = 0;
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
      DEBUG_PRINTF("Unhandled standard request %x\r\n", req->bRequest);
    }
    break;
 
  default: 
    DEBUG_PRINTF("Unknown setup request: %x\r\n", req->bmRequest);
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
  DEBUG_PRINTF("USBD_UVC_SOF()\n");

  // This is being done in our main loop, but this callback doesn't seem to get called right now anyhow

  // if (g_uvc_stream_status == 1)
  // {
  //     USBD_LL_FlushEP(pdev,USB_ENDPOINT_IN(1));
  //     USBD_LL_Transmit(pdev,USB_ENDPOINT_IN(1), (uint8_t*)0x0002, 2);//header
  //     g_uvc_stream_status = 2;
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
    uint8_t address = (hcdc->CmdIndex >> 0) & 0xff;
    uint8_t entity_id = (hcdc->CmdIndex >> 8) & 0xff;

    if (address == USB_UVC_VCIF_NUM)
    {
      ret = ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->ControlSet(entity_id,
                                                        hcdc->CmdOpCode,
                                                        (uint8_t *)hcdc->data,
                                                        hcdc->CmdLength,
                                                        hcdc->CmdIndex,
                                                        hcdc->CmdValue);
    }
    else
    {
      ret = ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->VS_CtrlSet(hcdc->CmdOpCode,
                                                        (uint8_t *)hcdc->data,
                                                        hcdc->CmdLength,
                                                        hcdc->CmdIndex,
                                                        hcdc->CmdValue);
    }


    hcdc->CmdOpCode = 0xFF; 
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
  if (g_lepton_type_3)
  {
    *length = sizeof(USBD_UVC_CfgFSDesc_L3);
    return (uint8_t*)&USBD_UVC_CfgFSDesc_L3;
  }
  else
  {
    *length = sizeof(USBD_UVC_CfgFSDesc_L2);
    return (uint8_t*)&USBD_UVC_CfgFSDesc_L2;
  }
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
    /* Prepare Out endpoint to receive next packet */
    // USBD_LL_PrepareReceive(pdev,
    //                        UVC_OUT_EP,
    //                        hcdc->RxBuffer,
    //                        UVC_DATA_FS_OUT_PACKET_SIZE);
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
