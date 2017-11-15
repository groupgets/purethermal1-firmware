#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_def.h"
#include "usbd_ioreq.h"

#include "lepton.h"
#include "lepton_i2c.h"
#include "project_config.h"
#include "tasks.h"
#include "circ_buf.h"
#include "usbd_uvc_lepton_xu.h"

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_VID.h"
#include "LEPTON_OEM.h"
#include "LEPTON_RAD.h"
#include "LEPTON_I2C_Reg.h"
#include "crc16.h"

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

#define MAX_I2C_BUFFER_SIZE 1024

extern USBD_HandleTypeDef  *hUsbDevice_0;
extern LEP_CAMERA_PORT_DESC_T hport_desc;

struct uvc_request* attribute_get_requests_buf[4] = { 0 };
DECLARE_CIRC_BUF_HANDLE(attribute_get_requests_buf);

HAL_StatusTypeDef enqueue_attribute_get_task(struct uvc_request req)
{
  if (full(CIRC_BUF_HANDLE(attribute_get_requests_buf))) {
    return HAL_ERROR;
  }
  else {
    static uint32_t reqnum = 0;
    static struct uvc_request uvc_requests[4];
    struct uvc_request *preq = &uvc_requests[reqnum++ % 4];
    *preq = req;
    push(CIRC_BUF_HANDLE(attribute_get_requests_buf), preq);
    return HAL_OK;
  }
}

HAL_StatusTypeDef dequeue_attribute_get_task(struct uvc_request *req)
{
  if (empty(CIRC_BUF_HANDLE(attribute_get_requests_buf))) {
    return HAL_ERROR;
  }
  else {
    struct uvc_request *preq = shift(CIRC_BUF_HANDLE(attribute_get_requests_buf));
    *req = *preq;
    return HAL_OK;
  }
}

PT_THREAD( LEP_I2C_GetAttribute_PT(struct pt *pt, LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
        LEP_COMMAND_ID commandID,
        LEP_ATTRIBUTE_T_PTR attributePtr,
        LEP_UINT16 attributeWordLength,
        LEP_RESULT *return_code))
{
    static LEP_RESULT result;
    static LEP_UINT16 statusReg;
    static LEP_INT16 statusCode;
    static LEP_UINT32 done;
    static LEP_UINT16 crcExpected, crcActual;

    PT_BEGIN(pt);

    /* Implement the Lepton TWI READ Protocol
    */
    /* First wait until the Camera is ready to receive a new
    ** command by polling the STATUS REGISTER BUSY Bit until it
    ** reports NOT BUSY.
    */

    do
    {
        /* Read the Status REGISTER and peek at the BUSY Bit
        */
        result = LEP_I2C_MasterReadData( portDescPtr->portID,
                                         portDescPtr->deviceAddress,
                                         LEP_I2C_STATUS_REG,
                                         &statusReg,
                                         1 );
        if(result != LEP_OK)
        {
            *return_code = result;
            PT_EXIT(pt);
        }
        done = (statusReg & LEP_I2C_STATUS_BUSY_BIT_MASK)? 0: 1;

        PT_YIELD(pt);
    }while( !done );

    /* Set the Lepton's DATA LENGTH REGISTER first to inform the
    ** Lepton Camera how many 16-bit DATA words we want to read.
    */
    result = LEP_I2C_MasterWriteData( portDescPtr->portID,
                                      portDescPtr->deviceAddress,
                                      LEP_I2C_DATA_LENGTH_REG,
                                      &attributeWordLength,
                                      1);
    if(result != LEP_OK)
    {
        *return_code = result;
        PT_EXIT(pt);
    }

    PT_YIELD(pt);

    /* Now issue the GET Attribute Command
    */
    result = LEP_I2C_MasterWriteData( portDescPtr->portID,
                                      portDescPtr->deviceAddress,
                                      LEP_I2C_COMMAND_REG,
                                      &commandID,
                                      1);

    if(result != LEP_OK)
    {
        *return_code = result;
        PT_EXIT(pt);
    }

    PT_YIELD(pt);

    /* Now wait until the Camera has completed this command by
    ** polling the statusReg REGISTER BUSY Bit until it reports NOT
    ** BUSY.
    */
    do
    {
        /* Read the statusReg REGISTER and peek at the BUSY Bit
        */
        result = LEP_I2C_MasterReadData( portDescPtr->portID,
                                         portDescPtr->deviceAddress,
                                         LEP_I2C_STATUS_REG,
                                         &statusReg,
                                         1 );

        if(result != LEP_OK)
        {
            *return_code = result;
            PT_EXIT(pt);
        }
        done = (statusReg & LEP_I2C_STATUS_BUSY_BIT_MASK)? 0: 1;

        PT_YIELD(pt);
    }while( !done );


    /* Check statusReg word for Errors?
    */
    statusCode = (statusReg >> 8) ? ((statusReg >> 8) | 0xFF00) : 0;
    if(statusCode)
    {
        *return_code = (LEP_RESULT)statusCode;
        PT_EXIT(pt);
    }

    /* If NO Errors then READ the DATA from the DATA REGISTER(s)
    */
    if( attributeWordLength <= 16 )
    {
        /* Read from the DATA Registers - always start from DATA 0
        ** Little Endean
        */
        result = LEP_I2C_MasterReadData(portDescPtr->portID,
                                        portDescPtr->deviceAddress,
                                        LEP_I2C_DATA_0_REG,
                                        attributePtr,
                                        attributeWordLength );
    }
    else if( attributeWordLength <= 1024 )
    {
        /* Read from the DATA Block Buffer
        */
      result = LEP_I2C_MasterReadData(portDescPtr->portID,
                                      portDescPtr->deviceAddress,
                                      LEP_I2C_DATA_BUFFER_0,
                                      attributePtr,
                                      attributeWordLength );
    }

    PT_YIELD(pt);

    if(result == LEP_OK && attributeWordLength > 0)
    {
       /* Check CRC */
       result = LEP_I2C_MasterReadData( portDescPtr->portID,
                                        portDescPtr->deviceAddress,
                                        LEP_I2C_DATA_CRC_REG,
                                        &crcExpected,
                                        1);
       crcActual = (LEP_UINT16)CalcCRC16Words(attributeWordLength, (short*)attributePtr);

       /* Check for 0 in the register in case the camera does not support CRC check
       */
       if(crcExpected != 0 && crcExpected != crcActual)
       {
           *return_code = LEP_CHECKSUM_ERROR;
           PT_EXIT(pt);
       }

    }

    *return_code = result;

    PT_END(pt);
}

PT_THREAD( lepton_attribute_get_task(struct pt *pt))
{
  static struct uvc_request req;
  static uint8_t pbuf[MAX_I2C_BUFFER_SIZE];
  static LEP_RESULT result;
  static uint16_t module_base;
  static struct pt lep_pt;
  static int retries;

  PT_BEGIN(pt);

  while (1)
  {
    PT_WAIT_UNTIL(pt, dequeue_attribute_get_task(&req) == HAL_OK);

    if (req.length < 1 || req.length > sizeof(pbuf))
      continue;

    module_base = vc_terminal_id_to_module_base(req.entity_id);

    // If reading from the OTP (serial number, part number, etc.), we
    // sometimes get back a -16 or -17 (correctable bit errors), so just try
    // one extra time regardless of what the problem was
    retries = 1;

    do {
      PT_INIT(&lep_pt);

      PT_WAIT_THREAD(pt, LEP_I2C_GetAttribute_PT(&lep_pt, &hport_desc,
                                          ( LEP_COMMAND_ID )(module_base + req.control_id),
                                          ( LEP_ATTRIBUTE_T_PTR )pbuf,
                                          req.length >> 1,
                                          &result));

      PT_YIELD(pt);
    } while (result != LEP_OK && retries--);

    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

    if (result == LEP_OK)
      USBD_CtlSendData(hUsbDevice_0, pbuf, req.length);
    else
      USBD_CtlError(hUsbDevice_0, 0);

    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  }

  PT_END(pt);
}
