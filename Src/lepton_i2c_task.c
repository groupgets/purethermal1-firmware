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
#include "custom_uvc_i2c.h"

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

#define MAX_I2C_BUFFER_SIZE 2048
#define REQUEST_QUEUE_SIZE 2

extern USBD_HandleTypeDef  *hUsbDevice_0;
extern LEP_CAMERA_PORT_DESC_T hport_desc;

static struct uvc_request uvc_requests[REQUEST_QUEUE_SIZE];
static uint8_t uvc_request_buffers[REQUEST_QUEUE_SIZE][MAX_I2C_BUFFER_SIZE];

struct uvc_request* attribute_xfer_requests[REQUEST_QUEUE_SIZE] = { 0 };
DECLARE_CIRC_BUF_HANDLE(attribute_xfer_requests);

HAL_StatusTypeDef enqueue_attribute_xfer_task(struct uvc_request req)
{
  if (full(CIRC_BUF_HANDLE(attribute_xfer_requests))) {
    return HAL_ERROR;
  }
  else {
    static uint32_t reqnum = 0;

    struct uvc_request *preq = &uvc_requests[reqnum];
    uint8_t* buffer = uvc_request_buffers[reqnum];

    /* for set transfers, make a copy of the data so it's not overwritten by another incoming request */
    if (req.type == UVC_REQUEST_TYPE_ATTR_SET)
    {
      memcpy(buffer, req.buffer, req.length);
    }

    /* assign static storage to use for this request */
    req.buffer = buffer;

    *preq = req;

    push(CIRC_BUF_HANDLE(attribute_xfer_requests), preq);
    reqnum = ((reqnum + 1) % REQUEST_QUEUE_SIZE);
    return HAL_OK;
  }
}

HAL_StatusTypeDef dequeue_attribute_xfer_task(struct uvc_request *req)
{
  if (empty(CIRC_BUF_HANDLE(attribute_xfer_requests))) {
    return HAL_ERROR;
  }
  else {
    struct uvc_request *preq = shift(CIRC_BUF_HANDLE(attribute_xfer_requests));
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

PT_THREAD( LEP_I2C_WaitForBusyBit(struct pt *pt,
								 LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
								 LEP_RESULT *return_code, LEP_INT16 *return_status)) {
    static LEP_RESULT result;
    static LEP_UINT16 statusReg;
    static LEP_INT16 statusCode;
    static LEP_UINT32 done;
    static LEP_UINT16 timeoutCount;

    PT_BEGIN(pt);

    timeoutCount = LEPTON_I2C_COMMAND_BUSY_WAIT_COUNT;

    do
    {
        /* Read the Status REGISTER and peek at the BUSY Bit
        */ 
        result = LEP_I2C_MasterReadRegister( portDescPtr->portID,
                                             portDescPtr->deviceAddress,
                                             LEP_I2C_STATUS_REG,
                                             &statusReg);
        if(result != LEP_OK)
        {
            *return_code = result;
            PT_EXIT(pt);
        }
        done = (statusReg & LEP_I2C_STATUS_BUSY_BIT_MASK)? 0: 1;
        /* Add timout check */
        if( timeoutCount-- == 0 )
        {
            /* Timed out waiting for command busy to go away
            */ 
            *return_code = LEP_TIMEOUT_ERROR;
            PT_EXIT(pt);
        }

        PT_YIELD(pt);
    }while( !done );

    statusCode = (statusReg >> 8) ? ((statusReg >> 8) | 0xFF00) : 0;
    if(statusCode)
    {
        if (return_status)
            *return_status = statusCode;
        else
            *return_code = (LEP_RESULT)statusCode;
    }

    PT_END(pt);
}

PT_THREAD( LEP_I2C_RunCommand_PT(struct pt *pt,
                                 LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                 LEP_COMMAND_ID commandID,
                                 LEP_RESULT *return_code))
{
    static LEP_RESULT result;
    static struct pt wait_pt;
    LEP_INT16 status;

    PT_BEGIN(pt);

    /* First wait until the Camera is ready to receive a new
    ** command by polling the STATUS REGISTER BUSY Bit until it
    ** reports NOT BUSY.
    ** Ignore the status reported from any previous operation.
    */
    PT_WAIT_THREAD(pt, LEP_I2C_WaitForBusyBit(&wait_pt, &hport_desc,
                                              &result, &status));

    /* Implement the Lepton TWI WRITE Protocol
    */

    if( result == LEP_OK )
    {
        /* Set the Lepton's DATA LENGTH REGISTER first to inform the
        ** Lepton Camera no 16-bit DATA words being transferred.
        */ 
        result = LEP_I2C_MasterWriteRegister( portDescPtr->portID,
                                              portDescPtr->deviceAddress,
                                              LEP_I2C_DATA_LENGTH_REG, 
                                              (LEP_UINT16)0);

        PT_YIELD(pt);

        if( result == LEP_OK )
        {
            /* Now issue the Run Command
            */ 
            result = LEP_I2C_MasterWriteRegister( portDescPtr->portID,
                                                  portDescPtr->deviceAddress,
                                                  LEP_I2C_COMMAND_REG, 
                                                  commandID);

            PT_YIELD(pt);

            if( result == LEP_OK )
            {
                /* Now wait until the Camera has completed this command by
                ** polling the statusReg REGISTER BUSY Bit until it reports NOT
                ** BUSY.
                */ 
                PT_WAIT_THREAD(pt, LEP_I2C_WaitForBusyBit(&wait_pt, &hport_desc,
                                                          &result, NULL));

                if(result)
                {
                  *return_code = (LEP_RESULT)result;
                  PT_EXIT(pt);
                }
            }
        }
    }

    /* Check statusReg word for Errors?
    */

    *return_code = result;

    PT_END(pt);
}

PT_THREAD( LEP_I2C_SetAttribute_PT(struct pt *pt,
                                   LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                   LEP_COMMAND_ID commandID, 
                                   LEP_ATTRIBUTE_T_PTR attributePtr,
                                   LEP_UINT16 attributeWordLength,
                                   LEP_RESULT *return_code))
{
    static LEP_RESULT result;
    static LEP_UINT16 statusReg;
    static LEP_INT16 statusCode;
    static LEP_UINT32 done;
    static LEP_UINT16 timeoutCount;

    PT_BEGIN(pt);

    timeoutCount = LEPTON_I2C_COMMAND_BUSY_WAIT_COUNT;

    /* Implement the Lepton TWI WRITE Protocol
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
        /* Add timout check */
        if( timeoutCount-- == 0 )
        {
            /* Timed out waiting for command busy to go away
            */ 
          *return_code = LEP_TIMEOUT_ERROR;
          PT_EXIT(pt);
        }

        PT_YIELD(pt);
    }while( !done );

    if( result == LEP_OK )
    {
        /* Now WRITE the DATA to the DATA REGISTER(s)
        */ 
        if( attributeWordLength <= 16 )
        {
            /* WRITE to the DATA Registers - always start from DATA 0
            */ 
            result = LEP_I2C_MasterWriteData(portDescPtr->portID,
                                             portDescPtr->deviceAddress,
                                             LEP_I2C_DATA_0_REG,
                                             attributePtr,
                                             attributeWordLength );
        }
        else if( attributeWordLength <= 1024 )
        {
            /* WRITE to the DATA Block Buffer
            */     
            result = LEP_I2C_MasterWriteData(portDescPtr->portID,
                                             portDescPtr->deviceAddress,
                                             LEP_I2C_DATA_BUFFER_0,
                                             attributePtr,
                                             attributeWordLength );

        }
        else
            result = LEP_RANGE_ERROR;
    }

    PT_YIELD(pt);

    if( result == LEP_OK )
    {
        /* Set the Lepton's DATA LENGTH REGISTER first to inform the
        ** Lepton Camera how many 16-bit DATA words we want to read.
        */ 
        result = LEP_I2C_MasterWriteData( portDescPtr->portID,
                                          portDescPtr->deviceAddress,
                                          LEP_I2C_DATA_LENGTH_REG, 
                                          &attributeWordLength, 
                                          1);

        PT_YIELD(pt);

        if( result == LEP_OK )
        {
            /* Now issue the SET Attribute Command
            */ 
            result = LEP_I2C_MasterWriteData( portDescPtr->portID,
                                              portDescPtr->deviceAddress,
                                              LEP_I2C_COMMAND_REG, 
                                              &commandID, 
                                              1);

            PT_YIELD(pt);

            if( result == LEP_OK )
            {
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

            }
        }
    }

    /* Check statusReg word for Errors?
    */

    *return_code = result;

    PT_END(pt);
}

union custom_uvc custom_uvc = {0};

PT_THREAD( lepton_attribute_xfer_task(struct pt *pt))
{
  static struct uvc_request req;
  static LEP_RESULT result;
  static uint16_t module_base;
  static struct pt lep_pt;
  static struct pt wait_pt;
  static int retries;
  static int cust_response_length;
  static struct custom_response *response;

  PT_BEGIN(pt);

  while (1)
  {
    PT_WAIT_UNTIL(pt, dequeue_attribute_xfer_task(&req) == HAL_OK);

    if (req.length < 1 || req.length > MAX_I2C_BUFFER_SIZE)
      continue;

	if (req.entity_id == VC_CONTROL_XU_LEP_CUST_ID) {
		req.control_id >>= 2;
		if (req.control_id == CUST_CONTROL_COMMAND) {
			result = LEP_OK;
			if (req.type == UVC_REQUEST_TYPE_ATTR_GET) {
				req.length = sizeof(custom_uvc);
				memcpy(req.buffer, &custom_uvc, sizeof(custom_uvc));

				HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

				if (result == LEP_OK)
					USBD_CtlSendData(hUsbDevice_0, req.buffer, req.length);
				else
					USBD_CtlError(hUsbDevice_0, 0);

				HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

			} else if (req.type == UVC_REQUEST_TYPE_ATTR_SET) {
				memcpy(&custom_uvc, req.buffer,
						req.length > sizeof(custom_uvc) ?
								sizeof(custom_uvc) : req.length);
				HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

				if (result == LEP_OK)
					USBD_CtlSendStatus(hUsbDevice_0);
				else
					USBD_CtlError(hUsbDevice_0, 0);

				HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

			}
		} else {
			if (req.type == UVC_REQUEST_TYPE_ATTR_SET) {
				HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
				USBD_CtlError(hUsbDevice_0, 0);
				HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
			} else if (req.type == UVC_REQUEST_TYPE_ATTR_GET) {
		        PT_INIT(&lep_pt);

		        response = (struct custom_response *)(req.buffer);

		        if (req.control_id == CUST_CONTROL_GET) {
			        PT_WAIT_THREAD(pt, LEP_I2C_GetAttribute_PT(&lep_pt, &hport_desc,
			                                                   custom_uvc.command.id | LEP_GET_TYPE,
			                                                   ( LEP_ATTRIBUTE_T_PTR )response->data,
			                                                   custom_uvc.command.length >> 1,
			                                                   &result));
			        cust_response_length = sizeof(struct custom_response);
		        }
		        if (req.control_id == CUST_CONTROL_SET) {
			        PT_WAIT_THREAD(pt, LEP_I2C_SetAttribute_PT(&lep_pt, &hport_desc,
			                                                   custom_uvc.command.id | LEP_SET_TYPE,
			                                                   ( LEP_ATTRIBUTE_T_PTR ) custom_uvc.command.buffer,
															   custom_uvc.command.length >> 1,
			                                                   &result));
			        cust_response_length = sizeof(LEP_RESULT);
		        }
		        if (req.control_id == CUST_CONTROL_RUN) {
			        PT_WAIT_THREAD(pt, LEP_I2C_RunCommand_PT(&lep_pt, &hport_desc,
			                                                 custom_uvc.command.id | LEP_RUN_TYPE,
			                                                 &result));
			        cust_response_length = sizeof(LEP_RESULT);
		        }
		        if (req.control_id == CUST_CONTROL_DIRECT_READ) {
		            PT_WAIT_THREAD(pt, LEP_I2C_WaitForBusyBit(&wait_pt, &hport_desc,
		                                                      &result, NULL));

		            if( result == LEP_OK )
		            {
						result = LEP_I2C_MasterReadData( hport_desc.portID,
															 hport_desc.deviceAddress,
															 custom_uvc.direct.address,
															 (LEP_UINT16 *)response->data,
															 custom_uvc.direct.length >> 1);

		                PT_YIELD(pt);
		            }
			        cust_response_length = sizeof(struct custom_response);
		        }
		        if (req.control_id == CUST_CONTROL_DIRECT_WRITE) {
		            PT_WAIT_THREAD(pt, LEP_I2C_WaitForBusyBit(&wait_pt, &hport_desc,
		                                                      &result, NULL));

		            if( result == LEP_OK )
		            {
						result = LEP_I2C_MasterWriteData( hport_desc.portID,
													     hport_desc.deviceAddress,
														 custom_uvc.direct.address,
														 custom_uvc.direct.data,
														 custom_uvc.direct.length >> 1);

		                PT_YIELD(pt);
		            }
			        cust_response_length = sizeof(LEP_RESULT);
		        }

		        response->result = result;

				HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
				USBD_CtlSendData(hUsbDevice_0, req.buffer, cust_response_length);
				HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
			}
		}
		continue;
	}

    module_base = vc_terminal_id_to_module_base(req.entity_id);

    if (req.type == UVC_REQUEST_TYPE_ATTR_GET)
    {
      // If reading from the OTP (serial number, part number, etc.), we
      // sometimes get back a -16 or -17 (correctable bit errors), so just try
      // one extra time regardless of what the problem was
      retries = 1;

      do {
        PT_INIT(&lep_pt);

        PT_WAIT_THREAD(pt, LEP_I2C_GetAttribute_PT(&lep_pt, &hport_desc,
                                                   ( LEP_COMMAND_ID )(module_base + req.control_id) | LEP_GET_TYPE,
                                                   ( LEP_ATTRIBUTE_T_PTR )req.buffer,
                                                   req.length >> 1,
                                                   &result));

        PT_YIELD(pt);
      } while (result != LEP_OK && retries--);

      HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

      if (result == LEP_OK)
        USBD_CtlSendData(hUsbDevice_0, req.buffer, req.length);
      else
        USBD_CtlError(hUsbDevice_0, 0);

      HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    }
    else if (req.type == UVC_REQUEST_TYPE_ATTR_SET)
    {
      if (req.length == 1)
      {
        if ((module_base + req.control_id) == (FLR_CID_SYS_RUN_FFC & 0xfffc))
          req.control_id = FLR_CID_SYS_RUN_FFC - module_base;

        PT_INIT(&lep_pt);

        PT_WAIT_THREAD(pt, LEP_I2C_RunCommand_PT(&lep_pt, &hport_desc,
                                                 ( LEP_COMMAND_ID )(module_base + req.control_id) | LEP_RUN_TYPE,
                                                 &result));
      }
      else
      {

        PT_INIT(&lep_pt);

        PT_WAIT_THREAD(pt, LEP_I2C_SetAttribute_PT(&lep_pt, &hport_desc,
                                                   ( LEP_COMMAND_ID )(module_base + req.control_id) | LEP_SET_TYPE,
                                                   ( LEP_ATTRIBUTE_T_PTR )req.buffer,
                                                   req.length >> 1,
                                                   &result));
      }

      HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

      if (result == LEP_OK)
        USBD_CtlSendStatus(hUsbDevice_0);
      else
        USBD_CtlError(hUsbDevice_0, 0);

      HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    }
  }

  PT_END(pt);
}
