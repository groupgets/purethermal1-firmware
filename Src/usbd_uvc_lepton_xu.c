#include "usbd_uvc.h"

#include "lepton_i2c.h"

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_VID.h"
#include "LEPTON_OEM.h"

extern LEP_CAMERA_PORT_DESC_T hport_desc;

static uint16_t vc_terminal_id_to_module_base(VC_TERMINAL_ID entity_id)
{
  switch(entity_id)
  {
  case VC_CONTROL_XU_LEP_AGC_ID:
    return LEP_AGC_MODULE_BASE;
  case VC_CONTROL_XU_LEP_OEM_ID:
  case VC_CONTROL_XU_LEP_RAD_ID:
  case VC_CONTROL_XU_LEP_SYS_ID:
  case VC_CONTROL_XU_LEP_VID_ID:
  default:
    return 0;
  }
}

int8_t VC_LEP_GetAttribute (VC_TERMINAL_ID entity_id, uint16_t offset, uint8_t* pbuf, uint16_t length)
{
  uint16_t module_base = vc_terminal_id_to_module_base(entity_id);

  LEP_RESULT result = LEP_GetAttribute(&hport_desc,
                                      ( LEP_COMMAND_ID )(module_base + offset),
                                      ( LEP_ATTRIBUTE_T_PTR )pbuf,
                                      length >> 1);
  return result;
}

int8_t VC_LEP_SetAttribute (VC_TERMINAL_ID entity_id, uint16_t offset, uint8_t* pbuf, uint16_t length)
{
  uint16_t module_base = vc_terminal_id_to_module_base(entity_id);

  LEP_RESULT result = LEP_SetAttribute(&hport_desc,
                                      ( LEP_COMMAND_ID )(module_base + offset),
                                      ( LEP_ATTRIBUTE_T_PTR )pbuf,
                                      length >> 1);
  return result;
}

int8_t VC_LEP_GetAttributeLen (VC_TERMINAL_ID entity_id, uint16_t offset, uint8_t* pbuf)
{
  switch (vc_terminal_id_to_module_base(entity_id) + offset)
  {
  case LEP_CID_AGC_ROI:
  case LEP_CID_AGC_STATISTICS:
    *pbuf = 8;
    break;
  case LEP_CID_AGC_ENABLE_STATE:
  case LEP_CID_AGC_POLICY:
  case LEP_CID_AGC_HEQ_SCALE_FACTOR:
  case LEP_CID_AGC_CALC_ENABLE_STATE:
    *pbuf = 4;
    break;
  default:
    *pbuf = 2;
    break;
  }
  return 0;
}
