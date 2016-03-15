#include "usbd_uvc.h"

#include "lepton_i2c.h"

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_VID.h"
#include "LEPTON_OEM.h"
#include "LEPTON_RAD.h"

#include <limits.h>

extern LEP_CAMERA_PORT_DESC_T hport_desc;

static uint16_t vc_terminal_id_to_module_base(VC_TERMINAL_ID entity_id)
{
  switch(entity_id)
  {
  case VC_CONTROL_XU_LEP_AGC_ID:
    return LEP_AGC_MODULE_BASE;
  case VC_CONTROL_XU_LEP_OEM_ID:
    return LEP_OEM_MODULE_BASE + 0x4000; // need OEM bit set for this module
  case VC_CONTROL_XU_LEP_RAD_ID:
    return LEP_RAD_MODULE_BASE;
  case VC_CONTROL_XU_LEP_SYS_ID:
    return LEP_SYS_MODULE_BASE;
  case VC_CONTROL_XU_LEP_VID_ID:
    return LEP_VID_MODULE_BASE;
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

int8_t VC_LEP_RunCommand (VC_TERMINAL_ID entity_id, uint16_t offset)
{
  uint16_t module_base = vc_terminal_id_to_module_base(entity_id);

  if ((module_base + offset) == (FLR_CID_SYS_RUN_FFC & 0xfffc))
    offset = FLR_CID_SYS_RUN_FFC - module_base;

  LEP_RESULT result = LEP_RunCommand(&hport_desc,
                                      ( LEP_COMMAND_ID )(module_base + offset));
  return result;
}

static int8_t getAttributeLen_AGC(uint16_t module_register, uint16_t *pbuf)
{
  switch (module_register)
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

static int8_t getAttributeLen_OEM(uint16_t module_register, uint16_t *pbuf)
{
  switch (module_register)
  {
  case LEP_CID_OEM_FLIR_PART_NUMBER:
  case LEP_CID_OEM_CUST_PART_NUMBER:
    *pbuf = 32;
    break;
  case LEP_CID_OEM_SOFTWARE_VERSION:
    *pbuf = 8;
    break;
  case LEP_CID_OEM_VIDEO_OUTPUT_ENABLE:
  case LEP_CID_OEM_VIDEO_OUTPUT_FORMAT:
  case LEP_CID_OEM_VIDEO_OUTPUT_SOURCE:
  case LEP_CID_OEM_VIDEO_OUTPUT_CHANNEL:
  case LEP_CID_OEM_VIDEO_GAMMA_ENABLE:
  case LEP_CID_OEM_STATUS:
  case LEP_CID_OEM_POWER_MODE:
  case LEP_CID_OEM_GPIO_MODE_SELECT:
  case LEP_CID_OEM_GPIO_VSYNC_PHASE_DELAY:
  case LEP_CID_OEM_USER_DEFAULTS: // also a command
  case LEP_CID_OEM_THERMAL_SHUTDOWN_ENABLE_STATE:
  case LEP_CID_OEM_SHUTTER_PROFILE_OBJ:
  case LEP_CID_OEM_BAD_PIXEL_REPLACE_CONTROL:
  case LEP_CID_OEM_TEMPORAL_FILTER_CONTROL:
  case LEP_CID_OEM_COLUMN_NOISE_ESTIMATE_CONTROL:
  case LEP_CID_OEM_PIXEL_NOISE_ESTIMATE_CONTROL:
    *pbuf = 4;
    break;
  case LEP_CID_OEM_POWER_DOWN:
  case LEP_CID_OEM_STANDBY:
  case LEP_CID_OEM_LOW_POWER_MODE_1:
  case LEP_CID_OEM_LOW_POWER_MODE_2:
  case LEP_CID_OEM_BIT_TEST:
  case LEP_CID_OEM_REBOOT:
  case LEP_CID_OEM_USER_DEFAULTS_RESTORE:
    *pbuf = 1;
    break;
  default:
    *pbuf = 2;
    break;
  }
  return 0;
}

static int8_t getAttributeLen_RAD(uint16_t module_register, uint16_t *pbuf)
{
  switch (module_register)
  {
  case LEP_CID_RAD_TFPA_LUT:
  case LEP_CID_RAD_TAUX_LUT:
    *pbuf = 512;
    break;
  case LEP_CID_RAD_RESPONSIVITY_VALUE_LUT:
  case LEP_CID_RAD_TEQ_SHUTTER_LUT:
  case LEP_CID_RAD_MLG_LUT:
    *pbuf = 256;
    break;
  case LEP_CID_RAD_RBFO_INTERNAL:
  case LEP_CID_RAD_RBFO_EXTERNAL:
  case LEP_CID_RAD_RBFO_INTERNAL_LG:
  case LEP_CID_RAD_RBFO_EXTERNAL_LG:
  case LEP_CID_RAD_FLUX_LINEAR_PARAMS:
    *pbuf = 16;
    break;
  case LEP_CID_AGC_ROI:
  case LEP_CID_AGC_STATISTICS:
  case LEP_CID_RAD_THOUSING_TCP:
  case LEP_CID_RAD_SHUTTER_TCP:
  case LEP_CID_RAD_LENS_TCP:
  case LEP_CID_RAD_SPOTMETER_ROI:
    *pbuf = 8;
    break;
  case LEP_CID_RAD_TSHUTTER_MODE:
  case LEP_CID_RAD_DEBUG_FLUX:
  case LEP_CID_RAD_ENABLE_STATE:
  case LEP_CID_RAD_TFPA_CTS_MODE:
  case LEP_CID_RAD_TAUX_CTS_MODE:
  case LEP_CID_RAD_RUN_STATUS:
  case LEP_CID_RAD_TEQ_SHUTTER_FLUX:
  case LEP_CID_RAD_MFFC_FLUX:
  case LEP_CID_RAD_TLINEAR_ENABLE_STATE:
  case LEP_CID_RAD_TLINEAR_RESOLUTION:
  case LEP_CID_RAD_TLINEAR_AUTO_RESOLUTION:
  case LEP_CID_RAD_ARBITRARY_OFFSET_MODE:
  case LEP_CID_RAD_ARBITRARY_OFFSET_PARAMS:
    *pbuf = 4;
    break;
  case LEP_CID_RAD_RUN_FFC:
    *pbuf = 1;
    break;
  default:
    *pbuf = 2;
    break;
  }
  return 0;
}

static int8_t getAttributeLen_SYS(uint16_t module_register, uint16_t *pbuf)
{
  switch (module_register)
  {
  case LEP_CID_SYS_CUST_SERIAL_NUMBER:
  case LEP_CID_SYS_FFC_SHUTTER_MODE_OBJ:
    *pbuf = 32;
    break;
  case LEP_CID_SYS_CAM_STATUS:
  case LEP_CID_SYS_FLIR_SERIAL_NUMBER:
  case LEP_CID_SYS_SCENE_STATISTICS:
  case LEP_CID_SYS_SCENE_ROI:
    *pbuf = 8;
    break;
  case LEP_CID_SYS_CAM_UPTIME:
  case LEP_CID_SYS_TELEMETRY_ENABLE_STATE:
  case LEP_CID_SYS_TELEMETRY_LOCATION:
  case LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE:
  case LEP_CID_SYS_SHUTTER_POSITION:
  case LEP_CID_SYS_FFC_STATUS:
    *pbuf = 4;
    break;
  case LEP_CID_SYS_PING:
  case LEP_CID_SYS_EXECTUE_FRAME_AVERAGE:
  case (FLR_CID_SYS_RUN_FFC & 0xfffc):
    *pbuf = 1;
    break;
  default:
    *pbuf = 2;
    break;
  }
  return 0;
}

static int8_t getAttributeLen_VID(uint16_t module_register, uint16_t *pbuf)
{
  switch (module_register)
  {
  case LEP_CID_VID_LUT_TRANSFER:
    *pbuf = 1024;
    break;
  case LEP_CID_VID_FOCUS_ROI:
    *pbuf = 8;
    break;
  case LEP_CID_VID_POLARITY_SELECT:
  case LEP_CID_VID_LUT_SELECT:
  case LEP_CID_VID_FOCUS_CALC_ENABLE:
  case LEP_CID_VID_FOCUS_METRIC:
  case LEP_CID_VID_FOCUS_THRESHOLD:
  case LEP_CID_VID_SBNUC_ENABLE:
  case LEP_CID_VID_FREEZE_ENABLE:
    *pbuf = 4;
    break;
  default:
    *pbuf = 2;
    break;
  }
  return 0;
}


int8_t VC_LEP_GetAttributeLen (VC_TERMINAL_ID entity_id, uint16_t offset, uint16_t* pbuf)
{
  uint16_t module_register =
      vc_terminal_id_to_module_base(entity_id) + offset;

  switch (entity_id)
  {
  case VC_CONTROL_XU_LEP_AGC_ID:
    return getAttributeLen_AGC(module_register, pbuf);
  case VC_CONTROL_XU_LEP_OEM_ID:
    return getAttributeLen_OEM(module_register, pbuf);
  case VC_CONTROL_XU_LEP_RAD_ID:
    return getAttributeLen_RAD(module_register, pbuf);
  case VC_CONTROL_XU_LEP_SYS_ID:
    return getAttributeLen_SYS(module_register, pbuf);
  case VC_CONTROL_XU_LEP_VID_ID:
    return getAttributeLen_VID(module_register, pbuf);
  default:
    return -1;
  }
}


static int8_t getMaxValue_AGC(uint16_t module_register, void *pbuf, uint16_t len)
{
  switch (module_register)
  {
#if 0
  case LEP_CID_AGC_ROI:
  case LEP_CID_AGC_STATISTICS:
    *((int64_t*)pbuf) = LLONG_MAX;
    break;
#else
  case LEP_CID_AGC_ROI:
  case LEP_CID_AGC_STATISTICS:
    memset(pbuf, 0xff, len);
    break;
#endif
  case LEP_CID_AGC_CALC_ENABLE_STATE:
  case LEP_CID_AGC_ENABLE_STATE:
    *((LEP_AGC_ENABLE_E_PTR)pbuf) = LEP_END_AGC_ENABLE - 1;
    break;
  case LEP_CID_AGC_POLICY:
    *((LEP_AGC_POLICY_E_PTR)pbuf) = LEP_END_AGC_POLICY - 1;
    break;
  case LEP_CID_AGC_HEQ_SCALE_FACTOR:
    *((LEP_AGC_HEQ_SCALE_FACTOR_E_PTR)pbuf) = LEP_AGC_END_SCALE_TO - 1;
    break;
  default:
    *((uint16_t*)pbuf) = USHRT_MAX;
    break;
  }
  return 0;
}

static int8_t getMaxValue_OEM(uint16_t module_register, void *pbuf, uint16_t len)
{
  switch (module_register)
  {
#if 0
  case LEP_CID_OEM_FLIR_PART_NUMBER:
  case LEP_CID_OEM_CUST_PART_NUMBER:
    *pbuf = 32;
    break;
#else
  case LEP_CID_OEM_FLIR_PART_NUMBER:
  case LEP_CID_OEM_CUST_PART_NUMBER:
    memset(pbuf, 0xff, len);
    break;
#endif
  case LEP_CID_OEM_SOFTWARE_VERSION:
    *((LEP_OEM_SW_VERSION_T_PTR)pbuf) = (LEP_OEM_SW_VERSION_T) {
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00
    };
    break;
  case LEP_CID_OEM_VIDEO_OUTPUT_ENABLE:
    *((LEP_OEM_VIDEO_OUTPUT_ENABLE_E_PTR)pbuf) = LEP_END_VIDEO_OUTPUT_ENABLE - 1;
    break;
  case LEP_CID_OEM_VIDEO_OUTPUT_FORMAT:
    *((LEP_OEM_VIDEO_OUTPUT_FORMAT_E_PTR)pbuf) = LEP_END_VIDEO_OUTPUT_FORMAT - 1;
    break;
  case LEP_CID_OEM_VIDEO_OUTPUT_SOURCE:
    *((LEP_OEM_VIDEO_OUTPUT_SOURCE_E_PTR)pbuf) = LEP_END_VIDEO_OUTPUT_SOURCE - 1;
    break;
  case LEP_CID_OEM_VIDEO_OUTPUT_CHANNEL:
    *((LEP_OEM_VIDEO_OUTPUT_CHANNEL_E_PTR)pbuf) = LEP_END_VIDEO_OUTPUT_CHANNEL - 1;
    break;
  case LEP_CID_OEM_VIDEO_GAMMA_ENABLE:
    *((LEP_OEM_VIDEO_GAMMA_ENABLE_E_PTR)pbuf) = LEP_END_VIDEO_GAMMA_ENABLE - 1;
    break;
  case LEP_CID_OEM_STATUS:
    *((LEP_OEM_STATUS_E_PTR)pbuf) = LEP_OEM_STATUS_END - 1;
    break;
  case LEP_CID_OEM_POWER_MODE:
    *((LEP_OEM_POWER_STATE_E_PTR)pbuf) = LEP_OEM_END_POWER_MODE - 1;
    break;
  case LEP_CID_OEM_GPIO_MODE_SELECT:
    *((LEP_OEM_GPIO_MODE_E_PTR)pbuf) = LEP_OEM_END_GPIO_MODE - 1;
    break;
  case LEP_CID_OEM_GPIO_VSYNC_PHASE_DELAY: // TODO: handle min
    *((LEP_OEM_VSYNC_DELAY_E_PTR)pbuf) = LEP_END_OEM_VSYNC_DELAY - 1;
    break;
  case LEP_CID_OEM_USER_DEFAULTS: // also a command
    *((LEP_OEM_USER_PARAMS_STATE_E_PTR)pbuf) = LEP_OEM_END_USER_PARAMS_STATE - 1;
    break;
  case LEP_CID_OEM_THERMAL_SHUTDOWN_ENABLE_STATE:
  case LEP_CID_OEM_BAD_PIXEL_REPLACE_CONTROL:
  case LEP_CID_OEM_TEMPORAL_FILTER_CONTROL:
  case LEP_CID_OEM_COLUMN_NOISE_ESTIMATE_CONTROL:
  case LEP_CID_OEM_PIXEL_NOISE_ESTIMATE_CONTROL:
    *((LEP_OEM_STATE_E_PTR)pbuf) = LEP_OEM_END_STATE - 1;
    break;
  case LEP_CID_OEM_SHUTTER_PROFILE_OBJ:
    *((LEP_OEM_SHUTTER_PROFILE_OBJ_T_PTR)pbuf) = (LEP_OEM_SHUTTER_PROFILE_OBJ_T) {
      0xffff, 0xffff
    };
    break;
  case LEP_CID_OEM_POWER_DOWN:
  case LEP_CID_OEM_STANDBY:
  case LEP_CID_OEM_LOW_POWER_MODE_1:
  case LEP_CID_OEM_LOW_POWER_MODE_2:
  case LEP_CID_OEM_BIT_TEST:
  case LEP_CID_OEM_REBOOT:
  case LEP_CID_OEM_USER_DEFAULTS_RESTORE:
    *((uint8_t*)pbuf) = 1;
    break;
  default:
    *((uint16_t*)pbuf) = USHRT_MAX;
    break;
  }
  return 0;
}

static int8_t getMaxValue_RAD(uint16_t module_register, void *pbuf, uint16_t len)
{
  switch (module_register)
  {
#if 0
  case LEP_CID_RAD_TFPA_LUT:
  case LEP_CID_RAD_TAUX_LUT:
    *pbuf = 512;
    break;
  case LEP_CID_RAD_RESPONSIVITY_VALUE_LUT:
  case LEP_CID_RAD_TEQ_SHUTTER_LUT:
  case LEP_CID_RAD_MLG_LUT:
    *pbuf = 256;
    break;
  case LEP_CID_RAD_RBFO_INTERNAL:
  case LEP_CID_RAD_RBFO_EXTERNAL:
  case LEP_CID_RAD_RBFO_INTERNAL_LG:
  case LEP_CID_RAD_RBFO_EXTERNAL_LG:
  case LEP_CID_RAD_FLUX_LINEAR_PARAMS:
    *pbuf = 16;
    break;
  case LEP_CID_AGC_ROI:
  case LEP_CID_AGC_STATISTICS:
  case LEP_CID_RAD_THOUSING_TCP:
  case LEP_CID_RAD_SHUTTER_TCP:
  case LEP_CID_RAD_LENS_TCP:
  case LEP_CID_RAD_SPOTMETER_ROI:
    *pbuf = 8;
    break;
#else
  case LEP_CID_RAD_TFPA_LUT:
  case LEP_CID_RAD_TAUX_LUT:
  case LEP_CID_RAD_RESPONSIVITY_VALUE_LUT:
  case LEP_CID_RAD_TEQ_SHUTTER_LUT:
  case LEP_CID_RAD_MLG_LUT:
  case LEP_CID_RAD_RBFO_INTERNAL:
  case LEP_CID_RAD_RBFO_EXTERNAL:
  case LEP_CID_RAD_RBFO_INTERNAL_LG:
  case LEP_CID_RAD_RBFO_EXTERNAL_LG:
  case LEP_CID_RAD_FLUX_LINEAR_PARAMS:
  case LEP_CID_AGC_ROI:
  case LEP_CID_AGC_STATISTICS:
  case LEP_CID_RAD_THOUSING_TCP:
  case LEP_CID_RAD_SHUTTER_TCP:
  case LEP_CID_RAD_LENS_TCP:
  case LEP_CID_RAD_SPOTMETER_ROI:
    memset(pbuf, 0xff, len);
    break;
#endif
  case LEP_CID_RAD_TSHUTTER_MODE:
    *((LEP_RAD_TS_MODE_E_PTR)pbuf) = LEP_RAD_TS_END_TS_MODE - 1;
    break;
  case LEP_CID_RAD_DEBUG_FLUX: // TODO: handle min
  case LEP_CID_RAD_TEQ_SHUTTER_FLUX:
  case LEP_CID_RAD_MFFC_FLUX:
    *((LEP_INT32*)pbuf) = LONG_MAX;
    break;
  case LEP_CID_RAD_ENABLE_STATE:
  case LEP_CID_RAD_TLINEAR_ENABLE_STATE:
  case LEP_CID_RAD_TLINEAR_AUTO_RESOLUTION:
    *((LEP_RAD_ENABLE_E_PTR)pbuf) = LEP_END_RAD_ENABLE - 1;
    break;
  case LEP_CID_RAD_TFPA_CTS_MODE:
  case LEP_CID_RAD_TAUX_CTS_MODE:
    *((LEP_RAD_TEMPERATURE_UPDATE_E_PTR)pbuf) = LEP_RAD_UPDATE_END - 1;
    break;
  case LEP_CID_RAD_RUN_STATUS:
    *((LEP_RAD_STATUS_E_PTR)pbuf) = LEP_RAD_STATUS_END - 1;
    break;
  case LEP_CID_RAD_TLINEAR_RESOLUTION:
    *((LEP_RAD_TLINEAR_RESOLUTION_E_PTR)pbuf) = LEP_RAD_END_RESOLUTION - 1;
    break;
  case LEP_CID_RAD_ARBITRARY_OFFSET_MODE:
    *((LEP_RAD_ARBITRARY_OFFSET_MODE_E_PTR)pbuf) = LEP_RAD_END_ARBITRARY_OFFSET_MODE - 1;
    break;
  case LEP_CID_RAD_ARBITRARY_OFFSET_PARAMS:
    *((LEP_RAD_ARBITRARY_OFFSET_PARAMS_T_PTR)pbuf) = (LEP_RAD_ARBITRARY_OFFSET_PARAMS_T) {
      0xffff, 0xffff
    };
    break;
  case LEP_CID_RAD_RUN_FFC:
    *((uint8_t*)pbuf) = 1;
    break;
  default:
    *((uint16_t*)pbuf) = USHRT_MAX;
    break;
  }
  return 0;
}

static int8_t getMaxValue_SYS(uint16_t module_register, void *pbuf, uint16_t len)
{
  switch (module_register)
  {
#if 0
  case LEP_CID_SYS_CUST_SERIAL_NUMBER:
  case LEP_CID_SYS_FFC_SHUTTER_MODE_OBJ:
    *pbuf = 32;
    break;
  case LEP_CID_SYS_SCENE_STATISTICS:
  case LEP_CID_SYS_SCENE_ROI:
    *pbuf = 8;
    break;
#else
  case LEP_CID_SYS_CUST_SERIAL_NUMBER:
  case LEP_CID_SYS_FFC_SHUTTER_MODE_OBJ:
  case LEP_CID_SYS_SCENE_STATISTICS:
  case LEP_CID_SYS_SCENE_ROI:
    memset(pbuf, 0xff, len);
    break;
#endif
  case LEP_CID_SYS_CAM_STATUS:
  case LEP_CID_SYS_FLIR_SERIAL_NUMBER:
    *((int64_t*)pbuf) = LLONG_MAX;
    break;
  case LEP_CID_SYS_CAM_UPTIME:
    *((LEP_SYS_UPTIME_NUMBER_T_PTR)pbuf) = ULONG_MAX;
    break;
  case LEP_CID_SYS_TELEMETRY_ENABLE_STATE:
    *((LEP_SYS_TELEMETRY_ENABLE_STATE_E_PTR)pbuf) = LEP_END_TELEMETRY_ENABLE_STATE - 1;
    break;
  case LEP_CID_SYS_TELEMETRY_LOCATION:
    *((LEP_SYS_TELEMETRY_LOCATION_E_PTR)pbuf) = LEP_END_TELEMETRY_LOCATION - 1;
    break;
  case LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE:
    *((LEP_SYS_FRAME_AVERAGE_DIVISOR_E_PTR)pbuf) = LEP_SYS_END_FA_DIV - 1;
    break;
  case LEP_CID_SYS_SHUTTER_POSITION:
    *((LEP_SYS_SHUTTER_POSITION_E_PTR)pbuf) = LEP_SYS_SHUTTER_POSITION_END - 1;
    break;
  case LEP_CID_SYS_FFC_STATUS: // TODO: handle min
    *((LEP_SYS_STATUS_E_PTR)pbuf) = LEP_SYS_STATUS_END -1;
    break;
  case LEP_CID_SYS_PING:
  case LEP_CID_SYS_EXECTUE_FRAME_AVERAGE:
  case (FLR_CID_SYS_RUN_FFC & 0xfffc):
    *((uint8_t*)pbuf) = 1;
    break;
  default:
    *((uint16_t*)pbuf) = USHRT_MAX;
    break;
  }
  return 0;
}

static int8_t getMaxValue_VID(uint16_t module_register, void *pbuf, uint16_t len)
{
  switch (module_register)
  {
#if 0
  case LEP_CID_VID_LUT_TRANSFER:
    *pbuf = 1024;
    break;
  case LEP_CID_VID_FOCUS_ROI:
    *pbuf = 8;
    break;
#else
  case LEP_CID_VID_LUT_TRANSFER:
  case LEP_CID_VID_FOCUS_ROI:
    memset(pbuf, 0xff, len);
    break;
#endif
  case LEP_CID_VID_POLARITY_SELECT:
    *((LEP_POLARITY_E_PTR)pbuf) = LEP_VID_END_POLARITY - 1;
    break;
  case LEP_CID_VID_LUT_SELECT:
    *((LEP_PCOLOR_LUT_E_PTR)pbuf) = LEP_VID_END_PCOLOR_LUT - 1;
    break;
  case LEP_CID_VID_FOCUS_CALC_ENABLE:
    *((LEP_VID_FOCUS_CALC_ENABLE_E_PTR)pbuf) = LEP_VID_END_FOCUS_CALC_ENABLE - 1;
    break;
  case LEP_CID_VID_FOCUS_METRIC:
    *((LEP_VID_FOCUS_METRIC_T_PTR)pbuf) = ULONG_MAX;
    break;
  case LEP_CID_VID_FOCUS_THRESHOLD:
    *((LEP_VID_FOCUS_METRIC_THRESHOLD_T_PTR)pbuf) = ULONG_MAX;
    break;
  case LEP_CID_VID_SBNUC_ENABLE:
    *((LEP_VID_SBNUC_ENABLE_E_PTR)pbuf) = LEP_VID_END_SBNUC_ENABLE - 1;
    break;
  case LEP_CID_VID_FREEZE_ENABLE:
    *((LEP_VID_FREEZE_ENABLE_E_PTR)pbuf) = LEP_VID_END_FREEZE_ENABLE - 1;
    break;
  default:
    *((uint16_t*)pbuf) = USHRT_MAX;
    break;
  }
  return 0;
}


int8_t VC_LEP_GetMaxValue (VC_TERMINAL_ID entity_id, uint16_t offset, void* pbuf, uint16_t len)
{
  uint16_t module_register =
      vc_terminal_id_to_module_base(entity_id) + offset;

  switch (entity_id)
  {
  case VC_CONTROL_XU_LEP_AGC_ID:
    return getMaxValue_AGC(module_register, pbuf, len);
  case VC_CONTROL_XU_LEP_OEM_ID:
    return getMaxValue_OEM(module_register, pbuf, len);
  case VC_CONTROL_XU_LEP_RAD_ID:
    return getMaxValue_RAD(module_register, pbuf, len);
  case VC_CONTROL_XU_LEP_SYS_ID:
    return getMaxValue_SYS(module_register, pbuf, len);
  case VC_CONTROL_XU_LEP_VID_ID:
    return getMaxValue_VID(module_register, pbuf, len);
  default:
    return -1;
  }
}
