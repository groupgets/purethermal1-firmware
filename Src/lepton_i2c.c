#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "lepton.h"
#include "lepton_i2c.h"
#include "project_config.h"

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_VID.h"
#include "LEPTON_OEM.h"
#include "LEPTON_RAD.h"

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

// HAL_OK       = 0x00,
//  HAL_ERROR    = 0x01,
//  HAL_BUSY     = 0x02,
//  HAL_TIMEOUT  = 0x03

extern I2C_HandleTypeDef hi2c1;
LEP_CAMERA_PORT_DESC_T hport_desc;

extern volatile uint8_t g_lepton_type_3;

static void set_lepton_type()
{
  LEP_RESULT result;
  LEP_OEM_PART_NUMBER_T part_number;
  result = LEP_GetOemFlirPartNumber(&hport_desc, &part_number);

  // 500-0643-00 : 50 deg (l2)
  // 500-0659-01 : shuttered 50 deg (l2)
  // 500-0690-00 : 25 deg (l2)
  // 500-0763-01 : shuttered 50 deg + radiometric (l2.5)
  // 500-0726-01 : shuttered 50 deg (l3)

  if (result != LEP_OK ||
      strncmp(part_number.value, "500-06xx", 6) == 0 ||
      strncmp(part_number.value, "500-0763", 8) == 0)
  {
    g_lepton_type_3 = 0;
  }
  else
  {
    // let default case be l3, because this will be more likely to cover new products
    g_lepton_type_3 = 1;
  }

  LEP_SetOemGpioVsyncPhaseDelay(&hport_desc,LEP_OEM_VSYNC_DELAY_PLUS_2);
  LEP_SetOemGpioMode(&hport_desc, LEP_OEM_GPIO_MODE_VSYNC);
}

static void set_startup_defaults()
{
  LEP_RESULT result;

  /* set a default color lut so radiometric parts produce reasonable pseudocolor images */
  result = LEP_SetVidPcolorLut(&hport_desc, PSUEDOCOLOR_LUT);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set default color lut: %d\r\n", result);
  }
}

static HAL_StatusTypeDef print_cust_serial_number()
{
  LEP_RESULT result;
  LEP_SYS_CUST_SERIAL_NUMBER_T cust_serial_number;
  int i;

  result = LEP_GetSysCustSerialNumber(&hport_desc, &cust_serial_number);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query camera customer serial number! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("SYS Customer Serial Number:\r\n");
  for (i = 0; i < LEP_SYS_MAX_SERIAL_NUMBER_CHAR_SIZE; i++)
    DEBUG_PRINTF("%x ", cust_serial_number.value[i]);
  DEBUG_PRINTF("\r\n");

  return HAL_OK;
}

static HAL_StatusTypeDef print_flir_serial_number()
{
  LEP_RESULT result;
  LEP_SYS_FLIR_SERIAL_NUMBER_T flir_serial_number;

  result = LEP_GetSysFlirSerialNumber(&hport_desc, &flir_serial_number);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query flir serial number! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("SYS FLIR Serial Number:\r\n");
  DEBUG_PRINTF("%08llX\r\n", flir_serial_number);

  return HAL_OK;
}

static HAL_StatusTypeDef print_camera_uptime()
{
  LEP_RESULT result;
  LEP_SYS_UPTIME_NUMBER_T uptime;

  result = LEP_GetSysCameraUpTime(&hport_desc, &uptime);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query camera uptime! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("SYS camera uptime:\r\n");
  DEBUG_PRINTF("%lu\r\n", uptime);

  return HAL_OK;
}

static HAL_StatusTypeDef print_fpa_temp_celcius()
{
  LEP_RESULT result;
  LEP_SYS_FPA_TEMPERATURE_CELCIUS_T temp;

  result = LEP_GetSysFpaTemperatureCelcius(&hport_desc, &temp);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query fpa temp! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("SYS fpa temp celcius:\r\n");
  DEBUG_PRINTF("%d.%d\r\n", (int)temp, (int)((temp-(int)temp)*100.0f));

  return HAL_OK;
}

static HAL_StatusTypeDef print_aux_temp_celcius()
{
  LEP_RESULT result;
  LEP_SYS_AUX_TEMPERATURE_CELCIUS_T temp;

  result = LEP_GetSysAuxTemperatureCelcius(&hport_desc, &temp);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query fpa temp! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("SYS aux temp celcius:\r\n");
  DEBUG_PRINTF("%d.%d\r\n", (int)temp, (int)((temp-(int)temp)*100.0f));

  return HAL_OK;
}

static HAL_StatusTypeDef print_sdk_version()
{
  LEP_RESULT result;
  LEP_SDK_VERSION_T version;

  result = LEP_GetSDKVersion(&hport_desc, &version);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query fpa temp! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("Lepton SDK version:\r\n");
  DEBUG_PRINTF("%u.%u.%u\r\n", version.major, version.minor, version.build);

  return HAL_OK;
}

HAL_StatusTypeDef get_scene_stats(uint16_t *min, uint16_t *max, uint16_t *avg)
{
  LEP_RESULT result;
  LEP_SYS_SCENE_STATISTICS_T stats;

  result = LEP_GetSysSceneStatistics(&hport_desc, &stats);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not get scene statistics! %d\r\n", result);
    return HAL_ERROR;
  }

  *min = stats.minIntensity;
  *max = stats.maxIntensity;
  *avg = stats.meanIntensity;

  return HAL_OK;
}

HAL_StatusTypeDef enable_lepton_agc()
{
  LEP_RESULT result;
  LEP_AGC_ENABLE_E enabled;

  result = LEP_GetAgcEnableState(&hport_desc, &enabled);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query AGC value %d\r\n", result);
    return HAL_ERROR;
  }
  DEBUG_PRINTF("Initial AGC value: %d\r\n", enabled);

  result = LEP_SetAgcCalcEnableState(&hport_desc, LEP_AGC_ENABLE);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not enable AGC calc\r\n");
    return HAL_ERROR;
  }

  LEP_SetAgcPolicy(&hport_desc, LEP_AGC_HEQ);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set AGC policy\r\n");
    return HAL_ERROR;
  }

  result = LEP_SetAgcEnableState(&hport_desc, LEP_AGC_ENABLE);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not enable AGC\r\n");
    return HAL_ERROR;
  }

  result = LEP_GetAgcEnableState(&hport_desc, &enabled);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query AGC value %d\r\n", result);
    return HAL_ERROR;
  }
  DEBUG_PRINTF("Current AGC value: %d\r\n", enabled);

  // result = LEP_SetAgcHeqScaleFactor(&hport_desc, LEP_AGC_SCALE_TO_14_BITS);
  // if (result != LEP_OK) {
  //   DEBUG_PRINTF("Could not set AGC scale factor\r\n");
  //   return HAL_ERROR;
  // }

  return HAL_OK;
}

HAL_StatusTypeDef disable_lepton_agc()
{
  LEP_RESULT result;

  result = LEP_SetAgcEnableState(&hport_desc, LEP_AGC_DISABLE);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not enable AGC\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef disable_telemetry(void)
{
  LEP_RESULT result;

  result = LEP_SetSysTelemetryEnableState(&hport_desc, LEP_TELEMETRY_DISABLED);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not disable telemetry %d\r\n", result);
    return HAL_ERROR;
  }

  g_telemetry_num_lines = 0;

  return HAL_OK;
}

HAL_StatusTypeDef enable_telemetry(void)
{
  LEP_RESULT result;

  result = LEP_SetSysTelemetryLocation(&hport_desc, LEP_TELEMETRY_LOCATION_FOOTER);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set telemetry location %d\r\n", result);
    return HAL_ERROR;
  }

  result = LEP_SetSysTelemetryEnableState(&hport_desc, LEP_TELEMETRY_ENABLED);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not enable telemetry %d\r\n", result);
    return HAL_ERROR;
  }

  g_telemetry_num_lines = g_lepton_type_3 ? 1 : 3;

  return HAL_OK;
}

static LEP_RAD_ENABLE_E rgb888_cached_tlinear_state;

HAL_StatusTypeDef disable_rgb888()
{
  LEP_RESULT result;

  result = LEP_SetRadTLinearEnableState(&hport_desc, rgb888_cached_tlinear_state);
  if (result == LEP_UNDEFINED_FUNCTION_ERROR) {
    DEBUG_PRINTF("LEP_SetRadTLinearEnableState() not available on this lepton\r\n");
  } else if (result != LEP_OK) {
    DEBUG_PRINTF("Could not restore tlinear setting %d\r\n", result);
    return HAL_ERROR;
  }

  return HAL_OK;
}


HAL_StatusTypeDef enable_rgb888(LEP_PCOLOR_LUT_E pcolor_lut)
{
  LEP_RESULT result;
  LEP_OEM_VIDEO_OUTPUT_FORMAT_E fmt;

  LEP_GetOemVideoOutputFormat(&hport_desc, &fmt);
  DEBUG_PRINTF("Current format: %d\r\n", fmt);

  // save the tlinear state to restore when we end rgb888
  result = LEP_GetRadTLinearEnableState(&hport_desc, &rgb888_cached_tlinear_state);
  if (result == LEP_UNDEFINED_FUNCTION_ERROR) {
    DEBUG_PRINTF("LEP_GetRadTLinearEnableState() not available on this lepton\r\n");
  } else if (result != LEP_OK) {
    DEBUG_PRINTF("Could not get tlinear state %d\r\n", result);
    return HAL_ERROR;
  }

  // disable tlinear because it messes with the AGC
  result = LEP_SetRadTLinearEnableState(&hport_desc, LEP_RAD_DISABLE);
  if (result == LEP_UNDEFINED_FUNCTION_ERROR) {
    DEBUG_PRINTF("LEP_SetRadTLinearEnableState() not available on this lepton\r\n");
  } else if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set tlinear state %d\r\n", result);
    return HAL_ERROR;
  }

  result = LEP_SetOemVideoOutputFormat(&hport_desc, LEP_VIDEO_OUTPUT_FORMAT_RGB888);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set output format %d\r\n", result);
    return HAL_ERROR;
  }

  LEP_GetOemVideoOutputFormat(&hport_desc, &fmt);
  DEBUG_PRINTF("New format: %d\r\n", fmt);

  if (pcolor_lut == -1) {
    // due to what I believe is a lepton bug,
    // even if we don't want to change the palette
    // we need to set it or we'll get noise on the
    // video stream
    result = LEP_GetVidPcolorLut(&hport_desc, &pcolor_lut);
    if (result != LEP_OK) {
      DEBUG_PRINTF("Could not get color lut: %d\r\n", result);
      pcolor_lut = PSUEDOCOLOR_LUT;
    }
  }

  result = LEP_SetVidPcolorLut(&hport_desc, pcolor_lut);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set color lut: %d\r\n", result);
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef enable_raw14()
{
  LEP_RESULT result;
  LEP_OEM_VIDEO_OUTPUT_FORMAT_E fmt;

  LEP_GetOemVideoOutputFormat(&hport_desc, &fmt);
  DEBUG_PRINTF("Current format: %d\r\n", fmt);

  result = LEP_SetOemVideoOutputFormat(&hport_desc, LEP_VIDEO_OUTPUT_FORMAT_RAW14);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set output format %d\r\n", result);
    return HAL_ERROR;
  }

  LEP_GetOemVideoOutputFormat(&hport_desc, &fmt);
  DEBUG_PRINTF("New format: %d\r\n", fmt);

  return HAL_OK;
}

// HAL_OK       = 0x00,
//  HAL_ERROR    = 0x01,
//  HAL_BUSY     = 0x02,
//  HAL_TIMEOUT  = 0x03
HAL_StatusTypeDef init_lepton_command_interface(void)
{
  LEP_RESULT result;

  result = LEP_OpenPort(0, LEP_CCI_TWI, 400, &hport_desc);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not open Lepton I2C port! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("Lepton I2C command interface opened, device %02x\r\n", hport_desc.deviceAddress);

  if (print_sdk_version() != HAL_OK)
    return HAL_ERROR;

  if (print_cust_serial_number() != HAL_OK)
    return HAL_ERROR;

  if (print_flir_serial_number() != HAL_OK)
    return HAL_ERROR;

  if (print_camera_uptime() != HAL_OK)
    return HAL_ERROR;

  if (print_fpa_temp_celcius() != HAL_OK)
    return HAL_ERROR;

  if (print_aux_temp_celcius() != HAL_OK)
    return HAL_ERROR;

  set_lepton_type();

  set_startup_defaults();

  return HAL_OK;
}

HAL_StatusTypeDef lepton_low_power()
{
  LEP_RESULT result;

  result = LEP_RunOemLowPowerMode2( &hport_desc );
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set low power mode 2: %d\r\n", result);
    return result;
  }

  return HAL_OK;
}

HAL_StatusTypeDef lepton_power_on()
{
  LEP_RESULT result;

  result = LEP_RunOemPowerOn( &hport_desc );
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set power on: %d\r\n", result);
    return result;
  }

  return HAL_OK;
}
