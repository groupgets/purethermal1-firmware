#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "lepton_i2c.h"
#include "project_config.h"

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_VID.h"
#include "LEPTON_OEM.h"
#include "LEPTON_RAD.h"

#ifdef USART_DEBUG
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

HAL_StatusTypeDef disable_telemetry_and_radiometry(void)
{
  LEP_RESULT result;

  result = LEP_SetSysTelemetryEnableState(&hport_desc, LEP_TELEMETRY_DISABLED);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not disable telemetry %d\r\n", result);
    return HAL_ERROR;
  }

  result = LEP_SetRadEnableState(&hport_desc, LEP_RAD_DISABLE);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not disable radiometry %d\r\n", result);
    return HAL_ERROR;
  }

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

  return HAL_OK;
}

HAL_StatusTypeDef enable_rgb888(LEP_PCOLOR_LUT_E pcolor_lut)
{
  LEP_RESULT result;
  LEP_OEM_VIDEO_OUTPUT_FORMAT_E fmt;

  LEP_GetOemVideoOutputFormat(&hport_desc, &fmt);
  DEBUG_PRINTF("Current format: %d\r\n", fmt);

  result = LEP_SetOemVideoOutputFormat(&hport_desc, LEP_VIDEO_OUTPUT_FORMAT_RGB888);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set output format %d\r\n", result);
    return HAL_ERROR;
  }

  LEP_GetOemVideoOutputFormat(&hport_desc, &fmt);
  DEBUG_PRINTF("New format: %d\r\n", fmt);

  result = LEP_SetVidPcolorLut(&hport_desc, pcolor_lut);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set color lut: %d\r\n", result);
    return HAL_ERROR;
  }

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

  return HAL_OK;
}
