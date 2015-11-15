#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "lepton_i2c.h"
#include "project_config.h"

#define I2C_TIMEOUT (10000)

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_VID.h"

#include "oem_sdk_shim.h"

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

uint16_t read_reg(unsigned int reg)
{
	int reading = 0;
	uint8_t data[2];
	HAL_StatusTypeDef status;

	data[0] = (reg >> 8 & 0xff);
	data[1] = (reg & 0xff);            // sends one byte
	HAL_I2C_Master_Transmit(&hi2c1,LEPTON_ADDRESS,data,2,I2C_TIMEOUT);

	if( (status = HAL_I2C_Master_Receive(&hi2c1,LEPTON_ADDRESS,data,2,I2C_TIMEOUT)) == HAL_OK)
	{
		reading =  (data[0]<<8 | (data[1]));
	}
	else
	{
		DEBUG_PRINTF("HAL_I2C_Master_Receive error %d\n\r",status);
	}
	return reading;
}

HAL_StatusTypeDef lepton_read_word(unsigned int reg,uint16_t * word)
{
	uint8_t data[2];
	HAL_StatusTypeDef status;

	data[0] = (reg >> 8 & 0xff);
	data[1] = (reg & 0xff);            // sends one byte
	HAL_I2C_Master_Transmit(&hi2c1,LEPTON_ADDRESS,data,2,I2C_TIMEOUT);

	if( (status = HAL_I2C_Master_Receive(&hi2c1,LEPTON_ADDRESS,data,2,I2C_TIMEOUT)) == HAL_OK)
	{
		*word =  (data[0]<<8 | (data[1]));
	}
	else
	{
		DEBUG_PRINTF("HAL_I2C_Master_Receive error %d\n\r",status);
	}
	return status;
}


HAL_StatusTypeDef lepton_wait_status(void )
{
	int timeout = 100;
	HAL_StatusTypeDef status = HAL_OK;

	while (read_reg(STATUS_REG) & 0x01)
	{
		if(timeout--==0)
		{
			DEBUG_PRINTF("error busy timeout!\n\r");
			break;
			status = HAL_TIMEOUT;
		}
	}

	return status;
}

HAL_StatusTypeDef lepton_write_word(unsigned int reg,uint16_t word)
{
	uint8_t data[4];

	data[0] = (reg >> 8 & 0xff);
	data[1] = (reg & 0xff);            // sends one byte

	data[2] = (word >> 8 & 0xff);
	data[3] = (word & 0xff);            // sends one byte

	return HAL_I2C_Master_Transmit(&hi2c1,LEPTON_ADDRESS,data,4,I2C_TIMEOUT);
}

HAL_StatusTypeDef lepton_read_data(uint8_t * data)
{
	int i;
	uint16_t payload_length;
	HAL_StatusTypeDef retrunval = 0;

	lepton_wait_status();

	payload_length = read_reg(DATA_LENGTH_REG);

	if((payload_length > 0) && (payload_length < 35) )
	{
		retrunval = HAL_I2C_Master_Receive(&hi2c1,LEPTON_ADDRESS,data,payload_length,I2C_TIMEOUT);
	}
	return retrunval;
}

HAL_StatusTypeDef lepton_read_command(unsigned int moduleID, unsigned int commandID,uint8_t * data )
{
	HAL_StatusTypeDef status;
	status |= lepton_wait_status();
	//status |= lepton_write_word(DATA_LENGTH_REG,0X01);
	status |= lepton_write_word(COMMAND_REG, (moduleID<<8) | (((commandID ) & 0xfc) | (GET & 0x3)));
	status |= lepton_wait_status();
	status |= lepton_read_data(data);
	return status;
}

HAL_StatusTypeDef lepton_write_command_reg(unsigned int moduleID, unsigned int commandID,uint16_t reg )
{
	HAL_StatusTypeDef status = 0; 
	status |= lepton_wait_status();
	status |= lepton_write_word(DATA_0_REG,reg);
	status |= lepton_write_word(DATA_LENGTH_REG,0X01);
	status |= lepton_write_word(COMMAND_REG, (moduleID<<8) | (commandID | SET));
	status |= lepton_wait_status();
	return status;
}

HAL_StatusTypeDef lepton_command(unsigned int moduleID, unsigned int commandID, unsigned int command)
{
	return lepton_write_word(COMMAND_REG, (moduleID<<8) | ((commandID) | command));
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

HAL_StatusTypeDef enable_rgb888(void)
{
  HAL_StatusTypeDef result;

  result = set_video_output_format(&hport_desc, VIDEO_OUTPUT_FORMAT_RGB888);
  if (result != HAL_OK) {
    DEBUG_PRINTF("Could not set video output mode %d\r\n", result);
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
