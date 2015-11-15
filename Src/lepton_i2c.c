#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

#include "lepton_i2c.h"
#include "project_config.h"

#define I2C_TIMEOUT (10000)

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

HAL_StatusTypeDef read_data()
{
	int i;
	uint8_t data[36];
	int payload_length;
	HAL_StatusTypeDef retrunval = 0;
	int timeout = 100;

	while (read_reg(0x2) & 0x01)
	{
		if(timeout--==0)
		{
			DEBUG_PRINTF("error busy timeout!\n\r");
			break;
		}
	}

	payload_length = read_reg(0x6);
	//DEBUG_PRINTF("payload_length=%d \n\r",payload_length);

	if((payload_length > 0) && (payload_length < 35) )
	{

		retrunval = HAL_I2C_Master_Receive(&hi2c1,LEPTON_ADDRESS,data,payload_length,I2C_TIMEOUT);

		for (i = 0; i < (payload_length); i++)
		{
			DEBUG_PRINTF("%02x",data[i]);
		}
		DEBUG_PRINTF("\n\r\n\r");
	}
	return retrunval;
}


int read_lepton_regs(void)
{
	uint8_t lepton_i2c_buffer[36];
	HAL_StatusTypeDef status;



	DEBUG_PRINTF("SYS Camera Customer Serial Number\n\r");
	status = lepton_command(SYS, CUST_SERIAL_NUMBER  , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

	DEBUG_PRINTF("SYS Flir Serial Number\n\r");
	status = lepton_command(SYS, FLIR_SERIAL_NUMBER  , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

	DEBUG_PRINTF("SYS Camera Uptime\n\r");
	status = lepton_command(SYS, CAM_UPTIME  , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }



	DEBUG_PRINTF("SYS Telemetry Enable State\n\r");
	status = lepton_command(SYS, TELEMETRY_ENABLE_STATE  , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

	DEBUG_PRINTF("SYS Telemetry Location\n\r");
	status = lepton_command(SYS, TELEMETRY_LOCATION  , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

	DEBUG_PRINTF("OEM Chip Mask Revision\n\r");
	status = lepton_command(OEM, 0x14  , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }


	DEBUG_PRINTF("OEM Camera Software Revision\n\r");
	status = lepton_command(OEM, 0x20  , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

	DEBUG_PRINTF("AGC READ\n\r");
	status = lepton_command(AGC, 0x00  , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

	//lepton_write_command_reg(SYS, TELEMETRY_LOCATION,0x0001);
	//lepton_write_command_reg(SYS, TELEMETRY_ENABLE_STATE,0x0001);

	DEBUG_PRINTF("SYS Telemetry Enable State\n\r");
	status = lepton_command(SYS, TELEMETRY_ENABLE_STATE , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

	DEBUG_PRINTF("SYS Telemetry Location\n\r");
	status = lepton_command(SYS, TELEMETRY_LOCATION , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

	lepton_read_command(SYS, FPA_TEMPERATURE_KELVIN,lepton_i2c_buffer );
	DEBUG_PRINTF("FPA_TEMPERATURE_KELVIN %x \n\r",(lepton_i2c_buffer[0]<<8 | (lepton_i2c_buffer[1])));

	DEBUG_PRINTF("SYS Fpa Temperature Kelvin\n\r");
	status = lepton_command(SYS, FPA_TEMPERATURE_KELVIN  , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

	lepton_read_command(SYS, AUX_TEMPERATURE_KELVIN,lepton_i2c_buffer );
	DEBUG_PRINTF("AUX_TEMPERATURE_KELVIN %x \n\r",(lepton_i2c_buffer[0]<<8 | (lepton_i2c_buffer[1])));

	DEBUG_PRINTF("SYS Aux Temperature Kelvin\n\r");
	status = lepton_command(SYS, AUX_TEMPERATURE_KELVIN  , GET);
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
	status = read_data();
	if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

	return 1;
}
