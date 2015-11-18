#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

#include "tmp007_i2c.h"

#include "project_config.h"


#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

extern I2C_HandleTypeDef hi2c1;

int tmp007_init = 0;
int16_t last_t_obj;
long last_mili_celisius;

long get_last_mili_celisius(void)
{
	return last_mili_celisius; 
}

HAL_StatusTypeDef tmp007_write_word(unsigned int reg,unsigned int word_data)
{
	uint8_t data[3];

	data[0] = (reg & 0xff);
	data[1] = ((word_data>>8) & 0xff);
	data[2] = (word_data & 0xff);

	return HAL_I2C_Master_Transmit(&hi2c1,TMP007_I2CADDR,data,3,10000);
}

uint16_t tmp007_read_reg(unsigned int reg)
{
	int reading = 0;
	uint8_t data[2];
	HAL_StatusTypeDef status;

	data[0] = (reg & 0xff);

	if( (status = HAL_I2C_Master_Transmit(&hi2c1,TMP007_I2CADDR,data,1,10000)) == HAL_OK)
	{
		if( (status = HAL_I2C_Master_Receive(&hi2c1,TMP007_I2CADDR,data,2,10000)) == HAL_OK)
		{
			reading =  (data[0]<<8 | (data[1]));
		}
		else
		{
			DEBUG_PRINTF("HAL_I2C_Master_Receive error %d\n\r",status);
		}
	}
	else
	{
		DEBUG_PRINTF("HAL_I2C_Master_Transmit error %d\n\r",status);
	}
	return reading;
}

long get_mili_celisius(void)
{
	int16_t  Tobj;
	long mili_celisius;

	Tobj = (int16_t) (tmp007_read_reg(TMP007_TOBJ));

	if(Tobj&0x1)
	{
		DEBUG_PRINTF("Invalid Data\n\r");
	}

	Tobj >>= 2;
	mili_celisius = ((long)Tobj * 3125)/100;

	last_t_obj = Tobj;
	last_mili_celisius = mili_celisius;

	return mili_celisius;
}

int convert_C_to_F(int C)
{
	return ((C*18 + 320)/10);
}

int read_tmp007_regs(void)
{
	long temperature;

	if(tmp007_init == 0)
	{
		if( tmp007_read_reg(TMP007_DEVID) != 0x0078 )
		{
			DEBUG_PRINTF("Bad device ID, got: %x \n\r",tmp007_read_reg(TMP007_DEVID));
		}

		DEBUG_PRINTF("tmp007 init\n\r");
		tmp007_write_word(TMP007_CONFIG, TMP007_CFG_MODEON | TMP007_CFG_TRANSC | TMP007_CFG_4SAMPLE);
		tmp007_init = 1;
	}

	temperature = get_mili_celisius();
	DEBUG_PRINTF("TMP007_TOBJ: %x  mC: %ld  C: %ld  F: %d\n\r",tmp007_read_reg(TMP007_TOBJ)>>2,temperature,temperature/1000, convert_C_to_F(temperature/1000));
	(void) temperature; //remove warning

	return 1;
}
