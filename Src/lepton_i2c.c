#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

#include "lepton_i2c.h"

// #define DEBUG_PRINTF(...) debug_printf( __VA_ARGS__);
#define DEBUG_PRINTF(...) printf(__VA_ARGS__);
//#define DEBUG_PRINTF(...) 

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef lepton_command(unsigned int moduleID, unsigned int commandID, unsigned int command)
{
  uint8_t data[4];

  // Command Register is a 16-bit register located at Register Address 0x0004
  data[0] = (0x00);
  data[1] =(0x04);

  if (moduleID == 0x08) //OEM module ID
  {
    data[2] =(0x48);
  }
  else
  {
    data[2] =(moduleID & 0x0f);
  }
  data[3] =( ((commandID << 2 ) & 0xfc) | (command & 0x3));


  return HAL_I2C_Master_Transmit(&hi2c1,LEPTON_ADDRESS,data,4,10000);
}

HAL_StatusTypeDef agc_enable()
{
  uint8_t data[4];

  data[0] = (0x01);
  data[1] = (0x05);
  data[2] = (0x00);
  data[3] = (0x01);

  return HAL_I2C_Master_Transmit(&hi2c1,LEPTON_ADDRESS,data,4,10000);
}

HAL_StatusTypeDef set_reg(unsigned int reg)
{
  uint8_t data[2];

  data[0] = (reg >> 8 & 0xff);
  data[1] = (reg & 0xff);            // sends one byte

  return HAL_I2C_Master_Transmit(&hi2c1,LEPTON_ADDRESS,data,2,10000);
}

//Status reg 15:8 Error Code  7:3 Reserved 2:Boot Status 1:Boot Mode 0:busy

uint16_t read_reg(unsigned int reg)
{
  int reading = 0;
  uint8_t data[2];
  HAL_StatusTypeDef status;
  

  set_reg(reg);

  if( (status = HAL_I2C_Master_Receive(&hi2c1,LEPTON_ADDRESS,data,2,10000)) == HAL_OK)
  {

    reading =  (data[0]<<8 | (data[1]));

    //DEBUG_PRINTF("reg: %d==0x%x (%d)\n\r",reg,reading,reading);
  }
  else
  {
    DEBUG_PRINTF("HAL_I2C_Master_Receive error %d\n\r",status);

  }
  return reading;
}

HAL_StatusTypeDef read_data()
{
  int i;
  int dataval;
  uint8_t data[36];
  int payload_length;
  HAL_StatusTypeDef retrunval;
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

    retrunval = HAL_I2C_Master_Receive(&hi2c1,LEPTON_ADDRESS,data,payload_length,10000);

    for (i = 0; i < (payload_length); i++)
    {
      DEBUG_PRINTF("%x",data[i]);
    }
    DEBUG_PRINTF("\n\r\n\r");
  }
  return retrunval;
}

// HAL_OK       = 0x00,
//  HAL_ERROR    = 0x01,
//  HAL_BUSY     = 0x02,
//  HAL_TIMEOUT  = 0x03
int read_lepton_regs(void)
{
  HAL_StatusTypeDef status;


  DEBUG_PRINTF("SYS Camera Customer Serial Number\n\r");
  status = lepton_command(SYS, 0x28 >> 2 , GET);
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
  status = read_data();
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

  DEBUG_PRINTF("SYS Flir Serial Number\n\r");
  status = lepton_command(SYS, 0x2 , GET);
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
  status = read_data();
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

  DEBUG_PRINTF("SYS Camera Uptime\n\r");
  status = lepton_command(SYS, 0x0C >> 2 , GET);
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
  status = read_data();
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

  DEBUG_PRINTF("SYS Fpa Temperature Kelvin\n\r");
  status = lepton_command(SYS, 0x14 >> 2 , GET);
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
  status = read_data();
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

  DEBUG_PRINTF("SYS Aux Temperature Kelvin\n\r");
  status = lepton_command(SYS, 0x10 >> 2 , GET);
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
  status = read_data();
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

  DEBUG_PRINTF("OEM Chip Mask Revision\n\r");
  status = lepton_command(OEM, 0x14 >> 2 , GET);
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
  status = read_data();
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }


  DEBUG_PRINTF("OEM Camera Software Revision\n\r");
  status = lepton_command(OEM, 0x20 >> 2 , GET);
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
  status = read_data();
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

  //DEBUG_PRINTF("AGC Enable");
  //agc_enable();
  //read_data();

  DEBUG_PRINTF("AGC READ\n\r");
  status = lepton_command(AGC, 0x00  , GET);
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
  status = read_data();
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

  return 1;
}
