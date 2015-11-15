
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"


#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"

#include "lepton_task.h"
#include "project_config.h"


lepton_buffer *current_buffer;
lepton_buffer *completed_buffer;
int frames;

uint8_t lepton_i2c_buffer[36];

unsigned short fpa_temperature_k;
unsigned short aux_temperature_k;


#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif


lepton_buffer * get_lepton_buffer(void)
{
	return completed_buffer;
}

int get_lepton_frame(void)
{
	return frames;
}

void init_lepton_state(void);
void init_lepton_state(void)
{
/*
	  DEBUG_PRINTF("SYS Telemetry Enable State\n\r");
  status = lepton_command(SYS, 0x18 >> 2 , SET);
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
  status = read_data();
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }

   DEBUG_PRINTF("SYS Telemetry Location\n\r");
  status = lepton_command(SYS, 0x1C >> 2 , SET);
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
  status = read_data();
  if(status != HAL_OK) { DEBUG_PRINTF("ERROR: %d\n\r",status); }
*/
}


PT_THREAD( lepton_task(struct pt *pt))
{
	static uint32_t last_tick;

	PT_BEGIN(pt);

	init_lepton_state();

	last_tick = HAL_GetTick();
	frames = 0;

	// kick off the first transfer
	current_buffer = lepton_transfer();

	while (1)
	{
		while (complete_lepton_transfer(current_buffer) == LEPTON_STATUS_TRANSFERRING)
		{
			PT_YIELD(pt);
			fflush(stdout);
			//HAL_Delay(1);
		}

		if ((current_buffer->status & LEPTON_STATUS_RESYNC) == LEPTON_STATUS_RESYNC)
		{
			//DEBUG_PRINTF("Synchronization lost\r\n");
			read_tmp007_regs();
			HAL_Delay(200);
		}

		if ((frames % 30) == 0)
		{
			uint32_t curtick = HAL_GetTick();
			uint32_t ticks = curtick - last_tick;
			last_tick = curtick;
			DEBUG_PRINTF("ms / frame: %lu, last end line: %d\r\n", ticks / 30, current_buffer->data[82*59] & 0xff);
			read_tmp007_regs();
			//lepton_command(SYS, FPA_TEMPERATURE_KELVIN >> 2 , GET);
			//lepton_read_data(lepton_i2c_buffer);

			lepton_read_command(SYS, FPA_TEMPERATURE_KELVIN,lepton_i2c_buffer );
			fpa_temperature_k = (lepton_i2c_buffer[0]<<8 | (lepton_i2c_buffer[1] ));

			lepton_read_command(SYS, AUX_TEMPERATURE_KELVIN,lepton_i2c_buffer );
			aux_temperature_k = (lepton_i2c_buffer[0]<<8 | (lepton_i2c_buffer[1] ));

			//lepton_command(SYS, AUX_TEMPERATURE_KELVIN >> 2 , GET);
			//lepton_read_data(lepton_i2c_buffer);
			//aux_temperature_k = (lepton_i2c_buffer[0] | (lepton_i2c_buffer[1] <<8));

			DEBUG_PRINTF("fpa %x, aux: %x\r\n", fpa_temperature_k, aux_temperature_k);
		}

		completed_buffer = current_buffer;
		current_buffer = lepton_transfer();
		frames++;
		//WHITE_LED_TOGGLE;

		PT_YIELD(pt);
	}
	PT_END(pt);
}


