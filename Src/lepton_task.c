
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


lepton_buffer *completed_buffer;
uint32_t completed_frame_count;

uint8_t lepton_i2c_buffer[36];

unsigned short fpa_temperature_k;
unsigned short aux_temperature_k;


#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif


uint32_t get_lepton_buffer(lepton_buffer **buffer)
{
  if (buffer != NULL)
    *buffer = completed_buffer;
	return completed_frame_count;
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
	PT_BEGIN(pt);

	static uint32_t curtick = 0;
	static uint32_t last_tick = 0;
	static uint32_t last_logged_count = 0;
	static unsigned int current_frame_count;
	static lepton_buffer *current_buffer;
	curtick = last_tick = HAL_GetTick();

	while (1)
	{
		current_buffer = lepton_transfer();

		PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING);

		if (complete_lepton_transfer(current_buffer) != LEPTON_STATUS_OK)
		{
			if (current_buffer->status | LEPTON_STATUS_RESYNC)
			{
				DEBUG_PRINTF("Synchronization lost, status: %d\r\n", current_buffer->status);
				HAL_Delay(250);
			}
			else
			{
				DEBUG_PRINTF("Transfer failed, status: %d\r\n", current_buffer->status);
			}
			continue;
		}

		current_frame_count =
			(current_buffer->lines[TELEMETRY_OFFSET_LINES].data.telemetry_data.frame_counter[1] << 16) |
			(current_buffer->lines[TELEMETRY_OFFSET_LINES].data.telemetry_data.frame_counter[0] <<  0);

		if (((curtick = HAL_GetTick()) - last_tick) > 3000)
		{
			DEBUG_PRINTF("fps: %lu, last end line: %d, frame #%u, buffer %p\r\n",
				(current_frame_count - last_logged_count) / 3,
				current_buffer->lines[IMAGE_OFFSET_LINES + IMAGE_NUM_LINES - 1].header[0] & 0xff,
				current_frame_count, current_buffer
			);

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

			last_tick = curtick;
			last_logged_count = current_frame_count;
		}

		// Need to update completed buffer for clients?
		if (completed_frame_count != current_frame_count)
		{
			completed_buffer = current_buffer;
			completed_frame_count = current_frame_count;
		}
	}
	PT_END(pt);
}


