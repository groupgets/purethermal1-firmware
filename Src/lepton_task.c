
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"


#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"

#include "tasks.h"
#include "project_config.h"


lepton_buffer *completed_buffer;
uint32_t completed_frame_count;

uint8_t lepton_i2c_buffer[36];


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

static float k_to_c(uint16_t unitsKelvin)
{
	return ( ( (float)( unitsKelvin / 100 ) + ( (float)( unitsKelvin % 100 ) * 0.01f ) ) - 273.15f );
}

static void print_telemetry_temps(telemetry_data_l2* telemetry)
{
	//
	uint16_t fpa_temperature_k = telemetry->fpa_temp_100k[0];
	uint16_t aux_temperature_k = telemetry->housing_temp_100k[0];

	float fpa_c = k_to_c(fpa_temperature_k);
	float aux_c = k_to_c(aux_temperature_k);

	DEBUG_PRINTF("fpa %d.%d°c, aux/housing: %d.%d°c\r\n",
		(int)(fpa_c), (int)((fpa_c-(int)fpa_c)*100),
		(int)(aux_c), (int)((aux_c-(int)aux_c)*100));
}

PT_THREAD( lepton_task(struct pt *pt))
{
	PT_BEGIN(pt);

	static uint32_t curtick = 0;
	static uint32_t last_tick = 0;
	static uint32_t last_logged_count = 0;
	static uint32_t current_frame_count = 0;
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
				if (current_frame_count != 0)
					DEBUG_PRINTF("Synchronization lost, status: %d\r\n", current_buffer->status);
				HAL_Delay(250);
			}
			else
			{
				DEBUG_PRINTF("Transfer failed, status: %d\r\n", current_buffer->status);
			}
			continue;
		}

#ifdef Y16
		current_frame_count =
			(current_buffer->lines[TELEMETRY_OFFSET_LINES].data.telemetry_data.frame_counter[1] << 16) |
			(current_buffer->lines[TELEMETRY_OFFSET_LINES].data.telemetry_data.frame_counter[0] <<  0);
#else
		current_frame_count++;
#endif

		if (((curtick = HAL_GetTick()) - last_tick) > 3000)
		{
			DEBUG_PRINTF("fps: %lu, last end line: %d, frame #%lu, buffer %p\r\n",
				(current_frame_count - last_logged_count) / 3,
				current_buffer->lines[IMAGE_OFFSET_LINES + IMAGE_NUM_LINES - 1].header[0] & 0xff,
				current_frame_count, current_buffer
			);

#ifdef Y16
			print_telemetry_temps(&current_buffer->lines[TELEMETRY_OFFSET_LINES].data.telemetry_data);
#endif

			read_tmp007_regs();

			last_tick = curtick;
			last_logged_count = current_frame_count;
		}

		// Need to update completed buffer for clients?
#ifdef Y16
		if (completed_frame_count != current_frame_count)
#else
		if ((current_frame_count % 3) == 0)
#endif
		{
			completed_buffer = current_buffer;
			completed_frame_count = current_frame_count;
		}
	}
	PT_END(pt);
}


