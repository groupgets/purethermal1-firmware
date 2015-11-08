
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
#include "image_support.h"


lepton_buffer *current_buffer;
lepton_buffer *completed_buffer;
int frames;


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


PT_THREAD( lepton_task(struct pt *pt))
{
	static uint32_t last_tick;

	PT_BEGIN(pt);

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
		}

		completed_buffer = current_buffer;
		current_buffer = lepton_transfer();
		frames++;
		//WHITE_LED_TOGGLE;

		PT_YIELD(pt);
	}
	PT_END(pt);
}


