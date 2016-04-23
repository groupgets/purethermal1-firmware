
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

#define LEPTON_USART_PORT (USART2)

uint8_t lepton_raw[60*80*2];

extern volatile int restart_frame;
#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif


PT_THREAD( uart_task(struct pt *pt))
{
	static uint16_t val;
	static int last_frame_count = 0;
	static int count;
	static int i,j;
	static lepton_buffer *last_buffer;
    static int n;
    static uint8_t *ptr;

	PT_BEGIN(pt);

	while (1)
	{
#ifdef THERMAL_DATA_UART 
		 PT_WAIT_UNTIL(pt, get_lepton_buffer(NULL) != last_frame_count);
		 last_frame_count = get_lepton_buffer(&last_buffer);

		 count = 0;
		 for (j = 0; j < 60; j++)
		 {
			 for (i = 0; i < 80; i++)
			 {
#ifdef Y16
				 val = last_buffer->lines[j].data.image_data[i];
#else
				 rgb_t rgbval = last_buffer->lines[j].data.image_data[i];
				 val = (0.257f * (float)rgbval.r) + (0.504f * (float)rgbval.g) + (0.098f * (float)rgbval.b);
#endif

				 lepton_raw[count++] = ((val>>8)&0xff);
				 lepton_raw[count++] = (val&0xff);
			 }
		 }

		 n = 0;
		 ptr = lepton_raw;

		 while ((LEPTON_USART_PORT->SR & UART_FLAG_TC) == (uint16_t) RESET)
		 {
		   PT_YIELD(pt);
		 }
		 LEPTON_USART_PORT->DR =0xde;
		 while ((LEPTON_USART_PORT->SR & UART_FLAG_TC) == (uint16_t) RESET)
		 {
		   PT_YIELD(pt);
		 }
		 LEPTON_USART_PORT->DR =0xad;
		 while ((LEPTON_USART_PORT->SR & UART_FLAG_TC) == (uint16_t) RESET)
		 {
		   PT_YIELD(pt);
		 }
		 LEPTON_USART_PORT->DR =0xbe;
		 while ((LEPTON_USART_PORT->SR & UART_FLAG_TC) == (uint16_t) RESET)
		 {
		   PT_YIELD(pt);
		 }
		 LEPTON_USART_PORT->DR = 0xef;
		 //LEPTON_USART_PORT->DR = 0xed;

		 for (n = 0; n < (60*80*2); n++)
		 {
		   while ((LEPTON_USART_PORT->SR & UART_FLAG_TC) == (uint16_t) RESET)
		   {
		     PT_YIELD(pt);
		   }
		   LEPTON_USART_PORT->DR = (*ptr++ );
		 }

#else
		 PT_YIELD(pt);
#endif
	}
	PT_END(pt);
}


