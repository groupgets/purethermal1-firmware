
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
#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif


PT_THREAD( uart_task(struct pt *pt))
{
	static uint16_t val;
	static int count;
	static int i,j;
	static lepton_buffer *last_buffer;
    static int n;
    static uint8_t *ptr;

	PT_BEGIN(pt);

	while (1)
	{
		PT_WAIT_UNTIL(pt, (last_buffer = dequeue_lepton_buffer()) != NULL);

		 count = 0;
		 for (j = 0; j < 60; j++)
		 {
			 for (i = 0; i < 80; i++)
			 {
				 val = last_buffer->lines.y16[j].data.image_data[i];

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

	}
	PT_END(pt);
}


