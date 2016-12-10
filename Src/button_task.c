
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


#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif


PT_THREAD( button_task(struct pt *pt))
{
	static uint32_t msCount;


	PT_BEGIN(pt);

	while (1)
	{

		PT_WAIT_UNTIL(pt, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0);
		msCount = HAL_GetTick();
		PT_YIELD(pt);
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0)
		{
			if( (HAL_GetTick() - msCount) > 4000 )
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			}

			PT_YIELD(pt);
		}
		change_overlay_mode();
	}
	PT_END(pt);
}


