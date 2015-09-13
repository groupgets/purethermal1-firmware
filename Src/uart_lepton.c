#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "uart_lepton.h"

#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

#define LEPTON_USART_PORT (USART2)


void send_lepton_via_usart(char * buffer)
{
	int n;
	char * ptr = buffer;
	while ((LEPTON_USART_PORT->SR & UART_FLAG_TC) == (uint16_t) RESET) {
	}
	LEPTON_USART_PORT->DR =0xde;
	while ((LEPTON_USART_PORT->SR & UART_FLAG_TC) == (uint16_t) RESET) {
	}
	LEPTON_USART_PORT->DR =0xad;
	while ((LEPTON_USART_PORT->SR & UART_FLAG_TC) == (uint16_t) RESET) {
	}
	LEPTON_USART_PORT->DR =0xbe;
	while ((LEPTON_USART_PORT->SR & UART_FLAG_TC) == (uint16_t) RESET) {
	}
	LEPTON_USART_PORT->DR = 0xef;
	//LEPTON_USART_PORT->DR = 0xed;


	for (n = 0; n < (60*80*2); n++) {
		while ((LEPTON_USART_PORT->SR & UART_FLAG_TC) == (uint16_t) RESET) {
		}
		LEPTON_USART_PORT->DR = (*ptr++ );

	}
}

