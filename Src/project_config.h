#ifndef _PROJECT_CONFIG_H_
#define _PROJECT_CONFIG_H_

#ifdef USART_DEBUG // this happens by 'make USART_DEBUG=1'
#undef THERMAL_DATA_UART
#else
#define USART_DEBUG
#define THERMAL_DATA_UART
#endif
#define TMP007_OVERLAY
#define SPLASHSCREEN_OVERLAY
#define ENABLE_LEPTON_AGC
// #define Y16

#ifndef USART_DEBUG_SPEED
#define USART_DEBUG_SPEED (921600)
#endif

#define WHITE_LED_TOGGLE  (HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6))



#endif


