#ifndef _PROJECT_CONFIG_H_
#define _PROJECT_CONFIG_H_

#ifdef USART_DEBUG // this happens by 'make USART_DEBUG=1'
#undef THERMAL_DATA_UART
#else
#define USART_DEBUG
#define THERMAL_DATA_UART
#endif

/* enable ST7734 160x128 TFT LCD output */
#define ENABLE_LCD_DISPLAY
#define USE_LCD_ST7735

#define TMP007_OVERLAY
#define SPLASHSCREEN_OVERLAY
#define ENABLE_LEPTON_AGC
// #define Y16

#ifndef Y16
// Values from LEP_PCOLOR_LUT_E in Middlewares/lepton_sdk/Inc/LEPTON_VID.h
#define PSUEDOCOLOR_LUT LEP_VID_FUSION_LUT
#endif

#ifndef USART_DEBUG_SPEED
#define USART_DEBUG_SPEED (921600)
#endif

#define WHITE_LED_TOGGLE  (HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6))



#endif


