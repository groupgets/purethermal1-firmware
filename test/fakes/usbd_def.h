/* Host-test stand-in for the ST USB device core types. Only what the real
 * usbd_uvc.h / usb_device.h headers mention by name. */
#ifndef FAKE_USBD_DEF_H
#define FAKE_USBD_DEF_H
#include "stm32f4xx_hal.h"
typedef struct { int unused; } USBD_HandleTypeDef;
typedef struct { int unused; } USBD_ClassTypeDef;
#endif
