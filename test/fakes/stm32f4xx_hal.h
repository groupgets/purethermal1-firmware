/*
 * Host-test stand-in for the STM32 HAL.
 *
 * It sits ahead of the real include paths when the tests build, so firmware
 * sources compile unmodified on a PC. Every function declared here is
 * implemented by the simulator in test/sim.c, which records what the firmware
 * did and drives simulated time.
 */
#ifndef FAKE_STM32F4XX_HAL_H
#define FAKE_STM32F4XX_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define __IO volatile

typedef enum {
  HAL_OK      = 0x00,
  HAL_ERROR   = 0x01,
  HAL_BUSY    = 0x02,
  HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

typedef struct { int id; } GPIO_TypeDef;
extern GPIO_TypeDef fake_gpioa, fake_gpiob, fake_gpioc;
#define GPIOA (&fake_gpioa)
#define GPIOB (&fake_gpiob)
#define GPIOC (&fake_gpioc)

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

#define GPIO_PIN_0  ((uint16_t)0x0001)
#define GPIO_PIN_1  ((uint16_t)0x0002)
#define GPIO_PIN_4  ((uint16_t)0x0010)
#define GPIO_PIN_5  ((uint16_t)0x0020)
#define GPIO_PIN_6  ((uint16_t)0x0040)
#define GPIO_PIN_7  ((uint16_t)0x0080)
#define GPIO_PIN_8  ((uint16_t)0x0100)
#define GPIO_PIN_9  ((uint16_t)0x0200)
#define GPIO_PIN_12 ((uint16_t)0x1000)
#define GPIO_PIN_13 ((uint16_t)0x2000)

typedef enum { EXTI15_10_IRQn = 40 } IRQn_Type;

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
void HAL_NVIC_DisableIRQ(IRQn_Type irq);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type irq);

/* EXTI->PR takes a pin mask; the simulator records what it was given. */
void fake_exti_clear_it(uint32_t pin_mask);
#define __HAL_GPIO_EXTI_CLEAR_IT(PIN) fake_exti_clear_it(PIN)

/* The real pin map, so the firmware sees the same pin names it does on target. */
#include "mxconstants.h"

#endif
