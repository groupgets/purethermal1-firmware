#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

#include "lepton.h"

#include "project_config.h"

#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

#define LEPTON_USART_PORT (USART2)

#define LEPTON_RESET_L_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)
#define LEPTON_RESET_L_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)

#define LEPTON_PW_DWN_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define LEPTON_PW_DWN_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

extern SPI_HandleTypeDef hspi2;

#define RING_SIZE (3)
lepton_buffer lepton_buffers[RING_SIZE];

lepton_buffer* get_next_lepton_buffer()
{
  static int buffer = 0;
  lepton_buffer* packet = &lepton_buffers[buffer];
  packet->status = LEPTON_STATUS_OK;
  buffer = ((buffer + 1) % RING_SIZE);
  return packet;
}

lepton_status complete_lepton_transfer(lepton_buffer* buffer)
{
  // TODO: additional synchronization desired?
  return buffer->status;
}

lepton_buffer* lepton_transfer(void)
{
  HAL_StatusTypeDef status;
  lepton_buffer* buf = get_next_lepton_buffer();

  do {
    if ((status = HAL_SPI_Receive(&hspi2, (uint8_t*)buf->data, 82, 200)) != HAL_OK)
    {
      DEBUG_PRINTF("Error setting up SPI receive: %d\r\n", status);
      continue;
    }

    if((buf->data[0] & 0x0f00) != 0x0f00)
    {
      status = HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)&buf->data[82], 82 * 59);
      if (status)
      {
        DEBUG_PRINTF("Error setting up SPI DMA receive: %d\r\n", status);
      }
      break;
    }

  } while (1);

  buf->status = LEPTON_STATUS_TRANSFERRING;
	return buf;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  DEBUG_PRINTF("SPI error!\n\r");
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  // lepton frame complete
  uint8_t* data = hspi->pRxBuffPtr;
  lepton_buffer* buffer = (lepton_buffer*)(data - VOSPI_FRAME_SIZE - 2);
  int frame;

  frame = buffer->data[59 * 82] & 0xff;

  if (frame != 59)
  {
    buffer->status = LEPTON_STATUS_RESYNC;
  }
  else
  {
    buffer->status = LEPTON_STATUS_OK;
  }
}

void lepton_init(void )
{
  int i;
  for (i = 0; i < RING_SIZE; i++)
  {
    lepton_buffers[i].number = i;
    lepton_buffers[i].status = LEPTON_STATUS_OK;
  }

	LEPTON_RESET_L_LOW;
  LEPTON_PW_DWN_LOW;

  HAL_Delay(190);
  LEPTON_PW_DWN_HIGH;

	HAL_Delay(190);
  LEPTON_RESET_L_HIGH;

}
