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

#define RING_SIZE (4)
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
  lepton_buffer *buf = get_next_lepton_buffer();
  vospi_packet *packet = (vospi_packet*)&buf->lines[0];

  // DEBUG_PRINTF("Transfer starting: %p@%p\r\n", buf, packet);

  do {
    if ((status = HAL_SPI_Receive(&hspi2, (uint8_t*)packet, FRAME_TOTAL_LENGTH, 200)) != HAL_OK)
    {
      DEBUG_PRINTF("Error setting up SPI receive to buf: %p@%p: %d\r\n", buf, packet, status);
      buf->status = LEPTON_STATUS_RESYNC;
      return buf;
    }
  } while ((buf->lines[0].header[0] & 0x0f00) == 0x0f00);

  status = HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)(packet + 1), FRAME_TOTAL_LENGTH * (IMAGE_NUM_LINES + TELEMETRY_NUM_LINES - 1));
  if (status)
  {
    DEBUG_PRINTF("Error setting up SPI DMA receive: %d\r\n", status);
    buf->status = LEPTON_STATUS_RESYNC;
    return buf;
  }

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
  vospi_packet *packet = (vospi_packet*)hspi->pRxBuffPtr;
  lepton_buffer *buffer = (lepton_buffer*)(packet - 1);
  uint8_t frame = buffer->lines[IMAGE_OFFSET_LINES + IMAGE_NUM_LINES - 1].header[0] & 0xff;

  if (frame != (IMAGE_NUM_LINES - 1))
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
    DEBUG_PRINTF("Initialized lepton buffer %d @ %p\r\n", i, &lepton_buffers[i]);
  }

	LEPTON_RESET_L_LOW;
  LEPTON_PW_DWN_LOW;

  HAL_Delay(190);
  LEPTON_PW_DWN_HIGH;

	HAL_Delay(190);
  LEPTON_RESET_L_HIGH;

}
