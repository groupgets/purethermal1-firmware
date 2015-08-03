#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

#include "lepton.h"

// #define DEBUG_PRINTF(...) debug_printf( __VA_ARGS__);
#define DEBUG_PRINTF(...) printf(__VA_ARGS__);

#define LEPTON_USART_PORT (USART2)

#define LEPTON_RESET_L_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)
#define LEPTON_RESET_L_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)

#define LEPTON_PW_DWN_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define LEPTON_PW_DWN_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

extern SPI_HandleTypeDef hspi2;

uint16_t leptonpacket[LEPTON_IMAGE_SIZE];

#define RING_SIZE (3)
lepton_buffer lepton_buffers[RING_SIZE];

int print_image_binary_state =-1;
int print_image_binary_i = 0;
int print_image_binary_j = 0;

uint8_t binary_header[] = {0xde,0xad,0xbe,0xef};
//uint8_t binary_header[] = {0xad,0xde,0xef,0xbe};

void print_image_binary_background(void)
{
  uint8_t *data_pointer;
  if( print_image_binary_state == -1)
  {
    return;
  }
  else if( print_image_binary_state == 0)
  {
    //VCP_DataTx((uint8_t *) binary_header,4);
    DEBUG_PRINTF("\n%02x %02x %02x %02x\n", binary_header[0], binary_header[1], binary_header[2], binary_header[3]);
    print_image_binary_state = 5;
    //print_image_binary_state = 6;
    print_image_binary_i=0;
  }
  else if( print_image_binary_state == 5)
  {
    //VCP_put_char((leptonpacket[print_image_binary_i*80 + print_image_binary_j]>>8)&0xff);
    //VCP_put_char(leptonpacket[print_image_binary_i*80 + print_image_binary_j]&0xff);
    DEBUG_PRINTF("%04x ", leptonpacket[print_image_binary_i*80 + print_image_binary_j]&0xff);
    DEBUG_PRINTF("%04x ", (leptonpacket[print_image_binary_i*80 + print_image_binary_j]>>8)&0xff);

    print_image_binary_j++;
    if(print_image_binary_j>=80)
    {
      print_image_binary_j=0;
      print_image_binary_i+= 1;
      if(print_image_binary_i>=60)
      {
        print_image_binary_state = -1;
      }
    }
  }
  else if( print_image_binary_state == 6)
  {
    data_pointer = (uint8_t *) leptonpacket;
    //VCP_DataTx(data_pointer+print_image_binary_i,1);
    DEBUG_PRINTF("%p", data_pointer+print_image_binary_i);

    print_image_binary_i+=1;

    if(print_image_binary_i>=(60*80*2))
    {
      print_image_binary_i=0;
      print_image_binary_state = -1;
    }

  }
}

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
    if ((status = HAL_SPI_Receive(&hspi2, (uint8_t*)buf->data, 82, 0)) != HAL_OK)
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
