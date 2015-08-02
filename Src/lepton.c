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

#define VOSPI_FRAME_SIZE (164)
#define LEPTON_IMAGE_SIZE ((60*80)*2)
uint16_t raw_lepton_frame_packet[VOSPI_FRAME_SIZE/2 * 60];
uint16_t leptonpacket[LEPTON_IMAGE_SIZE];

int lost_frame_counter = 0;
int last_frame_number;
int frame_complete = 0;
int start_image = 0;
int need_resync = 0;
int current_crc;
int last_crc;
int new_frame = 0;
int frame_counter = 0;

volatile int restart_frame = 0;


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

int lepton_transfer(void)
{
  HAL_StatusTypeDef status;

  do {
    if ((status = HAL_SPI_Receive(&hspi2, (uint8_t*)&raw_lepton_frame_packet, 82, 0)) != HAL_OK)
    {
      DEBUG_PRINTF("Error setting up SPI receive: %d\r\n", status);
      continue;
    }

    if((raw_lepton_frame_packet[0] & 0x0f00) != 0x0f00)
    {
      status = HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)&raw_lepton_frame_packet[82], 82 * 59);
      if (status)
      {
        DEBUG_PRINTF("Error setting up SPI DMA receive: %d\r\n", status);
      }
      break;
    }

  } while (1);

	return 0;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  DEBUG_PRINTF("SPI error!\n\r");
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  // lepton frame complete
  int frame;

  frame = raw_lepton_frame_packet[59 * 82] & 0xff;

  if (frame != 59)
  {
    restart_frame = -1;
  }
  else
  {
    restart_frame = frame;
  }
}

void lepton_init(void )
{

	LEPTON_RESET_L_LOW;
  LEPTON_PW_DWN_LOW;

  HAL_Delay(190);
  LEPTON_PW_DWN_HIGH;

	HAL_Delay(190);
  LEPTON_RESET_L_HIGH;

}

