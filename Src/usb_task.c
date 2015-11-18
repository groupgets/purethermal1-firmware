
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"


#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"


#include "usb_task.h"
#include "lepton_task.h"

#include "project_config.h"




uint8_t packet[VIDEO_PACKET_SIZE];
lepton_buffer *last_buffer;


#define WHITE_LED_TOGGLE  (HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6))
extern volatile int restart_frame;
#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

void HAL_RCC_CSSCallback(void) {
	DEBUG_PRINTF("Oh no! HAL_RCC_CSSCallback()\r\n");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	DEBUG_PRINTF("Yay! HAL_GPIO_EXTI_Callback()\r\n");
}

#ifdef TMP007_OVERLAY 
#include "ugui.h"
UG_GUI gui; 
void pixel_set(UG_S16 x , UG_S16 y ,UG_COLOR c )
{
	last_buffer->lines[y].data.image_data[x] = c;
}
#endif

extern volatile uint8_t uvc_stream_status;
extern USBD_UVC_VideoControlTypeDef videoCommitControl;


void draw_splash(int min, int max)
{
	int x_loc = 0;
	int y_loc = 0;

	//UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('P',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('U',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('R',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('E',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar(' ',x_loc,y_loc,max,min);

	x_loc=0;
	y_loc += 8;
	UG_PutChar('T',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('h',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('e',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('r',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('m',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('a',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('l',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar(' ',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('1',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('0',x_loc,y_loc,max,min);

	x_loc=0;
	y_loc += 8;
	y_loc += 8;
	UG_PutChar('G',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('r',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('o',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('u',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('p',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('G',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('e',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('t',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('s',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('0',x_loc,y_loc,max,min);

	x_loc=0;
	y_loc += 8;
	//UG_PutChar('G',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('r',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('o',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('u',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('p',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('.',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('c',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('o',x_loc,y_loc,max,min);
	x_loc+=8;
	UG_PutChar('m',x_loc,y_loc,max,min);
	x_loc+=8;
	//UG_PutChar('0',x_loc,y_loc,max,min);

}

void get_min_max(int * min, int * max)
{
	int i,j;
	int val;
	*min= 0xffff;
	*max= 0;
	for (j = 0; j < 60; j++)
	{
		for (i = 0; i < 80; i++)
		{
			val = last_buffer->lines[j].data.image_data[i];

			if( val > *max )
			{
				*max = val;
			}
			if( val < *min)
			{
				*min = val;
			}
		}
	}
}

void scale_image_8bit(int min, int max)
{
	int i,j;
	int val;

	for (j = 0; j < 60; j++)
	{
		for (i = 0; i < 80; i++)
		{
			val = last_buffer->lines[j].data.image_data[i];
			val -= min;
			val = (( val * 255) / (max-min));

			last_buffer->lines[j].data.image_data[i] = val;
		}
	}
}


PT_THREAD( usb_task(struct pt *pt))
{
	static uint16_t val;
	static int temperature;
	static uint16_t count = 0, i;
	static int last_frame = 0;
	static int current_min, current_max;

	static uint8_t uvc_header[2] = { 2, 0 };
	static uint32_t uvc_xmit_row = 0, uvc_xmit_plane = 0;

	PT_BEGIN(pt);



#ifdef TMP007_OVERLAY 
	UG_Init(&gui,pixel_set,80,60);
	UG_FontSelect(&FONT_8X8);     
#endif

	while (1)
	{
		 PT_WAIT_UNTIL(pt, get_lepton_buffer(NULL) != last_frame);
		 WHITE_LED_TOGGLE;
		 last_frame = get_lepton_buffer(&last_buffer);

#ifndef ENABLE_LEPTON_AGC
		get_min_max(&current_min, &current_max);
		scale_image_8bit(current_min, current_max);
#endif

		if (((last_frame % 1800) > 0)   && ((last_frame % 1800) < 150)  )
		{
#ifdef SPLASHSCREEN_OVERLAY 
			draw_splash(255, 0);
#endif
		}
		else
		{

#ifdef TMP007_OVERLAY 
			temperature = get_last_mili_celisius()/1000;

			if(temperature < 0)
			{
				UG_PutChar('-',0,51,10000,0);
				temperature = -temperature;
			}
			else if(((temperature/100)%10) != 0 ) 
			{
				UG_PutChar((temperature/100)%10 + '0',0,51,255,0);
			}

			UG_PutChar((temperature/10)%10 + '0',8,51,255,0);
			UG_PutChar(temperature%10 + '0',16,51,255,0);
			UG_PutChar(248,24,51,255,0);
			UG_PutChar('C',32,51,255,0);
#endif
		}



    // perform stream initialization
    if (uvc_stream_status == 1)
    {
      DEBUG_PRINTF("Starting UVC stream...\r\n");

      uvc_header[0] = 2;
      uvc_header[1] = 0;
      UVC_Transmit_FS(uvc_header, 2);

      uvc_stream_status = 2;
      uvc_xmit_row = 0;
      uvc_xmit_plane = 0;
    }

    // put image on stream as long as stream is open
    while (uvc_stream_status == 2)
    {
      count = 0;

      packet[count++] = uvc_header[0];
      packet[count++] = uvc_header[1];

      switch (videoCommitControl.bFormatIndex[0])
      {
        // HACK: NV12 is semi-planar but YU12/I420 is planar. Deal with it when we have actual color.
        default:
        case FMT_INDEX_NV12:
        case FMT_INDEX_YU12:
        {
          // printf("Writing format 1, plane %d...\r\n", uvc_xmit_plane);

          switch (uvc_xmit_plane)
          {
            default:
            case 0: // Y
            {
              // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
              while (uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE)
              {
                for (i = 0; i < FRAME_LINE_LENGTH; i++)
                {
                  uint16_t val = last_buffer->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
                  // AGC is on so just use lower 8 bits
                  packet[count++] = (uint8_t)val;
                }

                uvc_xmit_row++;
              }

              if (uvc_xmit_row == IMAGE_NUM_LINES)
              {
                uvc_xmit_plane = 1;
                uvc_xmit_row = 0;
              }

              break;
            }
            case 1: // VU
            case 2:
            {
              if (videoCommitControl.bFormatIndex[0] == FMT_INDEX_NV12)
              {
                while (uvc_xmit_row < (IMAGE_NUM_LINES/2) && count < VIDEO_PACKET_SIZE)
                {
                  for (i = 0; i < FRAME_LINE_LENGTH && uvc_xmit_row < (IMAGE_NUM_LINES/2) && count < VIDEO_PACKET_SIZE; i++)
                    packet[count++] = 128;

                  uvc_xmit_row++;
                }

                if (uvc_xmit_row == 30)
                  packet[1] |= 0x2; // Flag end of frame
              }
              else
              {
                while (uvc_xmit_row < (IMAGE_NUM_LINES/2) && count < VIDEO_PACKET_SIZE)
                {
                  for (i = 0; i < (FRAME_LINE_LENGTH/2) && uvc_xmit_row < (IMAGE_NUM_LINES/2) && count < VIDEO_PACKET_SIZE; i++)
                    packet[count++] = 128;

                  uvc_xmit_row++;
                }

                // image is done
                if (uvc_xmit_row == (IMAGE_NUM_LINES/2))
                {
                  if (uvc_xmit_plane == 1)
                  {
                    uvc_xmit_plane = 2;
                    uvc_xmit_row = 0;
                  }
                  else
                  {
                    packet[1] |= 0x2; // Flag end of frame
                  }
                }
              }
              break;
            }
          }
  
          break;
        }
        case FMT_INDEX_GREY:
        {
          // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
              uint16_t val = last_buffer->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
              // AGC is on, so just use lower 8 bits
              packet[count++] = (uint8_t)val;
            }

            uvc_xmit_row++;
          }

          // image is done
          if (uvc_xmit_row == IMAGE_NUM_LINES)
          {
            packet[1] |= 0x2; // Flag end of frame
          }

          break;
        }
        case FMT_INDEX_Y16:
        {
          // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
              uint16_t val = last_buffer->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
              packet[count++] = (uint8_t)((val >> 0) & 0xFF);
              packet[count++] = (uint8_t)((val >> 8) & 0xFF);
            }

            uvc_xmit_row++;
          }

          // image is done
          if (uvc_xmit_row == IMAGE_NUM_LINES)
          {
            packet[1] |= 0x2; // Flag end of frame
          }

          break;
        }
        case FMT_INDEX_YUYV:
        {
          // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
              uint16_t val = last_buffer->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
              // AGC is on so just use lower 8 bits
              packet[count++] = (uint8_t)val;
              packet[count++] = 128;
            }

            uvc_xmit_row++;
          }

          // image is done
          if (uvc_xmit_row == IMAGE_NUM_LINES)
          {
            packet[1] |= 0x2; // Flag end of frame
          }

          break;
        }
      }

      // printf("UVC_Transmit_FS(): packet=%p, count=%d\r\n", packet, count);
      // fflush(stdout);

      while (UVC_Transmit_FS(packet, count) == USBD_BUSY && uvc_stream_status == 2)
        ;

      if (packet[1] & 0x2)
      {
        uvc_header[1] ^= 1; // toggle bit 0 for next new frame
        uvc_xmit_row = 0;
        uvc_xmit_plane = 0;
        // DEBUG_PRINTF("Frame complete\r\n");
        break;
      }
      PT_YIELD(pt);
    }
    PT_YIELD(pt);
	}
	PT_END(pt);
}
