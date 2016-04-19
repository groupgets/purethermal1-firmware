
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"


#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"


#include "tasks.h"

#include "project_config.h"

extern volatile uint8_t uvc_stream_status;
extern struct uvc_streaming_control videoCommitControl;

static int overlay_mode = 0;

void change_overlay_mode(void)
{
	overlay_mode = (overlay_mode+1) % 2;
}

static int last_frame_count;
#ifdef Y16
static lepton_buffer *last_buffer;
#else
static yuv422_buffer_t *last_buffer;
static lepton_buffer *last_buffer_rgb;
#endif

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
static void pixel_set(UG_S16 x , UG_S16 y ,UG_COLOR c )
{
#ifdef Y16
	last_buffer->lines[y].data.image_data[x] = c;
#else
	last_buffer->data[y][x].y = c;
#endif
}
#endif

static void draw_splash(int min, int max)
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

#ifndef ENABLE_LEPTON_AGC
#ifdef Y16
static void get_min_max(lepton_buffer *buffer, uint16_t * min, uint16_t * max)
#else
static void get_min_max(yuv422_buffer_t *buffer, uint16_t * min, uint16_t * max)
#endif
{
	int i,j;
	*min= 0xffff;
	*max= 0;
	for (j = 0; j < IMAGE_NUM_LINES; j++)
	{
		for (i = 0; i < FRAME_LINE_LENGTH; i++)
		{
#ifdef Y16
			uint16_t val = buffer->lines[j].data.image_data[i];
#else
			uint8_t val = buffer->data[j][i].y;
#endif

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

#ifdef Y16
static void scale_image_8bit(lepton_buffer *buffer, uint16_t min, uint16_t max)
#else
static void scale_image_8bit(yuv422_buffer_t *buffer, uint16_t min, uint16_t max)
#endif
{
	int i,j;

	for (j = 0; j < IMAGE_NUM_LINES; j++)
	{
		for (i = 0; i < FRAME_LINE_LENGTH; i++)
		{
#ifdef Y16
			uint16_t val = buffer->lines[j].data.image_data[i];
			val -= min;
			val = (( val * 255) / (max-min));

			buffer->lines[j].data.image_data[i] = val;
#else
			uint8_t val = buffer->data[j][i].y;
			val -= min;
			val = (( val * 255) / (max-min));

			buffer->data[j][i].y = val;
#endif
		}
	}
}
#endif

PT_THREAD( usb_task(struct pt *pt))
{
	static int temperature;
	static uint16_t count = 0, i;
#ifndef ENABLE_LEPTON_AGC
	static uint16_t current_min, current_max;
#endif

	static uint8_t uvc_header[2] = { 2, 0 };
	static uint32_t uvc_xmit_row = 0, uvc_xmit_plane = 0;
	static uint8_t packet[VIDEO_PACKET_SIZE];

	PT_BEGIN(pt);

#ifdef TMP007_OVERLAY 
	UG_Init(&gui,pixel_set,80,60);
	UG_FontSelect(&FONT_8X8);     
#endif

	while (1)
	{
#ifdef Y16
		PT_WAIT_UNTIL(pt, get_lepton_buffer(NULL) != last_frame_count);
		last_frame_count = get_lepton_buffer(&last_buffer);
#else
		PT_WAIT_UNTIL(pt, get_lepton_buffer_yuv(NULL) != last_frame_count);
		last_frame_count = get_lepton_buffer_yuv(&last_buffer);
		get_lepton_buffer(&last_buffer_rgb);
#endif

#ifndef ENABLE_LEPTON_AGC
		switch (videoCommitControl.bFormatIndex)
		{
		case VS_FMT_INDEX(Y16):
			// leave the data alone
			break;
		default:
			// do our hoky linear agc for 8-bit types
			get_min_max(last_buffer, &current_min, &current_max);
			scale_image_8bit(last_buffer, current_min, current_max);
			break;
		}
#endif

		if (((last_frame_count % 1800) > 0)   && ((last_frame_count % 1800) < 150)  )
		{
#ifdef SPLASHSCREEN_OVERLAY 
			if(overlay_mode == 1)
			{
				draw_splash(255, 0);
			}
#endif
		}
		else
		{

#ifdef TMP007_OVERLAY 
			if(overlay_mode == 1)
			{
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
			}
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

      switch (videoCommitControl.bFormatIndex)
      {
        // HACK: NV12 is semi-planar but YU12/I420 is planar. Deal with it when we have actual color.
        case VS_FMT_INDEX(NV12):
        case VS_FMT_INDEX(YU12):
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
#ifdef Y16
                  uint16_t val = last_buffer->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
#else
                  uint8_t val = last_buffer->data[uvc_xmit_row][i].y;
#endif
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
              int incr = (videoCommitControl.bFormatIndex == VS_FMT_INDEX(NV12) ? 1 : 2);
              int plane_offset = uvc_xmit_plane - 1;

              while (uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE)
              {
                for (i = 0; i < FRAME_LINE_LENGTH && uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE; i += incr)
                {
#ifdef Y16
                  packet[count++] = 128;
#else
                  packet[count++] = last_buffer->data[uvc_xmit_row][i + plane_offset].uv;
#endif
                }

                uvc_xmit_row += 2;
              }

              // plane is done
              if (uvc_xmit_row == IMAGE_NUM_LINES)
              {
                if (uvc_xmit_plane == 1 && videoCommitControl.bFormatIndex == VS_FMT_INDEX(YU12))
                {
                  uvc_xmit_plane = 2;
                  uvc_xmit_row = 0;
                }
                else
                {
                  packet[1] |= 0x2; // Flag end of frame
                }
              }
              break;
            }
          }
  
          break;
        }
        case VS_FMT_INDEX(GREY):
        {
          // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
#ifdef Y16
              uint16_t val = last_buffer->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
#else
              uint8_t val = last_buffer->data[uvc_xmit_row][i].y;
#endif
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
        case VS_FMT_INDEX(Y16):
        {
          // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
#ifdef Y16
              uint16_t val = last_buffer->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
#else
              uint16_t val = last_buffer->data[uvc_xmit_row][i].y;
#endif
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
        default:
        case VS_FMT_INDEX(YUYV):
        {
#ifdef Y16
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
#else
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE)
          {
            memcpy(&packet[count], last_buffer->data[uvc_xmit_row], sizeof(yuv422_row_t));
            count += sizeof(yuv422_row_t);
            uvc_xmit_row++;
          }
#endif

          // image is done
          if (uvc_xmit_row == IMAGE_NUM_LINES)
          {
            packet[1] |= 0x2; // Flag end of frame
          }

          break;
        }
#ifndef Y16
        case VS_FMT_INDEX(BGR3):
        {
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
              rgb_t rgb = last_buffer_rgb->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
              packet[count++] = rgb.b;
              packet[count++] = rgb.g;
              packet[count++] = rgb.r;
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
        case VS_FMT_INDEX(RGB565):
        {
          while (uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE)
          {
            for (i = 0; i < FRAME_LINE_LENGTH; i++)
            {
              rgb_t rgb = last_buffer_rgb->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
              uint16_t val =
                ((rgb.r >> 3) << 11) |
                ((rgb.g >> 2) <<  5) |
                ((rgb.b >> 3) <<  0);

              packet[count++] = (val >> 0) & 0xff;
              packet[count++] = (val >> 8) & 0xff;
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
#endif
      }

      // printf("UVC_Transmit_FS(): packet=%p, count=%d\r\n", packet, count);
      // fflush(stdout);

      int retries = 1000;
      while (UVC_Transmit_FS(packet, count) == USBD_BUSY && uvc_stream_status == 2)
      {
        if (--retries == 0) {
//          DEBUG_PRINTF("UVC_Transmit_FS() failed (no one is acking)\r\n");
          break;
        }
      }

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
