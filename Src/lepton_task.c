
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"


#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"
#include "circ_buf.h"

#include "tasks.h"
#include "project_config.h"

extern volatile uint8_t g_lepton_type_3;

lepton_buffer *completed_buffer;
uint32_t completed_frame_count;

uint8_t lepton_i2c_buffer[36];

#define RING_SIZE (4)
lepton_buffer lepton_buffers[RING_SIZE];

lepton_buffer* completed_frames_buf[RING_SIZE] = { 0 };
DECLARE_CIRC_BUF_HANDLE(completed_frames_buf);

lepton_buffer* empty_frames_buf[RING_SIZE] = { 0 };
DECLARE_CIRC_BUF_HANDLE(empty_frames_buf);

uint32_t completed_yuv_frame_count;
yuv422_buffer_t yuv_buffers[2];

struct rgb_to_yuv_state {
  struct pt pt;
  lepton_buffer *restrict rgb;
  yuv422_buffer_t *restrict buffer;
};

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

uint32_t get_lepton_buffer(lepton_buffer **buffer)
{
  if (buffer != NULL)
    *buffer = completed_buffer;
	return completed_frame_count;
}

lepton_buffer* dequeue_lepton_buffer()
{
  if (empty(CIRC_BUF_HANDLE(completed_frames_buf)))
    return NULL;
  else
    return shift(CIRC_BUF_HANDLE(completed_frames_buf));
}

void return_lepton_buffer(lepton_buffer* buffer)
{
  push(CIRC_BUF_HANDLE(empty_frames_buf), buffer);
}

uint32_t get_lepton_buffer_yuv(yuv422_buffer_t **buffer)
{
  if (buffer != NULL)
    *buffer = &yuv_buffers[completed_yuv_frame_count%2];
	return completed_yuv_frame_count;
}

void init_lepton_task()
{
  int i;
  for (i = 0; i < RING_SIZE; i++)
  {
    lepton_buffers[i].number = i;
    lepton_buffers[i].status = LEPTON_STATUS_OK;
    push(CIRC_BUF_HANDLE(empty_frames_buf), &lepton_buffers[i]);
    DEBUG_PRINTF("Initialized lepton buffer %d @ %p\r\n", i, &lepton_buffers[i]);
  }
}

static float k_to_c(uint16_t unitsKelvin)
{
	return ( ( (float)( unitsKelvin / 100 ) + ( (float)( unitsKelvin % 100 ) * 0.01f ) ) - 273.15f );
}

static void print_telemetry_temps(telemetry_data_l2* telemetry)
{
	//
	uint16_t fpa_temperature_k = telemetry->fpa_temp_100k[0];
	uint16_t aux_temperature_k = telemetry->housing_temp_100k[0];

	float fpa_c = k_to_c(fpa_temperature_k);
	float aux_c = k_to_c(aux_temperature_k);

	DEBUG_PRINTF("fpa %d.%d°c, aux/housing: %d.%d°c\r\n",
		(int)(fpa_c), (int)((fpa_c-(int)fpa_c)*100),
		(int)(aux_c), (int)((aux_c-(int)aux_c)*100));
}

PT_THREAD( lepton_task(struct pt *pt))
{
	PT_BEGIN(pt);

	static uint32_t curtick = 0;
	static uint32_t last_tick = 0;
	static uint32_t last_logged_count = 0;
	static uint32_t current_frame_count = 0;
	static lepton_buffer *current_buffer = NULL;
	static struct pt rgb_to_yuv_pt;
	static int transferring_timer = 0;
	curtick = last_tick = HAL_GetTick();

	while (1)
	{
		if (current_buffer == NULL)
		{
			PT_YIELD_UNTIL(pt, !empty(CIRC_BUF_HANDLE(empty_frames_buf)));
			current_buffer = shift(CIRC_BUF_HANDLE(empty_frames_buf));
			current_buffer->status = LEPTON_STATUS_OK;
		}

		lepton_transfer(current_buffer);

		transferring_timer = HAL_GetTick();
		PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING || ((HAL_GetTick() - transferring_timer) > 200));

		if (complete_lepton_transfer(current_buffer) != LEPTON_STATUS_OK)
		{
			if (current_buffer->status == LEPTON_STATUS_TRANSFERRING){
				current_buffer->status = LEPTON_STATUS_RESYNC;
			}
			if (current_buffer->status == LEPTON_STATUS_RESYNC)
			{
				if (current_frame_count != 0)
					DEBUG_PRINTF("Synchronization lost, status: %d\r\n", current_buffer->status);
				HAL_Delay(250);
				push(CIRC_BUF_HANDLE(empty_frames_buf), current_buffer);
				current_buffer = NULL;
			}
			else if (current_buffer->status != LEPTON_STATUS_CONTINUE)
			{
				DEBUG_PRINTF("Transfer failed, status: %d\r\n", current_buffer->status);
				push(CIRC_BUF_HANDLE(empty_frames_buf), current_buffer);
				current_buffer = NULL;
			}
			continue;
		}

#ifdef Y16
		current_frame_count =
			(current_buffer->lines[TELEMETRY_OFFSET_LINES].data.telemetry_data.frame_counter[1] << 16) |
			(current_buffer->lines[TELEMETRY_OFFSET_LINES].data.telemetry_data.frame_counter[0] <<  0);
#else
		current_frame_count++;
#endif

		if (((curtick = HAL_GetTick()) - last_tick) > 3000)
		{
			DEBUG_PRINTF("fps: %lu, last end line: %d, frame #%lu, buffer %p\r\n",
				(current_frame_count - last_logged_count) / 3,
				current_buffer->lines[IMAGE_OFFSET_LINES + IMAGE_NUM_LINES - 1].header[0] & 0xff,
				current_frame_count, current_buffer
			);

#ifdef Y16
			print_telemetry_temps(&current_buffer->lines[TELEMETRY_OFFSET_LINES].data.telemetry_data);
#endif

			read_tmp007_regs();

			last_tick = curtick;
			last_logged_count = current_frame_count;
		}

		// if (g_lepton_type_3)
		// {
		// 	
		// }
		// else
		{
			// Need to update completed buffer for clients?
	#ifdef Y16
			if (completed_frame_count != current_frame_count)
	#else
			if ((current_frame_count % 3) == 0)
	#endif
			{
				completed_buffer = current_buffer;
				completed_frame_count = current_frame_count;

				HAL_GPIO_TogglePin(SYSTEM_LED_GPIO_Port, SYSTEM_LED_Pin);

	#ifndef Y16
				PT_SPAWN(
					pt,
					&rgb_to_yuv_pt,
					rgb_to_yuv(&rgb_to_yuv_pt, completed_buffer, &yuv_buffers[(completed_yuv_frame_count + 1) % 2])
				);
	#endif
			}
			else
			{
				push(CIRC_BUF_HANDLE(empty_frames_buf), completed_buffer);
			}
		}
	}
	PT_END(pt);
}

static inline uint8_t clamp (float x)
{
  if (x < 0)         return 0;
  else if (x > 255)  return 255;
  else               return (uint8_t)x;
}

PT_THREAD( rgb_to_yuv(struct pt *pt, lepton_buffer *restrict lepton, yuv422_buffer_t *restrict buffer))
{
  PT_BEGIN(pt);

  static int row, col;

  for (row = 0; row < IMAGE_NUM_LINES; row++)
  {
    uint16_t* lineptr = (uint16_t*)lepton->lines[IMAGE_OFFSET_LINES + row].data.image_data;
    while (lineptr < (uint16_t*)&lepton->lines[IMAGE_OFFSET_LINES + row].data.image_data[FRAME_LINE_LENGTH])
    {
      uint8_t* bytes = (uint8_t*)lineptr;
      *lineptr++ = bytes[0] << 8 | bytes[1];
    }

    for (col = 0; col < FRAME_LINE_LENGTH; col++)
    {
#ifdef Y16
      uint16_t val = lepton->lines[IMAGE_OFFSET_LINES + row].data.image_data[col];
      buffer->data[row][col] = (yuv422_t){ (uint8_t)val, 128 };
#else
      rgb_t val = lepton->lines[IMAGE_OFFSET_LINES + row].data.image_data[col];
      float r = val.r, g = val.g, b = val.b;

      float y1 = 0.299f * r + 0.587f * g + 0.114f * b;

      buffer->data[row][col].y =    clamp (0.859f *      y1  +  16.0f);
      if ((col % 2) == 0)
        buffer->data[row][col].uv = clamp (0.496f * (b - y1) + 128.0f);
      else
        buffer->data[row][col].uv = clamp (0.627f * (r - y1) + 128.0f);
#endif
    }
    PT_YIELD(pt);
  }

  completed_yuv_frame_count++;
  push(CIRC_BUF_HANDLE(completed_frames_buf), lepton);

	PT_END(pt);
}
