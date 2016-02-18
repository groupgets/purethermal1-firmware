#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "pt.h"
#include "lepton.h"
#include "tasks.h"
#include "lcd_display.h"
#include "project_config.h"

extern display16_t display_buffer[];

#define UPSCALING_X   ( LCD_WIDTH / FRAME_LINE_LENGTH )
#define UPSCALING_Y   ( LCD_HEIGHT / IMAGE_NUM_LINES )
#define FRAME_WIDTH   ( FRAME_LINE_LENGTH * UPSCALING_X )
#define FRAME_HEIGHT  ( IMAGE_NUM_LINES * UPSCALING_Y )
#define X_OFFSET      ( (LCD_WIDTH - FRAME_WIDTH) / 2 )
#define Y_OFFSET      ( (LCD_HEIGHT - FRAME_HEIGHT) / 2 )

static yuv422_buffer_t *last_buffer;
static lepton_buffer *last_buffer_rgb;

PT_THREAD( lcd_display_task(struct pt *pt))
{
  static int last_frame_count = 0;
  static int i,j,k,l;
  static rgb_t rgb;
  static display16_t val;

  PT_BEGIN(pt);

  while (1)
  {
    PT_WAIT_UNTIL(pt, get_lepton_buffer_yuv(NULL) != last_frame_count);
    last_frame_count = get_lepton_buffer_yuv(&last_buffer);
    get_lepton_buffer(&last_buffer_rgb);

    for (j = 0; j < IMAGE_NUM_LINES; j++)
    {
      for (i = 0; i < FRAME_LINE_LENGTH; i++)
      {
        rgb = last_buffer_rgb->lines[IMAGE_OFFSET_LINES + j].data.image_data[i];
        val = rgb_to_packed(rgb.r, rgb.g, rgb.b);

        for (k = 0; k < UPSCALING_Y; k++)
        {
          for (l = 0; l < UPSCALING_X; l++)
          {
            display_buffer[(j * UPSCALING_Y + k) * FRAME_WIDTH + i * UPSCALING_X + l] = val;
          }
        }
      }
    }

    lcd_set_mem_window(X_OFFSET, Y_OFFSET, X_OFFSET+FRAME_WIDTH-1, Y_OFFSET+FRAME_HEIGHT-1);
    lcd_blit_async(display_buffer, FRAME_WIDTH, FRAME_HEIGHT);
  }
  PT_END(pt);
}
