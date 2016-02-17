#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "pt.h"
#include "lepton.h"
#include "tasks.h"
#include "lcd_display.h"
#include "project_config.h"

extern display16_t display_buffer[160*128];

static yuv422_buffer_t *last_buffer;
static lepton_buffer *last_buffer_rgb;

PT_THREAD( lcd_display_task(struct pt *pt))
{
  static int last_frame_count = 0;
  static int i,j;
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

        display_buffer[j * 2 * LCD_WIDTH + i * 2 + 0] = val;
        display_buffer[j * 2 * LCD_WIDTH + i * 2 + 1] = val;

        display_buffer[(j * 2 + 1) * LCD_WIDTH + i * 2 + 0] = val;
        display_buffer[(j * 2 + 1) * LCD_WIDTH + i * 2 + 1] = val;
      }
    }

    lcd_set_mem_window(0, 4, LCD_WIDTH-1, LCD_HEIGHT-4-1);
    lcd_blit_async(display_buffer, LCD_WIDTH, LCD_HEIGHT-8);
  }
  PT_END(pt);
}
