#include <limits.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "project_config.h"

#include "lcd_display.h"

#ifdef USE_LCD_ST7735
#include "lcd_st7735_defs.h"
#endif

extern SPI_HandleTypeDef hspi1;
display16_t display_buffer[LCD_WIDTH * LCD_HEIGHT];

void lcd_init()
{
#ifdef USE_LCD_ST7735
  _st7735_init();
#endif
}

void lcd_set_mem_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
#ifdef USE_LCD_ST7735
  // Set column address
  _st7735_write_command(ST7735_CASET);
  _st7735_write_data((uint8_t[4]){ 0x00, x0, 0x00, x1}, 4);

  // Set row address
  _st7735_write_command(ST7735_RASET);
  _st7735_write_data((uint8_t[4]){ 0x00, y0, 0x00, y1}, 4);

  // Be ready for memory write
  _st7735_write_command(ST7735_RAMWR);
#endif
}

void lcd_blit_async(display16_t *buffer, uint16_t w, uint16_t h)
{
#ifdef USE_LCD_ST7735
  _st7735_write_data_dma((uint8_t*)buffer, w * h * sizeof(uint16_t));
#endif
}

void lcd_blit(display16_t *buffer, uint16_t w, uint16_t h)
{
#ifdef USE_LCD_ST7735
  _st7735_write_data((uint8_t*)buffer, w * h * sizeof(uint16_t));
#endif
}

void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, display16_t color)
{
  size_t i;

  if((x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) return;
  if((x + w - 1) >= LCD_WIDTH)  w = LCD_WIDTH  - x;
  if((y + h - 1) >= LCD_HEIGHT) h = LCD_HEIGHT - y;

  for(i = 0; i < (w*h); i++)
    display_buffer[i] = color;

  lcd_set_mem_window(x, y, x+w-1, y+h-1);
  lcd_blit(display_buffer, w, h);
}

void lcd_fill_screen(display16_t color)
{
  lcd_fill_rect(0, 0,  LCD_WIDTH, LCD_HEIGHT, color);
}
