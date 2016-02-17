#ifndef LCD_DISPLAY_H_
#define LCD_DISPLAY_H_

#include <stdint.h>

#ifdef USE_LCD_ST7735
#define LCD_WIDTH   160
#define LCD_HEIGHT  128
#else
#define LCD_WIDTH   0
#define LCD_HEIGHT  0
#endif

typedef struct __attribute__((packed)) _display16_t {
  uint8_t msb;
  uint8_t lsb;
} display16_t;

inline display16_t rgb_to_packed(uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t color16 = ((r >> 3) << 11) | ((g >> 2) <<  5) | ((b >> 3) <<  0);
  return (display16_t) { .msb = color16 >> 8, .lsb = color16 & 0xff };
}

// Initialize the lcd display
void lcd_init();

// Prepare a RAM window on the display so subsequent blits happen to this region
void lcd_set_mem_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

// Blast data from a buffer to the previously prepared mem window (sync, cpu)
void lcd_blit(display16_t *buffer, uint16_t w, uint16_t h);

// Blast data from a buffer to the previously prepared mem window (async, dma)
void lcd_blit_async(display16_t *buffer, uint16_t w, uint16_t h);

// Fill a region with a color (sync, cpu)
void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, display16_t color);

// Fill the entire screen with a color (sync, cpu)
void lcd_fill_screen(display16_t color);

#endif
