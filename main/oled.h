#ifndef OLED_H
#define OLED_H

#include "driver/i2c.h"
#include "stdint.h"

#define OLED_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_PAGES 8

#define OLED_CMD 0x00
#define OLED_DATA 0x40

void oled_init(void);
void oled_clear(void);
void oled_display_on(void);
void oled_display_off(void);
void oled_set_pos(uint8_t page, uint8_t col);
void oled_write_char(uint8_t page, uint8_t col, char c);
void oled_write_string(uint8_t page, uint8_t col, const char *str);
void oled_write_num(uint8_t page, uint8_t col, int32_t num, uint8_t len);
void oled_draw_bmp(uint8_t page, uint8_t col, const uint8_t *bmp, uint8_t width, uint8_t height);
void oled_refresh(void);
void oled_clear_line(uint8_t page);

#endif
