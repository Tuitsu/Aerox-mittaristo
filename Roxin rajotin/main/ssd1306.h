#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_PAGES  (SSD1306_HEIGHT / 8)

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t addr;
    uint8_t buffer[SSD1306_WIDTH * SSD1306_PAGES];
} ssd1306_t;

// Initialize the display using an already-created I2C bus handle.
esp_err_t ssd1306_init(ssd1306_t *disp, i2c_master_bus_handle_t bus, uint8_t addr);

// Clear the internal framebuffer (does not push to display until ssd1306_update).
void ssd1306_clear(ssd1306_t *disp);

// Push the internal framebuffer to the display over I2C.
esp_err_t ssd1306_update(ssd1306_t *disp);

// Draw a single pixel (color 0 = off, 1 = on).
void ssd1306_draw_pixel(ssd1306_t *disp, int x, int y, int color);

// Draw a text string at pixel column/row (row is 0..7, one text row = 8px tall) using 5x7 font, scaled.
void ssd1306_draw_string(ssd1306_t *disp, int x, int y, const char *str, int size);

// Draw a rectangle outline.
void ssd1306_draw_rect(ssd1306_t *disp, int x, int y, int w, int h, int color);

// Draw a filled rectangle.
void ssd1306_fill_rect(ssd1306_t *disp, int x, int y, int w, int h, int color);

// Draw a horizontal bar gauge with an outlined border and a filled portion representing
// percent (0-100). x, y, w, h define the outer border box in pixels.
void ssd1306_draw_bar_gauge(ssd1306_t *disp, int x, int y, int w, int h, int percent);

#ifdef __cplusplus
}
#endif
