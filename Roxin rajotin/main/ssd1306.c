#include "ssd1306.h"
#include "font5x7.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "ssd1306";

#define SSD1306_CMD_BYTE  0x00
#define SSD1306_DATA_BYTE 0x40

static esp_err_t ssd1306_cmd(ssd1306_t *disp, uint8_t cmd)
{
    uint8_t buf[2] = { SSD1306_CMD_BYTE, cmd };
    return i2c_master_transmit(disp->dev, buf, sizeof(buf), 1000 / portTICK_PERIOD_MS);
}

esp_err_t ssd1306_init(ssd1306_t *disp, i2c_master_bus_handle_t bus, uint8_t addr)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &disp->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        return err;
    }

    const uint8_t init_cmds[] = {
        0xAE,       // display off
        0xD5, 0x80, // clock divide
        0xA8, 0x3F, // multiplex ratio 64
        0xD3, 0x00, // display offset
        0x40,       // start line 0
        0x8D, 0x14, // charge pump enable
        0x20, 0x00, // memory addressing mode: horizontal
        0xA1,       // segment remap
        0xC8,       // COM output scan direction remapped
        0xDA, 0x12, // COM pins hw config
        0x81, 0xCF, // contrast
        0xD9, 0xF1, // pre-charge period
        0xDB, 0x40, // vcomh deselect level
        0xA4,       // resume to RAM content
        0xA6,       // normal display (not inverted)
        0xAF,       // display on
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        err = ssd1306_cmd(disp, init_cmds[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Init command 0x%02X failed: %s", init_cmds[i], esp_err_to_name(err));
            return err;
        }
    }

    ssd1306_clear(disp);
    return ssd1306_update(disp);
}

void ssd1306_clear(ssd1306_t *disp)
{
    memset(disp->buffer, 0, sizeof(disp->buffer));
}

esp_err_t ssd1306_update(ssd1306_t *disp)
{
    esp_err_t err;
    // Set full-screen addressing window each time (works with horizontal mode)
    uint8_t col_cmds[] = { SSD1306_CMD_BYTE, 0x21, 0x00, SSD1306_WIDTH - 1 };
    err = i2c_master_transmit(disp->dev, col_cmds, sizeof(col_cmds), 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) return err;

    uint8_t page_cmds[] = { SSD1306_CMD_BYTE, 0x22, 0x00, SSD1306_PAGES - 1 };
    err = i2c_master_transmit(disp->dev, page_cmds, sizeof(page_cmds), 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) return err;

    // Send data in chunks with the 0x40 control byte prefix
    uint8_t chunk[1 + 32];
    chunk[0] = SSD1306_DATA_BYTE;
    size_t total = sizeof(disp->buffer);
    size_t offset = 0;
    while (offset < total) {
        size_t chunk_len = total - offset;
        if (chunk_len > 32) chunk_len = 32;
        memcpy(&chunk[1], &disp->buffer[offset], chunk_len);
        err = i2c_master_transmit(disp->dev, chunk, chunk_len + 1, 1000 / portTICK_PERIOD_MS);
        if (err != ESP_OK) return err;
        offset += chunk_len;
    }
    return ESP_OK;
}

void ssd1306_draw_pixel(ssd1306_t *disp, int x, int y, int color)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;
    size_t idx = x + (y / 8) * SSD1306_WIDTH;
    uint8_t mask = 1 << (y % 8);
    if (color) {
        disp->buffer[idx] |= mask;
    } else {
        disp->buffer[idx] &= ~mask;
    }
}

static void draw_char(ssd1306_t *disp, int x, int y, char c, int size)
{
    if (c < 32 || c > 127) c = '?';
    const uint8_t *glyph = font5x7[c - 32];
    for (int col = 0; col < 5; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < 7; row++) {
            int on = (line >> row) & 0x01;
            if (size == 1) {
                ssd1306_draw_pixel(disp, x + col, y + row, on);
            } else {
                for (int dx = 0; dx < size; dx++) {
                    for (int dy = 0; dy < size; dy++) {
                        ssd1306_draw_pixel(disp, x + col * size + dx, y + row * size + dy, on);
                    }
                }
            }
        }
    }
}

void ssd1306_draw_string(ssd1306_t *disp, int x, int y, const char *str, int size)
{
    int cursor_x = x;
    int char_width = (5 + 1) * size; // 5 px glyph + 1 px spacing
    while (*str) {
        draw_char(disp, cursor_x, y, *str, size);
        cursor_x += char_width;
        str++;
    }
}

void ssd1306_draw_rect(ssd1306_t *disp, int x, int y, int w, int h, int color)
{
    for (int i = 0; i < w; i++) {
        ssd1306_draw_pixel(disp, x + i, y, color);
        ssd1306_draw_pixel(disp, x + i, y + h - 1, color);
    }
    for (int j = 0; j < h; j++) {
        ssd1306_draw_pixel(disp, x, y + j, color);
        ssd1306_draw_pixel(disp, x + w - 1, y + j, color);
    }
}

void ssd1306_fill_rect(ssd1306_t *disp, int x, int y, int w, int h, int color)
{
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            ssd1306_draw_pixel(disp, x + i, y + j, color);
        }
    }
}

void ssd1306_draw_bar_gauge(ssd1306_t *disp, int x, int y, int w, int h, int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    // Outer border
    ssd1306_draw_rect(disp, x, y, w, h, 1);

    // Inner fill area, inset by 2px on all sides from the border
    int inset = 2;
    int inner_x = x + inset;
    int inner_y = y + inset;
    int inner_w = w - (inset * 2);
    int inner_h = h - (inset * 2);
    if (inner_w < 0) inner_w = 0;
    if (inner_h < 0) inner_h = 0;

    // Clear the inner area first, then fill according to percent
    ssd1306_fill_rect(disp, inner_x, inner_y, inner_w, inner_h, 0);
    int fill_w = (inner_w * percent) / 100;
    if (fill_w > 0) {
        ssd1306_fill_rect(disp, inner_x, inner_y, fill_w, inner_h, 1);
    }
}

