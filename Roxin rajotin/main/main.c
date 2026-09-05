#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "ssd1306.h"
#include "ntc_sensor.h"
#include "fuel_sensor.h"
#include "button.h"
#include "kill_switch.h"

// ---- Pin configuration ----
#define I2C_SCL_GPIO   22
#define I2C_SDA_GPIO   21
#define BUTTON_GPIO    GPIO_NUM_19  // mode-switch button, active-low with internal pull-up
#define KILL_SWITCH_GPIO GPIO_NUM_17 // drives relay transistor; idle LOW = wire conducts (fail-safe)
#define KILL_SWITCH_HOLD_MS 5000    // hold BUTTON_GPIO this long to trigger the kill switch

#define SSD1306_I2C_ADDR 0x3C

static const char *TAG = "main";

typedef enum {
    DISPLAY_MODE_TEMPERATURE = 0,
    DISPLAY_MODE_FUEL,
    DISPLAY_MODE_COUNT
} display_mode_t;

static display_mode_t s_display_mode = DISPLAY_MODE_TEMPERATURE;

// Displayed (visually smoothed) temperature. This value chases the real sensor
// reading but is only allowed to move by a small step each loop iteration, so
// the number shown on screen never jumps/skips - it counts up or down smoothly.
static float s_displayed_temp_c = NAN;
#define TEMP_DISPLAY_MAX_STEP_C  0.6f   // max change per loop iteration (~200ms)

// Latest fuel level percentage, used to draw the bar gauge.
static float s_fuel_percent = 0.0f;

// Tracks whether the current long-press has already triggered a kill-switch
// toggle, so holding the button doesn't toggle repeatedly while still held.
static bool s_kill_toggle_armed = true;

static i2c_master_bus_handle_t init_i2c_bus(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    return bus_handle;
}

void app_main(void)
{
    // Init I2C bus and display
    i2c_master_bus_handle_t bus = init_i2c_bus();

    static ssd1306_t disp;
    esp_err_t err = ssd1306_init(&disp, bus, SSD1306_I2C_ADDR);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Display init failed: %s", esp_err_to_name(err));
    }

    // Init NTC temperature sensor
    ESP_ERROR_CHECK(ntc_sensor_init());

    // Init fuel level sensor
    ESP_ERROR_CHECK(fuel_sensor_init());

    // Init mode-switch button
    button_init(BUTTON_GPIO);

    // Init kill-switch output (idles LOW = wire conducts, fail-safe)
    kill_switch_init(KILL_SWITCH_GPIO);

    char line1[32];
    char line2[32];

    while (1) {
        if (button_was_pressed(BUTTON_GPIO)) {
            s_display_mode = (s_display_mode + 1) % DISPLAY_MODE_COUNT;
            s_kill_toggle_armed = true; // fresh press - allow a new long-press toggle
        }

        // Long-press detection: holding the button for KILL_SWITCH_HOLD_MS toggles
        // the kill switch on/off. s_kill_toggle_armed ensures only one toggle happens
        // per press, even though the button stays held well past the threshold.
        if (s_kill_toggle_armed && button_is_held_for(BUTTON_GPIO, KILL_SWITCH_HOLD_MS)) {
            kill_switch_toggle(KILL_SWITCH_GPIO);
            s_kill_toggle_armed = false;
        }

        ssd1306_clear(&disp);

        switch (s_display_mode) {
            case DISPLAY_MODE_TEMPERATURE: {
                float celsius = 0.0f;
                esp_err_t r = ntc_sensor_read_celsius(&celsius);
                snprintf(line1, sizeof(line1), "Engine Temp");
                if (r == ESP_OK) {
                    if (isnan(s_displayed_temp_c)) {
                        // First valid reading - snap directly, no need to ramp from nothing
                        s_displayed_temp_c = celsius;
                    } else {
                        float diff = celsius - s_displayed_temp_c;
                        if (diff > TEMP_DISPLAY_MAX_STEP_C) {
                            diff = TEMP_DISPLAY_MAX_STEP_C;
                        } else if (diff < -TEMP_DISPLAY_MAX_STEP_C) {
                            diff = -TEMP_DISPLAY_MAX_STEP_C;
                        }
                        s_displayed_temp_c += diff;
                    }
                    snprintf(line2, sizeof(line2), "%.0fC", s_displayed_temp_c);
                } else {
                    snprintf(line2, sizeof(line2), "--C");
                }
                break;
            }
            case DISPLAY_MODE_FUEL:
            default: {
                float percent = 0.0f;
                esp_err_t r = fuel_sensor_read_percent(&percent);
                snprintf(line1, sizeof(line1), "Fuel");
                if (r == ESP_OK) {
                    snprintf(line2, sizeof(line2), "%.0f%%", percent);
                } else {
                    snprintf(line2, sizeof(line2), "--%%");
                    percent = 0.0f;
                }
                s_fuel_percent = percent;
                break;
            }
        }

        ssd1306_draw_string(&disp, 0, 0, line1, 1);
        if (s_display_mode == DISPLAY_MODE_FUEL) {
            // Bar gauge spanning most of the display width, plus the percentage as text.
            ssd1306_draw_bar_gauge(&disp, 0, 24, 128, 20, (int)(s_fuel_percent + 0.5f));
            ssd1306_draw_string(&disp, 0, 50, line2, 2);
        } else {
            // Temperature value uses a large font size for readability.
            ssd1306_draw_string(&disp, 0, 20, line2, 5);
        }

        // Kill-switch indicator: small filled dot in the top-right corner, shown on
        // every screen once the kill switch has been triggered.
        if (kill_switch_is_active()) {
            ssd1306_fill_rect(&disp, SSD1306_WIDTH - 6, 0, 4, 4, 1);
        }

        ssd1306_update(&disp);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
