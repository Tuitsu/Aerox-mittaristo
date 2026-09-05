#include "button.h"
#include "esp_timer.h"

#define DEBOUNCE_US 30000 // 30ms

static int64_t s_last_change_time = 0;
static int s_last_stable_level = 1; // idle = high (pull-up, active low)
static int s_last_read_level = 1;
static int64_t s_press_start_time = 0; // timestamp when button became stably pressed, 0 if not pressed

void button_init(gpio_num_t pin)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    s_last_stable_level = gpio_get_level(pin);
    s_last_read_level = s_last_stable_level;
}

bool button_was_pressed(gpio_num_t pin)
{
    int level = gpio_get_level(pin);
    int64_t now = esp_timer_get_time();

    if (level != s_last_read_level) {
        s_last_change_time = now;
        s_last_read_level = level;
    }

    bool pressed_event = false;
    if ((now - s_last_change_time) > DEBOUNCE_US && level != s_last_stable_level) {
        s_last_stable_level = level;
        // Active low: pressed when level transitions to 0
        if (level == 0) {
            pressed_event = true;
            s_press_start_time = now;
        } else {
            s_press_start_time = 0;
        }
    }
    return pressed_event;
}

bool button_is_held_for(gpio_num_t pin, uint32_t hold_ms)
{
    (void)pin; // single-button state machine; pin kept for API symmetry/future multi-button support
    if (s_press_start_time == 0 || s_last_stable_level != 0) {
        return false; // not currently pressed
    }
    int64_t now = esp_timer_get_time();
    int64_t held_us = now - s_press_start_time;
    return held_us >= ((int64_t)hold_ms * 1000);
}
