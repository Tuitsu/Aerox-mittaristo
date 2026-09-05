#include "kill_switch.h"
#include "esp_log.h"

static const char *TAG = "kill_switch";
static bool s_active = false;

void kill_switch_init(gpio_num_t pin)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // ensure it defaults LOW even before first gpio_set_level
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(pin, 0); // idle LOW = relay de-energized = wire conducts (fail-safe)
    s_active = false;
}

void kill_switch_activate(gpio_num_t pin)
{
    if (!s_active) {
        ESP_LOGW(TAG, "Kill switch ACTIVATED - cutting wire");
    }
    gpio_set_level(pin, 1); // energize relay coil -> switches to NO -> wire cut
    s_active = true;
}

void kill_switch_deactivate(gpio_num_t pin)
{
    if (s_active) {
        ESP_LOGW(TAG, "Kill switch DEACTIVATED - restoring conduction");
    }
    gpio_set_level(pin, 0); // de-energize relay coil -> back to NC -> wire conducts
    s_active = false;
}

void kill_switch_toggle(gpio_num_t pin)
{
    if (s_active) {
        kill_switch_deactivate(pin);
    } else {
        kill_switch_activate(pin);
    }
}

bool kill_switch_is_active(void)
{
    return s_active;
}
