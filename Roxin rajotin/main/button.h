#pragma once
#include "driver/gpio.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configure a GPIO as an active-low button input with internal pull-up.
void button_init(gpio_num_t pin);

// Returns true exactly once per physical press (debounced, edge-triggered).
// Call this periodically (e.g. every loop iteration / 20-50ms).
bool button_was_pressed(gpio_num_t pin);

// Returns true if the button is currently held down continuously for at least
// hold_ms milliseconds. Unlike button_was_pressed(), this can return true
// repeatedly while the button remains held - callers should track their own
// "already triggered" state if they only want a one-shot action.
bool button_is_held_for(gpio_num_t pin, uint32_t hold_ms);

#ifdef __cplusplus
}
#endif
