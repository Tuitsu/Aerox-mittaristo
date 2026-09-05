#pragma once
#include "driver/gpio.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configure the kill-switch output GPIO. The output idles LOW (wire conducts,
// relay held in its normally-closed/fail-safe position). Driving it HIGH
// energizes the relay coil (via transistor driver) and cuts the wire.
void kill_switch_init(gpio_num_t pin);

// Activate the kill switch (cuts the wire).
void kill_switch_activate(gpio_num_t pin);

// Deactivate the kill switch (restores conduction).
void kill_switch_deactivate(gpio_num_t pin);

// Toggle the kill switch state (on->off or off->on).
void kill_switch_toggle(gpio_num_t pin);

// Returns true if the kill switch has been activated.
bool kill_switch_is_active(void);

#ifdef __cplusplus
}
#endif
