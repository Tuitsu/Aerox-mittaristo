#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize ADC for fuel level reading on the configured GPIO (see fuel_sensor.c).
esp_err_t fuel_sensor_init(void);

// Read the current fuel level as a percentage (0 = empty, 100 = full).
// Returns ESP_OK on success and writes result to *out_percent.
esp_err_t fuel_sensor_read_percent(float *out_percent);

#ifdef __cplusplus
}
#endif
