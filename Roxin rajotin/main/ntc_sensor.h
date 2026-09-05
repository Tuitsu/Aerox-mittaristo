#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize ADC for NTC reading on the configured GPIO (see ntc_sensor.c).
esp_err_t ntc_sensor_init(void);

// Read the current temperature in degrees Celsius.
// Returns ESP_OK on success and writes result to *out_celsius.
esp_err_t ntc_sensor_read_celsius(float *out_celsius);

#ifdef __cplusplus
}
#endif
