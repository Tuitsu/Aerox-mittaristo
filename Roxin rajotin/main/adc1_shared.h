#pragma once
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Returns the shared ADC1 oneshot unit handle, creating it on first call.
// All ADC1 sensors (NTC, fuel level, etc.) should use this instead of creating
// their own unit, since only one oneshot unit can exist per ADC peripheral.
esp_err_t adc1_shared_get_handle(adc_oneshot_unit_handle_t *out_handle);

#ifdef __cplusplus
}
#endif
