#include "ntc_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "adc1_shared.h"
#include <math.h>

// ---- Configuration ----
// GPIO34 on ESP32 = ADC1 channel 6
#define NTC_ADC_UNIT      ADC_UNIT_1
#define NTC_ADC_CHANNEL   ADC_CHANNEL_6
#define NTC_ADC_ATTEN     ADC_ATTEN_DB_12   // ~0-3.3V range

// NTC voltage divider: Vcc -- R_FIXED -- (ADC node) -- NTC -- GND
// Values below derived via least-squares regression (ln(R) vs 1/T) on measured
// sensor data across 20-80C: 20C/1000R, 25C/900R, 30C/746R, 35C/611R, 40C/510R,
// 45C/450R, 50C/357R, 55C/305R, 60C/260R, 65C/222R, 70C/190R, 75C/160R, 80C/121R.
#define NTC_VCC            3.3f
#define NTC_R_FIXED        220.0f    // ohms, fixed pull-up resistor (measured circuit value)
#define NTC_NOMINAL_RES    900.0f    // ohms at nominal temp (measured @ 25C)
#define NTC_NOMINAL_TEMP   25.0f     // Celsius
#define NTC_BETA           3589.0f   // Beta coefficient, fitted from measured sensor data

// Smoothing: exponential moving average applied to the computed temperature.
// Lower alpha = smoother but slower to respond; higher alpha = more responsive but noisier.
// This value is intentionally responsive since oversampling (below) already removes most noise.
#define NTC_EMA_ALPHA      0.4f

// Oversampling: number of raw ADC samples averaged per reading to reduce noise
// without adding perceptible lag (samples are taken back-to-back, near-instant).
#define NTC_OVERSAMPLE_COUNT 16

// Calibration offset: added to the final computed temperature to correct for
// systematic error (e.g. self-heating, wiring resistance, sensor placement).
// Positive value increases the reported temperature, negative decreases it.
#define NTC_TEMP_OFFSET_C   -3.0f

static const char *TAG = "ntc_sensor";

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;
static bool s_cali_enabled = false;
static float s_ema_celsius = 0.0f;
static bool s_ema_initialized = false;

esp_err_t ntc_sensor_init(void)
{
    esp_err_t err = adc1_shared_get_handle(&s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc1_shared_get_handle failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = NTC_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc_handle, NTC_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    // Try to enable calibration (line-fitting scheme, supported on original ESP32)
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = NTC_ADC_UNIT,
        .atten = NTC_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_cali_handle) == ESP_OK) {
        s_cali_enabled = true;
        ESP_LOGI(TAG, "ADC calibration enabled (line fitting)");
    } else {
        ESP_LOGW(TAG, "ADC calibration not available, using raw conversion");
        s_cali_enabled = false;
    }

    return ESP_OK;
}

esp_err_t ntc_sensor_read_celsius(float *out_celsius)
{
    if (s_adc_handle == NULL || out_celsius == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    int32_t raw_sum = 0;
    esp_err_t err;
    for (int i = 0; i < NTC_OVERSAMPLE_COUNT; i++) {
        err = adc_oneshot_read(s_adc_handle, NTC_ADC_CHANNEL, &raw);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_read failed: %s", esp_err_to_name(err));
            return err;
        }
        raw_sum += raw;
    }
    raw = raw_sum / NTC_OVERSAMPLE_COUNT;

    int voltage_mv = 0;
    float voltage;
    if (s_cali_enabled) {
        err = adc_cali_raw_to_voltage(s_cali_handle, raw, &voltage_mv);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_cali_raw_to_voltage failed: %s", esp_err_to_name(err));
            return err;
        }
        voltage = voltage_mv / 1000.0f;
    } else {
        // Fallback: assume 12-bit ADC and approx 3.3V full-scale at 12dB atten
        voltage = (raw / 4095.0f) * NTC_VCC;
    }

    if (voltage <= 0.001f || voltage >= (NTC_VCC - 0.001f)) {
        ESP_LOGW(TAG, "NTC voltage out of range: %.3f V", voltage);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Calculate NTC resistance from divider: Vcc -- R_FIXED -- node(V) -- NTC -- GND
    float r_ntc = NTC_R_FIXED * (voltage / (NTC_VCC - voltage));

    // Beta equation: 1/T = 1/T0 + (1/B) * ln(R/R0)
    float t0_kelvin = NTC_NOMINAL_TEMP + 273.15f;
    float inv_t = (1.0f / t0_kelvin) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_NOMINAL_RES);
    float temp_kelvin = 1.0f / inv_t;
    float raw_celsius = (temp_kelvin - 273.15f) + NTC_TEMP_OFFSET_C;

    // Apply exponential moving average to smooth out noise/volatility
    if (!s_ema_initialized) {
        s_ema_celsius = raw_celsius;
        s_ema_initialized = true;
    } else {
        s_ema_celsius += NTC_EMA_ALPHA * (raw_celsius - s_ema_celsius);
    }
    *out_celsius = s_ema_celsius;

    return ESP_OK;
}
