#include "fuel_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "adc1_shared.h"

// ---- Configuration ----
// GPIO35 on ESP32 = ADC1 channel 7
#define FUEL_ADC_UNIT      ADC_UNIT_1
#define FUEL_ADC_CHANNEL   ADC_CHANNEL_7
#define FUEL_ADC_ATTEN     ADC_ATTEN_DB_12   // ~0-3.3V range

// Fuel level sensor voltage divider: Vcc -- R_FIXED -- (ADC node) -- SENDER -- GND
// Sender is a variable resistor: ~1000 ohm = empty, ~100 ohm = full.
// Adjust R_FIXED to match your actual circuit (same divider style as NTC sensor).
#define FUEL_VCC            3.3f
#define FUEL_R_FIXED        220.0f    // ohms, fixed pull-up resistor (match hardware)
#define FUEL_RES_EMPTY       1000.0f  // ohms at empty
#define FUEL_RES_FULL        100.0f   // ohms at full

// Smoothing: exponential moving average applied to the computed percentage.
#define FUEL_EMA_ALPHA       0.15f

// Oversampling: number of raw ADC samples averaged per reading to reduce noise.
#define FUEL_OVERSAMPLE_COUNT 16

static const char *TAG = "fuel_sensor";

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;
static bool s_cali_enabled = false;
static float s_ema_percent = 0.0f;
static bool s_ema_initialized = false;

esp_err_t fuel_sensor_init(void)
{
    esp_err_t err = adc1_shared_get_handle(&s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc1_shared_get_handle failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = FUEL_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc_handle, FUEL_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    // Try to enable calibration (line-fitting scheme, supported on original ESP32)
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = FUEL_ADC_UNIT,
        .atten = FUEL_ADC_ATTEN,
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

esp_err_t fuel_sensor_read_percent(float *out_percent)
{
    if (s_adc_handle == NULL || out_percent == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    int32_t raw_sum = 0;
    esp_err_t err;
    for (int i = 0; i < FUEL_OVERSAMPLE_COUNT; i++) {
        err = adc_oneshot_read(s_adc_handle, FUEL_ADC_CHANNEL, &raw);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_read failed: %s", esp_err_to_name(err));
            return err;
        }
        raw_sum += raw;
    }
    raw = raw_sum / FUEL_OVERSAMPLE_COUNT;

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
        voltage = (raw / 4095.0f) * FUEL_VCC;
    }

    if (voltage <= 0.001f || voltage >= (FUEL_VCC - 0.001f)) {
        ESP_LOGW(TAG, "Fuel sensor voltage out of range: %.3f V", voltage);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Calculate sender resistance from divider: Vcc -- R_FIXED -- node(V) -- SENDER -- GND
    float r_sender = FUEL_R_FIXED * (voltage / (FUEL_VCC - voltage));

    // Linear mapping: FUEL_RES_EMPTY -> 0%, FUEL_RES_FULL -> 100%
    float raw_percent = (FUEL_RES_EMPTY - r_sender) / (FUEL_RES_EMPTY - FUEL_RES_FULL) * 100.0f;
    if (raw_percent < 0.0f) raw_percent = 0.0f;
    if (raw_percent > 100.0f) raw_percent = 100.0f;

    // Apply exponential moving average to smooth out noise/volatility
    if (!s_ema_initialized) {
        s_ema_percent = raw_percent;
        s_ema_initialized = true;
    } else {
        s_ema_percent += FUEL_EMA_ALPHA * (raw_percent - s_ema_percent);
    }
    *out_percent = s_ema_percent;

    return ESP_OK;
}
