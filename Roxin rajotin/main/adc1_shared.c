#include "adc1_shared.h"
#include "esp_log.h"

static const char *TAG = "adc1_shared";
static adc_oneshot_unit_handle_t s_handle = NULL;

esp_err_t adc1_shared_get_handle(adc_oneshot_unit_handle_t *out_handle)
{
    if (out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_handle == NULL) {
        adc_oneshot_unit_init_cfg_t init_cfg = {
            .unit_id = ADC_UNIT_1,
        };
        esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    *out_handle = s_handle;
    return ESP_OK;
}
