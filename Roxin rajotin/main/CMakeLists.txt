idf_component_register(SRCS "kill_switch.c" "adc1_shared.c" "fuel_sensor.c" "button.c" "ntc_sensor.c" "ssd1306.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_adc esp_timer)
