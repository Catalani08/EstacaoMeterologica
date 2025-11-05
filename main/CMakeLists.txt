idf_component_register(SRCS "app_main.c" "wifi_connect.c" "bme280.c"
                        PRIV_REQUIRES esp_driver_pcnt esp_driver_gpio
                       INCLUDE_DIRS "."
                       REQUIRES
                        driver
                        esp_adc
                        mqtt
                        esp_wifi
                        nvs_flash
                        esp_netif
                        esp_event)
