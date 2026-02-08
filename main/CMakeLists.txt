idf_component_register(SRCS "soil_moisture_sensor.c"
                    INCLUDE_DIRS "."
                    REQUIRES u8g2
                    PRIV_REQUIRES driver

                    REQUIRES nvs_flash esp_wifi esp_event esp_netif
                    REQUIRES esp_http_client esp_adc driver

    )
