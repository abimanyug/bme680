set(srcs "bme680_main.c")

idf_component_register(SRCS ${srcs}
                    PRIV_REQUIRES bt nvs_flash esp_driver_gpio esp_driver_i2c esp_bme680
                    INCLUDE_DIRS ".")
