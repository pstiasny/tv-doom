idf_component_register(SRCS "power.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_adc esp_driver_gpio esp_driver_tsens esp_hw_support soc esp_rom app_update)
