idf_component_register(
    SRCS "loader_main.c" "ota_display.c" "ota_menu.c" "ota_sound.c" "../../main/clkout.c"
    INCLUDE_DIRS "." "../../main"
    PRIV_REQUIRES esp_driver_i2s esp_wifi esp_netif esp_event esp_http_client app_update esp_timer
                  esp_lcd esp_driver_gpio esp_hw_support esp_hal_i2s hal soc esp_rom
                  nvs_flash gamepad settings
)
