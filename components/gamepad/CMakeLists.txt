# BLE gamepad input (Xbox Series X|S controller over BLE HID / NimBLE).
# esp_hid_gap.c is adapted from the IDF esp_hid_host example: scan filter
# widened to accept real HID gamepads (the stock one only matched ESP demo
# names) and security-manager config added for Just-Works bonding.
# Depends on doomgeneric for doomkeys.h / doom_push_key().
idf_component_register(
    SRCS "gamepad.c" "esp_hid_gap.c" "blelog.c"
    INCLUDE_DIRS "."
    REQUIRES bt esp_hid nvs_flash esp_timer doomgeneric esp_app_format esp_system
)
