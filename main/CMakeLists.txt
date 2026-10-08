idf_component_register(
    SRCS "main.c" "clkout.c" "sound.c" "ota.c" "crashlog.c" "testcard.c"
    INCLUDE_DIRS "."
    REQUIRES doomgeneric gamepad power settings
    PRIV_REQUIRES esp_driver_gpio esp_hw_support esp_hal_i2s soc esp_rom esp_driver_i2s app_update nvs_flash esp_wifi bt
)

# Bench diagnostic: -DTVDOOM_RADIO_OFF=1 on the idf.py command line.
# It MUST be turned into a compiler define here -- idf.py -D only sets a CMake
# cache variable, and a guard that nothing defines silently compiles as 0.
# That produced two "diagnostic" builds byte-identical to the normal firmware
# on 2026-09-18, and two published conclusions that measured nothing.
if(TVDOOM_RADIO_OFF)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TVDOOM_RADIO_OFF=1)
    message(WARNING "TVDOOM_RADIO_OFF=1: building with ALL RADIO DISABLED (bench only)")
endif()
# WiFi off, Bluetooth left running -- isolates any residual WiFi carried across
# the soft reset from the loader, which does bring WiFi up.
# (BLE transmit power is no longer a build option -- it is SETTINGS -> BLE TX
# POWER, sys_settings_t.ble_tx, applied via gamepad_set_tx_dbm().)
if(TVDOOM_WIFI_OFF)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TVDOOM_WIFI_OFF=1)
    message(WARNING "TVDOOM_WIFI_OFF=1: building with WIFI FORCED OFF, Bluetooth normal (bench only)")
endif()

target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_panic_handler")
