# System-level settings shared by the game and the OTA loader (NVS "sys").
idf_component_register(SRCS "settings.c" INCLUDE_DIRS "include" PRIV_REQUIRES nvs_flash)
