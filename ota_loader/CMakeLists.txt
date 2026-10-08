# OTA loader: minimal WiFi-AP + HTTP updater living in the "factory" slot.
# Built as its own project (the game must NOT link the WiFi stack -- it does
# not fit in internal DRAM next to BT + video). Flash once over USB together
# with the game; afterwards the game's pad combo reboots into this.
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "../components")
# Diagnostics budget the game can't afford: bigger BLE log ring + a 16 KB
# write-once boot transcript replayed to every log subscriber (blelog.c).
add_compile_definitions(BLELOG_RING_SZ=8192 BLELOG_BOOT_SZ=16384)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ota-loader)
