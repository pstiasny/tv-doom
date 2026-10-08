#!/bin/bash
# ONE-TIME migration to the OTA partition layout (2026-09-14). Run with the
# board awake on USB. After this, day-to-day updates go over WiFi:
# pad View+Menu 3 s -> on-TV loader menu (join network, on-screen keyboard)
# -> UPDATE pulls from the server that tools/ota.py runs. USB = fallback.
#
# Flashes: bootloader (rollback-enabled), partition table, otadata (preset to
# boot the game -- NOTE build/ota_data_initial.bin is ERASED content, which
# boots the FACTORY loader when one exists; tools/otadata_boot_ota0.bin is a
# hand-built entry [seq=1 + ROM crc32_le] that selects ota_0), OTA loader (factory slot), game (ota_0), WAD at its NEW
# offset 0x330000. NVS is left alone so the gamepad bond survives.
#
# Usage: tools/flash_ota_migration.sh [port] [wadfile]
set -e
cd "$(dirname "$0")/.."
PORT=${1:-$(ls /dev/cu.usbmodem* 2>/dev/null | grep -v 4001 | head -1)}
WAD=${2:-wads/doom.wad}
[ -n "$PORT" ] || { echo "no port"; exit 1; }
[ -f build/tv-doom.bin ] || { echo "build the game first"; exit 1; }
[ -f ota_loader/build/ota-loader.bin ] || { echo "build ota_loader first"; exit 1; }
[ -f "$WAD" ] || { echo "WAD $WAD missing"; exit 1; }

# Run inside an exported ESP-IDF environment (export.sh sets IDF_PYTHON_ENV_PATH).
PY="${IDF_PYTHON_ENV_PATH:?run the ESP-IDF export script first}/bin/python"
"$PY" -m esptool --chip esp32s3 -p "$PORT" -b 460800 \
    --before default-reset --after hard-reset write-flash \
    --flash-mode dio --flash-size 16MB --flash-freq 80m \
    0x0       build/bootloader/bootloader.bin \
    0x8000    build/partition_table/partition-table.bin \
    0x1A0000  tools/otadata_boot_ota0.bin \
    0x20000   ota_loader/build/ota-loader.bin \
    0x1B0000  build/tv-doom.bin \
    0x330000  "$WAD"
echo "migration complete: game boots from ota_0, loader in factory slot"
