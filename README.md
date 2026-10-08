# tv-doom

This repo contains two components:

*  The ESP32-S3-based board sandwich that fits around a battery in
   a 9.5mm antenna plug.
*  A port of Doom for that hardware, including a PAL video and audio
   signal synthesizer.

The detailed description of the build is described in this blog post:

https://raytriangle.com/making-the-worlds-smallest-game-console/

Note that parts of the hardware engineering and almost all
of coding in this project were done in collaboration with Claude.


## Layout

| Path | What |
|---|---|
| `main/` | Game app: engine task, sound subcarrier, clock output, OTA hand-over, crash log, test cards |
| `components/video/` | PAL composite encoder on the RGB-LCD DMA engine, plus the compiled-in RF calibration curve |
| `components/gamepad/` | Xbox controller HID-over-GATT client on NimBLE, and a BLE log console |
| `components/power/`, `components/settings/` | Supply/temperature monitoring and NVS-backed settings |
| `components/doomgeneric/` | The Doom engine, vendored. See its README for provenance and local changes |
| `ota_loader/` | Second ESP-IDF project: WiFi + HTTP updater for the factory slot |
| `tools/` | Host-side scripts: WAD flashing, OTA image server, BLE log console, perf stats |
| `hardware/` | tvplug board, KiCad 10 schematic and layout |

## Building

Toolchain: ESP-IDF `v6.1-dev` (tested at git `fa47e101d53`). The NimBLE
settings in `sdkconfig.defaults` work around behaviour specific to that
revision, so an older or newer IDF may need adjusting. Both projects target
the ESP32-S3R8 with 16 MB flash and octal PSRAM.

```sh
. $IDF_PATH/export.sh
idf.py build                       # game -> build/tv-doom.bin
(cd ota_loader && idf.py build)    # loader -> ota_loader/build/ota-loader.bin
```

Host tools need Python 3 with `pip install -r tools/requirements.txt`, run
inside the exported IDF environment so that `esptool` is on the path.

## Flashing

An IWAD is required and is not included. Put `doom.wad` (or the freely
redistributable shareware `doom1.wad`) in `wads/`. `doom2.wad` does not fit the
13.4 MB WAD partition next to the two OTA slots.

The first flash goes over USB and writes everything at once: bootloader,
partition table, OTA data, loader, game and WAD.

```sh
tools/flash_ota_migration.sh /dev/cu.usbmodemXXX wads/doom.wad
```

After that, updates can go over WiFi. Press the home (big X) button on the controller
to reboot into the loader, join a network with the on-screen keyboard,
and pick an image from the server started with `tools/ota.py` on the build
machine. The loader currently has the server address compiled in at the top of
`ota_loader/main/ota_menu.c`; change it to your own.

A game image that crashes before marking itself healthy rolls back to the
loader, and four crash-class resets in a row do the same, so a bad build can
always be replaced without opening the enclosure.

## Hardware

`hardware/` holds the KiCad 10 project for the tvplug2 board. Proprietary
footprints and 3D models are not included.

The physical layout was designed specifically to fit within a "Goobay 11502"
9.5 TV antenna plug and a 601520 150mAh LiPo battery.

The mixer calibration curve in `components/video/rfcal_default.h`
was measured on a real board with a TinySA and a Pluto+ SDR and might need
to be retuned.


## License

The firmware is licensed under the GNU General Public License, version 2 or
later (see `LICENSE`), which is the license of the Doom engine it links
against. The hardware design in `hardware/` is released under CC BY 4.0 (see
`hardware/LICENSE`).

Third-party code:

- `components/doomgeneric/` is Doom, copyright id Software, via Chocolate Doom,
  doomgeneric and the lilka project's embedded fork. GPL-2.0-or-later.
- `components/gamepad/esp_hid_gap.c` is from the ESP-IDF examples, copyright
  Espressif Systems, Unlicense or CC0-1.0.
