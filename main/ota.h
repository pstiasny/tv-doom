/*
 * Over-the-air updates, game side.
 *
 * Why: the assembled unit's USB connection is unreliable and BOOT/reset are
 * unreachable through the enclosure, so USB flashing cannot be the primary
 * update path. The actual updater is the ota_loader app in the factory slot
 * (WiFi AP + HTTP; see ota_loader/) -- the WiFi stack's static DRAM does not
 * fit in THIS image next to BT + video, and running the update with no
 * video/PSRAM/BT alive removes the flash-write-vs-PSRAM-DMA conflict
 * entirely. The game merely hands over:
 *
 *   hold View + Menu on the pad 3 s  ->  reboot into the loader's on-TV
 *   menu (B&W internal-SRAM display + the same BLE pad): pick a network,
 *   type the password on the on-screen keyboard (saved in NVS after the
 *   first join), then UPDATE pulls the image from the HTTP server that
 *   tools/ota.py runs on the Mac
 *
 * Bootloader rollback: a new game image that dies before ota_mark_healthy()
 * falls back to the loader on the next reset -- always re-pushable, never
 * bricked in the enclosure.
 */
#pragma once

/* Reboot into the OTA loader (does not return on success). */
void ota_request(void);

/* Call FIRST in app_main: counts crash-class resets in RTC RAM and, on the
 * 4th in a row, boots the factory loader (UART-free recovery from any
 * crash-looping build). Healthy uptime of 60 s clears the streak. */
void ota_bootloop_guard(void);

/* Rollback handshake: call ONCE after the full bring-up succeeded (video,
 * pad, monitor). First boot of a new image runs in PENDING_VERIFY; this call
 * marks it valid. If the image dies before reaching this, the next reset
 * boots the previous slot. No-op on images not pending verification. */
void ota_mark_healthy(void);
