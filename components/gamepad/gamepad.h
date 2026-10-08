/*
 * BLE gamepad input: connects an Xbox Series X|S controller (firmware 5.x+,
 * which advertises BLE HID) via the ESP-IDF esp_hidh host over NimBLE, parses
 * its input reports and feeds Doom's key queue (doom_push_key).
 *
 * Call gamepad_start() BEFORE video_start(): Bluetooth bring-up may write PHY
 * calibration data to flash, and flash writes stall the PSRAM cache that the
 * video DMA streams through. Once running, the only flash writes are bond
 * keys during initial pairing (hold the pad's sync button), a one-time event.
 */
#pragma once

#include <stdbool.h>

void gamepad_start(void);

/* Register a callback fired ONCE when View + Menu are held together for 3 s
 * (the OTA-mode entry combo -- the pad is the only reliable external input on
 * the assembled unit). Runs on the BLE host task: keep it non-blocking. */
void gamepad_set_combo_cb(void (*cb)(void));

/* True while the pad's HID link is up. */
bool gamepad_connected(void);

/* BLE transmit power in dBm, quantised to the controller's 3 dB grid
 * (-24..+20). Call it BEFORE gamepad_start() to have the controller come up at
 * that power, so not one advertising or scan packet goes out at the default
 * +9 dBm; calling it afterwards re-applies live. Values outside the range are
 * clamped. Lower power means a smaller current step per transmission on the
 * 3V3 rail -- see sys_settings_t.ble_tx. */
void gamepad_set_tx_dbm(int dbm);
