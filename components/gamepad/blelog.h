/* Wireless log stream over BLE GATT (the board's USB is dead; this replaces
 * the serial console). A vprintf hook mirrors every log line into a small
 * internal-RAM ring; a notify characteristic streams it to any subscribed
 * central. Host side: tools/blelog.py (bleak) -- a wireless idf.py monitor.
 *
 * Read-only by design: a command/control characteristic was tried 2026-09-15
 * and removed -- writes were unreliable under the pad scanner's duty cycle,
 * and the coexistence hacks it forced (scan deferral) locked the pad out.
 * Control stays on the pad combo and the wired jig.
 *
 * Call blelog_init() after NimBLE host config exists but BEFORE the host
 * starts (services must be registered pre-start); advertising ("tvdoom-log")
 * starts automatically once the host syncs and restarts on disconnect. */
#pragma once

#include <stdbool.h>

/* Start capturing log lines into the RAM ring (and the write-once boot
 * transcript, if BLELOG_BOOT_SZ > 0 for this app). Pure RAM, no BLE: call it
 * as the FIRST line of app_main so the whole boot sequence is captured, long
 * before the GATT service exists. Idempotent; blelog_init() calls it too. */
void blelog_capture_start(void);

void blelog_init(void);

/* Runtime switch (SETTINGS -> BLE DEBUGGING, NVS "sys"). OFF stops
 * advertising and drops a connected log client at once; ON resumes
 * advertising within ~60 ms. The GATT service stays registered either way,
 * so this is safe to call before blelog_init() / the host start (it just
 * records the wanted state) and at any time after. Default ON. */
void blelog_set_enabled(bool on);
