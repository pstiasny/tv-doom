/* Unmodulated PAL B/G sound carrier for the loader. See ota_sound.c. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Start the carrier task with the SETTINGS values; the stream runs only
 * while enabled. mode indexes carrier_mode() (settings.h), drive indexes
 * CARRIER_DRIVE_MA. */
void ota_sound_start(bool enabled, uint8_t mode, uint8_t drive);

/* Live switches (SETTINGS menu), each effective within a few ms:
 * OFF stops the I2S stream and parks GPIO18 at a DC level, ON restarts it;
 * a mode change reclocks I2S1 and rebuilds the pattern; drive sets the pad
 * current (edge rate). */
void ota_sound_set_enabled(bool on);
void ota_sound_set_mode(uint8_t mode);
void ota_sound_set_drive(uint8_t drive);
