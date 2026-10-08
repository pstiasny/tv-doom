/*
 * PAL intercarrier sound: a 5.5 MHz FM subcarrier, emitted as a 1-bit bitstream
 * on a spare GPIO and summed (externally) into the video baseband ahead of the
 * diode mixer. The existing 48 MHz LO then upconverts it alongside the vision
 * carrier, landing it 5.5 MHz above vision -- exactly where a PAL B/G TV's
 * intercarrier sound detector expects it.
 *
 * The carrier is a hard-switched square wave whose fundamental is the FM sound
 * carrier; an external LC bandpass turns it into a sine and sets the ~-13 dBc
 * sound/vision level (see sound.c header for the signal chain).
 *
 * Independent of everything else: I2S1 + its own GDMA channel, separate from the
 * LCD_CAM video path and the I2S0 MCLK clock generator. GPIO18 is otherwise
 * unused.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/* Bring up I2S1 and start streaming the (initially unmodulated) carrier from a
 * producer task on core 0. carrier_mode indexes carrier_mode() (settings.h:
 * bit clock, carrier offset, or the fixed jitter-free square that takes no
 * audio); drive indexes CARRIER_DRIVE_MA for the GPIO18 pad. */
void sound_start(uint8_t carrier_mode, uint8_t drive);

/* Feed mono audio to be FM-modulated onto the carrier. Samples are signed 16-bit
 * at SOUND_INPUT_RATE (see sound.c). Safe to call from another task (Doom's
 * audio backend); non-blocking, drops samples if the internal ring is full. */
void sound_push_s16(const int16_t *samples, size_t n);
