/* RF test-card generator (bench only). Replaces Doom as the frame producer
 * when SETTINGS -> TEST CARD is set (settings.h test_card): the video engine,
 * carrier, BLE log and sound stay exactly as in the game, only the picture
 * content is synthetic, so tools/pluto_regress.py can measure levels, burst,
 * linearity, colour and sound against known patterns.
 *
 * mode 1 cycles through every card, holding each TESTCARD_HOLD_MS and logging
 *   "testcard: <name> (<i>/<n>)" at every switch (the bench script keys its
 *   captures on that line); modes 2.. hold one card (still logging every hold
 *   period). sound_on: the TONE card pushes a 1 kHz tone onto the sound
 *   carrier only when the carrier is running. Never returns control to Doom. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TESTCARD_HOLD_MS 6000

/* True when this test_card setting selects the raw-DAC calibration card.
 * main.c must ask BEFORE video_start(), because the raw luma map has to be
 * armed before the video tables are built. */
bool testcard_is_calib(uint8_t mode);

void testcard_start(uint8_t mode, bool sound_on);
