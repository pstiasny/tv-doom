/* System-level settings shared by the game and the OTA loader, persisted in
 * NVS (namespace "sys"). The loader's SETTINGS menu edits them; both apps
 * read them at boot. Missing keys read as the defaults below, so a fresh NVS
 * (or an image from before the settings existed) behaves as it always did. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    bool sound_carrier;   /* 5.5 MHz sound subcarrier on GPIO18 (game: FM audio,
                           * loader: silent carrier for the TV's standard
                           * detection). Default ON. */
    bool line_sleep;      /* aerial-open deep sleep (game power monitor + wake
                           * recheck). Default OFF: ext0 wake is broken by R18,
                           * so an armed board on an open line only comes back
                           * on the recheck timer. */
    bool ble_debug;       /* BLE log service: advertise "tvdoom-log" and stream
                           * the log to a subscriber (tools/blelog.py). OFF =
                           * no advertising, no log link (the pad is unaffected;
                           * the RAM log capture keeps running). Default ON. */
    uint8_t carrier_mode; /* how the sound-carrier bitstream is generated, an
                           * index into carrier_mode(). Default 0 = as built
                           * until 2026-09-15 (32 Mbit/s DDS at 5.5 MHz). */
    uint8_t carrier_drive;/* GPIO18 pad drive, index into CARRIER_DRIVE_MA.
                           * Default 2 = 20 mA (the chip default). */
    bool server_rc;       /* loader only: the moment WiFi is up, GET the pull
                           * server's /cmd once and execute the one-shot
                           * command it returns (tools/ota.py cmd ...). For
                           * unattended bench cycles. Default OFF. */
    uint8_t ble_tx;       /* BLE transmit power, index into BLE_TX_DBM. Both apps
                           * apply it to the controller before advertising or
                           * scanning starts. Lower power shrinks the current
                           * step each transmission puts on the 3V3 rail, which
                           * is the knob for the RF envelope disturbances
                           * (measured 2026-09-18: 6.3 % deepest dip at +9 dBm,
                           * 2.7 % at -21 dBm, 1.9 % with the radio off).
                           * Default 0 = +9 dBm, the controller's own default. */
    uint8_t test_card;    /* game: 0 = play Doom; otherwise boot into the RF
                           * test-card generator (main/testcard.c) showing
                           * test_card_name(): 1 = cycle every card, 2.. = hold
                           * one. Bench only (tools/pluto_regress.py). Default 0. */
} sys_settings_t;

/* Test cards (index = test_card): 0 OFF, 1 CYCLE, then the cards themselves. */
#define TEST_CARDS 11
const char *test_card_name(uint8_t idx);     /* <= 10 chars, menu text */

/* Fill *s from NVS, initialising NVS if nobody has yet (never erases it).
 * Anything missing or unreadable falls back to the defaults. */
void settings_load(sys_settings_t *s);

/* Persist *s. Returns false if NVS is unavailable. */
bool settings_save(const sys_settings_t *s);

/* ---- sound-carrier generation modes (SETTINGS -> CARRIER) -----------------
 *
 * The carrier is a 1-bit comparator DDS on I2S1. At 32 Mbit/s its aliased odd
 * harmonics land IN the video band -- 4.5 MHz at -14 dBc, 3.5 at -20, 2.5 at
 * -23 -- as exact multiples of the 15.625 kHz line rate, i.e. stationary
 * vertical bars the LC bandpass cannot separate from 5.5 MHz. Levers, all
 * selectable live from the loader menu so they can be A/B'd on the set:
 *   40 Mbit/s   moves the strong spurs to 1.5/2.5 MHz, where the LC has
 *               ~20 dB more rejection (~10 dB less in-band after the filter);
 *               integer MCLK divider (160/2), no register poke needed.
 *   HALFLINE    carrier at 352.5 x f_line = 5.5078125 MHz: every spur becomes
 *               an odd multiple of f_line/2 -> dot-interlaced, not bars.
 *   SQUARE      bclk/6 = 5.333 MHz, 6 bits per cycle, no FM: no jitter spurs
 *               at all. Diagnostic: if the picture cleans up, the spurs are
 *               the mechanism; if not, it is edge coupling / mixer intermod.
 *               (Beware: 5.333 MHz misses the TV's 5.5 MHz sound trap, so
 *               on a COLOUR picture the untrapped carrier itself can beat
 *               with the chroma subcarrier -- it is a clean null test only
 *               on the loader's burst-less B&W picture.)
 *   DC TEST     the full 32M DDS machinery (I2S DMA, its interrupts, the
 *               game's PSRAM LUT memcpy) with every bit zero: GPIO18 holds
 *               low, no RF at all. Separates software timing effects from
 *               the signal: artifacts that survive this are not the carrier.
 * The carrier is bclk * fc_num / fc_den EXACTLY (reduced fraction), so the
 * loader can tile a seamless pattern of fc_den bits. */
typedef struct {
    const char *label;        /* menu text, <= 13 chars */
    uint32_t bclk_hz;         /* bitstream rate (I2S rate = bclk/64) */
    uint32_t fc_num, fc_den;  /* carrier = bclk * fc_num / fc_den */
    bool fm;                  /* false = fixed square, no modulation */
} carrier_mode_t;

#define CARRIER_MODES 6
const carrier_mode_t *carrier_mode(uint8_t m);      /* out of range -> mode 0 */

static inline double carrier_fc_hz(const carrier_mode_t *m)
{
    return (double)m->bclk_hz * m->fc_num / m->fc_den;
}

/* Tile the exact bit pattern for mode m into buf (MSB first per word):
 * bit n = ((n * fc_num) mod fc_den) >= fc_den/2 -- the comparator DDS with
 * exact rational phase, so the pattern repeats every fc_den bits and a
 * chunk holding a whole number of patterns loops seamlessly. Returns the
 * number of words written (<= max_words, a multiple of the pattern length). */
size_t carrier_build_chunk(const carrier_mode_t *m, uint32_t *buf, size_t max_words);

#define CARRIER_DRIVES 4
extern const uint8_t CARRIER_DRIVE_MA[CARRIER_DRIVES];   /* {5, 10, 20, 40} */

/* BLE transmit power (index = ble_tx). The controller takes -24..+20 dBm in
 * 3 dB steps; these are the useful rungs, strongest first so index 0 is the
 * stock behaviour. gamepad_set_tx_dbm() turns a dBm value into the enum. */
#define BLE_TX_LEVELS 6
extern const int8_t BLE_TX_DBM[BLE_TX_LEVELS];          /* {9, 3, -3, -9, -15, -21} */
const char *ble_tx_name(uint8_t idx);                   /* "+9 dBm" .. "-21 dBm" */
