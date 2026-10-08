/* testcard.c -- RF test patterns for the Pluto regression suite. See testcard.h.
 *
 * Geometry (video.h): chroma buffer FB_W x VIDEO_H palette indices (one PAL
 * subcarrier cycle per cell), luma buffer LUMA_W x VIDEO_H brightness codes
 * 0 (black = blanking) .. VIDEO_LUMA_MAX (white). Colour comes only from the
 * palette entry (fixed U/V per index, independent of luma), which is exactly
 * what a differential-gain measurement wants: STAIRS+RED keeps the chroma
 * constant while the luma steps through the whole span. */
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "video.h"
#include "settings.h"
#include "sound.h"
#include "testcard.h"

static const char *TAG = "testcard";

#define STEPS 10                       /* luma staircase steps per line */
#define TONE_HZ 1000
#define TONE_RATE 11025                /* SOUND_INPUT_RATE (sound.c) */
#define TONE_AMP 12000                 /* ~37 % of full scale */

typedef void (*draw_fn)(uint8_t *chroma, uint8_t *luma);

static void fill(uint8_t *chroma, uint8_t *luma, uint8_t c, uint8_t l)
{
    memset(chroma, c, (size_t)FB_W * VIDEO_H);
    memset(luma, l, (size_t)LUMA_W * VIDEO_H);
}

static void draw_blank(uint8_t *c, uint8_t *l) { fill(c, l, C_BLACK, 0); }
static void draw_white(uint8_t *c, uint8_t *l) { fill(c, l, C_BLACK, VIDEO_LUMA_MAX); }
static void draw_gray(uint8_t *c, uint8_t *l)  { fill(c, l, C_BLACK, VIDEO_LUMA_MAX / 2); }

/* n equal-width luma steps across the active line, black on the left, white on
 * the right, on every line; chroma flat (colour index per card). */
static void stairs_n(uint8_t *chroma, uint8_t *luma, uint8_t colour, int n)
{
    memset(chroma, colour, (size_t)FB_W * VIDEO_H);
    uint8_t row[LUMA_W];
    for (int x = 0; x < LUMA_W; x++) {
        int step = x * n / LUMA_W;
        row[x] = (uint8_t)((step * VIDEO_LUMA_MAX + (n - 1) / 2) / (n - 1));
    }
    for (int y = 0; y < VIDEO_H; y++)
        memcpy(luma + (size_t)y * LUMA_W, row, LUMA_W);
}
static void draw_stairs(uint8_t *c, uint8_t *l)     { stairs_n(c, l, C_BLACK, STEPS); }
static void draw_stairs_red(uint8_t *c, uint8_t *l) { stairs_n(c, l, C_RED,   STEPS); }

/* CALIB: one step per luma code, so with video_set_luma_map_raw() armed the
 * line walks raw DAC codes 32..255 in 32 equal steps and a single capture
 * reads the whole code -> RF-envelope curve. Chroma off so nothing rides on
 * the steps. Measured by tools/pluto_calib.py, which regenerates
 * components/video/rfcal_default.h. */
static void draw_calib(uint8_t *c, uint8_t *l)      { stairs_n(c, l, C_BLACK, VIDEO_LUMA_CODES); }

/* Eight vertical bars from the palette (video.c PAL_RGB), luma set to each
 * colour's own Y so the bars look right on the set and the analysis can
 * compare measured chroma amplitude/phase with the palette definition. */
static void draw_bars(uint8_t *chroma, uint8_t *luma)
{
    static const uint8_t col[8]  = { C_WHITE, C_YELLOW, C_TEAL, C_GREEN, C_ORANGE, C_RED, C_BLUE, C_BLACK };
    static const uint8_t lum[8]  = { 29, 26, 7, 13, 18, 12, 13, 0 };   /* round(31*Y) */
    uint8_t crow[FB_W], lrow[LUMA_W];
    for (int x = 0; x < FB_W; x++)   crow[x] = col[x * 8 / FB_W];
    for (int x = 0; x < LUMA_W; x++) lrow[x] = lum[x * 8 / LUMA_W];
    for (int y = 0; y < VIDEO_H; y++) {
        memcpy(chroma + (size_t)y * FB_W, crow, FB_W);
        memcpy(luma + (size_t)y * LUMA_W, lrow, LUMA_W);
    }
}

/* Four horizontal bands of black/white alternation at 2, 4, 8 and 16 luma
 * pixels (2.2, 1.1, 0.55, 0.28 MHz square waves): luma bandwidth, ringing
 * and the front porch (the last column is white in every band). */
static void draw_edges(uint8_t *chroma, uint8_t *luma)
{
    memset(chroma, C_BLACK, (size_t)FB_W * VIDEO_H);
    static const int period[4] = { 2, 4, 8, 16 };
    for (int y = 0; y < VIDEO_H; y++) {
        int p = period[y * 4 / VIDEO_H];
        uint8_t *row = luma + (size_t)y * LUMA_W;
        for (int x = 0; x < LUMA_W; x++)
            row[x] = ((LUMA_W - 1 - x) / p) & 1 ? 0 : VIDEO_LUMA_MAX;
    }
}

/* `bench` cards are measurement-only and are skipped by CYCLE: they need a
 * special build-time video mode, so they are reached by holding them
 * explicitly (SERVER RC "testcard <n>" / SETTINGS -> TEST CARD). Keep them
 * LAST so the indices of the picture cards never move. */
static const struct { const char *name; draw_fn draw; bool tone; bool bench; } CARDS[] = {
    { "blank",      draw_blank,      false, false },
    { "white",      draw_white,      false, false },
    { "gray50",     draw_gray,       false, false },
    { "stairs",     draw_stairs,     false, false },
    { "stairs_red", draw_stairs_red, false, false },
    { "bars",       draw_bars,       false, false },
    { "edges",      draw_edges,      false, false },
    { "tone",       draw_gray,       true,  false },
    { "calib",      draw_calib,      false, true  },
};
#define NCARDS ((int)(sizeof CARDS / sizeof CARDS[0]))
#define NCYCLE (NCARDS - 1)            /* cards CYCLE rotates through */

/* Does this test_card setting select the raw-DAC calibration card? main.c asks
 * before video_start(), because the raw luma map has to be armed before the
 * video tables are built. */
bool testcard_is_calib(uint8_t mode)
{
    return mode >= 2 && CARDS[(mode - 2) % NCARDS].bench;
}

static void push_tone(int ms)
{
    static uint32_t phase;                  /* in samples */
    static int16_t buf[TONE_RATE / 4];       /* 250 ms */
    int n = TONE_RATE * ms / 1000;
    if (n > (int)(sizeof buf / sizeof buf[0])) n = sizeof buf / sizeof buf[0];
    for (int i = 0; i < n; i++, phase++)
        buf[i] = (int16_t)(TONE_AMP * sinf(2.0f * (float)M_PI * TONE_HZ * (float)phase / TONE_RATE));
    sound_push_s16(buf, n);
}

static void testcard_task(void *arg)
{
    uint32_t a = (uint32_t)(uintptr_t)arg;
    uint8_t mode = a & 0xff;
    bool sound_on = (a >> 8) & 1;
    int idx = mode >= 2 ? (mode - 2) % NCARDS : 0;
    ESP_LOGW(TAG, "testcard mode %u (%s): %d cards, %d ms each, sound %s",
             mode, test_card_name(mode), NCARDS, TESTCARD_HOLD_MS, sound_on ? "on" : "off");
    for (;;) {
        /* The (i/n) denominator tells tools/pluto_cards.py how many cards to
         * collect before it stops, so it must be the rotation length. */
        ESP_LOGI(TAG, "testcard: %s (%d/%d)", CARDS[idx].name, idx + 1,
                 mode == 1 ? NCYCLE : NCARDS);
        TickType_t until = xTaskGetTickCount() + pdMS_TO_TICKS(TESTCARD_HOLD_MS);
        do {
            uint8_t *chroma = video_get_frame();
            uint8_t *luma = video_luma_buffer();
            CARDS[idx].draw(chroma, luma);
            video_put_frame();
            if (CARDS[idx].tone && sound_on) push_tone(250);
            /* Bench diagnostic: the redraw rate is the producer's PSRAM write
             * burst rate (~197 KB per redraw, contending with the video DMA).
             * Build with -DTESTCARD_REDRAW_MS=1000 to slow it fourfold and see
             * whether the RF envelope disturbances follow. */
#ifndef TESTCARD_REDRAW_MS
#define TESTCARD_REDRAW_MS 250
#endif
            vTaskDelay(pdMS_TO_TICKS(TESTCARD_REDRAW_MS));
        } while ((int32_t)(until - xTaskGetTickCount()) > 0);
        if (mode == 1) idx = (idx + 1) % NCYCLE;   /* bench cards excluded */
    }
}

void testcard_start(uint8_t mode, bool sound_on)
{
    uint32_t a = mode | (sound_on ? 0x100 : 0);
    if (xTaskCreatePinnedToCore(testcard_task, "testcard", 8192, (void *)(uintptr_t)a, 5, NULL, 0) != pdPASS)
        ESP_LOGE(TAG, "task create failed");
}
