/*
 * FM sound subcarrier generator (PAL B/G intercarrier, 5.5 MHz).
 *
 * Signal chain:
 *
 *   I2S1 DOUT (1-bit stream @ 32 or 40 Mbit/s)  ->  LC bandpass ~5.5 MHz  ->
 *   attenuator (to ~-13 dBc vs peak vision)  ->  sum node with video baseband
 *   ->  diode mixer (x 48 MHz LO)  ->  antenna.
 *
 * The bitstream is a hard-switched square wave whose *fundamental* is the FM
 * carrier. The bandpass keeps the fundamental (5.5 MHz) and discards the odd
 * harmonics (16.5, 27.5 MHz...) that would otherwise mix to spurs.
 *
 * How the FM is made (the design we settled on):
 *
 *   - A phase-accumulator NCO would cost ~half a core if run per output bit at
 *     32 MHz. Instead we PRE-BAKE the bitstream: one block of BLOCK_WORDS 32-bit
 *     words (BLOCK_BITS bits, = one control-rate sample period) for every
 *     (frequency step, start-phase bucket) pair. FM at run time is then just:
 *     pick the block for the current instantaneous frequency and current phase
 *     bucket, memcpy it into the DMA chunk, and advance to the block's recorded
 *     end-phase bucket. No per-bit math on the hot path.
 *
 *   - Phase continuity across blocks (essential, or the spectrum splatters) is
 *     preserved by indexing on the start-phase bucket and threading each block's
 *     end bucket into the next lookup. N_PHASE buckets quantise the phase; the
 *     residual is the dominant spectral-purity knob (raise N_PHASE for cleaner).
 *
 *   - Frequency is quantised into N_FREQ steps across +/-DEV_HZ. That's the FM
 *     resolution knob. Both tables live in internal RAM (see _Static_assert).
 *
 * DMA: the IDF I2S std driver owns the descriptor ring. We hand it ~1 ms chunks
 * via i2s_channel_write(), which blocks on DMA space and so paces us to real
 * time; its completion interrupt fires at ~1 kHz, not per sample.
 *
 * NOTE: the FM math here is bench-validated on a scope, not in CI -- the bit
 * clock must match the I2S clock the driver actually programs (verify the
 * carrier reads 5.5 MHz), and the injection level / bandpass are external
 * analog.
 *
 * Bit clock, carrier frequency and the jitter-free diagnostic square come from
 * SETTINGS (carrier_mode(), settings.h): the 32 Mbit/s DDS puts stationary
 * spurs at 4.5/3.5/2.5 MHz into the picture; 40 Mbit/s and the half-line
 * carrier offset are the software levers against that (2026-09-15).
 */

#include "sound.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "settings.h"

/* ---------- user-tunable parameters -------------------------------------- */

#define SOUND_DOUT_GPIO   GPIO_NUM_18   /* 1-bit carrier out (free on this board) */
/* Carrier centre (nominally 5.5 MHz PAL B/G) and bit clock: from the SETTINGS
 * carrier mode, see sound_start(). */
#define SOUND_DEV_HZ      50000.0       /* peak FM deviation (+/-50 kHz)          */
#define SOUND_INPUT_RATE  11025         /* rate of samples handed to sound_push_s16
                                         * (must match MIXRATE in i_fmsound.c)      */

#define SOUND_TEST_TONE   0             /* 1: ignore input, FM a 1 kHz test tone   */
#define SOUND_TONE_HZ     1000.0
#define SOUND_TONE_LEVEL  0.8f          /* fraction of full deviation              */

#define SOUND_PREEMPH_K   1.5f          /* 50 us pre-emphasis boost (approx, tune)  */
#define SOUND_GAIN        1.0f          /* input -> deviation scale (clamped +/-1)  */

/* ---------- bitstream / block geometry ----------------------------------- */

#define CTRL_RATE_HZ      50000         /* control/FM update rate = BCLK / block bits  */
#define BLOCK_WORDS_MAX   25            /* 40 Mbit/s: 800 bits per 20 us block; 32: 640 */

#define N_FREQ            16            /* instantaneous-frequency quantisation steps */
#define SOUND_PHASE_BITS  3
#define N_PHASE           (1 << SOUND_PHASE_BITS)   /* start-phase buckets (8). 32 was
                                         * tried 2026-09-15 (less block-boundary
                                         * splatter) but quadruples the PSRAM LUT
                                         * the hot memcpy walks -- cache pressure
                                         * next to the video DMA; back to 8 until
                                         * the picture-stability work is done.  */

#define CHUNK_WORDS       1000          /* internal staging: 50 blocks @ 32 Mbit/s,
                                         * 40 @ 40 Mbit/s (~1 / 0.8 ms per write)    */

/* Runtime carrier geometry (from the SETTINGS carrier mode). */
static const carrier_mode_t *s_mode;
static double   s_bclk_hz, s_fc_hz;
static int      s_block_words;           /* 20 or 25 */
static int      s_chunk_blocks;          /* CHUNK_WORDS / s_block_words */
static uint8_t  s_drive;

/* Pre-baked pattern LUT (13-51 KB) lives in PSRAM: Doom leaves almost no
 * internal DRAM, and it's only read ~4-5 MB/s on the hot path (memcpy
 * source). The end-phase transition table and the DMA staging buffer stay
 * internal. Indexed [freq][phase][word] with a runtime block length. */
static uint32_t *s_lut;                                /* PSRAM */
static uint8_t  s_end[N_FREQ][N_PHASE];                /* end-phase bucket per block */
#define LUT_BLOCK(fi, pj) (&s_lut[((fi) * N_PHASE + (pj)) * s_block_words])

static const char *TAG = "sound";

/* ---------- pattern pre-bake --------------------------------------------- */

/* Instantaneous frequency for step fi: FC - DEV .. FC + DEV, linear. */
static double freq_of_step(int fi)
{
    return s_fc_hz - SOUND_DEV_HZ +
           (2.0 * SOUND_DEV_HZ) * fi / (double)(N_FREQ - 1);
}

/* Map an audio value in [-1,1] (deviation ratio) to a frequency step. */
static inline int step_of_audio(float a)
{
    int fi = (int)lroundf((a + 1.0f) * 0.5f * (N_FREQ - 1));
    if (fi < 0) fi = 0;
    if (fi >= N_FREQ) fi = N_FREQ - 1;
    return fi;
}

static void build_patterns(void)
{
    const uint32_t bucket_step = 1u << (32 - SOUND_PHASE_BITS);   /* 2^32 / N_PHASE */
    const uint32_t half_bucket = 1u << (31 - SOUND_PHASE_BITS);

    for (int fi = 0; fi < N_FREQ; fi++) {
        /* phase increment per output bit for this frequency (Q32) */
        uint32_t inc = (uint32_t)llround(freq_of_step(fi) / s_bclk_hz * 4294967296.0);

        for (int pj = 0; pj < N_PHASE; pj++) {
            uint32_t ph = (uint32_t)pj * bucket_step;     /* start at bucket phase */
            uint32_t *blk = LUT_BLOCK(fi, pj);

            for (int w = 0; w < s_block_words; w++) {
                uint32_t word = 0;
                for (int b = 0; b < 32; b++) {
                    word = (word << 1) | (ph >> 31);       /* MSB-first, square */
                    ph += inc;
                }
                blk[w] = word;
            }
            /* nearest end bucket -> next block starts phase-continuous */
            s_end[fi][pj] = (uint8_t)(((ph + half_bucket) >> (32 - SOUND_PHASE_BITS))
                                      & (N_PHASE - 1));
        }
    }
}

/* ---------- audio input ring (SPSC: Doom writes, sound task reads) -------- */

#define RING_SAMPLES  4096              /* ~185 ms at 22050 Hz */
/* In PSRAM (only software touches it; keeps internal DRAM for the hot LUT and the
 * DMA staging buffer, which Doom leaves almost no room for). */
static int16_t *s_ring;
static volatile uint32_t s_ring_head;   /* producer writes */
static volatile uint32_t s_ring_tail;   /* consumer reads  */

void sound_push_s16(const int16_t *samples, size_t n)
{
    if (!s_ring) return;                 /* not started yet */
    uint32_t head = s_ring_head;
    for (size_t i = 0; i < n; i++) {
        uint32_t next = (head + 1) & (RING_SAMPLES - 1);
        if (next == s_ring_tail) break;           /* full: drop the rest */
        s_ring[head] = samples[i];
        head = next;
    }
    s_ring_head = head;
}

static inline bool ring_pop(int16_t *out)
{
    uint32_t tail = s_ring_tail;
    if (tail == s_ring_head) return false;        /* empty */
    *out = s_ring[tail];
    s_ring_tail = (tail + 1) & (RING_SAMPLES - 1);
    return true;
}

/* ---------- control-rate audio -> deviation ratio ------------------------ */

/* Resample the input ring to CTRL_RATE_HZ (zero-order hold) and apply
 * pre-emphasis + gain, returning the deviation ratio in [-1,1] for one block. */
static float next_deviation(void)
{
#if SOUND_TEST_TONE
    static float tph;
    tph += (float)(2.0 * M_PI * SOUND_TONE_HZ / CTRL_RATE_HZ);
    if (tph > (float)(2.0 * M_PI)) tph -= (float)(2.0 * M_PI);
    return SOUND_TONE_LEVEL * sinf(tph);
#else
    static uint32_t acc;
    static float cur, pe_prev;

    acc += SOUND_INPUT_RATE;
    while (acc >= CTRL_RATE_HZ) {
        acc -= CTRL_RATE_HZ;
        int16_t v;
        if (ring_pop(&v)) {
            float x = v / 32768.0f;
            float e = x + SOUND_PREEMPH_K * (x - pe_prev);   /* pre-emphasis */
            pe_prev = x;
            cur = e * SOUND_GAIN;
            if (cur > 1.0f)  cur = 1.0f;
            if (cur < -1.0f) cur = -1.0f;
        } else {
            cur *= 0.98f;                 /* underflow: glide to unmodulated carrier */
        }
    }
    return cur;
#endif
}

/* ---------- I2S1 bringup + producer task --------------------------------- */

static i2s_chan_handle_t s_tx;
static uint32_t s_chunk[CHUNK_WORDS];                  /* ~1 ms staging (internal RAM) */

static esp_err_t i2s_bringup(void)
{
    /* DMA buffers must be internal DMA-capable RAM, which is scarce once BT +
     * the video LCD DMA are up -- keep this small. frame = 2 slots x 32 bit = 8 B,
     * so 4 x 256 x 8 = 8 KB, refill IRQ ~2 kHz (still light; i2s_channel_write
     * paces us regardless of buffer size). */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 2;           /* ping-pong */
    chan_cfg.dma_frame_num = 128;         /* 2 x 128 x 8 = 2 KB internal DMA, IRQ ~3.9 kHz */
    chan_cfg.auto_clear    = true;        /* emit zeros (carrier gap) on underrun */

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx, NULL);
    if (err != ESP_OK) return err;

    /* 32-bit stereo Philips: BCLK = rate*2*32 = the bitstream rate (500 kHz ->
     * 32 MHz, 625 kHz -> 40 MHz). WS/BCLK are left unrouted; only DOUT carries
     * the (contiguous) bits. */
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)s_bclk_hz / 64),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_GPIO_UNUSED,
            .ws   = I2S_GPIO_UNUSED,
            .dout = SOUND_DOUT_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { 0 },
        },
    };
    std_cfg.clk_cfg.clk_src        = I2S_CLK_SRC_PLL_160M;
    /* MCLK must be <= sclk/1.99 (~80 MHz on the 160 MHz PLL); 256x would be
     * rejected. 128x -> MCLK 64 MHz (/2.5, whose 2-3-2-3 PLL-cycle pattern pairs
     * into uniform BCLK periods) or 80 MHz (/2), bclk_div 2 -> 32 / 40 MHz. */
    std_cfg.clk_cfg.mclk_multiple  = I2S_MCLK_MULTIPLE_128;
    err = i2s_channel_init_std_mode(s_tx, &std_cfg);
    if (err != ESP_OK) return err;
    gpio_set_drive_capability(SOUND_DOUT_GPIO, (gpio_drive_cap_t)s_drive);
    return i2s_channel_enable(s_tx);
}

static void sound_task(void *arg)
{
    esp_err_t err = i2s_bringup();
    if (err != ESP_OK) {
        /* Non-fatal: leave Doom/video/clock running rather than boot-looping. */
        ESP_LOGE(TAG, "I2S bringup failed (%s); sound disabled", esp_err_to_name(err));
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "carrier up: mode %s fc=%.4f MHz bclk=%.0f MHz drive=%u mA (%s)",
             s_mode->label, s_fc_hz / 1e6, s_bclk_hz / 1e6, CARRIER_DRIVE_MA[s_drive],
             !s_mode->fm ? "fixed square, no audio" :
             SOUND_TEST_TONE ? "1 kHz test tone" : "Doom audio");

    size_t wr;
    if (!s_mode->fm) {
        /* Diagnostic: the exact jitter-free pattern, looped forever. */
        size_t words = carrier_build_chunk(s_mode, s_chunk, CHUNK_WORDS);
        for (;;)
            i2s_channel_write(s_tx, s_chunk, words * sizeof(uint32_t), &wr, portMAX_DELAY);
    }

    unsigned bucket = 0;
    const size_t chunk_bytes = (size_t)s_chunk_blocks * s_block_words * sizeof(uint32_t);
    for (;;) {
        for (int bk = 0; bk < s_chunk_blocks; bk++) {
            int fi = step_of_audio(next_deviation());
            memcpy(&s_chunk[bk * s_block_words], LUT_BLOCK(fi, bucket),
                   s_block_words * sizeof(uint32_t));
            bucket = s_end[fi][bucket];
        }
        i2s_channel_write(s_tx, s_chunk, chunk_bytes, &wr, portMAX_DELAY);  /* paces us */
    }
}

void sound_start(uint8_t carrier_mode_idx, uint8_t drive)
{
    s_mode    = carrier_mode(carrier_mode_idx);
    s_bclk_hz = s_mode->bclk_hz;
    s_fc_hz   = carrier_fc_hz(s_mode);
    s_drive   = drive < CARRIER_DRIVES ? drive : 2;
    /* One block = one control-rate period: 640 bits at 32 Mbit/s, 800 at 40. */
    s_block_words  = (int)(s_mode->bclk_hz / 32 / CTRL_RATE_HZ);
    assert(s_block_words * 32 * CTRL_RATE_HZ == s_mode->bclk_hz &&
           s_block_words <= BLOCK_WORDS_MAX && "bit clock not a multiple of the control rate");
    s_chunk_blocks = CHUNK_WORDS / s_block_words;

    if (s_mode->fm) {
        s_ring = heap_caps_malloc(RING_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
        s_lut  = heap_caps_malloc((size_t)N_FREQ * N_PHASE * s_block_words * sizeof(uint32_t),
                                  MALLOC_CAP_SPIRAM);
        assert(s_ring && s_lut && "no PSRAM for sound buffers");
        if (s_mode->fc_num)
            build_patterns();
        else    /* DC TEST: same task, DMA and memcpy load, every bit zero */
            memset(s_lut, 0, (size_t)N_FREQ * N_PHASE * s_block_words * sizeof(uint32_t));
    }
    /* Light task on core 0: it mostly blocks in i2s_channel_write; leaves core 1
     * to the video engine. Priority above Doom so the carrier never underruns. */
    xTaskCreatePinnedToCore(sound_task, "sound", 4096, NULL, 8, NULL, 0);
}
