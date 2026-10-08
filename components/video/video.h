/*
 * Video output engine: turns a palette framebuffer into a PAL colour composite
 * signal and streams it out the RGB-LCD peripheral (R-2R DAC: 8-bit on
 * tvplug2, 7-bit on the prototype). PAL-B inversion for negative RF
 * modulation is board-dependent: the prototype inverts in the GPIO matrix
 * (direct diode mixer); tvplug2 outputs normal polarity because its PNP
 * modulator stage inverts in hardware. Runs completely independently of the
 * application.
 *
 * The engine owns a dedicated task pinned to core 1. The LCD DMA streams the
 * composite frame from PSRAM autonomously, so once started the video OUTPUT
 * never stops and can never be interrupted by application logic (which lives on
 * core 0). The application only feeds it palette frames through the handoff
 * below; if the application is slow, the engine simply keeps streaming the last
 * frame.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Target board. The assembled tvplug2 and the earlier prototype have
 * completely different R-2R ladder wiring (only IO47 is common), and on
 * tvplug2 the prototype's ladder pins collide with LINE_SENSE (IO9),
 * BATTERY_SENSE (IO10) and PWR_PWM (IO13) -- driving them from the LCD
 * peripheral makes those inputs unreadable. Set to 0 to build for the
 * prototype. */
#define VIDEO_BOARD_TVPLUG2 1

/* Palette framebuffer geometry. Colour rides one PAL subcarrier cycle per
 * chroma cell (FB_W = 230). Luminance is carried at TWICE that (LUMA_W = 460,
 * two samples per cycle) so brightness/text detail is ~2x sharper horizontally
 * while staying just under the subcarrier (no colour crosstalk). FB_H is the
 * ACTIVE lines per field. */
#define FB_W   230
#define LUMA_W (2 * FB_W)   /* 460: high-res luminance, 2 samples/cycle */
#define FB_H   286
#define NCOL   16

/* Luma vocabulary: producer luma bytes are ABSTRACT brightness codes
 * 0 (black) .. VIDEO_LUMA_MAX (white); the code->DAC mapping happens once at
 * table-build time in video.c. 32 codes match Doom's light-ramp granularity;
 * the 7-bit DAC's 82-code video span renders each as a distinct level. Cost:
 * pair-LUT internal .bss scales linearly with this (32 codes = 2 x 4 KB). */
#define VIDEO_LUMA_CODES 32
#define VIDEO_LUMA_MAX   (VIDEO_LUMA_CODES - 1)

/* Interlace: draw two half-line-offset fields per frame (odd/even scanlines)
 * for double vertical resolution. When on, the producer fills FB_W*VIDEO_H
 * (VIDEO_H = 2*FB_H) rows; row n is displayed scanline n (fields interleaved).
 * When off, it's progressive 288p and VIDEO_H = FB_H. */
#define VIDEO_INTERLACE 0

#if VIDEO_INTERLACE
#define VIDEO_H (2 * FB_H)     /* 572 interleaved source rows */
#else
#define VIDEO_H FB_H           /* 286 progressive rows */
#endif

/* Pace presentation to the display refresh (25 Hz interlaced / 50 Hz
 * progressive): each encoded frame is shown for exactly one refresh, and ring
 * backpressure paces the producer to the same rate. Caps fps at the refresh
 * rate but removes the producer-vs-display beat (judder) and tearing. Safe now
 * that encode (~27 ms) fits within one 40 ms refresh; 0 = free-run (fastest). */
#define VIDEO_VSYNC_PACE 1

/* Periodic DMA realign (seconds; 0 = off). The LCD DMA streams straight from
 * PSRAM; any cache-off window (flash write, e.g. a BLE bond store) can stall a
 * read, drain the LCD FIFO, and leave the data stream PERMANENTLY displaced a
 * few samples against the peripheral's line timing -- every line then carries
 * a ~3-sample sync-tip pre-pulse (the inter-line blanking overhead, no longer
 * merged into sync) a few hundred ns before the real sync edge, until reboot.
 * CONFIG_LCD_RGB_RESTART_IN_VSYNC would heal this every frame, but it glitches
 * a field whenever its ISR runs late (and its restart link is hardwired to
 * fb 0, fighting double buffering), so it stays OFF; instead we request a
 * restart (esp_lcd_rgb_panel_restart, applied inside the next VSYNC_END) at a
 * low rate: worst case one shifted/stale frame per realign vs a permanent
 * slip. video_realign() forces one on demand (e.g. after a known flash write).
 *
 * 2026-09-13: periodic mode DISABLED. On a real TV the signal dropped out
 * every few seconds, matching the 5 s cadence (and matching the earlier
 * "long no-signal" scope episode): a restart can apparently WEDGE the DMA
 * (suspect: stale owner bits on the driver's reused restart link,
 * check_owner=true), leaving the output dead until the liveness probe's
 * self-heal realign lands up to ~2 s later. The realign machinery stays for
 * on-demand use (probe self-heal + video_realign after known flash writes);
 * the cosmetic post-slip pre-pulse is preferable to periodic dropouts. */
#define VIDEO_REALIGN_PERIOD_S 0

/* DMA-liveness probe (1 = on). The LCD engine's timing generator free-runs
 * even if the GDMA data path halts, so fps/present stats (all driven by the
 * vsync heartbeat) look healthy while the output sits at blanking level ("no
 * signal", sync-tip after inversion). The probe samples the LCD TX channel's
 * current-descriptor pointer twice, 800 us apart, once per second: a live
 * stream must advance it (~230 us/descriptor). Two dead ticks -> "DMA STALL"
 * ERROR log + counted + a realign requested as self-heal; recovery is logged.
 * Adds a "dma: stalls N (realigns M)" summary line each present window
 * (parsed by tools/perfmon.py). */
#define VIDEO_DMA_PROBE 1

/* RF static-level test mode (bench only, 1 = on). Replaces ALL video output
 * with a constant DAC code across the entire framebuffer -- no sync, no burst,
 * just a fixed carrier drive level -- and steps through a code list every few
 * seconds, logging "testlvl: code N". With the GPIO-matrix inversion the RF
 * envelope is a pure CW tone at the 48 MHz LO whose amplitude maps that code
 * (code 0 = sync-tip drive = max carrier, 255 = min), so a spectrum analyzer
 * on the RF output reads the full static DAC->RF transfer curve (modulation
 * depth, mixer linearity, white-clip headroom). Normal encode/present never
 * runs; the game blocks harmlessly on the frame ring. Driven by
 * tools/rf_transfer.py. */
#define VIDEO_TEST_LEVELS 0

/* Just-in-time encode (needs VIDEO_VSYNC_PACE): the engine sleeps FIRST so
 * encode+flip completes just before the latch, betting that ring backpressure
 * shifts the game's tic sampling later -> fresher input. MEASURED A NO-OP here
 * (avg 87 -> 85 ms, max slightly worse): the game never blocks on the ring
 * ("wait 0 ms" -- it is producer-limited, ~27 ms tics+render + 13 ms fill per
 * 40 ms cycle), so there is no backpressure to propagate and the sleep only
 * relocates queue idle. Worth re-enabling ONLY if the game ever becomes
 * consumer-paced (i.e. game-side wait becomes nonzero). */
#define VIDEO_JIT_ENCODE 0

/* Palette colour indices (RGB values live in video.c). Doom-tuned ramp: a
 * black->white greyscale plus browns/reds and a few accents. */
enum { C_BLACK, C_DGRAY, C_GRAY, C_LGRAY, C_WHITE, C_DBROWN, C_BROWN, C_TAN,
       C_DRED, C_RED, C_ORANGE, C_YELLOW, C_OLIVE, C_GREEN, C_BLUE, C_TEAL };

/* Nearest palette index for each RGB332 value; built during video_start().
 * Read-only for the renderer (maps arbitrary colours to the 16-colour palette). */
extern uint8_t video_rgb332_to_pal[256];

/* ---- RF modulator calibration (tvplug2) ---------------------------------
 * The Q2/Q1 modulator's DAC-code -> RF-envelope transfer is strongly
 * nonlinear (saturated flat top below code ~48, half slope to ~128, fully
 * linear only above) and, with no bias servo on either transistor, it drifts
 * with temperature. Rather than hand-placing levels on that curve in CODE
 * space, video.c composes every sample in ENVELOPE space and maps it through
 * this measured curve, so luma, chroma and burst are all linear in RF
 * envelope and the PAL ratios hold exactly.
 *
 * Envelope is permille of the code-0 carrier level -- the saturated maximum,
 * which is also exactly what the LCD peripheral's forced-0 overhead samples
 * produce. Putting sync at code 0 therefore makes that overhead sync level
 * instead of the super-sync spikes it was with sync at code 128.
 *
 * Measure with the CALIB test card + tools/pluto_calib.py, which regenerates
 * main/rfcal_default.h. */
#define RFCAL_MAX 40
typedef struct {
    uint8_t  n;                 /* breakpoints in use, 2..RFCAL_MAX          */
    uint8_t  code[RFCAL_MAX];   /* ascending DAC codes                       */
    uint16_t env[RFCAL_MAX];    /* envelope, permille of the code-0 level    */
} rfcal_t;

/* Install a measured curve. MUST be called before video_start(); not calling
 * it (or passing NULL) uses the compiled-in default. The curve is copied and
 * repaired to be monotone non-increasing, so measurement noise on the
 * saturated flat top cannot produce a non-invertible table. */
void video_set_calibration(const rfcal_t *cal);

/* Bench only: build the luma tables as a RAW DAC-code staircase
 * (code = 32 + l*223/31 for luma code l) with chroma forced to zero, so a
 * capture of the CALIB test card reads the code->envelope curve directly.
 * Sync, blanking and burst keep their normal envelope targets. Must be called
 * before video_start(). */
void video_set_luma_map_raw(bool on);

/* Bring up the LCD and start the video task (core 1). Returns once the engine
 * is running; the first video_get_frame() blocks until it's ready. */
void video_start(void);

/* Request a one-shot LCD DMA realign (applied in the next VSYNC_END; see
 * VIDEO_REALIGN_PERIOD_S above). Safe from any task once video is running. */
void video_realign(void);

/* Quiesce the output engine before a software reset: parks the video task
 * and deletes the LCD panel (stops its GDMA -- the only autonomous PSRAM
 * master). On the ESP32-S3 a reset issued while that DMA is mid-burst on the
 * PSRAM leaves the shared SPI bus in a state the ROM bootloader cannot read
 * flash through: esp_restart() then only survives via its 1 s RTC-WDT rescue,
 * and a deep-sleep restart hangs until power-cycled (2026-09-15). */
void video_stop(void);

/*
 * Producer handoff. The application draws one frame per cycle into TWO buffers:
 *
 *     uint8_t *chroma = video_get_frame();     // FB_W  * VIDEO_H palette indices
 *     uint8_t *luma   = video_luma_buffer();   // LUMA_W * VIDEO_H luma codes 5..15
 *     ... write chroma[] palette indices 0..NCOL-1, luma[] brightness codes ...
 *     video_put_frame();                       // hand off to display
 *
 * Colour (hue/sat) comes from the chroma buffer's palette entry; brightness
 * comes from the luma buffer at double horizontal resolution. The handoff is a
 * depth-2 ring: video_get_frame() returns immediately while a spare pair
 * exists, so the producer can fill frame N+1 while the engine encodes frame N;
 * it blocks only when both pairs are in flight. video_luma_buffer() is valid
 * between get_frame and put_frame. Luma code 0 = black (blank level),
 * VIDEO_LUMA_MAX = white.
 */
uint8_t *video_get_frame(void);      /* chroma buffer (blocks for a free frame) */
uint8_t *video_luma_buffer(void);    /* luma buffer (valid after video_get_frame) */
void     video_put_frame(void);

/* Optional display-row -> handoff-row indirection (NULL = identity). With a map,
 * the producer fills only UNIQUE rows (e.g. one per Doom source row plus a
 * shared black border row) and the engine applies vertical scaling by reading
 * the mapped handoff row for each of the VIDEO_H display rows -- duplicated rows
 * then cost a cache-hot re-read instead of a recompute. map must have VIDEO_H
 * entries, each < VIDEO_H, and stay valid until changed. */
void video_set_row_map(const uint16_t *map);

/* Optional latency probe. The producer may tag the frame it is currently filling
 * with an opaque 64-bit value (an esp_timer timestamp, say) via video_stamp_frame()
 * between get_frame and put_frame. When that frame provably latches on-screen (right
 * after the vsync flip), the engine invokes the registered present callback with the
 * tag and the latch time -- so (present_us - tag) is the producer-visible latency up
 * to the frame reaching the screen. A tag of 0 means "no sample" and never fires the
 * callback. Both are no-ops with zero cost when no callback is registered. */
typedef void (*video_present_cb_t)(int64_t tag, int64_t present_us);
void video_set_present_cb(video_present_cb_t cb);
void video_stamp_frame(int64_t tag);
