// ESP32-S3 platform layer for doomgeneric.
//
// Phase 2: DG_DrawFrame scales Doom's 320x200 XRGB frame down/up into the
// tv-plug composite video engine's 230x286 palette framebuffer, quantizing to
// the 16-colour PAL palette, and hands it off for display on a real TV.

#include <stdarg.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "doomkeys.h"
#include "doomgeneric.h"
#include "video.h"

static const char *TAG = "doom";

// Overscan-safe inset: CRT TVs crop the outer edges of the frame, so we render
// Doom into an inner region and black-border the rest, keeping the whole frame
// (including the bottom HUD) inside the visible area. Per-side margins; raise if
// edges are still cut off, lower (toward 0) if there's too much border.
// Chroma is FB_W (230) wide, luma LUMA_W (460, 2x) wide; the luma margin is 2x
// the chroma margin so both cover the same physical area. Vertical is VIDEO_H.
#define OVERSCAN_X  10                        // chroma X margin per side (of 230)
#define OVERSCAN_XL (2 * OVERSCAN_X)          // luma X margin per side  (of 460)
#define OVERSCAN_Y  (VIDEO_H * 12 / FB_H)     // Y margin per side (~4.2% of VIDEO_H)
#define ACT_Y0      OVERSCAN_Y
#define ACT_H       (VIDEO_H - 2 * OVERSCAN_Y)   // active height (rows)
#define ACT_CW      (FB_W - 2 * OVERSCAN_X)      // active chroma width (cells)
#define ACT_LW      (LUMA_W - 2 * OVERSCAN_XL)   // active luma width  (samples)

// Nearest-neighbour scale tables: source Doom column for each output sample.
// Doom is 320x200; the chroma grid (230) and luma grid (460) both stretch across
// the full 4:3 picture, so the aspect stays right. Luma at 2x resolution keeps
// Doom's horizontal (text) detail; chroma stays coarse (colour needs no detail).
//
// Vertical scaling is NOT done by the fill. The handoff holds one row per Doom
// source row (handoff row 1+r; row 0 is the shared black border row), and the
// engine stretches vertically through a display-row -> handoff-row map
// (video_set_row_map). Filling 200 unique rows instead of ~548 stretched ones
// cuts the fill cost by ~2.7x; the engine's duplicate reads stay cache-hot.
static uint16_t s_cx[ACT_CW];        // chroma column -> Doom column
static uint16_t s_lx[ACT_LW];        // luma   column -> Doom column
static uint16_t s_rowmap[VIDEO_H];   // display row   -> handoff row
static uint8_t  s_lumacode[256];     // Doom brightness 0..255 -> luma code 0..VIDEO_LUMA_MAX

// Doom's frames are 8-bit indexed; i_video.c exports the post-gamma palette and
// raises DG_PaletteChanged on every palette swap (damage/pickup tints). We fold
// the palette into two 256-byte LUTs so the fill is pure byte->byte lookups.
extern uint8_t DG_PaletteRGB[256][3];
extern volatile int DG_PaletteChanged;
static uint8_t s_idx2lum[256];       // Doom palette index -> luma code 0..VIDEO_LUMA_MAX
static uint8_t s_idx2pal[256];       // Doom palette index -> composite colour 0..15

static void rebuild_palette_luts(void)
{
    for (int i = 0; i < 256; i++) {
        int r = DG_PaletteRGB[i][0], g = DG_PaletteRGB[i][1], b = DG_PaletteRGB[i][2];
        s_idx2lum[i] = s_lumacode[(r * 77 + g * 150 + b * 29) >> 8];
        uint8_t rgb332 = (uint8_t)((r & 0xE0) | ((g & 0xE0) >> 3) | ((b & 0xC0) >> 6));
        s_idx2pal[i] = video_rgb332_to_pal[rgb332];
    }
}

// ---------------------------------------------------------------------------
// On-screen FPS counter (top-right corner). Drawn straight into the handoff
// buffers after the fill: white 3x5 digits (x2 scale) on a black box, luma-only
// detail with the chroma cells under the box forced black for a clean readout.
// ---------------------------------------------------------------------------
#define SHOW_FPS 1

#if SHOW_FPS
static const uint8_t FONT35[10][5] = {
    {07,05,05,05,07}, {02,06,02,02,07}, {07,01,07,04,07}, {07,01,07,01,07},
    {05,05,07,01,01}, {07,04,07,01,07}, {07,04,07,05,07}, {07,01,02,02,02},
    {07,05,07,05,07}, {07,05,07,01,07},
};

static void draw_fps_overlay(uint8_t *luma, uint8_t *chroma, int fps)
{
    enum { SCALE = 2, DIGIT_W = 3 * SCALE + 2, BOX_H = 5 * SCALE + 4 };
    if (fps > 99) fps = 99;
    const int box_w = 2 * DIGIT_W + 4;
    const int lx0 = ACT_LW - box_w - 4;          // luma x of box, in active coords
    const int row0 = 3;                          // Doom source row of box top

    for (int by = 0; by < BOX_H; by++) {
        uint8_t *lrow = &luma[(size_t)(1 + row0 + by) * LUMA_W + OVERSCAN_XL];
        uint8_t *crow = &chroma[(size_t)(1 + row0 + by) * FB_W + OVERSCAN_X];
        memset(lrow + lx0, 0, box_w);                              // black box
        memset(crow + lx0 / 2, C_BLACK, box_w / 2 + 1);            // no colour
    }
    int d0 = fps / 10, d1 = fps % 10;
    for (int d = 0; d < 2; d++) {
        if (d == 0 && d0 == 0) continue;                           // no leading 0
        const uint8_t *glyph = FONT35[d == 0 ? d0 : d1];
        int gx0 = lx0 + 2 + d * DIGIT_W;
        for (int gy = 0; gy < 5; gy++)
            for (int gx = 0; gx < 3; gx++) {
                if (!(glyph[gy] & (1 << (2 - gx)))) continue;
                for (int sy = 0; sy < SCALE; sy++) {
                    uint8_t *lrow = &luma[(size_t)(1 + row0 + 2 + gy * SCALE + sy)
                                          * LUMA_W + OVERSCAN_XL];
                    for (int sx = 0; sx < SCALE; sx++)
                        lrow[gx0 + gx * SCALE + sx] = VIDEO_LUMA_MAX;  // white
                }
            }
    }
}

// ---------------------------------------------------------------------------
// Battery readout, directly under the FPS box and sharing its font and style.
// Shows volts to one decimal ("4.1"). Fed from the power monitor via
// DG_SetBatteryMv(); a negative value means "no trustworthy reading" (sense
// line open, ADC not up yet) and draws nothing at all rather than a wrong
// number.
// ---------------------------------------------------------------------------
static volatile int s_batt_mv = -1;

void DG_SetBatteryMv(int mv) { s_batt_mv = mv; }

static void draw_batt_overlay(uint8_t *luma, uint8_t *chroma)
{
    enum { SCALE = 2, DIGIT_W = 3 * SCALE + 2, BOX_H = 5 * SCALE + 4,
           DOT_W = SCALE + 2, FPS_BOX_H = 5 * SCALE + 4 };
    int mv = s_batt_mv;
    if (mv < 0) return;

    int whole = mv / 1000, tenth = (mv % 1000) / 100;
    if (whole > 9) { whole = 9; tenth = 9; }            // clamp, never wrap

    const int box_w = 2 * DIGIT_W + DOT_W + 4;
    const int lx0 = ACT_LW - box_w - 4;
    const int row0 = 3 + FPS_BOX_H + 2;                 // just below the FPS box

    for (int by = 0; by < BOX_H; by++) {
        uint8_t *lrow = &luma[(size_t)(1 + row0 + by) * LUMA_W + OVERSCAN_XL];
        uint8_t *crow = &chroma[(size_t)(1 + row0 + by) * FB_W + OVERSCAN_X];
        memset(lrow + lx0, 0, box_w);
        memset(crow + lx0 / 2, C_BLACK, box_w / 2 + 1);
    }

    const int digits[2] = { whole, tenth };
    for (int d = 0; d < 2; d++) {
        const uint8_t *glyph = FONT35[digits[d]];
        int gx0 = lx0 + 2 + d * DIGIT_W + (d ? DOT_W : 0);
        for (int gy = 0; gy < 5; gy++)
            for (int gx = 0; gx < 3; gx++) {
                if (!(glyph[gy] & (1 << (2 - gx)))) continue;
                for (int sy = 0; sy < SCALE; sy++) {
                    uint8_t *lrow = &luma[(size_t)(1 + row0 + 2 + gy * SCALE + sy)
                                          * LUMA_W + OVERSCAN_XL];
                    for (int sx = 0; sx < SCALE; sx++)
                        lrow[gx0 + gx * SCALE + sx] = VIDEO_LUMA_MAX;
                }
            }
    }

    // Decimal point: a SCALE-square block on the baseline, between the digits.
    int dx0 = lx0 + 2 + DIGIT_W;
    for (int sy = 0; sy < SCALE; sy++) {
        uint8_t *lrow = &luma[(size_t)(1 + row0 + 2 + 4 * SCALE + sy)
                              * LUMA_W + OVERSCAN_XL];
        for (int sx = 0; sx < SCALE; sx++)
            lrow[dx0 + sx] = VIDEO_LUMA_MAX;
    }
}
// ---------------------------------------------------------------------------
// Die temperature, directly under the battery readout and sharing its font and
// style. Two digits, whole degrees ("36") -- the chip's own sensor is only good
// to a degree or two absolute, so a decimal would be false precision. Fed from
// the power monitor via DG_SetTempC(); INT_MIN means "no reading yet" and draws
// nothing rather than a wrong number, the same rule the battery box follows.
//
// This is the DIE temperature, several degrees above the board and not the
// same as the modulator transistors' junctions. It is here as a drift
// indicator: the modulator has no bias servo, so its transfer curve moves with
// temperature and a calibration is only valid near the temperature it was
// measured at.
// ---------------------------------------------------------------------------
static volatile int s_temp_c = INT_MIN;

void DG_SetTempC(int c) { s_temp_c = c; }

static void draw_temp_overlay(uint8_t *luma, uint8_t *chroma)
{
    enum { SCALE = 2, DIGIT_W = 3 * SCALE + 2, BOX_H = 5 * SCALE + 4,
           DOT_W = SCALE + 2, FPS_BOX_H = 5 * SCALE + 4, BATT_BOX_H = 5 * SCALE + 4 };
    int c = s_temp_c;
    if (c == INT_MIN) return;
    if (c < 0) c = 0;                                   // below freezing: show 00
    if (c > 99) c = 99;                                 // clamp, never wrap

    const int box_w = 2 * DIGIT_W + 4;
    const int lx0 = ACT_LW - box_w - 4;
    // Same stack as the battery box, one box further down. The battery box
    // reserves room for its decimal point, so this one is narrower and stays
    // right-aligned with it rather than sharing an origin.
    const int row0 = 3 + FPS_BOX_H + 2 + BATT_BOX_H + 2;

    for (int by = 0; by < BOX_H; by++) {
        uint8_t *lrow = &luma[(size_t)(1 + row0 + by) * LUMA_W + OVERSCAN_XL];
        uint8_t *crow = &chroma[(size_t)(1 + row0 + by) * FB_W + OVERSCAN_X];
        memset(lrow + lx0, 0, box_w);
        memset(crow + lx0 / 2, C_BLACK, box_w / 2 + 1);
    }

    const int digits[2] = { c / 10, c % 10 };
    for (int d = 0; d < 2; d++) {
        const uint8_t *glyph = FONT35[digits[d]];
        int gx0 = lx0 + 2 + d * DIGIT_W;
        for (int gy = 0; gy < 5; gy++)
            for (int gx = 0; gx < 3; gx++) {
                if (!(glyph[gy] & (1 << (2 - gx)))) continue;
                for (int sy = 0; sy < SCALE; sy++) {
                    uint8_t *lrow = &luma[(size_t)(1 + row0 + 2 + gy * SCALE + sy)
                                          * LUMA_W + OVERSCAN_XL];
                    for (int sx = 0; sx < SCALE; sx++)
                        lrow[gx0 + gx * SCALE + sx] = VIDEO_LUMA_MAX;
                }
            }
    }
}
#endif  /* SHOW_FPS */

static void build_scale_tables(void)
{
    for (int x = 0; x < ACT_CW; x++)
        s_cx[x] = (uint16_t)((x * DOOMGENERIC_RESX) / ACT_CW);
    // Nearest-neighbor luma columns. (A sharp-bilinear 2-tap blend was tried
    // here -- smoother column widths, but it cost ~4 ms of fill = ~3 fps and
    // added latency, for a visual gain the CRT's own spot already provides.)
    for (int x = 0; x < ACT_LW; x++)
        s_lx[x] = (uint16_t)((x * DOOMGENERIC_RESX) / ACT_LW);
    for (int y = 0; y < VIDEO_H; y++) {
        if (y < ACT_Y0 || y >= ACT_Y0 + ACT_H)
            s_rowmap[y] = 0;                     // overscan border: black row
        else
            s_rowmap[y] = (uint16_t)(1 + ((y - ACT_Y0) * DOOMGENERIC_RESY) / ACT_H);
    }
    for (int b = 0; b < 256; b++)   // 0 (black) .. VIDEO_LUMA_MAX (white), rounded
        s_lumacode[b] = (uint8_t)((b * VIDEO_LUMA_MAX + 127) / 255);
}

// ---------------------------------------------------------------------------
// DG_printf: doomgeneric's logging hook (declared in d_log.h).
// ---------------------------------------------------------------------------
void DG_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    // Strip a single trailing newline; ESP_LOGI adds its own.
    size_t n = strlen(buf);
    if (n && buf[n - 1] == '\n') buf[n - 1] = '\0';
    ESP_LOGI(TAG, "%s", buf);
}

// ---------------------------------------------------------------------------
// Input: a small ring buffer fed by push_key() (from the console/UART or,
// later, GPIO buttons), drained by DG_GetKey() which I_GetEvent() polls.
// ---------------------------------------------------------------------------
#define KEYQUEUE_SIZE 16
static volatile uint16_t s_key_queue[KEYQUEUE_SIZE];
static volatile unsigned s_key_wr = 0;
static volatile unsigned s_key_rd = 0;

// Input-latency probe: stamp the moment a key-DOWN crosses from the BLE host
// task into Doom (doom_push_key is called synchronously from handle_report, so
// this is ~the notify-RX time, within the sub-ms host-task scheduling window).
// Only one press is tracked at a time; the next DrawFrame claims it as the tag
// for the first frame presented after the press. See input_latency_cb().
static volatile int64_t s_press_us;
static volatile int     s_press_pending;

void doom_push_key(int pressed, unsigned char doom_key)
{
    if (pressed && !s_press_pending) {
        s_press_us = esp_timer_get_time();
        s_press_pending = 1;
    }
    s_key_queue[s_key_wr] = (uint16_t)((pressed ? 0x100 : 0) | doom_key);
    s_key_wr = (s_key_wr + 1) % KEYQUEUE_SIZE;
}

int DG_GetKey(int *pressed, unsigned char *doomKey)
{
    if (s_key_rd == s_key_wr) return 0;
    uint16_t v = s_key_queue[s_key_rd];
    s_key_rd = (s_key_rd + 1) % KEYQUEUE_SIZE;
    *pressed = (v & 0x100) ? 1 : 0;
    *doomKey = (unsigned char)(v & 0xFF);
    return 1;
}

// Analog gamepad state, GZDoom-style: written by the BLE input callback
// (gamepad component), added into the movement command by G_BuildTiccmd.
// Units are ticcmd units: forward/side in forwardmove steps (+-50 = run
// speed), turn in angleturn units per tic (+-1280 = vanilla fast turn).
volatile int doom_analog_forward = 0;
volatile int doom_analog_side = 0;
volatile int doom_analog_turn = 0;

// Weapon-slot helpers for the gamepad's Y = next-weapon cycling: expose the
// player's current weapon slot and ownership so the firmware can skip empty
// slots. Read-only peeks at game state from another core — benign.
#include "doomstat.h"

int doom_current_weapon_slot(void)
{
    if (gamestate != GS_LEVEL)
        return 0;
    switch (players[consoleplayer].readyweapon) {
    case wp_fist:
    case wp_chainsaw:      return 1;
    case wp_pistol:        return 2;
    case wp_shotgun:
    case wp_supershotgun:  return 3;
    case wp_chaingun:      return 4;
    case wp_missile:       return 5;
    case wp_plasma:        return 6;
    case wp_bfg:           return 7;
    default:               return 0;
    }
}

int doom_weapon_slot_owned(int slot)
{
    if (gamestate != GS_LEVEL)
        return 0;
    player_t *p = &players[consoleplayer];
    switch (slot) {
    case 1: return 1;   // fists are always there
    case 2: return p->weaponowned[wp_pistol];
    case 3: return p->weaponowned[wp_shotgun] || p->weaponowned[wp_supershotgun];
    case 4: return p->weaponowned[wp_chaingun];
    case 5: return p->weaponowned[wp_missile];
    case 6: return p->weaponowned[wp_plasma];
    case 7: return p->weaponowned[wp_bfg];
    default: return 0;
    }
}

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------
uint32_t DG_GetTicksMs(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void DG_SleepMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// ---------------------------------------------------------------------------
// Display (phase 1: instrumentation only)
// ---------------------------------------------------------------------------
// Called by the video engine (core 1) when a tagged frame latches on-screen.
// tag = the press timestamp threaded through DG_DrawFrame; the difference is the
// software-side input latency: BLE notify + tic quantization + handoff queue +
// encode + vsync-flip wait -- i.e. everything we can actually optimize. Logged as
// a rolling min/avg/max every 8 samples (i.e. every 8 button presses).
static void input_latency_cb(int64_t tag, int64_t present_us)
{
    int64_t us = present_us - tag;
    if (us < 0 || us > 500000) return;          // torn 64-bit read / stale tag: drop

    static int64_t acc, mn, mx; static int n;
    if (n == 0) { mn = us; mx = us; }
    acc += us; n++;
    if (us < mn) mn = us;
    if (us > mx) mx = us;
    if (n >= 8) {
        ESP_LOGI("inlat", "input->onscreen: min=%lld avg=%lld max=%lld ms (n=%d)",
                 mn / 1000, (acc / n) / 1000, mx / 1000, n);
        acc = 0; n = 0;
    }
}

void DG_Init(void)
{
    build_scale_tables();
    video_set_row_map(s_rowmap);
    video_set_present_cb(input_latency_cb);
    ESP_LOGI(TAG, "DG_Init: %dx%d -> chroma %dx%d + luma %dx%d, free PSRAM %u",
             DOOMGENERIC_RESX, DOOMGENERIC_RESY, FB_W, VIDEO_H, LUMA_W, VIDEO_H,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

void DG_DrawFrame(void)
{
    // Blocks until the engine has consumed the previous frame (paces us to the
    // field rate). Fill both grids: chroma (colour, 230) and luma (brightness,
    // 460 = 2x for sharper text), then hand off.
    // The overscan border is static black (cleared once in video_start) and the
    // active region below is fully overwritten every frame, so we skip the
    // ~394 KB/frame border memset -- a meaningful chunk of PSRAM bandwidth.
    int64_t t0 = esp_timer_get_time();
    uint8_t *chroma = video_get_frame();
    uint8_t *luma   = video_luma_buffer();
    int64_t t1 = esp_timer_get_time();          /* t1-t0 = blocked on engine */
    const pixel_t *src = DG_ScreenBuffer;

    // Refold the palette when Doom swaps it (also covers the first frame).
    static int luts_ready = 0;
    if (DG_PaletteChanged || !luts_ready) {
        DG_PaletteChanged = 0;
        luts_ready = 1;
        rebuild_palette_luts();
    }

    // One pass per DOOM SOURCE row (200 of them), written to handoff row 1+r --
    // the engine stretches vertically via the row map, so no row is computed
    // twice. Source pixels are palette indices; colour resolution is two byte
    // LUTs. Luma is written every sample; chroma (half the resolution, sampling
    // the same source columns: s_lx[2k]==s_cx[k]) on even samples. Margins and
    // the border row 0 were blacked once at video_start and are never touched.
    for (int sr = 0; sr < DOOMGENERIC_RESY; sr++) {
        const pixel_t *srow = &src[(size_t)sr * DOOMGENERIC_RESX];
        uint8_t *ldrow = &luma[(size_t)(1 + sr) * LUMA_W + OVERSCAN_XL];
        uint8_t *cdrow = &chroma[(size_t)(1 + sr) * FB_W + OVERSCAN_X];
        for (int x = 0; x < ACT_LW; x++) {
            uint8_t idx = srow[s_lx[x]];                     // Doom palette index
            ldrow[x] = s_idx2lum[idx];
            if (!(x & 1))                                    // chroma on even samples
                cdrow[x >> 1] = s_idx2pal[idx];
        }
    }

    static int fps_shown = 0;
#if SHOW_FPS
    draw_fps_overlay(luma, chroma, fps_shown);
    draw_batt_overlay(luma, chroma);
    draw_temp_overlay(luma, chroma);
#endif

    /* Latency probe: claim a pending press (if any) as this frame's tag. This is
     * the first frame handed off after the press, so the engine will report when
     * it reaches the screen. Read us before re-checking pending to avoid a torn
     * 64-bit read racing the host task. */
    int64_t tag = 0;
    if (s_press_pending) {
        tag = s_press_us;
        s_press_pending = 0;
    }
    video_stamp_frame(tag);

    video_put_frame();
    int64_t t2 = esp_timer_get_time();          /* t2-t1 = fill time */

    static uint32_t frames = 0, last_ms = 0;
    static int64_t wait_acc = 0, fill_acc = 0, prev_t2 = 0, gap_max = 0;
    /* Frame-gap histogram: shape of the pacing, not just the worst outlier.
     * Buckets in ms, clustered around the ~35 ms steady-state gap; dumped and
     * reset every 10 s. */
    static uint32_t gap_hist[10];
    static const char *HIST_LBL =
        "<25|25-30|30-35|35-40|40-45|45-50|50-60|60-80|80-120|>=120";
    static int hist_secs = 0;

    frames++;
    wait_acc += t1 - t0;
    fill_acc += t2 - t1;
    if (prev_t2) {
        int64_t gap = t2 - prev_t2;
        if (gap > gap_max) gap_max = gap;
        int ms = (int)(gap / 1000);
        int b = ms < 25 ? 0 : ms < 30 ? 1 : ms < 35 ? 2 : ms < 40 ? 3 :
                ms < 45 ? 4 : ms < 50 ? 5 : ms < 60 ? 6 : ms < 80 ? 7 :
                ms < 120 ? 8 : 9;
        gap_hist[b]++;
    }
    prev_t2 = t2;
    uint32_t now = DG_GetTicksMs();
    if (now - last_ms >= 1000) {
        ESP_LOGI(TAG, "display: %" PRIu32 " fps (wait %lld ms, fill %lld ms, worst gap %lld ms)",
                 frames, wait_acc / frames / 1000, fill_acc / frames / 1000, gap_max / 1000);
        fps_shown = (int)frames;
        frames = 0; wait_acc = 0; fill_acc = 0; gap_max = 0;
        last_ms = now;
        if (++hist_secs >= 10) {
            ESP_LOGI(TAG, "gap hist (%s): %u %u %u %u %u %u %u %u %u %u", HIST_LBL,
                     (unsigned)gap_hist[0], (unsigned)gap_hist[1], (unsigned)gap_hist[2],
                     (unsigned)gap_hist[3], (unsigned)gap_hist[4], (unsigned)gap_hist[5],
                     (unsigned)gap_hist[6], (unsigned)gap_hist[7], (unsigned)gap_hist[8],
                     (unsigned)gap_hist[9]);
            memset(gap_hist, 0, sizeof(gap_hist));
            hist_secs = 0;
        }
    }
}

void DG_SetWindowTitle(const char *title)
{
    if (title) ESP_LOGI(TAG, "title: %s", title);
}
