/*
 * PAL colour composite video output engine (ESP32-S3, RGB-LCD -> R-2R DAC,
 * 8-bit on tvplug2 / 7-bit on the prototype, inverted in the GPIO matrix for
 * PAL-B negative RF modulation).
 *
 * A dedicated task on core 1 owns the LCD peripheral and two full-frame
 * composite framebuffers in PSRAM. The LCD DMA streams a composite frame out
 * continuously and autonomously; this task only re-encodes the video region and
 * swaps buffers (zero-copy, on vsync) when the application hands over a new
 * palette frame. Because the task lives alone on core 1 and does nothing but
 * encode+present, application logic (core 0) can never interrupt the output.
 *
 * Colour timing: pclk = 4 x PAL subcarrier (divider overridden below), so one
 * subcarrier cycle = 4 samples and each framebuffer pixel maps to one cycle.
 * Chroma is QAM: C[p] = U*sin(90p) + V*cos(90p); V's sign and the burst phase
 * alternate per line (PAL). The subcarrier free-runs (270 deg/line, baked into
 * 4 line-phase tables indexed by line_counter = line & 3).
 */

#include "video.h"

#include <string.h>
#include <math.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "soc/lcd_cam_struct.h"
#include "soc/gdma_struct.h"
#include "soc/soc_caps.h"
#include "esp_rom_sys.h"

/* ---------- timing (samples at pclk = 4 x fsc, ~56.4 ns each) ------------ */

#define PCLK_HZ          17734375   /* request; divider is overridden below   */
#define H_TOTAL          1135       /* line rate 15625 Hz, fsc/fh = 283.75    */
#define V_TOTAL          312        /* 50 Hz field rate                        */

/* LCD peripheral blanking overhead (flat 0 V, outside the framebuffer). Tiny:
 * the real vsync is encoded as frame lines, and the pre-rendered PSRAM frame
 * means the DMA no longer needs a big porch to catch up. */
#define LCD_HSYNC_PULSE  1
#define LCD_HSYNC_BP     2          /* H_OVERHEAD=3 -> H_FB=1132 (mult of 4)  */
#define LCD_HSYNC_FP     0
#define LCD_VSYNC_PULSE  1
#define LCD_VSYNC_BP     1
#define LCD_VSYNC_FP     1
#define H_OVERHEAD       (LCD_HSYNC_PULSE + LCD_HSYNC_BP + LCD_HSYNC_FP)  /* 3  */
#define V_OVERHEAD       (LCD_VSYNC_PULSE + LCD_VSYNC_BP + LCD_VSYNC_FP)  /* 3  */
#define H_FB             (H_TOTAL - H_OVERHEAD)   /* 1132 samples/line       */
#define V_FB             (V_TOTAL - V_OVERHEAD)   /* 309 framebuffer lines   */

/* Horizontal layout, SYNC-first: [sync | back porch(+burst) | video | fp] */
#define H_SYNC           84
#define H_BACK_PORCH     100
#define VIDEO_OFFSET     (H_SYNC + H_BACK_PORCH)                    /* 184     */
#define H_VIDEO          920                                        /* mult 4  */
#define H_FRONT_PORCH    (H_FB - VIDEO_OFFSET - H_VIDEO)            /* 28      */

#define BURST_START      100
#define BURST_LEN        40

/* Vertical structure inside the framebuffer (proper PAL-ish vsync). */
#define HALF_FB          (H_FB / 2)     /* 566 */
#define EQ_PULSE         42             /* equalizing pulse width  (~2.4 us)  */
#define BROAD_LOW        483            /* broad pulse low width   (~27 us)   */
#define V_PRE_EQ         2
#define V_BROAD          3
#define V_POST_EQ        2
#define V_SYNC_LINES     (V_PRE_EQ + V_BROAD + V_POST_EQ)   /* 7  */
#define V_BLANK          16
#define V_VIDEO_START    (V_SYNC_LINES + V_BLANK)           /* 23 */

/* DAC codes. The framebuffer holds a NORMAL-polarity composite (sync low,
 * white high). How that becomes PAL-B negative modulation (sync tip = peak
 * carrier) is board-dependent -- see the polarity block in lcd_bringup:
 * the prototype inverts every data pin in the GPIO matrix; tvplug2 outputs
 * normal polarity and its PNP modulator (Q2, MMBT3906: lower DAC voltage =
 * more carrier) does the inversion. Either way the LCD peripheral's own
 * blanking overhead (bus forced to 0 in the H/V porches) comes out at or
 * beyond sync-tip carrier and merges into sync instead of punching
 * peak-white holes. Burst is +-1/2 sync height around blanking. Burst and
 * chroma invert together with the luma, so decoded hue is unaffected. */
#if VIDEO_BOARD_TVPLUG2
/* tvplug2: 8-bit ladder, codes ARE the DAC bytes (bit 0 is a real rung), and
 * every level is placed in ENVELOPE space, not code space -- see the rfcal_t
 * block in video.h. Each target below is permille of the code-0 carrier (the
 * modulator's saturated maximum); build_levels() maps them through the
 * measured curve to actual DAC codes.
 *
 * Why sync is at envelope 1000 (= code 0) and not, as before, at code 128:
 * the LCD peripheral forces its data bus to 0 during 3 overhead samples every
 * line and 3 overhead lines every frame. Code 0 is the saturated carrier
 * MAXIMUM, so with sync anywhere below it those samples came out ABOVE sync
 * tip -- measured 138 % of it: a 0.2 us super-sync spike on every line plus a
 * 195 us block every frame. A TV's sync slicer keys on the spike and leaves
 * the real sync sitting on its threshold (the loader hit exactly this:
 * seconds to lock, no lock after a channel change, 2026-09-15, see
 * ota_loader/main/ota_display.c). With sync AT code 0 the overhead samples
 * ARE sync level and the artefact disappears by construction.
 *
 * The old code-space constants (SYNC 128 / BLANK 176 / WHITE 255 /
 * CHROMA_GAIN 62 / BURST_AMP 24) were hand-placed on the same curve and are
 * what these replace; measured, they gave blanking at 76.9 % (PAL wants 75),
 * burst at 1.12x the sync step (wants 1.0), ~14 % differential gain from the
 * curve's compression, and hard digital clipping of saturated chroma at code
 * 255. In envelope space all four come out right by construction. */
#define DAC_SHIFT        0
#define ENV_SYNC         1000  /* sync tip = the saturated maximum (code 0)   */
#define ENV_BLANK        750   /* PAL: blanking at 75 % of sync amplitude     */
#define ENV_BURST        125   /* burst = +-1/2 the sync step around blanking */
/* Video must never reach into the band a sync slicer looks at. This caps the
 * chroma excursion of dark saturated colours at 60 % of the sync step above
 * blanking; it replaces the old SYNC_GUARD code-space clamp, which becomes
 * vacuous once sync sits at code 0. */
#define ENV_CEIL         (ENV_BLANK + (3 * (ENV_SYNC - ENV_BLANK)) / 5)
/* Chroma amplitude per unit colour saturation, as a fraction of the luma
 * span. Carried over from the prototype's chroma/luma-span ratio so hue and
 * saturation render as they did before. */
#define CHROMA_RATIO     0.785f
/* Fraction of the maximum chroma amplitude kept as headroom between peak
 * white and the modulator's floor. 1.0 = no palette colour ever reaches the
 * floor at any luma (costs contrast); 0.0 = maximum contrast, saturated
 * bright colours soft-clip against the floor.
 *
 * Set to 0.2 on 2026-09-18 from the FIRST measured curve. The static sweep had
 * predicted a floor at 20 % of the saturated maximum; the modulator actually
 * holds 26.7 %, which leaves far less room between blanking and the floor than
 * the earlier arithmetic assumed. At 0.5 the modulation depth would have come
 * out 54.7 %, WORSE than the 58.3 % the un-calibrated build already managed,
 * so the headroom is spent where the agreed priorities put it: amplitude range
 * ranks above chroma headroom. At 0.2 the depth is 60.1 %.
 *
 * What this costs: a fully saturated colour at PEAK WHITE loses most of its
 * chroma into the floor. No palette entry is used that way -- the brightest
 * saturated colour in the bar pattern is yellow at luma 26, which clears the
 * floor by 1.8 points -- but the margin is thin, so re-check chroma_headroom_ok
 * on the bars card after any change to the palette or the floor. */
#define CHROMA_HEADROOM  0.2f
#else
/* Prototype: 7-bit ladder on byte bits 7..1 (bit 0 has no rung), so each
 * byte = code << DAC_SHIFT. */
#define DAC_SHIFT        1
#define SYNC_CODE        0
#define BLANK_CODE       32
#define WHITE_CODE       127
#define CHROMA_GAIN      75.0
#define BURST_AMP        16
#define SYNC_GUARD       4
#endif

_Static_assert(H_VIDEO / 4 == FB_W, "FB_W must equal H_VIDEO/4");
_Static_assert(V_FB - V_VIDEO_START == FB_H, "FB_H must equal V_FB-V_VIDEO_START");

/* ---------- interlace geometry ------------------------------------------- */
/* A full interlaced frame packs two fields: field 1 row-aligned, field 2 offset
 * by half a line (IL_HALF samples) so its scanlines fall between field 1's. Each
 * field reuses the per-field vertical structure (IL_FROWS rows). The half-line
 * makes field 2's rows straddle framebuffer-row boundaries -> the peripheral's
 * 3-sample inter-row blank lands mid-line on field 2 (a faint notch), which is
 * unavoidable with this peripheral. IL_HALF=568 (mult of 4) keeps the colour
 * subcarrier on its 4-sample grid; field 2 then needs a +1 line_counter bias
 * (derived: field-2 row r video sits at subcarrier phase (3r+3)%4 ->
 * line_counter=(r+1)&3). */
#define IL_FROWS   V_FB                      /* 309 rows per field            */
#define IL_HALF    568                       /* half-line offset (~567.5), %4=0 */
#define IL_VRES    (2 * IL_FROWS + 1)        /* 619 framebuffer rows           */
#define IL_OFF2    ((size_t)IL_FROWS * H_FB + IL_HALF)   /* field 2 byte start  */
#define IL_FIELD2_LINE_BIAS 1                /* +1 line_counter for field 2    */

#if VIDEO_INTERLACE
#define V_RES_CFG  IL_VRES
_Static_assert(VIDEO_H == 2 * FB_H, "interlace VIDEO_H must be 2*FB_H");
#else
#define V_RES_CFG  V_FB
_Static_assert(VIDEO_H == FB_H, "progressive VIDEO_H must be FB_H");
#endif

/* subcarrier sin/cos at 0/90/180/270 deg (phase 0 at p%4==0) */
static const int SC_SIN[4] = { 0, 1, 0, -1 };
static const int SC_COS[4] = { 1, 0, -1, 0 };

/* Per-line subcarrier phase, indexed by line_counter = line & 3: the phase
 * advances 270 deg per line (the 4-sample pattern rotates one sample per line)
 * and the PAL switch negates V on even lines. */
#define LINE_PHASE_OFFSET(line_counter) (( (3*(line_counter)) & 3 ))
#define LINE_VSIGN(line_counter)        ((line_counter) & 1 ? +1 : -1)

/* Final DAC bytes for the two structural levels, derived once by
 * build_levels() before any line template is written. Runtime values because
 * on tvplug2 they come out of the calibration curve. */
static uint8_t s_sync_byte;
static uint8_t s_blank_byte;

#if VIDEO_BOARD_TVPLUG2
/* ---------- RF envelope calibration ------------------------------------- */

#include "rfcal_default.h"

static rfcal_t s_cal;            /* active curve (copied and repaired)      */
static bool    s_cal_set;        /* a curve was installed explicitly        */
static bool    s_luma_raw;       /* CALIB card: raw code staircase, no chroma */

static int   s_env_white;        /* peak-white envelope, permille           */
static int   s_env_span;         /* ENV_BLANK - s_env_white                 */
static float s_chroma_scale;     /* permille of envelope per unit saturation */

void video_set_calibration(const rfcal_t *cal)
{
    if (!cal || cal->n < 2) { s_cal_set = false; return; }
    s_cal = *cal;
    if (s_cal.n > RFCAL_MAX) s_cal.n = RFCAL_MAX;
    s_cal_set = true;
}

void video_set_luma_map_raw(bool on) { s_luma_raw = on; }

/* Envelope (permille of the code-0 level) -> DAC code. The curve is monotone
 * non-increasing in envelope, so this is a plain inverse by binary search
 * plus linear interpolation, saturating at both ends: asking for more than
 * the saturated maximum gives code 0, asking for less than the feedthrough
 * floor gives the highest calibrated code. */
static int code_for_env(int env_pm)
{
    const rfcal_t *c = &s_cal;
    if (env_pm >= c->env[0])          return c->code[0];
    if (env_pm <= c->env[c->n - 1])   return c->code[c->n - 1];

    int lo = 0, hi = c->n - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (c->env[mid] >= env_pm) lo = mid; else hi = mid;
    }
    int e0 = c->env[lo],  e1 = c->env[hi];      /* e0 >= env_pm > e1 */
    int d0 = c->code[lo], d1 = c->code[hi];
    int de = e0 - e1;
    if (de <= 0) return d0;
    return d0 + ((e0 - env_pm) * (d1 - d0) + de / 2) / de;
}

static inline uint8_t env_byte(int env_pm) { return (uint8_t)code_for_env(env_pm); }

/* Derive the working levels from the installed curve. sat_max is the largest
 * chroma amplitude in the palette (computed by build_palette before this
 * runs), expressed as |u + jv|. */
static void build_levels(float sat_max)
{
    if (!s_cal_set) { s_cal = RFCAL_DEFAULT; s_cal_set = true; }

    /* Repair to monotone non-increasing: measurement noise on the saturated
     * flat top would otherwise make the curve non-invertible. */
    for (int i = 1; i < s_cal.n; i++)
        if (s_cal.env[i] > s_cal.env[i - 1]) s_cal.env[i] = s_cal.env[i - 1];

    const int floor_pm = s_cal.env[s_cal.n - 1];   /* feedthrough floor */

    /* White sits one headroom above the floor, where the headroom is a
     * fraction of the largest chroma swing the palette can ask for. That
     * swing is itself proportional to the span, so solve for white:
     *   white = floor + k*(ENV_BLANK - white),  k = HEADROOM*RATIO*sat_max
     *   white = (floor + k*ENV_BLANK) / (1 + k) */
    const float k = CHROMA_HEADROOM * CHROMA_RATIO * sat_max;
    int white = (int)lroundf((floor_pm + k * ENV_BLANK) / (1.0f + k));

    if (white < floor_pm)        white = floor_pm;
    if (white > ENV_BLANK - 100) white = ENV_BLANK - 100;   /* keep a usable span */

    s_env_white    = white;
    s_env_span     = ENV_BLANK - white;
    s_chroma_scale = CHROMA_RATIO * (float)s_env_span;

    /* Sync is DAC code 0 by DEFINITION: it is the modulator's saturated maximum
     * and the reference the whole envelope scale is expressed in. It must NOT
     * come from code_for_env(), because the calibration staircase starts at
     * code 32 and the lookup would saturate there -- which is exactly the bug
     * that put sync 6.8 % below the peripheral's own code-0 overhead and
     * inflated every level measured against it (2026-09-18). */
    s_sync_byte  = 0;
    s_blank_byte = env_byte(ENV_BLANK);

    ESP_LOGI("video",
             "rfcal: %d pts, floor %d permille -> sync %d (code %u) blank %d (code %u) "
             "white %d (code %u), span %d, mod depth %d%%, chroma max %d",
             s_cal.n, floor_pm, ENV_SYNC, s_sync_byte, ENV_BLANK, s_blank_byte,
             s_env_white, env_byte(s_env_white), s_env_span,
             (100 * s_env_span) / ENV_BLANK,
             (int)lroundf(s_chroma_scale * sat_max));
}

#else  /* prototype: levels are plain DAC codes */

static inline int clampcode_i(int code)
{
    if (code < SYNC_CODE + SYNC_GUARD) code = SYNC_CODE + SYNC_GUARD;  /* keep video off sync tips */
    if (code > WHITE_CODE) code = WHITE_CODE;        /* chroma may clip at full scale */
    return code;
}
static inline uint8_t clampcode(int code) { return (uint8_t)(clampcode_i(code) << DAC_SHIFT); }

void video_set_calibration(const rfcal_t *cal) { (void)cal; }
void video_set_luma_map_raw(bool on)           { (void)on;  }

static void build_levels(float sat_max)
{
    (void)sat_max;
    s_sync_byte  = (uint8_t)(SYNC_CODE  << DAC_SHIFT);
    s_blank_byte = (uint8_t)(BLANK_CODE << DAC_SHIFT);
}

#endif

/* ---------- palette ----------------------------------------------------- */

/* Doom-tuned 16-colour palette. Doom is dark and brown-heavy, so we spend most
 * of the budget on a black->white ramp with real dark/mid tones plus browns and
 * reds, and only a few saturated accents. (The generic saturated palette
 * collapsed all of Doom's dark browns/greys to pure black.) See the enum in
 * video.h for the slot names. */
static const float PAL_RGB[NCOL][3] = {
    {0.00f,0.00f,0.00f},   /*  0 black      */
    {0.20f,0.20f,0.20f},   /*  1 dark grey  */
    {0.44f,0.44f,0.44f},   /*  2 mid grey   */
    {0.68f,0.68f,0.68f},   /*  3 light grey */
    {0.95f,0.95f,0.95f},   /*  4 white      */
    {0.27f,0.17f,0.09f},   /*  5 dark brown */
    {0.47f,0.31f,0.16f},   /*  6 brown      */
    {0.72f,0.53f,0.31f},   /*  7 tan        */
    {0.42f,0.09f,0.05f},   /*  8 dark red   */
    {0.86f,0.16f,0.10f},   /*  9 red        */
    {0.95f,0.49f,0.13f},   /* 10 orange     */
    {0.93f,0.83f,0.36f},   /* 11 yellow     */
    {0.44f,0.45f,0.19f},   /* 12 olive      */
    {0.23f,0.53f,0.25f},   /* 13 green      */
    {0.29f,0.41f,0.63f},   /* 14 steel blue */
    {0.15f,0.26f,0.27f},   /* 15 dark teal  */
};

/* Chroma-only per-sample offset, separate from luma so luma can be carried at
 * higher resolution. [color_index][line_counter][sample_index]: palette colour,
 * line & 3, sample within the 4-sample subcarrier cycle. On tvplug2 the
 * unit is envelope permille; on the prototype it is signed DAC codes. */
#if VIDEO_BOARD_TVPLUG2
static int16_t pal_chroma[NCOL][4][4];
#else
static int8_t  pal_chroma[NCOL][4][4];

/* Clamp LUT: final DAC byte for a given (luma code + chroma offset). Indexed by
 * (luma + chroma + CLUT_BIAS); covers luma BLANK..WHITE and chroma ~ +-33
 * with clipping margin on both ends. (tvplug2 has no equivalent: there the
 * clamp happens in envelope space and code_for_env() saturates.) */
#define CLUT_BIAS 8
#define CLUT_N    176
static uint8_t clut[CLUT_N];
#endif

/* Fully precomputed sample pairs (build-time fold of pal_chroma + clut): for a
 * (color_index, line_counter, luma code) give the two packed DAC bytes of the
 * cycle's first half (samples 0,1 use luma a) and second half (samples 2,3 use
 * luma b).
 * 2 x 4 KB in internal .bss (scales with VIDEO_LUMA_CODES) -- the encode inner
 * loop stays two 16-bit loads and an OR per cycle. Producer luma bytes are
 * abstract codes 0..VIDEO_LUMA_MAX; luma_dac() below is the ONLY place they
 * meet the DAC grid (the single function to widen for an 8-bit DAC). */
/* PSRAM .bss (EXT_RAM_BSS_ATTR): freed 8 KB of DRAM for the IRAM panic
 * handler (2026-09-15). The tables are 2 x 4 KB and encode-hot, but small
 * enough to stay resident in the 32 KB data cache, so reads still hit
 * cache; perfmon after the move: encode 15-17 ms, zero DMA stalls. An A/B
 * with them back in internal .bss (same evening) did NOT change the TV's
 * picture instability but broke BT (internal heap exhausted), so they stay
 * here; the 8 KB is what the BLE stack lives on. */
/* Two test images exist (2026-09-15 late, picture-stability hunt):
 *   B (this tree): tables in PSRAM .bss, CONFIG_ESP_PANIC_HANDLER_IRAM=y.
 *   A: plain `static` here + CONFIG_ESP_PANIC_HANDLER_IRAM not set (the
 *      handler's ~5.2 KB of DIRAM pays for the tables). User reported the
 *      DRAM-table image looked more stable vertically; A/B on the set. */
EXT_RAM_BSS_ATTR static uint16_t pair01[NCOL][4][VIDEO_LUMA_CODES];   /* B: PSRAM */
EXT_RAM_BSS_ATTR static uint16_t pair23[NCOL][4][VIDEO_LUMA_CODES];

#if VIDEO_BOARD_TVPLUG2
/* Abstract luma code (0..VIDEO_LUMA_MAX) -> ENVELOPE, permille. Linear in
 * envelope: code 0 sits at blanking (black), VIDEO_LUMA_MAX at peak white. */
static inline int luma_env(int l)
{
    return ENV_BLANK - (l * s_env_span + VIDEO_LUMA_MAX / 2) / VIDEO_LUMA_MAX;
}

/* CALIB card only: abstract luma code -> raw DAC code, spanning the FULL
 * 0..255 range in 32 steps so one capture reads the whole code->envelope
 * curve. It must reach code 0: a curve that starts higher cannot represent the
 * saturated maximum, and code_for_env() then saturates at its first entry --
 * which is how sync silently ended up at code 32 on 2026-09-18. */
static inline int luma_raw_code(int l)
{
    return (l * 255 + VIDEO_LUMA_MAX / 2) / VIDEO_LUMA_MAX;
}
#else
/* Abstract luma code (0..VIDEO_LUMA_MAX) -> DAC luma level, rounded. */
static inline int luma_dac(int l)
{
    return BLANK_CODE + (l * (WHITE_CODE - BLANK_CODE) + VIDEO_LUMA_MAX / 2)
                        / VIDEO_LUMA_MAX;
}
#endif

uint8_t video_rgb332_to_pal[256];   /* nearest palette index for an RGB332 value */

static void build_palette(void)
{
    float pu[NCOL], pv[NCOL], sat_max = 0.0f;

    for (int color_index = 0; color_index < NCOL; color_index++) {
        float r = PAL_RGB[color_index][0];
        float g = PAL_RGB[color_index][1];
        float b = PAL_RGB[color_index][2];
        float y = 0.299f * r + 0.587f * g + 0.114f * b;
        pu[color_index] = 0.492f * (b - y);
        pv[color_index] = 0.877f * (r - y);
        float sat = sqrtf(pu[color_index] * pu[color_index]
                          + pv[color_index] * pv[color_index]);
        if (sat > sat_max) sat_max = sat;
    }

    /* Levels depend on how much chroma the palette can ask for, so they are
     * derived only once sat_max is known. */
    build_levels(sat_max);

#if VIDEO_BOARD_TVPLUG2
    for (int color_index = 0; color_index < NCOL; color_index++) {
        int U  = (int)lroundf(pu[color_index] * s_chroma_scale);  /* envelope permille */
        int Vv = (int)lroundf(pv[color_index] * s_chroma_scale);
        for (int line_counter = 0; line_counter < 4; line_counter++) {
            int V = Vv * LINE_VSIGN(line_counter);
            for (int sample_index = 0; sample_index < 4; sample_index++) {
                int ph = (LINE_PHASE_OFFSET(line_counter) + sample_index) & 3;
                pal_chroma[color_index][line_counter][sample_index] =
                    (int16_t)(U * SC_SIN[ph] + V * SC_COS[ph]);
            }
        }
    }

    for (int color_index = 0; color_index < NCOL; color_index++)
        for (int line_counter = 0; line_counter < 4; line_counter++)
            for (int l = 0; l < VIDEO_LUMA_CODES; l++) {
                const int16_t *ch = pal_chroma[color_index][line_counter];
                uint8_t s[4];
                if (s_luma_raw) {
                    /* CALIB: raw code staircase, chroma suppressed. */
                    uint8_t d = (uint8_t)luma_raw_code(l);
                    s[0] = s[1] = s[2] = s[3] = d;
                } else {
                    int base = luma_env(l);
                    for (int sample_index = 0; sample_index < 4; sample_index++) {
                        int e = base + ch[sample_index];
                        /* Ceiling keeps video (chroma of dark saturated
                         * colours) out of the sync slicer's band. The low end
                         * needs no clamp: code_for_env() saturates at the
                         * modulator's floor, which is where such a sample
                         * would land anyway. */
                        if (e > ENV_CEIL) e = ENV_CEIL;
                        s[sample_index] = env_byte(e);
                    }
                }
                pair01[color_index][line_counter][l] = (uint16_t)(s[0] | s[1] << 8);
                pair23[color_index][line_counter][l] = (uint16_t)(s[2] | s[3] << 8);
            }
#else
    for (int color_index = 0; color_index < NCOL; color_index++) {
        int U  = (int)lroundf(pu[color_index] * CHROMA_GAIN);
        int Vv = (int)lroundf(pv[color_index] * CHROMA_GAIN);
        for (int line_counter = 0; line_counter < 4; line_counter++) {
            int V = Vv * LINE_VSIGN(line_counter);
            for (int sample_index = 0; sample_index < 4; sample_index++) {
                int ph = (LINE_PHASE_OFFSET(line_counter) + sample_index) & 3;
                pal_chroma[color_index][line_counter][sample_index] =
                    (int8_t)(U * SC_SIN[ph] + V * SC_COS[ph]);
            }
        }
    }
    for (int i = 0; i < CLUT_N; i++)
        clut[i] = clampcode(i - CLUT_BIAS);

    for (int color_index = 0; color_index < NCOL; color_index++)
        for (int line_counter = 0; line_counter < 4; line_counter++)
            for (int l = 0; l < VIDEO_LUMA_CODES; l++) {
                const int8_t *ch = pal_chroma[color_index][line_counter];
                int d = luma_dac(l);
                pair01[color_index][line_counter][l] =
                    (uint16_t)(clut[d + ch[0] + CLUT_BIAS]
                             | clut[d + ch[1] + CLUT_BIAS] << 8);
                pair23[color_index][line_counter][l] =
                    (uint16_t)(clut[d + ch[2] + CLUT_BIAS]
                             | clut[d + ch[3] + CLUT_BIAS] << 8);
            }
#endif
}

static void build_color_lut(void)
{
    for (int v = 0; v < 256; v++) {
        float r = ((v >> 5) & 7) / 7.0f;
        float g = ((v >> 2) & 7) / 7.0f;
        float b = ( v       & 3) / 3.0f;
        int best = 0; float bd = 1e9f;
        for (int color_index = 0; color_index < NCOL; color_index++) {
            float dr = r - PAL_RGB[color_index][0];
            float dg = g - PAL_RGB[color_index][1];
            float db = b - PAL_RGB[color_index][2];
            float d = dr*dr + dg*dg + db*db;
            if (d < bd) { bd = d; best = color_index; }
        }
        video_rgb332_to_pal[v] = (uint8_t)best;
    }
}

/* ---------- static line templates (sync/burst/blank/vsync) --------------- */
/* PSRAM (allocated in video_start): only used as memcpy sources while writing
 * the static frame structure at startup -- never touched by the encode hot
 * path. Keeping them out of internal .bss funds the bigger luma pair tables
 * (internal DRAM has only ~5 KB of static headroom). */

static uint8_t *line_eq;
static uint8_t *line_broad;
static uint8_t *line_blank[4];        /* blank line = sync + burst, per line_counter */

static void fill_sync_and_burst(uint8_t *buf, int offset, int vsign)
{
    for (int p = 0; p < H_SYNC; p++)            buf[p] = s_sync_byte;
    for (int p = H_SYNC; p < H_FB; p++)         buf[p] = s_blank_byte;
    for (int p = BURST_START; p < BURST_START + BURST_LEN; p++) {
        int ph = (offset + p) & 3;
#if VIDEO_BOARD_TVPLUG2
        /* Burst is a subcarrier of peak-to-peak = the sync step, centred on
         * blanking, built in envelope space so it stays symmetric through the
         * modulator's curvature. */
        int e = ENV_BLANK + (-ENV_BURST) * SC_SIN[ph] + (vsign * ENV_BURST) * SC_COS[ph];
        buf[p] = env_byte(e);
#else
        int c = (-BURST_AMP) * SC_SIN[ph] + (vsign * BURST_AMP) * SC_COS[ph];
        buf[p] = clampcode(BLANK_CODE + c);
#endif
    }
}

static void build_eq_line(uint8_t *buf)
{
    memset(buf,                      s_sync_byte,  EQ_PULSE);
    memset(buf + EQ_PULSE,           s_blank_byte, HALF_FB - EQ_PULSE);
    memset(buf + HALF_FB,            s_sync_byte,  EQ_PULSE);
    memset(buf + HALF_FB + EQ_PULSE, s_blank_byte, H_FB - HALF_FB - EQ_PULSE);
}

static void build_broad_line(uint8_t *buf)
{
    memset(buf,                       s_sync_byte,  BROAD_LOW);
    memset(buf + BROAD_LOW,           s_blank_byte, HALF_FB - BROAD_LOW);
    memset(buf + HALF_FB,             s_sync_byte,  BROAD_LOW);
    memset(buf + HALF_FB + BROAD_LOW, s_blank_byte, H_FB - HALF_FB - BROAD_LOW);
}

/* ---------- composite framebuffers (PSRAM, double-buffered by driver) ---- */

static esp_lcd_panel_handle_t panel;
static uint8_t *fbs[2];                     /* driver's two PSRAM frame buffers */
static SemaphoreHandle_t vsync_sem;

/* Write the static (non-video) content of one whole composite frame: the
 * encoded vsync (equalizing + broad), the VBI blank lines, and the sync+burst
 * base of every video line. Called once per FB at startup. */
static void init_fb_static(uint8_t *fb)
{
    for (int N = 0; N < V_FB; N++) {
        uint8_t *dst = fb + (size_t)N * H_FB;
        if      (N < V_PRE_EQ)             memcpy(dst, line_eq,    H_FB);
        else if (N < V_PRE_EQ + V_BROAD)   memcpy(dst, line_broad, H_FB);
        else if (N < V_SYNC_LINES)         memcpy(dst, line_eq,    H_FB);
        else                               memcpy(dst, line_blank[N & 3], H_FB);
    }
}

/* Display-row -> handoff-row map (see video_set_row_map). NULL = identity. */
static const uint16_t *row_map;

void video_set_row_map(const uint16_t *map)
{
    row_map = map;
}

static inline int mapped_row(int display_row)
{
    return row_map ? row_map[display_row] : display_row;
}

/* Encode one active line: for each subcarrier cycle k, combine the two luma
 * samples (luma[2k], luma[2k+1]) with the cycle's chroma offset, emitting the
 * 4-sample cycle as one 32-bit store. chroma has FB_W entries (palette indices),
 * luma has LUMA_W = 2*FB_W entries (codes 0..VIDEO_LUMA_MAX). */
static inline void encode_line(const uint8_t *chroma, const uint8_t *luma,
                               int line_counter, uint32_t *vid)
{
    /* Two 16-bit loads from the precomputed pair tables per subcarrier cycle
     * (see pair01/pair23) -- all clamping and chroma math folded at build time. */
    for (int k = 0; k < FB_W; k++) {
        const uint16_t (*pv01)[VIDEO_LUMA_CODES] = &pair01[chroma[k]][line_counter];
        const uint16_t (*pv23)[VIDEO_LUMA_CODES] = &pair23[chroma[k]][line_counter];
        vid[k] = (uint32_t)(*pv01)[luma[2 * k]]
               | (uint32_t)(*pv23)[luma[2 * k + 1]] << 16;
    }
}

/* Progressive: one field, display row y <- handoff row mapped_row(y). */
static void encode_video(const uint8_t *chroma_fb, const uint8_t *luma_fb, uint8_t *fb)
{
    for (int y = 0; y < FB_H; y++) {
        int N = V_VIDEO_START + y;
        int r = mapped_row(y);
        encode_line(&chroma_fb[(size_t)r * FB_W], &luma_fb[(size_t)r * LUMA_W],
                    N & 3, (uint32_t *)(fb + (size_t)N * H_FB + VIDEO_OFFSET));
    }
}

#if VIDEO_INTERLACE
/* Write one field's static structure (vsync + burst base of every line) at a
 * given linear byte base, with a per-field line_counter bias. Field 2's base is half-line
 * shifted so its lines straddle framebuffer rows -- fine, the buffer is linear. */
static void init_field_static(uint8_t *fb, size_t base, int line_bias)
{
    for (int r = 0; r < IL_FROWS; r++) {
        uint8_t *dst = fb + base + (size_t)r * H_FB;
        int line_counter = (r + line_bias) & 3;
        if      (r < V_PRE_EQ)             memcpy(dst, line_eq,    H_FB);
        else if (r < V_PRE_EQ + V_BROAD)   memcpy(dst, line_broad, H_FB);
        else if (r < V_SYNC_LINES)         memcpy(dst, line_eq,    H_FB);
        else                               memcpy(dst, line_blank[line_counter], H_FB);
    }
}

static void init_fb_static_il(uint8_t *fb)
{
    memset(fb, s_blank_byte, (size_t)IL_VRES * H_FB);   /* incl. trailing gap */
    init_field_static(fb, 0,       0);                     /* field 1: aligned   */
    init_field_static(fb, IL_OFF2, IL_FIELD2_LINE_BIAS);    /* field 2: half-line */
}

/* Encode both fields: even source rows -> field 1 (aligned), odd source rows ->
 * field 2 (half-line shifted). Source row n is displayed as scanline n. */
static void encode_video_il(const uint8_t *chroma_fb, const uint8_t *luma_fb, uint8_t *fb)
{
    for (int y = 0; y < FB_H; y++) {
        int N = V_VIDEO_START + y;
        int r1 = mapped_row(2 * y), r2 = mapped_row(2 * y + 1);  /* even/odd rows */
        encode_line(&chroma_fb[(size_t)r1 * FB_W], &luma_fb[(size_t)r1 * LUMA_W],
                    N & 3, (uint32_t *)(fb + (size_t)N * H_FB + VIDEO_OFFSET));
        encode_line(&chroma_fb[(size_t)r2 * FB_W], &luma_fb[(size_t)r2 * LUMA_W],
                    (N + IL_FIELD2_LINE_BIAS) & 3,
                    (uint32_t *)(fb + IL_OFF2 + (size_t)N * H_FB + VIDEO_OFFSET));
    }
}
#endif  /* VIDEO_INTERLACE */

/* ---------- producer handoff (single shared palette buffer) -------------- */

/* Handoff buffers (PSRAM), DOUBLE-BUFFERED: two (chroma, luma) pairs in a
 * depth-2 ring, so the renderer fills frame N+1 while the engine encodes frame
 * N -- the serial fill->encode chain collapses to max(fill, encode). chroma
 * holds FB_W*VIDEO_H palette indices; luma LUMA_W*VIDEO_H brightness codes.
 * slots_free counts pairs the renderer may fill, slots_ready pairs the engine
 * may encode; each side owns exactly the pair its index points at. */
static uint8_t *chroma_fb[2];
static uint8_t *luma_fb[2];
static int wr_slot;                     /* pair the renderer fills next    */
static int rd_slot;                     /* pair the engine encodes next    */
static SemaphoreHandle_t slots_free;    /* counting: pairs free to fill    */
static SemaphoreHandle_t slots_ready;   /* counting: pairs ready to encode */

/* Optional latency probe: per-slot tag + present callback (see video.h). The tag
 * of the frame handed to the most recent flip is published to s_flip_tag; the
 * vsync ISR (where the flip actually latches on-screen) timestamps it into a
 * single-slot mailbox that video_task drains and forwards to present_cb. Routing
 * the timestamp through the ISR makes the measured latch instant identical whether
 * or not VIDEO_VSYNC_PACE is on -- so a pace-on vs pace-off comparison is fair. */
static int64_t slot_tag[2];
static video_present_cb_t present_cb;
static volatile int64_t s_flip_tag;     /* tag of frame at the last queued flip (0 = none) */
static volatile int64_t s_latch_tag;    /* ISR->task mailbox: tag that just became visible */
static volatile int64_t s_latch_us;     /* ...and the vsync instant it latched */
static volatile bool    s_latch_full;
static volatile int64_t s_vsync_us;     /* time of the most recent frame boundary */

/* JIT encode (see video.h): exact frame period from the panel timing, and a
 * us-precision sleep (the 100 Hz FreeRTOS tick is far too coarse) built from an
 * esp_timer one-shot that notifies the video task. */
#define JIT_FRAME_US  (((int64_t)(V_RES_CFG + V_OVERHEAD) * H_TOTAL * 1000000LL) / PCLK_HZ)
#define JIT_MARGIN_US 4000              /* headroom over the rolling peak; must also
                                           beat DMA tail prefetch (~0.25 ms) */
static TaskHandle_t      s_video_task;
static esp_timer_handle_t s_jit_timer;

/* Realign / DMA-probe accounting (see VIDEO_REALIGN_PERIOD_S / VIDEO_DMA_PROBE
 * in video.h). Summarized every present-stats window for perfmon.py. */
static unsigned s_realign_n;     /* total realign requests (periodic + heal)  */
static unsigned s_dma_stalls;    /* distinct stall episodes the probe caught  */

static void jit_timer_cb(void *arg)
{
    xTaskNotifyGive(s_video_task);
}

void video_set_present_cb(video_present_cb_t cb) { present_cb = cb; }
void video_stamp_frame(int64_t tag) { slot_tag[wr_slot] = tag; }  /* wr_slot valid get..put */

uint8_t *video_get_frame(void)
{
    xSemaphoreTake(slots_free, portMAX_DELAY);
    return chroma_fb[wr_slot];
}

uint8_t *video_luma_buffer(void)
{
    return luma_fb[wr_slot];            /* valid between get_frame and put_frame */
}

void video_put_frame(void)
{
    wr_slot ^= 1;
    xSemaphoreGive(slots_ready);
}

/* vsync ISR: the just-drawn FB is now on screen, the other one is free.
 * vsync_count is the refresh heartbeat used to detect dropped/repeated frames. */
static volatile uint32_t vsync_count;

static bool IRAM_ATTR on_vsync(esp_lcd_panel_handle_t p,
                               const esp_lcd_rgb_panel_event_data_t *ed, void *ctx)
{
    vsync_count++;
    int64_t now = esp_timer_get_time();   /* IRAM-safe (ESP_TIMER_IN_IRAM) */
    s_vsync_us = now;                     /* frame-boundary anchor for JIT encode */
    /* Latency probe: the flip queued since the previous vsync latches now. Stamp
     * its tag into the mailbox for the task to log. One-deep: an unconsumed
     * sample or an untagged frame just drops. */
    int64_t t = s_flip_tag;
    if (t && !s_latch_full) {
        s_latch_tag  = t;
        s_latch_us   = now;
        s_latch_full = true;
    }
    s_flip_tag = 0;
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(vsync_sem, &hp);
    return hp == pdTRUE;
}

static void lcd_bringup(void)
{
    esp_lcd_rgb_panel_config_t panel_cfg = {
        .clk_src          = LCD_CLK_SRC_DEFAULT,
        .data_width       = 8,
        .in_color_format  = LCD_COLOR_FMT_GRAY8,
        .out_color_format = LCD_COLOR_FMT_GRAY8,
        .num_fbs          = 2,
        /* NOTE: no bounce buffers. On the S3 even DMA reads of a PSRAM frame
         * buffer go through the cache; a bounce-buffer refill would run as an
         * IRAM ISR that memcpy's from PSRAM, and if it fires while the cache is
         * disabled (any flash write on the other core) it panics with a cache
         * error. Direct DMA from PSRAM instead merely stalls (a recoverable
         * glitch) during a cache-disable window -- which we avoid entirely by
         * keeping flash writes off the hot path, and heal via
         * CONFIG_LCD_RGB_RESTART_IN_VSYNC, which re-aligns the DMA each field. */
        .dma_burst_size   = 64,          /* larger PSRAM bursts -> better bandwidth */
        .timings = {
            .pclk_hz           = PCLK_HZ,
            .h_res             = H_FB,
            .v_res             = V_RES_CFG,
            .hsync_pulse_width = LCD_HSYNC_PULSE,
            .hsync_back_porch  = LCD_HSYNC_BP,
            .hsync_front_porch = LCD_HSYNC_FP,
            .vsync_pulse_width = LCD_VSYNC_PULSE,
            .vsync_back_porch  = LCD_VSYNC_BP,
            .vsync_front_porch = LCD_VSYNC_FP,
        },
        /* R-2R ladder. No pclk output: the DAC is clocked by the data
         * transitions themselves. Codes ride byte bits 7..1 on both boards, so
         * the level tables are identical; only the pin numbers differ. */
#if VIDEO_BOARD_TVPLUG2
        /* tvplug2 (assembled): 8-bit ladder DAC_D0..D7 = IO43,42,41,40,39,38,
         * 47,48 (R19..R27 2k rungs, R28..R34 1k series, R20 2k terminator, out
         * through R27 to Q2's base). Bit 0 is a real rung and, with
         * DAC_SHIFT=0 above, carries the LSB of the full 8-bit code.
         *
         * IO43/44 are the default UART0 console pins on this board -- IO43 is
         * DAC_D0 and IO44 is MIXER_SHUTDOWN -- so the console MUST be on
         * USB-Serial/JTAG only (see sdkconfig.defaults) or every log line
         * toggles the ladder. */
        .data_gpio_nums = {
            [0]=GPIO_NUM_43,
            [1]=GPIO_NUM_42,[2]=GPIO_NUM_41,[3]=GPIO_NUM_40,[4]=GPIO_NUM_39,
            [5]=GPIO_NUM_38,[6]=GPIO_NUM_47,[7]=GPIO_NUM_48,
        },
#else
        /* Prototype: 7-bit ladder, MSB..LSB = IO10, IO47, IO9, IO11, IO12,
         * IO13, IO14; byte bit 0 has no rung.
         * (IO47, not IO46: 46 is a strapping pin, and DC bias on the ladder
         * held it high at reset, blocking download mode / flashing.) */
        .data_gpio_nums = {
            [0]=GPIO_NUM_NC,
            [1]=GPIO_NUM_14,[2]=GPIO_NUM_13,[3]=GPIO_NUM_12,[4]=GPIO_NUM_11,
            [5]=GPIO_NUM_9, [6]=GPIO_NUM_47,[7]=GPIO_NUM_10,
        },
#endif
        .pclk_gpio_num  = GPIO_NUM_NC,
        .hsync_gpio_num = GPIO_NUM_NC,
        .vsync_gpio_num = GPIO_NUM_NC,
        .de_gpio_num    = GPIO_NUM_NC,
        .disp_gpio_num  = GPIO_NUM_NC,
        .flags.fb_in_psram = true,
    };

    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_cfg, &panel));
    vsync_sem = xSemaphoreCreateBinary();
    esp_lcd_rgb_panel_event_callbacks_t cbs = { .on_vsync = on_vsync };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel, &cbs, NULL));

    /* Fill both frame buffers with valid sync/burst/blank BEFORE starting output.
     * esp_lcd_panel_init() begins streaming immediately, so if the buffers still
     * held garbage the TV could lock its colour PLL onto junk and stay stuck
     * (a "smeared bad colour" that only a reset clears). The FBs are allocated in
     * esp_lcd_new_rgb_panel(), so we can fetch and fill them now. */
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel, 2,
                                                       (void **)&fbs[0], (void **)&fbs[1]));
    size_t fb_size = (size_t)V_RES_CFG * H_FB;
#if VIDEO_INTERLACE
    init_fb_static_il(fbs[0]);
    init_fb_static_il(fbs[1]);
#else
    init_fb_static(fbs[0]);
    init_fb_static(fbs[1]);
#endif
    esp_cache_msync(fbs[0], fb_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    esp_cache_msync(fbs[1], fb_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);

    /* Signal polarity, board-dependent (FB codes are normal polarity either
     * way: sync 0, white 255).
     *
     * Prototype: the DAC feeds the diode mixer directly, so the pins must
     * emit the PAL-B inverted signal -- route them INVERTED through the GPIO
     * matrix. The peripheral's hardware blanking (bus 0, outside the FB)
     * inverts to full scale = sync-tip level and merges into sync.
     *
     * tvplug2: the video modulator Q2 is a PNP (MMBT3906) -- LOWER DAC
     * voltage = more carrier -- so the inversion happens in hardware and the
     * pins must emit NORMAL polarity (sync code 0 -> 0 V -> max carrier).
     * Hardware blanking's bus 0 is then already the sync-tip level and still
     * merges into sync. (Measured 2026-09-13 on the TinySA: with the matrix
     * inversion ON the RF came out positive-modulated, sync = min carrier.) */
#if VIDEO_BOARD_TVPLUG2
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, false));
#else
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));   /* output starts here (valid content) */

    /* Override the LCD divider onto 4 x fsc (driver's own search missed it).
     * Done immediately after init so the subcarrier is correct within microseconds
     * -- far faster than the TV's colour PLL can lock. */
    LCD_CAM.lcd_clock.lcd_clkm_div_a = 45;
    LCD_CAM.lcd_clock.lcd_clkm_div_b = 23;

    uint32_t dn = LCD_CAM.lcd_clock.lcd_clkm_div_num; if (dn == 0) dn = 256;
    uint32_t da = LCD_CAM.lcd_clock.lcd_clkm_div_a;
    uint32_t db = LCD_CAM.lcd_clock.lcd_clkm_div_b;
    uint32_t pre = LCD_CAM.lcd_clock.lcd_clkcnt_n + 1;
    double pclk = 160000000.0 / ((double)dn + (da ? (double)db / da : 0.0)) / pre;
    ESP_LOGI("video", "pclk=%.1f Hz subcarrier=%.1f Hz (fsc err %.0f Hz), fb %dx%d",
             pclk, pclk / 4, pclk / 4 - 4433618.75, FB_W, FB_H);
}

/* The video task: bring up the LCD (so its DMA/vsync interrupt lands on core 1,
 * isolated from WiFi on core 0), then forever encode+present submitted frames.
 * This is the ONLY thing on core 1, so the output is never delayed by the app. */
static void video_task(void *arg)
{
    lcd_bringup();

    /* Hold the stable black + sync + burst frame (fb 0, now at the correct 4xfsc)
     * for a moment before feeding video, giving the TV's sync and colour PLLs
     * time to lock cleanly to a correct, unchanging signal. Prevents the
     * intermittent "won't colour-lock" boot. */
    vTaskDelay(pdMS_TO_TICKS(750));

#if VIDEO_TEST_LEVELS
    /* Bench RF transfer test (see video.h): hold each code across BOTH frame
     * buffers (sync structure included -- deliberately destroyed; CW output),
     * step every 6 s, forever. Only the peripheral's own h/v blanking overhead
     * (bus 0 -> sync-tip after inversion, ~0.3% duty) still toggles. */
    {
        /* Sound-only bench test: hold the vision carrier at blank (CW) so any
         * modulation seen on the analyzer is the sound path. Swap back to the
         * dense sweep list for DAC->RF transfer runs. */
        const uint8_t codes[] = { s_blank_byte };
        /* dense transfer sweep list:
        static const uint8_t codes[] = { 0, 8, 16, 24, 32, 40, 48, 56, 64, 72,
                                         80, 88, 96, 104, 112, 120, 128, 144,
                                         160, 176, 192, 208, 224, 240, 255 }; */
        const int ncodes = (int)(sizeof codes / sizeof codes[0]);
        const size_t fb_size = (size_t)V_RES_CFG * H_FB;
        for (int i = 0;; i = (i + 1) % ncodes) {
            memset(fbs[0], codes[i], fb_size);
            memset(fbs[1], codes[i], fb_size);
            esp_cache_msync(fbs[0], fb_size,
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
            esp_cache_msync(fbs[1], fb_size,
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
            /* Step marker parsed by tools/rf_transfer.py; i/N lets it know the
             * sweep length and detect step boundaries mid-measurement. */
            ESP_LOGW("video", "testlvl: step %d/%d code %u",
                     i + 1, ncodes, (unsigned)codes[i]);
            vTaskDelay(pdMS_TO_TICKS(8000));
        }
    }
#endif

    int back = 1;                       /* fb 0 shown after init; draw into 1 */

    /* Presentation accounting: refreshes consumed per presented frame, from the
     * vsync heartbeat. delta==1 is perfect; delta==0 means the previous frame
     * was overwritten before a full scan-out (a DROP; only possible with
     * VIDEO_VSYNC_PACE off); delta>1 means the previous frame stayed on screen
     * extra refreshes (a REPEAT), attributed to the game being late (we starved
     * waiting for a frame) or the engine being late (encode+flip overran). */
    uint32_t last_present_vs = vsync_count;
    uint32_t stat_vs0 = vsync_count;
    uint32_t stat_presents = 0, stat_drops = 0, stat_rep_game = 0, stat_rep_eng = 0;

#if VIDEO_VSYNC_PACE && VIDEO_JIT_ENCODE
    s_video_task = xTaskGetCurrentTaskHandle();
    const esp_timer_create_args_t jta = { .callback = jit_timer_cb, .name = "vid_jit" };
    ESP_ERROR_CHECK(esp_timer_create(&jta, &s_jit_timer));
    int64_t jit_peak = 30000;           /* conservative start; adapts down */
    int64_t jit_sleep_acc = 0; int jit_n = 0;
#endif

    /* When VIDEO_VSYNC_PACE is off there is NO vsync wait in this loop: encode,
     * queue the flip, immediately accept the next frame (tearing possible; the
     * static sync/burst structure keeps sync and colour lock safe). With pacing
     * on, the take(vsync_sem) below aligns one present per refresh, tear-free. */
    for (;;) {
#if VIDEO_VSYNC_PACE && VIDEO_JIT_ENCODE
        /* Sleep so encode+flip lands just before the next latch (see video.h).
         * If we're already inside the budget window (late take, slow frame),
         * skip the sleep and encode immediately -- same behavior as before. */
        {
            int64_t budget = jit_peak + JIT_MARGIN_US;
            int64_t target = s_vsync_us + JIT_FRAME_US - budget;
            int64_t nowj   = esp_timer_get_time();
            if (target > nowj + 1000) {
                jit_sleep_acc += target - nowj;
                ulTaskNotifyTake(pdTRUE, 0);                  /* clear stale */
                esp_timer_start_once(s_jit_timer, target - nowj);
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60));  /* timeout: fail open */
            }
            if (++jit_n == 64) {
                ESP_LOGI("video", "jit: sleep avg %lld ms, budget %lld ms",
                         jit_sleep_acc / 64 / 1000, budget / 1000);
                jit_sleep_acc = 0; jit_n = 0;
            }
        }
#endif
        uint32_t starve_vs0 = vsync_count;
        xSemaphoreTake(slots_ready, portMAX_DELAY);   /* wait for a filled pair */
        uint32_t starved = vsync_count - starve_vs0;  /* refreshes spent waiting */
        int64_t frame_tag = slot_tag[rd_slot];        /* latency probe (0 = none) */
        int64_t enc_t0 = esp_timer_get_time();
#if VIDEO_INTERLACE
        encode_video_il(chroma_fb[rd_slot], luma_fb[rd_slot], fbs[back]);
#else
        encode_video(chroma_fb[rd_slot], luma_fb[rd_slot], fbs[back]);
#endif
        /* Temporary perf probe: average encode time, logged every 32 frames. */
        static int64_t enc_acc; static int enc_n;
        enc_acc += esp_timer_get_time() - enc_t0;
        if (++enc_n == 32) {
            ESP_LOGI("video", "encode avg %lld ms", enc_acc / enc_n / 1000);
            enc_acc = 0; enc_n = 0;
        }
        xSemaphoreGive(slots_free);                   /* renderer may refill this pair */
        rd_slot ^= 1;
#if VIDEO_VSYNC_PACE
        while (xSemaphoreTake(vsync_sem, 0) == pdTRUE) { }   /* drain stale gives */
#endif
#if VIDEO_INTERLACE
        esp_lcd_panel_draw_bitmap(panel, 0, 0, H_FB, IL_VRES, fbs[back]);  /* queue flip */
#else
        esp_lcd_panel_draw_bitmap(panel, 0, V_VIDEO_START, H_FB, V_FB, fbs[back]);
#endif
        s_flip_tag = frame_tag;   /* latency probe: this frame latches at the next vsync */
#if VIDEO_VSYNC_PACE && VIDEO_JIT_ENCODE
        /* Rolling peak of encode+flip: attack instantly, decay ~1.5%/frame, so
         * one slow frame immediately widens the budget and it narrows back slowly. */
        int64_t jit_span = esp_timer_get_time() - enc_t0;
        jit_peak -= jit_peak >> 6;
        if (jit_span > jit_peak) jit_peak = jit_span;
#endif
#if VIDEO_VSYNC_PACE
        /* Wait until the refresh heartbeat provably advances past the queued
         * flip. A bare semaphore take is racy: a vsync firing between the drain
         * above and the flip being queued leaves a give pending, so the take
         * returns for a boundary the flip MISSED -- presentation then runs one
         * refresh skewed (boot-phase dependent: stutter at a rock-steady fps).
         * The counter can't be fooled by stale gives. */
        uint32_t flip_vs = vsync_count;
        while ((int32_t)(vsync_count - flip_vs) < 1)
            xSemaphoreTake(vsync_sem, portMAX_DELAY);
#endif
        back ^= 1;

        /* Latency probe: drain the ISR mailbox (the true on-screen latch instant,
         * consistent across pace on/off) and forward it to present_cb out here,
         * where logging is safe. */
        if (s_latch_full) {
            if (present_cb)
                present_cb(s_latch_tag, s_latch_us);
            s_latch_full = false;
        }

        /* Presentation accounting (see block above the loop). */
        uint32_t now_vs = vsync_count;
        uint32_t delta = now_vs - last_present_vs;
        last_present_vs = now_vs;
        stat_presents++;
        if (delta == 0) {
            stat_drops++;
        } else if (delta > 1) {
            if (starved) stat_rep_game += delta - 1;
            else         stat_rep_eng  += delta - 1;
        }
        if (stat_presents >= 256) {          /* ~10 s at 25 Hz */
            ESP_LOGI("video", "present: %u frames / %u refreshes; drops %u, "
                     "repeats game-late %u engine-late %u",
                     (unsigned)stat_presents, (unsigned)(now_vs - stat_vs0),
                     (unsigned)stat_drops, (unsigned)stat_rep_game,
                     (unsigned)stat_rep_eng);
            stat_presents = stat_drops = stat_rep_game = stat_rep_eng = 0;
            stat_vs0 = now_vs;
            ESP_LOGI("video", "dma: stalls %u (realigns %u)",
                     s_dma_stalls, s_realign_n);
        }

#if VIDEO_REALIGN_PERIOD_S
        /* Low-rate DMA realign against permanent slips (see video.h). Logged
         * so scope observations can be correlated with realign instants. */
        {
            static int64_t next_realign_us;
            int64_t now_us = esp_timer_get_time();
            if (next_realign_us == 0) {
                next_realign_us = now_us + VIDEO_REALIGN_PERIOD_S * 1000000LL;
            } else if (now_us >= next_realign_us) {
                next_realign_us = now_us + VIDEO_REALIGN_PERIOD_S * 1000000LL;
                video_realign();
                ESP_LOGI("video", "DMA realign #%u (periodic)", s_realign_n);
            }
        }
#endif

#if VIDEO_DMA_PROBE
        /* DMA-liveness probe, 1 Hz (see video.h). The LCD engine free-runs even
         * if the GDMA data path halts (the bus then idles at 0 -> inverted ->
         * sync-tip level = "no signal"), and every fps/present stat hangs off
         * the engine's vsync heartbeat -- so a dead bus looks healthy in the
         * logs. Ground truth is the TX channel's current-descriptor pointer:
         * a live stream advances it every ~230 us (and parks at most ~190 us
         * at the frame boundary), so two reads 800 us apart MUST differ. */
        {
            static int probe_ch = -2;            /* -2 unsearched, -1 not found */
            static int64_t next_probe_us;
            static int stall_ticks;
            static bool stalled;
            int64_t now_us = esp_timer_get_time();
            if (now_us >= next_probe_us) {
                next_probe_us = now_us + 1000000;
                if (probe_ch == -2) {
                    probe_ch = -1;
                    int nch = sizeof(GDMA.channel) / sizeof(GDMA.channel[0]);
                    for (int c = 0; c < nch; c++)
                        if (GDMA.channel[c].out.peri_sel.sel == 5) {  /* 5 = LCD_CAM */
                            probe_ch = c;
                            break;
                        }
                    ESP_LOGI("video", "dma probe: LCD TX on GDMA ch %d", probe_ch);
                }
                if (probe_ch >= 0) {
                    uint32_t d1 = GDMA.channel[probe_ch].out.dscr;
                    esp_rom_delay_us(800);
                    uint32_t d2 = GDMA.channel[probe_ch].out.dscr;
                    bool park = GDMA.channel[probe_ch].out.link.park;
                    if (d1 == d2) {
                        /* 2 consecutive dead ticks before declaring: belt and
                         * braces against a pathological sampling coincidence. */
                        if (++stall_ticks >= 2) {
                            if (!stalled) {
                                stalled = true;
                                s_dma_stalls++;
                            }
                            ESP_LOGE("video", "DMA STALL: dscr frozen at 0x%08x "
                                     "(park=%d, state=0x%08x); requesting realign",
                                     (unsigned)d1, (int)park,
                                     (unsigned)GDMA.channel[probe_ch].out.state.val);
                            video_realign();
                        }
                    } else {
                        if (stalled)
                            ESP_LOGW("video", "DMA recovered: dscr moving again");
                        stalled = false;
                        stall_ticks = 0;
                    }
                }
            }
        }
#endif
    }
}

/* Request a one-shot DMA realign; the driver applies it inside the next
 * VSYNC_END interrupt (lcd_rgb_panel_try_restart_transmission). */
void video_realign(void)
{
    if (panel) {
        s_realign_n++;
        esp_lcd_rgb_panel_restart(panel);
    }
}

static volatile bool s_stopped;

void video_stop(void)
{
    if (!panel || s_stopped) return;
    s_stopped = true;
    if (s_video_task) vTaskSuspend(s_video_task);   /* no more draw_bitmap / probe */
    esp_lcd_panel_del(panel);                        /* stops the LCD GDMA */
    panel = NULL;
}

void video_start(void)
{
    line_eq    = heap_caps_malloc(H_FB, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    line_broad = heap_caps_malloc(H_FB, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(line_eq && line_broad);
    build_palette();
    build_color_lut();
    build_eq_line(line_eq);
    build_broad_line(line_broad);
    for (int line_counter = 0; line_counter < 4; line_counter++) {
        line_blank[line_counter] = heap_caps_malloc(H_FB, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        assert(line_blank[line_counter]);
        fill_sync_and_burst(line_blank[line_counter],
                            LINE_PHASE_OFFSET(line_counter), LINE_VSIGN(line_counter));
    }

    for (int i = 0; i < 2; i++) {
        chroma_fb[i] = heap_caps_malloc((size_t)FB_W * VIDEO_H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        luma_fb[i]   = heap_caps_malloc((size_t)LUMA_W * VIDEO_H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        assert(chroma_fb[i] && luma_fb[i]);
        memset(chroma_fb[i], C_BLACK, (size_t)FB_W * VIDEO_H);
        memset(luma_fb[i], 0, (size_t)LUMA_W * VIDEO_H);   /* luma code 0 = black */
    }

    slots_free  = xSemaphoreCreateCounting(2, 2);   /* both pairs fillable at boot */
    slots_ready = xSemaphoreCreateCounting(2, 0);

    xTaskCreatePinnedToCore(video_task, "video", 4096, NULL, 6, NULL, 1);
}
