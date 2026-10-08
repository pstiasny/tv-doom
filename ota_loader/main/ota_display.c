/*
 * Minimal black-and-white PAL display for the OTA loader.
 *
 * The game's colour pipeline streams a ~354 KB frame from PSRAM at 4 x fsc;
 * flash writes suspend the cache and starve that DMA. This display avoids
 * the problem CLASS: monochrome needs no subcarrier, so the pixel clock
 * drops to 4.000 MHz (160/40, exact) -- a 64 us line is 256 samples and a
 * full 312-line progressive frame is ~79 KB, small enough to live in
 * INTERNAL SRAM. GDMA reading internal SRAM is immune to flash-cache
 * suspension: the picture stays rock solid through every OTA write burst
 * with zero CPU involvement (the CPU may stall; the scan-out never does).
 *
 * Same DAC pins and level codes as the game (tvplug2 remap: sync 128,
 * blank/black 176, white 255, normal polarity -- Q2 inverts in hardware).
 * No burst -> the TV decodes clean B&W. Sync lives in the framebuffer data
 * like the game's engine; the peripheral's 3-sample inter-row overhead
 * (bus 0 = saturated max carrier = sync level) merges into each line sync.
 *
 * v2 (menu UI): 208 x 280 active area and a 5x7 font -> 34 columns of text,
 * enough for SSID lists and an on-screen keyboard.
 */
#include "ota_display.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "soc/gpio_num.h"

static const char *TAG = "otadisp";

/* Levels. SYNC is code 0 here, NOT the game's 128: the LCD peripheral emits
 * bus 0 during its 3 overhead samples at every line start (and its 3 overhead
 * lines per frame). On tvplug2 code 0 is the saturated carrier maximum, ~124%
 * of the code-128 sync tip, so with SYNC=128 every line began with a 0.75 us
 * super-sync spike (at 4 MHz pclk it is resolvable; the game's 0.17 us blips
 * are not) -- the TV's sync slicer took the spike as tip and the real sync sat
 * on the threshold: seconds to lock, no lock after a channel change
 * (2026-09-15). With SYNC=0 the overhead samples ARE sync level. */
#define SYNC_CODE  0
#define BLACK_CODE 176            /* blank level doubles as black */
#define WHITE_CODE 255

/* 4.000 MHz pclk, 64 us line = 256 samples: 253 in the framebuffer + the
 * peripheral's ~3-sample inter-row overhead. */
#define PCLK_HZ    4000000
#define H_FB       253
#define H_SYNC     16             /* 4.0 us + 3 overhead samples = 4.75 us  */
#define H_BP       22
/* Vertical geometry = the game's (video.c), scaled to 4 MHz: 309 framebuffer
 * rows + the peripheral's 3 blanking lines = 312 lines = 50.08 Hz, and the
 * textbook PAL vertical interval (2 equalizing + 3 broad + 2 equalizing
 * lines, all serrated at half-line pitch) so the TV's line PLL keeps its
 * reference straight through vsync. The earlier "3 long broad pulses" on a
 * 315-line frame locked from cold, but a set that had just been locked to
 * the game's raster took seconds to re-acquire it (2026-09-15). */
#define V_FB       309
#define HALF_FB    (H_FB / 2)     /* 126 */
#define EQ_PULSE   9              /* ~2.35 us */
#define BROAD_LOW  109            /* ~27.2 us */
#define V_PRE_EQ   2
#define V_BROAD    3
#define V_POST_EQ  2
#define V_SYNC_LINES (V_PRE_EQ + V_BROAD + V_POST_EQ)   /* 7 */
#define V_TOP      23             /* first active row (matches the game)    */
/* Overscan safe-area margins inside the 208 x 280 active region: the API's
 * (0,0) maps to (SAFE_X, SAFE_Y) of the active area. */
#define SAFE_X     24             /* ~12% of 208 per side */
#define SAFE_Y     28             /* ~10% of 280 per side */

static esp_lcd_panel_handle_t s_panel;
static uint8_t *s_fb;             /* internal SRAM, V_FB * H_FB */

/* Classic 5x7 column-major font (public-domain GLCD set), ASCII 32..127,
 * bit 0 = top row. */
static const uint8_t FONT[96][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5f,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7f,0x14,0x7f,0x14},{0x24,0x2a,0x7f,0x2a,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1c,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1c,0x00},{0x14,0x08,0x3e,0x08,0x14},{0x08,0x08,0x3e,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3e,0x51,0x49,0x45,0x3e},{0x00,0x42,0x7f,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},{0x18,0x14,0x12,0x7f,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3c,0x4a,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1e},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3e},
    {0x7e,0x11,0x11,0x11,0x7e},{0x7f,0x49,0x49,0x49,0x36},{0x3e,0x41,0x41,0x41,0x22},
    {0x7f,0x41,0x41,0x22,0x1c},{0x7f,0x49,0x49,0x49,0x41},{0x7f,0x09,0x09,0x09,0x01},
    {0x3e,0x41,0x49,0x49,0x7a},{0x7f,0x08,0x08,0x08,0x7f},{0x00,0x41,0x7f,0x41,0x00},
    {0x20,0x40,0x41,0x3f,0x01},{0x7f,0x08,0x14,0x22,0x41},{0x7f,0x40,0x40,0x40,0x40},
    {0x7f,0x02,0x0c,0x02,0x7f},{0x7f,0x04,0x08,0x10,0x7f},{0x3e,0x41,0x41,0x41,0x3e},
    {0x7f,0x09,0x09,0x09,0x06},{0x3e,0x41,0x51,0x21,0x5e},{0x7f,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7f,0x01,0x01},{0x3f,0x40,0x40,0x40,0x3f},
    {0x1f,0x20,0x40,0x20,0x1f},{0x3f,0x40,0x38,0x40,0x3f},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7f,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7f,0x00},{0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7f,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7f},
    {0x38,0x54,0x54,0x54,0x18},{0x08,0x7e,0x09,0x01,0x02},{0x0c,0x52,0x52,0x52,0x3e},
    {0x7f,0x08,0x04,0x04,0x78},{0x00,0x44,0x7d,0x40,0x00},{0x20,0x40,0x44,0x3d,0x00},
    {0x7f,0x10,0x28,0x44,0x00},{0x00,0x41,0x7f,0x40,0x00},{0x7c,0x04,0x18,0x04,0x78},
    {0x7c,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7c,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7c},{0x7c,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3f,0x44,0x40,0x20},{0x3c,0x40,0x40,0x20,0x7c},{0x1c,0x20,0x40,0x20,0x1c},
    {0x3c,0x40,0x30,0x40,0x3c},{0x44,0x28,0x10,0x28,0x44},{0x0c,0x50,0x50,0x50,0x3c},
    {0x44,0x64,0x54,0x4c,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7f,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00},{0x08,0x04,0x08,0x10,0x08},{0x00,0x00,0x00,0x00,0x00},
};

static inline uint8_t *pix(int x, int y)
{
    return s_fb + (size_t)(V_TOP + SAFE_Y + y) * H_FB + H_SYNC + H_BP + SAFE_X + x;
}

void ota_display_rect(int x, int y, int w, int h, bool white)
{
    if (!s_fb) return;
    uint8_t code = white ? WHITE_CODE : BLACK_CODE;
    for (int r = 0; r < h; r++) {
        int row = y + r;
        if (row < 0 || row >= OTA_DISP_H) continue;
        uint8_t *p = pix(0, row);
        for (int c = 0; c < w; c++) {
            int col = x + c;
            if (col >= 0 && col < OTA_DISP_W) p[col] = code;
        }
    }
}

void ota_display_text(int x, int y, const char *s, bool invert)
{
    if (!s_fb) return;
    for (; *s; s++, x += OTA_FONT_W) {
        unsigned ch = (unsigned char)*s;
        if (ch < 32 || ch > 127) ch = '?';
        const uint8_t *g = FONT[ch - 32];
        for (int cx = 0; cx < OTA_FONT_W; cx++) {
            int col = x + cx;
            if (col < 0 || col >= OTA_DISP_W) continue;
            uint8_t bits = cx < 5 ? g[cx] : 0;
            for (int cy = 0; cy < OTA_FONT_H; cy++) {
                int row = y + cy;
                if (row < 0 || row >= OTA_DISP_H) continue;
                bool on = cy < 7 && ((bits >> cy) & 1);
                if (invert) on = !on;
                *pix(col, row) = on ? WHITE_CODE : BLACK_CODE;
            }
        }
    }
}

void ota_display_clear(void)
{
    ota_display_rect(0, 0, OTA_DISP_W, OTA_DISP_H, false);
}

void ota_display_progress(int percent)
{
    if (!s_fb) return;
    const int x = 8, y = OTA_DISP_H - 26, w = OTA_DISP_W - 16, h = 12;
    if (percent < 0) {                        /* hide */
        ota_display_rect(x - 2, y - 2, w + 4, h + 4, false);
        return;
    }
    if (percent > 100) percent = 100;
    ota_display_rect(x - 2, y - 2, w + 4, h + 4, true);
    ota_display_rect(x, y, w, h, false);
    ota_display_rect(x, y, w * percent / 100, h, true);
}

void ota_display_init(void)
{
    esp_lcd_rgb_panel_config_t cfg = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {
            .pclk_hz = PCLK_HZ,
            .h_res = H_FB,
            .v_res = V_FB,
            /* minimal peripheral overhead; it emits bus 0 = sync level and
             * merges into the in-data sync pulses (same trick as the game) */
            .hsync_pulse_width = 1, .hsync_back_porch = 1, .hsync_front_porch = 1,
            .vsync_pulse_width = 1, .vsync_back_porch = 1, .vsync_front_porch = 1,
        },
        .data_width = 8,
        .in_color_format  = LCD_COLOR_FMT_GRAY8,
        .out_color_format = LCD_COLOR_FMT_GRAY8,
        .num_fbs = 1,
        .data_gpio_nums = {
            [0]=GPIO_NUM_43,
            [1]=GPIO_NUM_42,[2]=GPIO_NUM_41,[3]=GPIO_NUM_40,[4]=GPIO_NUM_39,
            [5]=GPIO_NUM_38,[6]=GPIO_NUM_47,[7]=GPIO_NUM_48,
        },
        .pclk_gpio_num  = GPIO_NUM_NC,
        .hsync_gpio_num = GPIO_NUM_NC,
        .vsync_gpio_num = GPIO_NUM_NC,
        .de_gpio_num    = GPIO_NUM_NC,
        .disp_gpio_num  = GPIO_NUM_NC,
        /* the whole point: framebuffer in INTERNAL SRAM */
        .flags.fb_in_psram = false,
    };
    esp_err_t err = esp_lcd_new_rgb_panel(&cfg, &s_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rgb panel init failed (%s) -- continuing headless",
                 esp_err_to_name(err));
        return;
    }
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(s_panel, 1, (void **)&s_fb));
    /* static sync/black structure: PAL vertical interval, then plain lines */
    for (int r = 0; r < V_FB; r++) {
        uint8_t *p = s_fb + (size_t)r * H_FB;
        int low;                                        /* sync samples per half line */
        if      (r < V_PRE_EQ)           low = EQ_PULSE;
        else if (r < V_PRE_EQ + V_BROAD) low = BROAD_LOW;
        else if (r < V_SYNC_LINES)       low = EQ_PULSE;
        else                             low = 0;
        if (low) {                                      /* two pulses, half-line pitch */
            memset(p,                 SYNC_CODE,  low);
            memset(p + low,           BLACK_CODE, HALF_FB - low);
            memset(p + HALF_FB,       SYNC_CODE,  low);
            memset(p + HALF_FB + low, BLACK_CODE, H_FB - HALF_FB - low);
        } else {                                        /* normal line: sync + black  */
            memset(p, SYNC_CODE, H_SYNC);
            memset(p + H_SYNC, BLACK_CODE, H_FB - H_SYNC);
        }
    }
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));   /* streaming starts here */
    ESP_LOGI(TAG, "B&W display up: %d x %d @ %d Hz pclk, %d+3 lines, fb %u B internal",
             OTA_DISP_W, OTA_DISP_H, PCLK_HZ, V_FB, (unsigned)(V_FB * H_FB));
}
