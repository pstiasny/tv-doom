/*
 * Silent sound carrier for the loader.
 *
 * Why: a TV auto-detects the analog standard (B/G, D/K, I) by finding the
 * sound carrier relative to the vision carrier. The game transmits it (FM
 * audio, main/sound.c), so the set locks the game instantly; the loader used
 * to transmit vision only, and the same set took ~30 s to give up hunting
 * and show the menu after every game->menu handover (2026-09-15). Same
 * carrier, no modulation, and the TV treats both images alike.
 *
 * How: the game's I2S1 1-bit bitstream on GPIO18, but as an EXACTLY periodic
 * pattern (carrier = bclk * p/q, see carrier_build_chunk) tiled into a 4 KB
 * internal-RAM chunk -- no LUT, no PSRAM. The generation mode (bit clock,
 * carrier offset, jitter-free square) and the pad drive come from SETTINGS
 * and switch live: this menu picture is the test bed for the carrier-into-
 * video artifacts (settings.h explains the modes).
 */
#include "ota_sound.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "settings.h"

#define DOUT_GPIO   GPIO_NUM_18
#define CHUNK_WORDS 1024                /* 4 KB staging: >= 102 us of bitstream */

static const char *TAG = "otasnd";
static i2s_chan_handle_t s_tx;
static uint32_t s_chunk[CHUNK_WORDS];
static size_t   s_chunk_words;
static volatile bool    s_enabled;      /* wanted state (SETTINGS menu) */
static volatile uint8_t s_mode_req, s_drive_req;
static TaskHandle_t s_task;

/* 32-bit stereo slots: BCLK = 64 x rate, MCLK = 128 x rate = 2 x BCLK. Both
 * bit clocks come off the 160 MHz PLL cleanly: 40 MHz is MCLK 80 = /2, and
 * 32 MHz is MCLK 64 = /2.5, whose 2-3-2-3 PLL-cycle pattern pairs into
 * uniform 5-cycle BCLK periods. (Other rates, e.g. 33 MHz, would leave a
 * ragged fractional pattern on the bit clock and bring the spurs back.) */
static i2s_std_clk_config_t clk_for(const carrier_mode_t *m)
{
    i2s_std_clk_config_t c = I2S_STD_CLK_DEFAULT_CONFIG(m->bclk_hz / 64);
    c.clk_src       = I2S_CLK_SRC_PLL_160M;
    c.mclk_multiple = I2S_MCLK_MULTIPLE_128;
    return c;
}

static esp_err_t i2s_bringup(const carrier_mode_t *m)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 4;
    chan_cfg.dma_frame_num = 511;         /* 4 x 511 x 8 B = 16 KB, ~3-4 ms of stream */
    chan_cfg.auto_clear    = true;        /* underrun (flash erase) -> carrier gap  */
    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx, NULL);
    if (err != ESP_OK) return err;
    i2s_std_config_t std_cfg = {
        .clk_cfg  = clk_for(m),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = I2S_GPIO_UNUSED,
                      .ws = I2S_GPIO_UNUSED, .dout = DOUT_GPIO, .din = I2S_GPIO_UNUSED },
    };
    return i2s_channel_init_std_mode(s_tx, &std_cfg);   /* enabled by the task */
}

static void log_mode(const carrier_mode_t *m)
{
    ESP_LOGI(TAG, "carrier %s: %.4f MHz @ %lu Mbit/s, %u-bit pattern, %u words/chunk%s",
             m->label, carrier_fc_hz(m) / 1e6, (unsigned long)(m->bclk_hz / 1000000),
             (unsigned)m->fc_den, (unsigned)s_chunk_words,
             m->fm ? "" : " (fixed square)");
}

static void sound_task(void *arg)
{
    uint8_t mode = s_mode_req, drive = 0xff;
    const carrier_mode_t *m = carrier_mode(mode);
    esp_err_t err = i2s_bringup(m);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S bringup failed (%s); no sound carrier", esp_err_to_name(err));
        vTaskDelete(NULL);
    }
    s_chunk_words = carrier_build_chunk(m, s_chunk, CHUNK_WORDS);
    log_mode(m);
    bool running = false;
    for (;;) {
        if (drive != s_drive_req) {
            drive = s_drive_req;
            gpio_set_drive_capability(DOUT_GPIO, (gpio_drive_cap_t)drive);
            ESP_LOGI(TAG, "GPIO18 drive %u mA", CARRIER_DRIVE_MA[drive]);
        }
        if (mode != s_mode_req || s_enabled != running) {
            if (running) { i2s_channel_disable(s_tx); running = false; }   /* DOUT parks at DC */
            if (mode != s_mode_req) {
                mode = s_mode_req;
                m = carrier_mode(mode);
                i2s_std_clk_config_t c = clk_for(m);
                err = i2s_channel_reconfig_std_clock(s_tx, &c);
                if (err != ESP_OK)
                    ESP_LOGE(TAG, "reclock failed (%s)", esp_err_to_name(err));
                s_chunk_words = carrier_build_chunk(m, s_chunk, CHUNK_WORDS);
                log_mode(m);
            }
            if (s_enabled) { i2s_channel_enable(s_tx); running = true; }
            ESP_LOGI(TAG, "sound carrier %s", running ? "ON" : "OFF");
        }
        if (!running) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    /* until a setter pokes us */
            continue;
        }
        size_t wr;
        i2s_channel_write(s_tx, s_chunk, s_chunk_words * sizeof(uint32_t), &wr,
                          portMAX_DELAY);               /* paces us */
    }
}

void ota_sound_start(bool enabled, uint8_t mode, uint8_t drive)
{
    s_enabled   = enabled;
    s_mode_req  = mode < CARRIER_MODES ? mode : 0;
    s_drive_req = drive < CARRIER_DRIVES ? drive : 2;
    xTaskCreatePinnedToCore(sound_task, "otasnd", 3072, NULL, 8, &s_task, 0);
}

static void poke(void)
{
    if (s_task) xTaskNotifyGive(s_task);   /* a write in flight ends within a few ms */
}

void ota_sound_set_enabled(bool on)      { s_enabled = on; poke(); }
void ota_sound_set_mode(uint8_t mode)    { s_mode_req = mode < CARRIER_MODES ? mode : 0; poke(); }
void ota_sound_set_drive(uint8_t drive)  { s_drive_req = drive < CARRIER_DRIVES ? drive : 2; poke(); }
