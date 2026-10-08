/*
 * OTA loader (the "factory" app): join-the-network updater with an on-TV
 * menu. Brings up the B&W internal-SRAM display, the BLE gamepad and WiFi
 * (STA); the menu (ota_menu.c) lets the user pick a network, type the
 * password with an on-screen keyboard, and pull a new game image from the
 * hardcoded HTTP server into ota_0. No PSRAM streaming runs here, so the
 * flash writes have nothing to disturb -- the picture stays solid.
 *
 * How we get here: the game's pad combo (View+Menu 3 s) sets the boot
 * partition to factory and reboots; a game image that dies before
 * validating (bootloader rollback) also lands here. Both roads end at a
 * re-pushable board, which is the whole point.
 *
 * Leaving: menu item RUN GAME, or -- with SETTINGS -> SERVER RC on -- a
 * one-shot "run"/"update" command fetched from the pull server as soon as
 * WiFi is up (ota_menu.c). (A 10-min input-idle auto-boot of the game
 * existed until 2026-09-15; dropped at the user's request -- entry is a
 * deliberate Xbox-button press now, and the timeout kept yanking the board
 * out of the menu mid-diagnosis.)
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "clkout.h"
#include "gamepad.h"
#include "ota_display.h"
#include "ota_menu.h"
#include "ota_sound.h"
#include "settings.h"
#include "blelog.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "soc/i2s_struct.h"
#include "soc/gdma_struct.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_sig_map.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "loader";

static volatile int64_t s_last_activity;   /* last pad input (informational) */

void ota_touch_activity(void)
{
    s_last_activity = esp_timer_get_time();
}

void boot_game(void)
{
    const esp_partition_t *game = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (game && esp_ota_set_boot_partition(game) == ESP_OK) {
        ESP_LOGW(TAG, "booting game (ota_0)");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }
    ESP_LOGE(TAG, "no bootable game image in ota_0 -- staying in loader");
}

/* Stubs for blelog's game-side wake breadcrumbs (loader has no wake path). */
uint32_t power_wake_n, power_wake_done;

void app_main(void)
{
    blelog_capture_start();      /* capture the whole boot into the transcript */
    ESP_LOGW(TAG, "OTA loader up (reset reason %d, free internal %u KB)",
             (int)esp_reset_reason(),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    /* When STAGED into ota_0 (a loader update in transit), this boot runs
     * PENDING_VERIFY under bootloader rollback: mark it valid immediately,
     * or the next reset silently reverts to the old factory loader. */
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (run && esp_ota_get_state_partition(run, &st) == ESP_OK &&
        st == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGW(TAG, "running STAGED from %s -- use INSTALL LOADER->FACTORY",
                 run->label);
    }

    /* NVS first: NimBLE bond keys (shared with the game -- the pad connects
     * without re-pairing) and our saved WiFi credentials live there. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* System settings (NVS "sys", edited in this menu's SETTINGS page and
     * shared with the game). */
    sys_settings_t settings;
    settings_load(&settings);
    blelog_set_enabled(settings.ble_debug);   /* before gamepad_start() */

    /* RF + status picture: carrier, mixer rail, converter in forced-PWM, and
     * the internal-SRAM B&W display that keeps scanning through every flash
     * write burst. */
    gpio_config_t pwm = { .pin_bit_mask = 1ULL << 13, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&pwm);
    gpio_set_level(13, 1);
    clkout_start_48mhz();
    mixer_set_enabled(true);
    ota_display_init();
    ota_sound_start(settings.sound_carrier, settings.carrier_mode,
                    settings.carrier_drive);   /* silent 5.5 MHz carrier: TV standard detection */

    /* Wireless loader update: the OLD loader pulled this image into ota_0 as
     * if it were the game and rebooted into it. Install ourselves to factory
     * and restart from there (no UART, no jig). */
    if (run && run->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0)
        ota_menu_install_to_factory();      /* never returns */

    /* Input (BLE pad, bonds shared with the game) and the WiFi/update menu.
     * SETTINGS -> BLE TX POWER, same setting the game uses, applied before the
     * controller comes up. WiFi power is deliberately left alone: the OTA
     * download depends on it. */
    gamepad_set_tx_dbm(BLE_TX_DBM[settings.ble_tx]);
    gamepad_start();
    ota_menu_start();

    s_last_activity = esp_timer_get_time();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        /* Heartbeat: the physical state a late log subscriber needs -- mixer
         * gate read-back (contention shows as a non-0 level), carrier clock
         * registers, WiFi/pad link state. */
        wifi_ap_record_t ap;
        bool wifi = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
        /* LCD DMA liveness: the GDMA TX channel wired to LCD_CAM (peri_sel 5)
         * must advance its descriptor pointer within one line time (~60 us
         * at 4 MHz pclk). Two equal samples 800 us apart = DMA dead = no
         * picture even though every register above looks fine. */
        int lcd_ch = -1; uint32_t d1 = 0, d2 = 0;
        for (int c = 0; c < (int)(sizeof(GDMA.channel) / sizeof(GDMA.channel[0])); c++)
            if (GDMA.channel[c].out.peri_sel.sel == 5) { lcd_ch = c; break; }
        if (lcd_ch >= 0) {
            d1 = GDMA.channel[lcd_ch].out.dscr;
            esp_rom_delay_us(3000);     /* > one ~1 ms descriptor at 4 MHz pclk */
            d2 = GDMA.channel[lcd_ch].out.dscr;
        }
        ESP_LOGI(TAG, "hb up=%llus mixer_gate=%d pwr_pwm=%d clk[sel=%u act=%u div=%u] "
                 "pin17_func=%u lcd_dma=%s wifi=%s pad=%s free_int=%uKB",
                 (unsigned long long)(esp_timer_get_time() / 1000000ULL),
                 gpio_get_level(44), gpio_get_level(13),
                 (unsigned)I2S0.tx_clkm_conf.tx_clk_sel,
                 (unsigned)I2S0.tx_clkm_conf.tx_clk_active,
                 (unsigned)I2S0.tx_clkm_conf.tx_clkm_div_num,
                 (unsigned)GPIO.func_out_sel_cfg[17].func_sel,   /* expect I2S0_MCLK_OUT_IDX */
                 lcd_ch < 0 ? "NOCHAN" : (d1 != d2 ? "live" : "DEAD"),
                 wifi ? (const char *)ap.ssid : "down",
                 gamepad_connected() ? "up" : "down",
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    }
}
