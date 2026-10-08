// tv-doom: doomgeneric on ESP32-S3 (phase 1 — engine bring-up, no display yet).
//
// app_main spawns the Doom engine on its own FreeRTOS task with a large stack
// (the renderer's BSP recursion is deep). The IWAD is read from the "wad"
// flash partition by the port's WAD backend (see w_file_stdc.c); the -iwad
// name below is only used to pick the game mission.

#include <stdio.h>
#include <string.h>

#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"   // xTaskCreatePinnedToCoreWithCaps
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "video.h"
#include "gamepad.h"
#include "blelog.h"
#include "clkout.h"
#include "testcard.h"
#include "ota.h"
#include "crashlog.h"
#include "sound.h"
#include "power.h"
#include "settings.h"
#include "driver/gpio.h"
#ifndef TVDOOM_WIFI_OFF
#define TVDOOM_WIFI_OFF 0
#endif
#if TVDOOM_RADIO_OFF || TVDOOM_WIFI_OFF
#include "esp_wifi.h"
#endif
#if TVDOOM_RADIO_OFF
#include "esp_bt.h"
#endif

extern void doomgeneric_Create(int argc, char **argv);
extern void doomgeneric_Tick(void);
extern void D_AllocBuffers(void);   // allocates PSRAM-relocated renderer arrays
extern void doom_wad_map_init(void); // mmap WAD partition (needs internal stack)
extern void DG_SetPcmSink(void (*sink)(const int16_t *, size_t)); // i_fmsound.c
extern void DG_SetBatteryMv(int mv);        // doomgeneric_esp32.c on-screen readout
extern void DG_SetTempC(int c);             // ...and the die-temperature readout under it

// Shim: the power monitor publishes here, the Doom platform layer draws it.
static void power_report(int batt_mv, bool ext, bool terminated)
{
    (void)ext; (void)terminated;
    DG_SetBatteryMv(batt_mv);
    // The monitor task samples the die sensor on the same 1 Hz beat, so this is
    // the reading that goes with this battery sample. NaN before the sensor is
    // up; leave the readout blank rather than draw a bogus number.
    float t = power_temp_c();
    if (t == t) DG_SetTempC((int)(t + 0.5f));
}

static const char *TAG = "main";

static void doom_task(void *arg)
{
    char a0[] = "doomgeneric";
    char a1[] = "-iwad";
    char a2[] = "doom.wad";    // full/Ultimate DOOM; name selects mission, not a path
    char *argv[] = { a0, a1, a2 };

    // The lilka fork moves large renderer arrays (viewangletox, visplanes,
    // vissprites, drawsegs, ...) off .bss into PSRAM; they must be allocated
    // before the engine touches them in R_Init*.
    D_AllocBuffers();

    doomgeneric_Create(3, argv);   // runs D_DoomMain init, then returns

    for (;;) {
        doomgeneric_Tick();        // one frame per call
    }
}

void app_main(void)
{
/* Bench diagnostic: build with -DTVDOOM_RADIO_OFF=1 to leave the Bluetooth
 * controller UNINITIALISED. Not "advertising off" -- gamepad_start() is what
 * calls esp_bt_controller_init/enable and esp_nimble_enable, so skipping it
 * means the radio is never powered at all: no advertising, no scanning, no
 * connections. Used to decide whether the periodic RF envelope disturbances
 * come from this board's own transmissions. The log ring still fills in RAM,
 * it just cannot be read out, and the loader keeps its own radio so SERVER RC
 * still works for deploying. */
#ifndef TVDOOM_RADIO_OFF
#define TVDOOM_RADIO_OFF 0
#endif

    blelog_capture_start();      // ring-capture the whole boot for the BLE log
    ota_bootloop_guard();        // 4 crash resets in a row -> factory loader
    ota_mark_healthy();          // rollback OK + point NEXT boot at the menu (policy)
    crashlog_report();           // print any stored crash PC over BLE

    // System settings (NVS "sys", edited in the loader's SETTINGS menu). The
    // line-sleep arm must be known before the wake recheck below can act.
    sys_settings_t settings;
    settings_load(&settings);
    power_set_line_sleep(settings.line_sleep);
#if TVDOOM_RADIO_OFF || TVDOOM_WIFI_OFF
    /* WiFi off, explicitly rather than by omission. The game never starts it,
     * but the loader does and the game is entered by a SOFT reset -- which we
     * have already seen leave peripherals running on this chip. Stop and deinit
     * rather than trust the reset. Harmless no-ops if it was never up. */
    {
        esp_err_t w_stop = esp_wifi_stop();
        esp_err_t w_deinit = esp_wifi_deinit();
        ESP_LOGW(TAG, "WIFI FORCED OFF (bench build): stop=%s deinit=%s",
                 esp_err_to_name(w_stop), esp_err_to_name(w_deinit));
    }
#endif
#if TVDOOM_RADIO_OFF
    /* Bluetooth off too. gamepad_start() is what calls esp_bt_controller_init/
     * enable and esp_nimble_enable, so not calling it leaves the controller
     * unpowered; the disable/deinit below covers anything that brought it up. */
    blelog_set_enabled(false);
    {
        esp_err_t b_dis = esp_bt_controller_disable();
        esp_err_t b_deinit = esp_bt_controller_deinit();
        ESP_LOGW(TAG, "ALL RADIO DISABLED (bench build): bt_disable=%s bt_deinit=%s; "
                      "gamepad/BLE will not be started, controller status now %d (0 = off)",
                 esp_err_to_name(b_dis), esp_err_to_name(b_deinit),
                 (int)esp_bt_controller_get_status());
    }
#else
    blelog_set_enabled(settings.ble_debug);   // before gamepad_start() brings BLE up
#endif

    // ADC1 for BATTERY_SENSE (GPIO10) and LINE_SENSE (GPIO9). Early and cheap;
    // nothing else contends for ADC1 on this board.
    power_early_gate();          // may shut down here on a flat pack

    // Wake-from-line-sleep triage: bring up ONLY the carrier + mixer (cheap
    // register pokes) and re-probe the line and USB. If nothing appeared this
    // re-enters deep sleep within a few hundred ms of wake -- the periodic
    // recheck that stands in for the (broken-by-R18) ext0 aerial wake and the
    // missing VBUS sense. Falls through to the normal boot when a TV or
    // external power is found.
    if (power_line_recheck_pending()) {
        clkout_start_48mhz();
        mixer_set_enabled(true);
        power_set_carrier_gate(clkout_set_enabled);
        power_set_mixer_gate(mixer_set_enabled);
        power_line_recheck();    // no return unless something is attached
    }

    ESP_LOGI(TAG, "tv-doom boot: free PSRAM %u KB, free internal %u KB",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    // 48 MHz reference on GPIO17 (I2S0 MCLK, PLL240M/5) -- free-runs in
    // hardware, independent of the LCD_CAM video path and Doom on core 0.
    clkout_start_48mhz();
    clkout_enable_boot_toggle();   // BOOT (GPIO0) toggles the carrier on/off
    mixer_set_enabled(true);       // power Q4's MIXER_3V3 rail (gate low = on)

    // The aerial-load probe gates the carrier, so it can only start once the
    // carrier exists.
    power_set_carrier_gate(clkout_set_enabled);
    power_set_mixer_gate(mixer_set_enabled);   // powered down on sleep entry
#if !TVDOOM_RADIO_OFF
    gamepad_set_combo_cb(ota_request);         // Xbox button / View+Menu -> OTA
#endif
    power_set_report_cb(power_report);
    power_start_monitor();

    // Bring up the composite video output engine BEFORE Bluetooth and sound:
    // its LCD GDMA link list needs contiguous internal DMA RAM, and after
    // NimBLE's ~60 KB of internal allocations the heap can no longer supply
    // it (learned the hard way 2026-09-15: "gdma-link: no mem for link list
    // items" -> abort in video_start, a silent boot loop on the USJ console).
    // BT-before-video was only ever about PHY-calibration flash writes
    // glitching the video DMA -- moot since PHY cal storage is disabled in
    // sdkconfig. Give the video task a moment to finish lcd_bringup before
    // anything else allocates internal RAM.
    //
    // Bench calibration card: the raw DAC-code luma map replaces the envelope
    // mapping inside the video tables, so it has to be armed before
    // video_start() builds them.
    if (testcard_is_calib(settings.test_card)) {
        video_set_luma_map_raw(true);
        ESP_LOGW(TAG, "CALIB card: luma tables are a RAW DAC staircase (bench only)");
    }
    video_start();
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_LOGI(TAG, "after video: free internal %u KB",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

#if !TVDOOM_RADIO_OFF
    /* SETTINGS -> BLE TX POWER. Set before the controller comes up so the very
     * first advertising packet is already at the chosen power. */
    gamepad_set_tx_dbm(BLE_TX_DBM[settings.ble_tx]);
    gamepad_start();
#endif
    ESP_LOGI(TAG, "after BT: free internal %u KB",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    // 5.5 MHz FM sound subcarrier on GPIO18 (I2S1 bitstream) -- summed externally
    // into the video baseband; the 48 MHz mixer upconverts it as intercarrier
    // sound. Takes only a small slice of internal DMA RAM, and is non-fatal.
    if (settings.sound_carrier) {
        sound_start(settings.carrier_mode, settings.carrier_drive);
        DG_SetPcmSink(sound_push_s16);   // route Doom's SFX mixer onto the subcarrier
    } else {
        // No carrier: still own GPIO18 and hold it LOW. Left at its reset
        // default (input, floating) the sound filter's input node floats; the
        // loader parks it low when its carrier is off, so the game must too
        // (carrier-OFF picture-instability suspect, 2026-09-15).
        gpio_config_t off = { .pin_bit_mask = 1ULL << 18, .mode = GPIO_MODE_OUTPUT };
        gpio_config(&off);
        gpio_set_level(18, 0);
        ESP_LOGW(TAG, "sound carrier OFF (settings) -- GPIO18 parked low");
    }

    // Bench: RF test patterns instead of Doom (SETTINGS -> TEST CARD, or the
    // loader's SERVER RC "testcard <n>"). Everything the RF chain needs is up
    // (carrier, video, BLE log, sound); Doom is simply never started.
    if (settings.test_card) {
        testcard_start(settings.test_card, settings.sound_carrier);
        return;
    }

    // Map the WAD partition from here (internal stack): the MMU programming
    // freezes the cache, which the PSRAM-stacked Doom task must never do.
    // Everything critical is up: cancel any pending OTA rollback. If a new
    // image dies before reaching this line, the next reset boots the old one.

    doom_wad_map_init();

    // 48 KB stack: Doom's R_RenderBSPNode recursion plus deep call chains.
    // The stack lives in PSRAM (WithCaps): with the BT controller resident,
    // internal DRAM can no longer hold it. Doom (the frame producer) runs on
    // core 0, leaving core 1 to the video engine so the output is never
    // interrupted by game logic.
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(
        doom_task, "doom", 48 * 1024, NULL, 5, NULL, 0,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "doom task create failed!");
    }
}
