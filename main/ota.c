/* OTA handover + rollback handshake. The heavy lifting (WiFi AP + HTTP)
 * lives in the separate ota_loader app in the factory slot -- the WiFi
 * stack's static DRAM does not fit next to BT + video in THIS image, and
 * doing the update with no video/PSRAM/BT running sidesteps the
 * flash-write-vs-PSRAM-DMA conflict entirely. See ota.h. */

#include "ota.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_heap_caps.h"

#include "power.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include "esp_attr.h"
#include "esp_timer.h"
#include "video.h"

static const char *TAG = "ota";

#define OTA_HUNT_MODE 0   /* set 1 to keep re-attempting ota_0 + print backtraces */

/* Boot-loop guard: crash-class resets (panic/WDT/SW) increment an RTC-RAM
 * streak; POWERON/deep-sleep wakes clear it (sleep cycling is normal, not a
 * crash). Four rapid crashes -> boot the factory LOADER instead, which
 * auto-joins WiFi and polls the bench server -- a crash-looping build can
 * always be reflashed with no UART and no pad. Streak clears after 60 s of
 * healthy uptime. */
static RTC_NOINIT_ATTR uint32_t s_boot_streak;

static void guard_clear_cb(void *arg)
{
    s_boot_streak = 0;
}

void ota_bootloop_guard(void)
{
    esp_reset_reason_t rr = esp_reset_reason();

    /* ESP_RST_SW is NOT crash-class: panics/WDTs set their own reset hints
     * (esp_system/panic.c), so on this firmware a SW reset only ever comes
     * from a deliberate esp_restart() -- the Xbox handover, the loader's RUN
     * GAME, this guard's own divert. Counting those stranded quick
     * loader<->game round-trips in the loader (2026-09-15). */
    bool crash = (rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT ||
                  rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT ||
                  rr == ESP_RST_UNKNOWN);
    if (!crash) {
        s_boot_streak = 0;
        return;
    }
    if (!OTA_HUNT_MODE && ++s_boot_streak >= 4) {
        s_boot_streak = 0;
        const esp_partition_t *loader = esp_partition_find_first(
            ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
        if (loader && esp_ota_set_boot_partition(loader) == ESP_OK) {
            ESP_LOGE(TAG, "boot-loop guard: 4 crash resets -- entering loader");
            esp_restart();
        }
        return;   /* no loader partition: nothing better to do than continue */
    }
    ESP_LOGW(TAG, "boot-loop guard: crash reset #%u", (unsigned)s_boot_streak);
    const esp_timer_create_args_t a = { .callback = guard_clear_cb,
                                        .name = "bootguard" };
    esp_timer_handle_t t;
    if (esp_timer_create(&a, &t) == ESP_OK)
        esp_timer_start_once(t, 60 * 1000000);
}

/* The handover runs on a small task with an INTERNAL-RAM stack (the otadata
 * rewrite is a flash write, which a PSRAM-stacked task may not do). It is
 * created on demand: a task parked at boot cost 3.5 KB of the internal heap
 * that BT lives on (2026-09-15 evening: BT broke). If even the on-demand
 * allocation fails, the handover runs INLINE on the caller -- the NimBLE
 * host task, internal stack -- so BLE loses a few ms on its way into a
 * reboot and the guide button is never silently dead (which is what the
 * old unchecked xTaskCreate + one-shot latch produced). */
#define HANDOVER_STACK 3584
static volatile bool s_requested;        /* one-shot latch, cleared if the handover backs out */

/* Returns only if it had to back out. */
static bool handover_run(void)
{
    const esp_partition_t *loader = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (!loader) {
        ESP_LOGE(TAG, "no factory loader partition -- flashed with the old table?");
        return false;
    }
    ESP_LOGW(TAG, "OTA requested: rebooting into the loader menu");
    power_inhibit_actions(true);            /* no sleeping during the handover */
    esp_err_t e = esp_ota_set_boot_partition(loader);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "set_boot(factory) failed: %s", esp_err_to_name(e));
        power_inhibit_actions(false);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(200));         /* let the log line flush */
    /* Quiesce every PSRAM master BEFORE the reset. A reset issued while the
     * LCD GDMA is mid-burst on the PSRAM (same SPI bus as the flash) leaves
     * the bus in a state the ROM bootloader cannot read flash through
     * (2026-09-15, serial capture): esp_restart() then only came back via its
     * 1 s RTC-WDT rescue, and the deep-sleep restart tried in between hung
     * until power-cycled. So: park Doom and the sound task (CPU-side PSRAM
     * traffic), delete the panel (stops the DMA), then the WDT-protected
     * esp_restart() -- the rescue stays as a safety net, never the plan. */
    TaskHandle_t t;
    if ((t = xTaskGetHandle("doom")))  vTaskSuspend(t);
    if ((t = xTaskGetHandle("sound"))) vTaskSuspend(t);
    video_stop();
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
    return false;                           /* not reached */
}

static void ota_handover_task(void *arg)
{
    if (!handover_run()) s_requested = false;
    vTaskDelete(NULL);
}

void ota_request(void)
{
    /* Called from the NimBLE host task (the pad's guide-button report). */
    if (s_requested) return;
    s_requested = true;
    ESP_LOGW(TAG, "guide button: OTA handover requested");
    if (xTaskCreatePinnedToCore(ota_handover_task, "otahand", HANDOVER_STACK, NULL, 5,
                                NULL, 0) == pdPASS)
        return;
    ESP_LOGE(TAG, "handover task: no internal RAM (free %u B, largest block %u B) -- running inline",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    if (!handover_run()) s_requested = false;
}

void ota_mark_healthy(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (run && esp_ota_get_state_partition(run, &st) == ESP_OK &&
        st == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_ERROR_CHECK(esp_ota_mark_app_valid_cancel_rollback());
        ESP_LOGW(TAG, "first boot of %s verified healthy -- rollback cancelled",
                 run->label);
    }

    /* BOOT POLICY (user rule, 2026-09-15): EVERY boot lands in the menu.
     * RUN GAME is a one-shot launch -- the loader points otadata at ota_0
     * and restarts; we point it straight back at factory here, first thing
     * in app_main (no video DMA or BLE exists yet, so the flash write can't
     * disturb anything), so ANY later reset -- crash, power cycle, Xbox
     * button -- goes to the menu. Two otadata writes per launch; a zero-wear
     * RTC-flag bootloader override is the redesign-era upgrade. */
    const esp_partition_t *menu = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (menu && esp_ota_get_boot_partition() != menu) {
        esp_err_t e = esp_ota_set_boot_partition(menu);
        ESP_LOGW(TAG, "menu-first: next boot -> factory menu (%s)",
                 e == ESP_OK ? "ok" : esp_err_to_name(e));
    }
}
