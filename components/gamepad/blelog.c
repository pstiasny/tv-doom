/* BLE GATT log stream -- see blelog.h. Lives in the gamepad component
 * because it shares the NimBLE host the HID client owns (dual role:
 * central to the pad, peripheral to the log listener). */
#include "blelog.h"
#include "gamepad.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <stdbool.h>

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "blelog";

#define ADV_NAME  "tvdoom-log"
#ifndef BLELOG_RING_SZ
#define BLELOG_RING_SZ 3072       /* internal RAM: the hook may run while the
                                   * flash cache (and thus PSRAM) is off     */
#endif
#define RING_SZ   BLELOG_RING_SZ
/* Boot transcript: the first BLELOG_BOOT_SZ bytes logged after
 * blelog_capture_start() are kept write-once and replayed to every new
 * subscriber, so a late connection still shows exactly how the boot went.
 * 0 = disabled (the game's DRAM is too tight); the loader enables it from
 * its CMakeLists. */
#ifndef BLELOG_BOOT_SZ
#define BLELOG_BOOT_SZ 0
#endif

/* 128-bit vendor UUIDs (random, project-local). */
static const ble_uuid128_t SVC_UUID = BLE_UUID128_INIT(
    0x9e, 0x51, 0x43, 0x7a, 0xd2, 0x0c, 0x4a, 0x8b,
    0x9f, 0x36, 0x1c, 0x4e, 0x10, 0x67, 0x76, 0xb0);
static const ble_uuid128_t CHR_UUID = BLE_UUID128_INIT(
    0x9e, 0x51, 0x43, 0x7a, 0xd2, 0x0c, 0x4a, 0x8b,
    0x9f, 0x36, 0x1c, 0x4e, 0x11, 0x67, 0x76, 0xb0);
static char     s_ring[RING_SZ];
static size_t   s_head, s_tail;          /* head = write, tail = read */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_orig_vprintf;
static bool     s_capturing;

#if BLELOG_BOOT_SZ > 0
static char     s_boot[BLELOG_BOOT_SZ];
static size_t   s_boot_len;              /* bytes captured (write-once)    */
static size_t   s_boot_sent;             /* replay cursor for the subscriber */
static enum { RP_IDLE, RP_HDR, RP_BODY, RP_TAIL } s_replay;
#endif

static uint16_t s_val_handle;
static volatile uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_subscribed;
static bool s_adv_running;
static volatile bool s_enabled = true;   /* SETTINGS -> BLE DEBUGGING */

/* Post-video the console pin belongs to the LCD, so runtime diagnostics are
 * invisible; these RTC counters survive resets and are printed by the next
 * boot (blelog_init) in the UART-visible window. */
/* Cross-app diagnostic block at a FIXED RTC address (top 256 B of the 8 KB
 * RTC slow memory; both link maps end far below 0x50000100). Until
 * 2026-09-16 these were RTC_NOINIT_ATTR variables, which land at DIFFERENT
 * addresses in the game and the loader (other RTC data sits ahead of them
 * in the game), so the loader's "prev-run counters" line was reading the
 * game's power.c RTC data -- garbage. Whichever app boots next prints and
 * clears the block, so the loader transcript shows the game run's numbers:
 * did the drain task run at all, did it see the subscription, did notifies
 * succeed, and how much internal heap was left. */
typedef struct {
    uint32_t magic;
    uint32_t conn, subs, ntfy_ok, ntfy_err;
    uint32_t drain_loops;             /* drain_task iterations                */
    uint32_t synced, subscribed;      /* ble_hs_synced() / s_subscribed, last loop */
    uint32_t free_int, largest_int;   /* internal heap, bytes, sampled ~1 Hz  */
} blelog_rtc_t;
#define BLELOG_RTC_MAGIC 0xB1E10601u
#define BLELOG_RTC ((volatile blelog_rtc_t *)0x50001F00)
static volatile bool s_draining;   /* drop ring captures from the drain task's
                                    * own notify call: NimBLE logs every notify
                                    * at INFO, which otherwise feeds the ring it
                                    * is draining -- an infinite self-storm that
                                    * exhausts mbufs (pad connect rc=6 ENOMEM) */

/* ---------- ring (drop-oldest, never blocks the logger) ------------------ */

static void ring_push(const char *p, size_t n)
{
    portENTER_CRITICAL(&s_mux);
#if BLELOG_BOOT_SZ > 0
    if (s_boot_len < BLELOG_BOOT_SZ) {
        size_t k = BLELOG_BOOT_SZ - s_boot_len;
        if (k > n) k = n;
        memcpy(s_boot + s_boot_len, p, k);
        s_boot_len += k;
    }
#endif
    for (size_t i = 0; i < n; i++) {
        s_ring[s_head] = p[i];
        s_head = (s_head + 1) % RING_SZ;
        if (s_head == s_tail)                 /* full: drop oldest */
            s_tail = (s_tail + 1) % RING_SZ;
    }
    portEXIT_CRITICAL(&s_mux);
}

static size_t ring_pop(char *out, size_t max)
{
    size_t n = 0;
    portENTER_CRITICAL(&s_mux);
    while (n < max && s_tail != s_head) {
        out[n++] = s_ring[s_tail];
        s_tail = (s_tail + 1) % RING_SZ;
    }
    portEXIT_CRITICAL(&s_mux);
    return n;
}

static int log_vprintf(const char *fmt, va_list ap)
{
    char buf[192];
    va_list ap2;
    va_copy(ap2, ap);
    int n = vsnprintf(buf, sizeof buf, fmt, ap2);
    va_end(ap2);
    /* Second guard against the self-storm: NimBLE's per-notify chatter can
     * also arrive from the host task, where s_draining doesn't cover it. */
    if (n > 0 && !s_draining && !strstr(buf, "initiated: notify") &&
        !strstr(buf, "att_handle="))
        ring_push(buf, n < (int)sizeof buf ? (size_t)n : sizeof buf - 1);
    return s_orig_vprintf ? s_orig_vprintf(fmt, ap) : n;
}

/* ---------- GATT service -------------------------------------------------- */

/* Log chr is notify-only; a stray read gets an empty answer. (A write/command
 * characteristic lived here 2026-09-15 and was removed -- see blelog.h.) */
static int chr_access(uint16_t conn, uint16_t attr,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return 0;
}

static const struct ble_gatt_svc_def SVCS[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = &CHR_UUID.u,
              .access_cb = chr_access,
              .val_handle = &s_val_handle,
              .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ },
            { 0 },
        },
    },
    { 0 },
};

/* ---------- advertising --------------------------------------------------- */

static void adv_start(void);

static int adv_gap_cb(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        s_adv_running = false;
        if (ev->connect.status == 0) {
            BLELOG_RTC->conn++;
            s_conn = ev->connect.conn_handle;
        } else {
            adv_start();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        if (ev->disconnect.conn.conn_handle == s_conn) {
            s_conn = BLE_HS_CONN_HANDLE_NONE;
            s_subscribed = false;
        }
        adv_start();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        BLELOG_RTC->subs++;
        if (ev->subscribe.attr_handle == s_val_handle) {
            s_subscribed = ev->subscribe.cur_notify;
#if BLELOG_BOOT_SZ > 0
            if (s_subscribed) { s_boot_sent = 0; s_replay = RP_HDR; }
#endif
        }
        return 0;
    default:
        return 0;
    }
}

static void adv_start(void)
{
    if (s_adv_running || !s_enabled) return;
    struct ble_hs_adv_fields f = { 0 };
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.name = (const uint8_t *)ADV_NAME;
    f.name_len = strlen(ADV_NAME);
    f.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&f);
    if (rc == 0) {
        struct ble_gap_adv_params p = {
            .conn_mode = BLE_GAP_CONN_MODE_UND,
            .disc_mode = BLE_GAP_DISC_MODE_GEN,
        };
        rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &p, adv_gap_cb, NULL);
    }
    if (rc == 0) {
        s_adv_running = true;
    } else if (rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "adv start rc=%d (will retry)", rc);
    }
}

/* ---------- drain task ----------------------------------------------------- */

static void drain_task(void *arg)
{
    static char buf[244];
    bool announced = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60));
        BLELOG_RTC->drain_loops++;
        BLELOG_RTC->synced = ble_hs_synced();
        BLELOG_RTC->subscribed = s_subscribed;
        if ((BLELOG_RTC->drain_loops & 15) == 0) {
            BLELOG_RTC->free_int    = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            BLELOG_RTC->largest_int = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        }
        if (!ble_hs_synced()) continue;
        if (!announced) {
            announced = true;
            ESP_LOGI(TAG, "log service up, %s", s_enabled
                     ? "advertising as \"" ADV_NAME "\""
                     : "BLE debugging OFF (not advertising)");
        }
        /* Advertise whenever no log client is connected. (This was once
         * gated on the pad link out of fear of scan/adv interference, but
         * the real 2026-09-14 lockout was the notify storm exhausting mbufs
         * -- fixed. Unattended recovery needs the control channel reachable
         * with NO pad, so the gate is gone; coexistence is verified on the
         * bench.) */
        if (!s_adv_running && s_conn == BLE_HS_CONN_HANDLE_NONE)
            adv_start();
        if (!s_subscribed) continue;

        uint16_t conn = s_conn;
        if (conn == BLE_HS_CONN_HANDLE_NONE) continue;
        int mtu = ble_att_mtu(conn);
        size_t max = mtu > 3 ? (size_t)(mtu - 3) : 20;
        if (max > sizeof buf) max = sizeof buf;

        size_t n = 0;
#if BLELOG_BOOT_SZ > 0
        /* Replay the boot transcript first (header, body, tail marker),
         * then fall through to the live ring. */
        if (s_replay == RP_HDR) {
            n = snprintf(buf, max, "\n--- boot transcript (%u bytes) ---\n",
                         (unsigned)s_boot_len);
            if (n >= max) n = max - 1;      /* snprintf returns untruncated len */
            s_replay = RP_BODY;
        } else if (s_replay == RP_BODY) {
            size_t left = s_boot_len - s_boot_sent;
            n = left < max ? left : max;
            memcpy(buf, s_boot + s_boot_sent, n);
            s_boot_sent += n;
            if (s_boot_sent >= s_boot_len) s_replay = RP_TAIL;
        } else if (s_replay == RP_TAIL) {
            n = snprintf(buf, max, "\n--- live ---\n");
            if (n >= max) n = max - 1;
            s_replay = RP_IDLE;
        }
        if (!n)
#endif
        n = ring_pop(buf, max);
        if (!n) continue;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, n);
        if (!om) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
        s_draining = true;
        int rc = ble_gatts_notify_custom(conn, s_val_handle, om);
        s_draining = false;
        if (rc == 0) BLELOG_RTC->ntfy_ok++; else BLELOG_RTC->ntfy_err++;
        if (rc) vTaskDelay(pdMS_TO_TICKS(30));   /* controller busy: back off */
    }
}

void blelog_init(void)
{
    extern uint32_t power_wake_n, power_wake_done;   /* power.c RTC breadcrumbs (game-only
                                                       * layout: not valid across apps) */
    volatile blelog_rtc_t *d = BLELOG_RTC;
    if (d->magic == BLELOG_RTC_MAGIC) {
        ESP_LOGW(TAG, "prev-run: conn %u subs %u ntfy %u ok / %u err | drain loops %u "
                 "synced %u subscribed %u | free int %u B (largest %u) | WAKE n=%u done=%u",
                 (unsigned)d->conn, (unsigned)d->subs, (unsigned)d->ntfy_ok,
                 (unsigned)d->ntfy_err, (unsigned)d->drain_loops, (unsigned)d->synced,
                 (unsigned)d->subscribed, (unsigned)d->free_int, (unsigned)d->largest_int,
                 (unsigned)power_wake_n, (unsigned)power_wake_done);
    } else {
        ESP_LOGW(TAG, "prev-run: no diag block (magic %08x) -- previous image predates it",
                 (unsigned)d->magic);
    }
    for (volatile uint32_t *w = (volatile uint32_t *)d;
         w < (volatile uint32_t *)(d + 1); w++) *w = 0;
    d->magic = BLELOG_RTC_MAGIC;

    int rc = ble_gatts_count_cfg(SVCS);
    if (rc == 0) rc = ble_gatts_add_svcs(SVCS);
    if (rc) {
        ESP_LOGE(TAG, "service registration failed rc=%d -- no BLE log", rc);
        return;
    }
    blelog_capture_start();
    /* Priority ABOVE the Doom task (5): Doom's loop is producer-limited and
     * never blocks, so at priority 3 this task only ran when Doom happened to
     * wait for a frame slot -- in the game the log streamed for ~8 s after a
     * subscribe and then went silent for minutes while still connected
     * (2026-09-16). At 6 it wakes every 60 ms for one <=244 B notify, ~1% of
     * core 0. Same reasoning as the power monitor task. */
    xTaskCreatePinnedToCore(drain_task, "blelog", 3072, NULL, 6, NULL, 0);
}

void blelog_capture_start(void)
{
    if (s_capturing) return;
    s_capturing = true;
    s_orig_vprintf = esp_log_set_vprintf(log_vprintf);
}

void blelog_set_enabled(bool on)
{
    s_enabled = on;
    if (on || !ble_hs_synced()) return;      /* drain_task advertises when on */
    /* Going OFF live: stop advertising and drop the log client. The pad's
     * own connection is a different handle and is left alone. */
    if (s_adv_running) {
        ble_gap_adv_stop();
        s_adv_running = false;
    }
    uint16_t conn = s_conn;
    if (conn != BLE_HS_CONN_HANDLE_NONE)
        ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    ESP_LOGW(TAG, "BLE debugging OFF: log link closed, not advertising");
}
