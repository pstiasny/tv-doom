/*
 * OTA loader menu: gamepad-driven WiFi join + pull update, on the B&W display.
 *
 * Flow:  the main menu (RUN GAME / SELECT WIFI NETWORK / DOWNLOAD IMAGE)
 *        is up from the first frame. A saved network (NVS "otaldr") is
 *        joined by a BACKGROUND task meanwhile -- the title line tracks it
 *        (JOINING <ssid> -> the IP, or "(join failed)") and RUN GAME never
 *        waits for WiFi. SELECT WIFI NETWORK: SCAN -> list (dpad + A) ->
 *        password keyboard (dpad, A=key, B=backspace, X=page, OK=join) ->
 *        JOINING -> saved to NVS. DOWNLOAD IMAGE: GET the server's index
 *        (tools/ota.py, "<path>|<label>" per line) -> pick one -> pull it
 *        into ota_0 with a progress bar, then boot it (a loader image
 *        self-installs into the factory slot on that boot). SETTINGS:
 *        system-level switches (NVS "sys", read by the game too) --
 *        sound carrier ON/OFF (applied here at once), line detect sleep
 *        ON/OFF (arms the game's aerial-open deep sleep), BLE debugging
 *        ON/OFF (the "tvdoom-log" advertising + log stream; live), SERVER
 *        RC ON/OFF (below).
 *
 * SERVER RC: remote control over the pull server, for unattended bench
 *        cycles (the BLE command channel was retired 2026-09-15). With the
 *        setting ON, the moment WiFi comes up -- the boot auto-join or a
 *        manual join -- the loader GETs <server>/cmd once and executes the
 *        reply. The server hands a queued command out exactly once (the
 *        delivering GET clears it, tools/ota.py), so no later boot can
 *        replay it and a failed update cannot loop; an empty reply means
 *        nothing to do. Commands:
 *          update <path|url>   pull that image into ota_0, reboot into it
 *          run                 boot the game (= RUN GAME)
 *          reboot              restart the loader
 *          set <key> <value>   a SETTINGS switch: snd|lsleep|bledbg|srvrc
 *                              on|off, carmode|cardrv|tcard <index>
 *          testcard <index>    set TEST CARD <index> and boot the game into
 *                              the RF test-card generator (0 = plain game)
 *        The fetch runs on whichever task brought WiFi up; execution is
 *        handed to the menu task through the key queue (K_RCMD), so nothing
 *        ever draws over a menu from another task.
 *
 * Input comes from the same BLE gamepad component the game uses; the doom_*
 * symbols it expects are stubbed at the bottom (doom_push_key feeds a queue
 * here instead of Doom's key buffer).
 */
#include "ota_menu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "blelog.h"
#include "ota_display.h"
#include "ota_sound.h"
#include "settings.h"
#include "gamepad.h"

static const char *TAG = "otamenu";

/* Hardcoded pull server for now (the Mac on the bench LAN; tools/ota.py
 * serves both images). */
#define OTA_SRV        "192.168.1.21:8000"
#define OTA_INDEX_URL  "http://" OTA_SRV "/index.txt"
#define OTA_CMD_URL    "http://" OTA_SRV "/cmd"          /* SERVER RC, one-shot */

#define MAX_IMAGES     10      /* index entries shown */
#define IMG_PATH_MAX   64
#define IMG_LABEL_MAX  26      /* 25 chars fit the 160 px line */

#define MAX_APS      10
#define WIFI_PASS_MAX     63
#define CONNECT_TO_S 20          /* manual join (network picked by hand) */
#define AUTO_JOIN_TO_S 10        /* boot-time join of the saved network  */
#define JOIN_RETRY_MS 1000       /* pause between connect attempts       */

/* Doom key codes the gamepad component emits (doomkeys.h values). */
#define K_UP    0xad
#define K_DOWN  0xaf
#define K_LEFT  0xac
#define K_RIGHT 0xae
#define K_A     13     /* KEY_ENTER  */
#define K_B     27     /* KEY_ESCAPE (B or Menu) */
#define K_X     0xa2   /* KEY_USE    */
#define K_REFRESH 0    /* synthetic, not a pad key: WiFi state changed, redraw */
#define K_RCMD    1    /* synthetic: a SERVER RC command waits in s_rc */

static QueueHandle_t s_keys;              /* pressed-key events */
static EventGroupHandle_t s_wifi_ev;
#define EV_GOT_IP  BIT0
#define EV_FAILED  BIT1

extern void boot_game(void);              /* loader_main.c */
extern void ota_touch_activity(void);     /* loader_main.c idle timer */
static void rc_execute(void);             /* SERVER RC, below */

/* ---------- input: stubs for the gamepad component's doom_* hooks -------- */

volatile int doom_analog_forward, doom_analog_side, doom_analog_turn;
int doom_current_weapon_slot(void) { return 0; }
int doom_weapon_slot_owned(int slot) { (void)slot; return 0; }

void doom_push_key(int pressed, unsigned char key)
{
    if (pressed && s_keys) {
        xQueueSend(s_keys, &key, 0);
        ota_touch_activity();
    }
}

static unsigned char key_wait_raw(void)
{
    unsigned char k = 0;
    xQueueReceive(s_keys, &k, portMAX_DELAY);
    if (k == K_RCMD) {          /* SERVER RC: run it here, on the menu task */
        rc_execute();
        return K_REFRESH;       /* whatever we interrupted redraws itself */
    }
    return k;
}

/* A real pad key: K_REFRESH pokes only matter to menu_pick's header. */
static unsigned char key_wait(void)
{
    unsigned char k;
    do k = key_wait_raw(); while (k == K_REFRESH);
    return k;
}

/* ---------- WiFi ---------------------------------------------------------- */

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = data;
        ESP_LOGW(TAG, "sta disconnected: reason %d rssi %d", d->reason, d->rssi);
        xEventGroupSetBits(s_wifi_ev, EV_FAILED);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_ev, EV_GOT_IP);
    }
}

static int wifi_scan(wifi_ap_record_t *out, int max)
{
    esp_wifi_scan_start(NULL, true);
    uint16_t n = max;
    if (esp_wifi_scan_get_ap_records(&n, out) != ESP_OK) return 0;
    /* dedup by SSID, keep strongest (records arrive sorted by RSSI) */
    int m = 0;
    for (int i = 0; i < n; i++) {
        if (!out[i].ssid[0]) continue;
        bool dup = false;
        for (int j = 0; j < m; j++)
            if (!strcmp((char *)out[j].ssid, (char *)out[i].ssid)) { dup = true; break; }
        if (!dup) out[m++] = out[i];
    }
    return m;
}

/* Join, retrying until the deadline. One esp_wifi_connect() is NOT enough:
 * the driver gives up after a single failed association, and after a power
 * cut (no deauth sent) a PMF/802.11w AP still holds our previous session --
 * it answers the new association with "refused temporarily" (status 30,
 * comeback ~400 TU) while it runs an SA Query on the stale session, the
 * driver's one comeback retry lands before that query has timed out, and
 * the boot ends with no WiFi. The failing boot is exactly what makes the AP
 * forget the stale session, so the NEXT boot joins first try: WiFi on every
 * other power cycle (2026-09-15, BLE boot transcript). A fresh attempt ~1 s
 * later goes straight through, hence the loop. */
static bool wifi_join_to(const char *ssid, const char *pass, char *ip_out,
                         size_t ip_len, int timeout_s)
{
    wifi_config_t sta = { 0 };
    strlcpy((char *)sta.sta.ssid, ssid, sizeof sta.sta.ssid);
    strlcpy((char *)sta.sta.password, pass, sizeof sta.sta.password);

    /* Leave any current association first and let its DISCONNECTED event
     * land before the attempts start, so it can't be mistaken for a failure
     * of the new one. */
    wifi_ap_record_t cur;
    if (esp_wifi_sta_get_ap_info(&cur) == ESP_OK) {
        xEventGroupClearBits(s_wifi_ev, EV_GOT_IP | EV_FAILED);
        esp_wifi_disconnect();
        xEventGroupWaitBits(s_wifi_ev, EV_FAILED, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));
    }
    esp_wifi_set_config(WIFI_IF_STA, &sta);

    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_s * 1000000;
    for (int attempt = 1;; attempt++) {
        xEventGroupClearBits(s_wifi_ev, EV_GOT_IP | EV_FAILED);
        esp_err_t e = esp_wifi_connect();
        EventBits_t b = 0;
        if (e == ESP_OK) {
            int64_t left = deadline - esp_timer_get_time();
            if (left > 0)
                b = xEventGroupWaitBits(s_wifi_ev, EV_GOT_IP | EV_FAILED,
                                        pdTRUE, pdFALSE,
                                        pdMS_TO_TICKS(left / 1000));
        } else {
            ESP_LOGW(TAG, "esp_wifi_connect: %s", esp_err_to_name(e));
        }
        if (b & EV_GOT_IP) {
            if (attempt > 1)
                ESP_LOGW(TAG, "joined %s on attempt %d", ssid, attempt);
            break;
        }
        if (deadline - esp_timer_get_time() <= JOIN_RETRY_MS * 1000LL) {
            ESP_LOGW(TAG, "join %s: giving up after %d attempt(s)", ssid, attempt);
            return false;
        }
        ESP_LOGW(TAG, "join %s: attempt %d %s, retrying", ssid, attempt,
                 (b & EV_FAILED) ? "failed" : "not started");
        vTaskDelay(pdMS_TO_TICKS(JOIN_RETRY_MS));
    }
    esp_netif_ip_info_t ip;
    esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (nif && esp_netif_get_ip_info(nif, &ip) == ESP_OK)
        snprintf(ip_out, ip_len, IPSTR, IP2STR(&ip.ip));
    else
        strlcpy(ip_out, "?", ip_len);
    return true;
}

/* ---------- saved credentials (NVS namespace "otaldr") ------------------- */

static bool creds_load(char *ssid, size_t sl, char *pass, size_t pl)
{
    nvs_handle_t h;
    if (nvs_open("otaldr", NVS_READONLY, &h) != ESP_OK) return false;
    bool ok = nvs_get_str(h, "ssid", ssid, &sl) == ESP_OK &&
              nvs_get_str(h, "pass", pass, &pl) == ESP_OK;
    nvs_close(h);
    return ok && ssid[0];
}

static void creds_save(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    if (nvs_open("otaldr", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, "ssid", ssid);
    nvs_set_str(h, "pass", pass);
    nvs_commit(h);
    nvs_close(h);
}

static void creds_forget(void)
{
    nvs_handle_t h;
    if (nvs_open("otaldr", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
}

/* ---------- UI helpers ---------------------------------------------------- */

static void header(const char *title)
{
    ota_display_clear();
    ota_display_rect(0, 0, OTA_DISP_W, 12, true);
    ota_display_text(4, 2, title, true);
}

static void status_line(const char *msg)
{
    ota_display_rect(0, OTA_DISP_H - 12, OTA_DISP_W, 12, false);
    ota_display_text(4, OTA_DISP_H - 10, msg, false);
}

static void splash(const char *l1, const char *l2)
{
    header("TVPLUG2 OTA");
    ota_display_text(4, 40, l1, false);
    if (l2) ota_display_text(4, 40 + 2 * OTA_FONT_H, l2, false);
}

static char s_title[40];                 /* main-menu header; the auto-join
                                          * task rewrites it (see wifi_set_state) */
static portMUX_TYPE s_title_mux = portMUX_INITIALIZER_UNLOCKED;

/* Generic vertical menu; returns index or -1 on B. The header is redrawn
 * on every K_REFRESH so a title that lives in s_title follows the WiFi
 * state while the user sits in the menu. hint = status line text, NULL for
 * the default. */
static int menu_pick(const char *title, const char *items[], int n, int cur,
                     const char *hint)
{
    for (;;) {
        char t[sizeof s_title];
        portENTER_CRITICAL(&s_title_mux);   /* title may be s_title mid-rewrite */
        strlcpy(t, title, sizeof t);
        portEXIT_CRITICAL(&s_title_mux);
        header(t);
        status_line(hint ? hint : "A:SELECT B:BACK");
        for (;;) {
            for (int i = 0; i < n; i++) {
                ota_display_rect(0, 16 + i * (OTA_FONT_H + 2), OTA_DISP_W, OTA_FONT_H + 2, i == cur);
                ota_display_text(4, 17 + i * (OTA_FONT_H + 2), items[i], i == cur);
            }
            unsigned char k = key_wait_raw();
            if (k == K_REFRESH) break;      /* redraw the header, keep the cursor */
            switch (k) {
            case K_UP:   cur = (cur + n - 1) % n; break;
            case K_DOWN: cur = (cur + 1) % n; break;
            case K_A:    return cur;
            case K_B:    return -1;
            }
        }
    }
}

/* ---------- on-screen keyboard -------------------------------------------- */

static const char *KB_PAGES[3][4] = {
    { "abcdefghij", "klmnopqrst", "uvwxyz-_. ", NULL },
    { "ABCDEFGHIJ", "KLMNOPQRST", "UVWXYZ-_. ", NULL },
    { "0123456789", "!@#$%^&*()", "+=[]{}:;/ ", NULL },
};

/* Returns true with text in buf; false on B-cancel. */
static bool keyboard(const char *prompt, char *buf, size_t buflen)
{
    int page = 0, row = 0, col = 0;
    size_t len = strlen(buf);
    for (;;) {
        header(prompt);
        status_line("A:KEY B:DEL X:PG OK=JOIN");
        ota_display_rect(2, 16, OTA_DISP_W - 4, 12, true);
        ota_display_text(4, 18, buf, true);
        int y0 = 32;
        for (int r = 0; r < 3; r++) {
            const char *line = KB_PAGES[page][r];
            for (int c = 0; c < 10; c++) {
                char cell[2] = { line[c], 0 };
                bool sel = (r == row && c == col);
                int x = 2 + c * 16, y = y0 + r * 14;
                ota_display_rect(x, y - 1, 14, 11, sel);
                ota_display_text(x + 4, y, cell[0] == ' ' ? "_" : cell, sel);
            }
        }
        /* fourth row: OK cell */
        bool sel_ok = (row == 3);
        ota_display_rect(2, y0 + 3 * 14, 40, 12, sel_ok);
        ota_display_text(6, y0 + 3 * 14 + 2, "[OK]", sel_ok);

        switch (key_wait()) {
        case K_UP:    row = (row + 3) % 4; break;
        case K_DOWN:  row = (row + 1) % 4; break;
        case K_LEFT:  if (row < 3) col = (col + 9) % 10; break;
        case K_RIGHT: if (row < 3) col = (col + 1) % 10; break;
        case K_X:     page = (page + 1) % 3; break;
        case K_A:
            if (row == 3) return true;
            if (len + 1 < buflen) { buf[len++] = KB_PAGES[page][row][col]; buf[len] = 0; }
            break;
        case K_B:
            if (len) buf[--len] = 0;
            else return false;
            break;
        }
    }
}

/* ---------- pull update ---------------------------------------------------- */

static bool pull_update(const char *url, const char *what)
{
    splash("FETCHING", what);
    ota_display_progress(0);

    esp_http_client_config_t hc = {
        .url = url,
        .timeout_ms = 15000,
    };
    esp_http_client_handle_t cl = esp_http_client_init(&hc);
    if (!cl) return false;
    bool ok = false;
    esp_ota_handle_t oh = 0;
    const esp_partition_t *dst = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);

    if (esp_http_client_open(cl, 0) != ESP_OK) { status_line("CONNECT FAILED"); goto out; }
    int64_t total = esp_http_client_fetch_headers(cl);
    int code = esp_http_client_get_status_code(cl);
    if (code != 200 || total <= 0) {
        char m[40]; snprintf(m, sizeof m, "HTTP %d", code);
        status_line(m); goto out;
    }
    if (!dst || esp_ota_begin(dst, OTA_SIZE_UNKNOWN, &oh) != ESP_OK) {
        status_line("OTA BEGIN FAILED"); goto out;
    }
    {
        static char buf[8192];
        int64_t got = 0;
        while (got < total) {
            int n = esp_http_client_read(cl, buf, sizeof buf);
            if (n <= 0) { status_line("READ FAILED"); goto out; }
            if (esp_ota_write(oh, buf, n) != ESP_OK) { status_line("FLASH WRITE FAILED"); goto out; }
            got += n;
            ota_display_progress((int)(got * 100 / total));
            ota_touch_activity();
        }
    }
    if (esp_ota_end(oh) != ESP_OK) { oh = 0; status_line("IMAGE INVALID"); goto out; }
    oh = 0;
    if (esp_ota_set_boot_partition(dst) != ESP_OK) { status_line("SET BOOT FAILED"); goto out; }
    ok = true;
out:
    if (oh) esp_ota_abort(oh);
    esp_http_client_cleanup(cl);
    return ok;
}

/* ---------- image index --------------------------------------------------- */

/* GET a small text resource into buf (NUL-terminated). Returns the body
 * length -- 0 for an empty body, which is /cmd's "nothing queued" -- or -1
 * on any transport or HTTP error. */
static int http_get_text(const char *url, char *buf, size_t len)
{
    esp_http_client_config_t hc = { .url = url, .timeout_ms = 5000 };
    esp_http_client_handle_t cl = esp_http_client_init(&hc);
    if (!cl) return -1;
    int ret = -1;
    size_t got = 0;
    if (esp_http_client_open(cl, 0) != ESP_OK) goto out;
    esp_http_client_fetch_headers(cl);
    if (esp_http_client_get_status_code(cl) != 200) goto out;
    while (got < len - 1) {
        int n = esp_http_client_read(cl, buf + got, len - 1 - got);
        if (n < 0) goto out;
        if (n == 0) break;
        got += n;
    }
    buf[got] = 0;
    ret = (int)got;
out:
    esp_http_client_cleanup(cl);
    return ret;
}

/* Image <path> from the index or a command -> full URL: relative to the
 * server root, or an http:// URL passed through. */
static void image_url(const char *path, char *url, size_t n)
{
    if (!strncmp(path, "http://", 7))
        strlcpy(url, path, n);
    else
        snprintf(url, n, "http://%s%s%s", OTA_SRV, path[0] == '/' ? "" : "/", path);
}

/* index.txt: one image per line, "<path>|<label>"; <path> is relative to
 * the server (or a full http:// URL), <label> is what the menu shows.
 * Parses in place; returns the number of entries kept. */
static int index_parse(char *text, char paths[][IMG_PATH_MAX],
                       char labels[][IMG_LABEL_MAX], int max)
{
    int n = 0;
    for (char *line = strtok(text, "\r\n"); line && n < max;
         line = strtok(NULL, "\r\n")) {
        char *bar = strchr(line, '|');
        if (!bar) continue;
        *bar = 0;
        if (!line[0] || !bar[1]) continue;
        strlcpy(paths[n], line, IMG_PATH_MAX);
        strlcpy(labels[n], bar + 1, IMG_LABEL_MAX);
        n++;
    }
    return n;
}

/* ---------- loader self-install (running from ota_0) ----------------------
 *
 * The factory loader cannot rewrite itself (you never write the partition
 * you execute from). Loader updates therefore go in two hops: the running
 * loader pulls the new loader image into ota_0 and boots it ("staged");
 * the staged copy -- executing from ota_0 -- copies its own slot into the
 * factory partition with plain esp_partition calls, erases otadata (erased
 * otadata boots factory, as learned the hard way), and reboots into the
 * freshly installed loader. The game is then restored with UPDATE GAME. */

static bool running_staged(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    return run && run->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0;
}

static void install_to_factory(void)
{
    const esp_partition_t *fac = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    const esp_partition_t *self = esp_ota_get_running_partition();
    const esp_partition_t *od = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, NULL);
    if (!fac || !self || !od || fac->size != self->size) return;

    splash("INSTALLING LOADER", "DO NOT POWER OFF");
    ota_display_progress(0);
    if (esp_partition_erase_range(fac, 0, fac->size) != ESP_OK) return;
    static char buf[4096];
    for (size_t off = 0; off < self->size; off += sizeof buf) {
        if (esp_partition_read(self, off, buf, sizeof buf) != ESP_OK) return;
        if (esp_partition_write(fac, off, buf, sizeof buf) != ESP_OK) return;
        ota_display_progress((int)((off + sizeof buf) * 100 / self->size));
    }
    /* Erased otadata = boot factory (the new loader). */
    if (esp_partition_erase_range(od, 0, od->size) != ESP_OK) return;
    splash("LOADER INSTALLED", "REBOOTING...");
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
}

void ota_menu_install_to_factory(void)
{
    install_to_factory();
    splash("LOADER INSTALL FAILED", "POWER CYCLE");
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

/* ---------- WiFi state shared between the auto-join task and the menu ------ */

typedef enum { WIFI_NONE, WIFI_JOINING, WIFI_UP, WIFI_FAILED } wifi_state_t;
static volatile wifi_state_t s_wifi_state = WIFI_NONE;
static char s_ssid[33], s_pass[WIFI_PASS_MAX + 1], s_ip[20];
static SemaphoreHandle_t s_wifi_lock;    /* one scan/join at a time: the auto-join
                                          * task vs SELECT WIFI NETWORK */

/* Publish a state change: rebuild the main-menu title and poke the menu
 * task (K_REFRESH) so it redraws even though it is blocked on a key. */
static void wifi_set_state(wifi_state_t st)
{
    s_wifi_state = st;
    char t[sizeof s_title];
    switch (st) {
    case WIFI_UP:      snprintf(t, sizeof t, "TVPLUG2  %s", s_ip); break;
    case WIFI_JOINING: snprintf(t, sizeof t, "JOINING %.18s", s_ssid); break;
    case WIFI_FAILED:  snprintf(t, sizeof t, "TVPLUG2  (join failed)"); break;
    default:           snprintf(t, sizeof t, "TVPLUG2  (no wifi)"); break;
    }
    portENTER_CRITICAL(&s_title_mux);
    strcpy(s_title, t);
    portEXIT_CRITICAL(&s_title_mux);
    unsigned char k = K_REFRESH;
    if (s_keys) xQueueSend(s_keys, &k, 0);
}

/* ---------- SERVER RC: one-shot commands from the pull server ------------- */

static char s_rc[96];                     /* pending command; the menu task consumes it */

/* Called on the task that just brought WiFi up. One GET, no retry: the
 * server answers an empty body when nothing is queued, and a queued command
 * is consumed by this very request, so there is nothing to come back for. */
static void rc_fetch(void)
{
    sys_settings_t st;
    settings_load(&st);
    if (!st.server_rc) return;
    static char buf[sizeof s_rc];
    int n = http_get_text(OTA_CMD_URL, buf, sizeof buf);
    if (n < 0) { ESP_LOGW(TAG, "server rc: %s unreachable", OTA_CMD_URL); return; }
    buf[strcspn(buf, "\r\n")] = 0;              /* first line only */
    char *p = buf;
    while (*p == ' ' || *p == '\t') p++;
    if (!*p) { ESP_LOGI(TAG, "server rc: nothing queued"); return; }
    ESP_LOGW(TAG, "server rc: \"%s\"", p);
    strlcpy(s_rc, p, sizeof s_rc);
    unsigned char k = K_RCMD;
    if (s_keys) xQueueSend(s_keys, &k, 0);
}

static bool parse_onoff(const char *v, bool *out)
{
    if (!strcmp(v, "on") || !strcmp(v, "1")) { *out = true; return true; }
    if (!strcmp(v, "off") || !strcmp(v, "0")) { *out = false; return true; }
    return false;
}

/* Runs on the menu task (key_wait_raw). Nobody is at the pad by definition,
 * so a failure shows for a moment and drops back to the menu instead of
 * waiting for a key. The command was consumed on the server, so nothing here
 * can repeat on the next boot. */
static void rc_execute(void)
{
    char cmd[sizeof s_rc];
    strlcpy(cmd, s_rc, sizeof cmd);
    s_rc[0] = 0;
    char *verb = strtok(cmd, " ");
    char *a1 = strtok(NULL, " ");
    char *a2 = strtok(NULL, " ");
    if (!verb) return;
    ESP_LOGW(TAG, "server rc: executing %s %s %s", verb, a1 ? a1 : "", a2 ? a2 : "");

    const char *fail = NULL;
    if (!strcmp(verb, "run")) {
        boot_game();                               /* returns only on failure */
        fail = "RC: NO VALID GAME IMAGE";
    } else if (!strcmp(verb, "reboot")) {
        splash("SERVER RC: REBOOT", NULL);
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
    } else if (!strcmp(verb, "update") && a1) {
        if (s_wifi_state != WIFI_UP) {
            fail = "RC UPDATE: NO WIFI";
        } else {
            char url[IMG_PATH_MAX + sizeof OTA_SRV + 8];
            image_url(a1, url, sizeof url);
            ESP_LOGW(TAG, "pulling %s (server rc)", url);
            if (pull_update(url, a1)) {
                splash("UPDATE OK", "REBOOTING...");
                vTaskDelay(pdMS_TO_TICKS(800));
                esp_restart();
            }
            fail = "RC UPDATE FAILED";
        }
    } else if (!strcmp(verb, "testcard") && a1) {
        sys_settings_t st;
        settings_load(&st);
        int v = atoi(a1);
        if (v < 0 || v >= TEST_CARDS)  fail = "RC TESTCARD: BAD INDEX";
        else {
            st.test_card = (uint8_t)v;
            if (!settings_save(&st))   fail = "RC TESTCARD: SAVE FAILED";
            else {
                boot_game();                       /* returns only on failure */
                fail = "RC: NO VALID GAME IMAGE";
            }
        }
    } else if (!strcmp(verb, "set") && a1 && a2) {
        sys_settings_t st;
        settings_load(&st);
        bool b = false, ok = true;
        int v = atoi(a2);
        if      (!strcmp(a1, "snd")    && parse_onoff(a2, &b)) { st.sound_carrier = b; ota_sound_set_enabled(b); }
        else if (!strcmp(a1, "lsleep") && parse_onoff(a2, &b)) { st.line_sleep = b; }
        else if (!strcmp(a1, "bledbg") && parse_onoff(a2, &b)) { st.ble_debug = b; blelog_set_enabled(b); }
        else if (!strcmp(a1, "srvrc")  && parse_onoff(a2, &b)) { st.server_rc = b; }
        else if (!strcmp(a1, "carmode") && v >= 0 && v < CARRIER_MODES)  { st.carrier_mode = v; ota_sound_set_mode(v); }
        else if (!strcmp(a1, "cardrv")  && v >= 0 && v < CARRIER_DRIVES) { st.carrier_drive = v; ota_sound_set_drive(v); }
        else if (!strcmp(a1, "tcard")   && v >= 0 && v < TEST_CARDS)     { st.test_card = v; }
        else if (!strcmp(a1, "bletx")   && v >= 0 && v < BLE_TX_LEVELS)  { st.ble_tx = v; gamepad_set_tx_dbm(BLE_TX_DBM[v]); }
        else ok = false;
        if (!ok)                       fail = "RC SET: BAD KEY OR VALUE";
        else if (!settings_save(&st))  fail = "RC SET: SAVE FAILED";
        else {
            char m[40];
            snprintf(m, sizeof m, "RC: SET %s %s", a1, a2);
            splash(m, NULL);
            vTaskDelay(pdMS_TO_TICKS(800));
        }
    } else {
        fail = "RC: UNKNOWN COMMAND";
    }
    if (fail) {
        ESP_LOGE(TAG, "server rc: %s", fail);
        splash(fail, NULL);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

/* Boot-time join of the saved network, OFF the menu task: the menu is usable
 * from the first frame and RUN GAME never waits for WiFi to connect, time
 * out or fail (user rule, 2026-09-15). */
static void auto_join_task(void *arg)
{
    xSemaphoreTake(s_wifi_lock, portMAX_DELAY);
    wifi_set_state(WIFI_JOINING);
    bool ok = wifi_join_to(s_ssid, s_pass, s_ip, sizeof s_ip, AUTO_JOIN_TO_S);
    wifi_set_state(ok ? WIFI_UP : WIFI_FAILED);
    xSemaphoreGive(s_wifi_lock);
    if (ok) rc_fetch();                   /* SERVER RC: connection is up */
    vTaskDelete(NULL);
}

/* ---------- top-level state machine --------------------------------------- */

/* SELECT WIFI NETWORK: scan, pick, type the password, join, remember.
 * Called with s_wifi_lock held. */
static void select_network_ui(wifi_ap_record_t *aps)
{
    splash("SCANNING...", NULL);
    int n = wifi_scan(aps, MAX_APS);
    if (!n) { splash("NO NETWORKS FOUND", "ANY KEY: MENU"); key_wait(); return; }
    const char *nitems[MAX_APS];
    char labels[MAX_APS][36];
    for (int i = 0; i < n; i++) {
        snprintf(labels[i], sizeof labels[0], "%-20.20s %4d",
                 (char *)aps[i].ssid, aps[i].rssi);
        nitems[i] = labels[i];
    }
    int pick = menu_pick("JOIN WIFI", nitems, n, 0, NULL);
    if (pick < 0) return;
    char ssid[33], pass[WIFI_PASS_MAX + 1] = "";
    strlcpy(ssid, (char *)aps[pick].ssid, sizeof ssid);
    if (aps[pick].authmode != WIFI_AUTH_OPEN && !keyboard(ssid, pass, sizeof pass))
        return;
    splash("JOINING", ssid);
    strlcpy(s_ssid, ssid, sizeof s_ssid);
    wifi_set_state(WIFI_JOINING);
    bool ok = wifi_join_to(ssid, pass, s_ip, sizeof s_ip, CONNECT_TO_S);
    if (ok) {
        strlcpy(s_pass, pass, sizeof s_pass);
        creds_save(ssid, pass);
    }
    wifi_set_state(ok ? WIFI_UP : WIFI_FAILED);
    if (!ok) { splash("JOIN FAILED", "ANY KEY: MENU"); key_wait(); }
    else rc_fetch();                      /* SERVER RC: runs at the next key wait */
}

/* DOWNLOAD IMAGE: fetch the server's index, let the user pick an entry,
 * pull it into ota_0 and reboot into it. Needs WiFi up. */
static void download_image_ui(void)
{
    static char text[1024];
    static char paths[MAX_IMAGES][IMG_PATH_MAX], labels[MAX_IMAGES][IMG_LABEL_MAX];

    splash("FETCHING INDEX", OTA_SRV);
    if (http_get_text(OTA_INDEX_URL, text, sizeof text) <= 0) {
        splash("INDEX FETCH FAILED", "ANY KEY: MENU");
        key_wait();
        return;
    }
    int n = index_parse(text, paths, labels, MAX_IMAGES);
    if (!n) {
        splash("INDEX EMPTY", "ANY KEY: MENU");
        key_wait();
        return;
    }
    const char *items[MAX_IMAGES];
    for (int i = 0; i < n; i++) items[i] = labels[i];
    int pick = menu_pick("DOWNLOAD IMAGE", items, n, 0, NULL);
    if (pick < 0) return;

    char url[IMG_PATH_MAX + sizeof OTA_SRV + 8];
    image_url(paths[pick], url, sizeof url);
    ESP_LOGW(TAG, "pulling %s (%s)", url, labels[pick]);
    if (pull_update(url, labels[pick])) {
        splash("UPDATE OK", "REBOOTING...");
        vTaskDelay(pdMS_TO_TICKS(800));
        esp_restart();
    }
    splash("UPDATE FAILED", "ANY KEY: MENU");
    key_wait();
}

/* SETTINGS: system-level switches persisted in NVS ("sys") and read by both
 * the loader and the game at boot. A toggles, B returns. Sound carrier and
 * BLE debugging take effect here immediately; line sleep is acted on by the
 * game; SERVER RC is consulted the next time WiFi comes up. */
static void settings_ui(void)
{
    sys_settings_t st;
    settings_load(&st);
    int cur = 0;
    for (;;) {
        char l0[32], l1[32], l2[32], l3[32], l4[32], l5[32], l6[32], l7[32];
        snprintf(l0, sizeof l0, "%-18s%s", "SOUND CARRIER", st.sound_carrier ? "ON" : "OFF");
        snprintf(l1, sizeof l1, "%-18s%s", "LINE DETECT SLEEP", st.line_sleep ? "ON" : "OFF");
        snprintf(l2, sizeof l2, "%-18s%s", "BLE DEBUGGING", st.ble_debug ? "ON" : "OFF");
        /* Carrier experiments (settings.h): applied to the loader's own
         * carrier at once, so the effect on this picture is immediate. */
        snprintf(l3, sizeof l3, "CARRIER  %s", carrier_mode(st.carrier_mode)->label);
        snprintf(l4, sizeof l4, "CARRIER DRIVE  %uMA", CARRIER_DRIVE_MA[st.carrier_drive]);
        snprintf(l5, sizeof l5, "%-18s%s", "SERVER RC", st.server_rc ? "ON" : "OFF");
        /* BLE transmit power, shared with the game. Applied to this loader's
         * own radio at once, so a change is live before you leave the page. */
        snprintf(l7, sizeof l7, "%-14s%s", "BLE TX POWER", ble_tx_name(st.ble_tx));
        /* Game-side RF test patterns (main/testcard.c); the game reads it at boot. */
        snprintf(l6, sizeof l6, "%-12s%s", "TEST CARD", test_card_name(st.test_card));
        const char *items[] = { l0, l1, l2, l3, l4, l5, l7, l6 };
        cur = menu_pick("SETTINGS", items, 8, cur, "A:TOGGLE B:BACK");
        if (cur < 0) return;
        if (cur == 0) {
            st.sound_carrier = !st.sound_carrier;
            ota_sound_set_enabled(st.sound_carrier);
        } else if (cur == 1) {
            st.line_sleep = !st.line_sleep;
        } else if (cur == 2) {
            st.ble_debug = !st.ble_debug;
            blelog_set_enabled(st.ble_debug);
        } else if (cur == 3) {
            st.carrier_mode = (st.carrier_mode + 1) % CARRIER_MODES;
            ota_sound_set_mode(st.carrier_mode);
        } else if (cur == 4) {
            st.carrier_drive = (st.carrier_drive + 1) % CARRIER_DRIVES;
            ota_sound_set_drive(st.carrier_drive);
        } else if (cur == 5) {
            st.server_rc = !st.server_rc;
        } else if (cur == 6) {
            st.ble_tx = (st.ble_tx + 1) % BLE_TX_LEVELS;
            gamepad_set_tx_dbm(BLE_TX_DBM[st.ble_tx]);
        } else {
            st.test_card = (st.test_card + 1) % TEST_CARDS;
        }
        if (!settings_save(&st)) {
            splash("SAVE FAILED", "ANY KEY: MENU");
            key_wait();
            return;
        }
        ESP_LOGW(TAG, "settings: sound carrier %s, line sleep %s, BLE debug %s, "
                 "carrier %s, drive %u mA, server rc %s, BLE TX %s, test card %s",
                 st.sound_carrier ? "ON" : "OFF", st.line_sleep ? "ON" : "OFF",
                 st.ble_debug ? "ON" : "OFF", carrier_mode(st.carrier_mode)->label,
                 CARRIER_DRIVE_MA[st.carrier_drive], st.server_rc ? "ON" : "OFF",
                 ble_tx_name(st.ble_tx), test_card_name(st.test_card));
    }
}

static void menu_task(void *arg)
{
    static wifi_ap_record_t aps[MAX_APS];

    /* Unattended flows go through SERVER RC (rc_fetch/rc_execute): the
     * NVS-staged directive of the retired BLE command channel is gone. */
    for (;;) {
        /* ---- main management menu (title = s_title, live) --------------- */
        const char *items[] = { "RUN GAME", "SELECT WIFI NETWORK",
                                "DOWNLOAD IMAGE", "SETTINGS" };
        int a = menu_pick(s_title, items, 4, 0, NULL);

        if (a == 0) {                                  /* RUN GAME */
            boot_game();
            splash("NO VALID GAME IMAGE", "ANY KEY: MENU");
            key_wait();
        } else if (a == 1) {                           /* SELECT WIFI */
            if (xSemaphoreTake(s_wifi_lock, 0) != pdTRUE) {
                splash("AUTO-JOIN IN PROGRESS", "WAITING...");
                xSemaphoreTake(s_wifi_lock, portMAX_DELAY);
            }
            select_network_ui(aps);
            xSemaphoreGive(s_wifi_lock);
        } else if (a == 2) {                           /* DOWNLOAD IMAGE */
            if (s_wifi_state != WIFI_UP) {
                if (s_wifi_state == WIFI_JOINING)
                    splash("WIFI STILL JOINING", "TRY AGAIN IN A MOMENT");
                else
                    splash("NO WIFI", "SELECT A NETWORK FIRST");
                vTaskDelay(pdMS_TO_TICKS(1200));
                continue;
            }
            download_image_ui();
        } else if (a == 3) {                           /* SETTINGS */
            settings_ui();
        }
    }
}

void ota_menu_start(void)
{
    s_keys = xQueueCreate(16, sizeof(unsigned char));
    s_wifi_ev = xEventGroupCreate();
    s_wifi_lock = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_set_state(WIFI_NONE);                       /* initial title */
    /* Auto-join first so it owns s_wifi_lock before any pad input can. */
    if (creds_load(s_ssid, sizeof s_ssid, s_pass, sizeof s_pass))
        xTaskCreatePinnedToCore(auto_join_task, "wifijoin", 6144, NULL, 4, NULL, 0);   /* + rc_fetch's HTTP GET */
    xTaskCreatePinnedToCore(menu_task, "otamenu", 6144, NULL, 5, NULL, 0);
}
