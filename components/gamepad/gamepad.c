/*
 * Xbox Series X|S controller over BLE -> Doom key events.
 *
 * This is a self-contained HOGP (HID-over-GATT) *client* on raw NimBLE,
 * modeled on the proven asukiaaa/XboxSeriesXControllerESP32 flow. It
 * deliberately does NOT use ESP-IDF's esp_hidh: on IDF v6.1-dev the NimBLE
 * HID host (nimble_hidh.c) never sets dev->connected (so CLOSE events are
 * never delivered), and its open flow deadlocks waiting for a CCCD write
 * completion mid-subscription — the pad stayed mute and blinking.
 *
 * Flow, all driven from the GAP event callback (NimBLE host task):
 *   scan (esp_hid_gap.c) -> connect -> initiate security (the pad never
 *   requests it and rejects reads until encrypted) -> on encryption: discover
 *   HID service -> characteristics -> CCCDs -> subscribe to every
 *   notify-capable characteristic -> parse notifications into Doom keys.
 *
 * Self-healing: if the link dies right after security starts, our stored
 *   bond is stale (pad was re-synced) — delete it so the next attempt runs a
 *   fresh pairing. REPEAT_PAIRING drops the old bond and retries likewise.
 *
 * Report layout (same as xpadneo / asukiaaa's parser):
 *   [0..7]  LX LY RX RY  uint16 LE, 0..65535, centre 32768 (Y grows downward)
 *   [8..11] LT RT        uint16 LE, 0..1023
 *   [12]    dpad hat     0 = idle, 1..8 = N NE E SE S SW W NW
 *   [13]    A=01 B=02 X=08 Y=10 LB=40 RB=80
 *   [14]    View=04 Menu=08 Xbox=10 LS=20 RS=40
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "esp_hid_gap.h"
#include "gamepad.h"
#include "blelog.h"

#include "doomkeys.h"

void doom_push_key(int pressed, unsigned char doom_key); /* doomgeneric_esp32.c */
int doom_current_weapon_slot(void);                      /* doomgeneric_esp32.c */
int doom_weapon_slot_owned(int slot);                    /* doomgeneric_esp32.c */
void ble_store_config_init(void);

static const char *TAG = "gamepad";

/* ---------- link / discovery state (touched from the NimBLE host task; the
 * scanner task only reads the volatile flags) ---------- */

enum link_state { LS_IDLE, LS_CONNECTING, LS_UP };

static volatile enum link_state s_state = LS_IDLE;
static uint16_t  s_conn = BLE_HS_CONN_HANDLE_NONE;
static ble_addr_t s_peer;

/* HID service layout */
#define MAX_CHRS 12
typedef struct {
    uint16_t def;       /* characteristic declaration handle */
    uint16_t val;       /* value handle                      */
    uint16_t cccd;      /* 0x2902 descriptor handle (0=none) */
    uint8_t  props;
} hog_chr_t;

static uint16_t  s_hid_start, s_hid_end;
static hog_chr_t s_chrs[MAX_CHRS];
static int       s_nchr;
static int       s_walk;    /* descriptor-walk / subscribe progress index */
static int       s_nsubs;

/* ---------- report parsing + Doom mapping ---------- */

enum {
    LK_UP, LK_DOWN, LK_LEFT, LK_RIGHT,   /* dpad: menus + coarse move */
    LK_FIRE, LK_USE, LK_ENTER, LK_ESC, LK_TAB, LK_RUN, LK_YES,
    LK_COUNT
};

static const unsigned char lk_doomkey[LK_COUNT] = {
    KEY_UPARROW, KEY_DOWNARROW, KEY_LEFTARROW, KEY_RIGHTARROW,
    KEY_FIRE, KEY_USE, KEY_ENTER, KEY_ESCAPE, KEY_TAB, KEY_RSHIFT, 'y',
};

/* Analog channel into G_BuildTiccmd (defined in doomgeneric_esp32.c).
 * GZDoom-style: sticks feed the movement command directly — proportional
 * and instant, no key emulation, no turn-acceleration ramp. */
extern volatile int doom_analog_forward, doom_analog_side, doom_analog_turn;

static uint32_t s_prev_keys;
static uint8_t  s_prev_btn13;
static uint8_t  s_prev_btn14;

/* OTA-menu entry callback, fired by the Xbox/guide button (the View+Menu
 * 3 s hold combo was dropped 2026-09-15 as redundant). */
static void (*s_combo_cb)(void);

bool gamepad_connected(void)
{
    return s_state == LS_UP;
}

void gamepad_set_combo_cb(void (*cb)(void))
{
    s_combo_cb = cb;
}

/* weapon-digit key held briefly (see Y handling); released by this timer */
static esp_timer_handle_t s_digit_timer;
static volatile unsigned char s_digit_down;

static void digit_release_cb(void *arg)
{
    if (s_digit_down) {
        doom_push_key(0, s_digit_down);
        s_digit_down = 0;
    }
}

/* Stick/trigger to on/off with hysteresis: harder to press than to release,
 * so a value hovering at the threshold doesn't spam key events. */
static inline bool axis_on(bool was_on, int32_t excursion, int32_t on_thr, int32_t off_thr)
{
    return excursion > (was_on ? off_thr : on_thr);
}

#define TRIG_ON      280   /* of 0..1023 */
#define TRIG_OFF     160

/* Stick value -> +-out_max with an inner deadzone and linear rescale
 * (deflection just past the deadzone maps to a small output, full
 * deflection to out_max). */
static int32_t stick_scale(int32_t v, int32_t out_max)
{
    const int32_t DZ = 5000;            /* ~15% of half-scale */
    int32_t a = v < 0 ? -v : v;
    if (a <= DZ) {
        return 0;
    }
    if (a > 32767) {
        a = 32767;
    }
    int32_t out = (a - DZ) * out_max / (32767 - DZ);
    return v < 0 ? -out : out;
}

static void release_all_keys(void)
{
    for (int i = 0; i < LK_COUNT; i++) {
        if (s_prev_keys & (1u << i)) {
            doom_push_key(0, lk_doomkey[i]);
        }
    }
    s_prev_keys = 0;
    s_prev_btn13 = 0;
    doom_analog_forward = 0;
    doom_analog_side = 0;
    doom_analog_turn = 0;
}

static void handle_report(const uint8_t *d, int len)
{
    if (len < 16) {
        /* Short reports. The Xbox/guide button lives on its own consumer-
         * control characteristic: a 2-byte usage bitmap, bit 0 = guide
         * (1-byte short reports are battery level). Guide press -> OTA menu
         * handover, immediately (user choice; the loader's BOOT GAME and
         * 10-min idle timeout cover accidental presses). */
        if (len == 2 && (d[0] & 0x01) && s_combo_cb) s_combo_cb();
        return;
    }

    int32_t lx = (d[0] | d[1] << 8) - 32768;
    int32_t ly = (d[2] | d[3] << 8) - 32768;
    int32_t rx = (d[4] | d[5] << 8) - 32768;
    int32_t lt = d[8] | d[9] << 8;
    int32_t rt = d[10] | d[11] << 8;
    uint8_t hat = d[12], b13 = d[13], b14 = d[14];

    uint32_t want = 0;
    uint32_t prev = s_prev_keys;
#define WAS(lk)        ((prev >> (lk)) & 1)
#define SET(lk, cond)  do { if (cond) want |= 1u << (lk); } while (0)

    /* dpad (menus + coarse movement) */
    SET(LK_UP,    hat == 8 || hat == 1 || hat == 2);
    SET(LK_RIGHT, hat == 2 || hat == 3 || hat == 4);
    SET(LK_DOWN,  hat == 4 || hat == 5 || hat == 6);
    SET(LK_LEFT,  hat == 6 || hat == 7 || hat == 8);

    /* sticks: analog, straight into the movement command (GZDoom-style).
     * Left stick = move/strafe (+-50/40 = vanilla run/strafe-run speed),
     * right stick X = turn rate with a squared response curve for fine aim
     * (max 1024/tic, between vanilla's normal 640 and fast 1280). */
    doom_analog_forward = stick_scale(-ly, 50);
    doom_analog_side    = stick_scale(lx, 40);
    int32_t t = stick_scale(rx, 1024);
    doom_analog_turn    = (int)((t * (t < 0 ? -t : t)) / 1024);

    /* fire: RT trigger (analog) or either bumper (digital); LT: run */
    SET(LK_FIRE, axis_on(WAS(LK_FIRE), rt, TRIG_ON, TRIG_OFF) || (b13 & 0xC0));
    SET(LK_RUN,  axis_on(WAS(LK_RUN),  lt, TRIG_ON, TRIG_OFF));

    /* face + system buttons */
    SET(LK_ENTER, b13 & 0x01);              /* A: menu select        */
    SET(LK_ESC,  (b13 & 0x02) || (b14 & 0x08)); /* B or Menu: back   */
    SET(LK_USE,   b13 & 0x08);              /* X: open doors         */
    SET(LK_TAB,   b14 & 0x40);              /* RS click: automap     */
    SET(LK_YES,   b14 & 0x04);              /* View: 'y' confirms    */

    uint32_t changed = want ^ prev;
    for (int i = 0; i < LK_COUNT; i++) {
        if (changed & (1u << i)) {
            doom_push_key((want >> i) & 1, lk_doomkey[i]);
        }
    }
    s_prev_keys = want;

    /* Y cycles to the next OWNED weapon (empty slots skipped by peeking at
     * the player's inventory), starting from the actually-equipped weapon so
     * pickups/auto-switches can't desync the cycle. The slot digit must stay
     * DOWN for a few game tics: Doom's weapon keys are level-checked per tic
     * in G_BuildTiccmd, so an instant press+release is never seen —
     * digit_release_cb lifts it after 100 ms. */
    /* Xbox/guide button (b14 bit 0x10 in the MAIN report on Series pads,
     * fw 5.x -- NOT the separate 2-byte report, which is the Share button):
     * immediate OTA menu handover. */
    if ((b14 & 0x10) && !(s_prev_btn14 & 0x10) && s_combo_cb) s_combo_cb();
    s_prev_btn14 = b14;

    uint8_t pressed13 = b13 & ~s_prev_btn13;
    if (pressed13 & 0x10) {
        int cur = doom_current_weapon_slot();
        if (cur > 0) {
            int slot = cur;
            for (int i = 0; i < 7; i++) {
                slot = slot % 7 + 1;
                if (slot != cur && doom_weapon_slot_owned(slot)) {
                    break;
                }
            }
            if (slot != cur && doom_weapon_slot_owned(slot)) {
                if (s_digit_down) {     /* rapid re-press: lift previous */
                    esp_timer_stop(s_digit_timer);
                    doom_push_key(0, s_digit_down);
                }
                s_digit_down = (unsigned char)('0' + slot);
                doom_push_key(1, s_digit_down);
                esp_timer_start_once(s_digit_timer, 100 * 1000);
            }
        }
    }
    s_prev_btn13 = b13;
}

/* ---------- HOGP discovery + subscription (all on the NimBLE host task) ---- */

static void hog_subscribe_next(void);

static int on_cccd_written(uint16_t conn_handle, const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr, void *arg)
{
    int i = (int)(intptr_t)arg;
    ESP_LOGI(TAG, "subscribe chr val=%u cccd=%u: status=%d",
             s_chrs[i].val, s_chrs[i].cccd, error->status);
    if (error->status == 0) {
        s_nsubs++;
    }
    s_walk = i + 1;
    hog_subscribe_next();
    return 0;
}

static void hog_subscribe_next(void)
{
    while (s_walk < s_nchr) {
        if ((s_chrs[s_walk].props & BLE_GATT_CHR_PROP_NOTIFY) && s_chrs[s_walk].cccd) {
            uint8_t v[2] = { 1, 0 };    /* enable notifications */
            int rc = ble_gattc_write_flat(s_conn, s_chrs[s_walk].cccd, v, 2,
                                          on_cccd_written, (void *)(intptr_t)s_walk);
            if (rc == 0) {
                return;                 /* continue from on_cccd_written */
            }
            ESP_LOGW(TAG, "CCCD write start failed: rc=%d", rc);
        }
        s_walk++;
    }
    ESP_LOGI(TAG, "HOGP ready: %d input subscriptions — connected: Xbox pad", s_nsubs);
    s_state = LS_UP;
}

static void hog_walk_next_dscs(void);

static int on_dsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                  uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    int i = (int)(intptr_t)arg;
    if (error->status == 0 && dsc &&
        ble_uuid_u16(&dsc->uuid.u) == 0x2902 /* CCCD */) {
        s_chrs[i].cccd = dsc->handle;
    } else if (error->status == BLE_HS_EDONE) {
        s_walk = i + 1;
        hog_walk_next_dscs();
    } else if (error->status != 0) {
        ESP_LOGW(TAG, "descriptor discovery error %d on chr %u", error->status, s_chrs[i].val);
        s_walk = i + 1;
        hog_walk_next_dscs();
    }
    return 0;
}

static void hog_walk_next_dscs(void)
{
    while (s_walk < s_nchr) {
        int i = s_walk;
        if (s_chrs[i].props & BLE_GATT_CHR_PROP_NOTIFY) {
            uint16_t end = (i + 1 < s_nchr) ? (uint16_t)(s_chrs[i + 1].def - 1) : s_hid_end;
            if (end > s_chrs[i].val) {
                int rc = ble_gattc_disc_all_dscs(s_conn, s_chrs[i].val, end,
                                                 on_dsc, (void *)(intptr_t)i);
                if (rc == 0) {
                    return;             /* continue from on_dsc(EDONE) */
                }
                ESP_LOGW(TAG, "disc_all_dscs rc=%d", rc);
            }
        }
        s_walk++;
    }
    /* descriptor walk finished -> subscribe pass */
    s_walk = 0;
    hog_subscribe_next();
}

static int on_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                  const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == 0 && chr) {
        if (s_nchr < MAX_CHRS) {
            s_chrs[s_nchr++] = (hog_chr_t){
                .def = chr->def_handle, .val = chr->val_handle,
                .props = chr->properties, .cccd = 0,
            };
        }
    } else if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "HID service: %d characteristics", s_nchr);
        s_walk = 0;
        hog_walk_next_dscs();
    } else {
        ESP_LOGW(TAG, "characteristic discovery error %d", error->status);
    }
    return 0;
}

static int on_svc(uint16_t conn_handle, const struct ble_gatt_error *error,
                  const struct ble_gatt_svc *svc, void *arg)
{
    if (error->status == 0 && svc) {
        s_hid_start = svc->start_handle;
        s_hid_end = svc->end_handle;
    } else if (error->status == BLE_HS_EDONE) {
        if (!s_hid_start) {
            ESP_LOGW(TAG, "no HID service found, dropping link");
            ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        s_nchr = 0;
        ble_gattc_disc_all_chrs(s_conn, s_hid_start, s_hid_end, on_chr, NULL);
    }
    return 0;
}

static void hog_start_discovery(void)
{
    s_hid_start = s_hid_end = 0;
    int rc = ble_gattc_disc_svc_by_uuid(s_conn, BLE_UUID16_DECLARE(0x1812), on_svc, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "service discovery start failed: rc=%d", rc);
    }
}

/* ---------- GAP events ---------- */

static int hog_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn = event->connect.conn_handle;
            ESP_LOGI(TAG, "link up, starting security");
            /* The pad never asks for security itself and refuses HID access
             * until encrypted — always initiate from our side. With a stored
             * bond this re-encrypts, otherwise it pairs (Just Works). */
            int rc = ble_gap_security_initiate(s_conn);
            if (rc != 0 && rc != BLE_HS_EALREADY) {
                ESP_LOGW(TAG, "security initiate rc=%d, dropping", rc);
                ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
            }
        } else {
            ESP_LOGW(TAG, "connect failed: status=%d", event->connect.status);
            s_conn = BLE_HS_CONN_HANDLE_NONE;
            s_state = LS_IDLE;
        }
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
                /* conn_itvl is in 1.25 ms units. The pad can renegotiate
                 * params per connection — log them to correlate with any
                 * boot-dependent performance differences. */
                ESP_LOGI(TAG, "encrypted=%d bonded=%d key_size=%d itvl=%.1fms lat=%d timeout=%dms — discovering HID service",
                         desc.sec_state.encrypted, desc.sec_state.bonded,
                         desc.sec_state.key_size, desc.conn_itvl * 1.25,
                         desc.conn_latency, desc.supervision_timeout * 10);
            }
            hog_start_discovery();
        } else {
            /* Encryption failed: classic stale-bond mismatch (pad was
             * re-synced and lost our key). Drop our bond so the next attempt
             * runs a fresh pairing — needs the pad in sync mode. */
            ESP_LOGW(TAG, "encryption failed (status=%d) — deleting stale bond; "
                     "re-sync the pad if this repeats", event->enc_change.status);
            if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
                ble_store_util_delete_peer(&desc.peer_id_addr);
            }
            ble_gap_terminate(event->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "disconnected (reason=%d), will rescan", event->disconnect.reason);
        /* If the link died within seconds of starting security and we never
         * got encrypted, the pad silently rejected our stored key. */
        if (s_state == LS_CONNECTING &&
            event->disconnect.reason == 520 /* supervision timeout */) {
            ESP_LOGW(TAG, "link died before HID came up — deleting possibly "
                     "stale bond; hold the pad's sync button to re-pair");
            ble_store_util_delete_peer(&event->disconnect.conn.peer_id_addr);
        }
        release_all_keys();
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        s_state = LS_IDLE;
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint8_t buf[32];
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len > sizeof buf) {
            len = sizeof buf;
        }
        os_mbuf_copydata(event->notify_rx.om, 0, len, buf);
        static bool dumped;
        if (!dumped) {
            dumped = true;
            ESP_LOGI(TAG, "first report: attr=%u len=%u", event->notify_rx.attr_handle, len);
            ESP_LOG_BUFFER_HEX(TAG, buf, len);
        }
        handle_report(buf, len);
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* Pad wants to pair although we have a bond: drop ours, accept. */
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        /* Just Works should never ask, but answer permissively if it does. */
        struct ble_sm_io pkey = { 0 };
        if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
            pkey.action = BLE_SM_IOACT_NUMCMP;
            pkey.numcmp_accept = 1;
        } else {
            pkey.action = event->passkey.params.action;
            pkey.passkey = 0;
        }
        ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        return 0;
    }

    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
    case BLE_GAP_EVENT_L2CAP_UPDATE_REQ:
        /* The pad asks for a short connection interval (7.5-15 ms), making
         * radio load (and PSRAM traffic) vary per boot. On the LL event
         * self_params is writable — clamp there. On the L2CAP event
         * self_params is NULL (accept/reject only): accept, and the
         * CONN_UPDATE case below re-negotiates if the result is too fast. */
        if (event->conn_update_req.self_params && event->conn_update_req.peer_params) {
            *event->conn_update_req.self_params = *event->conn_update_req.peer_params;
            if (event->conn_update_req.self_params->itvl_min < 12) {
                event->conn_update_req.self_params->itvl_min = 12;
            }
            if (event->conn_update_req.self_params->itvl_max < 12) {
                event->conn_update_req.self_params->itvl_max = 12;
            }
        }
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* the pad renegotiated connection parameters mid-session */
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
            ESP_LOGI(TAG, "conn update: itvl=%.1fms lat=%d timeout=%dms (status=%d)",
                     desc.conn_itvl * 1.25, desc.conn_latency,
                     desc.supervision_timeout * 10, event->conn_update.status);
            if (desc.conn_itvl < 12) {
                /* pad talked us below 15 ms via L2CAP — as central, take it
                 * back up (bounded identical radio load every boot) */
                struct ble_gap_upd_params up = {
                    .itvl_min = 12, .itvl_max = 24, .latency = 0,
                    .supervision_timeout = 300,
                    .min_ce_len = 0, .max_ce_len = 0,
                };
                int rc = ble_gap_update_params(event->conn_update.conn_handle, &up);
                ESP_LOGI(TAG, "re-clamping interval to >=15ms: rc=%d", rc);
            }
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        return 0;

    default:
        return 0;
    }
}

/* ---------- connection manager ---------- */

static void ble_host_task(void *param)
{
    nimble_port_run();              /* returns only on nimble_port_stop() */
    nimble_port_freertos_deinit();
}

static void gamepad_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(300)); /* let the host sync before first scan */

    /* Persistence check: how many bonds survived this boot (NVS). */
    int bonds = 0;
    ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &bonds);
    ESP_LOGI(TAG, "bonds in store at boot: %d", bonds);

    for (;;) {
        /* Hunt aggressively for the first 30 s after boot (the user is
         * usually waiting for the pad right then); relax afterwards. The
         * scan API blocks for its FULL window before returning results, so
         * window length directly delays (re)connect. */
        bool startup = xTaskGetTickCount() < pdMS_TO_TICKS(30000);
        uint32_t scan_s = startup ? 1 : 2;
        uint32_t idle_ms = startup ? 150 : 1000;

        if (s_state != LS_IDLE) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        size_t n = 0;
        esp_hid_scan_result_t *results = NULL;
        esp_hid_scan(scan_s, &n, &results);

        esp_hid_scan_result_t *best = NULL;
        for (esp_hid_scan_result_t *r = results; r; r = r->next) {
            if (r->transport != ESP_HID_TRANSPORT_BLE) {
                continue;
            }
            if (!best || r->rssi > best->rssi) {
                best = r;
            }
        }
        if (best) {
            ESP_LOGI(TAG, "opening %02x:%02x:%02x:%02x:%02x:%02x (rssi %d, name %s)",
                     best->bda[5], best->bda[4], best->bda[3],
                     best->bda[2], best->bda[1], best->bda[0],
                     best->rssi, best->name ? best->name : "?");
            s_peer.type = best->ble.addr_type;
            memcpy(s_peer.val, best->bda, 6);

            uint8_t own_addr_type = 0;
            ble_hs_id_infer_auto(0, &own_addr_type);
            /* Default conn params except supervision_timeout: 1.5 s instead
             * of 2.56 s, so the doomed first bond attempt after a board
             * reset (pad silently rejects our restored key) fails faster.
             * Still 30+ intervals of slack for normal-play interference. */
            struct ble_gap_conn_params cp = {
                .scan_itvl = 16, .scan_window = 16,
                .itvl_min = 24, .itvl_max = 40,
                .latency = 0, .supervision_timeout = 150,
                .min_ce_len = 0, .max_ce_len = 0,
            };
            s_state = LS_CONNECTING;
            int rc = ble_gap_connect(own_addr_type, &s_peer, 10000, &cp,
                                     hog_gap_event, NULL);
            if (rc != 0) {
                ESP_LOGW(TAG, "ble_gap_connect rc=%d", rc);
                s_state = LS_IDLE;
            }
        } else if (n == 0) {
            ESP_LOGI(TAG, "no gamepad found, rescanning (hold the pad's sync button to pair)");
        }
        if (results) {
            esp_hid_scan_results_free(results);
        }
        vTaskDelay(pdMS_TO_TICKS(idle_ms));
    }
}

void gamepad_start(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));

    ble_store_config_init();        /* bond keys <-> NVS (BT_NIMBLE_NVS_PERSIST) */
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    const esp_timer_create_args_t targs = {
        .callback = digit_release_cb, .name = "wpn_digit",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_digit_timer));

    /* Wireless log stream (USB console is dead on the assembled unit):
     * GATT service must be registered before the host starts. */
    blelog_init();

    ESP_ERROR_CHECK(esp_nimble_enable(ble_host_task));

    /* Scanner/reconnector on core 0 with the rest of the BT stack. BT lived
     * on core 1 originally, but with a pad CONNECTED its radio scheduling
     * preempts the video encoder enough to cause visible engine-late frame
     * repeats. Core 0 (Doom) has 10-24 ms of idle backpressure wait per
     * frame at the 26 fps pace — BT fits in that slack without costing fps,
     * and core 1 goes back to being exclusively the video engine's. */
    xTaskCreatePinnedToCore(gamepad_task, "gamepad", 4096, NULL, 6, NULL, 0);

    /* Transmit power, again: esp_nimble_enable() above resets what was set at
     * controller-enable time. Verified by measuring advertising RSSI, not by
     * the return code -- see esp_hid_gap_apply_tx_power(). */
    esp_hid_gap_apply_tx_power();
}
