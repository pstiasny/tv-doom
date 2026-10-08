#include "settings.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "settings";
#define NS "sys"

static const sys_settings_t DEFAULTS = { .sound_carrier = true, .line_sleep = false,
                                         .ble_debug = true, .carrier_mode = 0,
                                         .carrier_drive = 2, .server_rc = false,
                                         .ble_tx = 0, .test_card = 0 };

static const char *const TEST_CARD_NAMES[TEST_CARDS] = {
    "OFF", "CYCLE", "BLANK", "WHITE", "GRAY50", "STAIRS", "STAIRS+RED", "BARS", "EDGES", "TONE",
    "CALIB" };

const char *test_card_name(uint8_t idx)
{
    return TEST_CARD_NAMES[idx < TEST_CARDS ? idx : 0];
}

static const carrier_mode_t MODES[CARRIER_MODES] = {
    { "32M DDS",      32000000,  11,   64, true  },  /* 5.5 MHz, as built      */
    { "40M DDS",      40000000,  11,   80, true  },  /* 5.5 MHz                */
    { "32M HALFLINE", 32000000, 705, 4096, true  },  /* 5.5078125 = 352.5 f_l  */
    { "40M HALFLINE", 40000000, 141, 1024, true  },  /* 5.5078125              */
    { "5.33M SQUARE", 32000000,   1,    6, false },  /* jitter-free diagnostic */
    { "DC TEST",      32000000,   0,    1, true  },  /* DMA + load, no signal  */
};

const uint8_t CARRIER_DRIVE_MA[CARRIER_DRIVES] = { 5, 10, 20, 40 };

const int8_t BLE_TX_DBM[BLE_TX_LEVELS] = { 9, 3, -3, -9, -15, -21 };
static const char *const BLE_TX_NAMES[BLE_TX_LEVELS] = {
    "+9 dBm", "+3 dBm", "-3 dBm", "-9 dBm", "-15 dBm", "-21 dBm" };

const char *ble_tx_name(uint8_t idx)
{
    return BLE_TX_NAMES[idx < BLE_TX_LEVELS ? idx : 0];
}

const carrier_mode_t *carrier_mode(uint8_t m)
{
    return &MODES[m < CARRIER_MODES ? m : 0];
}

static uint32_t gcd32(uint32_t a, uint32_t b)
{
    while (b) { uint32_t t = a % b; a = b; b = t; }
    return a;
}

size_t carrier_build_chunk(const carrier_mode_t *m, uint32_t *buf, size_t max_words)
{
    uint32_t p = m->fc_num, q = m->fc_den;
    uint32_t pat_words = q / gcd32(q, 32);              /* lcm(q, 32) / 32 */
    size_t words = (max_words / pat_words) * pat_words;
    uint64_t n = 0;
    for (size_t w = 0; w < words; w++) {
        uint32_t word = 0;
        for (int b = 0; b < 32; b++, n++)
            word = (word << 1) | (2 * ((n * p) % q) >= q);
        buf[w] = word;
    }
    return words;
}

static bool get_bool(nvs_handle_t h, const char *key, bool dflt)
{
    uint8_t v;
    return nvs_get_u8(h, key, &v) == ESP_OK ? v != 0 : dflt;
}

static uint8_t get_u8(nvs_handle_t h, const char *key, uint8_t dflt, uint8_t limit)
{
    uint8_t v;
    return nvs_get_u8(h, key, &v) == ESP_OK && v < limit ? v : dflt;
}

static void log_settings(const char *what, const sys_settings_t *s)
{
    ESP_LOGI(TAG, "%s: sound carrier %s, line sleep %s, BLE debug %s, carrier %s, drive %u mA, server rc %s, BLE TX %s, test card %s",
             what, s->sound_carrier ? "ON" : "OFF", s->line_sleep ? "ON" : "OFF",
             s->ble_debug ? "ON" : "OFF", carrier_mode(s->carrier_mode)->label,
             CARRIER_DRIVE_MA[s->carrier_drive], s->server_rc ? "ON" : "OFF",
             ble_tx_name(s->ble_tx), test_card_name(s->test_card));
}

void settings_load(sys_settings_t *s)
{
    *s = DEFAULTS;
    /* ESP_OK when already initialised by someone else. Do NOT erase on
     * failure here: the BLE bonds live in the same partition, and the app's
     * own NVS bring-up owns that decision. */
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS init %s -- using defaults", esp_err_to_name(err));
        return;
    }
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        log_settings("nothing stored, defaults", s);
        return;
    }
    s->sound_carrier = get_bool(h, "snd", DEFAULTS.sound_carrier);
    s->line_sleep    = get_bool(h, "lsleep", DEFAULTS.line_sleep);
    s->ble_debug     = get_bool(h, "bledbg", DEFAULTS.ble_debug);
    s->carrier_mode  = get_u8(h, "carmode", DEFAULTS.carrier_mode, CARRIER_MODES);
    s->carrier_drive = get_u8(h, "cardrv", DEFAULTS.carrier_drive, CARRIER_DRIVES);
    s->server_rc     = get_bool(h, "srvrc", DEFAULTS.server_rc);
    s->ble_tx        = get_u8(h, "bletx", DEFAULTS.ble_tx, BLE_TX_LEVELS);
    s->test_card     = get_u8(h, "tcard", DEFAULTS.test_card, TEST_CARDS);
    nvs_close(h);
    log_settings("loaded", s);
}

bool settings_save(const sys_settings_t *s)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = nvs_set_u8(h, "snd", s->sound_carrier) == ESP_OK &&
              nvs_set_u8(h, "lsleep", s->line_sleep) == ESP_OK &&
              nvs_set_u8(h, "bledbg", s->ble_debug) == ESP_OK &&
              nvs_set_u8(h, "carmode", s->carrier_mode) == ESP_OK &&
              nvs_set_u8(h, "cardrv", s->carrier_drive) == ESP_OK &&
              nvs_set_u8(h, "srvrc", s->server_rc) == ESP_OK &&
              nvs_set_u8(h, "bletx", s->ble_tx) == ESP_OK &&
              nvs_set_u8(h, "tcard", s->test_card) == ESP_OK &&
              nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}
