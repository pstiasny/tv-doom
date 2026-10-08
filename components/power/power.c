#include "power.h"

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_random.h"
#include "driver/temperature_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_attr.h"
#include "esp_ota_ops.h"
#include "esp_sleep.h"
#include "soc/usb_serial_jtag_struct.h"


/* Flat-battery SHUTDOWN disarmed 2026-09-15 (user directive): with ext0 wake
 * broken by R18 and the board's USB data lines gone, an armed build sleeps
 * into a state only the jig can recover. Detection + logging keep running;
 * re-arm only with a working unattended wake path. The aerial-open SLEEP is
 * a runtime switch since 2026-09-15 (power_set_line_sleep, from the loader's
 * SETTINGS menu, default off) so it can be tried without a rebuild. */
#define POWER_ACTIONS_ENABLED 0

/* BENCH switch: take external power (USB host / charger at CV) out of the
 * sleep/shutdown decisions, so the board sleeps on an open line even while
 * powered from the bench. Detection itself keeps running and is still logged
 * -- only the action gating ignores it. WARNING with this on (plus actions
 * armed) and no TV attached the board deep-sleeps ~POWER_LINE_OPEN_TICKS s
 * after boot and the USB console goes away with it -- the debug connection is
 * LOST until wake (ground LINE_SENSE/aerial pin, or BOOT-hold for download
 * mode). Rolled back 2026-09-13 for exactly that reason; enable both only for
 * a deliberate sleep-current measurement session. */
#define POWER_IGNORE_EXT 0

/* Bench diagnostics. Both gate the 48 MHz carrier, which puts a visible glitch
 * in the picture, so they stay off in normal operation. LINE_RF_PROBE adds the
 * carrier-on/off rectification test to each probe; LINE_SWEEP_DIAG additionally
 * logs the full decay curve once a second. */
#define LINE_RF_PROBE    0
#define LINE_SWEEP_DIAG  0
#define LINE_MEAN_DIAG   0

/* Bench switch: leave LINE_SENSE (GPIO9) fully high-Z -- no pull-up,
 * no pull-down, no charge pulses, no ADC clamp reads -- so scope measurements
 * of the aerial/RF node see only the passive network (R18/C4/D1). The monitor
 * keeps logging batt/ext but reports the line as "probe off". Set back to 0
 * before trusting any line-detection or deep-sleep behaviour: with this on,
 * the probe always reports unterminated and ext0 wake would never fire. */
#define LINE_PROBE_OFF   0

#define LINE_GPIO   GPIO_NUM_9
/* PWR_PWM (GPIO13 -> J1.5 -> flex -> J6.5 -> RT6150B PS pin): the buck-boost's
 * mode select. High = forced PWM (constant frequency, lowest output ripple --
 * what the video DAC and RF chain want while running). Low or floating = PFM
 * auto power-save (R17 100k pulls PS to ground on the power board), the right
 * mode for the deep-sleep microamp load. Nothing drove this pin before
 * 2026-09-13, so the converter had been in PFM even during operation. In deep
 * sleep the digital pad releases and R17 takes over, so PFM holds through
 * sleep with no RTC hold needed -- same self-securing pattern as R35 on the
 * mixer P-FET. */
#define PWM_GPIO    GPIO_NUM_13
#define BATT_CHAN   ADC_CHANNEL_9    /* ADC1_CH9 == GPIO10 */
#define LINE_CHAN   ADC_CHANNEL_8    /* ADC1_CH8 == GPIO9  */

/* Hold the line at the rail long enough for it to actually get there. Charging
 * goes through R18 (100k) into whatever hangs off the centre conductor: a bare
 * pin is ~10 pF (1 us), a metre of coax ~100 pF (10 us), a tuner's DC blocking
 * capacitor could be nanofarads (hundreds of us). 2 ms covers all of them. */
#define LINE_CHARGE_US  2000

/* How long to let the node sit after release before sampling. Rectification
 * pumps it down fast (a microamp into ~10 pF moves 100 mV/us), so this is set
 * by the ADC conversion time rather than the physics. */
#define LINE_HOLD_US     200

/* Clamped-mean threshold, in mV at the pin, with the carrier RUNNING and the
 * internal pull-up holding the bias. Measured on this board:
 *
 *   open line   ~400 mV   swing clips hard on the pad clamp, mean dragged down
 *   bias level ~2040 mV   what the pull-up alone holds (carrier gated off)
 *
 * A terminated input should sit at the bias, because the amplitude at LINE
 * collapses ~36 dB across R2/75R and never reaches a clamp. Threshold is put
 * midway, leaving ~800 mV of margin either side.
 *
 * PROVISIONAL for the terminated side: the open number is measured, the
 * terminated number is predicted and still needs a set (or a 75R terminator)
 * on the aerial to confirm. */
#define LINE_CLAMP_THRESHOLD_MV 1200

/* The battery divider is R36/R37 = 470k/470k, so the pin sees exactly half the
 * cell. Both legs are 1% parts; the ratio error is what matters, not the
 * absolute value. */
#define BATT_DIVIDER_NUM  2
#define BATT_DIVIDER_DEN  1

/* Whole-chain trim, applied on top of the divider ratio: parts per 1000.
 * 1000 = no correction. Worst-case stack-up on the raw path is around +/-150 mV
 * at 4.2 V (up to 2% of divider ratio error from two 1% legs, plus the ADC's
 * own calibration error), which is too coarse to tell the charger's 4.20 V CV
 * target from a rested cell. Measure +BATT at J7 with a bench meter, compare
 * with the reported reading and set this to 1000 * actual / reported. */
#define BATT_TRIM_PER_MILLE 1000

/* Settling time for the line probe. R18 (100k) working into the internal pull
 * (~45k) and whatever capacitance hangs off the centre conductor: a metre of
 * coax is ~100 pF, so tau is well under 20 us. 500 us is generous. */
#define LINE_SETTLE_US 500

static const char *TAG = "power";

/* Aerial-open sleep arm (see power_set_line_sleep in power.h). */
static volatile bool s_line_sleep;

void power_set_line_sleep(bool enabled)
{
    s_line_sleep = enabled;
    ESP_LOGW(TAG, "line-open deep sleep %s", enabled ? "ARMED" : "disarmed");
}

/* Carrier gate, injected by the application. The RF probe needs to toggle the
 * 48 MHz carrier, but that lives in the app (clkout.c) and this component must
 * not depend upward on it -- main already REQUIRES power. If no gate is
 * registered the RF probe is skipped and only the DC test runs. */
static void (*s_carrier_gate)(bool);

/* Mixer rail gate (Q4's MIXER_3V3), injected like the carrier gate and for
 * the same layering reason: the pin lives in the app (clkout.c). Used only on
 * the way into deep sleep; if unregistered (early-boot flat-battery path) the
 * rail is off anyway -- R35 holds Q4's gate up whenever nothing drives it. */
static void (*s_mixer_gate)(bool);

/* Rail states for the two regimes the board runs in (see PWM_GPIO above):
 * regular operation = converter forced-PWM + mixer rail on (set at init/boot);
 * sleep = converter PFM + mixer off, both held through deep sleep passively. */
static void power_rails_for_sleep(void)
{
    gpio_set_level(PWM_GPIO, 0);            /* PFM now; R17 keeps it in sleep */
    if (s_mixer_gate) s_mixer_gate(false);  /* rail off; R35 keeps it in sleep */

    /* Sleep is transparent: wakes must resume the GAME directly, not the
     * management menu (menu-first applies to cold boots only -- see
     * ota_mark_healthy). Conditional write: repeated sleep/recheck cycles on
     * a shelf must not wear otadata. */
    const esp_partition_t *game = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (game && esp_ota_get_boot_partition() != game)
        esp_ota_set_boot_partition(game);
}

/* Where the monitor publishes each reading. Kept as a callback for the same
 * reason as the carrier gate: the on-screen readout lives in the Doom platform
 * layer, and this component must not depend upward on it. */
static void (*s_report_cb)(int batt_mv, bool ext, bool terminated);

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali_batt, s_cali_line;
static bool s_ready;

static int read_mv(adc_channel_t ch, adc_cali_handle_t cali)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc, ch, &raw) != ESP_OK) return -1;
    int mv = 0;
    if (cali && adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK) return mv;
    /* No calibration scheme: fall back to the nominal 12 dB full scale. */
    return (int)((int64_t)raw * 3100 / 4095);
}

/* Averaged read for the high-impedance battery divider. The 235k source
 * impedance (R36||R37) cannot refill the ADC's sample-and-hold capacitor
 * between back-to-back conversions, so the first few reads sit low and only
 * settle once C28 (100n) has topped the node back up. Discard, then average. */
static int read_mv_avg(adc_channel_t ch, adc_cali_handle_t cali)
{
    for (int i = 0; i < 4; i++) (void)read_mv(ch, cali);
    int64_t acc = 0;
    int n = 0;
    for (int i = 0; i < 16; i++) {
        int mv = read_mv(ch, cali);
        if (mv >= 0) { acc += mv; n++; }
    }
    return n ? (int)(acc / n) : -1;
}

/* LINE_SENSE is a purely DIGITAL input here -- see power_line_probe(). It must
 * not be handed to the ADC driver: adc_oneshot_config_channel() puts the pad
 * into analog mode and takes the RTC pull resistors with it, after which the
 * pulls this probe depends on have no effect on the pin (measured: pull-up,
 * pull-down and driven-high all read the same ~0.5 V). */
static void line_set_pull(gpio_pull_mode_t mode)
{
    gpio_set_pull_mode(LINE_GPIO, mode);
    esp_rom_delay_us(LINE_SETTLE_US);
}

void power_set_carrier_gate(void (*gate)(bool))
{
    s_carrier_gate = gate;
}

void power_set_mixer_gate(void (*gate)(bool))
{
    s_mixer_gate = gate;
}

void power_set_report_cb(void (*cb)(int, bool, bool))
{
    s_report_cb = cb;
}

void power_init(void)
{
    if (s_ready) return;

    gpio_config_t line = {
        .pin_bit_mask = 1ULL << LINE_GPIO,
        .mode         = GPIO_MODE_INPUT,
        /* LINE_PROBE_OFF: park the pin high-Z so the bench sees only the
         * passive network on the aerial node. */
        .pull_up_en   = LINE_PROBE_OFF ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&line));

    /* Regular operation: force the RT6150B into PWM mode for minimum supply
     * ripple under the video/RF load. Runs from power_early_gate(), so the
     * converter is in the right mode before any of the heavy rails come up. */
    gpio_config_t pwm = {
        .pin_bit_mask = 1ULL << PWM_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&pwm));
    gpio_set_level(PWM_GPIO, 1);

    adc_oneshot_unit_init_cfg_t unit = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &s_adc));

    /* 12 dB attenuation: ~3.1 V full scale. The battery divider spans
     * 1.50-2.10 V over a 3.0-4.2 V cell, so nothing narrower will do. */
    adc_oneshot_chan_cfg_t ch = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, BATT_CHAN, &ch));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = BATT_CHAN,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cfg, &s_cali_batt) != ESP_OK)
        s_cali_batt = NULL;
    cfg.chan = LINE_CHAN;
    if (adc_cali_create_scheme_curve_fitting(&cfg, &s_cali_line) != ESP_OK)
        s_cali_line = NULL;
#endif

    s_ready = true;
    ESP_LOGI(TAG, "up: line=GPIO9 (digital), batt=GPIO10/ADC1_CH9, "
             "converter=PWM (GPIO13 high), cali %s",
             s_cali_batt ? "curve-fitting" : "NONE (raw scaling)");
}

int power_battery_mv(void)
{
    if (!s_ready) return -1;
    int pin = read_mv_avg(BATT_CHAN, s_cali_batt);
    if (pin < 0) return -1;
    int mv = pin * BATT_DIVIDER_NUM / BATT_DIVIDER_DEN;
    return (int)((int64_t)mv * BATT_TRIM_PER_MILLE / 1000);
}

bool power_battery_connected(int mv)
{
    /* The sense line runs across the flex (J6 -> J9/J3 -> J1), so a broken
     * conductor leaves R37 (470k) pulling the node to ground and we read a
     * few tens of millivolts. That is NOT a flat cell: the board is executing
     * this code, so something is powering it, and a real pack under any load
     * at all cannot be down here (the protection IC cuts off around 2.5 V and
     * the supply would have collapsed long before). Treat it as "no reading"
     * and never act on it. */
    return mv >= POWER_BATT_MIN_PLAUSIBLE_MV;
}

/*
 * Aerial load detection.
 *
 * The obvious approach -- measure the RF amplitude on LINE_SENSE -- does not
 * work on this hardware, for two independent reasons:
 *
 *   1. R18 (100k) works into the pin's own capacitance (~5-10 pF including
 *      package, ESD and track), which is a low-pass at roughly 200-300 kHz.
 *      At the 48 MHz carrier that is ~45 dB of attenuation ON TOP of whatever
 *      is on LINE, so even the open-circuit case arrives at the pin as a few
 *      millivolts.
 *   2. ADC1 tops out around 83 kSPS, so the carrier could not be sampled even
 *      if it survived, and nothing in the path rectifies it -- D1 is a 3.3 V
 *      standoff ESD diode that never conducts at these levels, and a purely
 *      linear RC network cannot recover the AM envelope.
 *
 * What the hardware IS well suited to is a DC probe, and that turns out to be
 * a far stronger signal. LINE floats at DC: C4 (100p) blocks the modulator,
 * D1 only leaks, so the node's only DC connections are R18 to this pin and
 * whatever is on the centre conductor. A terrestrial tuner input presents a DC
 * path to ground through its input balun -- typically a few ohms to a few
 * hundred. So with the internal pull-up (~45k) enabled:
 *
 *   open      pin sits at 3.3 V   (nothing to discharge it)
 *   75 R load pin sits at ~0 V    (75/(45k+100k+75) of the rail)
 *
 * That is rail-to-ground rather than the tens of millivolts an RF measurement
 * would have given, so a plain digital read resolves it with enormous margin
 * and no calibration -- and it is exactly the signal an ext0 deep-sleep wake
 * trigger consumes, which is what makes the sleep design below so cheap.
 * R18 keeps the probe invisible to the RF path (100k against 75 R) and limits
 * what we put on the centre conductor to ~33 uA.
 *
 * Reading the pin under BOTH pulls separates the three cases cleanly:
 *
 *   pull-up 1, pull-down 0   nothing attached; the pin just follows the pull
 *   pull-up 0, pull-down 0   held down externally  -> terminated
 *   pull-up 1, pull-down 1   held up externally    -> fault (short to 3V3)
 *
 * CAVEAT: this assumes the TV's aerial socket is DC-coupled to ground. Most
 * terrestrial inputs are, but some have a series blocking capacitor. Both
 * raw levels are reported so this can be checked against a real set before
 * the verdict is trusted.
 */
static int line_mean(bool pullup, bool carrier_on);          /* defined below */
static int line_clamped_median(int *out_min, int *out_max);  /* defined below */

static void line_pad_digital(void)
{
    rtc_gpio_deinit(LINE_GPIO);
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LINE_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

static void line_pad_analog(void)
{
    adc_oneshot_chan_cfg_t ch = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(s_adc, LINE_CHAN, &ch);
}

/* Charge LINE to the rail through the pin, release it, and read what is left a
 * moment later. This is the "short pulse" probe: it says nothing about DC
 * coupling by itself, but combined with the carrier toggle below it measures
 * the RF amplitude at the node. */
static int line_charge_and_read(int hold_us)
{
    line_pad_digital();
    gpio_set_direction(LINE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LINE_GPIO, 1);
    esp_rom_delay_us(LINE_CHARGE_US);

    gpio_set_direction(LINE_GPIO, GPIO_MODE_INPUT);   /* release to high-Z */
    line_pad_analog();
    esp_rom_delay_us(hold_us);
    return read_mv(LINE_CHAN, s_cali_line);
}

static int line_level_with_pull(gpio_pull_mode_t mode)
{
    line_pad_digital();
    gpio_set_pull_mode(LINE_GPIO, mode);
    esp_rom_delay_us(LINE_SETTLE_US);
    return gpio_get_level(LINE_GPIO);
}

void power_line_probe(power_line_t *out)
{
    out->pullup_level = out->pulldown_level = -1;
    out->carrier_off_mv = out->carrier_on_mv = out->rectify_mv = -1;
    out->clamp_min_mv = out->clamp_max_mv = -1;
    out->terminated = false;
    if (!s_ready) return;

#if LINE_PROBE_OFF
    return;     /* bench: pin stays high-Z, node untouched (see top of file) */
#endif

    /* --- DC probe: is something holding the centre conductor down? --------
     * Conclusive when it fires, but it only sees a DC-COUPLED input. A tuner
     * with a series blocking capacitor looks identical to an open line here,
     * which is why it is not the primary test. */
    out->pullup_level   = line_level_with_pull(GPIO_PULLUP_ONLY);
    out->pulldown_level = line_level_with_pull(GPIO_PULLDOWN_ONLY);
    bool dc_loaded = (out->pullup_level == 0) && (out->pulldown_level == 0);

    /* --- RF probe: is the carrier being absorbed? ------------------------
     * This is the test that does not care about DC coupling at all, because a
     * blocking capacitor is a short at 48 MHz -- a terminated input loads the
     * carrier either way.
     *
     * LINE floats at DC (C4 blocks the modulator), so after charging it to the
     * rail the only things that can move it are leakage and rectification. Sit
     * it at 3.3 V and the carrier's positive peaks reach 3.3 + Vamp; unloaded,
     * that is enough to push D1 (ESD9B3.3ST5G, 3.3 V standoff) into
     * conduction, and every peak pumps a little charge off the node. The mean
     * settles where the peaks just barely reach D1's knee, i.e. roughly
     * Vknee - Vamp -- so the DROP is a direct measure of carrier amplitude.
     *
     * Terminated, the amplitude at LINE collapses by R2/(R2+75) = ~36 dB, the
     * peaks never reach D1, nothing rectifies, and the node stays at the rail.
     *
     * Measuring it as a difference against the carrier-off reading makes it
     * self-referencing: pad leakage, ADC offset and the exact clamp voltage
     * all cancel. */
    /* Primary test: the clamped mean, with the carrier left running. No
     * gating, so no glitch in the picture, and one reading is enough -- the
     * carrier-off reference only matters for diagnostics. */
    out->carrier_on_mv = line_clamped_median(&out->clamp_min_mv, &out->clamp_max_mv);

#if LINE_RF_PROBE
    out->carrier_off_mv = line_mean(true, false);
    if (out->carrier_off_mv >= 0 && out->carrier_on_mv >= 0)
        out->rectify_mv = out->carrier_off_mv - out->carrier_on_mv;
    if (s_carrier_gate) s_carrier_gate(true);       /* restore */
#endif

    /* Leave the pin parked with the pull-up: the resting state, and the one
     * deep sleep wakes from. */
    line_level_with_pull(GPIO_PULLUP_ONLY);

    /* Either test is sufficient. The clamp test is the primary one and works
     * regardless of how the tuner's input is coupled; the DC test only fires
     * on a DC-coupled input but is conclusive when it does. */
    out->terminated = dc_loaded ||
        (out->carrier_on_mv >= 0 && out->carrier_on_mv > LINE_CLAMP_THRESHOLD_MV);
}

bool power_line_terminated(void)
{
    power_line_t l;
    power_line_probe(&l);
    return l.terminated;
}

bool power_usb_host_present(void)
{
    /* A USB host emits a start-of-frame every 1 ms and the peripheral latches
     * the frame index; a bare charger (or nothing) never advances it. */
    uint32_t a = USB_SERIAL_JTAG.fram_num.sof_frame_index;
    vTaskDelay(pdMS_TO_TICKS(20));
    uint32_t b = USB_SERIAL_JTAG.fram_num.sof_frame_index;
    return a != b;
}

bool power_external_power_present(void)
{
    /* Cell held at the charger's CV target => VBUS is present. This is the
     * branch that catches a plain (non-data) USB charger, which the SOF check
     * below cannot see at all. Skipped entirely when the sense line is dead,
     * otherwise a broken flex would read as "no external power". */
    int batt = power_battery_mv();
    if (power_battery_connected(batt) && batt >= POWER_EXT_PWR_MV) return true;

    /* Otherwise fall back to the host check, which still gets us the answer
     * immediately when a computer is plugged into a flat pack -- exactly the
     * case the voltage test misses during constant-current charging. */
    return power_usb_host_present();
}

/* Re-check interval while parked on a flat battery. There is no VBUS sense
 * GPIO, so a sleeping board cannot be woken by plugging in a charger -- it has
 * to wake on its own and look. A minute is frequent enough to feel responsive
 * and rare enough to be irrelevant to the leakage budget. */
#define POWER_FLAT_RECHECK_US (60ULL * 1000000ULL)

/* Re-check interval while asleep on an open line. Needed for the same
 * no-VBUS-sense reason as above, and doubly so on the assembled unit: BOOT is
 * not reachable through the enclosure, and the ext0 aerial wake cannot fire
 * with R18 = 100k (the pin only divides down to ~2.3 V against the ~45k
 * internal pull-up -- still logic high; R18 -> 10k fixes that, see the wake
 * recheck below for the interim). Each recheck is a partial boot costing
 * ~200-300 ms at tens of mA, so the period sets the standby current:
 * ~10 s =~ 1 mA average. After the R18 rework the ext0 path wakes instantly
 * and this timer only matters for USB detection -- it can then be lengthened
 * substantially. */
#define POWER_LINE_RECHECK_US (10ULL * 1000000ULL)

/* Persists across deep sleep: tells the next boot it woke from line-open
 * sleep and must run power_line_recheck() before the expensive bring-up. */
static RTC_DATA_ATTR uint8_t s_slept_on_line;

/* Wake-path breadcrumbs (RTC RAM): entries counted at the TOP of
 * power_line_recheck, completions at the BOTTOM. A wake that crashes
 * mid-path bumps n but not done -> the next full boot surfaces n>done over
 * BLE (blelog), no UART needed. */
RTC_NOINIT_ATTR uint32_t power_wake_n, power_wake_done;

/* Recheck breadcrumbs, kept across sleeps in RTC RAM: the last few clamp
 * means, stored positive when the verdict was TERMINATED and negative when
 * "open". Printed at the next full boot -- the recheck itself runs before
 * the console's USB device has enumerated, so its live logs are unreadable. */
#define RC_HIST 8
static RTC_DATA_ATTR int16_t s_rc_hist[RC_HIST];
static RTC_DATA_ATTR uint8_t s_rc_n;

void power_sleep_until_line_connected(void)
{
    ESP_LOGW(TAG, "line open -- entering deep sleep, wake on aerial connect "
                  "or %llu s recheck", POWER_LINE_RECHECK_US / 1000000ULL);
    power_rails_for_sleep();
    s_slept_on_line = 1;
    esp_sleep_enable_timer_wakeup(POWER_LINE_RECHECK_US);

    /* The whole reason the DC probe is worth doing: the "terminated" condition
     * is a static logic level, so the RTC can wait on it directly. No ULP
     * program and no periodic timer wake are needed -- ext0 sits on GPIO9 with
     * the pull-up held through sleep and wakes the chip the instant a set is
     * plugged in. That is both lower power than polling and instant, rather
     * than up to a poll interval late. */
    rtc_gpio_init(LINE_GPIO);
    rtc_gpio_set_direction(LINE_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(LINE_GPIO);
    rtc_gpio_pullup_en(LINE_GPIO);

    /* ext0 needs the RTC peripheral domain left powered, otherwise the pull-up
     * dies with it and the pin floats. */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(LINE_GPIO, 0));  /* wake on LOW */

    esp_deep_sleep_start();
}

void power_shutdown_flat_battery(void)
{
    ESP_LOGE(TAG, "battery flat -- shutting down, re-checking every %llu s",
             POWER_FLAT_RECHECK_US / 1000000ULL);
    power_rails_for_sleep();
    s_slept_on_line = 0;   /* flat wake re-checks the battery, not the line */

    /* Wake on the aerial too: if the user plugs into a TV we want to come up
     * and at least say why we are not playing, rather than look dead. */
    rtc_gpio_init(LINE_GPIO);
    rtc_gpio_set_direction(LINE_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(LINE_GPIO);
    rtc_gpio_pullup_en(LINE_GPIO);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_sleep_enable_ext0_wakeup(LINE_GPIO, 0);
    esp_sleep_enable_timer_wakeup(POWER_FLAT_RECHECK_US);

    esp_deep_sleep_start();
}

bool power_line_recheck_pending(void)
{
    return s_slept_on_line && esp_sleep_get_wakeup_causes() != 0;
}

void power_line_recheck(void)
{
    /* Wake-from-line-sleep triage, run by main BEFORE the expensive bring-up
     * (BLE, video, sound), with only the carrier + mixer rails up. At this
     * point the carrier is UNMODULATED CW -- the video engine has not started
     * and the DAC node rests at full drive -- so the clamped mean carries no
     * video envelope and a single biased reading is as trustworthy as the
     * monitor's 16-sample median: open line -> swing clips, mean well below
     * threshold; terminated -> no swing, mean at bias. If neither a load nor
     * external power appeared, this re-enters deep sleep a few hundred ms
     * after wake; otherwise it returns and the normal boot continues. */
    s_slept_on_line = 0;
    power_wake_n++;                 /* breadcrumb: entered the wake path */

    /* The sleep path put GPIO9 under RTC control (rtc_gpio_init + held
     * pull-up for ext0). That mux SURVIVES the wake: left in place, every
     * later probe reads the held bias (~3.0 V) instead of the line -- a
     * permanent false-TERMINATED (observed 2026-09-15: wake rechecks read
     * ~3000 mV on an open bench line). Return the pad to the digital/analog
     * matrix before any probing. */
    rtc_gpio_deinit(LINE_GPIO);
    rtc_gpio_pullup_dis(LINE_GPIO);
    rtc_gpio_pulldown_dis(LINE_GPIO);

    /* ext0 means the LINE pin itself was pulled low -- an unambiguous
     * external event (a terminated line once R18 is 10k, or a deliberate
     * bench rescue pull). Trust it and boot fully; the monitor's hysteresis
     * re-verifies within seconds and re-sleeps if it was a glitch. Running
     * the clamp verdict here instead was a bug: a pin HELD low reads ~0 mV,
     * far below the threshold, so the rescue pull got classified "open" and
     * the board went straight back to sleep. */
    if (esp_sleep_get_wakeup_causes() & BIT(ESP_SLEEP_WAKEUP_EXT0)) {
        ESP_LOGW(TAG, "wake recheck: ext0 (line pulled low) -- booting");
        return;
    }

    /* Timer wake: look for a USB host FIRST, and give it time. The USJ
     * re-attaches from scratch on every wake, and the host's SOF stream
     * (what power_usb_host_present() detects) only starts a few hundred ms
     * after attach -- a single 20 ms sample right after boot ALWAYS missed
     * it, so a sleeping board could never be woken by plugging in USB. Poll
     * up to ~800 ms with early exit; the no-USB cost is what sets the
     * standby draw together with POWER_LINE_RECHECK_US. */
    bool ext = power_external_power_present();
    for (int i = 0; !ext && i < 6; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ext = power_usb_host_present();
    }
    int mean = line_mean(true, true);
    bool term = mean >= LINE_CLAMP_THRESHOLD_MV;
    ESP_LOGI(TAG, "wake recheck: line mean %d mV -> %s, ext %s",
             mean, term ? "TERMINATED" : "open", ext ? "YES" : "no");

    /* Breadcrumbs for the next full boot: recheck logs are unreadable live
     * (the console's USB device isn't enumerated that early), so keep the
     * last few verdicts across sleeps in RTC RAM. */
    s_rc_hist[s_rc_n++ % RC_HIST] = (int16_t)(term ? mean : -mean);

    power_wake_done++;              /* breadcrumb: wake path completed clean */

    if (!term && !ext) {
        if (s_line_sleep)
            power_sleep_until_line_connected();      /* does not return */
        ESP_LOGW(TAG, "wake recheck: line open, but line sleep is OFF -- booting");
    }
}

bool power_early_gate(void)
{
    /* Called before the expensive bring-up (BLE, video, sound). If the pack is
     * genuinely flat, go straight back to sleep rather than browning out
     * halfway through starting the radio. A disconnected sense line is NOT
     * flat -- see power_battery_connected(). */
    power_init();
    int batt = power_battery_mv();
    if (!power_battery_connected(batt)) {
        ESP_LOGW(TAG, "battery sense reads %d mV -- treating as unknown, continuing", batt);
        return true;
    }
    if (batt < POWER_BATT_SHUTDOWN_MV && !power_external_power_present()) {
#if POWER_ACTIONS_ENABLED
        power_shutdown_flat_battery();
#else
        ESP_LOGE(TAG, "battery %d mV below cutoff (actions disarmed)", batt);
#endif
        return false;
    }
    return true;
}

/* Consecutive one-second ticks with the aerial open before we sleep. A few
 * seconds of hysteresis stops a wobbly connector from putting the board to
 * sleep mid-game. */
/* Consecutive one-second ticks that must agree before we act. Each tick is
 * already a median of 16 phase-randomized samples, so this is a second,
 * coarser layer of agreement guarding against slow effects the median cannot
 * see: a wobbly connector, a scene that stays black long enough to shift the
 * envelope, a cell sagging under a transient load.
 *
 * Both are deliberately ASYMMETRIC -- the counters only ever count toward the
 * disruptive action and are reset instantly by any contrary sample. Going to
 * sleep or shutting down wrongly costs the user a session; staying awake a few
 * seconds too long costs almost nothing. The wake path needs no such
 * hysteresis at all: ext0 is a hardware level trigger and is instant.
 *
 * The battery counter is the longer of the two because a shutdown is the more
 * annoying of the two to recover from, and because a sagging reading during a
 * heavy video/BLE moment is exactly the false positive we must not act on. */
#define POWER_LINE_OPEN_TICKS 5
#define POWER_BATT_LOW_TICKS  10
static int s_open_ticks;
static int s_batt_low_ticks;

/* Runtime inhibit for the sleep/shutdown actions (monitoring continues).
 * Used by OTA mode: nothing may power the board down mid-update. */
static volatile bool s_actions_inhibited;

void power_inhibit_actions(bool inhibit)
{
    s_actions_inhibited = inhibit;
    s_open_ticks = 0;
    s_batt_low_ticks = 0;
}

/* Bias the pin to a rail and read the resulting MEAN, with the carrier gated
 * off and on.
 *
 * This is the measurement that matters, and it is not a peak-detector effect:
 * the pad's protection diodes CLAMP the waveform to roughly [-0.4, +3.7] V, so
 * once the AC swing at the pin is large enough to reach a clamp the waveform is
 * clipped on that side and its DC MEAN moves away from the bias. R18 working
 * into the pin capacitance averages the 48 MHz down to that mean, which is
 * exactly what the ADC reads.
 *
 * Held at the 3.3 V rail by the pull-up, any appreciable swing clips on top and
 * drags the mean DOWN. Terminated, the amplitude at LINE collapses ~36 dB
 * across R2/75R, nothing reaches a clamp, and the mean stays at the bias.
 *
 * Reading both pull directions also self-tests the probe: on a floating line
 * pull-up must read high and pull-down low. If they agree, the pulls are not
 * reaching the pad and every other number here is meaningless. */
static int line_mean(bool pullup, bool carrier_on)
{
    if (s_carrier_gate) s_carrier_gate(carrier_on);

    line_pad_analog();                       /* pad -> analog, pulls cleared */
    rtc_gpio_pullup_dis(LINE_GPIO);
    rtc_gpio_pulldown_dis(LINE_GPIO);
    if (pullup) rtc_gpio_pullup_en(LINE_GPIO);
    else        rtc_gpio_pulldown_en(LINE_GPIO);

    esp_rom_delay_us(5000);                  /* settle: 145k into the node's C */
    return read_mv_avg(LINE_CHAN, s_cali_line);
}

/* Median of N clamped-mean samples, spread across several video fields.
 *
 * The carrier is AM-modulated by the picture, so the clamped mean at the pin
 * carries the whole video envelope -- strong structure at the 15.625 kHz line
 * rate and the 50 Hz field rate, all of it well inside the ~227 kHz corner R18
 * forms with the pin capacitance. (The 48 MHz carrier itself is NOT the
 * problem: the ~31k source impedance at the pin working into the ADC's
 * sample-and-hold capacitor rolls it off long before the sampler sees it.)
 *
 * A single reading is a ~500 us burst -- about 1/40th of a field -- so it picks
 * up whatever the picture happened to be doing in that sliver: bright, dark, or
 * vertical blanking where the carrier sits at sync level throughout. That is
 * what scatters single readings across 20..120 mV.
 *
 * Spacing must be RANDOMIZED, not merely spread out. With a 100 Hz tick every
 * whole-tick interval is a multiple of 10 ms, and against a 20 ms field that
 * lands on only one or two phases forever -- textbook aliasing. Jittering each
 * gap by a few milliseconds makes the phase random-walk across the field within
 * a handful of samples.
 *
 * Median rather than mean: it discards the VBI and scene-cut outliers outright
 * instead of letting them drag the result. */
#define LINE_SAMPLES 16

static int cmp_int(const void *a, const void *b)
{
    return *(const int *)a - *(const int *)b;
}

static int line_clamped_median(int *out_min, int *out_max)
{
    int v[LINE_SAMPLES];
    int n = 0;

    /* Establish the bias ONCE. The pull-up working into R18 and the node's
     * capacitance needs a few milliseconds to settle, but only on the first
     * sample -- re-settling per sample was costing 16 x 5 ms of busy-wait at a
     * priority above the game, which took ~14 fps off Doom for no benefit. */
    if (s_carrier_gate) s_carrier_gate(true);
    line_pad_analog();
    rtc_gpio_pulldown_dis(LINE_GPIO);
    rtc_gpio_pullup_en(LINE_GPIO);
    esp_rom_delay_us(5000);

    for (int i = 0; i < LINE_SAMPLES; i++) {
        if (i) {
            vTaskDelay(1);                              /* yield ~10 ms */
            esp_rom_delay_us(esp_random() % 3000);      /* sub-tick phase jitter */
        }
        /* ONE conversion, not an average: we are deliberately sampling the
         * envelope at random phases, so averaging inside a burst would just
         * smear one phase rather than cover several. */
        int mv = read_mv(LINE_CHAN, s_cali_line);
        if (mv >= 0) v[n++] = mv;
    }
    if (!n) return -1;

    qsort(v, n, sizeof(v[0]), cmp_int);
    if (out_min) *out_min = v[0];
    if (out_max) *out_max = v[n - 1];
    return v[n / 2];
}

static void line_mean_diag(void)
{
    int pu_off = line_mean(true,  false);
    int pu_on  = line_mean(true,  true);
    int pd_off = line_mean(false, false);
    int pd_on  = line_mean(false, true);

    ESP_LOGI(TAG, "mean pu off=%4d on=%4d (d=%4d) | pd off=%4d on=%4d (d=%4d) | pulls %s",
             pu_off, pu_on, pu_off - pu_on, pd_off, pd_on, pd_off - pd_on,
             (pu_off - pd_off) > 500 ? "OK" : "NOT REACHING PAD");
}

__attribute__((unused)) static void line_sweep(void)
{
    static const int holds_us[] = { 200, 2000, 20000, 100000 };
    int off[4], on[4];

    for (int i = 0; i < 4; i++) {
        if (s_carrier_gate) s_carrier_gate(false);
        esp_rom_delay_us(200);
        off[i] = line_charge_and_read(holds_us[i]);
        if (s_carrier_gate) s_carrier_gate(true);
        esp_rom_delay_us(200);
        on[i]  = line_charge_and_read(holds_us[i]);
    }
    ESP_LOGI(TAG, "sweep off %4d %4d %4d %4d | on %4d %4d %4d %4d | d %4d %4d %4d %4d",
             off[0], off[1], off[2], off[3], on[0], on[1], on[2], on[3],
             off[0]-on[0], off[1]-on[1], off[2]-on[2], off[3]-on[3]);
}

/* ---------- die temperature -------------------------------------------- *
 * The RF modulator is two bare transistors with no bias servo, so its
 * DAC-code to RF-envelope transfer drifts with temperature. Nothing else on
 * this board measures temperature, so the SoC's own sensor is the reference
 * every capture and every calibration is tagged with.
 *
 * It is the DIE temperature, several degrees above the board and not the same
 * as the modulator's junction. It is a reproducible proxy and a drift
 * indicator, not a transistor temperature. */
static temperature_sensor_handle_t s_tsens;
static float s_temp_c = -273.0f, s_temp_min = 1e3f, s_temp_max = -1e3f;

float power_temp_c(void)     { return s_temp_c; }
float power_temp_min_c(void) { return s_temp_min; }
float power_temp_max_c(void) { return s_temp_max; }

static void temp_init(void)
{
    /* -10..80 C keeps the best resolution over the range this board lives in;
     * a reading outside it saturates rather than failing. */
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&cfg, &s_tsens) != ESP_OK ||
        temperature_sensor_enable(s_tsens) != ESP_OK) {
        ESP_LOGW(TAG, "die temperature sensor unavailable");
        s_tsens = NULL;
    }
}

static void temp_sample(void)
{
    float c;
    if (!s_tsens || temperature_sensor_get_celsius(s_tsens, &c) != ESP_OK) return;
    s_temp_c = c;
    if (c < s_temp_min) s_temp_min = c;
    if (c > s_temp_max) s_temp_max = c;
}

static void monitor_task(void *arg)
{
    temp_init();
    int temp_tick = 0;
    for (;;) {
#if LINE_MEAN_DIAG
        line_mean_diag();
#endif
#if LINE_SWEEP_DIAG
        line_sweep();
#endif
        temp_sample();
        /* Every 10th tick: one line the bench tools parse into each capture,
         * so no measurement is ever compared across an unrecorded thermal gap. */
        if (s_tsens && ++temp_tick >= 10) {
            temp_tick = 0;
            ESP_LOGI(TAG, "temp: die %.1f C (min %.1f max %.1f)",
                     s_temp_c, s_temp_min, s_temp_max);
        }

        power_line_t l;
        power_line_probe(&l);
        int batt = power_battery_mv();
        bool conn = power_battery_connected(batt);
        bool host = power_usb_host_present();
        bool ext  = (conn && batt >= POWER_EXT_PWR_MV) || host;

        /* Publish -1 rather than a bogus number when the sense line is open,
         * so the readout hides instead of showing a wrong voltage. */
        if (s_report_cb) s_report_cb(conn ? batt : -1, ext, l.terminated);

#if LINE_PROBE_OFF
        ESP_LOGI(TAG, "batt %d mV%s | line PROBE OFF (GPIO9 high-Z, bench) | ext %s (host %s)",
                 batt, conn ? "" : " (SENSE DISCONNECTED)",
                 ext ? "YES" : "no", host ? "yes" : "no");
#else
        ESP_LOGI(TAG, "batt %d mV%s | line pu=%d pd=%d clamp %d/%d/%d -> %s | ext %s (host %s)",
                 batt, conn ? "" : " (SENSE DISCONNECTED)",
                 l.pullup_level, l.pulldown_level,
                 l.clamp_min_mv, l.carrier_on_mv, l.clamp_max_mv,
                 l.terminated ? "TERMINATED" : "open",
                 ext ? "YES" : "no", host ? "yes" : "no");
#endif

        /* POWER_IGNORE_EXT: pretend nothing external is attached so the sleep
         * paths can be exercised on a bench supply. Real ext is still logged. */
        bool ext_gate = POWER_IGNORE_EXT ? false : ext;
#if POWER_ACTIONS_ENABLED
        if (!s_actions_inhibited) {
            if (conn && batt < POWER_BATT_SHUTDOWN_MV && !ext_gate) {
                if (++s_batt_low_ticks >= POWER_BATT_LOW_TICKS)
                    power_shutdown_flat_battery();
            } else {
                s_batt_low_ticks = 0;
            }
        }
#endif
        /* Aerial-open sleep, armed at runtime (SETTINGS -> line sleep).
         * External power inhibits it: plugged in, the user is most likely
         * developing or charging, and dropping into deep sleep mid-session
         * because no TV happens to be attached would be obnoxious. */
        if (s_line_sleep && !s_actions_inhibited && !l.terminated && !ext_gate) {
            if (++s_open_ticks >= POWER_LINE_OPEN_TICKS)
                power_sleep_until_line_connected();
        } else {
            s_open_ticks = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void power_start_monitor(void)
{
    /* Surface what the pre-console wake rechecks decided while we couldn't
     * watch (positive = TERMINATED, negative = judged open, magnitude = the
     * clamp mean in mV). */
    if (s_rc_n) {
        char buf[8 * RC_HIST + 1]; int off = 0;
        int n = s_rc_n < RC_HIST ? s_rc_n : RC_HIST;
        for (int i = 0; i < n; i++)
            off += snprintf(buf + off, sizeof buf - off, " %d",
                            s_rc_hist[(s_rc_n - n + i) % RC_HIST]);
        ESP_LOGW(TAG, "wake recheck history (%u total, oldest first):%s",
                 s_rc_n, buf);
        s_rc_n = 0;
    }
    /* Priority ABOVE the Doom task (5), on core 0. Doom's loop is producer-
     * limited and never blocks, so anything at or below its priority on this
     * core is starved outright -- the first version of this task at priority 2
     * ran exactly once. Sitting above it is harmless: this wakes once a second
     * for a few milliseconds. Core 1 is left alone deliberately; the video
     * engine there must never be interrupted. */
    xTaskCreatePinnedToCore(monitor_task, "power", 4096, NULL, 6, NULL, 0);
}
