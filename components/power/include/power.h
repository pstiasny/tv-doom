/*
 * Power / environment sensing: battery voltage, aerial-load detection and
 * USB-host presence.
 *
 * Two analog inputs come off the ESP32-S3 (both ADC1, both RTC-capable so they
 * survive deep sleep):
 *
 *   GPIO9  LINE_SENSE    ADC1_CH8   100k (R18) to the aerial centre conductor
 *   GPIO10 BATTERY_SENSE ADC1_CH9   470k/470k divider across the cell (/2)
 *
 * LINE_SENSE cannot see the 48 MHz carrier -- see the long comment on
 * power_line_probe() in power.c -- so load detection is done at DC instead,
 * which the same network handles well.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* External power (USB) is inferred from the cell voltage rather than sensed
 * directly: this board has no VBUS sense GPIO, and the charger's STAT pin
 * drives D2 only. When VBUS is present the MCP73831 holds +BATT at its 4.20 V
 * constant-voltage target, which a cell on discharge can never reach. The
 * threshold sits just under 4.20 V to leave room for divider and ADC error.
 *
 * Known limitation, accepted deliberately: during the charger's constant-
 * CURRENT phase (a flat cell just plugged in) +BATT is still well below 4.2 V,
 * so external power reads as absent until the pack tops up. It also reads as
 * present for a while on a freshly charged cell that has just been unplugged. */
#define POWER_EXT_PWR_MV        4150

/* Cell thresholds (millivolts at the battery, not at the divider). A single
 * Li-ion/LiPo cell: 4.2 V full, ~3.7 V nominal, 3.0 V is the usual protection
 * cutoff. We stop well above the cutoff so the pack's own protection IC never
 * has to act, and because these are open-circuit-ish readings taken while the
 * radio and video DMA are loading the cell (so the true resting voltage is a
 * little higher than what we measure). */
#define POWER_BATT_WARN_MV      3400
#define POWER_BATT_SHUTDOWN_MV  3200

/* Below this, treat the reading as "no reading" rather than as a flat cell.
 * BATTERY_SENSE reaches the ESP across the flex (J6 -> J9/J3 -> J1); a broken
 * conductor leaves R37 pulling the node to ground. Since the CPU is running,
 * the pack cannot actually be at this voltage -- so it means the sense path is
 * open, and acting on it would shut the board down over a wiring fault. */
#define POWER_BATT_MIN_PLAUSIBLE_MV 1000

/* Bring up ADC1 (oneshot + curve-fitted calibration) for both channels.
 * Safe to call once, early; everything below returns defaults until it runs. */
void power_init(void);

/* Battery terminal voltage in mV, or -1 if unavailable. Averaged over several
 * conversions -- the divider's 235k source impedance means single reads are
 * noisy even with C28 (100n) as the reservoir. */
int power_battery_mv(void);

/* False if the reading is implausibly low, which on this board means the
 * BATTERY_SENSE conductor is open rather than the cell being flat. Callers
 * must gate any shutdown decision on this. */
bool power_battery_connected(int mv);

/* Result of one DC probe of the aerial line. Voltages are at the ESP pin, in
 * mV; the LINE node itself sits behind R18 (100k). */
typedef struct {
    int  pullup_level;    /* pin level with the internal pull-up, settled */
    int  pulldown_level;  /* pin level with the internal pull-down, settled */
    int  carrier_off_mv;  /* node voltage after charge+release, carrier gated off */
    int  carrier_on_mv;   /* MEDIAN clamped mean, carrier running -- the verdict */
    int  rectify_mv;      /* off - on, diagnostics only */
    int  clamp_min_mv;    /* spread of the clamped-mean samples: min ... */
    int  clamp_max_mv;    /* ... and max, to show how much envelope got through */
    bool terminated;      /* true = a 75 ohm load is present */
} power_line_t;

/* Register the 48 MHz carrier gate (clkout_set_enabled). Required for the RF
 * half of the line probe; without it only the DC test runs. */
void power_set_carrier_gate(void (*gate)(bool));

/* Register the mixer rail gate (mixer_set_enabled). Called with false on the
 * way into deep sleep so the RF chain powers down with the chip; R35 then
 * holds the rail off passively for the duration of the sleep. Optional -- if
 * never registered the rail is simply left to R35 (off at/after reset). */
void power_set_mixer_gate(void (*gate)(bool));

/* Register a sink for each monitor sample (battery mV or -1 if untrustworthy,
 * external power, aerial terminated). Used to drive the on-screen readout. */
void power_set_report_cb(void (*cb)(int batt_mv, bool ext, bool terminated));

/* True when this boot is a wake from line-open deep sleep (ext0 or the
 * periodic recheck timer). main must then bring up ONLY the carrier + mixer,
 * register the two gates, and call power_line_recheck() before anything
 * expensive. */
bool power_line_recheck_pending(void);

/* Runtime inhibit for the sleep/shutdown actions (monitoring and logging
 * continue). OTA mode sets this for the duration of an update. */
void power_inhibit_actions(bool inhibit);

/* Arm/disarm the aerial-open deep-sleep action (the loader's SETTINGS menu
 * stores it in NVS, main applies it at boot). Default DISARMED: ext0 wake
 * is broken by R18, so an armed board on an open line only returns on the
 * recheck timer. Detection and logging run either way. The flat-battery
 * shutdown is a separate compile-time switch (POWER_ACTIONS_ENABLED).
 * Call before power_line_recheck() / power_start_monitor(). */
void power_set_line_sleep(bool enabled);

/* Probe the line (single clamped mean against the CW carrier) and external
 * power. Re-enters deep sleep -- and does not return -- if the line is still
 * open and no USB host/charger is present; returns to continue the boot
 * otherwise. Requires power_early_gate() and both gates registered. */
void power_line_recheck(void);

/* Probe the aerial line. Takes ~1.5 ms and puts up to 3.3 V (through R18, so
 * <=33 uA) on the centre conductor; harmless, and invisible to the modulator
 * because C4 blocks DC. Leaves the pull-up enabled on exit. */
void power_line_probe(power_line_t *out);

/* Convenience wrapper: just the verdict. */
bool power_line_terminated(void);

/* True if a USB HOST is actively enumerating us, from the USB-Serial/JTAG
 * start-of-frame counter (a host sends SOF every 1 ms). Catches a data host
 * instantly but is blind to a dumb charger. Blocks for a few ms. */
bool power_usb_host_present(void);

/* True if the board appears to be running on external power: either a USB host
 * is enumerating, or the cell is being held at the charger's CV target. This
 * is the one to gate sleep on -- it covers non-data chargers, which is the
 * common case for a device like this. */
bool power_external_power_present(void);

/* Call early in app_main, before BLE/video/sound bring-up. Returns false if
 * the board is shutting down on a flat pack (in which case it does not
 * return at all once actions are armed). */
bool power_early_gate(void);

/* Deep sleep until a set is plugged into the aerial. Wakes on ext0 (GPIO9
 * going low), so it is instant rather than poll-rate limited, and needs
 * neither the ULP nor a timer. */
void power_sleep_until_line_connected(void);

/* Deep sleep on a flat pack, re-checking periodically in case a charger turns
 * up (there is no VBUS pin to wake on). */
void power_shutdown_flat_battery(void);

/* Start the background monitor task: logs a periodic summary, and (once
 * thresholds are calibrated) handles low-battery shutdown and line-open sleep.
 * Also samples the SoC die temperature and logs it every 10 s as
 * "temp: die <c> C (min <a> max <b>)", which the bench tools parse into every
 * capture -- the RF modulator's transfer curve drifts with temperature and
 * nothing else on the board measures it. */
void power_start_monitor(void);

/* Latest die temperature in Celsius, and the min/max since boot. Returns
 * -273 before the first sample or if the sensor could not be brought up.
 * DIE temperature: a few degrees above the board, not the modulator's
 * junction -- a reproducible proxy, not a transistor temperature. */
float power_temp_c(void);
float power_temp_min_c(void);
float power_temp_max_c(void);
