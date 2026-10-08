/*
 * 48 MHz reference clock on GPIO17 via the I2S0 MCLK divider (PLL240M / 5).
 *
 * PLL240M is tx_clk_sel == 1 in i2s_struct.h, but IDF's
 * i2s_ll_tx_clk_set_src() refuses to select it, so the source mux and the
 * divider are programmed directly. 240 MHz / 5 is a plain integer divide
 * (num == 0 -> all fractional fields zero), which gives a clean 48.000 MHz with
 * no dither spurs. See clkout.h for the validation story.
 */

#include "clkout.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"
#include "esp_private/periph_ctrl.h"
#include "hal/i2s_ll.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_struct.h"

#define CLK_GPIO   17    /* any GPIO -- MCLK has no IO_MUX path, matrix only */
#define BOOT_GPIO  0     /* devkit BOOT button (active low, external pull-up) */
#define PLL240M    1     /* tx_clk_sel value for the 240 MHz PLL             */
#define DIV_INT    5     /* 240 MHz / 5 = 48.000 MHz                         */

static bool s_clk_on;    /* current carrier state */

/* Program the raw MCLK divider. For an integer divide the fractional fields
 * (x/y/z/yn1) are all zero; see the encoding in i2s_ll_tx_set_mclk(). */
static void i2s0_set_raw_div(uint32_t n, uint32_t x, uint32_t y,
                             uint32_t z, uint32_t yn1)
{
    I2S0.tx_clkm_conf.tx_clkm_div_num = n;
    typeof(I2S0.tx_clkm_div_conf) d = {};
    d.tx_clkm_div_x   = x;
    d.tx_clkm_div_y   = y;
    d.tx_clkm_div_z   = z;
    d.tx_clkm_div_yn1 = yn1;
    I2S0.tx_clkm_div_conf.val = d.val;
}

void clkout_start_48mhz(void)
{
    /* Idempotent: the wake-recheck path starts the carrier early and the
     * normal boot flow calls this again -- re-programming the running I2S
     * PLL dividers mid-stream is at best a glitch and suspected in a
     * post-wake TG1WDT reset (2026-09-15). */
    static bool s_started;
    if (s_started) return;
    s_started = true;

    PERIPH_RCC_ATOMIC() {
        i2s_ll_enable_bus_clock(0, true);
        i2s_ll_reset_register(0);
    }

    gpio_config_t out = {
        .pin_bit_mask = 1ULL << CLK_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);
    gpio_set_drive_capability(CLK_GPIO, GPIO_DRIVE_CAP_3);

    I2S0.tx_clkm_conf.clk_en   = 1;   /* ungate the clkm registers */
    I2S0.rx_clkm_conf.mclk_sel = 0;   /* MCLK pin follows TX clock */

    I2S0.tx_clkm_conf.tx_clk_active = 0;   /* stop before touching mux/divider */
    I2S0.tx_clkm_conf.tx_clk_sel    = PLL240M;

    /* Espressif's workaround: latching a small divide straight from the reset
     * default can capture an inaccurate value, so write a nonsensical set of
     * coefficients first, then the real integer divide. */
    i2s0_set_raw_div(7, 317, 7, 3, 0);
    i2s0_set_raw_div(DIV_INT, 0, 0, 0, 0);

    esp_rom_gpio_connect_out_signal(CLK_GPIO, I2S0_MCLK_OUT_IDX, false, false);
    I2S0.tx_clkm_conf.tx_clk_active = 1;
    s_clk_on = true;
}

/* Route the pin to the MCLK signal (carrier on) or to a static-low GPIO output
 * (carrier off). The I2S clock keeps running; only the pin connection changes,
 * so re-enabling is glitch-free and instant. */
#define MIXER_SHUTDOWN_GPIO 44

void mixer_set_enabled(bool on)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << MIXER_SHUTDOWN_GPIO,
        .mode         = GPIO_MODE_INPUT_OUTPUT,   /* INPUT too: read the pin back */
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(MIXER_SHUTDOWN_GPIO, on ? 0 : 1);   /* P-FET: low = rail on */
    ESP_LOGI("clkout", "mixer rail %s (GPIO%d reads %d)", on ? "ON" : "off",
             MIXER_SHUTDOWN_GPIO, gpio_get_level(MIXER_SHUTDOWN_GPIO));
}

void clkout_set_enabled(bool on)
{
    if (on) {
        esp_rom_gpio_connect_out_signal(CLK_GPIO, I2S0_MCLK_OUT_IDX, false, false);
    } else {
        esp_rom_gpio_connect_out_signal(CLK_GPIO, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_level(CLK_GPIO, 0);
    }
    s_clk_on = on;
}

/* Poll-and-debounce BOOT: fire once per press, on the falling edge (3 x 20 ms). */
static bool boot_pressed(void)
{
    static bool    stable = true;   /* released == high */
    static uint8_t agree  = 0;
    bool raw = gpio_get_level(BOOT_GPIO);

    if (raw == stable) { agree = 0; return false; }
    if (++agree < 3)   { return false; }
    agree  = 0;
    stable = raw;
    return (stable == false);       /* press, not release */
}

static void clkout_button_task(void *arg)
{
    for (;;) {
        if (boot_pressed()) {
            clkout_set_enabled(!s_clk_on);
            ESP_LOGI("clkout", "48 MHz carrier %s (BOOT)", s_clk_on ? "ON" : "OFF");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void clkout_enable_boot_toggle(void)
{
    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << BOOT_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);
    xTaskCreatePinnedToCore(clkout_button_task, "clkbtn", 2560, NULL, 3, NULL, 0);
}
