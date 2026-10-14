/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * DFRobot UNIHIKER K10: ESP32-S3 N16R8 (16 MB flash, 8 MB octal PSRAM),
 * 2.8" 240x320 ILI9341 SPI LCD (no touch), speaker (raw I2S TX, no codec
 * control) + ES7243E mic pair (I2S RX, TDM mode) sharing one I2S bus, and a
 * TCA9555 I/O expander @0x20 carrying the LCD backlight (P0.0) and buttons
 * A (P0.2) and B (P1.4). BOOT is GPIO0, which is also the I2S BCLK, so it is
 * never used as a runtime button.
 *
 * Pins follow the proven k10-genius firmware (bapul-droid/k10-genius,
 * main/boards/dfrobot/df-k10/): config.h, df_k10_board.cc
 * (InitializeIli9341Display, InitializeIoExpander, InitializeButtons) and
 * k10_audio_codec.cc (CreateDuplexChannels, ES7243E input).
 *
 * Audio path (the critical finding): the donor NEVER performs I2C to the
 * ES8311. Its K10AudioCodec constructor takes an es8311_addr parameter but
 * never uses it; no output codec is ever created; the speaker is driven by
 * raw i2s_channel_write() with the codec at power-up default. We replicate
 * that: the speaker esp_codec_dev is created with codec_if = NULL, which
 * esp_codec_dev supports by design (esp_codec_dev_new/open/write/
 * set_out_vol/close are all NULL-safe; software volume is auto-allocated).
 * The mic uses es7243e_codec_new @0x23 exactly like the donor.
 *
 * There is no vendor BSP in the ESP Component Registry, so like the AIPI Lite
 * board file this drives esp_lcd, the codecs and the GPIOs directly. The
 * TCA9555 is driven with minimal direct I2C register writes (no
 * esp_io_expander dependency): backlight (P0.0) and P0.1 are outputs;
 * the button pins stay inputs. P0.1 is undocumented in the donor but the
 * proven k10-genius pulses P0+P1 low/high at init (required for audio);
 * we mirror that sequence exactly and leave P0.1 output-high.
 */
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "muse_settings.h"
#include "muse_audio.h"
#include "muse_menu.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "muse_audio.h"
#include "muse_board.h"
#include "muse_mem.h"

static const char *TAG = "board";

#define LCD_H_RES 240
#define LCD_V_RES 320
#define LCD_HOST SPI3_HOST
#define LCD_SCLK GPIO_NUM_12
#define LCD_MOSI GPIO_NUM_21
#define LCD_CS GPIO_NUM_14
#define LCD_DC GPIO_NUM_13
#define LCD_RST GPIO_NUM_NC   /* panel reset is tied to the board reset */
#define DRAW_BUF_LINES 32

#define I2C_PORT I2C_NUM_1
#define I2C_SDA GPIO_NUM_47   /* shared by the codecs and the TCA9555 */
#define I2C_SCL GPIO_NUM_48

/* TCA9555 I/O expander. Minimal direct register access; see the file header. */
#define TCA9555_ADDR 0x20
#define TCA9555_REG_INPUT0  0x00
#define TCA9555_REG_OUTPUT0 0x02
#define TCA9555_REG_CONFIG0 0x06
#define TCA9555_REG_CONFIG1 0x07
#define TCA_BL_BIT   0  /* P0.0: LCD backlight, output, active high */
#define TCA_P1_BIT   1  /* P0.1: pulsed low/high at init per proven donor; left output-high */
#define TCA_BTNA_BIT 2  /* P0.2: input, active low. NOTE: software calls this "A"
                         * but hardware verification (2026-10-06) proves P0.2 =
                         * PHYSICAL B (the PTT button). Do not rename the symbol;
                         * use PHYSICAL_B below for new code. */
#define TCA9555_REG_INPUT1 0x01
/* Physical button mapping (hardware-verified 2026-10-06):
 *   PHYSICAL_A = TCA9555 P1.4 (pin 12), active-low. Volume/menu button.
 *   PHYSICAL_B = TCA9555 P0.2, active-low. PTT button (existing, unchanged).
 * The old TCA_BTNA_BIT symbol name is reversed vs. physical labels; it is
 * kept for the proven PTT path. New code uses PHYSICAL_A/B. */
#define PHYSICAL_A_BIT 4  /* P1.4 */
#define PHYSICAL_B_BIT 2  /* P0.2 (same as TCA_BTNA_BIT) */

#define I2S_MCLK GPIO_NUM_3
#define I2S_BCLK GPIO_NUM_0   /* also BOOT: never a runtime button */
#define I2S_WS GPIO_NUM_38
#define I2S_DOUT GPIO_NUM_45
#define I2S_DIN GPIO_NUM_39
/* Donor passes 0x23 as the mic address to es7243e_codec_new (config.h:
 * AUDIO_CODEC_ES7210_ADDR 0x23); the ES7243E driver's own default is 0x20,
 * but the K10 board wires it at 0x23. */
#define ES7243E_ADDR 0x23

static i2c_master_bus_handle_t s_i2c;
static i2c_master_dev_handle_t s_tca;
static esp_lcd_panel_handle_t s_panel;

static esp_err_t tca_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_tca, buf, sizeof(buf), 50);
}

static esp_err_t tca_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_tca, &reg, 1, val, 1, 50);
}

static esp_err_t init(void)
{
    const i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_cfg, &s_i2c), TAG, "i2c");

    const i2c_device_config_t tca_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA9555_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c, &tca_cfg, &s_tca), TAG, "tca9555");

    /* The expander must answer; without it there is no backlight or buttons. */
    uint8_t probe = 0;
    ESP_RETURN_ON_ERROR(tca_read_reg(TCA9555_REG_CONFIG0, &probe), TAG, "tca9555 probe");

    /* P0.0 (backlight) + P0.1 = output; everything else stays input.
     * Proven donor (df_k10_board.cc InitializeIoExpander) drives P0+P1 low
     * for 100 ms then high at init; audio codecs stay silent without it. */
    ESP_RETURN_ON_ERROR(tca_write_reg(TCA9555_REG_CONFIG0, 0xFC), TAG, "tca config0");
    ESP_RETURN_ON_ERROR(tca_write_reg(TCA9555_REG_CONFIG1, 0xFF), TAG, "tca config1");
    ESP_RETURN_ON_ERROR(tca_write_reg(TCA9555_REG_OUTPUT0, 0x00), TAG, "tca out0 low");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(tca_write_reg(TCA9555_REG_OUTPUT0, 0x03), TAG, "tca out0 high");
    /* Backlight (P0) is driven by set_brightness() from here on; P1 stays high. */
    return ESP_OK;
}

static lv_display_t *display_start(lv_indev_t **touch)
{
    (void)touch;   /* no touch controller on this board */
    const spi_bus_config_t bus = {
        .sclk_io_num = LCD_SCLK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_H_RES * DRAW_BUF_LINES * 2,
    };
    if (spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_io_handle_t io;
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = LCD_CS,
        .dc_gpio_num = LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    if (esp_lcd_new_panel_io_spi(LCD_HOST, &io_cfg, &io) != ESP_OK) {
        return NULL;
    }
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    if (esp_lcd_new_panel_ili9341(io, &panel_cfg, &s_panel) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    /* Proven k10-genius values (config.h): no inversion, mirror Y. */
    esp_lcd_panel_invert_color(s_panel, false);
    esp_lcd_panel_mirror(s_panel, false, true);
    esp_lcd_panel_disp_on_off(s_panel, true);

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = s_panel,
        .panel_io = io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = LCD_H_RES,
            .ver_res = LCD_V_RES,
            .buffer_height = DRAW_BUF_LINES,
            .use_psram = false,
            .require_double_buffer = true,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    if (!disp || esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    return disp;
}

static bool display_lock(int timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

static void set_brightness(int pct)
{
    /* Backlight is digital on this board: on for any pct > 0. */
    uint8_t out = 0;
    if (tca_read_reg(TCA9555_REG_OUTPUT0, &out) != ESP_OK) {
        return;
    }
    if (pct > 0) {
        out |= (1u << TCA_BL_BIT);
    } else {
        out &= ~(1u << TCA_BL_BIT);
    }
    tca_write_reg(TCA9555_REG_OUTPUT0, out);
}

static void panel_sleep(bool sleep)
{
    esp_lcd_panel_disp_sleep(s_panel, sleep);   /* SLPIN/SLPOUT; GRAM is kept */
}

static void display_pause(bool pause)
{
    if (pause) {
        esp_lv_adapter_pause(-1);
    } else {
        esp_lv_adapter_resume();
    }
}

/*
 * Audio: one duplex I2S bus, replicating the proven k10-genius layout
 * (k10_audio_codec.cc CreateDuplexChannels @ 37ad771).
 *
 * The donor NEVER performs I2C to the ES8311: its K10AudioCodec takes an
 * es8311_addr parameter but never uses it, no output codec is ever created,
 * and the speaker is driven by raw i2s_channel_write() with the codec at
 * power-up default. We replicate that exactly: the speaker esp_codec_dev
 * is created with codec_if = NULL. This is not a fake driver; it is
 * esp_codec_dev's designed no-codec path (esp_codec_dev.c: esp_codec_dev_new
 * only requires data_if; open/write/set_out_vol/close are all NULL-safe,
 * and a software volume stage is auto-allocated when the codec has no
 * set_vol). No I2C transaction to 0x18 is ever issued.
 *
 * TX: I2S STD mode like the donor (16-bit, left-aligned, bit shift), but
 * STEREO slot mode instead of the donor's MONO: Muse writes
 * stereo-interleaved buffers, and muse_audio_write always sets L == R, so
 * the wire format is identical to the donor's MONO+SLOT_BOTH. MCLK is left
 * unassigned on TX; the RX/TDM channel drives it on GPIO3, as in the
 * proven firmware.
 *
 * RX: I2S TDM mode like the donor (bclk_div 8: 512 kHz BCLK from the
 * 4.096 MHz MCLK), but with 2 slots (0|1) instead of the donor's 4: the two
 * mics live on slots 0 and 1 (the donor's channel mask), and Muse reads
 * stereo pairs via esp_codec_dev_read.
 *
 * Mic: es7243e_codec_new @0x23, ctrl_if only, exactly like the donor.
 *
 * Both directions run at MUSE_AUDIO_RATE (16 kHz) instead of the donor's
 * 24 kHz; the MCLK/BCLK ratios are unchanged.
 */
static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    i2s_chan_handle_t tx, rx;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx, &rx), TAG, "i2s channel");

    /* TX: donor's proven STD slot layout, stereo for Muse's writes. */
    i2s_std_config_t tx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,   /* driven by the RX channel, per donor */
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &tx_cfg), TAG, "i2s tx");

    /* RX: donor's proven TDM layout, 2 slots for Muse's stereo reads. */
    i2s_tdm_config_t rx_cfg = {
        .clk_cfg = {
            .sample_rate_hz = MUSE_AUDIO_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            .bclk_div = 8,   /* proven: 512 kHz BCLK from 4.096 MHz MCLK */
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = (1u << 0) | (1u << 1),  /* TDM slots 0+1: the two mics */
            .ws_width = I2S_TDM_AUTO_WS_WIDTH,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = false,   /* donor's proven RX alignment */
            .big_endian = false,
            .bit_order_lsb = false,
            .skip_mask = false,
            .total_slot = I2S_TDM_AUTO_SLOT_NUM,
        },
        .gpio_cfg = {
            .mclk = I2S_MCLK,
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DIN,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_tdm_mode(rx, &rx_cfg), TAG, "i2s rx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx), TAG, "i2s tx on");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx), TAG, "i2s rx on");

    audio_codec_i2s_cfg_t i2s_cfg = { .port = I2S_NUM_0, .rx_handle = rx, .tx_handle = tx };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(data_if, ESP_ERR_NO_MEM, TAG, "i2s data if");

    /* Speaker: no codec control device. esp_codec_dev accepts codec_if = NULL
     * by design; volume is applied in software and samples go straight to
     * the I2S TX DMA, exactly like the donor's raw i2s_channel_write. */
    esp_codec_dev_cfg_t out_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = NULL,
        .data_if = data_if,
    };
    *spk = esp_codec_dev_new(&out_cfg);

    /* Mic: ES7243E @0x23, donor-proven (ctrl_if only). */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_NUM_1,
        .addr = ES7243E_ADDR,
        .bus_handle = s_i2c,
    };
    const audio_codec_ctrl_if_t *mic_ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(mic_ctrl, ESP_ERR_NO_MEM, TAG, "mic ctrl if");
    es7243e_codec_cfg_t es7243e_cfg = { .ctrl_if = mic_ctrl };
    const audio_codec_if_t *mic_codec = es7243e_codec_new(&es7243e_cfg);
    ESP_RETURN_ON_FALSE(mic_codec, ESP_FAIL, TAG, "ES7243E not responding");

    esp_codec_dev_cfg_t in_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = mic_codec,
        .data_if = data_if,
    };
    *mic = esp_codec_dev_new(&in_cfg);
    return *spk && *mic ? ESP_OK : ESP_FAIL;
}

static bool s_a_pressed = false;
static uint8_t s_a_stable = 0;

/* Physical A (P1.4) gesture state machine.
 * Poll rate: 10ms (POLL_MS in muse_input.c), 50ms when display paused.
 * Timings below assume 10ms; at 50ms they scale 5x (still usable).
 */
#define PHYA_DEBOUNCE_TICKS 3    /* 30ms: match existing P0.2 debounce */
#define PHYA_LONG_TICKS     80   /* 800ms: hold -> volume-up mode */
#define PHYA_DOUBLE_TICKS   40   /* 400ms: second press -> double-click */
#define PHYA_REPEAT_TICKS   15   /* 150ms: volume-up repeat while held */
#define PHYA_VOL_STEP       5    /* volume step percent */

static bool s_phya_pressed = false;      /* debounced P1.4 state */
static uint8_t s_phya_stable = 0;        /* debounce counter */
static uint16_t s_phya_hold = 0;         /* ticks held (for long-press) */
static uint16_t s_phya_repeat = 0;       /* ticks since last repeat */
static uint16_t s_phya_double = 0;       /* ticks since last short release */
static bool s_phya_long = false;         /* in long-press volume-up mode */
static bool s_phya_vol_live = false;     /* live volume applied, needs persist on release */
static int s_phya_live_vol = -1;           /* live volume during hold-repeat (-1 = not tracking) */

/* Physical A gesture helper: apply volume change immediately (live). */
static void phya_volume_apply(int pct)
{
    muse_audio_set_volume(pct);  /* live, no NVS yet */
    s_phya_vol_live = true;
}

/* Physical A gesture helper: persist volume after gesture completes. */
static void phya_volume_commit(void)
{
    if (s_phya_vol_live && s_phya_live_vol >= 0) {
        muse_settings_set_volume(s_phya_live_vol);
        s_phya_vol_live = false;
        s_phya_live_vol = -1;
    }
}

static unsigned poll_buttons(void)
{
    unsigned ev = 0;

    /* ---- PHYSICAL B (P0.2): PTT path. UNCHANGED. ---- */
    uint8_t in0 = 0xFF;
    if (tca_read_reg(TCA9555_REG_INPUT0, &in0) == ESP_OK) {
        bool raw = !(in0 & (1u << TCA_BTNA_BIT));  /* TCA_BTNA_BIT = P0.2 = PHYSICAL B */
        if (raw != s_a_pressed) {
            if (++s_a_stable >= 3) {
                s_a_stable = 0;
                s_a_pressed = raw;
                ev |= raw ? MUSE_BTN_TALK_PRESS : MUSE_BTN_TALK_RELEASE;
            }
        } else {
            s_a_stable = 0;
        }
    }

    /* ---- PHYSICAL A (P1.4): gesture state machine. ---- */
    uint8_t in1 = 0xFF;
    bool phya_raw = s_phya_pressed;  /* default: no change */
    if (tca_read_reg(TCA9555_REG_INPUT1, &in1) == ESP_OK) {
        phya_raw = !(in1 & (1u << PHYSICAL_A_BIT));  /* P1.4 active-low */
    }
    /* Debounce */
    if (phya_raw != s_phya_pressed) {
        if (++s_phya_stable >= PHYA_DEBOUNCE_TICKS) {
            s_phya_stable = 0;
            bool was = s_phya_pressed;
            s_phya_pressed = phya_raw;
            bool menu_open = muse_menu_is_open();

            if (phya_raw && !was) {
                /* PRESS edge */
                s_phya_hold = 0;
                s_phya_repeat = 0;
                s_phya_long = false;
                s_phya_live_vol = -1;
                if (menu_open) {
                    /* In menu: A short = next item (existing behavior via menu_button).
                     * Return AUX_PRESS so muse_input.c calls menu_button() ->
                     * muse_menu_key(MUSE_MENU_DOWN). */
                    ev |= MUSE_BTN_AUX_PRESS;
                }
                /* In normal mode: wait to distinguish short/long/double. No event yet. */
            } else if (!phya_raw && was) {
                /* RELEASE edge */
                if (menu_open) {
                    ev |= MUSE_BTN_AUX_RELEASE;
                } else if (s_phya_long) {
                    /* Was in long-press mode: persist volume, done. No click. */
                    phya_volume_commit();
                    int v = muse_settings_volume();
                    ESP_LOGI("VOL", "volume: %d%%", v);
                } else {
                    /* Short release: check for double-click */
                    if (s_phya_double > 0) {
                        /* Double-click: open menu. Do NOT volume-down. */
                        s_phya_double = 0;
                        ev |= MUSE_BTN_AUX_PRESS;  /* triggers menu open via menu_button() */
                        /* Note: menu_button() on press calls muse_menu_key(MUSE_MENU_DOWN)
                         * which opens the menu. We don't want the DOWN action, just open.
                         * The menu opens on first DOWN; this is the existing behavior for
                         * aux button on non-touch boards. */
                    } else {
                        /* First click: start double-click window. Volume-down deferred. */
                        s_phya_double = PHYA_DOUBLE_TICKS;
                    }
                }
                s_phya_hold = 0;
                s_phya_long = false;
            }
        }
    } else {
        s_phya_stable = 0;
    }

    /* Physical A held: check for long-press and repeat (normal mode only) */
    if (s_phya_pressed && !muse_menu_is_open()) {
        s_phya_hold++;
        if (!s_phya_long && s_phya_hold >= PHYA_LONG_TICKS) {
            /* Enter long-press mode: volume UP (do NOT do volume-down first) */
            s_phya_long = true;
            s_phya_double = 0;  /* cancel pending double-click */
            s_phya_live_vol = muse_settings_volume() + PHYA_VOL_STEP;
            if (s_phya_live_vol > 100) s_phya_live_vol = 100;
            phya_volume_apply(s_phya_live_vol);
            ESP_LOGI("VOL", "volume: %d%%", s_phya_live_vol);
            s_phya_repeat = 0;
        } else if (s_phya_long) {
            s_phya_repeat++;
            if (s_phya_repeat >= PHYA_REPEAT_TICKS) {
                s_phya_repeat = 0;
                if (s_phya_live_vol < 0) s_phya_live_vol = muse_settings_volume();
                s_phya_live_vol += PHYA_VOL_STEP;
                if (s_phya_live_vol > 100) s_phya_live_vol = 100;
                phya_volume_apply(s_phya_live_vol);
                ESP_LOGI("VOL", "volume: %d%%", s_phya_live_vol);
            }
        }
    }

    /* Double-click timeout: if window expires with no second press, do volume-down */
    if (s_phya_double > 0 && !s_phya_pressed) {
        s_phya_double--;
        if (s_phya_double == 0 && !muse_menu_is_open()) {
            /* Single short press confirmed: volume DOWN */
            int v = muse_settings_volume() - PHYA_VOL_STEP;
            if (v < 0) v = 0;
            muse_settings_set_volume(v);  /* persist immediately (infrequent) */
            ESP_LOGI("VOL", "volume: %d%%", v);
        }
    }

    return ev;
}

static esp_err_t power_off(void)
{
    /* No power latch on this board. Backlight and panel off, then deep sleep
     * with no wake source configured: restarting needs a power cycle. */
    set_brightness(0);
    esp_lcd_panel_disp_on_off(s_panel, false);
    esp_deep_sleep_start();
    return ESP_FAIL;
}

static const muse_board_t s_board = {
    .name = "UNIHIKER K10",
    .width = LCD_H_RES,
    .height = LCD_V_RES,
    .round = false,
    .touch = false,
    .diagonal_in = 2.8f,
    .talk_button = "left",
    .aux_button = "right",
    .talk_hint = { LV_ALIGN_BOTTOM_LEFT, 15, -4 },
    .aux_hint = { LV_ALIGN_BOTTOM_RIGHT, -14, -4 },
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .panel_sleep = panel_sleep,
    .display_pause = display_pause,
    .audio_init = audio_init,
    .mic_slot = -1,             /* two mics; mix them down */
    .poll_buttons = poll_buttons,
    .power_off = power_off,
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
