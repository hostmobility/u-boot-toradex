/*
 * Copyright (c) 2011 The Chromium OS Authors.
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <lcd.h>
#include <fdt_decode.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pinmux.h>
#include <asm/arch-tegra/power.h>
#include <asm/arch-tegra/pwfm.h>
#include <asm/arch-tegra/display.h>
#include <asm/system.h>

DECLARE_GLOBAL_DATA_PTR;

/* These are the stages we go throuh in enabling the LCD */
enum stage_t {
	STAGE_START,
	STAGE_LVDS,
	STAGE_BACKLIGHT_VDD,
	STAGE_PWFM,
	STAGE_BACKLIGHT_EN,
	STAGE_DONE,
};

static enum stage_t stage;	/* Current stage we are at */
static unsigned long timer_next; /* Time we can move onto next stage */
static struct fdt_lcd config;	/* Our LCD config, set up in handle_stage() */

enum {
	/* Maximum LCD size we support */
	LCD_MAX_WIDTH		= 1920,
	LCD_MAX_HEIGHT		= 1200,
	LCD_MAX_LOG2_BPP	= 5,		/* 32 bpp */
};

int lcd_line_length;
int lcd_color_fg;
int lcd_color_bg;

void *lcd_base;			/* Start of framebuffer memory	*/
void *lcd_console_address;	/* Start of console buffer	*/

short console_col;
short console_row;

vidinfo_t panel_info = {
	/* Insert a value here so that we don't end up in the BSS */
	.vl_col = -1,
};

char lcd_cursor_enabled;	/* set initial value to false */

ushort lcd_cursor_width;
ushort lcd_cursor_height;

#if defined(CONFIG_TEGRA2)
/*
 * The PINMUX macro is used per board to setup the pinmux configuration.
 */
#define PINMUX(grp, mux, pupd, tri)                   \
	{PINGRP_##grp, PMUX_FUNC_##mux, PMUX_PULL_##pupd, PMUX_TRI_##tri}

struct pingroup_config pinmux_cros_1[] = {
	PINMUX(GPU,   PWM,        NORMAL,    NORMAL),
	PINMUX(LD0,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD1,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD10,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD11,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD12,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD13,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD14,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD15,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD16,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD17,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD2,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD3,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD4,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD5,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD6,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD7,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD8,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD9,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LDI,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LHP0,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LHP1,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LHP2,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LHS,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LM0,   RSVD4,      NORMAL,    NORMAL),
	PINMUX(LPP,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LPW0,  RSVD4,      NORMAL,    NORMAL),
	PINMUX(LPW1,  RSVD4,      NORMAL,    TRISTATE),
	PINMUX(LPW2,  RSVD4,      NORMAL,    NORMAL),
	PINMUX(LSC0,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LSPI,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LVP1,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LVS,   DISPA,      NORMAL,    NORMAL),
	PINMUX(SLXD,  SPDIF,      NORMAL,    NORMAL),
};
#endif

/* Initialize the Tegra LCD panel and controller */
void init_lcd(struct fdt_lcd *config)
{
	display_clk_init();
	power_enable_partition(POWERP_3D);
	tegra2_display_register(config);
}

void lcd_cursor_size(ushort width, ushort height)
{
	lcd_cursor_width = width;
	lcd_cursor_height = height;
}

void lcd_toggle_cursor(void)
{
	ushort x, y;
	uchar *dest;
	ushort row;

	x = console_col * lcd_cursor_width;
	y = console_row * lcd_cursor_height;
	dest = (uchar *)(lcd_base + y * lcd_line_length + x * (1 << LCD_BPP) /
			8);

	for (row = 0; row < lcd_cursor_height; ++row, dest += lcd_line_length) {
		ushort *d = (ushort *)dest;
		ushort color;
		int i;

		for (i = 0; i < lcd_cursor_width; ++i) {
			color = *d;
			color ^= lcd_color_fg;
			*d = color;
			++d;
		}
	}
}

void lcd_cursor_on(void)
{
	lcd_cursor_enabled = 1;
	lcd_toggle_cursor();
}
void lcd_cursor_off(void)
{
	lcd_cursor_enabled = 0;
	lcd_toggle_cursor();
}

char lcd_is_cursor_enabled(void)
{
	return lcd_cursor_enabled;
}

static void update_panel_size(struct fdt_lcd *config)
{
	panel_info.vl_col = config->width;
	panel_info.vl_row = config->height;
	panel_info.vl_bpix = config->log2_bpp;
}

/*
 *  Main init function called by lcd driver.
 *  Inits and then prints test pattern if required.
 */

void lcd_ctrl_init(void *lcdbase)
{
	int size;

	/*
	 * The framebuffer address should be specified in the device tree.
	 * This FDT value should be the same as the one defined in Linux kernel;
	 * otherwise, it causes screen flicker. The FDT value overrides the
	 * framebuffer allocated at the top of memory by board_init_f().
	 *
	 * If the framebuffer address is not defined in the FDT, falls back to
	 * use the address allocated by board_init_f().
	 */
	if (config.frame_buffer != ADDR_T_NONE) {
		gd->fb_base = config.frame_buffer;
		lcd_base = (void *)(gd->fb_base);
	} else {
		config.frame_buffer = (u32)lcd_base;
	}

	/* Make sure that we can acommodate the selected LCD */
	assert(config.width <= LCD_MAX_WIDTH);
	assert(config.height <= LCD_MAX_HEIGHT);
	assert(config.log2_bpp <= LCD_MAX_LOG2_BPP);
	if (config.width <= LCD_MAX_WIDTH &&
		config.height <= LCD_MAX_HEIGHT &&
			config.log2_bpp <= LCD_MAX_LOG2_BPP)
		update_panel_size(&config);
	size = lcd_get_size(&lcd_line_length),

	/* call board specific hw init */
	init_lcd(&config);

	/* For write-through or cache off, change the LCD memory region */
	if (!(config.cache_type & FDT_LCD_CACHE_WRITE_BACK))
		mmu_set_region_dcache(config.frame_buffer, size,
			config.cache_type & FDT_LCD_CACHE_WRITE_THROUGH ?
				DCACHE_WRITETHROUGH : DCACHE_OFF);

	/* Enable flushing after LCD writes if requested */
	lcd_set_flush_dcache(config.cache_type & FDT_LCD_CACHE_FLUSH);

	debug("LCD frame buffer at %p\n", lcd_base);
}

ulong calc_fbsize(void)
{
	return (panel_info.vl_col * panel_info.vl_row *
		NBITS(panel_info.vl_bpix)) / 8;
}

void lcd_setcolreg(ushort regno, ushort red, ushort green, ushort blue)
{
}

void lcd_early_init(const void *blob)
{
	/*
	 * Go with the maximum size for now. We will fix this up after
	 * relocation. These values are only used for memory alocation.
	 */
	panel_info.vl_col = LCD_MAX_WIDTH;
	panel_info.vl_row = LCD_MAX_HEIGHT;
	panel_info.vl_bpix = LCD_MAX_LOG2_BPP;
}

static int handle_stage(const void *blob)
{
	/* do the things for this stage */
	switch (stage) {
	case STAGE_START:
		/* get panel details */
		if (fdt_decode_lcd(blob, &config)) {
			printf("No LCD information in device tree\n");
			return -1;
		}

#if defined(CONFIG_TEGRA2)
		/* TODO: put pinmux into the FDT */
		pinmux_config_table(pinmux_cros_1, ARRAY_SIZE(pinmux_cros_1));
#endif

		fdt_setup_gpio(&config.panel_vdd);
		fdt_setup_gpio(&config.lvds_shutdown);
		fdt_setup_gpio(&config.backlight_vdd);
		fdt_setup_gpio(&config.backlight_en);

		gpio_set_value(config.panel_vdd.gpio, 1);
		break;

	case STAGE_LVDS:
		gpio_set_value(config.lvds_shutdown.gpio, 1);
		break;

	case STAGE_BACKLIGHT_VDD:
		if (fdt_gpio_isvalid(&config.backlight_vdd))
			gpio_set_value(config.backlight_vdd.gpio, 1);
		break;

	case STAGE_PWFM:
		/* Enable PWM at 15/16 high, divider 1 */
		pwfm_setup(config.pwfm, 1, 0xdf, 1);
		break;

	case STAGE_BACKLIGHT_EN:
		gpio_set_value(config.backlight_en.gpio, 1);
		break;

	case STAGE_DONE:
		break;
	}

	/* set up timer for next stage */
	timer_next = timer_get_us() + config.panel_timings[stage] * 1000;

	/* move to next stage */
	stage++;
	return 0;
}

int tegra_lcd_check_next_stage(const void *blob, int wait)
{
	int err = 0;

	if (stage == STAGE_DONE)
		return 0;

	bootstage_start(BOOTSTAGE_LCD_WAIT, "lcd_wait");
	do {
		/* wait if we need to */
		debug("%s: stage %d\n", __func__, stage);
		if (stage != STAGE_START) {
			int delay = timer_next - timer_get_us();

			if (delay > 0) {
				if (wait)
					udelay(delay);
				else
					return 0;
			}
		}

		if (handle_stage(blob))
			err = -1;

	} while (wait && !err && stage != STAGE_DONE);
	bootstage_accum(BOOTSTAGE_LCD_WAIT);
	return err;
}

void lcd_enable(void)
{
	/*
	 * This will be done when tegra_lcd_check_next_stage() is called
	 * board_late_init().
	 */
}
