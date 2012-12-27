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

#ifndef _TEGRA3_GPIO_H_
#define _TEGRA3_GPIO_H_

#define CONFIG_TEGRA_MAX_GPIO_PORT	30	/* GPIO_PEEx */

#include <asm/arch-tegra/gpio.h>

/*
 * The Tegra 3x GPIO controller has 246 GPIOs arranged in 8 banks of 4 ports,
 * each with 8 GPIOs.
 */

/* GPIO Controller registers for a single bank */
struct gpio_ctlr_bank {
	uint gpio_config[TEGRA_GPIO_PORTS];
	uint gpio_dir_out[TEGRA_GPIO_PORTS];
	uint gpio_out[TEGRA_GPIO_PORTS];
	uint gpio_in[TEGRA_GPIO_PORTS];
	uint gpio_int_status[TEGRA_GPIO_PORTS];
	uint gpio_int_enable[TEGRA_GPIO_PORTS];
	uint gpio_int_level[TEGRA_GPIO_PORTS];
	uint gpio_int_clear[TEGRA_GPIO_PORTS];

	uint gpio_masked_config[TEGRA_GPIO_PORTS];
	uint gpio_masked_dir_out[TEGRA_GPIO_PORTS];
	uint gpio_masked_out[TEGRA_GPIO_PORTS];
	uint gpio_masked_in[TEGRA_GPIO_PORTS];
	uint gpio_masked_int_status[TEGRA_GPIO_PORTS];
	uint gpio_masked_int_enable[TEGRA_GPIO_PORTS];
	uint gpio_masked_int_level[TEGRA_GPIO_PORTS];
	uint gpio_masked_int_clear[TEGRA_GPIO_PORTS];
};

struct gpio_ctlr {
	struct gpio_ctlr_bank gpio_bank[TEGRA_GPIO_BANKS];
};

#define GPIO_PCC0   224
#define GPIO_PCC1   225
#define GPIO_PCC2   226
#define GPIO_PCC3   227
#define GPIO_PCC4   228
#define GPIO_PCC5   229
#define GPIO_PCC6   230
#define GPIO_PCC7   231
#define GPIO_PDD0   232
#define GPIO_PDD1   233
#define GPIO_PDD2   234
#define GPIO_PDD3   235
#define GPIO_PDD4   236
#define GPIO_PDD5   237
#define GPIO_PDD6   238
#define GPIO_PDD7   239
#define GPIO_PEE0   240
#define GPIO_PEE1   241
#define GPIO_PEE2   242
#define GPIO_PEE3   243
#define GPIO_PEE4   244
#define GPIO_PEE5   245
#define GPIO_PEE6   246
#define GPIO_PEE7   247

#endif	/* TEGRA3_GPIO_H_ */
