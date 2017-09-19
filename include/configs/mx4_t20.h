/*
 * Copyright (c) 2017 Host Mobility AB
 *
 * Configuration settings for the Toradex Colibri T20 modules on a MX-4 board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx4-tegra20-common.h"

#define MX4_PRODUCT_TYPE "t20"

#define MTDIDS_DEFAULT		"nand0=tegra_nand"
#define MTDPARTS_DEFAULT	"mtdparts=tegra_nand:"		\
				"2m(u-boot)ro,"			\
				"1m(u-boot-env),"		\
				"1m(cfgblock)ro,"		\
				"8m(kernel),"		\
				"256m(config),"		\
				"-(ubi)"

#define BOARD_EXTRA_ENV_SETTINGS \
	COMMON_EXTRA_ENV_SETTINGS \
	"kernel_addr_nand=0x00400000\0"

#include "tegra-common-usb-gadget.h"
#include "tegra-common-post.h"

#endif /* __CONFIG_H */
