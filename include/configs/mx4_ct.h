/*
 * Copyright (c) 2017 Host Mobility AB
 *
 * Configuration settings for the Toradex Colibri T20 modules on a MX-4 CT board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx4-tegra20-common.h"

/* LCD support */
#define CONFIG_SYS_WHITE_ON_BLACK
#define CONFIG_CMD_BMP
#define CONFIG_LCD_LOGO

#define MX4_PRODUCT_TYPE "ct"

#define MTDIDS_DEFAULT		"nand0=tegra_nand"
#define MTDPARTS_DEFAULT	"mtdparts=tegra_nand:"		\
				"3072K@0K(BCT),"		\
				"256K@4096K(PT),"		\
				"2048K@5376K(EBT),"		\
				"256K@8448K(BMP),"		\
				"2048K@9728K(ENV),"		\
				"8192K@12800K(LNX),"		\
				"256K@22272K(ARG),"		\
				"10240K@23808K(PER),"		\
				"10240K@35328K(SER),"		\
				"476416K@46848K(USR)"

#define BOARD_EXTRA_ENV_SETTINGS \
	COMMON_EXTRA_ENV_SETTINGS \
	"vidargs=video=tegrafb0:800x480CT-32@60 video=tegrafb1:1920x1080-32@60\0" \
	"kernel_addr_nand=0x00c80000\0"

#include "tegra-common-usb-gadget.h"
#include "tegra-common-post.h"

#endif /* __CONFIG_H */
