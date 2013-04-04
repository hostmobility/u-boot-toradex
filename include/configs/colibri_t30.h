/*
 * Copyright (C) 2012 Toradex, Inc.
 * Portions Copyright (c) 2010, 2011 NVIDIA Corporation
 * Portions Copyright (c) 2011 The Chromium OS Authors
 *
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

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/sizes.h>

/* High-level configuration options */
#define TEGRA3_SYSMEM		"mem=1023M@2048M vmalloc=128M"
#define V_PROMPT		"Tegra3 # "

#define CONFIG_COLIBRI_T30	/* Toradex Colibri T30 module */
#define CONFIG_TEGRA3_CARDHU
/* This currently fails environment memory allocation */
#undef CONFIG_SYS_SKIP_ARM_RELOCATION
//#define CONFIG_SYS_SKIP_ARM_RELOCATION
//#define CONFIG_SYS_PLLP_BASE_IS_408MHZ

#include "tegra3-common.h"
/* undef the enviroment settings in tegra3-common.h / tegra-common.h */
#undef CONFIG_ENV_SIZE
#undef CONFIG_LOADADDR
#undef CONFIG_BOOTDELAY
#undef CONFIG_BOOTARGS
#undef CONFIG_BOOTCOMMAND
#undef CONFIG_DIRECT_BOOTARGS

//careful this might fail kernel booting
#undef CONFIG_BOOTSTAGE			/* Record boot time */
#undef CONFIG_BOOTSTAGE_REPORT		/* Print a boot time report */

//#define USB_KBD_DEBUG
#define CONFIG_USB_KEYBOARD

#define CONFIG_CONSOLE_MUX
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_STD_DEVICES_SETTINGS	"stdin=serial,usbkbd\0" \
					"stdout=serial,lcd\0" \
					"stderr=serial,lcd\0"

#define CONFIG_SYS_BOARD_ODMDATA	0x40080105	/* 1GB, UARTA, etc */

/* PMU */
/* disabled for now #define CONFIG_HW_WATCHDOG */

#define CONFIG_BOOTDELAY	5
#define CONFIG_NETMASK		255.255.255.0
#undef CONFIG_IPADDR
#define CONFIG_IPADDR		192.168.10.2
#undef CONFIG_SERVERIP
#define CONFIG_SERVERIP		192.168.10.1

/* ENV */
#define CONFIG_ENV_SIZE         SZ_4K /* env size is a maximum of 4k */
#define CONFIG_ENV_SECT_SIZE    CONFIG_ENV_SIZE

#define CONFIG_NFSBOOTCOMMAND						\
	"setenv bootargs ${defargs} ${nfsargs}; "	\
	"echo Booting from NFS...; "				\
	"usb start; "						\
	"dhcp; "						\
	"bootm ${loadaddr} "

#define USB_BOOTCMD \
	"setenv bootargs ${defargs} root=/dev/sda3 rw rootwait; " \
	"echo Booting from USB Stick...; " \
	"usb start; " \
	"fatload usb 0:1 ${loadaddr} uimage; " \
	"bootm ${loadaddr} "

#define EMMC_BOOTCMD \
	"setenv bootargs ${defargs} ${mmcargs} ;" \
	"echo Booting from internal eMMC card...; " \
	"mmc read 0 ${loadaddr} ${mmc_kernel_sec}; " \
	"bootm ${loadaddr} "

#define CONFIG_BOOTCOMMAND "run emmcboot; run usbboot; run nfsboot;"
#define CONFIG_LOADADDR 0x80408000
#define CONFIG_ETHADDR 00:0e:c6:87:72:01
#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_STD_DEVICES_SETTINGS \
	"board=colibri_t30\0" \
	"defargs=console=ttyS0,115200n8 debug_uartport=lsport lp0_vec=0x00002000@0x9C406000 video=tegrafb mem=1023M@2048M vmalloc=128M noinitrd usbcore.old_scheme_first=1 core_edp_mv=1300 panel=lvds tegraid=30.1.2.0.0 tegra_fbmem=3072K@0xBFE00000 \0" \
	"emmcboot=" EMMC_BOOTCMD "\0" \
	"mmcargs=root=/dev/mmcblk0p1 ip=:::::eth0:off rw,noatime rootwait rootfstype=ext3 gpt gpt_sector=40960 \0" \
	"mmc_kernel_sec=0x5000 0x4000 \0" \
	"nfsargs=ip=:::::eth0:on root=/dev/nfs rw netdevwait\0" \
	"usbboot=" USB_BOOTCMD "\0"


/* GPIO */
#define CONFIG_TEGRA_GPIO
#define CONFIG_CMD_TEGRA_GPIO_INFO

/* I2C */
#define CONFIG_TEGRA_I2C
#define CONFIG_SYS_I2C_INIT_BOARD
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_MAX_I2C_BUS		4
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_CMD_I2C

/* PMU */
#define CONFIG_TEGRA_PMU

/* SD/MMC */
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_TEGRA_MMC
#define CONFIG_CMD_MMC

#define CONFIG_DOS_PARTITION
#define CONFIG_EFI_PARTITION
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT

/* Environment not stored */
//#define CONFIG_ENV_IS_NOWHERE
#ifndef CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_IS_IN_MMC	1 /* use eMMC for environment vars */
#endif
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV 0 /* use MMC0 */
#define CONFIG_ENV_OFFSET       (16384 * 512) /* Byteaddress of the beginning of the ENV partition */
#endif

/* support USB ethernet adapters */
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX

/*
 * Ethernet support
 */
#define CONFIG_CMD_NET
#define CONFIG_NET_MULTI
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP

/*
 *  LCDC configuration
 */
#define CONFIG_LCD
#define CONFIG_VIDEO_TEGRA

/* TODO: This needs to be configurable at run-time */
#define LCD_BPP				LCD_COLOR16
#define CONFIG_SYS_WHITE_ON_BLACK	/* Console colors */

#define CONFIG_DEFAULT_DEVICE_TREE	"colibri_t30"

#endif /* __CONFIG_H */
