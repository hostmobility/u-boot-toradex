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

#define CONFIG_COLIBRI_T20	/* Toradex Colibri T20 module */

//#define CONFIG_TEGRA2_LP0

/* High-level configuration options */
#define TEGRA2_SYSMEM		"mem=256M@0M"
#define V_PROMPT		"Tegra2 # "

#define CONFIG_OF_UPDATE_FDT_BEFORE_BOOT 1

#include "tegra2-common.h"

//careful this might fail kernel booting
#undef CONFIG_BOOTSTAGE			/* Record boot time */
#undef CONFIG_BOOTSTAGE_REPORT		/* Print a boot time report */

#define CONFIG_SYS_NAND_BASE_LIST {}

//#define USB_KBD_DEBUG
#define CONFIG_USB_KEYBOARD

#define CONFIG_CONSOLE_MUX
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_STD_DEVICES_SETTINGS	"stdin=serial,usbkbd\0" \
					"stdout=serial,lcd\0" \
					"stderr=serial,lcd\0"

#define CONFIG_SYS_BOARD_ODMDATA	0x300d8011 /* lp1, 1GB */

#define CONFIG_REVISION_TAG		1
#define CONFIG_SERIAL_TAG		1

#define CONFIG_TRDX_CFG_BLOCK

#define CONFIG_CMD_ASKENV
#define CONFIG_CMD_DIAG
#define CONFIG_CMD_ELF
#undef CONFIG_CMD_FLASH
#define CONFIG_CMD_SAVEENV
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_LZO

#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY	5
#define CONFIG_NETMASK		255.255.255.0
#undef CONFIG_IPADDR
#define CONFIG_IPADDR		192.168.10.2
#undef CONFIG_SERVERIP
#define CONFIG_SERVERIP		192.168.10.1
#undef CONFIG_BOOTFILE		/* passed by BOOTP/DHCP */

#define CONFIG_BZIP2
#define CONFIG_CRC32_VERIFY
#define CONFIG_TIMESTAMP

#define CONFIG_AUTO_COMPLETE

#define CONFIG_SYS_USE_UBI
#define CONFIG_MTD_DEVICE
#define CONFIG_CMD_JFFS2
#define CONFIG_JFFS2_NAND
#define CONFIG_JFFS2_CMDLINE
#define CONFIG_RBTREE

#undef CONFIG_LINUXCONSOLE	/* dynamically adjusted */

#define DEFAULT_BOOTCOMMAND					\
	"run flashboot; run nfsboot"

#define FLASH_BOOTCMD						\
	"run setup; "						\
	"setenv bootargs ${defargs} ${flashargs} ${mtdparts} ${setupargs}; "	\
	"echo Booting from NAND...; "				\
	"nboot ${loadaddr} 0 ${lnxoffset} && bootm"

#define MMC_BOOTCMD						\
	"echo Loading RAM disk and kernel from MMC/SD card...; "\
	"mmc init && "						\
	"fatload mmc 0:1 0xC08000 rootfs-ext2.img.gz && "	\
	"fatload mmc 0:1 ${loadaddr} uImage;"			\
	"run ramboot"

#define NFS_BOOTCMD						\
	"run setup; "						\
	"setenv bootargs ${defargs} ${nfsargs} ${mtdparts} ${setupargs}; "	\
	"echo Booting from NFS...; "				\
	"usb start; "						\
	"dhcp; "						\
	"bootm"

#define RAM_BOOTCMD						\
	"run setup; "						\
	"setenv bootargs ${defargs} ${ramargs} ${mtdparts} ${setupargs}; "	\
	"echo Booting from RAM...; "				\
	"bootm"

#define UBI_BOOTCMD						\
	"run setup; "						\
	"setenv bootargs ${defargs} ${ubiargs} ${mtdparts} ${setupargs}; "	\
	"echo Booting from NAND...; "				\
	"ubi part kernel-ubi && ubi read ${loadaddr} kernel; "	\
	"bootm"

#define USB_BOOTCMD						\
	"echo Loading RAM disk and kernel from USB stick...; "	\
	"usb start && "						\
	"fatload usb 0:1 0xC08000 rootfs-ext2.img.gz && "	\
	"fatload usb 0:1 ${loadaddr} uImage;"			\
	"run ramboot"

#define SD_BOOTCMD					\
	"run setup; "						\
	"setenv bootargs ${defargs} ${sdargs} ${mtdparts} ${setupargs}; " \
	"echo Booting from MMC/SD card...; " \
	"mmc read 0 ${loadaddr} 0x2a00 0x4000; " \
	"bootm"

#undef CONFIG_BOOTARGS
#undef CONFIG_BOOTCOMMAND
#undef CONFIG_DIRECT_BOOTARGS
#define CONFIG_BOOTCOMMAND	DEFAULT_BOOTCOMMAND
#define CONFIG_NFSBOOTCOMMAND	NFS_BOOTCMD
#define CONFIG_RAMBOOTCOMMAND	RAM_BOOTCMD
//moved from disk/part_efi.h to here, give the block where the GP1 partition starts
//compare with sdargs below
#ifdef __CONFIG_SDBOOT_H
#define GPT_PRIMARY_PARTITION_TABLE_LBA	18945ULL
#else
#define GPT_PRIMARY_PARTITION_TABLE_LBA	1ULL
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_STD_DEVICES_SETTINGS \
	"defargs=video=tegrafb vmalloc=128M usb_high_speed=1\0" \
	"flashargs=ip=off root=/dev/mtdblock0 rw rootfstype=yaffs2\0" \
	"flashboot=" FLASH_BOOTCMD "\0" \
	"mmcboot=" MMC_BOOTCMD "\0" \
	"nfsargs=ip=:::::eth0:on root=/dev/nfs rw netdevwait\0" \
	"ramargs=initrd=0xA1800000,32M ramdisk_size=32768 root=/dev/ram0 rw\0" \
	"sdargs=root=/dev/mmcblk0p1 ip=:::::eth0:off rw,noatime rootfstype=ext3 rootwait gpt gpt_sector=18945\0" \
	"sdboot=" SD_BOOTCMD "\0" \
	"setup=setenv setupargs asix_mac=${ethaddr} no_console_suspend=1 console=tty1 console=ttyS0,${baudrate}n8 debug_uartport=lsport,0 ${memargs}\0" \
	"ubiargs=ubi.mtd=0 root=ubi0:rootfs rootfstype=ubifs\0" \
	"ubiboot=" UBI_BOOTCMD "\0" \
	"usbboot=" USB_BOOTCMD "\0" \
	""

/* Dynamic MTD partition support */
#define CONFIG_CMD_MTDPARTS	/* Enable 'mtdparts' command line support */
#define CONFIG_MTD_PARTITIONS	/* ??? */
#define CONFIG_MTD_DEVICE	/* needed for mtdparts commands */
#define MTDIDS_DEFAULT		"nand0=tegra_nand"

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

/* PMU and EMC support, requires i2c */
#define CONFIG_TEGRA_PMU
#define CONFIG_TEGRA_CLOCK_SCALING

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
/* Environment stored in NAND flash */
#define CONFIG_ENV_IS_IN_NAND		1 /* use NAND for environment vars */
#if defined(CONFIG_ENV_IS_IN_NAND)
/* once the nand is detected the corresponding setting is taken */
#define CONFIG_ENV_OFFSET		(gd->env_offset)
#define CONFIG_ENV_RANGE		0x80000
#endif /* CONFIG_ENV_IS_IN_NAND */
#endif /* !CONFIG_ENV_IS_NOWHERE */

/*
 *  LCDC configuration
 */
#define CONFIG_LCD
#define CONFIG_VIDEO_TEGRA

/* TODO: This needs to be configurable at run-time */
#define LCD_BPP				LCD_COLOR16
#define CONFIG_SYS_WHITE_ON_BLACK	/* Console colors */

#define CONFIG_DEFAULT_DEVICE_TREE	"colibri_t20"

/* NAND support */
#define CONFIG_CMD_NAND
#define CONFIG_TEGRA2_NAND

/* Max number of NAND devices */
#define CONFIG_SYS_MAX_NAND_DEVICE	1

#define CONFIG_CMD_IMI

#endif /* __CONFIG_H */
