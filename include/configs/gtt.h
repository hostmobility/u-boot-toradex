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
#define V_PROMPT		"MX4 # "

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
#define CONFIG_BOOTDELAY	0
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

/* Note! HUSH scripting needs to be enabled for bootcommand/autoscripting */
#define DEFAULT_BOOTCOMMAND \
    "if run probe_usb || run probe_ubi; then " \
	    "if source ${loadaddr}; then " \
	    	"exit; " \
	    "else " \
	    	"bootm ${loadaddr}; " \
	    "fi; " \
    "fi; " \
    "run ubiboot; run flashboot;"

#define FLASH_BOOTCMD						\
	"run setup; "						\
	"setenv bootargs ${defargs} ${flashargs} ${mtdparts} ${setupargs}; "	\
	"echo Booting from NAND...; "				\
	"nboot ${loadaddr} 0 ${lnxoffset} && bootm"

#define RAM_BOOTCMD						\
	"run setup; " \
	"setenv bootargs ${defargs} ${ramargs} ${mtdparts} ${setupargs}; " \
	"echo Booting from RAM...; "\
	"bootm ${loadaddr} ${ramdisk_loadaddr}; " \

#define UBI_BOOTCMD						\
	"mx4_pic restart; "					\
	"run setup; "						\
	"setenv bootargs ${defargs} ${ubiargs} ${mtdparts} ${setupargs}; "	\
	"echo Booting from UBI NAND...; "				\
	"nboot ${loadaddr} 0 ${lnxoffset} && bootm"

#define PROBE_USB_FOR_HMUPDATE \
	"if mx4_pic is_extr; then " \
	"usb start && fatls usb 0:1 && " \
	"mx4_pic set_state 2 && fatload usb 0:1 ${loadaddr} ${updatefilename}; fi "

#define PROBE_UBI_FOR_HMUPDATE \
	"if ${firmware_update} -eq true; then " \
	"ubi part USR; ubifsmount rootfs; "\
	"ubifsload ${loadaddr} /boot/${updatefilename} && mx4_pic set_state 2; fi "

#define PROBE_USB_FOR_RAMDISK \
	"if mx4_pic is_extr; then " \
	"usb start && fatload usb 0:1 ${loadaddr} ${kernelfilename} " \
	"&& fatload usb 0:1 ${ramdisk_loadaddr} ${ramdiskfilename} " \
	"&& run ramboot; fi "

#undef CONFIG_BOOTARGS
#undef CONFIG_BOOTCOMMAND
#undef CONFIG_DIRECT_BOOTARGS
#define CONFIG_BOOTCOMMAND	DEFAULT_BOOTCOMMAND
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
	"firmware_update=false\0" \
	"probe_usb=" PROBE_USB_FOR_HMUPDATE "\0" \
	"probe_ubi=" PROBE_UBI_FOR_HMUPDATE "\0" \
	"probe_ramdisk=" PROBE_USB_FOR_RAMDISK "\0" \
	"updatefilename=hmupdate.img\0" \
	"kernelfilename=uImage\0" \
	"ramdiskfilename=uRamdisk\0" \
	"PRODUCT=VCC\0" \
	"ramdisk_loadaddr=1000000\0" \
	"splashimage=0x408000\0" \
	"flashargs=ip=off root=/dev/mtdblock0 rw rootfstype=yaffs2\0" \
	"flashboot=" FLASH_BOOTCMD "\0" \
	"ubiboot=" UBI_BOOTCMD "\0" \
	"ubiargs=ubi.mtd=USR root=ubi0:rootfs rootfstype=ubifs\0" \
	"ramargs=root=/dev/ram0 rw\0" \
	"memargs=mem=498M@0M fbmem=12M@498M nvmem=2M@510M\0" \
	"defargs=video=tegrafb vmalloc=128M usb_high_speed=1 quiet fbcon=map:1 consoleblank=0\0" \
	"vidargs=video=tegrafb0:800x480CT-32@60 video=tegrafb1:1920x1080-32@60\0" \
	"setup=setenv setupargs asix_mac=${ethaddr} asix_mac2=${ethaddr2} no_console_suspend=1 console=tty1 console=ttyS0,${baudrate}n8 debug_uartport=lsport,0 ${memargs} ${vidargs}\0" \
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

/* SPI */
#define CONFIG_TEGRA_SPI
#define CONFIG_USE_SLINK        /* SPI to PIC on SPI4_CS0 */
#define CONFIG_CMD_SPI
#define CONFIG_PING_MX4 /* Ping co-processor on MX4 via SPI */
#define CONFIG_CMD_MX4_PIC


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

/* Enable ramdisk boot */
#define CONFIG_INITRD_TAG 1
#define CONFIG_SETUP_MEMORY_TAGS 1
/* Enable ramdisk boot, end */

#endif /* __CONFIG_H */
