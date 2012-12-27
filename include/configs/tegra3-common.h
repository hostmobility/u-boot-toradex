/*
 *  (C) Copyright 2010-2012
 *  NVIDIA Corporation <www.nvidia.com>
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

#ifndef __TEGRA3_COMMON_H
#define __TEGRA3_COMMON_H
#include <asm/sizes.h>

#include "tegra-common.h"

#define CONFIG_TEGRA3			/* NVidia Tegra3 core */

/*
 * USB Host.
 */
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_TEGRA

/* Tegra requires USB buffers to be aligned to a 32-byte boundary */
#define CONFIG_USB_EHCI_DATA_ALIGN	32

/*
 * This parameter affects a TXFILLTUNING field that controls how much data is
 * sent to the latency fifo before it is sent to the wire. Without this
 * parameter, the default (2) causes occasional Data Buffer Errors in OUT
 * packets depending on the buffer address and size.
 */
#define CONFIG_USB_EHCI_TXFIFO_THRESH	10

#define CONFIG_EHCI_IS_TDI
#define CONFIG_EHCI_DCACHE
#define CONFIG_EHCI_HAS_STATUS2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 3
#define CONFIG_USB_STORAGE
#define CONFIG_USB_STOR_NO_RETRY

#define CONFIG_CMD_USB		/* USB Host support		*/

/* partition types and file systems we want */
#define CONFIG_DOS_PARTITION
#define CONFIG_EFI_PARTITION
#define CONFIG_CMD_EXT2

/* include default commands */
#include <config_cmd_default.h>

/* remove unused commands */
#undef CONFIG_CMD_FLASH		/* flinfo, erase, protect */
#undef CONFIG_CMD_FPGA		/* FPGA configuration support */
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_NFS		/* NFS support */
#undef CONFIG_CMD_NET

#define CONFIG_CMD_CACHE

/* turn on command-line edit/hist/auto */
#define CONFIG_CMDLINE_EDITING
#define CONFIG_COMMAND_HISTORY
#define CONFIG_AUTOCOMPLETE

#define CONFIG_SYS_NO_FLASH

#ifdef CONFIG_TEGRA3_WARMBOOT
#define TEGRA_LP0_ADDR			0x9C406000
#define TEGRA_LP0_SIZE			SZ_8K
#define TEGRA_LP0_VEC \
	"lp0_vec=" QUOTE(TEGRA_LP0_SIZE) "@" QUOTE(TEGRA_LP0_ADDR) " "
#else
#define TEGRA_LP0_VEC
#endif

#define CONFIG_LOADADDR		0x80408000	/* def. location for kernel */
#define CONFIG_BOOTDELAY	3		/* -1 to disable auto boot */
#define CONFIG_ZERO_BOOTDELAY_CHECK

/* Passed on the kernel command line to specify the console. */
#define CONFIG_LINUXCONSOLE "console=ttyS0,115200n8"

/*
 * Defines the standard boot args; these are used in the vboot case (which
 * doesn't run regen_all) as well as used as part of regen_all.
 */
#define CONFIG_BOOTARGS \
		CONFIG_LINUXCONSOLE " " \
		"usbcore.old_scheme_first=1 " \
		"core_edp_mv=1300 " \
		TEGRA3_SYSMEM " " \
		CONFIG_EXTRA_BOOTARGS


/*
 * Extra bootargs used for direct booting, but not for vboot.
 * Not needed for T30.
 */
#define CONFIG_DIRECT_BOOTARGS		CONFIG_BOOTARGS

#define CONFIG_SYS_LOAD_ADDR		(0x80A00800)	/* default */

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKBASE	(NV_PA_SDRAM_BASE + 0x2800000)	/* 40MB */
#define CONFIG_STACKSIZE	0x20000		/* 128K regular stack*/

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	1
#define PHYS_SDRAM_1		TEGRA_SDRC_CS0
#define PHYS_SDRAM_1_SIZE	0x40000000	/* 1GB */

#define CONFIG_SYS_TEXT_BASE	0x80108000
#define CONFIG_SYS_SDRAM_BASE	PHYS_SDRAM_1

#define CONFIG_INITRD_TAG		/* enable initrd ATAG */
#define BCT_SDRAM_PARAMS_OFFSET	(BCT_OFFSET + 0x148)

#endif /* __TEGRA3_COMMON_H */
