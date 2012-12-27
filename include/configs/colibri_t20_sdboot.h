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

/*
 * Use this configuration for an u-boot which stores its environment in SD/MMC
 * Useful if SD/MMC is selected as the boot device, so also the environment is taken from there
 *
 * Use colibri_t20.h for everything which is not specific to using SD/MMC as the boot device!!
 */

/*
SD-BOOT
The sd sector numbers are used as follows:
u-boot needs to find ENV and LNX to get to the environment and the kernel, the kernel needs to find the APP partition for the rootfs.
ENV: colibri_t20_sdboot.h
Set CONFIG_ENV_MMC_OFFSET to the byte start address of ENV, take the sector address of Partid 6 (which is in 2024 byte sectors)

LNX: colibri_t20.h
Set the u-boot environment SDBOOTCMD below, mmc read RAMloadaddr, 512byte sector start, 512byte copy length
take the sector address of Partid 7 (which is in 2024 byte sectors)

APP: colibri_t20.h
Set the u-boot environment sdargs and the variable GPT_PRIMARY_PARTITION_TABLE_LBA below.
The kernel finds the partition with the help of the GP1/GPT partitions. The kernel commandline must include "gpt gpt_sector=xxx"
to force it to use GUID Partition Table (GPT) and to give the 512byte sector start address of the primary GUID.
Set this in
Take the start address of Partid 8 (which is in 2048 byte sectors). add 1 to the resulting 512byte block. (at pos. 0 is the MBR)

E.g. Output during nvflash procedure on serial console:
SD Alloc Partid=2, start sector=0,num=1536
SD Alloc Partid=3, start sector=1536,num=64
SD Alloc Partid=4, start sector=1600,num=960
SD Alloc Partid=5, start sector=2560,num=64
SD Alloc Partid=6, start sector=2624,num=64
SD Alloc Partid=7, start sector=2688,num=2048
SD Alloc Partid=8, start sector=4736,num=512
SD Alloc Partid=9, start sector=5248,num=500032
SD Alloc Partid=10, start sector=505280,num=460352

sector start address 4736 * 2048 -> 18944 * 512 -> GPT start sector is 18945.
*/


#ifndef __CONFIG_SDBOOT_H
#define __CONFIG_SDBOOT_H

#include "colibri_t20.h"

#undef DEFAULT_BOOTCOMMAND
#undef CONFIG_BOOTCOMMAND

#define DEFAULT_BOOTCOMMAND \
	"run sdboot; run nfsboot"
#define CONFIG_BOOTCOMMAND	DEFAULT_BOOTCOMMAND


#ifdef CONFIG_ENV_IS_NOWHERE
#undef CONFIG_ENV_IS_NOWHERE
#endif
#ifdef CONFIG_ENV_IS_IN_NAND
#undef CONFIG_ENV_IS_IN_NAND
#endif

/* Environment stored in SD/MMC */
#define CONFIG_ENV_IS_IN_MMC 1

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV 0 /* use MMC0, slot on eval board and Iris */
#define CONFIG_ENV_MMC_OFFSET  (10496 * 512) /* Byteaddress of the beginning of the ENV partition */
#endif

#endif /* __CONFIG_H */
