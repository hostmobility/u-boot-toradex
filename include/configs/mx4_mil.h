/*
 * Copyright (c) 2016 Host Mobility AB
 *
 * Configuration settings for the MX-4 MIL board
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx4-tegra20-common.h"

#undef CONFIG_ENV_SIZE	/* undef size from mx4-tegra20-common.h */
#define CONFIG_ENV_SIZE		(SZ_4K)

#undef HM_UPDATE_FILE_NAME
#define HM_UPDATE_FILE_NAME	"mil_hmupdate.img"

/* hmupdate.img from USB is not enabled on MX-4 MIL*/
#undef PROBE_USB_FOR_RAMDISK
#define PROBE_USB_FOR_RAMDISK	""
#undef PROBE_USB_FOR_HMUPDATE
#define PROBE_USB_FOR_HMUPDATE	""
#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND \
	"if run probe_ubi; then " \
		"if source ${loadaddr}; then " \
			"exit; " \
		"else " \
			"bootm ${loadaddr}; " \
		"fi; " \
	"fi; " \
	"run ubiboot;"

#define MX4_PRODUCT_TYPE	"mil"

/* High-level configuration options */
#define V_PROMPT			"MX-4 MIL # "

#define MTDIDS_DEFAULT		"nand0=tegra_nand"
#define MTDPARTS_DEFAULT	"mtdparts=tegra_nand:"		\
				"2m(u-boot)ro,"			\
				"1m(u-boot-env),"		\
				"1m(cfgblock)ro,"		\
				"8m(kernel),"			\
				"256m(config),"			\
				"-(ubi)"

/* Fixme: prob not the right way */
#define FDT_FILENAME "tegra20-mx4-mil.dtb"

#define BOARD_EXTRA_ENV_SETTINGS				\
	CONFIG_COMMON_EXTRA_ENV_SETTINGS			\
	"kernel_addr_nand=0x00400000\0"				\
	"fdt_filename=" FDT_FILENAME "\0"			\
	"ubiload_fdt=ubi part ubi &&"				\
 	 " ubifsmount ubi:rootfs &&"				\
	 " ubifsload ${fdt_addr_r} /boot/${fdt_filename}\0"

#undef UBI_BOOTCMD
#define UBI_BOOTCMD							\
	"ubiboot=run setup; setenv bootargs ${defargs} ${hmargs}"	\
	  " ${ubiargs} ${mtdparts} ${setupargs} ${vidargs};"		\
	  " mx4_pic restart;"						\
	  " run ubiload_fdt;"						\
	  " nand read ${kernel_addr_r} kernel &&"			\
	  " bootz ${kernel_addr_r} - ${fdt_addr_r}"

#undef CONFIG_SYS_DFU_DATA_BUF_SIZE
#include "tegra-common-post.h"

#endif /* __CONFIG_H */
