/*
 * Copyright 2017 Host Mobility AB
 *
 * Configuration settings for the Toradex VF50/VF61 module on MX-4 V61 board.
 *
 * Based on colibri_vf.h:
 * Copyright 2015-2016 Toradex, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX4_VF_COMMON_H
#define __MX4_VF_COMMON_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>

#define CONFIG_SYS_THUMB_BUILD
#define CONFIG_USE_ARCH_MEMCPY
#define CONFIG_USE_ARCH_MEMSET
#define CONFIG_SYS_FSL_CLK

#define CONFIG_ARCH_MISC_INIT
#define CONFIG_DISPLAY_BOARDINFO_LATE	/* Calls show_board_info() */

#define CONFIG_SKIP_LOWLEVEL_INIT

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define MX4_PIC_SPI_BUS		1
#define MX4_PIC_SPI_CS		0

/* Enable passing of ATAGs */
#define CONFIG_CMDLINE_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 8 * 1024 * 1024)

#define CONFIG_BOARD_EARLY_INIT_F

/* Allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_VARS_UBOOT_CONFIG
#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_BAUDRATE			115200

/* NAND support */
#define CONFIG_CMD_NAND
#define CONFIG_CMD_WRITEBCB
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		NFC_BASE_ADDR

/* Dynamic MTD partition support */
#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_CMD_MTDPARTS	/* Enable 'mtdparts' command line support */
#define CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_DEVICE	/* needed for mtdparts commands */
#define MTDIDS_DEFAULT		"nand0=vf610_nfc"
#define MTDPARTS_DEFAULT	"mtdparts=vf610_nfc:"		\
					"128k(vf-bcb)ro,"		\
					"1408k(u-boot)ro,"		\
					"512k(u-boot-env),"		\
					"512k(u-boot-env2),"	\
					"8m(kernel),"			\
					"128m(config),"			\
					"-(ubi)"

#undef CONFIG_CMD_IMLS

#define CONFIG_MMC
#define CONFIG_FSL_ESDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_FSL_ESDHC_NUM	1

#define CONFIG_SYS_FSL_ERRATUM_ESDHC111

#define CONFIG_GENERIC_MMC
#define CONFIG_DOS_PARTITION

#define CONFIG_RBTREE

#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET1_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RMII
#define CONFIG_FEC_MXC_PHYADDR          0
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_IP_DEFRAG
#define CONFIG_TFTP_BLOCKSIZE		16352
#define CONFIG_TFTP_TSIZE

#define CONFIG_IPADDR		192.168.10.2
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_SERVERIP		192.168.10.1

#define CONFIG_BOARD_LATE_INIT

#define CONFIG_LOADADDR			0x80008000
#define CONFIG_FDTADDR			0x84000000

/* We boot from the gfxRAM area of the OCRAM. */
#define CONFIG_SYS_TEXT_BASE		0x3f408000
#define CONFIG_BOARD_SIZE_LIMIT		491520

#define UBI_BOOTCMD \
		"ubiboot=run setup; setenv bootargs ${defargs} ${ubiargs} " \
		"${mtdparts} ${setupargs} ${vidargs};" \
		"echo Booting from NAND...; " \
		"mx4_pic restart; " \
		"nboot ${kernel_addr_r} 0 0x00280000; && bootm ${kernel_addr_r}"

#define PROBE_USB_FOR_HMUPDATE \
	"if run is_firmware_update || mx4_pic is_extr; " \
	"then usb start && fatls usb 0:1 && " \
	"mx4_pic set_state 2 && fatload usb 0:1 ${loadaddr} ${updatefilename}; fi "

#define PROBE_UBI_FOR_HMUPDATE \
	"if ${firmware_update} -eq true; then " \
	"ubi part ubi; ubifsmount ubi0:rootfs; "\
	"ubifsload ${loadaddr} /boot/${updatefilename} && mx4_pic set_state 2; " \
	"ubifsumount; fi "

#define PROBE_USB_FOR_RAMDISK \
	"if mx4_pic is_extr; then " \
	"usb start && fatload usb 0:1 ${loadaddr} ${kernelfilename} " \
	"&& fatload usb 0:1 ${ramdisk_loadaddr} ${ramdiskfilename} " \
	"&& run ramboot; fi "

#define IS_FIRMWARE_UPDATE \
	"if ${firmware_update} -eq true; then true; fi"

#define CONFIG_BOOTCOMMAND \
    "if run probe_usb || run probe_ubi; then " \
	    "if source ${loadaddr}; then " \
	    	"exit; " \
	    "else " \
	    	"bootm ${loadaddr}; " \
	    "fi; " \
	"fi; " \
	"run probe_ramdisk; run ubiboot;"

#define MX4_CONSOLE_PORT "ttyLP0"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"firmware_update=false\0" \
	"probe_usb=" PROBE_USB_FOR_HMUPDATE "\0" \
	"probe_ubi=" PROBE_UBI_FOR_HMUPDATE "\0" \
	"probe_ramdisk=" PROBE_USB_FOR_RAMDISK "\0" \
	"is_firmware_update=" IS_FIRMWARE_UPDATE "\0" \
	"ramargs=root=/dev/ram0 rw\0" \
	"updatefilename=vf_hmupdate.img\0" \
	"kernelfilename=uImage\0" \
	"ramdiskfilename=uRamdisk\0" \
	"ramdisk_loadaddr=1000000\0" \
	"ramargs=root=/dev/ram0 rw\0" \
	"kernel_addr_r=0x82000000\0" \
	"fdt_addr_r=0x84000000\0" \
	"defargs=quiet\0" \
	"console=" MX4_CONSOLE_PORT"\0" \
	"setup=setenv setupargs " \
		"fec_mac=${ethaddr} console=tty1 console=${console}" \
		",${baudrate}n8 ${memargs}\0" \
	"mtdparts=" MTDPARTS_DEFAULT "\0" \
	"ubiargs=ubi.mtd=ubi root=ubi0:rootfs rootfstype=ubifs "	\
		"ubi.fm_autoconvert=1\0" \
	UBI_BOOTCMD

/* Miscellaneous configurable options */
#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_AUTOBOOT_PROMPT		\
	"\nMX4 - booting... stop with ENTER: %d\n"
#define CONFIG_AUTOBOOT_DELAY_STR	"\r"
#define CONFIG_RESET_TO_RETRY
#define CONFIG_BOOT_RETRY_TIME	30

#undef CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		1024	/* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE		\
			(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		16	/* max number of command args */
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START	0x80010000
#define CONFIG_SYS_MEMTEST_END		0x87C00000

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000
#define CONFIG_CMDLINE_EDITING

/*
 * Stack sizes
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE		SZ_256K

/* Physical memory map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			(0x80000000)
#define PHYS_SDRAM_SIZE			(256 * 1024 * 1024)

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE				(64 * 2048)
#define CONFIG_ENV_RANGE			(4 * 64 * 2048)
#define CONFIG_ENV_OFFSET			(12 * 64 * 2048)
#define CONFIG_ENV_OFFSET_REDUND		(CONFIG_ENV_OFFSET + 512 * 1024)

/* USB Host Support */
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_VF
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET

/* USB Storage */
#define CONFIG_USB_FUNCTION_MASS_STORAGE

#endif /* __MX4_VF_COMMON_H */
