/*
 *  (C) Copyright 2012
 *  Toradex, Inc.
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

#include <common.h> /* do not change order of include file */

#include <asm/arch/clock.h>
#ifdef CONFIG_TEGRA2
#include <asm/arch/emc.h>
#include <asm/arch/gpio.h>
#endif /* CONFIG_TEGRA2 */
#include <asm/arch/pinmux.h>
#ifdef CONFIG_TEGRA3
#include <asm/arch/pmu_core.h>
#include <asm/arch/pmu.h>
#endif /* CONFIG_TEGRA3 */
#ifdef CONFIG_TEGRA_MMC
#include <asm/arch/pmu.h>
#endif
#include <asm/arch/sys_proto.h>
#include <asm/arch/tegra.h>
#ifdef CONFIG_USB_EHCI_TEGRA
#include <asm/arch/usb.h>
#endif
#include <asm/arch-tegra/bitfield.h>
#include <asm/arch-tegra/clk_rst.h>
#include <asm/arch-tegra/pmc.h>
#include <asm/arch-tegra/uart.h>
#include <asm/arch-tegra/warmboot.h>
#include <asm/clocks.h>
#include <asm/io.h>

#include <fdt_decode.h>
#ifdef CONFIG_TEGRA_I2C
#include <i2c.h>
#endif
#include <libfdt.h>
#include <malloc.h>
#ifdef CONFIG_TEGRA_MMC
#include <mmc.h>
#endif
#include <nand.h>
#include <ns16550.h>
#include <watchdog.h>

#ifdef CONFIG_TEGRA3
#include "../colibri_t30/pinmux-config-common.h"
#endif
#include "../../nvidia/common/pmu.h"
#include "board.h"
#include "tegra2_partitions.h"

#if defined(CONFIG_TEGRA_CLOCK_SCALING) && !defined(CONFIG_TEGRA_I2C)
#error "tegra: We need CONFIG_TEGRA_I2C to support CONFIG_TEGRA_CLOCK_SCALING"
#endif

#if defined(CONFIG_TEGRA_CLOCK_SCALING) && !defined(CONFIG_TEGRA_PMU)
#error "tegra: We need CONFIG_TEGRA_PMU to support CONFIG_TEGRA_CLOCK_SCALING"
#endif

#define NV_ADDRESS_MAP_FUSE_BASE 0x7000f800
// Register FUSE_BOOT_DEVICE_INFO_0
#define FUSE_BOOT_DEVICE_INFO_0                 0x1bc
// Register FUSE_RESERVED_SW_0
#define FUSE_RESERVED_SW_0                      0x1c0

#define CLK_RST_BASE 0x60006000
// Register CLK_RST_CONTROLLER_MISC_CLK_ENB_0
#define CLK_RST_CONTROLLER_MISC_CLK_ENB_0       0x48

// Register APB_MISC_PP_STRAPPING_OPT_A_0
#define APB_MISC_PP_STRAPPING_OPT_A_0                   0x8

enum {
	/* UARTs which we can enable */
	UARTA		= 1 << 0,
	UARTB		= 1 << 1,
	UARTD		= 1 << 3,
	UART_ALL	= 0xf
};

typedef enum
{
    NvBootFuseBootDevice_Sdmmc,
    NvBootFuseBootDevice_SnorFlash,
    NvBootFuseBootDevice_SpiFlash,
    NvBootFuseBootDevice_NandFlash,
    NvBootFuseBootDevice_NandFlash_x8  = NvBootFuseBootDevice_NandFlash,
    NvBootFuseBootDevice_NandFlash_x16 = NvBootFuseBootDevice_NandFlash,
    NvBootFuseBootDevice_MobileLbaNand,
    NvBootFuseBootDevice_MuxOneNand,
    NvBootFuseBootDevice_Sata,
    NvBootFuseBootDevice_BootRom_Reserved_Sdmmc3,   /* !!! this enum is strictly used by BootRom code only !!! */
    NvBootFuseBootDevice_Max, /* Must appear after the last legal item */
    NvBootFuseBootDevice_Force32 = 0x7fffffff
} NvBootFuseBootDevice;

typedef enum
{
    NvStrapDevSel_Emmc_Primary_x4 = 0, /* eMMC primary (x4)          */
    NvStrapDevSel_Emmc_Primary_x8,     /* eMMC primary (x8)          */
    NvStrapDevSel_Emmc_Secondary_x4,   /* eMMC secondary (x4)        */
    NvStrapDevSel_Nand,                /* NAND (x8 or x16)           */
    NvStrapDevSel_Nand_42nm_x8,           /* NAND_42nm (x8)           */
    NvStrapDevSel_MobileLbaNand,       /* mobileLBA NAND             */
    NvStrapDevSel_MuxOneNand,          /* MuxOneNAND                 */
    NvStrapDevSel_Esd_x4,              /* eSD (x4)                   */
    NvStrapDevSel_SpiFlash,            /* SPI Flash                  */
    NvStrapDevSel_Snor_Muxed_x16,      /* Sync NOR (Muxed, x16)      */
    NvStrapDevSel_Snor_Muxed_x32,      /* Sync NOR (Muxed, x32)      */
    NvStrapDevSel_Snor_NonMuxed_x16,   /* Sync NOR (NonMuxed, x16)   */
    NvStrapDevSel_FlexMuxOneNand,      /* FlexMuxOneNAND             */
    NvStrapDevSel_Sata,                /* Sata                       */
    NvStrapDevSel_Emmc_Secondary_x8,   /* eMMC secondary (x8)        */
    NvStrapDevSel_UseFuses,            /* Use fuses instead          */

    /* The following definitions must be last. */
    NvStrapDevSel_Num, /* Must appear after the last legal item */
    NvStrapDevSel_Force32 = 0x7fffffff
} NvStrapDevSel;

typedef enum
{
    Trdx_BootDevice_NandFlash,
    Trdx_BootDevice_eMMC,
    Trdx_BootDevice_extSD,
    Trdx_BootDevice_Unknown,
    Trdx_BootDevice_Max, /* Must appear after the last legal item */
    Trdx_BootDevice_Force32 = 0x7fffffff
} TrdxBootDevice;

const char* sTrdxBootDeviceStr[] =
{
	"Trdx_BootDevice_NandFlash",
	"Trdx_BootDevice_eMMC",
	"Trdx_BootDevice_extSD",
	"Trdx_BootDevice_Unknown"
};

DECLARE_GLOBAL_DATA_PTR;

#if defined(BOARD_LATE_INIT) && (defined(CONFIG_TRDX_CFG_BLOCK) || \
		defined(CONFIG_REVISION_TAG) || defined(CONFIG_SERIAL_TAG))
static unsigned char *config_block = NULL;
#endif

#ifdef CONFIG_HW_WATCHDOG
static int i2c_is_initialized = 0;
#endif

/*
 * Possible UART locations: we ignore UARTC at 0x70006200 and UARTE at
 * 0x70006400, since we don't have code to init them
 */
static u32 uart_reg_addr[] = {
	NV_PA_APB_UARTA_BASE,
	NV_PA_APB_UARTB_BASE,
	NV_PA_APB_UARTD_BASE,
	0
};

static void board_voltage_init(void);
#ifdef CONFIG_TEGRA3
static void clock_init_misc(void);
static void enable_clock(enum periph_id pid, int src);
#endif /* CONFIG_TEGRA3 */
static void enable_uart(enum periph_id pid);
static void init_uarts(const void *blob);
#ifdef CONFIG_TEGRA_MMC
static void pin_mux_mmc(void);
#endif
static void pin_mux_uart(int uart_ids);
#ifdef CONFIG_TEGRA3
static void pinmux_init(void);
#endif
static void power_det_init(void);
static void send_output_with_pllp(ulong pllp_rate, const char *str);

#ifdef CONFIG_BOARD_EARLY_INIT_F
int board_early_init_f(void)
{
	ulong pllp_rate = 216000000;	/* default PLLP clock rate */

	/* Initialize essential common plls */
	pllp_rate = fdt_decode_clock_rate(gd->blob, "pllp", pllp_rate);
	clock_early_init(pllp_rate);

#ifdef CONFIG_TEGRA3
	pinmux_init();
#endif

#ifndef CONFIG_DELAY_CONSOLE
	init_uarts(gd->blob);
#endif

#ifdef CONFIG_TEGRA3
	/* Initialize misc clocks for kernel booting */
	clock_init_misc();
#endif

#ifdef CONFIG_VIDEO_TEGRA
	/* Get LCD panel size */
	lcd_early_init(gd->blob);
#endif

	return 0;
}
#endif /* CONFIG_BOARD_EARLY_INIT_F */

#define GENERATE_FUSE_DEV_INFO 0
static TrdxBootDevice board_get_current_bootdev(void)
{
	unsigned reg;
#if GENERATE_FUSE_DEV_INFO	
	unsigned reg1 = 0;
	unsigned reg2;
#endif /* GENERATE_FUSE_DEV_INFO */
	unsigned strap_select;
	unsigned skip_strap;
	unsigned fuse_select;
#if GENERATE_FUSE_DEV_INFO
	unsigned fuse_device_info;
	unsigned sdmmc_instance;
#endif /* GENERATE_FUSE_DEV_INFO */
	TrdxBootDevice boot_device;

	//get the latched strap pins, bit [26:29]
	reg = readl( ((unsigned *)NV_APB_MISC_BASE) + APB_MISC_PP_STRAPPING_OPT_A_0);
/*
	printf("Strappings Reg 0x%x, BootSelect 0x%x, Recovery %x, NorBoot %x, JTAG %x, MIO_WIDTH %x, RAM_Code %x, NOR_Width %x\n",
			reg, (reg>>26)&0xf, (reg>>25)&0x1, (reg>>24)&0x1, (reg>>22)&0x3, (reg>>8)&0x1, (reg>>4)&0xf, (reg)&0x1);
*/
	strap_select = (reg>>26)&0xf;

#if 0 //Max: this does not work, there is more to read fuses than that
	//check if we can access BIT /tegra/core/include/nvbit.h et.al.

	clock_enable(PERIPH_ID_FUSE);
	//make fuses visible
	reg = readl( ((unsigned *)CLK_RST_BASE) + CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
	reg = reg | (1<<28);
	writel(reg, ((unsigned *)CLK_RST_BASE) + CLK_RST_CONTROLLER_MISC_CLK_ENB_0 );

	printf("1");
	reg = readl( ((unsigned *)NV_ADDRESS_MAP_FUSE_BASE) + FUSE_RESERVED_SW_0);
	printf("2");
	reg1 = readl( ((unsigned *)NV_ADDRESS_MAP_FUSE_BASE) + FUSE_BOOT_DEVICE_INFO_0);
	printf("3");

	//make fuses invisible
	reg2 = readl( ((unsigned *)CLK_RST_BASE) + CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
	reg2 = reg2 & ~(1<<28);
	writel(reg2, ((unsigned *)CLK_RST_BASE) + CLK_RST_CONTROLLER_MISC_CLK_ENB_0 );

	clock_disable(PERIPH_ID_FUSE);
#else
#ifdef CONFIG_TEGRA3
	//simulate a T30 fuse setting
	reg = NvBootFuseBootDevice_Sdmmc;
#else /* CONFIG_TEGRA3 */
	//simulate a T20 fuse setting
	reg = NvBootFuseBootDevice_NandFlash;
#endif /* CONFIG_TEGRA3 */
#endif
	//get the fuse 'SKIP_DEV_SEL_STRAPS', bit 3
	skip_strap = (reg & 8)>>3;
	//get the fuse 'BOOT_DEV_SEL', bit [0:2]
	fuse_select = reg & 7;

	if(skip_strap || strap_select == NvStrapDevSel_UseFuses)
	{
		printf("Using fuses, %u\n", fuse_select);
		//getting fuse device info and sdmmc instance, bit 7 of fuse_device info
#if GENERATE_FUSE_DEV_INFO
		fuse_device_info = reg1 & 0x3fff;
		sdmmc_instance = ((reg1 & 0x80)==0x80) ? 2 : 3;
#endif /* GENERATE_FUSE_DEV_INFO */
		switch(fuse_select)
		{
		case NvBootFuseBootDevice_Sdmmc:
			boot_device = Trdx_BootDevice_eMMC;
			break;
		case NvBootFuseBootDevice_NandFlash:
			boot_device = Trdx_BootDevice_NandFlash;
			break;
		default:
			boot_device = Trdx_BootDevice_Unknown;
			break;
		}
	}
	else
	{
		/* printf("Using straps, %u\n", strap_select);*/
#if GENERATE_FUSE_DEV_INFO
		sdmmc_instance = 3;
#endif
		switch(strap_select)
		{
		case NvStrapDevSel_Emmc_Primary_x4:
		case NvStrapDevSel_Emmc_Secondary_x4:
		case NvStrapDevSel_Emmc_Secondary_x8:
		case NvStrapDevSel_Esd_x4:
			boot_device = Trdx_BootDevice_extSD;
			break;
		case NvStrapDevSel_Nand:
		case NvStrapDevSel_Nand_42nm_x8:
			boot_device = Trdx_BootDevice_NandFlash;
			break;
		default:
			boot_device = Trdx_BootDevice_Unknown;
			break;
		}
	}
	/*
	if(boot_device < sizeof(sTrdxBootDeviceStr)/sizeof(sTrdxBootDeviceStr[0]))
	{
		printf("%s is used\n",sTrdxBootDeviceStr[boot_device]);
	}
	*/
	return boot_device;
}

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
#ifdef CONFIG_VIDEO_TEGRA
	tegra_lcd_check_next_stage(gd->blob, 0);
#endif

#ifdef CONFIG_DELAY_CONSOLE
	init_uarts(gd->blob);
#endif

	/* Do clocks and UART first so that printf() works */
	clock_init();

#ifdef CONFIG_USB_EHCI_TEGRA
	board_usb_init(gd->blob);
#endif

	clock_verify();

	power_det_init();

	(void)board_get_current_bootdev();

#ifdef CONFIG_TEGRA_I2C
	/* Ramp up the core voltage, then change to full CPU speed */
	i2c_init_board();
#endif

#ifdef CONFIG_TEGRA_CLOCK_SCALING
	pmu_set_nominal();
	arch_full_speed();
#endif /* CONFIG_TEGRA_CLOCK_SCALING */

	/* board id for Linux */
	gd->bd->bi_arch_number = fdt_decode_get_machine_arch_id(gd->blob);
	if (gd->bd->bi_arch_number == -1U)
		printf("Warning: No /config/machine-arch-id defined in fdt\n");

#ifdef CONFIG_TEGRA_CLOCK_SCALING
	board_emc_init();
#endif

	board_voltage_init();

#ifdef CONFIG_HW_WATCHDOG
	i2c_is_initialized = 1;
#endif

#ifdef CONFIG_TEGRA_LP0
	/* prepare the WB code to LP0 location */
//ToDo: determine LP0 address dynamically
	warmboot_prepare_code(TEGRA_LP0_ADDR, TEGRA_LP0_SIZE);
#endif /* CONFIG_TEGRA_LP0 */

	/* boot param addr */
	gd->bd->bi_boot_params = (NV_PA_SDRAM_BASE + 0x100);

	return 0;
}

#ifdef BOARD_LATE_INIT
int board_late_init(void)
{
#ifdef CONFIG_CMD_NAND
	char env_str[256 ];
#endif

#if defined(CONFIG_CMD_NAND) || defined(CONFIG_TRDX_CFG_BLOCK)
	int i;
#endif

#ifdef CONFIG_TRDX_CFG_BLOCK
	char *addr_str, *end;
	unsigned char bi_enetaddr[6]	= {0, 0, 0, 0, 0, 0}; /* Ethernet address */
	unsigned char *mac_addr;
	unsigned char mac_addr00[6]	= {0, 0, 0, 0, 0, 0};

	size_t size			= 4096;
	unsigned char toradex_oui[3]	= { 0x00, 0x14, 0x2d };
	int valid			= 0;

        int ret;
#endif /* CONFIG_TRDX_CFG_BLOCK */

#ifdef CONFIG_VIDEO_TEGRA
	/* Make sure we finish initing the LCD */
	tegra_lcd_check_next_stage(gd->blob, 1);
#endif

#ifdef CONFIG_TRDX_CFG_BLOCK
	/* Allocate RAM area for config block */
	config_block = malloc(size);
	if (!config_block) {
		printf("Not enough malloc space available!\n");
		return -1;
	}

	/* Clear it */
	memset((void *)config_block, 0, size);

	/* Read production parameter config block */
#ifdef CONFIG_CMD_NAND
	ret = nand_read_skip_bad(&nand_info[0], gd->conf_blk_offset, &size,
			(unsigned char *) config_block);
#endif
	/* Check validity */
	if ((ret == 0) && (size > 0)) {
		mac_addr = config_block + 8;
		if (!(memcmp(mac_addr, toradex_oui, 3))) {
			valid = 1;
		}
	}

	/* Check validity */
	if (!valid) {
		printf("Missing Colibri config block\n");
		memset((void *)config_block, 0, size);
	} else {
		/* Get MAC address from environment */
		if ((addr_str = getenv("ethaddr")) != NULL) {
			for (i = 0; i < 6; i++) {
				bi_enetaddr[i] = addr_str ? simple_strtoul(addr_str, &end, 16) : 0;
				if (addr_str) {
					addr_str = (*end) ? end + 1 : end;
				}
			}
		}

		/* Set Ethernet MAC address from config block if not already set */
		if (memcmp(mac_addr00, bi_enetaddr, 6) == 0) {
			sprintf(env_str, "%02x:%02x:%02x:%02x:%02x:%02x",
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5]);
			setenv("ethaddr", env_str);
#ifndef CONFIG_ENV_IS_NOWHERE
			saveenv();
#endif
		}
	}

	/* Default memory arguments */
	if (!getenv("memargs")) {
#ifndef CONFIG_TEGRA3
		if (gd->ram_size == 0x10000000) {
			/* 256 MB */
			setenv("memargs", "mem=148M@0M fbmem=12M@148M nvmem=96M@160M");
		} else {
			/* 512 MB */
			setenv("memargs", "mem=372M@0M fbmem=12M@372M nvmem=128M@384M");
		}
#endif /* !CONFIG_TEGRA3 */
	}

#endif /* CONFIG_TRDX_CFG_BLOCK */

#ifdef CONFIG_CMD_NAND
	/* set the nand kernel offset */
	if (!getenv("lnxoffset")) {
		sprintf(env_str, "0x%x", (unsigned)(gd->kernel_offset));
		setenv("lnxoffset", env_str);
	}

	/* set the mtdparts string */
	if (!getenv("mtdparts")) {
		sprintf(env_str, "mtdparts=tegra_nand:");
		i = strlen(env_str);
		nvtegra_mtdparts_string(env_str + i, sizeof(env_str) - i);

		setenv("mtdparts", env_str);
	}
#endif /* CONFIG_CMD_NAND */

	return 0;
}
#endif /* BOARD_LATE_INIT */

#ifdef CONFIG_TEGRA_MMC
/* this is a weak define that we are overriding */
int board_mmc_init(bd_t *bd)
{
	debug("board_mmc_init called\n");

	/* Enable muxes, etc. for SDMMC controllers */
	pin_mux_mmc();

	tegra_mmc_init(gd->blob);

	return 0;
}
#endif /* CONFIG_TEGRA_MMC */

/*
 * This is called when we have no console. About the only reason that this
 * happen is if we don't have a valid fdt. So we don't know what kind of
 * Tegra board we are. We blindly try to print a message every which way we
 * know.
 */
void board_panic_no_console(const char *str)
{
	/* We don't know what PLLP to use, so try both */
	send_output_with_pllp(216000000, str);
	send_output_with_pllp(408000000, str);
}

/*
 * MK: Do I2C/PMU writes to set nominal voltages for Colibri_T30
 *
 */
static void board_voltage_init(void)
{
#ifdef CONFIG_TEGRA3
	uchar reg, data_buffer[1];
	int i;

	i2c_set_bus_num(0);		/* PMU is on bus 0 */

	//switch v-core to 1.2V
	data_buffer[0] = VDD_CORE_NOMINAL_T30; 
	reg = PMU_CORE_VOLTAGE_DVFS_REG;
	for (i = 0; i < MAX_I2C_RETRY; ++i) {
		if (!i2c_write(PMU_CORE_I2C_ADDRESS, reg, 1, data_buffer, 1))
			break;
		udelay(100);
	}

#ifndef CONFIG_HW_WATCHDOG
	//disable watchdog if enabled
	reg = 0x69;
	for (i = 0; i < MAX_I2C_RETRY; ++i) {
		if (!i2c_read(PMU_I2C_ADDRESS, reg, 1, data_buffer, 1))
			break;
		udelay(100);
	}
	if(data_buffer[0] & 0x7) {
		data_buffer[0] = 0x10; /* kick the dog once before disabling it or disabling will fail */
		reg = 0x54;
		for (i = 0; i < MAX_I2C_RETRY; ++i) {
			if (!i2c_write(PMU_I2C_ADDRESS, reg, 1, data_buffer, 1))
				break;
			udelay(100);
		}
		data_buffer[0] = 0xf0; /* at least one of the reserved bits must be '1' or disabling will fail  */
		reg = 0x69;
		for (i = 0; i < MAX_I2C_RETRY; ++i) {
			if (!i2c_write(PMU_I2C_ADDRESS, reg, 1, data_buffer, 1))
				break;
			udelay(100);
		}
	}
#endif /* CONFIG_HW_WATCHDOG */
#endif /* CONFIG_TEGRA3 */
}

#ifdef CONFIG_TEGRA3
/* Init misc clocks for kernel booting */
static void clock_init_misc(void)
{
	/* 0 = PLLA_OUT0, -1 = CLK_M (default) */
	enable_clock(PERIPH_ID_I2S0, -1);
	enable_clock(PERIPH_ID_I2S1, 0);
	enable_clock(PERIPH_ID_I2S2, 0);
	enable_clock(PERIPH_ID_I2S3, 0);
	enable_clock(PERIPH_ID_I2S4, -1);
	enable_clock(PERIPH_ID_SPDIF, -1);
}
#endif /* CONFIG_TEGRA3 */

/*
 * Routine: clock_init_uart
 * Description: init clock for the UART(s)
 */
static void clock_init_uart(int uart_ids)
{
	if (uart_ids & UARTA)
		enable_uart(PERIPH_ID_UART1);
	if (uart_ids & UARTB)
		enable_uart(PERIPH_ID_UART2);
	if (uart_ids & UARTD)
		enable_uart(PERIPH_ID_UART4);
}

#ifdef CONFIG_TEGRA3
static void enable_clock(enum periph_id pid, int src)
{
	/* Assert reset and enable clock */
	reset_set_enable(pid, 1);
	clock_enable(pid);

	/* Use 'src' if provided, else use default */
	if (src != -1)
		clock_ll_set_source(pid, src);

	/* wait for 2us */
	udelay(2);

	/* De-assert reset */
	reset_set_enable(pid, 0);
}
#endif /* CONFIG_TEGRA3 */

static void enable_uart(enum periph_id pid)
{
	/* Assert UART reset and enable clock */
	reset_set_enable(pid, 1);
	clock_enable(pid);
	clock_ll_set_source(pid, 0);	/* UARTx_CLK_SRC = 00, PLLP_OUT0 */

	/* wait for 2us */
	udelay(2);

	/* De-assert reset to UART */
	reset_set_enable(pid, 0);
}

#ifdef CONFIG_REVISION_TAG
u32 get_board_rev(void)
{
#ifdef BOARD_LATE_INIT
	int i;
	unsigned short major = 0, minor = 0, release = 0;
	size_t size = 4096;

	if(config_block == NULL) {
		return 0;
	}

	/* Parse revision information in config block */
	for (i = 0; i < (size - 8); i++) {
		if (config_block[i] == 0x02 && config_block[i+1] == 0x40 &&
				config_block[i+2] == 0x08) {
			break;
		}
	}

	major = (config_block[i+3] << 8) | config_block[i+4];
	minor = (config_block[i+5] << 8) | config_block[i+6];
	release = (config_block[i+7] << 8) | config_block[i+8];

	/* Check validity */
	if (major)
		return ((major & 0xff) << 8) | ((minor & 0xf) << 4) | ((release & 0xf) + 0xa);
	else
		return 0;
#else
	return 0;
#endif /* BOARD_LATE_INIT */
}
#endif /* CONFIG_REVISION_TAG */

#ifdef CONFIG_SERIAL_TAG
void get_board_serial(struct tag_serialnr *serialnr)
{
#ifdef BOARD_LATE_INIT
	int array[8];
	int i;
	unsigned int serial		= 0;
	unsigned int serial_offset	= 11;

	if(config_block == NULL) {
		serialnr->low = 0;
		serialnr->high = 0;
		return;
	}

	/* Get MAC address from config block */
	memcpy(&serial, config_block + serial_offset, 3);
	serial = ntohl(serial);
	serial >>= 8;

	/* Check validity */
	if (serial) {
		/* Convert to Linux serial number format (hexadecimal coded decimal) */
		i = 7;
		while (serial) {
			array[i--] = serial % 10;
			serial /= 10;
		}
		serial = 0;
		for (i = 0; i < 8; i++) {
			serial += array[i];
			serial *= 16;
		}
		serial /= 16;
	}

	serialnr->low = serial;
#else
	serialnr->low = 0;
#endif /* BOARD_LATE_INIT */
	serialnr->high = 0;
}
#endif /* CONFIG_SERIAL_TAG */

#ifdef CONFIG_HW_WATCHDOG
/*
 * kick watchdog in PMU 
 *
 */
extern void hw_watchdog_reset(void)
{
#ifdef CONFIG_TEGRA3
	uchar reg, data_buffer[1];
	static unsigned long last_kick = 0;
	unsigned long now;
	// only kick the watchdog every 10 seconds, the watchdog timeout is 100s
	now = get_ticks();
	//first kick after 10 seconds
	if( !i2c_is_initialized || (last_kick == 0) ) {
		last_kick = now;
	}
	else if( (signed)(now - last_kick) > (10 * 1000) ) {
		last_kick = now;
		data_buffer[0] = 0x10;
		reg = 0x54;
		i2c_write(PMU_I2C_ADDRESS, reg, 1, data_buffer, 1);
	}
#endif /* CONFIG_TEGRA3 */
}
#endif /* CONFIG_HW_WATCHDOG */

static void init_uarts(const void *blob)
{
	int uart_ids = 0;	/* bit mask of which UART ids to enable */
	struct fdt_uart uart;

	if (!fdt_decode_uart_console(blob, &uart, gd->baudrate))
		uart_ids = 1 << uart.id;

	/* Initialize UART clocks */
	clock_init_uart(uart_ids);

	/* Initialize periph pinmuxes */
#ifdef CONFIG_TEGRA2
	pin_mux_uart(uart_ids);
#endif
}

#ifdef CONFIG_TEGRA_MMC
/*
 * Routine: pin_mux_mmc
 * Description: setup the pin muxes/tristate values for the SDMMC(s)
 */
static void pin_mux_mmc(void)
{
#ifdef CONFIG_TEGRA2
	/* SDMMC4: config 3, x4 on 2nd set of pins */
	pinmux_set_func(PINGRP_ATB, PMUX_FUNC_SDIO4);
	pinmux_set_func(PINGRP_GMA, PMUX_FUNC_SDIO4);

	pinmux_tristate_disable(PINGRP_ATB);
	pinmux_tristate_disable(PINGRP_GMA);
#endif /* CONFIG_TEGRA2 */
}
#endif /* CONFIG_TEGRA_MMC */

/*
 * Routine: pin_mux_uart
 * Description: setup the pin muxes/tristate values for the UART(s)
 */
static void pin_mux_uart(int uart_ids)
{
#ifdef CONFIG_TEGRA2
	if (uart_ids & UARTA) {
		/* Disable UART1 where primary function */
		pinmux_tristate_enable(PINGRP_IRRX);
		pinmux_tristate_enable(PINGRP_IRTX);
		pinmux_set_func(PINGRP_IRRX, PMUX_FUNC_GMI);
		pinmux_set_func(PINGRP_IRTX, PMUX_FUNC_GMI);
		pinmux_tristate_enable(PINGRP_SDB);
		pinmux_tristate_enable(PINGRP_SDD);
		pinmux_set_func(PINGRP_SDB, PMUX_FUNC_PWM);
		pinmux_set_func(PINGRP_SDD, PMUX_FUNC_PWM);

		pinmux_set_func(PINGRP_SDMMC1, PMUX_FUNC_UARTA);
		pinmux_tristate_disable(PINGRP_SDMMC1);
	}
	if (uart_ids & UARTB) {
		pinmux_set_func(PINGRP_UAD, PMUX_FUNC_IRDA);
		pinmux_tristate_disable(PINGRP_UAD);
	}
	if (uart_ids & UARTD) {
		pinmux_set_func(PINGRP_GMC, PMUX_FUNC_UARTD);
		pinmux_tristate_disable(PINGRP_GMC);
	}
#endif	/* CONFIG_TEGRA2 */
}

/*
 * Routine: pinmux_init
 * Description: Do individual peripheral pinmux configs
 */
#ifdef CONFIG_TEGRA3
static void pinmux_init(void)
{
	pinmux_config_table(tegra3_pinmux_common,
				ARRAY_SIZE(tegra3_pinmux_common));

	pinmux_config_table(unused_pins_lowpower,
				ARRAY_SIZE(unused_pins_lowpower));
}
#endif /* CONFIG_TEGRA3 */

/*
 * Routine: power_det_init
 * Description: turn off power detects
 */
static void power_det_init(void)
{
#ifdef CONFIG_TEGRA2
	struct pmc_ctlr *const pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;

	/* turn off power detects */
	writel(0, &pmc->pmc_pwr_det_latch);
	writel(0, &pmc->pmc_pwr_det);
#endif /* CONFIG_TEGRA2 */
}

/**
 * Send out serial output wherever we can.
 *
 * This function produces a low-level panic message, after setting PLLP
 * to the given value.
 *
 * @param pllp_rate	Required PLLP rate (408000000 or 216000000)
 * @param str		String to output
 */
static void send_output_with_pllp(ulong pllp_rate, const char *str)
{
	int uart_ids = UART_ALL;	/* turn it all on! */
	u32 *uart_addr;
	int clock_freq, multiplier, baudrate, divisor;

	clock_early_init(pllp_rate);

	/* Try to enable all possible UARTs */
	clock_init_uart(uart_ids);
	pin_mux_uart(uart_ids);

#ifdef CONFIG_TEGRA3
	/* Until we sort out pinmux, we must do the global Tegra3 init */
	pinmux_init();
#endif

	/*
	 * Now send the string out all the Tegra UARTs. We don't try all
	 * possible configurations, but this could be added if required.
	 */
	clock_freq = pllp_rate;
	multiplier = CONFIG_DEFAULT_NS16550_MULT;
	baudrate = CONFIG_BAUDRATE;
	divisor = (clock_freq + (baudrate * (multiplier / 2))) /
			(multiplier * baudrate);

	for (uart_addr = uart_reg_addr; *uart_addr; uart_addr++) {
		const char *s;

		NS16550_init((NS16550_t)*uart_addr, divisor);
		for (s = str; *s; s++) {
			NS16550_putc((NS16550_t)*uart_addr, *s);
			if (*s == '\n')
				NS16550_putc((NS16550_t)*uart_addr, '\r');
		}
	}
}

/*
 * Routine: timer_init
 * Description: init the timestamp and lastinc value
 */
int timer_init(void)
{
	reset_timer();
	return 0;
}
