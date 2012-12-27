/*
 *  (C) Copyright 2010,2011
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

#include <common.h>
#include <ns16550.h>
#include <asm/clocks.h>
#include <asm/io.h>

/* TBD: bring these over when Tegra3 is ready, then remove these #ifdefs */
#include <asm/arch-tegra/bitfield.h>
#include <asm/arch-tegra/clk_rst.h>
#include <asm/arch-tegra/pmc.h>
#include <asm/arch-tegra/uart.h>
#include <asm/arch-tegra/warmboot.h>
#include <asm/arch/clock.h>
#ifdef CONFIG_TEGRA2
#include <asm/arch/emc.h>
#include <asm/arch/gpio.h>
#endif
#include <asm/arch/pinmux.h>
#include <asm/arch/sys_proto.h>
#ifdef CONFIG_USB_EHCI_TEGRA
#include <asm/arch/usb.h>
#endif
#include <asm/arch/tegra.h>

#ifdef CONFIG_TEGRA_SPI
#include <spi.h>
#endif
#ifdef CONFIG_TEGRA_I2C
#include <i2c.h>
#endif
#include "board.h"
#include "pmu.h"

#ifdef CONFIG_TEGRA_MMC
#include <asm/arch/pmu.h>
#include <mmc.h>
#endif
#ifdef CONFIG_OF_CONTROL
#include <fdt_decode.h>
#include <libfdt.h>
#endif

#ifdef CONFIG_CHROMEOS
#include <chromeos/common.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_TEGRA_CLOCK_SCALING) && !defined(CONFIG_TEGRA_I2C)
#error "tegra: We need CONFIG_TEGRA_I2C to support CONFIG_TEGRA_CLOCK_SCALING"
#endif

#if defined(CONFIG_TEGRA_CLOCK_SCALING) && !defined(CONFIG_TEGRA_PMU)
#error "tegra: We need CONFIG_TEGRA_PMU to support CONFIG_TEGRA_CLOCK_SCALING"
#endif

enum {
	/* UARTs which we can enable */
	UARTA	= 1 << 0,
	UARTB	= 1 << 1,
	UARTD	= 1 << 3,
	UART_ALL	= 0xf
};

#ifndef CONFIG_OF_CONTROL
const struct tegra_sysinfo sysinfo = {
	CONFIG_TEGRA_BOARD_STRING
};
#endif

/*
 * Routine: timer_init
 * Description: init the timestamp and lastinc value
 */
int timer_init(void)
{
	reset_timer();
	return 0;
}

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

/*
 * Routine: pin_mux_uart
 * Description: setup the pin muxes/tristate values for the UART(s)
 */
static void pin_mux_uart(int uart_ids)
{
#if defined(CONFIG_TEGRA2)
	if (uart_ids & UARTA) {
		pinmux_set_func(PINGRP_IRRX, PMUX_FUNC_UARTA);
		pinmux_set_func(PINGRP_IRTX, PMUX_FUNC_UARTA);
		pinmux_tristate_disable(PINGRP_IRRX);
		pinmux_tristate_disable(PINGRP_IRTX);
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

#if defined(CONFIG_TEGRA3)
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
#endif

/*
 * Routine: pin_mux_switches
 * Description: Disable internal pullups for the write protect, SDIO3 write
 * protect and the Google Recovery switch.  All of these switches have external
 * pull ups or pull downs.
 */
static void pin_mux_switches(void)
{
#if defined(CONFIG_TEGRA2)
	/*
	 * TODO(robotboy): Move this to the FDT once there is pin mux support
	 * there.  Currently all Tegra based boards use the same GPIOs for
	 * these switches.
	 */
	pinmux_set_pullupdown(PINGRP_ATD, PMUX_PULL_NORMAL);
#endif	/* CONFIG_TEGRA2 */
}

#ifdef CONFIG_TEGRA_MMC
/*
 * Routine: pin_mux_mmc
 * Description: setup the pin muxes/tristate values for the SDMMC(s)
 */
static void pin_mux_mmc(void)
{
#ifdef CONFIG_TEGRA2
	/* SDMMC4: config 3, x8 on 2nd set of pins */
	pinmux_set_func(PINGRP_ATB, PMUX_FUNC_SDIO4);
	pinmux_set_func(PINGRP_GMA, PMUX_FUNC_SDIO4);
	pinmux_set_func(PINGRP_GME, PMUX_FUNC_SDIO4);

	pinmux_tristate_disable(PINGRP_ATB);
	pinmux_tristate_disable(PINGRP_GMA);
	pinmux_tristate_disable(PINGRP_GME);

	/* SDMMC3: SDIO3_CLK, SDIO3_CMD, SDIO3_DAT[3:0] */
	pinmux_set_func(PINGRP_SDB, PMUX_FUNC_SDIO3);
	pinmux_set_func(PINGRP_SDC, PMUX_FUNC_SDIO3);
	pinmux_set_func(PINGRP_SDD, PMUX_FUNC_SDIO3);

	pinmux_set_func(PINGRP_SLXA, PMUX_FUNC_SDIO3);
	pinmux_set_func(PINGRP_SLXC, PMUX_FUNC_SDIO3);
	pinmux_set_func(PINGRP_SLXD, PMUX_FUNC_SDIO3);
	pinmux_set_func(PINGRP_SLXK, PMUX_FUNC_SDIO3);

	pinmux_tristate_disable(PINGRP_SDC);
	pinmux_tristate_disable(PINGRP_SDD);
	pinmux_tristate_disable(PINGRP_SDB);

	pinmux_tristate_disable(PINGRP_SLXA);
	pinmux_tristate_disable(PINGRP_SLXC);
	pinmux_tristate_disable(PINGRP_SLXD);
	pinmux_tristate_disable(PINGRP_SLXK);
#endif
}
#endif

#ifdef CONFIG_TEGRA3
#include "../cardhu/pinmux-config-common.h"
#endif

/*
 * Routine: pinmux_init
 * Description: Do individual peripheral pinmux configs
 */
static void pinmux_init(void)
{
	pin_mux_switches();

#if defined(CONFIG_TEGRA3)
	pinmux_config_table(tegra3_pinmux_common,
				ARRAY_SIZE(tegra3_pinmux_common));

	pinmux_config_table(unused_pins_lowpower,
				ARRAY_SIZE(unused_pins_lowpower));
#endif
}

static void init_uarts(const void *blob)
{
	int uart_ids = 0;	/* bit mask of which UART ids to enable */
#ifdef CONFIG_OF_CONTROL
	struct fdt_uart uart;

	if (!fdt_decode_uart_console(blob, &uart, gd->baudrate))
		uart_ids = 1 << uart.id;
#else
#ifdef CONFIG_TEGRA2_ENABLE_UARTA
	uart_ids |= UARTA;
#endif
#ifdef CONFIG_TEGRA2_ENABLE_UARTB
	uart_ids |= UARTB;
#endif
#ifdef CONFIG_TEGRA2_ENABLE_UARTD
	uart_ids |= UARTD;
#endif
#endif /* CONFIG_OF_CONTROL */
	/* Initialize UART clocks */
	clock_init_uart(uart_ids);

	/* Initialize periph pinmuxes */
#if defined(CONFIG_TEGRA2)
	pin_mux_uart(uart_ids);
#endif

	/* Initialize periph GPIOs */
#ifdef CONFIG_SPI_UART_SWITCH
	gpio_early_init_uart(blob);
#endif
}

/*
 * Do I2C/PMU writes to bring up SD card bus power
 *
 */
static void board_sdmmc_voltage_init(void)
{
#if defined(CONFIG_TEGRA3) && defined(CONFIG_TEGRA_MMC)
	/*
	 * Voltage for SDMMC on Tegra30 Cardhu variants is on
	 * LDO5 and should be at 3.3.
	 *
	 * TODO(dianders): Should be in device tree.
	 */
	uchar ldo5_to_3_3v = PMU_LDO5_SEL(33) | PMU_LDO5_ON;

	pmu_write(PMU_LDO5_REG, &ldo5_to_3_3v, 1);
#endif
}

/*
 * Routine: power_det_init
 * Description: turn off power detects
 */
static void power_det_init(void)
{
#if defined(CONFIG_TEGRA2)
	struct pmc_ctlr *const pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;

	/* turn off power detects */
	writel(0, &pmc->pmc_pwr_det_latch);
	writel(0, &pmc->pmc_pwr_det);
#endif
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
#ifdef CONFIG_SPI_UART_SWITCH
	gpio_config_uart(gd->blob);
#endif
#ifdef CONFIG_USB_EHCI_TEGRA
	board_usb_init(gd->blob);
#endif
	clock_verify();
#ifdef CONFIG_TEGRA_SPI
	spi_init();
#endif
	power_det_init();

#ifdef CONFIG_TEGRA_I2C
	/* Ramp up the core voltage, then change to full CPU speed */
	i2c_init_board();
#endif

#ifdef CONFIG_TEGRA_CLOCK_SCALING
	pmu_set_nominal();
	arch_full_speed();
#endif

	/* board id for Linux */
#ifdef CONFIG_OF_CONTROL
	gd->bd->bi_arch_number = fdt_decode_get_machine_arch_id(gd->blob);
	if (gd->bd->bi_arch_number == -1U)
		printf("Warning: No /config/machine-arch-id defined in fdt\n");
#else
	gd->bd->bi_arch_number = CONFIG_MACH_TYPE;
#endif

#ifdef CONFIG_TEGRA_CLOCK_SCALING
	board_emc_init();
#endif

#ifdef CONFIG_TEGRA_LP0
	/* prepare the WB code to LP0 location */
	warmboot_prepare_code(TEGRA_LP0_ADDR, TEGRA_LP0_SIZE);
#endif

	board_sdmmc_voltage_init();

	/* boot param addr */
	gd->bd->bi_boot_params = (NV_PA_SDRAM_BASE + 0x100);

#ifdef CONFIG_CHROMEOS
	vbexport_init();
#endif

	return 0;
}

#ifdef CONFIG_BOARD_EARLY_INIT_F

int board_early_init_f(void)
{
	ulong pllp_rate = 216000000;	/* default PLLP clock rate */

	/* Initialize essential common plls */
#ifdef CONFIG_OF_CONTROL
	pllp_rate = fdt_decode_clock_rate(gd->blob, "pllp", pllp_rate);
#endif
	clock_early_init(pllp_rate);

	pinmux_init();
#ifndef CONFIG_DELAY_CONSOLE
	init_uarts(gd->blob);
#endif

#if defined(CONFIG_TEGRA3)
	/* Initialize misc clocks for kernel booting */
	clock_init_misc();
#endif

#ifdef CONFIG_VIDEO_TEGRA
	/* Get LCD panel size */
	lcd_early_init(gd->blob);
#endif

	return 0;
}
#endif	/* EARLY_INIT */

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
	 * Seaboard has a UART switch on PI3. We might be a Seaboard,
	 * so flip it!
	 */
#ifdef CONFIG_SPI_UART_SWITCH
	gpio_direction_output(GPIO_PI3, 0);
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

int board_late_init(void)
{
	/* Make sure we finish initing the LCD */
#ifdef CONFIG_VIDEO_TEGRA
	tegra_lcd_check_next_stage(gd->blob, 1);
#endif
	return 0;
}
