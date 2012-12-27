/*
 * Copyright (c) 2010-2011 NVIDIA Corporation
 * With help from the mpc8xxx SPI driver
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

#include <malloc.h>
#include <ns16550.h> /* for NS16550_drain and NS16550_clear */
#include <spi.h>
#include <asm/clocks.h>
#include <asm/io.h>
#include <asm/arch-tegra/bitfield.h>
#include <asm/arch-tegra/clk_rst.h>
#include <asm/arch-tegra/spi.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pinmux.h>
#include "uart-spi-fix.h"

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	/* Tegra2 SPI-Flash - only 1 device ('bus/cs') */
	if (bus > 0 && cs != 0)
		return 0;
	else
		return 1;
}


struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	struct spi_slave *slave;

	if (!spi_cs_is_valid(bus, cs))
		return NULL;

	slave = malloc(sizeof(struct spi_slave));
	if (!slave)
		return NULL;

	slave->bus = bus;
	slave->cs = cs;

	/*
	 * Currently, Tegra2 SFLASH uses mode 0 & a 24MHz clock.
	 * Use 'mode' and 'maz_hz' to change that here, if needed.
	 */

	return slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	free(slave);
}

void spi_init(void)
{
	struct spi_tegra *spi = (struct spi_tegra *)TEGRA_SPI_BASE;
	u32 reg;

	/* Change SPI clock to 24MHz, PLLP_OUT0 source */
	clock_start_periph_pll(PERIPH_ID_SPI1, CLOCK_ID_PERIPH, CLK_24M);

	/* Clear stale status here */
	reg = SPI_STAT_RDY | SPI_STAT_RXF_FLUSH | SPI_STAT_TXF_FLUSH | \
		SPI_STAT_RXF_UNR | SPI_STAT_TXF_OVF;
	writel(reg, &spi->status);
	debug("spi_init: STATUS = %08x\n", readl(&spi->status));

	/*
	 * Use sw-controlled CS, so we can clock in data after ReadID, etc.
	 */

	reg = readl(&spi->command);
	writel(reg | SPI_CMD_CS_SOFT, &spi->command);
	debug("spi_init: COMMAND = %08x\n", readl(&spi->command));

	/*
	 * SPI pins on Tegra2 are muxed - change pinmux last due to UART
	 * issue.
	 */
	pinmux_set_func(PINGRP_GMD, PMUX_FUNC_SFLASH);
	pinmux_tristate_disable(PINGRP_LSPI);

#ifndef CONFIG_SPI_UART_SWITCH
	/*
	 * NOTE:
	 * Only set PinMux bits 3:2 to SPI here on boards that don't have the
	 * SPI UART switch or subsequent UART data won't go out!  See
	 * spi_uart_switch().
	 */
	pinmux_set_func(PINGRP_GMC, PMUX_FUNC_SFLASH);
#endif
}

int spi_claim_bus(struct spi_slave *slave)
{
	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
	/*
	 * We can't release UART_DISABLE and set pinmux to UART4 here since
	 * some code (e,g, spi_flash_probe) uses printf() while the SPI
	 * bus is held. That is arguably bad, but it has the advantage of
	 * already being in the source tree.
	 */
}

void spi_cs_activate(struct spi_slave *slave)
{
	struct spi_tegra *spi = (struct spi_tegra *)TEGRA_SPI_BASE;
	u32 val;

	spi_enable();

	/* CS is negated on Tegra, so drive a 1 to get a 0 */
	val = readl(&spi->command);
	writel(val | SPI_CMD_CS_VAL, &spi->command);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	struct spi_tegra *spi = (struct spi_tegra *)TEGRA_SPI_BASE;
	u32 val;

	/* CS is negated on Tegra, so drive a 0 to get a 1 */
	val = readl(&spi->command);
	writel(val & ~SPI_CMD_CS_VAL, &spi->command);
}

/* Helper function to calculate the clock cycle in this round.
 * Also updates the byte count remaining to be used this round.
 *
 * For example, we want to write 6 bytes to SPI and then read 5 bytes back.
 *
 * +---+---+---+---+---+---+
 * | W | W | W | W | W | W |
 * +---+---+---+---+---+---+---+---+---+---+---+
 *                         | R | R | R | R | R |
 *                         +---+---+---+---+---+
 * |<-- round 0 -->|
 *                 |<-- round 1 -->|
 *                                 |<-- round 2 -->|
 *
 * So that the continuous calling this function would get:
 *
 * round| RET| writecnt  readcnt  bytes  to_write  to_read
 * -----+----+---------------------------------------------
 * INIT |    |     6        5
 *    0 |  1 |     2        5       4       4        0
 *    1 |  1 |     0        3       4       2        2
 *    2 |  1 |     0        0       3       0        3
 *    3 |  0 |     -        -       -       -        -
 *
 */
static int next_4_bytes(uint32_t *writecnt, uint32_t *readcnt,
			uint32_t *num_bytes, uint32_t *to_write,
			uint32_t *to_read)
{
        *to_write = min(*writecnt, 4);
        *to_read = min(*readcnt, 4 - *to_write);

        *writecnt -= *to_write;
        *readcnt -= *to_read;

        *num_bytes = *to_write + *to_read;

        if (*num_bytes)
                return 1;  /* need to be called again. */
        else
                return 0;  /* handled write and read requests. */
}

int spi_xfer(struct spi_slave *slave, const void *dout, unsigned int bitsout,
		void *din, unsigned int bitsin)
{
	int retval = 0;
	char *delayed_msg = NULL;
	struct spi_tegra *spi = (struct spi_tegra *)TEGRA_SPI_BASE;
	uint32_t status;
	uint32_t writecnt = (bitsout + 7) / 8;
	uint32_t readcnt = (bitsin + 7) / 8;
	uint32_t to_write, to_read;  /*  byte counts to fill FIFO. */
	uint32_t bytes;  /* byte count to tell SPI controller. */
	uint8_t *writearr = (uint8_t *)dout;
	uint8_t *readarr = (uint8_t *)din;

	/* We don't currently handle sending partial bytes. */
	assert(bitsin % 8 == 0);
	assert(bitsout % 8 == 0);

	writel(readl(&spi->status), &spi->status);
	writel(readl(&spi->command) | SPI_CMD_TXEN | SPI_CMD_RXEN,
		&spi->command);
	spi_cs_activate(slave);

	while (next_4_bytes(&writecnt, &readcnt, &bytes, &to_write, &to_read)) {
		int i;
		uint32_t tmp;
		uint32_t tm;  /* timeout counter */

		/* prepare Tx FIFO */
		for (tmp = 0, i = 0; i < to_write; ++i) {
			tmp |= *writearr++ << ((bytes - i - 1) * 8);
		}
		writel(tmp, &spi->tx_fifo);

		/* Kick the SCLK running: Shift out TX FIFO, and receive RX. */
		writel(readl(&spi->command) & ~SPI_CMD_BIT_LENGTH_MASK,
			&spi->command);
		writel(readl(&spi->command) | (bytes * 8 - 1), &spi->command);
		writel(readl(&spi->command) | SPI_CMD_GO, &spi->command);

		/* Wait for controller completes the task. */
		for (tm = 0; tm < SPI_TIMEOUT; ++tm) {
			if (((status = readl(&spi->status)) &
			    (SPI_STAT_BSY | SPI_STAT_RDY)) == SPI_STAT_RDY)
				break;
			udelay(10);
		}
		writel(readl(&spi->status) | SPI_STAT_RDY, &spi->status);

		/* Since the UART is disabled here, we delay printing the
		 * message until spi_cs_deactivate() is called.
		 */
		if (tm >= SPI_TIMEOUT) {
			static char err[256];
			retval = -1;
			sprintf(err,
				"%s():%d BSY&RDY timeout, status = 0x%08x\n",
				__func__, __LINE__, status);
			delayed_msg = err;
			break;
		}

		/* read RX FIFO */
		tmp = readl(&spi->rx_fifo);
		for (i = 0; i < to_read; ++i) {
			*readarr++ = tmp >> ((to_read - 1 - i) * 8);
		}
	}

	/* Clear write-on-clear status bits */
	writel(readl(&spi->status), &spi->status);

	spi_cs_deactivate(slave);
	if (delayed_msg) {
		printf("%s\n", delayed_msg);
	}

	return retval;
}
