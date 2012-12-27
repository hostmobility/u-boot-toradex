/*
 * ulpi_linux.c
 *
 * this file is ulpi_phy_power_on() function taken from
 * arch/arm/mach-tegra/usb_phy.c linux kernel code, slightly 
 * modified for U-Boot by Ant Micro <www.antmicro.com>
 *
 * Original arch/arm/mach-tegra/usb_phy.c Copyrights:
 *   Copyright (C) 2010 Google, Inc.
 *   Copyright (C) 2010 - 2011 NVIDIA Corporation
 *   Erik Gilling <konkers@google.com>
 *   Benoit Goby <benoit@android.com>
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
#include <asm/io.h>

#define ULPI_VIEWPORT		0x170
#define   ULPI_WAKEUP		(1 << 31)
#define   ULPI_RUN		(1 << 30)
#define   ULPI_RD_RW_WRITE	(1 << 29)
#define   ULPI_RD_RW_READ	(0 << 29)
#define   ULPI_PORT(x)		(((x) & 0x7) << 24)
#define   ULPI_ADDR(x)		(((x) & 0xff) << 16)
#define   ULPI_DATA_RD(x)	(((x) & 0xff) << 8)
#define   ULPI_DATA_WR(x)	(((x) & 0xff) << 0)

#define USB_PORTSC1		0x184
#define   USB_PORTSC1_PTS(x)	(((x) & 0x3) << 30)
#define   USB_PORTSC1_PSPD(x)	(((x) & 0x3) << 26)
#define   USB_PORTSC1_PHCD	(1 << 23)
#define   USB_PORTSC1_WKOC	(1 << 22)
#define   USB_PORTSC1_WKDS	(1 << 21)
#define   USB_PORTSC1_WKCN	(1 << 20)
#define   USB_PORTSC1_PTC(x)	(((x) & 0xf) << 16)
#define   USB_PORTSC1_PP	(1 << 12)
#define   USB_PORTSC1_SUSP	(1 << 7)
#define   USB_PORTSC1_PE	(1 << 2)
#define   USB_PORTSC1_CCS	(1 << 0)

#define USB_SUSP_CTRL		0x400
#define   USB_WAKE_ON_CNNT_EN_DEV	(1 << 3)
#define   USB_WAKE_ON_DISCON_EN_DEV	(1 << 4)
#define   USB_SUSP_CLR		(1 << 5)
#define   USB_PHY_CLK_VALID	(1 << 7)
#define   UTMIP_RESET			(1 << 11)
#define   UHSIC_RESET			(1 << 11)
#define   UTMIP_PHY_ENABLE		(1 << 12)
#define   ULPI_PHY_ENABLE	(1 << 13)
#define   USB_SUSP_SET		(1 << 14)
#define   USB_WAKEUP_DEBOUNCE_COUNT(x)	(((x) & 0x7) << 16)

#define ULPI_TIMING_CTRL_0	0x424
#define   ULPI_OUTPUT_PINMUX_BYP	(1 << 10)
#define   ULPI_CLKOUT_PINMUX_BYP	(1 << 11)

#define ULPI_TIMING_CTRL_1	0x428
#define   ULPI_DATA_TRIMMER_LOAD	(1 << 0)
#define   ULPI_DATA_TRIMMER_SEL(x)	(((x) & 0x7) << 1)
#define   ULPI_STPDIRNXT_TRIMMER_LOAD	(1 << 16)
#define   ULPI_STPDIRNXT_TRIMMER_SEL(x)	(((x) & 0x7) << 17)
#define   ULPI_DIR_TRIMMER_LOAD		(1 << 24)
#define   ULPI_DIR_TRIMMER_SEL(x)	(((x) & 0x7) << 25)

#define uint32_t unsigned int
#define USBADDR2 0xc5004000

#define CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0           			0x10
#define CLK_RST_CONTROLLER_RST_DEVICES_L_0          			0x4

static int wait_for_register(uint32_t addr, uint32_t mask, uint32_t result, int timeout_)
{
	int timeout = timeout_;
	while (timeout-- > 0) {
		if ((readl(addr) & mask) == result) return 0;
		udelay(2);
	}
	return -1;
}

/* ulpi phy power on */
void ulpi_phy_power_on(void)
{
	uint32_t val;
	uint32_t base = USBADDR2;
	uint32_t RegVal = 0;

	udelay(1000);

	/* begin USB reset */

	/* enable USB clock */
	RegVal = readl(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0 + 4);
	RegVal |= (1 << 26);
//	writel(NV_PA_CLK_RST_BASE+CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0+4, RegVal);
//above not working!
*((uint *) (NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0 + 4)) = RegVal;

	/* reset USB */
	RegVal = readl(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_RST_DEVICES_L_0 + 4);
	if (RegVal & (1 << 26)) {
//		writel(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_RST_DEVICES_L_0 + 4, RegVal);
*((uint *) (NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_RST_DEVICES_L_0 + 4)) = RegVal;
		udelay(2);
		RegVal = readl(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_RST_DEVICES_L_0 + 4);
		RegVal &= ~(1 << 26);
//		writel(NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_RST_DEVICES_L_0 + 4, RegVal);
*((uint *) (NV_PA_CLK_RST_BASE + CLK_RST_CONTROLLER_RST_DEVICES_L_0 + 4)) = RegVal;
		udelay(2);
	}

	/* set UTMIP_RESET/UHSIC_RESET */
	RegVal = readl(base + USB_SUSP_CTRL);
	RegVal |= (1 << 11);
	writel(base + USB_SUSP_CTRL, RegVal);

	/* end USB reset */

	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_OUTPUT_PINMUX_BYP | ULPI_CLKOUT_PINMUX_BYP;
	writel(val, base + ULPI_TIMING_CTRL_0);

	val = readl(base + USB_SUSP_CTRL);
	val |= ULPI_PHY_ENABLE;
	writel(val, base + USB_SUSP_CTRL);

	val = 0;
	writel(val, base + ULPI_TIMING_CTRL_1);

	val |= ULPI_DATA_TRIMMER_SEL(4);
	val |= ULPI_STPDIRNXT_TRIMMER_SEL(4);
	val |= ULPI_DIR_TRIMMER_SEL(4);
	writel(val, base + ULPI_TIMING_CTRL_1);

	udelay(10);

	val |= ULPI_DATA_TRIMMER_LOAD;
	val |= ULPI_STPDIRNXT_TRIMMER_LOAD;
	val |= ULPI_DIR_TRIMMER_LOAD;
	writel(val, base + ULPI_TIMING_CTRL_1);

	val = ULPI_WAKEUP | ULPI_RD_RW_WRITE | ULPI_PORT(0);
	writel(val, base + ULPI_VIEWPORT);

	if (wait_for_register(base + ULPI_VIEWPORT, ULPI_WAKEUP, 0, 1000)) {
		printf("%s: timeout waiting for ulpi phy wakeup\n", __func__);
		return;
	}

	/* Fix VbusInvalid due to floating VBUS */
	val = ULPI_RUN | ULPI_RD_RW_WRITE | ULPI_PORT(0) | ULPI_ADDR(0x08) | ULPI_DATA_WR(0x40);
	writel(val, base + ULPI_VIEWPORT);
	if (wait_for_register(base + ULPI_VIEWPORT, ULPI_RUN, 0, 1000)) {
		printf("%s: timeout accessing ulpi phy\n", __func__);
		return;
	}
	val = ULPI_RUN | ULPI_RD_RW_WRITE | ULPI_PORT(0) | ULPI_ADDR(0x0B) | ULPI_DATA_WR(0x80);
	writel(val, base + ULPI_VIEWPORT);
	if (wait_for_register(base + ULPI_VIEWPORT, ULPI_RUN, 0, 1000)) {
		printf("%s: timeout accessing ulpi phy\n", __func__);
		return;
	}

	val = readl(base + USB_PORTSC1);
	val |= USB_PORTSC1_WKOC | USB_PORTSC1_WKDS | USB_PORTSC1_WKCN;
	writel(val, base + USB_PORTSC1);

	val = readl(base + USB_SUSP_CTRL);
	val |= USB_SUSP_CLR;
	writel(val, base + USB_SUSP_CTRL);
	udelay(100);

	val = readl(base + USB_SUSP_CTRL);
	val &= ~USB_SUSP_CLR;
	writel(val, base + USB_SUSP_CTRL);
}
