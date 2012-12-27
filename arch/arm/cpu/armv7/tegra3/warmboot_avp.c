/*
 * (C) Copyright 2010 - 2012
 * NVIDIA Corporation <www.nvidia.com>
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

#include <asm/arch-tegra/clk_rst.h>
#include <asm/arch-tegra/pmc.h>
#include <asm/arch-tegra/flow.h>
#include <asm/arch-tegra/ap20.h>
#include <asm/arch-tegra/power.h>
#include <asm/arch/ahb.h>
#include <asm/arch-tegra/warmboot.h>
#include <asm/arch-tegra/warmboot_avp.h>
#include <asm/arch/clock.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/tegra.h>
#include <asm/arch/sdmmc.h>

#define RESET_CORESIGHT

void wb_start(void)
{
	struct pmux_tri_ctlr *pmt = (struct pmux_tri_ctlr *)NV_PA_APB_MISC_BASE;
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	struct flow_ctlr *flow = (struct flow_ctlr *)NV_PA_FLOW_BASE;
	struct sdmmc_ctlr *sdmmc4 = (struct sdmmc_ctlr *)NV_PA_SDMMC4_BASE;
	struct clk_rst_ctlr *clkrst =
			(struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	struct ahb_ctlr *ahb = (struct ahb_ctlr *)TEGRA2_AHB_BASE;
	struct clk_pll *pllp = &clkrst->crc_pll[CLOCK_ID_PERIPH];
	u32 reg, saved_reg;
	u32 divm, divn;
	u32 cpcon, lfcon;
	u32 base, misc;
	u32 status_mask, toggle, clamp;
	u32 internal_periph_id;

	/* enable JTAG & TBE */
	writel(CONFIG_CTL_TBE | CONFIG_CTL_JTAG, &pmt->pmt_cfg_ctl);

	/* Are we running where we're supposed to be? */
	asm volatile (
		"adr	%0, wb_start;"	/* reg: wb_start address */
		: "=r"(reg)		/* output */
					/* no input, no clobber list */
	);

	if (reg != AP20_WB_RUN_ADDRESS)
		goto do_reset;

	/* Are we running with AVP? */
	if (readl(NV_PA_PG_UP_BASE + PG_UP_TAG_0) != PG_UP_TAG_AVP)
		goto do_reset;

	/*
	 * Save the current setting for CLK_BURST_POLICY and change clock
	 * source to CLKM
	 */
	saved_reg = readl(&clkrst->crc_sclk_brst_pol);

	reg = SCLK_SWAKE_FIQ_SRC_CLKM | SCLK_SWAKE_IRQ_SRC_CLKM |
		SCLK_SWAKE_RUN_SRC_CLKM | SCLK_SWAKE_IDLE_SRC_CLKM |
		(SCLK_SYS_STATE_RUN << SCLK_SYS_STATE_SHIFT);
	writel(reg, &clkrst->crc_sclk_brst_pol);

	/* Update PLLP output dividers for 408 MHz operation */
	/* Set OUT1 to 9.6MHz and OUT2 to 48MHz (based on 408MHz of PLLP) */
	writel(PLLP_408_OUTA, &pllp->pll_out);

	writel(PLLP_408_OUTB, &pllp->pll_out_b);

	/* Set oscillator the drive strength */
	reg = readl(&clkrst->crc_osc_ctrl);
	reg &= ~(OSC_CTRL_XOE | OSC_CTRL_XOFS);
	reg |= OSC_CTRL_XOE_ENABLE  | OSC_CTRL_XOFS_4;
	writel(reg, &clkrst->crc_osc_ctrl);

	/* Set CPCON and enable PLLP lock bit */
	writel(PLLP_MISC_PLLP_CPCON_8 | PLLP_MISC_PLLP_LOCK_ENABLE,
		&pllp->pll_misc);

	/* Find out the current osc frequency */
	reg = readl(&clkrst->crc_osc_ctrl);

	/* Find out the PLLP_BASE value to use */

	/*
	 * Note: can not use switch statement: compiler builds tables on the
	 * local stack. We don't want to use anything on the stack.
	 */
	reg >>= OSC_CTRL_OSC_FREQ_SHIFT;
	if ((reg == OSC_FREQ_OSC12) || (reg == OSC_FREQ_OSC48)) {
		divm = 0x0c;
		divn = 0x198;
	} else if (reg == OSC_FREQ_OSC16P8) {
		divm = 0x0e;
		divn = 0x154;
	} else if ((reg == OSC_FREQ_OSC19P2) || (reg == OSC_FREQ_OSC38P4)) {
		divm = 0x10;
		divn = 0x154;
	} else if (reg == OSC_FREQ_OSC26) {
		divm = 0x1a;
		divn = 0x198;
	} else {
		/*
		 * Unused code in OSC_FREQ is mapped to 13MHz - use 13MHz as
		 * default settings.
		 */
		divm = 0x0d;
		divn = 0x198;
	}

	/* Change PLLP to be 408MHz */
	reg = (divm << PLLP_BASE_PLLP_DIVM_SHIFT) |
			(divn << PLLP_BASE_PLLP_DIVN_SHIFT) |
			PLLP_BASE_OVRRIDE_ENABLE |
			PLLP_BASE_PLLP_ENABLE;
	writel(reg, &pllp->pll_base);

	/* Wait till PLLP locks */
	while (1) {
		reg = readl(&pllp->pll_base);
		if (reg & PLLP_BASE_PLLP_LOCK_LOCK)
			break;
	}

	/*
	 * Wait for 250uS after lock bit is set to make sure pll is stable.
	 * The typical wait time is 300uS. Since we already check the lock
	 * bit, reduce the wait time to 250uS.
	 */
	reg = EVENT_ZERO_VAL_250 | EVENT_USEC | EVENT_MODE_STOP;
	writel(reg, &flow->halt_cop_events);

	/* Restore setting for SCLK_BURST_POLICY */
	writel(saved_reg, &clkrst->crc_sclk_brst_pol);

	/*
	 * Enable the PPSB_STOPCLK feature to allow SCLK to be run at
	 * higher frequencies. See bug 811773.
	 */
	reg = readl(&clkrst->crc_misc_clk_enb);
	reg |= MISC_CLK_ENB_EN_PPSB_STOPCLK_ENABLE;
	writel(reg, &clkrst->crc_misc_clk_enb);

	reg = readl(&ahb->arbitration_xbar_ctrl);
	reg |= ARBITRATION_XBAR_CTRL_PPSB_ENABLE_ENABLE;
	writel(reg, &ahb->arbitration_xbar_ctrl);

#ifdef RESET_CORESIGHT
	/* Assert CoreSight reset */
	writel(SWR_CSITE_RST, &clkrst->crc_rst_dev_ex[TEGRA_DEV_U].set);
#endif

	/*
	 * Halt the G complex CPUs at the flow controller in case the G
	 * complex was running in a uni-processor configuration.
	 */
	writel(EVENT_MODE_STOP, &flow->halt_cpu_events);
	writel(EVENT_MODE_STOP, &flow->halt_cpu1_events);
	writel(EVENT_MODE_STOP, &flow->halt_cpu2_events);
	writel(EVENT_MODE_STOP, &flow->halt_cpu3_events);

	/*
	 * Find out which CPU (LP or G) to wake up. The default setting
	 * in flow controller is to wake up GCPU.
	 *
	 * Select the LP CPU cluster. All accesses to the cluster-dependent
	 * CPU registers (legacy clock enables, resets, burst policy, flow
	 * controller) now refer to the LP CPU.
	 */
	reg = readl(&pmc->pmc_scratch4);
	reg &= CPU_WAKEUP_CLUSTER;
	if (reg)
		setbits_le32(&flow->cluster_control, ACTIVE_LP);

	/* Hold all CPUs in reset. */
	reg = CPU_CMPLX_CPURESET0 | CPU_CMPLX_CPURESET1 |
		CPU_CMPLX_CPURESET2 | CPU_CMPLX_CPURESET3 |
		CPU_CMPLX_DERESET0 | CPU_CMPLX_DERESET1 |
		CPU_CMPLX_DERESET2 | CPU_CMPLX_DERESET3 |
		CPU_CMPLX_DBGRESET0 | CPU_CMPLX_DBGRESET1 |
		CPU_CMPLX_DBGRESET2 | CPU_CMPLX_DBGRESET3;
	writel(reg, &clkrst->crc_cpu_cmplx_set);

	/* Assert CPU complex reset. */
	reg = readl(&clkrst->crc_rst_dev[TEGRA_DEV_L]);
	reg |= CPU_RST;
	writel(reg, &clkrst->crc_rst_dev[TEGRA_DEV_L]);

	/* Program SUPER_CCLK_DIVIDER */
	writel(SUPER_CDIV_ENB, &clkrst->crc_super_cclk_div);

	/* Stop the clock to all CPUs. */
	reg = SET_CPU0_CLK_STP | SET_CPU1_CLK_STP | SET_CPU2_CLK_STP |
		SET_CPU3_CLK_STP;
	writel(reg, &clkrst->crc_clk_cpu_cmplx_set);

	/* Make sure the resets are held for at least 2 microseconds. */
	reg = readl(TIMER_USEC_CNTR);
	while ((int)(readl(TIMER_USEC_CNTR) - reg) < 3)
		{}

	writel(CLK_ENB_CSITE, &clkrst->crc_clk_enb_ex[TEGRA_DEV_U].set);

	/* De-assert CoreSight reset */
	writel(SWR_CSITE_RST, &clkrst->crc_rst_dev_ex[TEGRA_DEV_U].clr);

	/* Unlock debugger access. */
	writel(0xC5ACCE55, CSITE_CPU_DBG0_LAR);

	/* Find out the current osc frequency */
	reg = readl(&clkrst->crc_osc_ctrl);
	reg >>= OSC_CTRL_OSC_FREQ_SHIFT;
	if ((reg == OSC_FREQ_OSC12) || (reg == OSC_FREQ_OSC48)) {
		divm = 0x0c;
		divn = 0x3c0;
		cpcon = 0x0c;
		lfcon = 0x01;
	} else if (reg == OSC_FREQ_OSC16P8) {
		divm = 0x07;
		divn = 0x190;
		cpcon = 0x05;
		lfcon = 0x00;
	} else if ((reg == OSC_FREQ_OSC19P2) || (reg == OSC_FREQ_OSC38P4)) {
		divm = 0x04;
		divn = 0xc8;
		cpcon = 0x03;
		lfcon = 0x00;
	} else if (reg == OSC_FREQ_OSC26) {
		divm = 0x1a;
		divn = 0x3c0;
		cpcon = 0x0c;
		lfcon = 0x01;
	} else {
		/*
		 * Unused code in OSC_FREQ is mapped to 13MHz - use 13MHz as
		 * default settings.
		 */
		divm = 0x0d;
		divn = 0x3c0;
		cpcon = 0x0c;
		lfcon = 0x01;
	}

	base = PLLU_BYPASS_ENABLE | (divn << 8) | (divm << 0);
	writel(base, &clkrst->crc_pll[CLOCK_ID_USB].pll_base);
	misc = (cpcon << 8) | (lfcon << 4);
	writel(misc, &clkrst->crc_pll[CLOCK_ID_USB].pll_misc);

	base &= ~PLLU_BYPASS_ENABLE;
	base |= PLLU_ENABLE;
	writel(base, &clkrst->crc_pll[CLOCK_ID_USB].pll_base);
	misc |= PLLU_LOCK_ENABLE;
	writel(misc, &clkrst->crc_pll[CLOCK_ID_USB].pll_misc);

	/*
	 * Reenable eMMC boot block write protection
	 *   1. Enable SDMMC4 clock (it's OK to leave it on the default clock
	 *      source, CLK_M, since only one register will be written).
	 *   2. Take SDMMC4 controller out of reset.
	 *   3. Set SDMMC4_VENDOR_CLOCK_CNTRL_0_HW_RSTN_OVERRIDE.
	 *   4. Restore SDMMC4 reset state.
	 *   5. Stop the clock to SDMMC4 controller.
	 */
	writel(SET_CLK_ENB_SDMMC4, &clkrst->crc_clk_enb_ex[TEGRA_DEV_L].set);

	saved_reg = readl(&clkrst->crc_rst_dev[TEGRA_DEV_L]);
	reg = CLR_SDMMC4_RST_ENABLE;
	writel(reg, &clkrst->crc_rst_dev_ex[TEGRA_DEV_L].clr);

	reg = readl(sdmmc4->vendor_clk_cntrl);
	reg |= HW_RSTN_OVERRIDE_OVERRIDE;
	writel(reg, &sdmmc4->vendor_clk_cntrl);

	writel(saved_reg, &clkrst->crc_rst_dev[TEGRA_DEV_L]);

	writel(CLR_CLK_ENB_SDMMC4_ENABLE,
		&clkrst->crc_clk_enb_ex[TEGRA_DEV_L].clr);

	/*
	 * Set the CPU reset vector. SCRATCH41 contains the physical
	 * address of the CPU-side restoration code.
	 */
	reg = readl(&pmc->pmc_scratch41);
	writel(reg, EXCEP_VECTOR_CPU_RESET_VECTOR);

	/* Select CPU complex clock source */
	writel(CCLK_PLLP_BURST_POLICY, &clkrst->crc_cclk_brst_pol);

	/* Enable CPU0 clock */
	reg = CPU_CMPLX_CLR_CPU0_CLK_STP;
	writel(reg, &clkrst->crc_clk_cpu_cmplx_clr);

	/* Enable the CPU complex clock */
	writel(CLK_ENB_CPU, &clkrst->crc_clk_enb_ex[TEGRA_DEV_L].set);

	/* Set MSELECT clock source to PLL_P */
	reg = MSELECT_CLK_SRC_PLLP_OUT0;
	internal_periph_id = 1;
	writel(reg, &clkrst->crc_clk_src_vw[internal_periph_id]);

	/* Enable clock to MSELECT */
	writel(SET_CLK_ENB_MSELECT,
		&clkrst->crc_clk_enb_ex_vw[TEGRA_DEV_V].set);

	/* Bring MSELECT out of reset */
	writel(SET_MSELECT_RST_ENABLE,
		&clkrst->crc_rst_dev_ex_vw[TEGRA_DEV_V].clr);

	/*
	 * Find out which CPU (LP or G) to power on
	 * Power up the CPU0 partition if necessary.
	 * status_mask: bit mask for CPU enable in APBDEV_PMC_PWRGATE_STATUS_0
	 * toggle: value to power on cpu
	 * clamp: value to remove clamping to CPU
	 */
	reg = readl(&pmc->pmc_scratch4);
	if (reg & CPU_WAKEUP_CLUSTER) {
		/*
		 * Setup registers for powering up LPCPU
		 * - PWRGATE_STATUS, A9LP
		 * - PWRGATE_TOGGLE, PARTID, A9LP , START, ENABLE
		 * - REMOVE_CLAMPING_CMD, A9LP, ENABLE
		 */
		status_mask = PWRGATE_STATUS_A9LP_ENABLE;
		toggle = PWRGATE_TOGGLE_START | PWRGATE_TOGGLE_PARTID_A9LP;
		clamp = REMOVE_CLAMPING_CMD_A9LP_ENABLE;
	} else {
		/*
		 * Setup registers for powering up GCPU
		 * - PWRGATE_STATUS, CPU
		 * - PWRGATE_TOGGLE, PARTID, CP , START, ENABLE
		 * - REMOVE_CLAMPING_CMD, CPU, ENABLE
		 */
		status_mask = PWRGATE_STATUS_CPU_ENABLE;
		toggle = PWRGATE_TOGGLE_START | PWRGATE_TOGGLE_PARTID_CP;
		clamp = REMOVE_CLAMPING_CMD_CPU_ENABLE;
	}

	reg = readl(&pmc->pmc_pwrgate_status);
	if (!(reg & status_mask))
		writel(toggle, &pmc->pmc_pwrgate_toggle);

	while (!(readl(&pmc->pmc_pwrgate_status) & status_mask))
		{}

	/* Remove the I/O clamps from the CPU0 power partition. */
	writel(clamp, &pmc->pmc_remove_clamping);

	/* Give I/O signals 20ms to stabilize. */
	writel(EVENT_ZERO_VAL_20 | EVENT_MSEC | EVENT_MODE_STOP,
		&flow->halt_cop_events);

	/* Take CPU0 out of reset. */
	writel(CPU_CMPLX_CPURESET0 | CPU_CMPLX_DERESET0 | CPU_CMPLX_DBGRESET0,
		&clkrst->crc_cpu_cmplx_clr);

	/* De-assert CPU complex reset. */
	writel(CPU_RST, &clkrst->crc_rst_dev_ex[TEGRA_DEV_L].clr);

	/* Unhalt the CPU at the flow controller. */
	writel(0, &flow->halt_cpu_events);

	/* avp_resume: no return after the write */
	reg = readl(&clkrst->crc_rst_dev[TEGRA_DEV_L]);
	reg &= ~CPU_RST;
	writel(reg, &clkrst->crc_rst_dev[TEGRA_DEV_L]);

	/* avp_halt: */
	while (1)
		writel(EVENT_MODE_STOP | EVENT_JTAG, &flow->halt_cop_events);

do_reset:
	/*
	 * Execution comes here if something goes wrong. The chip is reset and
	 * a cold boot is performed.
	 */
	while (1)
		writel(SWR_TRIG_SYS_RST, &clkrst->crc_rst_dev[TEGRA_DEV_L]);
}

/*
 * wb_end() is a dummy function, and must be directly following wb_start(),
 * and is used to calculate the size of wb_start().
 */
void wb_end(void)
{
}