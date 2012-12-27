/*
* (C) Copyright 2010-2011
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
#include <asm/arch/clock.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/tegra.h>
#include <asm/arch-tegra/ap20.h>
#include <asm/arch-tegra/bitfield.h>
#include <asm/arch-tegra/clk_rst.h>
#include <asm/arch-tegra/flow.h>
#include <asm/arch-tegra/fuse.h>
#include <asm/arch-tegra/i2c.h>
#include <asm/arch-tegra/pmc.h>
#include <asm/arch-tegra/scu.h>
#include <asm/arch-tegra/warmboot.h>

struct clk_pll_table {
	u16		n;
	u16		m;
	u8		p;
	u8		cpcon;
};

/* ~0=uninitialized/unknown, 0=false, 1=true */
uint32_t is_tegra_processor_reset = 0xffffffff;

/*
 * Timing tables for each SOC for all four oscillator options.
 */
static struct clk_pll_table tegra_pll_x_table[TEGRA_SOC_COUNT]
						[CLOCK_OSC_FREQ_COUNT] = {
	/* T20: 1 GHz */
	{{ 1000, 13, 0, 12},	/* OSC 13M */
	 { 625,  12, 0, 8},	/* OSC 19.2M */
	 { 1000, 12, 0, 12},	/* OSC 12M */
	 { 1000, 26, 0, 12},	/* OSC 26M */
	},

	/* T25: 1.2 GHz */
	{{ 923, 10, 0, 12},
	 { 750, 12, 0, 8},
	 { 600,  6, 0, 12},
	 { 600, 13, 0, 12},
	},

	/* T30: 1.5 GHz with slower (216MHz) PLLP */
	{{ 0xd8,  13, 1, 8},
	 { 0xb4,  22, 1, 4},
	 { 0x1b0, 12, 1, 8},
	 { 0xd8,  26, 1, 8},
	},

	/* T30: 1.5 GHz with 408MHz PLLP */
	{{ 0x198, 13, 0, 8},
	 { 0x154, 22, 0, 4},
	 { 0x198, 12, 0, 8},
	 { 0x198, 26, 0, 8},
	},

	/* TEGRA_SOC2_SLOW: 312 MHz */
	{{ 312, 13, 0, 12},	/* OSC 13M */
	 { 260, 16, 0, 8},	/* OSC 19.2M */
	 { 312, 12, 0, 12},	/* OSC 12M */
	 { 312, 26, 0, 12},	/* OSC 26M */
	},
};

enum tegra_family_t {
	TEGRA_FAMILY_T2x,
	TEGRA_FAMILY_T3x,
};

#define GP_HIDREV	0x804

int tegra_get_chip_type(void)
{
	struct fuse_regs *fuse = (struct fuse_regs *)NV_PA_FUSE_BASE;
	uint tegra_sku_id;

	tegra_sku_id = readl(&fuse->sku_info) & 0xff;

	switch (tegra_sku_id) {
	case SKU_ID_T20:
		return TEGRA_SOC_T20;
	case SKU_ID_T25SE:
	case SKU_ID_AP25:
	case SKU_ID_T25:
	case SKU_ID_AP25E:
	case SKU_ID_T25E:
		return TEGRA_SOC_T25;
	case SKU_ID_T30:
		/*
		 * T30 has two options. We will return TEGRA_SOC_T30 until
		 * we have the fdt set up when it may change to
		 * TEGRA_SOC_T30_408MHZ depending on what we set PLLP to.
		 */
		if (clock_get_rate(CLOCK_ID_PERIPH) == 408000000)
			return TEGRA_SOC_T30_408MHZ;
		else
			return TEGRA_SOC_T30;

	default:
		/* unknown sku id */
		return TEGRA_SOC_UNKNOWN;
	}
}

static enum tegra_family_t ap20_get_family(void)
{
	u32 reg, chip_id;

	reg = readl(NV_PA_APB_MISC_BASE + GP_HIDREV);

	chip_id = reg >> 8;
	chip_id &= 0xff;
	if (chip_id == 0x30)
		return TEGRA_FAMILY_T3x;
	else
		return TEGRA_FAMILY_T2x;
}

int ap20_get_num_cpus(void)
{
	return ap20_get_family() == TEGRA_FAMILY_T3x ? 4 : 2;
}

/* Returns 1 if the current CPU executing is a Cortex-A9, else 0 */
int ap20_cpu_is_cortexa9(void)
{
	u32 id = readb(NV_PA_PG_UP_BASE + PG_UP_TAG_0);
	return id == (PG_UP_TAG_0_PID_CPU & 0xff);
}

static void adjust_pllp_out_freqs(void)
{
	struct clk_rst_ctlr *clkrst = (struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	struct clk_pll *pll = &clkrst->crc_pll[CLOCK_ID_PERIPH];
	u32 reg;

	/* Set T30 PLLP_OUT1, 2, 3 & 4 freqs to 9.6, 48, 102 & 204MHz */
	reg = readl(&pll->pll_out);	/* OUTA, contains OUT2 / OUT1 */
	reg |= (IN_408_OUT_48_DIVISOR << PLLP_OUT2_RATIO) | PLLP_OUT2_OVR
		| (IN_408_OUT_9_6_DIVISOR << PLLP_OUT1_RATIO) | PLLP_OUT1_OVR;
	writel(reg, &pll->pll_out);

	reg = readl(&pll->pll_out_b);	/* OUTB, contains OUT4 / OUT3 */
	reg |= (IN_408_OUT_204_DIVISOR << PLLP_OUT4_RATIO) | PLLP_OUT4_OVR
		| (IN_408_OUT_102_DIVISOR << PLLP_OUT3_RATIO) | PLLP_OUT3_OVR;
	writel(reg, &pll->pll_out_b);
}

static int pllx_set_rate(struct clk_pll_simple *pll , u32 divn, u32 divm, u32 divp,
			 u32 cpcon)
{
	u32 reg;

	reg = readl(&pll->pll_base);
	/* Set m, n and p to PLLX_BASE and clear bypass */
	bf_update(PLL_DIVM, reg, divm);
	bf_update(PLL_DIVN, reg, divn);
	bf_update(PLL_DIVP, reg, divp);
	bf_update(PLL_BYPASS, reg, 0);	/* Disable BYPASS */
	writel(reg, &pll->pll_base);

	/* Set cpcon to PLLX_MISC */
	reg = bf_pack(PLL_CPCON, cpcon);
	writel(reg, &pll->pll_misc);

	reg = readl(&pll->pll_base);
	/* Enable PLLX if not enabled */
	if (!bf_unpack(PLL_ENABLE, reg)) {
		bf_update(PLL_ENABLE, reg, 1);
		writel(reg, &pll->pll_base);
	}

	return 0;
}

void ap20_init_pllx(int slow)
{
	struct clk_rst_ctlr *clkrst = (struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	struct clk_pll_simple *pll = &clkrst->crc_pll_simple[CLOCK_ID_XCPU - CLOCK_ID_FIRST_SIMPLE];
	int chip_type;
	enum clock_osc_freq osc;
	struct clk_pll_table *sel;

	/* get chip type. If unknown, assign to T20 */
	chip_type = tegra_get_chip_type();
	if (chip_type == TEGRA_SOC_UNKNOWN)
		chip_type = TEGRA_SOC_T20;

	/* slow mode only works on T2x now */
	if (slow && ((chip_type == TEGRA_SOC_T20) ||
		     (chip_type == TEGRA_SOC_T25)))
		chip_type = TEGRA_SOC2_SLOW;

	/* get osc freq */
	osc = clock_get_osc_freq();

	/* set pllx */
	sel = &tegra_pll_x_table[chip_type][osc];
	pllx_set_rate(pll, sel->n, sel->m, sel->p, sel->cpcon);

	/* once we are out of slow mode, set up the T30 PLLs also */
	if (!slow && chip_type == TEGRA_SOC_T30_408MHZ)
		adjust_pllp_out_freqs();
}

static void enable_cpu_clock(int enable)
{
	struct clk_rst_ctlr *clkrst = (struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	u32 clk;

	/*
	 * NOTE:
	 * Regardless of whether the request is to enable or disable the CPU
	 * clock, every processor in the CPU complex except the master (CPU 0)
	 * will have it's clock stopped because the AVP only talks to the
	 * master. The AVP does not know (nor does it need to know) that there
	 * are multiple processors in the CPU complex.
	 */

	if (enable) {
		/*
		 * Initialize PLLX to a safe speed, as we are running at
		 * a lower voltage for now until we get i2c up in board_init().
		 */
		ap20_init_pllx(1);

		/* Wait until all clocks are stable */
		udelay(PLL_STABILIZATION_DELAY);

		writel(CCLK_BURST_POLICY, &clkrst->crc_cclk_brst_pol);
		writel(SUPER_CCLK_DIVIDER, &clkrst->crc_super_cclk_div);
	}

	/*
	 * Read the register containing the individual CPU clock enables and
	 * always stop the clock to CPU 1.
	 */
	clk = readl(&clkrst->crc_clk_cpu_cmplx);
	clk |= bf_pack(CPU1_CLK_STP, 1);

	/* Stop/Unstop the CPU clock */
	bf_update(CPU0_CLK_STP, clk, enable == 0);
	writel(clk, &clkrst->crc_clk_cpu_cmplx);

	clock_enable(PERIPH_ID_CPU);
}

static int is_cpu_powered(void)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;

	return (readl(&pmc->pmc_pwrgate_status) & CPU_PWRED) ? 1 : 0;
}

static void remove_cpu_io_clamps(void)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	u32 reg;

	/* Remove the clamps on the CPU I/O signals */
	reg = readl(&pmc->pmc_remove_clamping);
	reg |= CPU_CLMP;
	writel(reg, &pmc->pmc_remove_clamping);

	/* Give I/O signals time to stabilize */
	udelay(IO_STABILIZATION_DELAY);
}

static void powerup_cpu(void)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	u32 reg;
	int timeout = IO_STABILIZATION_DELAY;

	if (!is_cpu_powered()) {
		/* Toggle the CPU power state (OFF -> ON) */
		reg = readl(&pmc->pmc_pwrgate_toggle);
		reg &= PARTID_CP;
		reg |= START_CP;
		writel(reg, &pmc->pmc_pwrgate_toggle);

		/* Wait for the power to come up */
		while (!is_cpu_powered()) {
			if (timeout-- == 0)
				printf("CPU failed to power up!\n");
			else
				udelay(10);
		}

		/*
		 * Remove the I/O clamps from CPU power partition.
		 * Recommended only on a Warm boot, if the CPU partition gets
		 * power gated. Shouldn't cause any harm when called after a
		 * cold boot according to HW, probably just redundant.
		 */
		remove_cpu_io_clamps();
	}
}

static void enable_cpu_power_rail(enum tegra_family_t family)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	u32 reg;

	reg = readl(&pmc->pmc_cntrl);
	reg |= CPUPWRREQ_OE;
	writel(reg, &pmc->pmc_cntrl);

	if (family == TEGRA_FAMILY_T3x) {
		/*
		 * TODO(sjg):
		 * For now we do this here. We need to find out what this is
		 * doing, tidy up the code and find a better place for it.
		 */
		/* Write to i2c addr 2d, 2 byte data length, i.e. the PMIC */
		tegra_i2c_ll_write_addr(0x005a, 0x0002);
		/* Write 28 then 23, Register 0x28, 0x23: VDDCtrl Voltage to
		   1000mV */
		tegra_i2c_ll_write_data(0x2328, 0x0a02);
		udelay(1000);
		/* Write 27 then 1, Register 0x27, 1: VDDCtrl Voltage On */
		tegra_i2c_ll_write_data(0x0127, 0x0a02);
		udelay(10 * 1000);
	}

	/*
	 * The TI PMU65861C needs a 3.75ms delay between enabling
	 * the power rail and enabling the CPU clock.  This delay
	 * between SM1EN and SM1 is for switching time + the ramp
	 * up of the voltage to the CPU (VDD_CPU from PMU). We use 0xf00 as
	 * is is ARM-friendly (can fit in a single ARMv4T mov immmediate
	 * instruction).
	 */
	udelay(3840);
}

static void reset_A9_cpu(int reset)
{
	/*
	* NOTE:  Regardless of whether the request is to hold the CPU in reset
	*        or take it out of reset, every processor in the CPU complex
	*        except the master (CPU 0) will be held in reset because the
	*        AVP only talks to the master. The AVP does not know that there
	*        are multiple processors in the CPU complex.
	*/
	int mask = crc_rst_cpu | crc_rst_de | crc_rst_debug;
	int num_cpus = ap20_get_num_cpus();
	int cpu;

	/* Hold CPUs 1 onwards in reset, and CPU 0 if asked */
	for (cpu = 1; cpu < num_cpus; cpu++)
		reset_cmplx_set_enable(cpu, mask, 1);
	reset_cmplx_set_enable(0, mask, reset);

	/* Enable/Disable master CPU reset */
	reset_set_enable(PERIPH_ID_CPU, reset);
}

/**
 * The T30 requires some special clock initialization, including setting up
 * the dvc i2c, turning on mselect and selecting the G CPU cluster
 */
void t30_init_clocks(void)
{
#if defined(CONFIG_TEGRA3)
	/*
	 * Sadly our clock functions don't support the V and W clocks of T30
	 * yet, as well as a few other functions, so use low-level register
	 * access for now. This eventual removable of low-level code from
	 * ap20.c is the same process we went through for T20.
	 */
	struct clk_rst_ctlr *clkrst =
			(struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	struct flow_ctlr *flow = (struct flow_ctlr *)NV_PA_FLOW_BASE;
	u32 val;

	/* Set active CPU cluster to G */
	clrbits_le32(flow->control, 1 << 0);

	/*
	 * Switch system clock to PLLP_OUT4 (108 MHz), AVP will now run
	 * at 108 MHz. This is glitch free as only the source is changed, no
	 * special precaution needed.
	 */
	val = (SCLK_SOURCE_PLLP_OUT4 << SCLK_SWAKEUP_FIQ_SOURCE_SHIFT) |
		(SCLK_SOURCE_PLLP_OUT4 << SCLK_SWAKEUP_IRQ_SOURCE_SHIFT) |
		(SCLK_SOURCE_PLLP_OUT4 << SCLK_SWAKEUP_RUN_SOURCE_SHIFT) |
		(SCLK_SOURCE_PLLP_OUT4 << SCLK_SWAKEUP_IDLE_SOURCE_SHIFT) |
		(SCLK_SYS_STATE_RUN << SCLK_SYS_STATE_SHIFT);
	writel(val, &clkrst->crc_sclk_brst_pol);

	writel(SUPER_SCLK_ENB_MASK, &clkrst->crc_super_sclk_div);

	val = (0 << CLK_SYS_RATE_HCLK_DISABLE_SHIFT) |
		(1 << CLK_SYS_RATE_AHB_RATE_SHIFT) |
		(0 << CLK_SYS_RATE_PCLK_DISABLE_SHIFT) |
		(0 << CLK_SYS_RATE_APB_RATE_SHIFT);
	writel(val, &clkrst->crc_clk_sys_rate);

	/* Put i2c, mselect in reset and enable clocks */
	reset_set_enable(PERIPH_ID_DVC_I2C, 1);
	clock_set_enable(PERIPH_ID_DVC_I2C, 1);
	reset_set_enable(PERIPH_ID_MSELECT, 1);
	clock_set_enable(PERIPH_ID_MSELECT, 1);

	/* Switch MSELECT clock to PLLP (00) */
	clock_ll_set_source(PERIPH_ID_MSELECT, 0);

	/*
	 * Our high-level clock routines are not available prior to
	 * relocation. We use the low-level functions which require a
	 * hard-coded divisor. Use CLK_M with divide by (n + 1 = 17)
	 */
	clock_ll_set_source_divisor(PERIPH_ID_DVC_I2C, 3, 16);

	/*
	 * Give clocks time to stabilize, then take i2c and mselect out of
	 * reset
	 */
	udelay(1000);
	reset_set_enable(PERIPH_ID_DVC_I2C, 0);
	reset_set_enable(PERIPH_ID_MSELECT, 0);
#endif	/* CONFIG_TEGRA3 */
}

static void clock_enable_coresight(int enable)
{
	u32 rst, src;

	clock_set_enable(PERIPH_ID_CORESIGHT, enable);
	reset_set_enable(PERIPH_ID_CORESIGHT, !enable);

	if (enable) {
		/*
		 * Put CoreSight on PLLP_OUT0 (216 MHz) and divide it down by
		 *  1.5, giving an effective frequency of 144MHz.
		 * Set PLLP_OUT0 [bits31:30 = 00], and use a 7.1 divisor
		 *  (bits 7:0), so 00000001b == 1.5 (n+1 + .5)
		 *
		 * Clock divider request for 204MHz would setup CSITE clock as
		 * 144MHz for PLLP base 216MHz and 204MHz for PLLP base 408MHz
		 */
		if (tegra_get_chip_type() == TEGRA_SOC_T30_408MHZ)
			src = CLK_DIVIDER(NVBL_PLLP_KHZ, 204000);
		else
			src = CLK_DIVIDER(NVBL_PLLP_KHZ, 144000);
		clock_ll_set_source_divisor(PERIPH_ID_CSI, 0, src);

		/* Unlock the CPU CoreSight interfaces */
		rst = 0xC5ACCE55;
		writel(rst, CSITE_CPU_DBG0_LAR);
		writel(rst, CSITE_CPU_DBG1_LAR);
	}
}

static void set_cpu_running(int run)
{
	struct flow_ctlr *flow = (struct flow_ctlr *)NV_PA_FLOW_BASE;

	writel(run ? (FLOW_MODE_NONE << 29) : (FLOW_MODE_STOP << 29), &flow->halt_cpu_events);
}

void start_cpu(enum tegra_family_t family, u32 reset_vector)
{
	if (family == TEGRA_FAMILY_T3x)
		t30_init_clocks();

	/* Enable VDD_CPU */
	enable_cpu_power_rail(family);

	if (family == TEGRA_FAMILY_T3x)
		set_cpu_running(0);

	/* Hold the CPUs in reset */
	reset_A9_cpu(1);

	/* Disable the CPU clock */
	enable_cpu_clock(0);

	/* Enable CoreSight */
	clock_enable_coresight(1);

	/*
	 * Set the entry point for CPU execution from reset,
	 *  if it's a non-zero value.
	 */
	if (reset_vector)
		writel(reset_vector, EXCEP_VECTOR_CPU_RESET_VECTOR);

	/* Enable the CPU clock */
	enable_cpu_clock(1);

	/* If the CPU doesn't already have power, power it up */
	powerup_cpu();

	/* Take the CPU out of reset */
	reset_A9_cpu(0);

	if (family == TEGRA_FAMILY_T3x)
		set_cpu_running(1);
}

void halt_avp(void)
{
	for (;;) {
		writel((HALT_COP_EVENT_JTAG | HALT_COP_EVENT_IRQ_1 \
			| HALT_COP_EVENT_FIQ_1 | (FLOW_MODE_STOP<<29)),
			FLOW_CTLR_HALT_COP_EVENTS);
	}
}

void enable_scu(void)
{
	struct scu_ctlr *scu = (struct scu_ctlr *)NV_PA_ARM_PERIPHBASE;
	u32 reg;

	/* If SCU already setup/enabled, return */
	if (readl(&scu->scu_ctrl) & SCU_CTRL_ENABLE)
		return;

	/* Invalidate all ways for all processors */
	writel(0xFFFF, &scu->scu_inv_all);

	/* Enable SCU - bit 0 */
	reg = readl(&scu->scu_ctrl);
	reg |= SCU_CTRL_ENABLE;
	writel(reg, &scu->scu_ctrl);
}

void init_pmc_scratch(void)
{
	struct pmc_ctlr *const pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	int i;

	/* SCRATCH0 is initialized by the boot ROM and shouldn't be cleared */
	for (i = 0; i < 23; i++)
		writel(0, &pmc->pmc_scratch1+i);

	/* ODMDATA is for kernel use to determine RAM size, LP config, etc. */
	writel(CONFIG_SYS_BOARD_ODMDATA, &pmc->pmc_scratch20);

#ifdef CONFIG_TEGRA_LP0
	/* save Sdram params to PMC 2, 4, and 24 for WB0 */
	warmboot_save_sdram_params();
#endif
}

void tegra_start(void)
{
	struct pmux_tri_ctlr *pmt = (struct pmux_tri_ctlr *)NV_PA_APB_MISC_BASE;

	/* If we are the AVP, start up the first Cortex-A9 */
	if (!ap20_cpu_is_cortexa9()) {
		extern void _start(void);

		/* enable JTAG */
		writel(0xC0, &pmt->pmt_cfg_ctl);

		/*
		* If we are ARM7 - give it a different stack. We are about to
		* start up the A9 which will want to use this one.
		*/
		asm volatile("ldr	sp, =%c0\n"
			: : "i"(AVP_EARLY_BOOT_STACK_LIMIT));

		start_cpu(ap20_get_family(), (u32)_start);
		halt_avp();
		/* not reached */
	}

	/* Init PMC scratch memory */
	init_pmc_scratch();

	enable_scu();

	/* enable SMP mode and FW for CPU0, by writing to Auxiliary Ctl reg */
	asm volatile(
		"mrc	p15, 0, r0, c1, c0, 1\n"
		"orr	r0, r0, #0x41\n"
		"mcr	p15, 0, r0, c1, c0, 1\n");

	/* FIXME: should have ap20's L2 disabled too? */

	/* Init is_tegra_processor_reset */
	is_tegra_processor_reset = check_is_tegra_processor_reset();
}

void tegra_update_clocks(void)
{
	/* Enable CoreSight with new clock speed */
	clock_enable_coresight(1);
}
