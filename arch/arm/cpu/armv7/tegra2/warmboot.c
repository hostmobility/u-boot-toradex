/*
 * (C) Copyright 2010 - 2011
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
#include <asm/errno.h>

#include <asm/arch-tegra/ap20.h>
#include <asm/arch-tegra/fuse.h>

#include <asm/arch-tegra/bitfield.h>
#include <asm/arch-tegra/clk_rst.h>
#include <asm/arch/clock.h>
#include <asm/arch/emc.h>
#include <asm/arch/gp_padctrl.h>
#include <asm/arch/pinmux.h>
#include <asm/arch-tegra/pmc.h>
#include <asm/arch/sdram_param.h>
#include <asm/arch/tegra.h>
#include <asm/arch-tegra/warmboot.h>

union osc_ctrl_reg {
	struct {
		u32 xoe:1;
		u32 xobp:1;
		u32 reserved0:2;
		u32 xofs:6;
		u32 reserved1:2;
		u32 xods:5;
		u32 reserved2:3;
		u32 oscfi_spare:8;
		u32 pll_ref_div:2;
		u32 osc_freq:2;
	};
	u32 word;
};

union pll_base_reg {
	struct {
		u32 pll_divm:5;
		u32 reserved0:3;
		u32 pll_divn:10;
		u32 reserved1:2;
		u32 pll_divp:3;
		u32 reserved2:4;
		u32 pll_lock:1;
		u32 reserved3:1;
		u32 pll_ref_dis:1;
		u32 pll_enable:1;
		u32 pll_bypass:1;
	};
	u32 word;
};

union pll_misc_reg {
	struct {
		u32 pll_vcocon:4;
		u32 pll_lfcon:4;
		u32 pll_cpcon:4;
		u32 pll_lock_sel:6;
		u32 reserved0:1;
		u32 pll_lock_enable:1;
		u32 reserved1:1;
		u32 pll_dccon:1;
		u32 pll_pts:2;
		u32 reserved2:6;
		u32 pll_out1_div_byp:1;
		u32 pll_out1_inv_clk:1;
	};
	u32 word;
};

union xm2cfga_reg {
	struct {
		u32 reserved0:2;
		u32 hsm_en:1;
		u32 reserved1:2;
		u32 preemp_en:1;
		u32 vref_en:1;
		u32 reserved2:5;
		u32 cal_drvdn:5;
		u32 reserved3:3;
		u32 cal_drvup:5;
		u32 reserved4:3;
		u32 cal_drvdn_slwr:2;
		u32 cal_drvup_slwf:2;
	};
	u32 word;
};

union xm2cfgd_reg {
	struct {
		u32 reserved0:2;
		u32 hsm_en:1;
		u32 schmt_en:1;
		u32 lpmd:2;
		u32 vref_en:1;
		u32 reserved1:5;
		u32 cal_drvdn:5;
		u32 reserved2:3;
		u32 cal_drvup:5;
		u32 reserved3:3;
		u32 cal_drvdn_slwr:2;
		u32 cal_drvup_slwf:2;
	};
	u32 word;
};

union fbio_spare_reg {
	struct {
		u32 reserved:24;
		u32 cfg_wb0:8;
	};
	u32 word;
};

union scratch2_reg {
	struct {
		u32 pllm_base_divm:5;
		u32 pllm_base_divn:10;
		u32 pllm_base_divp:3;
		u32 pllm_misc_lfcon:4;
		u32 pllm_misc_cpcon:4;
		u32 gp_xm2cfga_padctrl_preemp:1;
		u32 gp_xm2cfgd_padctrl_schmt:1;
		u32 osc_ctrl_xobp:1;
		u32 memory_type:3;
	};
	u32 word;
};

union scratch4_reg {
	struct {
		u32 emc_clock_divider:8;
		u32 pllm_stable_time:8;
		u32 pllx_stable_time:8;
		u32 emc_fbio_spare_cfg_wb0:8;
	};
	u32 word;
};

union scratch24_reg {
	struct {
		u32 emc_auto_cal_wait:8;
		u32 emc_pin_program_wait:8;
		u32 warmboot_wait:8;
		u32 reserved:8;
	};
	u32 word;
};

void warmboot_save_sdram_params(void)
{
	u32 ram_code;
	struct sdram_params sdram;
	struct pmux_tri_ctlr *pmt = (struct pmux_tri_ctlr *)NV_PA_APB_MISC_BASE;
	struct clk_rst_ctlr *clkrst = (struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	struct apb_misc_gp_ctlr *gp =
			(struct apb_misc_gp_ctlr *)NV_PA_APB_MISC_GP_BASE;
	struct emc_ctlr *emc = (struct emc_ctlr *)NV_PA_EMC_BASE;
	union osc_ctrl_reg osc_ctrl;
	union pll_base_reg pllm_base;
	union pll_misc_reg pllm_misc;
	union scratch2_reg scratch2;
	union scratch4_reg scratch4;
	union scratch24_reg scratch24;
	union xm2cfga_reg xm2cfga;
	union xm2cfgd_reg xm2cfgd;
	union fbio_spare_reg fbio_spare;

	/* get ram code that is used as index to array sdram_params in BCT */
	ram_code = bf_readl(STRAP_OPT_A_RAM_CODE, &pmt->pmt_strap_opt_a) & 3;
	memcpy(&sdram,
	       (char *)((struct sdram_params *)SDRAM_PARAMS_BASE + ram_code),
	       sizeof(sdram));

	osc_ctrl.word = readl(&clkrst->crc_osc_ctrl);
	pllm_base.word = readl(&clkrst->crc_pll[CLOCK_ID_MEMORY].pll_base);
	pllm_misc.word = readl(&clkrst->crc_pll[CLOCK_ID_MEMORY].pll_misc);
	xm2cfga.word = readl(&gp->xm2cfga);
	xm2cfgd.word = readl(&gp->xm2cfgd);

	scratch2.word = 0;
	scratch2.osc_ctrl_xobp = osc_ctrl.xobp;
	scratch2.pllm_base_divm = pllm_base.pll_divm;
	scratch2.pllm_base_divn = pllm_base.pll_divn;
	scratch2.pllm_base_divp = pllm_base.pll_divp;
	scratch2.pllm_misc_cpcon = pllm_misc.pll_cpcon;
	scratch2.pllm_misc_lfcon = pllm_misc.pll_lfcon;
	scratch2.gp_xm2cfga_padctrl_preemp = xm2cfga.preemp_en;
	scratch2.gp_xm2cfgd_padctrl_schmt = xm2cfgd.schmt_en;
	scratch2.memory_type = sdram.memory_type;
	writel(scratch2.word, &pmc->pmc_scratch2);

	/* collect data from various sources for pmc_scratch4 */
	fbio_spare.word = readl(&emc->fbio_spare);
	scratch4.word = 0;
	scratch4.emc_fbio_spare_cfg_wb0 = fbio_spare.cfg_wb0;
	scratch4.emc_clock_divider = sdram.emc_clock_divider;
	scratch4.pllm_stable_time = -1;
	scratch4.pllx_stable_time = -1;
	writel(scratch4.word, &pmc->pmc_scratch4);

	/* collect various data from sdram for pmc_scratch24 */
	scratch24.word = 0;
	scratch24.emc_pin_program_wait = sdram.emc_pin_program_wait;
	scratch24.emc_auto_cal_wait = sdram.emc_auto_cal_wait;
	scratch24.warmboot_wait = sdram.warm_boot_wait;
	writel(scratch24.word, &pmc->pmc_scratch24);
}
