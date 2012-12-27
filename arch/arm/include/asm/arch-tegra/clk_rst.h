/*
 *  (C) Copyright 2010-2012
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

#ifndef _CLK_RST_H_
#define _CLK_RST_H_


/* PLL registers - there are several PLLs in the clock controller */
struct clk_pll {
	uint pll_base;		/* the control register */
	uint pll_out;		/* output control */
	uint pll_out_b;		/* some have output B control */
	uint pll_misc;		/* other misc things */
};

/* PLL registers - there are several PLLs in the clock controller */
struct clk_pll_simple {
	uint pll_base;		/* the control register */
	uint pll_misc;		/* other misc things */
};

/* RST_DEV_(L,H,U,V,W)_(SET,CLR) and CLK_ENB_(L,H,U,V,W)_(SET,CLR) */
struct clk_set_clr {
	uint set;
	uint clr;
};

/*
 * Most PLLs use the clk_pll structure, but some have a simpler two-member
 * structure for which we use clk_pll_simple. The reason for this non-
 * othogonal setup is not stated.
 */
enum {
	TEGRA_CLK_PLLS		= 6,	/* Number of normal PLLs */
	TEGRA_CLK_SIMPLE_PLLS	= 3,	/* Number of simple PLLs */
	TEGRA_CLK_REGS		= 3,	/* Number of clock enable regs L/H/U */
	TEGRA_CLK_SOURCES	= 64,	/* Number of ppl clock sources L/H/U */
	TEGRA_CLK_REGS_VW	= 2,	/* Number of clock enable regs V/W */
	TEGRA_CLK_SOURCES_VW	= 32,	/* Number of ppl clock sources V/W*/
};

/* Clock/Reset Controller (CLK_RST_CONTROLLER_) regs */
struct clk_rst_ctlr {
	uint crc_rst_src;			/* _RST_SOURCE_0,0x00 */
	uint crc_rst_dev[TEGRA_CLK_REGS];	/* _RST_DEVICES_L/H/U_0 */
	uint crc_clk_out_enb[TEGRA_CLK_REGS];	/* _CLK_OUT_ENB_L/H/U_0 */
	uint crc_reserved0;		/* reserved_0,		0x1C */
	uint crc_cclk_brst_pol;		/* _CCLK_BURST_POLICY_0,0x20 */
	uint crc_super_cclk_div;	/* _SUPER_CCLK_DIVIDER_0,0x24 */
	uint crc_sclk_brst_pol;		/* _SCLK_BURST_POLICY_0, 0x28 */
	uint crc_super_sclk_div;	/* _SUPER_SCLK_DIVIDER_0,0x2C */
	uint crc_clk_sys_rate;		/* _CLK_SYSTEM_RATE_0,	0x30 */
	uint crc_prog_dly_clk;		/* _PROG_DLY_CLK_0,	0x34 */
	uint crc_aud_sync_clk_rate;	/* _AUDIO_SYNC_CLK_RATE_0,0x38 */
	uint crc_reserved1;		/* reserved_1,		0x3C */
	uint crc_cop_clk_skip_plcy;	/* _COP_CLK_SKIP_POLICY_0,0x40 */
	uint crc_clk_mask_arm;		/* _CLK_MASK_ARM_0,	0x44 */
	uint crc_misc_clk_enb;		/* _MISC_CLK_ENB_0,	0x48 */
	uint crc_clk_cpu_cmplx;		/* _CLK_CPU_CMPLX_0,	0x4C */
	uint crc_osc_ctrl;		/* _OSC_CTRL_0,		0x50 */
	uint crc_pll_lfsr;		/* _PLL_LFSR_0,		0x54 */
	uint crc_osc_freq_det;		/* _OSC_FREQ_DET_0,	0x58 */
	uint crc_osc_freq_det_stat;	/* _OSC_FREQ_DET_STATUS_0,0x5C */
	uint crc_reserved2[8];		/* reserved_2[8],	0x60-7C */

	struct clk_pll crc_pll[TEGRA_CLK_PLLS];	/* PLLs from 0x80 to 0xdc */

	/* PLLs from 0xe0 to 0xf4    */
	struct clk_pll_simple crc_pll_simple[TEGRA_CLK_SIMPLE_PLLS];

	uint crc_reserved10;		/* _reserved_10,	0xF8 */
	uint crc_reserved11;		/* _reserved_11,	0xFC */

	uint crc_clk_src[TEGRA_CLK_SOURCES]; /*_I2S1_0...	0x100-1fc */

	uint crc_reserved20[64];	/* _reserved_20,	0x200-2fc */

	/* _RST_DEV_L/H/U_SET_0 0x300 ~ 0x314 */
	struct clk_set_clr crc_rst_dev_ex[TEGRA_CLK_REGS];

	uint crc_reserved30[2];		/* _reserved_30,	0x318, 0x31c */

	/* _CLK_ENB_L/H/U_CLR_0 0x320 ~ 0x334 */
	struct clk_set_clr crc_clk_enb_ex[TEGRA_CLK_REGS];

	uint crc_reserved31[2];		/* _reserved_31,	0x338, 0x33c */

	uint crc_cpu_cmplx_set;		/* _RST_CPU_CMPLX_SET_0,    0x340 */
	uint crc_cpu_cmplx_clr;		/* _RST_CPU_CMPLX_CLR_0,    0x344 */

	/* Additional (T30) registers */
	uint crc_clk_cpu_cmplx_set;	/* _CLK_CPU_CMPLX_SET_0,    0x348 */
	uint crc_clk_cpu_cmplx_clr;	/* _CLK_CPU_CMPLX_SET_0,    0x34c */

	uint crc_reserved32[2];		/* _reserved_32,      0x350,0x354 */

	uint crc_rst_dev_vw[TEGRA_CLK_REGS_VW]; /* _RST_DEVICES_V/W_0 */
	uint crc_clk_out_enb_vw[TEGRA_CLK_REGS_VW]; /* _CLK_OUT_ENB_V/W_0 */
	uint crc_cclkg_brst_pol;	/* _CCLKG_BURST_POLICY_0,   0x368 */
	uint crc_super_cclkg_div;	/* _SUPER_CCLKG_DIVIDER_0,  0x36C */
	uint crc_cclklp_brst_pol;	/* _CCLKLP_BURST_POLICY_0,  0x370 */
	uint crc_super_cclkp_div;	/* _SUPER_CCLKLP_DIVIDER_0, 0x374 */
	uint crc_clk_cpug_cmplx;	/* _CLK_CPUG_CMPLX_0,       0x378 */
	uint crc_clk_cpulp_cmplx;	/* _CLK_CPULP_CMPLX_0,      0x37C */
	uint crc_cpu_softrst_ctrl;	/* _CPU_SOFTRST_CTRL_0,     0x380 */
	uint crc_reserved33[11];	/* _reserved_33,        0x384-3ac */
	uint crc_clk_src_vw[TEGRA_CLK_SOURCES_VW]; /* _G3D2_0..., 0x3b0-0x42c */
	/* _RST_DEV_V/W_SET_0 0x430 ~ 0x43c */
	struct clk_set_clr crc_rst_dev_ex_vw[TEGRA_CLK_REGS_VW];
	/* _CLK_ENB_V/W_CLR_0 0x440 ~ 0x44c */
	struct clk_set_clr crc_clk_enb_ex_vw[TEGRA_CLK_REGS_VW];
	uint crc_reserved40[12];	/* _reserved_40,	0x450-47C */
	uint crc_pll_cfg0;		/* _PLL_CFG0_0,		0x480 */
	uint crc_pll_cfg1;		/* _PLL_CFG1_0,		0x484 */
	uint crc_pll_cfg2;		/* _PLL_CFG2_0,		0x488 */
};

/* CLK_RST_CONTROLLER_CLK_CPU_CMPLX_0 */
#define CPU1_CLK_STP_RANGE	9:9
#define CPU0_CLK_STP_RANGE	8:8

/* CLK_RST_CONTROLLER_PLLx_BASE_0 */
#define PLL_BYPASS_RANGE	31:31
#define PLL_ENABLE_RANGE	30:30
#define PLL_BASE_OVRRIDE_RANGE	28:28
#define PLL_DIVP_RANGE		22:20
#define PLL_DIVN_RANGE		17:8
#define PLL_DIVM_RANGE		4:0

/* CLK_RST_CONTROLLER_PLLx_MISC_0 */
#define PLL_CPCON_RANGE		11:8
#define PLL_LFCON_RANGE		7:4
#define PLLU_VCO_FREQ_RANGE	20:20
#define PLL_VCO_FREQ_RANGE	3:0

#define PLLP_OUT1_OVR		(1 << 2)
#define PLLP_OUT2_OVR		(1 << 18)
#define PLLP_OUT3_OVR		(1 << 2)
#define PLLP_OUT4_OVR		(1 << 18)
#define PLLP_OUT1_RATIO		8
#define PLLP_OUT2_RATIO		24
#define PLLP_OUT3_RATIO		8
#define PLLP_OUT4_RATIO		24
enum {
	IN_408_OUT_204_DIVISOR = 2,
	IN_408_OUT_102_DIVISOR = 6,
	IN_408_OUT_48_DIVISOR = 15,
	IN_408_OUT_9_6_DIVISOR = 83,
};

/* CLK_RST_CONTROLLER_OSC_CTRL_0 */
#define OSC_FREQ_RANGE		31:30

/* CLK_RST_CONTROLLER_CLK_SOURCE_x_OUT_0 */
#define OUT_CLK_DIVISOR_RANGE	7:0
#define OUT_CLK_SOURCE_RANGE	31:30
#define OUT_CLK_SOURCE4_RANGE	31:28

/* CLK_RST_CONTROLLER_CLK_SOURCE_MSELECT */
#define MSELECT_CLK_M_SHIFT	30
#define MSELECT_CLK_M_MASK	(3U << MSELECT_CLK_M_SHIFT)

/* CLK_RST_CONTROLLER_SCLK_BURST_POLICY */
#define SCLK_SYS_STATE_SHIFT	28U
#define SCLK_SYS_STATE_MASK	(15U << SCLK_SYS_STATE_SHIFT)
enum {
	SCLK_SYS_STATE_STDBY,
	SCLK_SYS_STATE_IDLE,
	SCLK_SYS_STATE_RUN,
	SCLK_SYS_STATE_IRQ = 4U,
	SCLK_SYS_STATE_FIQ = 8U,
};
#define SCLK_COP_FIQ_MASK	(1 << 27)
#define SCLK_CPU_FIQ_MASK	(1 << 26)
#define SCLK_COP_IRQ_MASK	(1 << 25)
#define SCLK_CPU_IRQ_MASK	(1 << 24)
#define SCLK_SWAKEUP_FIQ_SOURCE_SHIFT		12
#define SCLK_SWAKEUP_FIQ_SOURCE_MASK		\
		(7 << SCLK_SWAKEUP_FIQ_SOURCE_SHIFT)
#define SCLK_SWAKEUP_IRQ_SOURCE_SHIFT		8
#define SCLK_SWAKEUP_IRQ_SOURCE_MASK		\
		(7 << SCLK_SWAKEUP_FIQ_SOURCE_SHIFT)
#define SCLK_SWAKEUP_RUN_SOURCE_SHIFT		4
#define SCLK_SWAKEUP_RUN_SOURCE_MASK		\
		(7 << SCLK_SWAKEUP_FIQ_SOURCE_SHIFT)
#define SCLK_SWAKEUP_IDLE_SOURCE_SHIFT		0
#define SCLK_SWAKEUP_IDLE_SOURCE_MASK		\
		(7 << SCLK_SWAKEUP_FIQ_SOURCE_SHIFT)
enum {
	SCLK_SOURCE_CLKM,
	SCLK_SOURCE_PLLC_OUT1,
	SCLK_SOURCE_PLLP_OUT4,
	SCLK_SOURCE_PLLP_OUT3,
	SCLK_SOURCE_PLLP_OUT2,
	SCLK_SOURCE_CLKD,
	SCLK_SOURCE_CLKS,
	SCLK_SOURCE_PLLM_OUT1,
};
#define SCLK_SWAKE_FIQ_SRC_PLLM_OUT1	(7 << 12)
#define SCLK_SWAKE_IRQ_SRC_PLLM_OUT1	(7 << 8)
#define SCLK_SWAKE_RUN_SRC_PLLM_OUT1	(7 << 4)
#define SCLK_SWAKE_IDLE_SRC_PLLM_OUT1	(7 << 0)

/* CLK_RST_CONTROLLER_SUPER_SCLK_DIVIDER */
#define SUPER_SCLK_ENB_SHIFT		31U
#define SUPER_SCLK_ENB_MASK		(1U << 31)
#define SUPER_SCLK_DIVIDEND_SHIFT	8
#define SUPER_SCLK_DIVIDEND_MASK	(0xff << SUPER_SCLK_DIVIDEND_SHIFT)
#define SUPER_SCLK_DIVISOR_SHIFT	0
#define SUPER_SCLK_DIVISOR_MASK		(0xff << SUPER_SCLK_DIVISOR_SHIFT)


/* CLK_RST_CONTROLLER_CLK_SYSTEM_RATE */
#define CLK_SYS_RATE_HCLK_DISABLE_SHIFT	7
#define CLK_SYS_RATE_HCLK_DISABLE_MASK	(1 << CLK_SYS_RATE_HCLK_DISABLE_SHIFT)
#define CLK_SYS_RATE_AHB_RATE_SHIFT	4
#define CLK_SYS_RATE_AHB_RATE_MASK	(3 << CLK_SYS_RATE_AHB_RATE_SHIFT)
#define CLK_SYS_RATE_PCLK_DISABLE_SHIFT	3
#define CLK_SYS_RATE_PCLK_DISABLE_MASK	(1 << CLK_SYS_RATE_PCLK_DISABLE_SHIFT)
#define CLK_SYS_RATE_APB_RATE_SHIFT	0
#define CLK_SYS_RATE_APB_RATE_MASK	(3 << CLK_SYS_RATE_AHB_RATE_SHIFT)

#if defined(CONFIG_TEGRA3)
/* UTMIP PLL config regs moved from USB to CLK/RST domain on T30 */
/* CLK_RST_CONTROLLER_UTMIP_PLL_CFG1_0 */
#define UTMIP_PLLU_ENABLE_DLY_COUNT_RANGE		31:27
#define UTMIP_PLL_SETUP_RANGE				26:18
#define UTMIP_FORCE_PLLU_POWERUP_RANGE			17:17
#define UTMIP_FORCE_PLLU_POWERDOWN_RANGE		16:16
#define UTMIP_FORCE_PLL_ENABLE_POWERUP_RANGE		15:15
#define UTMIP_FORCE_PLL_ENABLE_POWERDOWN_RANGE		14:14
#define UTMIP_FORCE_PLL_ACTIVE_POWERUP_RANGE		13:13
#define UTMIP_FORCE_PLL_ACTIVE_POWERDOWN_RANGE		12:12
#define UTMIP_XTAL_FREQ_COUNT_RANGE			11:0

/* CLK_RST_CONTROLLER_UTMIP_PLL_CFG2_0 */
#define UTMIP_PHY_XTAL_CLOCKEN_RANGE			30:30
#define UTMIP_FORCE_PD_CLK60_POWERUP_RANGE		29:29
#define UTMIP_FORCE_PD_CLK60_POWERDOWN_RANGE		28:28
#define UTMIP_FORCE_PD_CLK48_POWERUP_RANGE		27:27
#define UTMIP_FORCE_PD_CLK48_POWERDOWN_RANGE		26:26
#define UTMIP_PLL_ACTIVE_DLY_COUNT_RANGE		23:18
#define UTMIP_PLLU_STABLE_COUNT_RANGE			17:6
#define UTMIP_FORCE_PD_SAMP_C_POWERUP_RANGE		5:5
#define UTMIP_FORCE_PD_SAMP_C_POWERDOWN_RANGE		4:4
#define UTMIP_FORCE_PD_SAMP_B_POWERUP_RANGE		3:3
#define UTMIP_FORCE_PD_SAMP_B_POWERDOWN_RANGE		2:2
#define UTMIP_FORCE_PD_SAMP_A_POWERUP_RANGE		1:1
#define UTMIP_FORCE_PD_SAMP_A_POWERDOWN_RANGE		0:0

/* CRC_CCLK_BURST_POLICY_0 20h */
#define CCLK_PLLP_BURST_POLICY			0x20004444

/* CRC_SUPER_CCLK_DIVIDER_0 24h */
#define SUPER_CDIV_ENB				(1 << 31)

/* CRC_SCLK_BURST_POLICY_0 28h */
#define SCLK_SWAKE_FIQ_SRC_CLKM			(0 << 12)
#define SCLK_SWAKE_IRQ_SRC_CLKM			(0 << 8)
#define SCLK_SWAKE_RUN_SRC_CLKM			(0 << 4)
#define SCLK_SWAKE_IDLE_SRC_CLKM		(0 << 0)

/* CRC_CLK_CPU_CMPLX_CLR_0 34ch */
#define CPU_CMPLX_CLR_CPU0_CLK_STP		(1 << 8)

/* CRC_MISC_CLK_ENN_0 48h */
#define MISC_CLK_ENB_EN_PPSB_STOPCLK_ENABLE	(1 << 0)

/* CRC_OSC_CTRL_0 50h */
#define OSC_CTRL_XOE		(1 << 0)
#define OSC_CTRL_XOE_ENABLE	(1 << 0)
#define OSC_CTRL_XOFS		(0x3f << 4)
#define OSC_CTRL_XOFS_4		(0x4 << 4)
#define OSC_CTRL_OSC_FREQ_SHIFT	28
#define OSC_FREQ_OSC19P2		4	/* 19.2MHz */
#define OSC_FREQ_OSC12			8	/* 12.0MHz */
#define OSC_FREQ_OSC26			12	/* 26.0MHz */
#define OSC_FREQ_OSC16P8		1	/* 16.8MHz */
#define OSC_FREQ_OSC38P4		5	/* 38.4MHz */
#define OSC_FREQ_OSC48			9	/* 48.0MHz */

/* CRC_PLLP_BASE_0 a0h */
#define PLLP_BASE_PLLP_DIVM_SHIFT		0
#define PLLP_BASE_PLLP_DIVN_SHIFT		8
#define PLLP_BASE_PLLP_LOCK_LOCK		(1 << 27)
#define PLLP_BASE_OVRRIDE_ENABLE		(1 << 28)
#define PLLP_BASE_PLLP_ENABLE			(1 << 30)

/* CRC_PLLP_OUTA_0 a4h */
#define PLLP_OUTA_OUT1_RSTN_RESET_DISABLE	(1 << 0)
#define PLLP_OUTA_OUT1_CLKEN			(1 << 1)
#define PLLP_OUTA_OUT1_OVRRIDE			(1 << 2)
#define PLLP_OUTA_OUT1_RATIO_83			(83 << 8)
#define PLLP_OUTA_OUT2_RSTN_RESET_DISABLE	(1 << 16)
#define PLLP_OUTA_OUT2_CLKEN			(1 << 17)
#define PLLP_OUTA_OUT2_OVRRIDE			(1 << 18)
#define PLLP_OUTA_OUT2_RATIO_15			(15 << 24)
#define PLLP_408_OUTA (PLLP_OUTA_OUT2_RATIO_15 |		\
			PLLP_OUTA_OUT2_OVRRIDE |		\
			PLLP_OUTA_OUT2_CLKEN |		\
			PLLP_OUTA_OUT2_RSTN_RESET_DISABLE |	\
			PLLP_OUTA_OUT1_RATIO_83 |		\
			PLLP_OUTA_OUT1_OVRRIDE |		\
			PLLP_OUTA_OUT1_CLKEN |		\
			PLLP_OUTA_OUT1_RSTN_RESET_DISABLE)

/* CRC_PLLP_OUTB_0 a8h */
#define PLLP_OUTA_OUT3_RSTN_RESET_DISABLE	(1 << 0)
#define PLLP_OUTA_OUT3_CLKEN			(1 << 1)
#define PLLP_OUTA_OUT3_OVRRIDE			(1 << 2)
#define PLLP_OUTA_OUT3_RATIO_6			(6 << 8)
#define PLLP_OUTA_OUT4_RSTN_RESET_DISABLE	(1 << 16)
#define PLLP_OUTA_OUT4_CLKEN			(1 << 17)
#define PLLP_OUTA_OUT4_OVRRIDE			(1 << 18)
#define PLLP_OUTA_OUT4_RATIO_6			(6 << 24)
#define PLLP_408_OUTB (PLLP_OUTA_OUT4_RATIO_6 |		\
			PLLP_OUTA_OUT4_OVRRIDE |	\
			PLLP_OUTA_OUT4_CLKEN |	\
			PLLP_OUTA_OUT4_RSTN_RESET_DISABLE |	\
			PLLP_OUTA_OUT3_RATIO_6 |	\
			PLLP_OUTA_OUT3_OVRRIDE |	\
			PLLP_OUTA_OUT3_CLKEN |	\
			PLLP_OUTA_OUT3_RSTN_RESET_DISABLE)

/* CRC_PLLP_MISC_0 ach */
#define PLLP_MISC_PLLP_CPCON_8			(8 << 8)
#define PLLP_MISC_PLLP_LOCK_ENABLE		(1 << 18)

/* CRC_PLLU_BASE_0 c0h */
#define PLLU_BYPASS_ENABLE			(1 << 31)
#define PLLU_ENABLE				(1 << 30)

/* CRC_PLLU_MISC_0 cch */
#define PLLU_LOCK_ENABLE			(1 << 22)

/* CRC_RST_DEV_L_CLR_0 304h */
#define CLR_SDMMC4_RST_ENABLE			(1 << 15)

/* CRC_CLK_ENB_L_SET_0 320h */
#define SET_CLK_ENB_SDMMC4			(1 << 15)

/* CRC_CLK_ENB_L_CLR_0 324h */
#define CLR_CLK_ENB_SDMMC4_ENABLE		(1 << 15)

/* CRC_CLK_CPU_CMPLX_SET_0 348h */
#define SET_CPU0_CLK_STP			(1 << 8)
#define SET_CPU1_CLK_STP			(1 << 9)
#define SET_CPU2_CLK_STP			(1 << 10)
#define SET_CPU3_CLK_STP			(1 << 11)

/* CRC_CLK_SOURCE_MSELECT_0 3b4 */
#define MSELECT_CLK_SRC_PLLP_OUT0		(0 << 30)

/* CRC_RST_DEV_V_SET_0 430h */
#define SET_MSELECT_RST_ENABLE			(1 << 3)

/* CRC_CLK_ENB_V_SET_0 440h */
#define SET_CLK_ENB_MSELECT			(1 << 3)

#endif	/* Tegra3 */

#endif	/* CLK_RST_H */
