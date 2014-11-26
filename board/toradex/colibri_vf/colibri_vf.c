/*
 * Copyright 2013-2014 Toradex, Inc.
 *
 * Based on vf610twr.c:
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux-vf610.h>
#include <asm/arch/ddr-vf610.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/clock.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <i2c.h>

#include "../common/configblock.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
			PAD_CTL_DSE_25ohm | PAD_CTL_OBE_IBE_ENABLE)

#define ESDHC_PAD_CTRL	(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_HIGH | \
			PAD_CTL_DSE_20ohm | PAD_CTL_OBE_IBE_ENABLE)

#define ENET_PAD_CTRL	(PAD_CTL_PUS_47K_UP | PAD_CTL_SPEED_HIGH | \
			PAD_CTL_DSE_50ohm | PAD_CTL_OBE_IBE_ENABLE)

int dram_init(void)
{
	setup_iomux_ddr();

	ddr_ctrl_init(3120, 64, 1, 2, NULL);
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

static void setup_iomux_uart(void)
{
	static const iomux_v3_cfg_t uart_pads[] = {
		NEW_PAD_CTRL(VF610_PAD_PTB4__UART1_TX, UART_PAD_CTRL),	/* UART_C_TXD: SCI1_TX */
		NEW_PAD_CTRL(VF610_PAD_PTB5__UART1_RX, UART_PAD_CTRL),	/* UART_C_RXD: SCI1_RX */
		NEW_PAD_CTRL(VF610_PAD_PTB10__UART0_TX, UART_PAD_CTRL),	/* UART_A_TXD: SCI0_TX */
		NEW_PAD_CTRL(VF610_PAD_PTB11__UART0_RX, UART_PAD_CTRL),	/* UART_A_RXD: SCI0_RX */
	};

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));
}

static void setup_iomux_enet(void)
{
	static const iomux_v3_cfg_t enet0_pads[] = {
		NEW_PAD_CTRL(VF610_PAD_PTA6__RMII0_CLKOUT, ENET_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTC10__RMII1_MDIO, ENET_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTC9__RMII1_MDC, ENET_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTC11__RMII1_CRS_DV, ENET_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTC12__RMII1_RD1, ENET_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTC13__RMII1_RD0, ENET_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTC14__RMII1_RXER, ENET_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTC15__RMII1_TD1, ENET_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTC16__RMII1_TD0, ENET_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTC17__RMII1_TXEN, ENET_PAD_CTRL),
	};

	imx_iomux_v3_setup_multiple_pads(enet0_pads, ARRAY_SIZE(enet0_pads));
}

static void setup_iomux_i2c(void)
{
	static const iomux_v3_cfg_t i2c0_pads[] = {
		VF610_PAD_PTB14__I2C0_SCL,
		VF610_PAD_PTB15__I2C0_SDA,
	};

	imx_iomux_v3_setup_multiple_pads(i2c0_pads, ARRAY_SIZE(i2c0_pads));
}

#ifdef CONFIG_NAND_VF610_NFC
static void setup_iomux_nfc(void)
{
	static const iomux_v3_cfg_t nfc_pads[] = {
		VF610_PAD_PTD23__NF_IO7,
		VF610_PAD_PTD22__NF_IO6,
		VF610_PAD_PTD21__NF_IO5,
		VF610_PAD_PTD20__NF_IO4,
		VF610_PAD_PTD19__NF_IO3,
		VF610_PAD_PTD18__NF_IO2,
		VF610_PAD_PTD17__NF_IO1,
		VF610_PAD_PTD16__NF_IO0,
		VF610_PAD_PTB24__NF_WE_B,
		VF610_PAD_PTB25__NF_CE0_B,
		VF610_PAD_PTB27__NF_RE_B,
		VF610_PAD_PTC26__NF_RB_B,
		VF610_PAD_PTC27__NF_ALE,
		VF610_PAD_PTC28__NF_CLE
	};

	imx_iomux_v3_setup_multiple_pads(nfc_pads, ARRAY_SIZE(nfc_pads));
}
#endif

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg esdhc_cfg[1] = {
	{ESDHC1_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	/* eSDHC1 is always present */
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	static const iomux_v3_cfg_t esdhc1_pads[] = {
		NEW_PAD_CTRL(VF610_PAD_PTA24__ESDHC1_CLK, ESDHC_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTA25__ESDHC1_CMD, ESDHC_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTA26__ESDHC1_DAT0, ESDHC_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTA27__ESDHC1_DAT1, ESDHC_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTA28__ESDHC1_DAT2, ESDHC_PAD_CTRL),
		NEW_PAD_CTRL(VF610_PAD_PTA29__ESDHC1_DAT3, ESDHC_PAD_CTRL),
	};

	esdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);

	imx_iomux_v3_setup_multiple_pads(
		esdhc1_pads, ARRAY_SIZE(esdhc1_pads));

	return fsl_esdhc_initialize(bis, &esdhc_cfg[0]);
}
#endif

static inline int is_colibri_vf61(void)
{
	struct mscm *mscm = (struct mscm*)MSCM_BASE_ADDR;

	/*
	 * Detect board type by Level 2 Cache: VF50 don't have any
	 * Level 2 Cache.
	 */
	return !!mscm->cpxcfg1;
}

static void clock_init(void)
{
	struct ccm_reg *ccm = (struct ccm_reg *)CCM_BASE_ADDR;
	struct anadig_reg *anadig = (struct anadig_reg *)ANADIG_BASE_ADDR;
	u32 pfd_clk_sel, ddr_clk_sel;

	clrsetbits_le32(&ccm->ccgr0, CCM_REG_CTRL_MASK,
		CCM_CCGR0_UART0_CTRL_MASK);
	clrsetbits_le32(&ccm->ccgr1, CCM_REG_CTRL_MASK,
		CCM_CCGR1_PIT_CTRL_MASK | CCM_CCGR1_WDOGA5_CTRL_MASK |
		CCM_CCGR1_USBC0_CTRL_MASK);
	clrsetbits_le32(&ccm->ccgr2, CCM_REG_CTRL_MASK,
		CCM_CCGR2_IOMUXC_CTRL_MASK | CCM_CCGR2_PORTA_CTRL_MASK |
		CCM_CCGR2_PORTB_CTRL_MASK | CCM_CCGR2_PORTC_CTRL_MASK |
		CCM_CCGR2_PORTD_CTRL_MASK | CCM_CCGR2_PORTE_CTRL_MASK);
	clrsetbits_le32(&ccm->ccgr3, CCM_REG_CTRL_MASK,
		CCM_CCGR3_ANADIG_CTRL_MASK | CCM_CCGR3_SCSC_CTRL_MASK);
	clrsetbits_le32(&ccm->ccgr4, CCM_REG_CTRL_MASK,
		CCM_CCGR4_WKUP_CTRL_MASK | CCM_CCGR4_CCM_CTRL_MASK |
		CCM_CCGR4_GPC_CTRL_MASK | CCM_CCGR4_I2C0_CTRL_MASK);
	clrsetbits_le32(&ccm->ccgr6, CCM_REG_CTRL_MASK,
		CCM_CCGR6_OCOTP_CTRL_MASK | CCM_CCGR6_DDRMC_CTRL_MASK);
	clrsetbits_le32(&ccm->ccgr7, CCM_REG_CTRL_MASK,
		CCM_CCGR7_SDHC1_CTRL_MASK | CCM_CCGR7_USBC1_CTRL_MASK);
	clrsetbits_le32(&ccm->ccgr9, CCM_REG_CTRL_MASK,
		CCM_CCGR9_FEC0_CTRL_MASK | CCM_CCGR9_FEC1_CTRL_MASK);
	clrsetbits_le32(&ccm->ccgr10, CCM_REG_CTRL_MASK,
		CCM_CCGR10_NFC_CTRL_MASK);

	clrsetbits_le32(&anadig->pll7_ctrl, ANADIG_PLL7_CTRL_BYPASS |
		ANADIG_PLL7_CTRL_POWERDOWN, ANADIG_PLL7_CTRL_ENABLE |
		ANADIG_PLL7_CTRL_DIV_SELECT);
	clrsetbits_le32(&anadig->pll5_ctrl, ANADIG_PLL5_CTRL_BYPASS |
		ANADIG_PLL5_CTRL_POWERDOWN, ANADIG_PLL5_CTRL_ENABLE |
		ANADIG_PLL5_CTRL_DIV_SELECT);
	clrsetbits_le32(&anadig->pll3_ctrl, ANADIG_PLL3_CTRL_BYPASS |
		ANADIG_PLL3_CTRL_POWERDOWN, ANADIG_PLL3_CTRL_ENABLE |
		ANADIG_PLL3_CTRL_DIV_SELECT);

	if (is_colibri_vf61()) {
		clrsetbits_le32(&anadig->pll2_ctrl, ANADIG_PLL5_CTRL_BYPASS |
			ANADIG_PLL2_CTRL_POWERDOWN, ANADIG_PLL2_CTRL_ENABLE |
			ANADIG_PLL2_CTRL_DIV_SELECT);
	}

	clrsetbits_le32(&anadig->pll1_ctrl, ANADIG_PLL1_CTRL_POWERDOWN,
		ANADIG_PLL1_CTRL_ENABLE | ANADIG_PLL1_CTRL_DIV_SELECT);


	clrsetbits_le32(&ccm->ccr, CCM_CCR_OSCNT_MASK,
		CCM_CCR_FIRC_EN | CCM_CCR_OSCNT(5));

	/* See "Typical PLL Configuration" */
	if (is_colibri_vf61()) {
		pfd_clk_sel = CCM_CCSR_PLL1_PFD_CLK_SEL(1);
		ddr_clk_sel = CCM_CCSR_DDRC_CLK_SEL(0);
	} else {
		pfd_clk_sel = CCM_CCSR_PLL1_PFD_CLK_SEL(3);
		ddr_clk_sel = CCM_CCSR_DDRC_CLK_SEL(1);
	}

	clrsetbits_le32(&ccm->ccsr, CCM_REG_CTRL_MASK, pfd_clk_sel |
		CCM_CCSR_PLL3_PFD4_EN | CCM_CCSR_PLL3_PFD3_EN |
		CCM_CCSR_PLL3_PFD2_EN | CCM_CCSR_PLL3_PFD1_EN |
		CCM_CCSR_PLL2_PFD4_EN | CCM_CCSR_PLL2_PFD3_EN |
		CCM_CCSR_PLL2_PFD2_EN | CCM_CCSR_PLL2_PFD1_EN |
		CCM_CCSR_PLL1_PFD4_EN | CCM_CCSR_PLL1_PFD3_EN |
		CCM_CCSR_PLL1_PFD2_EN | CCM_CCSR_PLL1_PFD1_EN |
		ddr_clk_sel | CCM_CCSR_FAST_CLK_SEL(1) |
		CCM_CCSR_SYS_CLK_SEL(4));

	clrsetbits_le32(&ccm->cacrr, CCM_REG_CTRL_MASK,
		CCM_CACRR_IPG_CLK_DIV(1) | CCM_CACRR_BUS_CLK_DIV(2) |
		CCM_CACRR_ARM_CLK_DIV(0));
	clrsetbits_le32(&ccm->cscmr1, CCM_REG_CTRL_MASK,
		CCM_CSCMR1_ESDHC1_CLK_SEL(3) | CCM_CSCMR1_NFC_CLK_SEL(0));
	clrsetbits_le32(&ccm->cscdr1, CCM_REG_CTRL_MASK,
		CCM_CSCDR1_RMII_CLK_EN);
	clrsetbits_le32(&ccm->cscdr2, CCM_REG_CTRL_MASK,
		CCM_CSCDR2_ESDHC1_EN | CCM_CSCDR2_ESDHC1_CLK_DIV(0) |
		CCM_CSCDR2_NFC_EN);
	clrsetbits_le32(&ccm->cscdr3, CCM_REG_CTRL_MASK,
		CCM_CSCDR3_NFC_PRE_DIV(5));
	clrsetbits_le32(&ccm->cscmr2, CCM_REG_CTRL_MASK,
		CCM_CSCMR2_RMII_CLK_SEL(2));	/* PLL5 main clock */
}

static void mscm_init(void)
{
	struct mscm_ir *mscmir = (struct mscm_ir *)MSCM_IR_BASE_ADDR;
	int i;

	for (i = 0; i < MSCM_IRSPRC_NUM; i++)
		writew(MSCM_IRSPRC_CP0_EN, &mscmir->irsprc[i]);
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_early_init_f(void)
{
	clock_init();
	mscm_init();

	setup_iomux_uart();
	setup_iomux_enet();
	setup_iomux_i2c();
#ifdef CONFIG_NAND_VF610_NFC
	setup_iomux_nfc();
#endif

	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
	struct src *src = (struct src *)SRC_BASE_ADDR;

	/* Default memory arguments */
	if (!getenv("memargs")) {
		switch (gd->ram_size) {
		case 0x08000000:
			/* 128 MB */
			setenv("memargs", "mem=128M");
			break;
		case 0x10000000:
			/* 256 MB */
			setenv("memargs", "mem=256M");
			break;
		default:
			printf("Failed detecting RAM size.\n");
		}
	}

	if (((src->sbmr2 & SRC_SBMR2_BMOD_MASK) >> SRC_SBMR2_BMOD_SHIFT)
			== SRC_SBMR2_BMOD_SERIAL) {
		printf("Serial Downloader recovery mode, disable autoboot\n");
		setenv("bootdelay", "-1");
	}

#ifdef CONFIG_TRDX_CFG_BLOCK
	if (read_trdx_cfg_block())
		printf("Missing Colibri config block\n");
#endif

	return 0;
}
#endif /* CONFIG_BOARD_LATE_INIT */

int board_init(void)
{
	struct scsc_reg *scsc = (struct scsc_reg *)SCSC_BASE_ADDR;

	if (!is_colibri_vf61())
		gd->bd->bi_arch_number = MACH_TYPE_COLIBRI_VF50;
	else
		gd->bd->bi_arch_number = MACH_TYPE_COLIBRI_VF61;
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	/*
	 * Enable external 32K Oscillator
	 *
	 * The internal clock experiences significant drift
         * so we must use the external oscillator in order
	 * to maintain correct time in the hwclock
	 */
	setbits_le32(&scsc->sosc_ctr, SCSC_SOSC_CTR_SOSC_EN);

	return 0;
}

int checkboard(void)
{
	if (is_colibri_vf61())
		puts("Board: Colibri VF61\n");
	else
		puts("Board: Colibri VF50\n");

	return 0;
}
