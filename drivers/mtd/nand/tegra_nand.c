/*
 * Copyright (c) 2011 The Chromium OS Authors.
 * (C) Copyright 2011 NVIDIA Corporation <www.nvidia.com>
 * (C) Copyright 2006 Detlev Zundel, dzu@denx.de
 * (C) Copyright 2006 DENX Software Engineering
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <nand.h>
#include <asm/arch/clock.h>
#include <asm/arch/funcmux.h>
#include <asm/arch-tegra/clk_rst.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <fdtdec.h>
#include "tegra_nand.h"

DECLARE_GLOBAL_DATA_PTR;

#define NAND_CMD_TIMEOUT_MS		10

static struct nand_ecclayout tegra_nand_oob_16 = {
	.eccbytes = 4,
	.eccpos = { 4, 5, 6, 7 },
	.oobfree = {
		{ .offset = 8, . length = 8 }
	}
};

static struct nand_ecclayout tegra_nand_oob_64 = {
	.eccbytes = 36,
	.eccpos = {
		 4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
		20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
		36, 37, 38, 39
	},
	.oobfree = {
		{ .offset = 40, .length = 24 }
	}
};

static struct nand_ecclayout tegra_nand_oob_128 = {
	.eccbytes = 72,
	.eccpos = {
		 4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
		20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
		36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
		52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
		68, 69, 70, 71, 72, 73, 74, 75
	},
	.oobfree = {
		{ .offset = 76, .length = 52 }
	}
};

static struct nand_ecclayout tegra_nand_oob_224 = {
	.eccbytes = 144,
	.eccpos = {
		  4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15,  16,
		 17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,
		 30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,
		 43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,
		 56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,
		 69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,
		 82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,
		 95,  96,  97,  98,  99, 100, 101, 102, 103, 104, 105, 106, 107,
		108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120,
		121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133,
		134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146,
		147
	},
	.oobfree = {
		{ .offset = 148, .length = 76 }
	}
};

enum {
	ECC_OK,
	ECC_TAG_ERROR = 1 << 0,
	ECC_DATA_ERROR = 1 << 1
};

/* Timing parameters */
enum {
	FDT_NAND_MAX_TRP_TREA,
	FDT_NAND_TWB,
	FDT_NAND_MAX_TCR_TAR_TRR,
	FDT_NAND_TWHR,
	FDT_NAND_MAX_TCS_TCH_TALS_TALH,
	FDT_NAND_TWH,
	FDT_NAND_TWP,
	FDT_NAND_TRH,
	FDT_NAND_TADL,

	FDT_NAND_TIMING_COUNT
};

/* Information about an attached NAND chip */
struct fdt_nand {
	struct nand_ctlr *reg;
	int enabled;		/* 1 to enable, 0 to disable */
	struct gpio_desc wp_gpio;	/* write-protect GPIO */
	s32 width;		/* bit width, normally 8 */
	u32 timing[FDT_NAND_TIMING_COUNT];
};

struct nand_drv {
	struct nand_ctlr *reg;
	struct fdt_nand config;
	uint8_t *data_buf;	/* cache alignment bounce buffer */
};

static struct nand_drv nand_ctrl;
static struct mtd_info *our_mtd;
static struct nand_chip nand_chip[CONFIG_SYS_MAX_NAND_DEVICE];

#ifdef CONFIG_SYS_DCACHE_OFF
static inline void dma_prepare(void *start, unsigned long length,
			       int is_writing)
{
}
#else
/**
 * Prepare for a DMA transaction
 *
 * For a write we flush out our data. For a read we invalidate, since we
 * need to do this before we read from the buffer after the DMA has
 * completed, so may as well do it now.
 *
 * @param start		Start address for DMA buffer (should be cache-aligned)
 * @param length	Length of DMA buffer in bytes
 * @param is_writing	0 if reading, non-zero if writing
 */
static void dma_prepare(void *start, unsigned long length, int is_writing)
{
	unsigned long addr = (unsigned long)start;

	length = ALIGN(length, ARCH_DMA_MINALIGN);
	if (is_writing)
		flush_dcache_range(addr, addr + length);
	else
		invalidate_dcache_range(addr, addr + length);
}
#endif

/**
 * Wait for command completion
 *
 * @param reg	nand_ctlr structure
 * @return
 *	1 - Command completed
 *	0 - Timeout
 */
static int nand_waitfor_cmd_completion(struct nand_ctlr *reg)
{
	u32 reg_val;
	int running;
	int i;

	for (i = 0; i < NAND_CMD_TIMEOUT_MS * 1000; i++) {
		if ((readl(&reg->command) & CMD_GO) ||
				!(readl(&reg->status) & STATUS_RBSY0) ||
				!(readl(&reg->isr) & ISR_IS_CMD_DONE)) {
			udelay(1);
			continue;
		}
		reg_val = readl(&reg->dma_mst_ctrl);
		/*
		 * If DMA_MST_CTRL_EN_A_ENABLE or DMA_MST_CTRL_EN_B_ENABLE
		 * is set, that means DMA engine is running.
		 *
		 * Then we have to wait until DMA_MST_CTRL_IS_DMA_DONE
		 * is cleared, indicating DMA transfer completion.
		 */
		running = reg_val & (DMA_MST_CTRL_EN_A_ENABLE |
				DMA_MST_CTRL_EN_B_ENABLE);
		if (!running || (reg_val & DMA_MST_CTRL_IS_DMA_DONE))
			return 1;
		udelay(1);
	}
	return 0;
}

/**
 * Read one byte from the chip
 *
 * @param mtd	MTD device structure
 * @return	data byte
 *
 * Read function for 8bit bus-width
 */
static uint8_t read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_drv *info;

	info = (struct nand_drv *)chip->priv;

	writel(CMD_GO | CMD_PIO | CMD_RX | CMD_CE0 | CMD_A_VALID,
	       &info->reg->command);
	if (!nand_waitfor_cmd_completion(info->reg))
		printf("Command timeout\n");

	return (uint8_t)readl(&info->reg->resp);
}

/**
 * Read len bytes from the chip into a buffer
 *
 * @param mtd	MTD device structure
 * @param buf	buffer to store data to
 * @param len	number of bytes to read
 *
 * Read function for 8bit bus-width
 */
static void read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;
	struct nand_chip *chip = mtd->priv;
	struct nand_drv *info = (struct nand_drv *)chip->priv;

	for (i = 0; i < len; i += 4) {
		u32 value;
		size_t n = min_t(size_t, len - i, 4);

		value = CMD_GO | CMD_PIO | CMD_RX | CMD_A_VALID |
			CMD_CE0 | CMD_TRANS_SIZE(n - 1);

		writel(value, &info->reg->command);

		if (!nand_waitfor_cmd_completion(info->reg))
			puts("Command timeout during read_buf\n");

		value = readl(&info->reg->resp);
		memcpy(buf + i, &value, n);
	}
}

/**
 * Check NAND status to see if it is ready or not
 *
 * @param mtd	MTD device structure
 * @return
 *	1 - ready
 *	0 - not ready
 */
static int nand_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	int reg_val;
	struct nand_drv *info;

	info = (struct nand_drv *)chip->priv;

	reg_val = readl(&info->reg->status);
	if (reg_val & STATUS_RBSY0)
		return 1;
	else
		return 0;
}

/* Dummy implementation: we don't support multiple chips */
static void nand_select_chip(struct mtd_info *mtd, int chipnr)
{
	switch (chipnr) {
	case -1:
	case 0:
		break;

	default:
		BUG();
	}
}

/**
 * Clear all interrupt status bits
 *
 * @param reg	nand_ctlr structure
 */
static void nand_clear_interrupt_status(struct nand_ctlr *reg)
{
	u32 reg_val;

	/* Clear interrupt status */
	reg_val = readl(&reg->isr);
	writel(reg_val, &reg->isr);
}

/**
 * Send command to NAND device
 *
 * @param mtd		MTD device structure
 * @param command	the command to be sent
 * @param column	the column address for this command, -1 if none
 * @param page_addr	the page address for this command, -1 if none
 */
static void nand_command(struct mtd_info *mtd, unsigned int command,
	int column, int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_drv *nand = (struct nand_drv *)chip->priv;
	u32 value;

	switch (command) {
	case NAND_CMD_READOOB:
		column += mtd->writesize;
		/* fall-through */

	case NAND_CMD_READ0:
		writel(NAND_CMD_READ0, &nand->reg->cmd_reg1);
		writel(NAND_CMD_READSTART, &nand->reg->cmd_reg2);

		value = (page_addr << 16) | (column & 0xffff);
		writel(value, &nand->reg->addr_reg1);

		value = page_addr >> 16;
		writel(value, &nand->reg->addr_reg2);

		value = CMD_CLE | CMD_ALE | CMD_ALE_SIZE(4) | CMD_SEC_CMD |
			CMD_R_BSY_CHK | CMD_CE0 | CMD_GO;
		writel(value, &nand->reg->command);
		break;

	case NAND_CMD_SEQIN:
		writel(NAND_CMD_SEQIN, &nand->reg->cmd_reg1);

		value = (page_addr << 16) | (column & 0xffff);
		writel(value, &nand->reg->addr_reg1);

		value = page_addr >> 16;
		writel(value, &nand->reg->addr_reg2);

		value = CMD_CLE | CMD_ALE | CMD_ALE_SIZE(4) |
			CMD_CE0 | CMD_GO;
		writel(value, &nand->reg->command);
		break;

	case NAND_CMD_PAGEPROG:
		writel(NAND_CMD_PAGEPROG, &nand->reg->cmd_reg1);

		value = CMD_CLE | CMD_CE0 | CMD_GO;
		writel(value, &nand->reg->command);
		break;

	case NAND_CMD_READID:
		writel(NAND_CMD_READID, &nand->reg->cmd_reg1);
		writel(column & 0xff, &nand->reg->addr_reg1);

		value = CMD_GO | CMD_CLE | CMD_ALE | CMD_CE0;
		writel(value, &nand->reg->command);
		break;

	case NAND_CMD_ERASE1:
		writel(NAND_CMD_ERASE1, &nand->reg->cmd_reg1);
		writel(NAND_CMD_ERASE2, &nand->reg->cmd_reg2);
		writel(page_addr, &nand->reg->addr_reg1);

		value = CMD_GO | CMD_CLE | CMD_ALE | CMD_ALE_SIZE(2) |
			CMD_SEC_CMD | CMD_R_BSY_CHK | CMD_CE0;
		writel(value, &nand->reg->command);
		break;

	case NAND_CMD_ERASE2:
		return;

	case NAND_CMD_STATUS:
		writel(NAND_CMD_STATUS, &nand->reg->cmd_reg1);

		value = CMD_GO | CMD_CLE | CMD_CE0;
		writel(value, &nand->reg->command);
		break;

	case NAND_CMD_PARAM:
		writel(NAND_CMD_PARAM, &nand->reg->cmd_reg1);
		writel(column & 0xff, &nand->reg->addr_reg1);
		value = CMD_GO | CMD_CLE | CMD_ALE | CMD_CE0;
		writel(value, &nand->reg->command);
		break;

	case NAND_CMD_RESET:
		writel(NAND_CMD_RESET, &nand->reg->cmd_reg1);

		value = CMD_GO | CMD_CLE | CMD_CE0;
		writel(value, &nand->reg->command);
		break;

	default:
		printf("unsupported command: %x\n", command);
		return;
	}

	if (!nand_waitfor_cmd_completion(nand->reg))
		printf("timeout running CMD: %x\n", command);
}

/**
 * Check whether the pointed buffer are all 0xff (blank).
 *
 * @param buf	data buffer for blank check
 * @param len	length of the buffer in byte
 * @return
 *	1 - blank
 *	0 - non-blank
 */
static int blank_check(u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++)
		if (buf[i] != 0xFF)
			return 0;
	return 1;
}

/**
 * After a DMA transfer for read, we call this function to see whether there
 * is any uncorrectable error on the pointed data buffer or oob buffer.
 *
 * @param reg		nand_ctlr structure
 * @param databuf	data buffer
 * @param a_len		data buffer length
 * @param oobbuf	oob buffer
 * @param b_len		oob buffer length
 * @return
 *	ECC_OK - no ECC error or correctable ECC error
 *	ECC_TAG_ERROR - uncorrectable tag ECC error
 *	ECC_DATA_ERROR - uncorrectable data ECC error
 *	ECC_DATA_ERROR + ECC_TAG_ERROR - uncorrectable data+tag ECC error
 */
static int check_ecc_error(struct nand_ctlr *reg, u8 *databuf,
	int a_len, u8 *oobbuf, int b_len)
{
	int return_val = ECC_OK;
	u32 reg_val;

	if (!(readl(&reg->isr) & ISR_IS_ECC_ERR))
		return ECC_OK;

	/*
	 * Area A is used for the data block (databuf). Area B is used for
	 * the spare block (oobbuf)
	 */
	reg_val = readl(&reg->dec_status);
	if ((reg_val & DEC_STATUS_A_ECC_FAIL) && databuf) {
		reg_val = readl(&reg->bch_dec_status_buf);
		/*
		 * If uncorrectable error occurs on data area, then see whether
		 * they are all FF. If all are FF, it's a blank page.
		 * Not error.
		 */
		if ((reg_val & BCH_DEC_STATUS_FAIL_SEC_FLAG_MASK) &&
				!blank_check(databuf, a_len))
			return_val |= ECC_DATA_ERROR;
	}

	if ((reg_val & DEC_STATUS_B_ECC_FAIL) && oobbuf) {
		reg_val = readl(&reg->bch_dec_status_buf);
		/*
		 * If uncorrectable error occurs on tag area, then see whether
		 * they are all FF. If all are FF, it's a blank page.
		 * Not error.
		 */
		if ((reg_val & BCH_DEC_STATUS_FAIL_TAG_MASK) &&
				!blank_check(oobbuf, b_len))
			return_val |= ECC_TAG_ERROR;
	}

	return return_val;
}

/**
 * Hardware ecc based page read function
 *
 * @param mtd	mtd info structure
 * @param chip	nand chip info structure
 * @param buf	buffer to store read data
 * @param page	page number to read
 * @return	0 when successfully completed
 *		-EIO when command timeout
 */
static int nand_read_page_hwecc(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int oob_required, int page)
{
	struct nand_drv *nand = (struct nand_drv *)chip->priv;
	ALLOC_CACHE_ALIGN_BUFFER(u32, oob_buf, 128);
	uint8_t *bounce_buf = NULL;
	char *oob_ptr;
	int oob_size;
	u32 value;

	value = readl(&nand->reg->config);
	value |= CFG_HW_ECC_ENABLE | CFG_HW_ECC_CORRECTION_ENABLE;
	writel(value, &nand->reg->config);

	/* cache alignment */
	if ((uintptr_t)buf & (ARCH_DMA_MINALIGN - 1)) {
		bounce_buf = buf;
		buf = nand->data_buf;
	}

	/* Need to be 4-byte aligned */
	oob_ptr = (char *)oob_buf;

	writel(mtd->writesize - 1, &nand->reg->dma_cfg_a);
	writel(virt_to_phys(buf), &nand->reg->data_block_ptr);

	if (oob_required) {
		oob_size = chip->ecc.layout->oobfree[0].length - 1;
		writel(oob_size, &nand->reg->dma_cfg_b);
		writel(virt_to_phys(oob_ptr), &nand->reg->tag_ptr);
		dma_prepare(oob_ptr, oob_size, 0);

	} else {
		writel(0, &nand->reg->dma_cfg_b);
		writel(0, &nand->reg->tag_ptr);
	}

	dma_prepare(buf, mtd->writesize, 0);
	nand_clear_interrupt_status(nand->reg);

	value = DMA_MST_CTRL_GO_ENABLE | DMA_MST_CTRL_DIR_READ | DMA_MST_CTRL_PERF_EN_ENABLE |
		DMA_MST_CTRL_REUSE_BUFFER_ENABLE | DMA_MST_CTRL_IE_DONE | DMA_MST_CTRL_IS_DMA_DONE |
		DMA_MST_CTRL_BURST_8WORDS | DMA_MST_CTRL_EN_A_ENABLE;
	if (oob_required)
		value |= DMA_MST_CTRL_EN_B_ENABLE;
	writel(value, &nand->reg->dma_mst_ctrl);

	value = CMD_GO | CMD_RX | CMD_TRANS_SIZE(8) |
		CMD_A_VALID | CMD_CE0;
	if (oob_required)
		value |= CMD_B_VALID;
	writel(value, &nand->reg->command);

	if (!nand_waitfor_cmd_completion(nand->reg)) {
		printf("Read page timeout\n");
		return -EIO;
	}

	if (oob_required)
		memcpy(chip->oob_poi,
		       oob_buf + chip->ecc.layout->oobfree[0].offset,
		       chip->ecc.layout->oobfree[0].length);

	/* cache alignment */
	if (bounce_buf) {
		memcpy(bounce_buf, buf, mtd->writesize);
	}

	value = readl(&nand->reg->config);
	value &= ~(CFG_HW_ECC_ENABLE | CFG_HW_ECC_CORRECTION_ENABLE);
	writel(value, &nand->reg->config);

	return check_ecc_error(nand->reg, (u8 *)buf,
			mtd->writesize,
			(u8 *)(oob_buf + chip->ecc.layout->oobfree[0].offset),
			chip->ecc.layout->oobfree[0].length);
}

/**
 * Hardware ecc based page write function
 *
 * @param mtd	mtd info structure
 * @param chip	nand chip info structure
 * @param buf	data buffer
 */
static int nand_write_page_hwecc(struct mtd_info *mtd,
	struct nand_chip *chip, const uint8_t *buf, int oob_required)
{
	struct nand_drv *nand = (struct nand_drv *)chip->priv;
	ALLOC_CACHE_ALIGN_BUFFER(u32, oob_buf, 128);
	char *oob_ptr;
	int oob_size;
	unsigned long value;

	value = readl(&nand->reg->config);
	value |= CFG_HW_ECC_ENABLE | CFG_HW_ECC_CORRECTION_ENABLE;
	writel(value, &nand->reg->config);

	/* Need to be 4-byte aligned */
	oob_ptr = (char *)oob_buf;

	writel(mtd->writesize - 1, &nand->reg->dma_cfg_a);
	writel(virt_to_phys((uint8_t *)buf), &nand->reg->data_block_ptr);

	if (oob_required) {
		oob_size = chip->ecc.layout->oobfree[0].length - 1;
		memcpy(oob_buf,
		       chip->oob_poi + chip->ecc.layout->oobfree[0].offset,
		       chip->ecc.layout->oobfree[0].length);
		writel(oob_size, &nand->reg->dma_cfg_b);
		writel(virt_to_phys(oob_ptr), &nand->reg->tag_ptr);
		dma_prepare(oob_ptr, oob_size, 1);
	} else {
		writel(0, &nand->reg->dma_cfg_b);
		writel(0, &nand->reg->tag_ptr);
	}

	dma_prepare((uint8_t *)buf, mtd->writesize, 1);

	nand_clear_interrupt_status(nand->reg);

	value = DMA_MST_CTRL_GO_ENABLE | DMA_MST_CTRL_DIR_WRITE | DMA_MST_CTRL_PERF_EN_ENABLE |
		DMA_MST_CTRL_IE_DONE | DMA_MST_CTRL_IS_DMA_DONE |
		DMA_MST_CTRL_BURST_8WORDS | DMA_MST_CTRL_EN_A_ENABLE;
	if (oob_required)
		value |= DMA_MST_CTRL_EN_B_ENABLE;
	writel(value, &nand->reg->dma_mst_ctrl);

	value = CMD_GO | CMD_TX | CMD_A_VALID | CMD_TRANS_SIZE(8) |
		CMD_CE0;
	if (oob_required)
		value |= CMD_B_VALID;
	writel(value, &nand->reg->command);

	if (!nand_waitfor_cmd_completion(nand->reg)) {
		printf("Write page timeout\n");
		return -EIO;
	}

	value = readl(&nand->reg->config);
	value &= ~(CFG_HW_ECC_ENABLE | CFG_HW_ECC_CORRECTION_ENABLE);
	writel(value, &nand->reg->config);

	return 0;
}

/**
 * Set up NAND memory timings according to the provided parameters
 *
 * @param timing	Timing parameters
 * @param reg		NAND controller register address
 */
static void setup_timing(unsigned timing[FDT_NAND_TIMING_COUNT],
			 struct nand_ctlr *reg)
{
	u32 reg_val, clk_rate, clk_period, time_val;

	clk_rate = (u32)clock_get_periph_rate(PERIPH_ID_NDFLASH,
		CLOCK_ID_PERIPH) / 1000000;
	clk_period = 1000 / clk_rate;
	reg_val = ((timing[FDT_NAND_MAX_TRP_TREA] / clk_period) <<
		TIMING_TRP_RESP_CNT_SHIFT) & TIMING_TRP_RESP_CNT_MASK;
	reg_val |= ((timing[FDT_NAND_TWB] / clk_period) <<
		TIMING_TWB_CNT_SHIFT) & TIMING_TWB_CNT_MASK;
	time_val = timing[FDT_NAND_MAX_TCR_TAR_TRR] / clk_period;
	if (time_val > 2)
		reg_val |= ((time_val - 2) << TIMING_TCR_TAR_TRR_CNT_SHIFT) &
			TIMING_TCR_TAR_TRR_CNT_MASK;
	reg_val |= ((timing[FDT_NAND_TWHR] / clk_period) <<
		TIMING_TWHR_CNT_SHIFT) & TIMING_TWHR_CNT_MASK;
	time_val = timing[FDT_NAND_MAX_TCS_TCH_TALS_TALH] / clk_period;
	if (time_val > 1)
		reg_val |= ((time_val - 1) << TIMING_TCS_CNT_SHIFT) &
			TIMING_TCS_CNT_MASK;
	reg_val |= ((timing[FDT_NAND_TWH] / clk_period) <<
		TIMING_TWH_CNT_SHIFT) & TIMING_TWH_CNT_MASK;
	reg_val |= ((timing[FDT_NAND_TWP] / clk_period) <<
		TIMING_TWP_CNT_SHIFT) & TIMING_TWP_CNT_MASK;
	reg_val |= ((timing[FDT_NAND_TRH] / clk_period) <<
		TIMING_TRH_CNT_SHIFT) & TIMING_TRH_CNT_MASK;
	reg_val |= ((timing[FDT_NAND_MAX_TRP_TREA] / clk_period) <<
		TIMING_TRP_CNT_SHIFT) & TIMING_TRP_CNT_MASK;
	writel(reg_val, &reg->timing);

	reg_val = 0;
	time_val = timing[FDT_NAND_TADL] / clk_period;
	if (time_val > 2)
		reg_val = (time_val - 2) & TIMING2_TADL_CNT_MASK;
	writel(reg_val, &reg->timing2);
}

/**
 * Decode NAND parameters from the device tree
 *
 * @param blob	Device tree blob
 * @param node	Node containing "nand-flash" compatble node
 * @return 0 if ok, -ve on error (FDT_ERR_...)
 */
static int fdt_decode_nand(const void *blob, int node, struct fdt_nand *config)
{
	int err;

	config->reg = (struct nand_ctlr *)fdtdec_get_addr(blob, node, "reg");
	config->enabled = fdtdec_get_is_enabled(blob, node);
	config->width = fdtdec_get_int(blob, node, "nvidia,nand-width", 8);
	err = gpio_request_by_name_nodev(blob, node, "nvidia,wp-gpios", 0,
				 &config->wp_gpio, GPIOD_IS_OUT);
	if (err)
		return err;
	err = fdtdec_get_int_array(blob, node, "nvidia,timing",
			config->timing, FDT_NAND_TIMING_COUNT);
	if (err < 0)
		return err;

	/* Now look up the controller and decode that */
	node = fdt_next_node(blob, node, NULL);
	if (node < 0)
		return node;

	return 0;
}

/**
 * Board-specific NAND initialization
 *
 * @param nand	nand chip info structure
 * @return 0, after initialized, -1 on error
 */
int tegra_nand_init(struct nand_chip *nand, int devnum)
{
	struct nand_drv *info = &nand_ctrl;
	struct fdt_nand *config = &info->config;
	int node, ret;
	u32 value;

	node = fdtdec_next_compatible(gd->fdt_blob, 0,
				      COMPAT_NVIDIA_TEGRA20_NAND);
	if (node < 0)
		return -1;
	if (fdt_decode_nand(gd->fdt_blob, node, config)) {
		printf("Could not decode nand-flash in device tree\n");
		return -1;
	}
	if (!config->enabled)
		return -1;

	/* Adjust controller clock rate */
	clock_start_periph_pll(PERIPH_ID_NDFLASH, CLOCK_ID_PERIPH, 52000000);

	info->reg = config->reg;

	/* Adjust timing for NAND device */
	setup_timing(config->timing, info->reg);

	dm_gpio_set_value(&config->wp_gpio, 1);

	/* reset config */
	writel(0, &info->reg->config);

	nand->options = NAND_NO_SUBPAGE_WRITE;
	nand->cmdfunc = nand_command;
	nand->read_byte = read_byte;
	nand->read_buf = read_buf;
	nand->select_chip = nand_select_chip;
	nand->dev_ready  = nand_dev_ready;
	nand->priv = &nand_ctrl;

	our_mtd = &nand_info[devnum];
	our_mtd->priv = nand;
	ret = nand_scan_ident(our_mtd, CONFIG_SYS_NAND_MAX_CHIPS, NULL);
	if (ret)
		return ret;

	nand->bbt_options = NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB |
				    NAND_BBT_CREATE;

	nand->ecc.mode = NAND_ECC_HW;
	nand->ecc.size = 512;
	nand->ecc.bytes = our_mtd->oobsize;
	nand->ecc.read_page = nand_read_page_hwecc;
	nand->ecc.write_page = nand_write_page_hwecc;

	value =  CFG_PIPELINE_EN_ENABLE | CFG_SKIP_SPARE_ENABLE | CFG_SKIP_SPARE_SEL_4;

	if (config->width == 8)
		value |= CFG_BUS_WIDTH_8BIT;
	else if (config->width == 16)
		value |= CFG_BUS_WIDTH_16BIT;
	else {
		debug("%s: Unsupported bus width %d\n", __func__,
		      config->width);
		return -1;
	}

	switch (our_mtd->oobsize) {
	case 16:
		nand->ecc.layout = &tegra_nand_oob_16;
		nand->ecc.strength = 1;
		value |= CFG_TAG_BYTE_SIZE(tegra_nand_oob_16.oobfree[0].length
			 - 1);
		break;
	case 64:
		nand->ecc.layout = &tegra_nand_oob_64;
		nand->ecc.strength = 8;
		value |= CFG_HW_ECC_SEL_RS | CFG_TVAL8 |
			 CFG_TAG_BYTE_SIZE(tegra_nand_oob_64.oobfree[0].length
			 - 1);
		break;
	case 128:
		nand->ecc.layout = &tegra_nand_oob_128;
		nand->ecc.strength = 8;
		value |= CFG_HW_ECC_SEL_RS | CFG_TVAL8 |
			 CFG_TAG_BYTE_SIZE(tegra_nand_oob_128.oobfree[0].length
			 - 1);
		break;
	case 224:
		nand->ecc.layout = &tegra_nand_oob_224;
		nand->ecc.strength = 8;
		value |= CFG_HW_ECC_SEL_RS | CFG_TVAL8 |
			 CFG_TAG_BYTE_SIZE(tegra_nand_oob_224.oobfree[0].length
			 - 1);
		break;
	default:
		dev_err(&pdev->dev, "unhandled OOB size %d\n", our_mtd->oobsize);
		return -ENODEV;
	}

	switch (our_mtd->writesize) {
	case 256:
		value |= CFG_PAGE_SIZE_256;
		break;
	case 512:
		value |= CFG_PAGE_SIZE_512;
		break;
	case 1024:
		value |= CFG_PAGE_SIZE_1024;
		break;
	case 2048:
		value |= CFG_PAGE_SIZE_2048;
		break;
	case 4096:
		value |= CFG_PAGE_SIZE_4096;
		break;
	default:
		dev_err(&pdev->dev, "unhandled writesize %d\n", our_mtd->writesize);
		return -ENODEV;
	}

	writel(value, &info->reg->config);

	/* cache alignment */
	info->data_buf = memalign(ARCH_DMA_MINALIGN, our_mtd->writesize);
	if (!info->data_buf)
		return -ENOMEM;

	ret = nand_scan_tail(our_mtd);
	if (ret)
		return ret;

	ret = nand_register(devnum);
	if (ret)
		return ret;

	return 0;
}

void board_nand_init(void)
{
	struct nand_chip *nand = &nand_chip[0];

	if (tegra_nand_init(nand, 0))
		puts("Tegra NAND init failed\n");
}
