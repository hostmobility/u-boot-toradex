/*
 * Copyright (c) 2011-2014 Toradex, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#include <malloc.h>
#include <mmc.h>
#include <nand.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_REVISION_TAG) || defined(CONFIG_SERIAL_TAG) || defined(CONFIG_TRDX_CFG_BLOCK)
unsigned char *config_block = NULL;
#endif

#ifdef CONFIG_REVISION_TAG
u32 get_board_rev(void)
{
	int i;
	unsigned short major = 0, minor = 0, release = 0;
	size_t size = CONFIG_TRDX_CFG_BLOCK_SIZE;

	if (config_block == NULL)
		return 0;

	/* Parse revision information in config block */
	for (i = 0; i < (size - 8); i++) {
		if (config_block[i] == 0x02 && config_block[i+1] == 0x40 &&
		    config_block[i+2] == 0x08) {
			break;
		}
	}

	/* Parse revision information in config block */
	major = (config_block[i+3] << 8) | config_block[i+4];
	minor = (config_block[i+5] << 8) | config_block[i+6];
	release = (config_block[i+7] << 8) | config_block[i+8];

	/* Check validity */
	if (major)
		return ((major & 0xff) << 8) | ((minor & 0xf) << 4) |
		       ((release & 0xf) + 0xa);
	else
		return 0;
}
#endif /* CONFIG_REVISION_TAG */

#ifdef CONFIG_SERIAL_TAG
void get_board_serial(struct tag_serialnr *serialnr)
{
	int array[8];
	int i;
	unsigned int serial = 0;
	unsigned int serial_offset = 11;

	serialnr->low = 0;
	serialnr->high = 0;

	if (config_block == NULL)
		return;

	/* Get MAC address from config block */
	memcpy(&serial, config_block + serial_offset, 3);
	serial = ntohl(serial);
	serial >>= 8;

	/* Check validity */
	if (serial) {
		/*
		 * Convert to Linux serial number format (hexadecimal coded
		 * decimal)
		 */
		i = 7;
		while (serial) {
			array[i--] = serial % 10;
			serial /= 10;
		}
		while (i >= 0)
			array[i--] = 0;
		serial = array[0];
		for (i = 1; i < 8; i++) {
			serial *= 16;
			serial += array[i];
		}

		serialnr->low = serial;
	}
}
#endif /* CONFIG_SERIAL_TAG */

#ifdef CONFIG_TRDX_CFG_BLOCK
int read_trdx_cfg_block(void)
{
	size_t size = CONFIG_TRDX_CFG_BLOCK_SIZE;
#ifdef CONFIG_TRDX_CFG_BLOCK_IS_IN_MMC
	struct mmc *mmc;
	int dev = CONFIG_TRDX_CFG_BLOCK_DEV;
	uint part = CONFIG_TRDX_CFG_BLOCK_PART;
	uint blk_start;
#endif /* CONFIG_TRDX_CFG_BLOCK_IS_IN_MMC */
	int offset = CONFIG_TRDX_CFG_BLOCK_OFFSET;
	unsigned char *cfg_block_ethaddr;
	unsigned char toradex_oui[3] = { 0x00, 0x14, 0x2d };
	unsigned char ethaddr[6];

	/* Allocate RAM area for config block */
	config_block = malloc(size);
	if (!config_block) {
		printf("Not enough malloc space available!\n");
		return -1;
	}

	/* Clear it */
	memset((void *)config_block, 0, size);

#ifdef CONFIG_TRDX_CFG_BLOCK_IS_IN_MMC
	/* Read production parameter config block from eMMC */
	mmc = find_mmc_device(dev);
	if (!mmc) {
		puts("No MMC card found\n");
		return 1;
	}
	if (part != mmc->part_num) {
		if (mmc_switch_part(dev, part)) {
			puts("MMC partition switch failed\n");
			return 1;
		}
	}
	if (offset < 0)
		offset += mmc->capacity;
	blk_start = ALIGN(offset, mmc->write_bl_len) / mmc->write_bl_len;

	/* Just reading one 512 byte block */
	if (mmc->block_dev.block_read(dev, blk_start, 1,
				      (unsigned char *)config_block) != 1) {
		return 1;
	}

	/* Switch back to regular eMMC user partition */
	mmc_switch_part(0, 0);
#endif /* CONFIG_TRDX_CFG_BLOCK_IS_IN_MMC */
#ifdef CONFIG_TRDX_CFG_BLOCK_IS_IN_NAND
	/* Read production parameter config block from first NAND block */
	if (nand_read_skip_bad(&nand_info[0], CONFIG_TRDX_CFG_BLOCK_OFFSET,
			       &size, NULL, nand_info[0].size, config_block))
		return 1;
#endif

	/* Check validity */
	cfg_block_ethaddr = config_block + 8;
	if (memcmp(cfg_block_ethaddr, toradex_oui, 3)) {
		memset((void *)config_block, 0, size);
		return 2;
	}

	/*
	 * Check if environment contains a valid MAC address, set the one from
	 * config block if not
	 */
	if (!eth_getenv_enetaddr("ethaddr", ethaddr))
		eth_setenv_enetaddr("ethaddr", cfg_block_ethaddr);

	return 0;
}
#endif /* CONFIG_TRDX_CFG_BLOCK */
