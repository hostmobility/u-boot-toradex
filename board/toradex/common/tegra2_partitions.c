#include <common.h>
#include <asm/io.h>
#include <nand.h>
#include <malloc.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/gpio.h>
#include <asm/arch-tegra/ap20.h>
#include <asm/errno.h>
#include <fdt_decode.h>
#include "tegra2_nand.h"
#include "tegra2_partitions.h"

#define DEBUG 0

DECLARE_GLOBAL_DATA_PTR;

/**
 * nvtegra_print_partition_table - prints partition table info
 * @param pt:  nvtegra_parttable_t structure
 */
void nvtegra_print_partition_table(nvtegra_parttable_t * pt)
{
	int i;
	nvtegra_partinfo_t *p;

	// sanity check
	if (!pt) {
		printf("%s: Error! pt arg is NULL\n", __FUNCTION__);
		return;
	}

	p = &(pt->partinfo[0]);
	printf("\n------ NVTEGRA Partitions ------\n");

#if DEBUG > 1
	printf("UNK:\n");
	for (i = 0; i < 18; i++) {
		if (pt->_unknown[i] > 1000000)
			printf("0x%08X\t", pt->_unknown[i]);
		else
			printf("%u\t", pt->_unknown[i]);
		if ((i + 1) % 6 == 0)
			printf("\n");
	}
	printf("\n");
#endif

	for (i = 0; (p->id < 128) && (i < TEGRA_MAX_PARTITIONS); i++) {
		printf
		    ("\n[%u]\tPart:\tID=%u\tNAME=%s\tNAME2=%s\n\t\tTYPE=%u\tALLOC_POLICY=%u\tFS_TYPE=%u\n\t\tVIRTUAL START SEC=%u\tVIRTUAL SIZE=%u\n",
		     i, p->id, p->name, p->name2, p->type, p->allocation_policy,
		     p->filesystem_type, p->virtual_start_sector,
		     p->virtual_size);
		printf("\t\tSTART SEC=%u\tEND SEC=%u\tTOTAL=%u\n",
		       p->start_sector, p->end_sector,
		       p->end_sector + 1 - p->start_sector);

#if DEBUG > 1
		printf("\t\tUNK1=%u\t%u\n", p->_unknown1[0], p->_unknown1[1]);
		printf("\t\tUNK2=%u\t%u\t0x%08x\n", p->_unknown2[0],
		       p->_unknown2[1], p->_unknown2[2]);
		printf("\t\tUNK3=%u\n", p->_unknown3);
		printf("\t\tUNK4=%u\n", p->_unknown4);
		printf("\t\tUNK5=%u\n", p->_unknown5);
		printf("\t\tUNK6=%u\t%u\n", p->_unknown6[0], p->_unknown6[1]);
#endif

		p++;
	}
	printf("--------------------------------\n");
}

/**
 * nvtegra_read_partition_table - reads nvidia's partition table
 * @return:
 *  1 - Success
 *  0 - Error
 */
int nvtegra_read_partition_table(nvtegra_parttable_t * pt)
{
	size_t size;
	int i;
	nvtegra_partinfo_t *p;
	u32 bct_start, pt_logical, pt_offset;

	// sanity check
	if (!pt) {
		printf("%s: Error! pt arg is NULL\n", __FUNCTION__);
		return 0;
	}

	/*
	 * Partition table offset is stored in the BCT in IRAM by the BootROM.
	 * The BCT start and size are stored in the BIT in IRAM.
	 * Read the data @ bct_start + (bct_size - 260). This works
	 * on T20 and T30 BCTs, which are locked down. If this changes
	 * in new chips (T114, etc.), we can revisit this algorithm.
	 */
	bct_start = readl(AP20_BASE_PA_SRAM + NVBOOTINFOTABLE_BCTPTR);
#if DEBUG > 1
	printf("bct_start=0x%08x\n", bct_start);
#endif
	pt_logical = readw(bct_start + BCT_PTINFO_OFFSET);
	/* In case we are running with a recovery BCT missing the partition
	   table offset information */
#ifdef CONFIG_ENV_IS_IN_MMC
	if (1) {
#else
	if (pt_logical == 0) {
#endif
		/* BCT partition size is 3 M in our default layout */
		pt_logical = 3 * 1024 * 1024 / nand_info->writesize;
	}
	/* StartLogicalSector * PageSize + 4 * BlockSize */
	pt_offset = pt_logical * nand_info->writesize + 4 * nand_info->erasesize;
#if DEBUG > 1
	printf("logical=0x%08x writesize=0x%08x erasesize=0x%08x\n", readw(bct_start + BCT_PTINFO_OFFSET), nand_info->writesize, nand_info->erasesize);
	printf("pt_offset=0x%08x\n", pt_offset);
#endif

	size = sizeof(nvtegra_parttable_t);
	i = nand_read_skip_bad(&nand_info[0], pt_offset, &size,
			       (unsigned char *)pt);
	if ((i != 0) || (size != sizeof(nvtegra_parttable_t))) {
		printf
		    ("%s: Error! nand_read_skip_bad failed. nand_info->writesize=%d ret=%d\n",
		     __FUNCTION__, nand_info->writesize, i);
		return 0;
	}

	/* some heuristics */
	p = &(pt->partinfo[0]);
	if ((p->id != 2) || memcmp(p->name, "BCT\0", 4)
	    || memcmp(p->name2, "BCT\0", 4) || (p->virtual_start_sector != 0)) {
		printf
		    ("%s: Error! Partition table offset is probably incorrect. name='%s'\n",
		     __FUNCTION__, p->name);
		return 0;
	}

	return 1;
}

/**
 * nvtegra_find_partition - parse nvidia partition table
 * @param pt:  nvtegra_parttable_t structure
 * @param name:  partition name
 * @param partinfo:  output pointer to nvtegra_partinfo_t structure
 * @return:
 *  1 - Success, partinfo contains partition info
 *  0 - Not found or error
 */
int nvtegra_find_partition(nvtegra_parttable_t * pt, const char *name,
			   nvtegra_partinfo_t ** partinfo)
{
	int i, l;
	nvtegra_partinfo_t *p;

	// sanity checks
	if (!pt) {
		printf("%s: Error! pt arg is NULL\n", __FUNCTION__);
		return 0;
	}
	if (!name) {
		printf("%s: Error! name arg is NULL\n", __FUNCTION__);
		return 0;
	}
	if (!partinfo) {
		printf("%s: Error! partinfo arg is NULL\n", __FUNCTION__);
		return 0;
	}

	p = &(pt->partinfo[0]);
	l = strlen(name) + 1;	// string length + \0
	for (i = 0; (p->id < 128) && (i < TEGRA_MAX_PARTITIONS); i++) {
		if (memcmp(p->name, name, l) == 0
		    || memcmp(p->name2, name, l) == 0) {
			*partinfo = p;
			return 1;
		}
		p++;
	}

	printf("%s: Error! Partition '%s' not found.\n", __FUNCTION__, name);
	return 0;
}

int nvtegra_mtdparts_string(char *output, int size)
{
	int i, j = 0;
	nvtegra_parttable_t *pt;
	nvtegra_partinfo_t *p, *usr;
	char buffer[512];

	// sanity checks
	if (!output) {
		printf("%s: Error! output arg is NULL.\n", __FUNCTION__);
		return 0;
	}
	if (size > 256) {
		printf
		    ("%s: Error! size is too large. Increase buffer size here.\n",
		     __FUNCTION__);
		return 0;
	}
	// parse nvidia partition table
	pt = malloc(sizeof(nvtegra_parttable_t));
	if (!pt) {
		printf("%s: Error calling malloc(%d)\n", __FUNCTION__,
		       sizeof(nvtegra_parttable_t));
		return 0;
	}

	if (!nvtegra_read_partition_table(pt)) {
		free(pt);
		return 0;
	}
	// make rootfs partition the first in the list
	if (nvtegra_find_partition(pt, "USR", &p)) {
		sprintf(buffer + j, "%uK@%uK(USR)",
			p->virtual_size * nand_info->writesize / 1024,
			p->start_sector * nand_info->writesize / 1024);
		j += strlen(buffer + j);
	}
	usr = p;

	p = &(pt->partinfo[0]);
	for (i = 0; (p->id < 128) && (i < TEGRA_MAX_PARTITIONS); i++) {
		if (p != usr) {
			/* add coma separator after previous entries */
			if (j > 0) {
				sprintf(buffer + j, ",");
				j++;
			}

			if (strlen(p->name))
				sprintf(buffer + j, "%uK@%uK(%s)",
					p->virtual_size *
					nand_info->writesize / 1024,
					p->start_sector *
					nand_info->writesize / 1024, p->name);
			else
				sprintf(buffer + j, "%uK@%uK",
					p->virtual_size *
					nand_info->writesize / 1024,
					p->start_sector *
					nand_info->writesize / 1024);

			j += strlen(buffer + j);
		}
		if (strlen(buffer) >= size)
			break;
		p++;
	}

	memcpy(output, buffer, (j < size ? j + 1 : size));
	free(pt);
	return 1;
}

void tegra_partition_init(void)
{
	nvtegra_parttable_t *pt;
	nvtegra_partinfo_t *partinfo;

	// parse nvidia partition table
	pt = malloc(sizeof(nvtegra_parttable_t));
	if (!pt) {
		printf("%s: Error calling malloc(%d)\n", __FUNCTION__,
		       sizeof(nvtegra_parttable_t));
		return;
	}
	//copy partition information to global data
	if (!nvtegra_read_partition_table(pt)) {
		free(pt);
		return;
	}

	if (nvtegra_find_partition(pt, "ENV", &partinfo)) {
		gd->env_offset =
		    partinfo->start_sector * nand_info->writesize;
#if DEBUG > 1
		printf
		    ("\nPart:\tID=%u\tNAME=%s\tNAME2=%s\n\t\tTYPE=%u\tALLOC_POLICY=%u\tFS_TYPE=%u\n\t\tVIRTUAL START SEC=%u\tVIRTUAL SIZE=%u\n",
		     partinfo->id, partinfo->name, partinfo->name2,
		     partinfo->type, partinfo->allocation_policy,
		     partinfo->filesystem_type, partinfo->virtual_start_sector,
		     partinfo->virtual_size);
		printf("\t\tSTART SEC=%u\tEND SEC=%u\tTOTAL=%u\n",
		       partinfo->start_sector, partinfo->end_sector,
		       partinfo->end_sector + 1 - partinfo->start_sector);
#endif
	}

	if (nvtegra_find_partition(pt, "ARG", &partinfo)) {
		gd->conf_blk_offset =
		    partinfo->start_sector * nand_info->writesize;
#if DEBUG > 1
		printf
		    ("\nPart:\tID=%u\tNAME=%s\tNAME2=%s\n\t\tTYPE=%u\tALLOC_POLICY=%u\tFS_TYPE=%u\n\t\tVIRTUAL START SEC=%u\tVIRTUAL SIZE=%u\n",
		     partinfo->id, partinfo->name, partinfo->name2,
		     partinfo->type, partinfo->allocation_policy,
		     partinfo->filesystem_type, partinfo->virtual_start_sector,
		     partinfo->virtual_size);
		printf("\t\tSTART SEC=%u\tEND SEC=%u\tTOTAL=%u\n",
		       partinfo->start_sector, partinfo->end_sector,
		       partinfo->end_sector + 1 - partinfo->start_sector);
#endif
	}

	if (nvtegra_find_partition(pt, "LNX", &partinfo)) {
		gd->kernel_offset =
		    partinfo->start_sector * nand_info->writesize;
#if DEBUG > 1
		printf
		    ("\nPart:\tID=%u\tNAME=%s\tNAME2=%s\n\t\tTYPE=%u\tALLOC_POLICY=%u\tFS_TYPE=%u\n\t\tVIRTUAL START SEC=%u\tVIRTUAL SIZE=%u\n",
		     partinfo->id, partinfo->name, partinfo->name2,
		     partinfo->type, partinfo->allocation_policy,
		     partinfo->filesystem_type, partinfo->virtual_start_sector,
		     partinfo->virtual_size);
		printf("\t\tSTART SEC=%u\tEND SEC=%u\tTOTAL=%u\n",
		       partinfo->start_sector, partinfo->end_sector,
		       partinfo->end_sector + 1 - partinfo->start_sector);
#endif
	}

	if (nvtegra_find_partition(pt, "USR", &partinfo)) {
#if DEBUG > 1
		printf
		    ("\nPart:\tID=%u\tNAME=%s\tNAME2=%s\n\t\tTYPE=%u\tALLOC_POLICY=%u\tFS_TYPE=%u\n\t\tVIRTUAL START SEC=%u\tVIRTUAL SIZE=%u\n",
		     partinfo->id, partinfo->name, partinfo->name2,
		     partinfo->type, partinfo->allocation_policy,
		     partinfo->filesystem_type, partinfo->virtual_start_sector,
		     partinfo->virtual_size);
		printf("\t\tSTART SEC=%u\tEND SEC=%u\tTOTAL=%u\n",
		       partinfo->start_sector, partinfo->end_sector,
		       partinfo->end_sector + 1 - partinfo->start_sector);
#endif
	}
#if DEBUG > 1
	printf("gd->env_offset=%u\n", gd->env_offset);
	printf("gd->conf_blk_offset=%u\n", gd->conf_blk_offset);
	printf("gd->kernel_offset=%u\n", gd->kernel_offset);
#endif

#if DEBUG > 0
	nvtegra_print_partition_table(pt);
#endif

	free(pt);
}

