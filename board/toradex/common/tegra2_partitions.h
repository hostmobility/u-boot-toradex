#ifndef _TEGRA2_PARTITIONS_H_
#define _TEGRA2_PARTITIONS_H_

#define TEGRA_MAX_PARTITIONS	24

typedef struct {
	unsigned id;
	char name[4];
	unsigned allocation_policy;
	unsigned _unknown1[2];
	char name2[4];
	unsigned filesystem_type;
	unsigned _unknown2[3];
	unsigned virtual_start_sector;
	unsigned _unknown3;
	unsigned virtual_size;
	unsigned _unknown4;
	unsigned start_sector;
	unsigned _unknown5;
	unsigned end_sector;
	unsigned _unknown6[2];
	unsigned type;
} __attribute__ ((packed)) nvtegra_partinfo_t;

typedef struct {
	unsigned _unknown[18];
	nvtegra_partinfo_t partinfo[TEGRA_MAX_PARTITIONS];
} __attribute__ ((packed)) nvtegra_parttable_t;

void nvtegra_print_partition_table(nvtegra_parttable_t * pt);
int nvtegra_read_partition_table(nvtegra_parttable_t * pt);
int nvtegra_find_partition(nvtegra_parttable_t * pt, const char *name,
			   nvtegra_partinfo_t ** partinfo);
int nvtegra_mtdparts_string(char *output, int size);
void tegra_partition_init(void);

#endif /* _TEGRA2_PARTITIONS_H_ */

