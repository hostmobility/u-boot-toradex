/*
 * Copyright (c) 2011, Google Inc. All rights reserved.
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


/*
 * This module records the progress of boot and arbitrary commands, and
 * permits accurate timestamping of each. The records can optionally be
 * passed to kernel in the ATAGs
 */

#include <common.h>
#include <libfdt.h>
#include <fdt_decode.h>

DECLARE_GLOBAL_DATA_PTR;

struct bootstage_record {
	uint32_t time_us;
	uint32_t start_us;
	const char *name;
};

static struct bootstage_record record[BOOTSTAGE_COUNT] = {{1, 0, "avoid_bss"} };

uint32_t bootstage_mark(enum bootstage_id id, const char *name)
{
	struct bootstage_record *rec = &record[id];

	/* Only record the first event for each */
	if (!rec->name) {
		rec->time_us = (uint32_t)timer_get_us();
		rec->start_us = 0;
		rec->name = name;
	}
	return rec->time_us;
}

uint32_t bootstage_start(enum bootstage_id id, const char *name)
{
	struct bootstage_record *rec = &record[id];

	rec->start_us = timer_get_us();
	rec->name = name;
	return rec->start_us;
}

uint32_t bootstage_accum(enum bootstage_id id)
{
	struct bootstage_record *rec = &record[id];
	uint32_t duration;

	duration = (uint32_t)timer_get_us() - rec->start_us;
	rec->time_us += duration;
	return duration;
}

static void print_time(unsigned long us_time)
{
	char str[12], *s;
	int grab = 3;

	/* We don't seem to have %'d in U-Boot */
	sprintf(str, "%9ld", us_time);
	for (s = str; *s; s += grab) {
		if (s != str)
			putc(s[-1] != ' ' ? ',' : ' ');
		printf("%.*s", grab, s);
		grab = 3;
	}
}

static uint32_t print_time_record(enum bootstage_id id,
			struct bootstage_record *rec, uint32_t prev)
{
	if (prev == -1U) {
		printf("%11s", "");
		print_time(rec->time_us);
	} else {
		print_time(rec->time_us);
		print_time(rec->time_us - prev);
	}
	if (rec->name)
		printf("  %s\n", rec->name);
	else
		printf("  id=%d\n", id);
	return rec->time_us;
}

static int h_compare_record(const void *r1, const void *r2)
{
	const struct bootstage_record *rec1 = r1, *rec2 = r2;

	return rec1->time_us - rec2->time_us;
}

#define BOOTSTAGE_NODE "bootstage"

/**
 * This function add a new node to the device tree.
 *
 * @param pathp	the path where the new node is added.
 * @param nodep	the name of the new node
 * @return 0 on success, != 0 on failure.
 */
static int add_new_node(struct fdt_header *fdt,
		const char *pathp, const char *nodep)
{
	int nodeoffset;
	int err;

	nodeoffset = fdt_path_offset(fdt, pathp);
	if (nodeoffset < 0)
		return nodeoffset;
	err = fdt_add_subnode(working_fdt, nodeoffset, nodep);
	if (err < 0)
		return err;

	return 0;
}

/**
 * this function add all bootstage timings to the device tree.
 *
 * @param fdt	the point to the device tree.
 * @return 0 on success, != 0 on failure.
 */
static int add_bootstages_devicetree(struct fdt_header *fdt)
{
	int id;
	int i;
	int ret = 0;

	if (fdt == NULL)
		return 0;

	/*
	 * Create the node for bootstage.
	 * The address of flat device tree is set up by the command bootm.
	 */
	ret = add_new_node(fdt, "/", BOOTSTAGE_NODE);

	/*
	 * insert the timings to the device tree in the reverse order,
	 * so that they can be printed in the Linux kernel in the right order.
	 */
	for (id = BOOTSTAGE_COUNT - 1, i = 0; id >= 0 && ret == 0; id--) {
		struct bootstage_record *rec = &record[id];
		static char path[32];
		static char node[8];
		int nodeoffset;

		sprintf(path, "/%s/", BOOTSTAGE_NODE);
		if (id != BOOTSTAGE_AWAKE && rec->time_us == 0)
			continue;

		sprintf(node, "%d", i);
		if ((ret = add_new_node(fdt, path, node)))
			break;

		/* add properties to the node. */
		strncat(path, node, sizeof(path) - strlen(path));
		nodeoffset = fdt_path_offset(fdt, path);
		if (nodeoffset < 0)
			ret = nodeoffset;
		if (!ret)
			ret = fdt_setprop_string(fdt, nodeoffset, "name", rec->name);
		if (!ret)
			ret = fdt_setprop_cell(fdt, nodeoffset, "time", rec->time_us);

		i++;
	}
	if (ret)
		printf("add_bootstages: failed to add to device tree\n");
	return ret;
}

void bootstage_report(void)
{
	struct bootstage_record *rec = record;
	int id;
	uint32_t prev;
	u32 flags = gd->flags;

#ifdef CONFIG_OF_CONTROL
	/* Force this information to display even on silent console */
	if (fdt_decode_get_config_int(gd->blob, "bootstage-force-report", 0))
		gd->flags &= ~GD_FLG_SILENT;
#endif
	puts("Timer summary in microseconds:\n");
	printf("%11s%11s  %s\n", "Mark", "Elapsed", "Stage");

	/* Fake the first record - we could get it from early boot */
	rec->name = "reset";
	rec->time_us = 0;
	prev = print_time_record(BOOTSTAGE_AWAKE, rec, 0);

	/* Sort records by increasing time */
	qsort(record, ARRAY_SIZE(record), sizeof(*rec), h_compare_record);

	for (id = 0; id < BOOTSTAGE_COUNT; id++, rec++) {
		if (rec->time_us != 0 && !rec->start_us)
			prev = print_time_record(id, rec, prev);
	}

	puts("\nAccumulated time:\n");
	for (id = 0, rec = record; id < BOOTSTAGE_COUNT; id++, rec++) {
		if (rec->start_us)
			prev = print_time_record(id, rec, -1);
	}

	add_bootstages_devicetree(working_fdt);

	if (flags & GD_FLG_SILENT)
		gd->flags |= GD_FLG_SILENT;
}

