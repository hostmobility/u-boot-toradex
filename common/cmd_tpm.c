/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

#include <common.h>
#include <command.h>
#include <tpm.h>

#define MAX_TRANSACTION_SIZE 100
static u8 tpm_response[MAX_TRANSACTION_SIZE];
static void report_error(const char *msg);

/*
 * The following TPM command definitions were borrowed from the
 * vboot_reference package.
 */

const struct s_tpm_nv_read_cmd {
	u8 buffer[22];
	u16 index;
	u16 length;
} tpm_nv_read_cmnd = {
	{ 0x0, 0xc1, 0x0, 0x0, 0x0, 0x16, 0x0, 0x0, 0x0, 0xcf, },
	10,
	18,
};

const struct s_tpm_nv_write_cmd {
	u8 buffer[MAX_TRANSACTION_SIZE];
	u16 index;
	u16 length;
	u16 data;
} tpm_nv_write_cmnd = {
	{ 0x0, 0xc1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xcd, },
	10,
	18,
	22,
};

/*
 * paste_u32()
 *
 * Insert a 32 bit number in big endian representation at the supplied location.
 *
 */
static void paste_u32(u8 *location, u32 value)
{
	u32 be = cpu_to_be32(value);
	memcpy(location, &be, sizeof(be));
}

/*
 * tpm_read() expects two parameters, internal address of the tpm NVRAM, and
 * length of data to read.
 */
static int tpm_read(int argc, char * const argv[])
{
	u32 addr, size;
	char *p;
	struct s_tpm_nv_read_cmd cmd;
	int rv;

	if (argc != 2) {
		report_error("wrong number of parameters\n");
		return ~0;
	}

	addr = simple_strtoul(argv[0], &p, 0);
	if (*p) {
		report_error("bad address value\n");
		return ~0;
	}

	memcpy(&cmd, &tpm_nv_read_cmnd, sizeof(cmd));
	size = simple_strtoul(argv[1], &p, 0);
	if (*p || (size >= (sizeof(cmd.buffer) - cmd.index))) {
		report_error("bad size value\n");
		return ~0;
	}

	paste_u32(cmd.buffer + cmd.index, addr);
	paste_u32(cmd.buffer + cmd.length, size);

	size = sizeof(tpm_response);
	if (!tis_sendrecv(cmd.buffer, sizeof(cmd.buffer),
			  tpm_response, &size)) {
		int i;
		for(i = 0; i < size; i++)
			printf(" %2.2x", tpm_response[i]);
		printf("\n");
		rv = 0;
	} else {
		printf("tpm read command failed\n");
		rv = ~0;
	}
	return rv;
}

/*
 * tpm_write() expects a variable number of parameters: the internal address
 * followed by data to write, byte by byte.
 *
 * Returns 0 on success or ~0 on errors (wrong arguments or TPM failure).
 */
static int tpm_write(int argc, char * const argv[])
{
	u32 addr, size, datum, total_length;
	char *p;
	struct s_tpm_nv_write_cmd cmd;
	int rv = ~0;

	if (argc < 2) {
		report_error("too few paramters\n");
		return rv;
	}

	addr = simple_strtoul(argv[0], &p, 0);
	if (*p) {
		report_error("bad address value\n");
		return rv;
	}

	memcpy(&cmd, &tpm_nv_write_cmnd, sizeof(cmd));

	argc--;
	if (argc >= (sizeof(cmd.buffer) - cmd.data)) {
		report_error("too many arguments\n");
		return rv;
	}
	for (size = 0; size < argc; size++) {
		datum = simple_strtoul(argv[size + 1], &p, 0);
		if (*p || (datum > 0xff)) {
			report_error("bad data value\n");
			return rv;
		}
		cmd.buffer[cmd.data + size] = (u8)datum;
	}

	paste_u32(cmd.buffer + cmd.index, addr);
	paste_u32(cmd.buffer + cmd.length, size);

	/* this sets the total TPM command length at the appropriate offset. */
	total_length = size + cmd.data;
	paste_u32(cmd.buffer + 2, total_length);

	if (!tis_sendrecv(cmd.buffer, total_length, tpm_response, &size)) {
		int i;
		for(i = 0; i < size; i++)
			printf(" %2.2x", tpm_response[i]);
		printf("\n");
		rv = 0;
	} else {
		printf("tpm read command failed\n");
	}
	return rv;
}

static int do_tpm(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int rv = 0;
	/*
	 * Verify that in case it is present, the first argument, it is
	 * exactly one character in size.
	 */
	if ((argc > 1) && (argv[1][1] != '\0')) {
		report_error("bad parameter\n");
		return ~0;
	}

	if (tis_init()) {
		printf("tis_init() failed!\n");
		return ~0;
	}

	if (tis_open()) {
		printf("tis_open() failed!\n");
		return ~0;
	}

	if (argc > 2) {
		switch (argv[1][0]) {
		case 'r':
			rv = tpm_read(argc - 2, argv + 2);
			break;
		case 'w':
			rv = tpm_write(argc - 2, argv + 2);
			break;
		default:
			report_error("unknown option\n");
			rv = ~0;
			break;
		};
	}

	if (tis_close()) {
		printf("tis_close() failed!\n");
		rv = ~0;
	}

	return rv;
}

U_BOOT_CMD(tpm, 10, 1, do_tpm,
	   "tpm debugging commands",
	   "\tr <index> <number of bytes> - read data at location index\n"
	   "\tw <index> <data> [<data>]   - write data at "
	   "location index\n"
);

static void report_error(const char *msg)
{
	if (msg && *msg)
		printf("%s\n", msg);
	cmd_usage(&__u_boot_cmd_tpm);
}
