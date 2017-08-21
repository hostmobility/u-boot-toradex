/*
 * (C) Copyright 2011
 * Andreas Pretzsch, carpe noctem engineering, apr@cn-eng.de
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <command.h>
#include <net.h>
#include <image.h>

static int do_fit_verify(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong addr = 0UL;
	void *hdr;

	if (argc =! 2) {
		return CMD_RET_USAGE;
	}

	addr = simple_strtoul(argv[1], NULL, 16);
	hdr = (void*)addr;

	if (!fit_check_format(hdr))
		return 1;

	if (!fit_all_image_verify(hdr))
		return 1;

	int cfg_noffset = fit_conf_get_node(hdr, NULL);
	if (fit_config_verify(hdr, cfg_noffset))
		return 1;

	return 0;
}

U_BOOT_CMD(fit_verify, 2, 0, do_fit_verify,
	"verify FIT image",
	"[addr]\n"
	"\t- verify FIT image at addr\n"
);
