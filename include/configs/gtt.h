/*
 * Copyright (C) 2012 Toradex, Inc.
 * Portions Copyright (c) 2010, 2011 NVIDIA Corporation
 * Portions Copyright (c) 2011 The Chromium OS Authors
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

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx4_common.h"

#define V_PROMPT		"MX-4 GTT # "

#define MX4_PRODUCT_TYPE "gtt"

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_STD_DEVICES_SETTINGS \
	CONFIG_COMMON_EXTRA_ENV_SETTINGS \
	"PRODUCT=GTT\0" \
	"memargs=mem=498M@0M fbmem=12M@498M nvmem=2M@510M\0" \
	"vidargs=video=tegrafb0:800x480CT-32@60 video=tegrafb1:1920x1080-32@60\0" \
	"setup=setenv setupargs asix_mac=${ethaddr} asix_mac2=${ethaddr2} no_console_suspend=1 console=tty1 console=ttyS0,${baudrate}n8 debug_uartport=lsport,0 ${memargs} ${vidargs}\0" \
	""

#endif /* __CONFIG_H */
