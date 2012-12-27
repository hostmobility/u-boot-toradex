/*
 * (C) Copyright 2010,2011
 * NVIDIA Corporation <www.nvidia.com>
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

#ifndef _TEGRA3_H_
#define _TEGRA3_H_

#define TEGRA2_AHB_BASE		0x6000C000
#define TEGRA_SLINK4_BASE	0x7000DA00	/* aka SBC4 */

#define NV_PA_SDRAM_BASE	0x80000000

#define NV_PA_SDMMC1_BASE	0x78000000
#define NV_PA_SDMMC2_BASE	0x78000200
#define NV_PA_SDMMC3_BASE	0x78000400
#define NV_PA_SDMMC4_BASE	0x78000600

#define NV_PA_USB1_BASE		0x7D000000
#define NV_PA_USB2_BASE		0x7D004000
#define NV_PA_USB3_BASE		0x7D008000

#define NV_APB_MISC_BASE    0x70000000

#include <asm/arch-tegra/tegra.h>

#endif
