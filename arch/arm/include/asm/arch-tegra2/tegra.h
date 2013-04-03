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

#ifndef _TEGRA2_H_
#define _TEGRA2_H_

#define TEGRA_SLINK4_BASE	0x7000DA00 /* MX4/VCB SPI4_CS0 */
/* #define TEGRA_SPIFLASH_BASE	0x7000C380 */	/* SPI1, T20 */

#define NV_PA_SDRAM_BASE	0x00000000

#define NV_PA_SDMMC1_BASE	0xC8000000
#define NV_PA_SDMMC2_BASE	0xC8000200
#define NV_PA_SDMMC3_BASE	0xC8000400
#define NV_PA_SDMMC4_BASE	0xC8000600

#define NV_PA_USB1_BASE		0xC5000000
#define NV_PA_USB3_BASE		0xC5008000

#define NV_APB_MISC_BASE    0x70000000

#include <asm/arch-tegra/tegra.h>

#endif
