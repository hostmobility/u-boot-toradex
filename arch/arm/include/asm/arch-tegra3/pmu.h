/*
 *  (C) Copyright 2010,2011
 *  NVIDIA Corporation <www.nvidia.com>
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

#ifndef _ARCH_PMU_H_
#define _ARCH_PMU_H_

#define DVC_I2C_BUS_NUMBER	0
#define PMU_I2C_ADDRESS		0x2D

#define PMU_LDO5_REG		0x32	/* VDD_SDMMC1 */
#define PMU_LDO5_ON		0x01
#define PMU_LDO5_SLEEP		0x02
#define PMU_LDO5_SEL_1_0V	0x08
#define PMU_LDO5_SEL_0_1V_DELTA	0x04
#define PMU_LDO5_SEL(decivolts)	((decivolts - 10) * PMU_LDO5_SEL_0_1V_DELTA + \
				 PMU_LDO5_SEL_1_0V)

#define MAX_I2C_RETRY	3

#endif	/* _ARCH_PMU_H_ */
