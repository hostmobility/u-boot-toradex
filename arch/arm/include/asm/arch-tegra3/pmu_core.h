/*
 *  (C) Copyright 2012
 *  Toradex AG <www.toradex.com>
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

/* definitions for TPS62360 / TPS62362 */
#ifndef _PMU_CORE_H_
#define _PMU_CORE_H_

#define DVC_I2C_BUS_NUMBER	0
#define PMU_CORE_I2C_ADDRESS		0x60

#define PMU_CORE_VOLTAGE_START_REG	0x02
#define PMU_CORE_VOLTAGE_DVFS_REG	0x00

#define VDD_CORE_NOMINAL_T30	43	/* 1.2V = 0.77V + x * 10mV */

#endif	/* PMU_CORE_H */
