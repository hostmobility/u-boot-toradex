/*
 * Copyright (c) 2011 The Chromium OS Authors.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef _PMU_H_
#define _PMU_H_

int pmu_read(int reg);
int pmu_write(int reg, uchar *data, uint len);

#ifdef CONFIG_TEGRA_CLOCK_SCALING
int pmu_set_nominal(void);
int pmu_is_voltage_nominal(void);
#endif /* CONFIG_TEGRA_CLOCK_SCALING */

#endif	/* PMU_H */
