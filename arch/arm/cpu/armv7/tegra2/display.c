/*
 *  (C) Copyright 2011
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
#include <common.h>
#include <asm/clocks.h>
#include <asm/arch/clock.h>

void display_clk_init(void)
{
	/* TODO: Put this into the FDT when we have clock support there */
	clock_start_periph_pll(PERIPH_ID_3D, CLOCK_ID_MEMORY, CLK_300M);
	clock_start_periph_pll(PERIPH_ID_2D, CLOCK_ID_MEMORY, CLK_300M);
	clock_start_periph_pll(PERIPH_ID_HOST1X, CLOCK_ID_PERIPH, CLK_144M);
	clock_start_periph_pll(PERIPH_ID_DISP1, CLOCK_ID_CGENERAL, CLK_600M);
	clock_start_periph_pll(PERIPH_ID_PWM, CLOCK_ID_SFROM32KHZ, CLK_32768);
}
