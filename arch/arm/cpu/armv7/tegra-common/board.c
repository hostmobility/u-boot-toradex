/*
 *  (C) Copyright 2010,2011
 *  NVIDIA Corporation <www.nvidia.com>
 *  (C) Copyright 2012
 *  Toradex, Inc.
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
#include <asm/io.h>
#include <asm/arch-tegra/ap20.h>
#include <asm/arch-tegra/pmc.h>
#include <asm/arch/tegra.h>
#include <fdt_decode.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Boot ROM initializes the odmdata in APBDEV_PMC_SCRATCH20_0,
 * so we are using this value to identify memory size.
 */

unsigned int board_query_sdram_size(void)
{
#ifndef CONFIG_COLIBRI_T20
	struct pmc_ctlr *const pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	u32 reg;

	reg = readl(&pmc->pmc_scratch20);
	debug("pmc->pmc_scratch20 (ODMData) = 0x%08X\n", reg);

	/* bits 31:28 in OdmData are used for RAM size  */
	switch ((reg) >> 28) {
	case 1:
		return 0x10000000;	/* 256 MB */
	case 2:
		return 0x20000000;	/* 512 MB */
	case 4:
		return 0x40000000;	/* 1GB */
	case 8:
		/*
		 * On tegra3, out of 2GB, 1MB(0xFFF00000 - FFFFFFFF) is used for
		 * Bootcode(IROM) and arm specific exception vector code.
		 */
		return 0x7ff00000;	/* 2GB - 1MB */
	default:
		return 0x40000000;	/* 1GB */
	}
#else /* CONFIG_COLIBRI_T20 */
	/* Colibri T20 does not use OdmData but rather relies on memory controller
	   configuration done by boot ROM based on BCT information */

	u32 *pa_emc_adr_cfg = (void *)NV_PA_EMC_ADR_CFG_BASE;

	u32 reg = readl(pa_emc_adr_cfg);

	/* 4 MB shifted by EMEM_DEVSIZE */
	u32 memsize = (4 << 20) << ((reg & EMEM_DEVSIZE_MASK) >> EMEM_DEVSIZE_SHIFT);

	/* Unfortunately it is possible to at least boot a 256 MB module with
	   a 512 MB BCT therefore double check whether we really do have that
	   amount of physical memory */
	if (memsize == 512*1024*1024) {
		volatile u32 *pMem = 0;
		u32 temp = pMem[0];
		pMem[0] = 0xabadcafe;
		asm volatile("" ::: "memory");
		if (pMem[0] == pMem[128*1024*1024/4]) // Why we have to read at 128MB?
			panic("512 MB BCT running on a 256 MB module!\n");
		pMem[0] = temp;
	}

	return memsize;
#endif /* CONFIG_COLIBRI_T20 */
}

#if defined(CONFIG_DISPLAY_BOARDINFO) || defined(CONFIG_DISPLAY_BOARDINFO_LATE)
int checkboard(void)
{
	const char *board_name;
#ifdef CONFIG_OF_CONTROL
	board_name = fdt_decode_get_model(gd->blob);
#else
	board_name = sysinfo.board_string;
#endif
	printf("Board: %s\n", board_name);
	return 0;
}
#endif	/* CONFIG_DISPLAY_BOARDINFO */

#ifdef CONFIG_ARCH_CPU_INIT
/*
 * Note this function is executed by the ARM7TDMI AVP. It does not return
 * in this case. It is also called once the A9 starts up, but does nothing in
 * that case.
 */
int arch_cpu_init(void)
{
	/* Fire up the Cortex A9 */
	if (ap20_cpu_is_cortexa9())
		bootstage_mark(BOOTSTAGE_MAIN_CPU_AWAKE, "arch_cpu_init A9");
	else
		bootstage_mark(BOOTSTAGE_CPU_AWAKE, "arch_cpu_init AVP");
	tegra_start();
	/* If tegra_start() returns, we are running on the A9 */

	/* We didn't do this init in start.S, so do it now */
	cpu_init_crit();
	bootstage_mark(BOOTSTAGE_MAIN_CPU_READY, "arch_cpu_init done");
	return 0;
}
#endif

void arch_full_speed(void)
{
	ap20_init_pllx(0);
	debug("CPU at %d\n", clock_get_rate(CLOCK_ID_XCPU));
	debug("Memory at %d\n", clock_get_rate(CLOCK_ID_MEMORY));
	debug("Periph at %d\n", clock_get_rate(CLOCK_ID_PERIPH));
}
