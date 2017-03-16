/**
 * @file
 * @brief netx4000 Generic platform initialzation code
 *
 * netx4000 Generic platform initialzation code
 *
 * (C) Copyright 2014 Hilscher Gesellschaft fuer Systemautomation mbH
 * http://www.hilscher.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; version 2 of
 * the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <common.h>
#include <io.h>
#include <init.h>
#include <asm/barebox-arm.h>
#include <asm/barebox-arm-head.h>
#include <mach/netx4000_regs.h>
#include <mach/netx4000_ddr.h>

#include <mach/hardware.h>

extern void ddr400_init(void);
extern void ddr600_init(void);

#ifdef CONFIG_ENABLE_DDR_ECC
extern void neon_memset(void* mem, int fill, unsigned long size);

static uint32_t __bare_init get_sdram_size(void) {
	uint32_t tmp;
	uint32_t max_row, max_col;
	uint32_t row_diff, col_diff, bank_diff;
	uint32_t dp_width;
	uint32_t size;

	tmp = readl(DENALI_CTL(1));
	max_col = (tmp & DENALI_CTL1_MAXCOL_MASK) >> DENALI_CTL1_MAXCOL_SHIFT;
	max_row = (tmp & DENALI_CTL1_MAXROW_MASK) >> DENALI_CTL1_MAXROW_SHIFT;

	tmp = readl(DENALI_CTL(53));
	col_diff  = (tmp & DENALI_CTL53_COL_DIFF_MASK) >> DENALI_CTL53_COL_DIFF_SHIFT;
	row_diff  = (tmp & DENALI_CTL53_ROW_DIFF_MASK) >> DENALI_CTL53_ROW_DIFF_SHIFT;
	bank_diff = (tmp & DENALI_CTL53_BANK_DIFF_MASK)>> DENALI_CTL53_BANK_DIFF_SHIFT;

        tmp = readl(DENALI_CTL(58));
	dp_width = (tmp & DENALI_CTL58_REDUC_MASK) ? 2: 4; /* 16 / 32 Bit */

	size =  (1 << (max_col - col_diff));
	size *= (1 << (max_row - row_diff));
	size *= (8 >> bank_diff);
	size *= dp_width;

	return size;
}
#endif

void __naked __bare_init barebox_arm_reset_vector(uint32_t *data)
{
	uint32_t cpu_rate;

	arm_cpu_lowlevel_init();

	arm_setup_stack(NETX4000_AXI_RAM_SPACE_START + SZ_512K - 8);

	/* Initialize DDR controller */

	cpu_rate = get_netx4000_cpu_rate();
	if (cpu_rate == 400000000)
		ddr400_init();
	else if ((cpu_rate == 600000000))
		ddr600_init();
	else
		while (1); /* FIXME */

	/* enable SIMD and floating-point support */
	__asm__ __volatile__("mrc %p15, 0,r0, c1, c0, 2"); /* Read Non-secure Access Control Register data  */
	__asm__ __volatile__("orr r0,r0,#(3<<20)");        /* permit access to CP10 for privileged and user level */
	__asm__ __volatile__("orr r0,r0,#(3<<22)");        /* permit access to CP11 for privileged and user level */
	__asm__ __volatile__("bic r0,r0,#(3<<30)");        /* enable SIMD and VFP instructions encoding */
	__asm__ __volatile__("mcr p15, 0,r0, c1, c0, 2");  /* Write secure Access Control Register data */

	__asm__ __volatile__("mrc p15, 0,r0, c1, c1, 2");  /* Write Non-secure Access Control Register data */
	__asm__ __volatile__("orr r0,r0,#(3<<10)");        /* permit non-secure access to CP10 and CP11 */
	__asm__ __volatile__("bic r0,r0,#(3<<14)");        /* full access */
	__asm__ __volatile__("mrc p15, 0,r0, c1, c1, 2");  /* Write Non-secure Access Control Register data */

	__asm__ __volatile__("isb");                       /* update changes */
	__asm__ __volatile__("mov r0,#(1<<30)");
	__asm__ __volatile__("vmsr fpexc,r0");             /* enable most advanced SIMD and VFP instructions */

	/* initialize memory to safely enable ecc */
#ifdef CONFIG_ENABLE_DDR_ECC
	{
		uint32_t sdram_size = get_sdram_size();
		uint32_t* volatile ram_addr = (uint32_t* volatile)NETX4000_DDR_ADDR_SPACE_START;

		neon_memset(ram_addr, 0, sdram_size);
	}
#endif

	barebox_arm_entry(NETX4000_DDR_ADDR_SPACE_START, SZ_128M, 0);
}
