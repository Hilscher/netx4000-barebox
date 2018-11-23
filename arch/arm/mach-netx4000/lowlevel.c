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
#include <asm/memory.h>

#include <mach/hardware.h>
#include <mach/netx4000_ddr.h>
#include <mach/regdef_netx4000.h>

extern int ddr400_init(void);
extern int ddr600_init(void);

static uint32_t sdram_size = -1;
char sdram_type[32] = "unknown - RAM is preconfigured";

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

static uint64_t fill_char[2];
static void ddr_ecc_init(void)
{
	uint32_t ram_size = sdram_size & ~0x1;
	uint32_t ram_addr = NETX4000_DDR_ADDR_SPACE_START;
	uint32_t chunk_size = ram_size / 8;
	int ch;
	NX4000_RAP_DMAC_CH_AREA_T*  dmach;
	NX4000_RAP_DMAC_REG_AREA_T* dmareg;

	dmareg = (NX4000_RAP_DMAC_REG_AREA_T*)Addr_NX4000_RAP_DMAC0_REG;

	/* Prepare source buffer for DMA transfer */
	memset((void*)&fill_char[0], 0, sizeof(fill_char));

	/* Setup 8 DMA channels to clear DDR */
	for(ch=0;ch<8;ch++) {
		dmach = (NX4000_RAP_DMAC_CH_AREA_T*)(Addr_NX4000_RAP_DMAC0 + ch * 0x40);

		/* Reset */
		dmach->ulRAP_DMAC_CH_CHCTRL = MSK_NX4000_RAP_DMAC_CH_CHCTRL_CLREN;
		dmach->ulRAP_DMAC_CH_CHCTRL = MSK_NX4000_RAP_DMAC_CH_CHCTRL_SWRST;

		/* Setup and start */
		dmach->asRAP_DMAC_CH_N[0].ulSA = (uint32_t)&fill_char[0];
		dmach->asRAP_DMAC_CH_N[0].ulDA = (uint32_t)(ram_addr + ch * chunk_size);
		dmach->asRAP_DMAC_CH_N[0].ulTB = chunk_size;
		dmach->ulRAP_DMAC_CH_CHCFG = MSK_NX4000_RAP_DMAC_CH_CHCFG_SAD |        /* no source increment */
					     MSK_NX4000_RAP_DMAC_CH_CHCFG_TM  |        /* Block transfer */
					     (4 << SRT_NX4000_RAP_DMAC_CH_CHCFG_DDS) | /* 128 bit dest size */
					     (4 << SRT_NX4000_RAP_DMAC_CH_CHCFG_SDS);  /* 128 Bit source size */
		dmach->ulRAP_DMAC_CH_CHCTRL = MSK_NX4000_RAP_DMAC_CH_CHCTRL_SETEN |
					      MSK_NX4000_RAP_DMAC_CH_CHCTRL_CLRSUS;
		dmach->ulRAP_DMAC_CH_CHCTRL = MSK_NX4000_RAP_DMAC_CH_CHCTRL_STG;       /* Software triggered DMA */
	}

	/* Wait for DMA to finish */
	while((dmareg->ulRAP_DMAC_REG_DST_END & 0xFF) != 0xFF) ;

	/* Reset all DMA channels */
	for(ch=0;ch<8;ch++) {
		dmach = (NX4000_RAP_DMAC_CH_AREA_T*)(Addr_NX4000_RAP_DMAC0 + ch * 0x40);
		dmach->ulRAP_DMAC_CH_CHCTRL = MSK_NX4000_RAP_DMAC_CH_CHCTRL_CLREN;
		dmach->ulRAP_DMAC_CH_CHCTRL = MSK_NX4000_RAP_DMAC_CH_CHCTRL_SWRST;
		dmach->ulRAP_DMAC_CH_CHCFG = DFLT_VAL_NX4000_RAP_DMAC_CH_CHCFG;
		dmach->asRAP_DMAC_CH_N[0].ulSA = 0;
		dmach->asRAP_DMAC_CH_N[0].ulDA = 0;
		dmach->asRAP_DMAC_CH_N[0].ulTB = 0;
	}
}

static int netx4000_mem_init(void)
{
	uint32_t ram_size = sdram_size & ~0x1;
	struct memory_bank *bank;

	if (!barebox_add_memory_bank("ram0", NETX4000_DDR_ADDR_SPACE_START, ram_size))
		pr_info("Info: %s sdram_size is used.\n", (sdram_size & 0x1) ? "Limited calculated" : "Calculated");
	else
		pr_warn("Warning: DT defined sdram_size is used!\n");

	for_each_memory_bank(bank)
		pr_info("sdram_size: 0x%lx-0x%lx (%luMiB)\n", bank->start, bank->start + bank->size - 1, bank->size/1024/1024);

	pr_info("sdram_type: %s\n", sdram_type);

	return 0;
}
mem_initcall(netx4000_mem_init);

#define AddressFilteringStartRegister  0xFAF10C00
#define AddressFilteringEndRegister    0xFAF10C04
static void fix_l2c_address_filtering_issue(void)
{
	*(uint32_t*)AddressFilteringStartRegister = 0x0;
	*(uint32_t*)AddressFilteringEndRegister = 0xc0000000;
	*(uint32_t*)AddressFilteringStartRegister = 0x40000001;
}

void __naked __bare_init barebox_arm_reset_vector(uint32_t *data)
{
	uint32_t cpu_rate;
	int ecc;

	arm_cpu_lowlevel_init();

	arm_setup_stack(NETX4000_AXI_RAM_SPACE_START + SZ_512K - 8);

	fix_l2c_address_filtering_issue();

	/* Initialize DDR controller */

	cpu_rate = get_netx4000_cpu_rate();
	if (cpu_rate == 400000000)
		ecc = ddr400_init();
	else if ((cpu_rate == 600000000))
		ecc = ddr600_init();
	else
		while (1); /* FIXME */

	sdram_size = get_sdram_size();
	if (sdram_size > 0x40000000) {
		memset((void*)0x80000000-32,0x55,32);
		memset((void*)0xc0000000-32,0xaa,32);
		/* Limit sdram_size if the upper GiB is inaccessible ('netX4000 RLXD' and 'netX4000 FULL v1'). */
		if (*(volatile uint32_t*)0x7ffffffc != 0x55555555)
			sdram_size = 0x40000001;
	}

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
	if (ecc)
		ddr_ecc_init();

	barebox_arm_entry(NETX4000_DDR_ADDR_SPACE_START, SZ_128M, 0);
}
