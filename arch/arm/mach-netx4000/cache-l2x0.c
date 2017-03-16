#include <common.h>
#include <init.h>
#include <io.h>

#include <asm/mmu.h>
#include <asm/cache-l2x0.h>

#include <mach/netx4000_regs.h>

static int netx4000_l2x0_init(void) {
	void __iomem *l2x0_base = IOMEM(NETX4000_A9_PL310_BASE);
	u32 val;

	val = (1 << L2X0_AUX_CTRL_DATA_PREFETCH_SHIFT) |
		(1 << L2X0_AUX_CTRL_INSTR_PREFETCH_SHIFT);
	l2x0_init(l2x0_base, val, ~0UL);

	return 0;
}
postmmu_initcall(netx4000_l2x0_init);
