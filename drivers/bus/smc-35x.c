/*
 * OMAP GPMC driver. Based upon the corresponding Linux Code
 *
 * Copyright (C) 2013 Sascha Hauer, Pengutronix, <s.hauer@pengutronix.de>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#define DRIVER_DESC  "ARM SMC35x Static Memory Controller driver"
#define DRIVER_NAME "arm-smc-35x"

#include <common.h>
#include <driver.h>
#include <malloc.h>
#include <init.h>
#include <io.h>
#include <of.h>
#include <linux/clk.h>
#include <of_address.h>
#include <of_mtd.h>
#include <smc35x.h>
#include <linux/amba/bus.h>

#define PL353_SMC_MEMC_STATUS_OFFS	0x0000	/* Controller status reg, RO */
#define PL353_SMC_MEMC_CFG_SET_OFFS	0x0008	/* Memory if-cfg reg, WO */
#define PL353_SMC_CFG_CLR_OFFS		0x000C	/* Clear config reg, WO */
#define PL353_SMC_DIRECT_CMD_OFFS	0x0010	/* Direct command reg, WO */
#define PL353_SMC_SET_CYCLES_OFFS	0x0014	/* Set cycles register, WO */
#define PL353_SMC_SET_OPMODE_OFFS	0x0018	/* Set opmode register, WO */
#define PL353_SMC_REFRESH_OFFS		0x0020
#define PL353_SMC_GET_OPMODE_OFFS	0x0104  /* Get opmode register RO  */
#define PL353_SMC_ECC_STATUS_OFFS	0x0400	/* ECC status register */
#define PL353_SMC_ECC_MEMCFG_OFFS	0x0404	/* ECC mem config reg */
#define PL353_SMC_ECC_MEMCMD1_OFFS	0x0408	/* ECC mem cmd1 reg */
#define PL353_SMC_ECC_MEMCMD2_OFFS	0x040C	/* ECC mem cmd2 reg */
#define PL353_SMC_ECC_VALUE0_OFFS	0x0418	/* ECC value 0 reg */

#define PL353_SMC_SET_OPMODE_MASK	0xFFFF
#define PL353_SMC_BUS_WIDTH_MASK	0x3

/* Set cycles register specific constants */
#define PL353_SMC_SET_CYCLES_T6_SHIFT	20
#define PL353_SMC_SET_CYCLES_T6_MASK	0xF
#define PL353_SMC_SET_CYCLES_T5_SHIFT	17
#define PL353_SMC_SET_CYCLES_T5_MASK	0x7
#define PL353_SMC_SET_CYCLES_T4_SHIFT	14
#define PL353_SMC_SET_CYCLES_T4_MASK	0x7
#define PL353_SMC_SET_CYCLES_T3_SHIFT	11
#define PL353_SMC_SET_CYCLES_T3_MASK	0x7
#define PL353_SMC_SET_CYCLES_T2_SHIFT	8
#define PL353_SMC_SET_CYCLES_T2_MASK	0x7
#define PL353_SMC_SET_CYCLES_T1_SHIFT	4
#define PL353_SMC_SET_CYCLES_T1_MASK	0xF
#define PL353_SMC_SET_CYCLES_T0_SHIFT	0
#define PL353_SMC_SET_CYCLES_T0_MASK	0xF

/* direct commands */
#define PL353_SMC_DC_UPT_REGS		(2 << 21) /* update regs */
#define PL353_SMC_DC_ADDR_CS_SHIFT	23
#define PL353_SMC_DC_ADDR_IF_SHIFT	25

/* Controller status register */
#define PL353_SMC_MEMC_STATUS_RAW_INT_1_SHIFT	6
#define PL353_SMC_MEMC_STATUS_RAW_INT_1_MASK	0x40
#define PL353_SMC_MEMC_STATUS_RAW_INT_0_SHIFT	5
#define PL353_SMC_MEMC_STATUS_RAW_INT_0_MASK	0x20
/* Clear configuration register specific constants */
#define PL353_SMC_CFG_CLR_INT_CLR_0_MASK	0x8
#define PL353_SMC_CFG_CLR_INT_CLR_1_MASK	0x10
#define PL353_SMC_CFG_CLR_ECC_INT_DIS_1	0x40
#define PL353_SMC_CFG_CLR_INT_DIS_1	0x2
#define PL353_SMC_CFG_CLR_DEFAULT_MASK	(PL353_SMC_CFG_CLR_INT_CLR_0_MASK | \
					 PL353_SMC_CFG_CLR_INT_CLR_1_MASK | \
					 PL353_SMC_CFG_CLR_ECC_INT_DIS_1 | \
					 PL353_SMC_CFG_CLR_INT_DIS_1)

/* ECC status register specific constants */
#define PL353_SMC_ECC_STATUS_BUSY	(1 << 6)

/* ECC memory config register specific constants */
#define PL353_SMC_ECC_MEMCFG_MODE_MASK		0xC
#define PL353_SMC_ECC_MEMCFG_MODE_SHIFT		2
#define PL353_SMC_ECC_MEMCFG_PGSIZE_MASK	0xC

#define PL353_SMC_MEM_WIDTH_8  0
#define PL353_SMC_MEM_WIDTH_16 1

struct smc_data {
	void __iomem *smc_base;
	struct device_d *dev;
	struct clk *clk;
	int childs;
	struct smc_dev_data* child_data[4];
};

struct smc_dev_data {
	void __iomem *base;
	int cs;
	int ifc;
	unsigned long timing;
	int bw;
	struct device_node *of_node;
	struct smc_data *parent;
};

static const struct of_device_id matches_sram[] = {
	{ .compatible = "mmio-sram" },
	{}
};

static const struct of_device_id matches_nor[] = {
	{ .compatible = "cfi-flash" },
	{}
};

static const struct of_device_id matches_nand[] = {
	{ .compatible = "arm,pl353-nand-r2p1" },
	{}
};

struct smc_sram_timings {
	u32 we;
	u32 tr;
	u32 pc;
	u32 wp;
	u32 ceoe;
	u32 wc;
	u32 rc;
};

struct smc_nand_timings {
	u32 rc;   /* RE# cycle time (nand min=2)                    */
	u32 wc;   /* WE# cycle time (nand min=2)                    */
	u32 rea;  /* RE# access time (nand assertion delay min=1)   */
	u32 wp;   /* WE# pulse width (nand deassertion delay min=1) */
	u32 clr;  /* CLE to RE# delay (nand status read time min=0) */
	u32 ar;   /* ALE to RE# delay (ID read time min=0)          */
	u32 rr;   /* Ready to RE# low (busy to re_n time min=0)     */
};

void pl353_smc_update_register(struct smc_dev_data *smc_plat_data);

/**
 * smc35x_ecc_is_busy_noirq - Read ecc busy flag
 * Return: the ecc_status bit from the ecc_status register. 1 = busy, 0 = idle
 */
static int smc35x_ecc_is_busy_noirq(struct device_d *dev)
{
	struct smc_data *smc_data = (struct smc_data *)dev->priv;

	return !!(readl(smc_data->smc_base + PL353_SMC_ECC_STATUS_OFFS) &
		  PL353_SMC_ECC_STATUS_BUSY);
}

/**
 * smc35x_ecc_is_busy - Read ecc busy flag
 * Return: the ecc_status bit from the ecc_status register. 1 = busy, 0 = idle
 */
int smc35x_ecc_is_busy(struct device_d *dev)
{
	int ret;

	ret = smc35x_ecc_is_busy_noirq(dev);

	return ret;
}
EXPORT_SYMBOL_GPL(smc35x_ecc_is_busy);

/**
 * smc35x_get_ecc_val - Read ecc_valueN registers
 * @ecc_reg:	Index of the ecc_value reg (0..3)
 * Return: the content of the requested ecc_value register.
 *
 * There are four valid ecc_value registers. The argument is truncated to stay
 * within this valid boundary.
 */
u32 smc35x_get_ecc_val(struct device_d *dev, int ecc_reg)
{
	struct smc_data *smc_data = (struct smc_data *)dev->priv;
	u32 addr, reg;

	ecc_reg &= 3;
	addr = PL353_SMC_ECC_VALUE0_OFFS + (ecc_reg << 2);
	reg = readl(smc_data->smc_base + addr);

	return reg;
}
EXPORT_SYMBOL_GPL(smc35x_get_ecc_val);

/**
 * smc35x_set_ecc_mode - Set SMC ECC mode
 * @mode:	ECC mode (BYPASS, APB, MEM)
 * Return: 0 on success or negative errno.
 */
int smc35x_set_ecc_mode(struct device_d *dev, enum pl353_smc_ecc_mode mode)
{
	struct smc_data *smc_data = (struct smc_data *)dev->priv;
	u32 reg;
	int ret = 0;

	switch (mode) {
	case PL353_SMC_ECCMODE_BYPASS:
	case PL353_SMC_ECCMODE_APB:
	case PL353_SMC_ECCMODE_MEM:

		reg = readl(smc_data->smc_base + PL353_SMC_ECC_MEMCFG_OFFS);
		reg &= ~PL353_SMC_ECC_MEMCFG_MODE_MASK;
		reg |= mode << PL353_SMC_ECC_MEMCFG_MODE_SHIFT;
		writel(reg, smc_data->smc_base + PL353_SMC_ECC_MEMCFG_OFFS);

		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(smc35x_set_ecc_mode);

/**
 * smc35x_set_ecc_pg_size - Set SMC ECC page size
 * @pg_sz:	ECC page size
 * Return: 0 on success or negative errno.
 */
int smc35x_set_ecc_pg_size(struct device_d *dev, unsigned int pg_sz)
{
	struct smc_data *smc_data = (struct smc_data *)dev->priv;
	u32 reg, sz;

	switch (pg_sz) {
	case 0:
		sz = 0;
		break;
	case 512:
		sz = 1;
		break;
	case 1024:
		sz = 2;
		break;
	case 2048:
		sz = 3;
		break;
	default:
		return -EINVAL;
	}

	reg = readl(smc_data->smc_base + PL353_SMC_ECC_MEMCFG_OFFS);
	reg &= ~PL353_SMC_ECC_MEMCFG_PGSIZE_MASK;
	reg |= sz;
	writel(reg, smc_data->smc_base + PL353_SMC_ECC_MEMCFG_OFFS);

	return 0;
}
EXPORT_SYMBOL_GPL(smc35x_set_ecc_pg_size);

/**
 * smc35x_get_nand_int_status_raw - Get NAND interrupt status bit
 * Return: the raw_int_status1 bit from the memc_status register
 */
int smc35x_get_nand_int_status_raw(struct device_d *dev, int ifc)
{
	struct smc_data *smc_data = (struct smc_data *)dev->priv;
	u32 reg;

	reg = readl(smc_data->smc_base + PL353_SMC_MEMC_STATUS_OFFS);
	if (ifc == 0)
		reg &= PL353_SMC_MEMC_STATUS_RAW_INT_0_MASK;
	else
		reg &= PL353_SMC_MEMC_STATUS_RAW_INT_1_MASK;

	return reg;
}
EXPORT_SYMBOL_GPL(smc35x_get_nand_int_status_raw);

/**
 * smc35x_clr_nand_int - Clear NAND interrupt
 */
void smc35x_clr_nand_int(struct device_d *dev, int ifc)
{
	struct smc_data *smc_data = (struct smc_data *)dev->priv;

	if (ifc == 0)
		writel(PL353_SMC_CFG_CLR_INT_CLR_0_MASK,
			smc_data->smc_base + PL353_SMC_CFG_CLR_OFFS);
	else
		writel(PL353_SMC_CFG_CLR_INT_CLR_1_MASK,
			smc_data->smc_base + PL353_SMC_CFG_CLR_OFFS);
}
EXPORT_SYMBOL_GPL(smc35x_clr_nand_int);

/**
 * smc_get_fclk_period_ps -
 * @smc_data:
 * returns period in ps
 */
static unsigned long smc_get_fclk_period_ps(struct smc_data *smc_data)
{
	unsigned long rate = clk_get_rate(smc_data->clk);

	rate /= 1000;
	rate = (1000000000 / rate); /* In pcioseconds */

	return rate;
}

/**
 * smc_ps_to_ticks -
 * @tick_ps:		duration of period (in ps)
 * @time_ns:		time to convert
 * returns number of ticks
 */
static unsigned int smc_ps_to_ticks(unsigned long tick_ps,
				unsigned int time_ns)
{
	/* Calculate in picosecs to yield more exact results */
	return (time_ns * 1000 + tick_ps - 1) / tick_ps;
}

/**
 * pl353_smc_set_cycles - Set memory timing parameters
 * @t0:	t_rc		read cycle time
 * @t1:	t_wc		write cycle time
 * @t2:	t_rea/t_ceoe	output enable assertion delay
 * @t3:	t_wp		write enable deassertion delay
 * @t4:	t_clr/t_pc	page cycle time
 * @t5:	t_ar/t_ta	ID read time/turnaround time
 * @t6:	t_rr		busy to RE timing
 *
 * Sets NAND chip specific timing parameters.
 */
static unsigned long smc_set_cycles(struct smc_dev_data *smc_plat_data,
			    u32 t0,
			    u32 t1,
			    u32 t2,
			    u32 t3,
			    u32 t4,
			    u32 t5,
			    u32 t6)
{
	unsigned long cycle = smc_get_fclk_period_ps(smc_plat_data->parent);
	u32 ticks;

	ticks = smc_ps_to_ticks(cycle, t0);
	t0 = (ticks & PL353_SMC_SET_CYCLES_T0_MASK);
	ticks = smc_ps_to_ticks(cycle, t1);
	t1 = (ticks & PL353_SMC_SET_CYCLES_T1_MASK) <<
			PL353_SMC_SET_CYCLES_T1_SHIFT;
	ticks = smc_ps_to_ticks(cycle, t2);
	t2 = (ticks & PL353_SMC_SET_CYCLES_T2_MASK) <<
			PL353_SMC_SET_CYCLES_T2_SHIFT;
	ticks = smc_ps_to_ticks(cycle, t3);
	t3 = (ticks & PL353_SMC_SET_CYCLES_T3_MASK) <<
			PL353_SMC_SET_CYCLES_T3_SHIFT;
	ticks = smc_ps_to_ticks(cycle, t4);
	t4 = (ticks & PL353_SMC_SET_CYCLES_T4_MASK) <<
			PL353_SMC_SET_CYCLES_T4_SHIFT;
	ticks = smc_ps_to_ticks(cycle, t5);
	t5 = (ticks & PL353_SMC_SET_CYCLES_T5_MASK) <<
			PL353_SMC_SET_CYCLES_T5_SHIFT;
	ticks = smc_ps_to_ticks(cycle, t6);
	t6 = (ticks & PL353_SMC_SET_CYCLES_T6_MASK) <<
			PL353_SMC_SET_CYCLES_T6_SHIFT;

	t0 |= t1 | t2 | t3 | t4 | t5 | t6;

	writel(t0, smc_plat_data->base + PL353_SMC_SET_CYCLES_OFFS);

	return t0;
}

/**
 * setup_sram_parameter_from_dt - Configure smc (timing parameter from DT)
 * @dev_node:		SRAM device node
 * @smc_plat_data:	Private information of the SRAM device
 * @smc_t:		Timing parameter
 * 0 on success
 */
static int setup_sram_parameter_from_dt(struct device_node *dev_node,
					struct smc_dev_data *smc_plat_data,
					struct smc_sram_timings *smc_t)
{
	return smc_set_cycles(smc_plat_data,
			smc_t->rc,
			smc_t->wc,
			smc_t->ceoe,
			smc_t->wp,
			smc_t->pc,
			smc_t->tr,
			smc_t->we);
}

/**
 * setup_nand_parameter_from_dt - Configure smc (timing parameter from DT)
 * @dev_node:		NAND device node
 * @smc_plat_data:	Private information of the NAND device
 * @smc_t:		Timing parameter
 * 0 on success
 */
static int setup_nand_parameter_from_dt(struct device_node *dev_node,
					struct smc_dev_data *smc_plat_data,
					struct smc_nand_timings *smc_t)
{
	return smc_set_cycles(smc_plat_data,
			smc_t->rc,
			smc_t->wc,
			smc_t->rea,
			smc_t->wp,
			smc_t->clr,
			smc_t->ar,
			smc_t->rr);
}

/**
 * read_nand_parameter_from_dt - Reads timing parameter from DT (SRAM)
 * @dev_node:		SRAM device node
 * @smc_plat_data:	Private information of the SRAM device
 * @smc_t:		Returned timing parameter
 * 0 on success
 */
static int read_sram_parameter_from_dt(struct device_node *dev_node,
					struct smc_dev_data *smc_plat_data,
					struct smc_sram_timings *smc_t)
{
	int ret = 0;

	if (!dev_node || !smc_t)
		return -1;

	memset(smc_t, 0, sizeof(*smc_t));

	ret |= of_property_read_u32(dev_node, "sram,tWE-ns", &smc_t->we);
	ret |= of_property_read_u32(dev_node, "sram,tTR-ns", &smc_t->tr);
	ret |= of_property_read_u32(dev_node, "sram,tPC-ns", &smc_t->pc);
	ret |= of_property_read_u32(dev_node, "sram,tWP-ns", &smc_t->wp);
	ret |= of_property_read_u32(dev_node, "sram,tCEOE-ns", &smc_t->ceoe);
	ret |= of_property_read_u32(dev_node, "sram,tWC-ns", &smc_t->wc);
	ret |= of_property_read_u32(dev_node, "sram,tRC-ns", &smc_t->rc);

	if (ret) {
		unsigned long mp = smc_get_fclk_period_ps(smc_plat_data->parent)*1000;
		smc_t->we  = 0;
		smc_t->tr  = 0;
		smc_t->pc = 0;
		smc_t->wp  = 1*mp;
		smc_t->ceoe = 1*mp;
		smc_t->wc  = 2*mp;
		smc_t->rc  = 2*mp;
		pr_warning("Using default timing for sram");
		pr_warning("we=%d", smc_t->we);
		pr_warning("tr=%d", smc_t->tr);
		pr_warning("pc=%d", smc_t->pc);
		pr_warning("wp=%d", smc_t->wp);
		pr_warning("ceoe=%d", smc_t->ceoe);
		pr_warning("wc=%d", smc_t->wc);
		pr_warning("rc=%d", smc_t->rc);
	}
	return ret;
}

/**
 * read_nand_parameter_from_dt - Reads timing parameter from DT (NAND)
 * @dev_node:		NAND device node
 * @smc_plat_data:	Private information of the NAND device
 * @smc_t:		Returned timing parameter
 * 0 on success
 */
static int read_nand_parameter_from_dt(struct device_node *dev_node,
					struct smc_dev_data *smc_plat_data,
					struct smc_nand_timings *smc_t)
{
	int ret = 0;

	if (!dev_node || !smc_t)
		return -1;

	memset(smc_t, 0, sizeof(*smc_t));

	ret |= of_property_read_u32(dev_node, "nand,tAR-ns", &smc_t->ar);
	ret |= of_property_read_u32(dev_node, "nand,tCLR-ns", &smc_t->clr);
	ret |= of_property_read_u32(dev_node, "nand,tRC-ns", &smc_t->rc);
	ret |= of_property_read_u32(dev_node, "nand,tREA-ns", &smc_t->rea);
	ret |= of_property_read_u32(dev_node, "nand,tRR-ns", &smc_t->rr);
	ret |= of_property_read_u32(dev_node, "nand,tWC-ns", &smc_t->wc);
	ret |= of_property_read_u32(dev_node, "nand,tWP-ns", &smc_t->wp);

	if (ret) {
		unsigned long mp = smc_get_fclk_period_ps(smc_plat_data->parent)*1000;
		smc_t->rr  = 0;
		smc_t->ar  = 0;
		smc_t->clr = 0;
		smc_t->wp  = 1*mp;
		smc_t->rea = 1*mp;
		smc_t->wc  = 2*mp;
		smc_t->rc  = 2*mp;
		pr_warning("Using default timing for nand flash");
		pr_warning("rr=%d", smc_t->rr);
		pr_warning("ar=%d", smc_t->ar);
		pr_warning("clr=%d", smc_t->clr);
		pr_warning("wp=%d", smc_t->wp);
		pr_warning("rea=%d", smc_t->rea);
		pr_warning("wc=%d", smc_t->wc);
		pr_warning("rc=%d", smc_t->rc);
	}
	return ret;
}

void pl353_smc_set_refresh_period(struct smc_dev_data *smc_plat_data,
					unsigned int period)
{
	u32 val = period & 0xF;
	writel(val, smc_plat_data->base + PL353_SMC_REFRESH_OFFS +
		(smc_plat_data->ifc*sizeof(u32)));
}

struct smc_dev_data *get_child_device(struct smc_data *smc_data, int ifc, int cs)
{
	int i=0;
	for (i=0;i<smc_data->childs;i++) {
		struct smc_dev_data *smc_plat_data = smc_data->child_data[i];
		if ((smc_plat_data->ifc == ifc) &&  (smc_plat_data->cs == cs))
			return smc_plat_data;
	}
	return NULL;
}

int smc35x_set_buswidth( struct device_d *dev, int ifc, int cs, unsigned int bw)
{
	struct smc_data *smc_data = (struct smc_data *)dev->priv;
	struct smc_dev_data *smc_plat_data = NULL;

	if (NULL == (smc_plat_data = get_child_device( smc_data, ifc, cs)))
		return -EINVAL;

	smc_plat_data->bw = bw;;
	pl353_smc_update_register(smc_plat_data);

	return 0;
}

void write_timing(struct smc_dev_data *smc_plat_data)
{
	writel(smc_plat_data->timing, smc_plat_data->base + PL353_SMC_SET_CYCLES_OFFS);
}

static int write_buswidth( struct smc_dev_data *smc_plat_data)
{
	void *__iomem reg = 0;
	u32 val = 0;

	if (smc_plat_data->bw != PL353_SMC_MEM_WIDTH_8  && smc_plat_data->bw != PL353_SMC_MEM_WIDTH_16)
		return -EINVAL;

	reg = smc_plat_data->base + PL353_SMC_GET_OPMODE_OFFS +
	(smc_plat_data->ifc * 0x80) + (smc_plat_data->cs * 0x20);
	val = readl(reg);
	val = 0;//NOTE: currently set this to NULL, reg comes up with suspect values
	val &= ~PL353_SMC_BUS_WIDTH_MASK;
	val |= (PL353_SMC_BUS_WIDTH_MASK & smc_plat_data->bw);
	val &= PL353_SMC_SET_OPMODE_MASK;

	writel(val, smc_plat_data->base + PL353_SMC_SET_OPMODE_OFFS);

	return 0;
}

void pl353_smc_update_register(struct smc_dev_data *smc_plat_data)
{
	u32 cmd = 0;

	write_buswidth(smc_plat_data);
	write_timing(smc_plat_data);

	cmd  = PL353_SMC_DC_UPT_REGS;/* cmd type */
	cmd |= (smc_plat_data->ifc<<PL353_SMC_DC_ADDR_IF_SHIFT) | /* address */
	(smc_plat_data->cs<<PL353_SMC_DC_ADDR_CS_SHIFT);
	writel(cmd, smc_plat_data->base + PL353_SMC_DIRECT_CMD_OFFS);
}

/**
 * configure_nor_interface - Configures smc for NOR device
 * @dev_node:		NOR device node
 * @smc_plat_data:	Private information of the NOR device
 * 0 on success
 */
static int configure_nor_interface(struct device_node *dev_node,
				   struct smc_dev_data *smc_plat_data)
{
	struct smc_sram_timings smc_t;
	int ret = -1;

	ret = read_sram_parameter_from_dt(dev_node, smc_plat_data, &smc_t);
	if (0 == ret) {
		smc_plat_data->timing = setup_sram_parameter_from_dt(dev_node,
								     smc_plat_data,
						       &smc_t);
		pl353_smc_update_register(smc_plat_data);
	}
	return ret;
}

/**
 * configure_sram_interface - Configures smc for sram device
 * @dev_node:		SRAM device node
 * @smc_plat_data:	Private information of the SRAM device
 * 0 on success
 */
static int configure_sram_interface(struct device_node *dev_node,
				    struct smc_dev_data *smc_plat_data)
{
	struct smc_sram_timings smc_t;
	int ret = -1;

	ret = read_sram_parameter_from_dt(dev_node, smc_plat_data, &smc_t);
	if (0 == ret) {
		int period = 1;//TODO: check if refresh is required
		smc_plat_data->timing = setup_sram_parameter_from_dt(dev_node,
								     smc_plat_data,
						       &smc_t);

		pl353_smc_set_refresh_period(smc_plat_data, period);
		pl353_smc_update_register(smc_plat_data);
	}
	return ret;
}

/**
 * configure_nand_interface - Configures smc for nand device
 * @dev_node:		Nand device node
 * @smc_plat_data:	Private information of the nand device
 * 0 on success
 */
static int configure_nand_interface(struct device_node *dev_node,
				    struct smc_dev_data *smc_plat_data)
{
	struct smc_nand_timings smc_t;
	int ret = -1;

	ret = read_nand_parameter_from_dt(dev_node, smc_plat_data, &smc_t);
	if (0 == ret) {
		smc_plat_data->timing = setup_nand_parameter_from_dt(dev_node,
								     smc_plat_data,
						       &smc_t);
		pl353_smc_update_register(smc_plat_data);
	}
	return ret;
}

/**
 * smc_probe_nor_child - Probes nor child device
 * @smc_data:		Private information of the smc device
 * @child:		Child to probe
 * 0 on success
 */
static int smc35x_probe_nor_child(struct smc_data *smc_data,
				  struct device_node *child)
{
	struct smc_dev_data* smc_plat_data = malloc(sizeof(struct smc_dev_data));
	u32 ifc;
	u32 cs;
	int ret;
	int bw;

	if (of_property_read_u32(child, "if", &ifc) < 0) {
		dev_err(smc_data->dev, "%s has no 'if' property\n",
			child->full_name);
		return -ENODEV;
	}
	if (of_property_read_u32(child, "cs", &cs) < 0) {
		dev_err(smc_data->dev, "%s has no 'cs' property\n",
			child->full_name);
		return -ENODEV;
	}
	if (of_property_read_u32(child, "bank-width", &bw) < 0) {
		dev_warn(smc_data->dev, "No bank-width given for NOR - defaulting to 8bit");
		bw = 1;
	}
	smc_plat_data->cs = (3 & cs);
	smc_plat_data->ifc = (1 & ifc);
	/* we only support 8 or 16 bit */
	smc_plat_data->bw = (bw == 1) ? PL353_SMC_MEM_WIDTH_8 : PL353_SMC_MEM_WIDTH_16;
	smc_plat_data->of_node = child;
	smc_plat_data->base = smc_data->smc_base;
	smc_plat_data->parent = smc_data;

	ret = configure_nor_interface(child, smc_plat_data);

	if (ret)
		pr_err("error initialising interface on if:%d -cs:%d\n",
		       smc_plat_data->ifc,
	 smc_plat_data->cs);

	smc_data->child_data[smc_data->childs++] = smc_plat_data;

	return 0;
}

/**
 * smc_probe_sram_child - Probes sram child device
 * @smc_data:		Private information of the smc device
 * @child:		Child to probe
 * 0 on success
 */
static int smc35x_probe_sram_child(struct smc_data *smc_data,
				   struct device_node *child)
{
	struct smc_dev_data* smc_plat_data = malloc(sizeof(struct smc_dev_data));
	u32 ifc;
	u32 cs;
	int ret;
	int bw;

	if (of_property_read_u32(child, "if", &ifc) < 0) {
		dev_err(smc_data->dev, "%s has no 'if' property\n",
			child->full_name);
		return -ENODEV;
	}
	if (of_property_read_u32(child, "cs", &cs) < 0) {
		dev_err(smc_data->dev, "%s has no 'cs' property\n",
			child->full_name);
		return -ENODEV;
	}
	if (of_property_read_u32(child, "bank-width", &bw) < 0) {
		dev_warn(smc_data->dev, "No bank-width given for SRAM - defaulting to 8bit");
		bw = 1;
	}
	smc_plat_data->cs = (3 & cs);
	smc_plat_data->ifc = (1 & ifc);
	/* we only support 8 or 16 bit */
	smc_plat_data->bw = (bw == 1) ? PL353_SMC_MEM_WIDTH_8 : PL353_SMC_MEM_WIDTH_16;
	smc_plat_data->of_node = child;
	smc_plat_data->base = smc_data->smc_base;
	smc_plat_data->parent = smc_data;

	ret = configure_sram_interface(child, smc_plat_data);

	if (ret)
		pr_err("error initialising interface on if:%d -cs:%d\n",
		       smc_plat_data->ifc,
	 smc_plat_data->cs);

	smc_data->child_data[smc_data->childs++] = smc_plat_data;

	return 0;
}

/**
 * smc_probe_nand_child - Probes nand child device
 * @smc_data:		Private information of the smc device
 * @child:		Child to probe
 * 0 on success
 */
static int smc35x_probe_nand_child(struct smc_data *smc_data,
				   struct device_node *child)
{
	struct smc_dev_data* smc_plat_data = malloc(sizeof(struct smc_dev_data));
	u32 ifc;
	u32 cs;
	int ret;

	if (of_property_read_u32(child, "if", &ifc) < 0) {
		dev_err(smc_data->dev, "%s has no 'if' property\n",
			child->full_name);
		return -ENODEV;
	}
	if (of_property_read_u32(child, "cs", &cs) < 0) {
		dev_err(smc_data->dev, "%s has no 'cs' property\n",
			child->full_name);
		return -ENODEV;
	}

	smc_plat_data->cs = (3 & cs);
	smc_plat_data->ifc = (1 & ifc);
	/* NAND always starts at 8bit */
	smc_plat_data->bw = PL353_SMC_MEM_WIDTH_8;
	smc_plat_data->of_node = child;
	smc_plat_data->base = smc_data->smc_base;
	smc_plat_data->parent = smc_data;

	ret = configure_nand_interface(child, smc_plat_data);

	if (ret)
		pr_err("error initialising interface on if:%d -cs:%d\n",
		       smc_plat_data->ifc,
	 smc_plat_data->cs);

	smc_data->child_data[smc_data->childs++] = smc_plat_data;

	return 0;
}

static int smc35x_probe(struct amba_device *adev, const struct amba_id *id)
{
	struct smc_data *smc_data;
	struct device_d *dev = &adev->dev;
	struct device_node *node  = dev->device_node;
	struct device_node *child = NULL;
	const struct of_device_id *matches = NULL;
	int ret = 0;

	smc_data = malloc(sizeof(*smc_data));
	if (!smc_data)
		return -ENOMEM;

	//TODO: get the corresponding clock
	smc_data->clk = clk_get(dev, NULL);
	if (!smc_data->clk) {
		free(smc_data);
		return -ENOMEM;
	}
	smc_data->dev = dev;
	smc_data->smc_base = amba_get_mem_region(adev);
	if (IS_ERR(smc_data->smc_base)) {
		free(smc_data);
		return -ENOMEM;
	}
	dev->priv = smc_data;

	/* enable pl353 and clear all interrupts */
	writel(0x7F, smc_data->smc_base + PL353_SMC_CFG_CLR_OFFS);
	writel(0x7F, smc_data->smc_base + PL353_SMC_CFG_CLR_OFFS);
	/* enable interrupts, leave ecc disabled */
	writel(0x03, smc_data->smc_base + PL353_SMC_MEMC_CFG_SET_OFFS);

	smc_data->childs = 0;
	for_each_child_of_node(node, child) {
		if (of_match_node(matches_nand, child)) {
			ret = smc35x_probe_nand_child(smc_data, child);
			if (!ret)
				matches = matches_nand;

		} else if (of_match_node(matches_nor, child)) {
			ret = smc35x_probe_nor_child(smc_data, child);
			if (!ret)
				matches = matches_nor;

		} else if (of_match_node(matches_sram, child)) {
			ret = smc35x_probe_sram_child(smc_data, child);
			if (!ret)
				matches = matches_sram;

		} else {
			dev_warn(smc_data->dev,
				"unsupported child %s\n",
				child->name);
			continue;
		}
		if (ret)
			break;

		matches++;
	}
	if (matches)
		of_platform_populate(node, matches, smc_data->dev);

	return ret;
}

static struct amba_id smc35x_ids[] = {
	{
		.id	= 0x00341352,
		.mask	= 0x00ffffff,
		.data	= NULL,
	},
	{
		.id	= 0x00641353,
		.mask	= 0x00ffffff,
		.data	= NULL,
	},
	{ 0, 0 },
};

static struct of_device_id smc35x_id_table[] = {
	{ .compatible = "arm,smc-35x" },	/* arm35x devices */
	{ }
};

struct amba_driver smc53x_driver = {
	.drv = {
		.name          = DRIVER_NAME,
		.of_compatible = DRV_OF_COMPAT(smc35x_id_table),
	},
	.probe		= smc35x_probe,
	.id_table	= smc35x_ids,
};

static int smc35x_init(void)
{
	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return amba_driver_register(&smc53x_driver);
}
device_initcall(smc35x_init);
