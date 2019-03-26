/*
* MDIO XC driver for Hilscher netx4000 based platforms
*
* drivers/net/phy/mdio-netx4000.c
*
* (C) Copyright 2015 Hilscher Gesellschaft fuer Systemautomation mbH
* http://www.hilscher.com
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation; version 2 of
* the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
*/

#define DRIVER_DESC "MDIO XC driver for Hilscher netX4000 based platforms"
#define DRIVER_NAME "mdio-xc-netx4000"

#include <common.h>
#include <init.h>

#include <linux/mdio-bitbang.h>

#define INTPHY0_ID  0
#define INTPHY1_ID  1

/* Regdef */

#define regdef(mask,shift,name) \
static inline int g##name(int val) { \
	return (val >> shift) & mask; \
} \
static inline int s##name(int val) { \
	return (val & mask) << shift; \
}

#define ASIC_CTRL_ACCESS_KEY		(volatile void*)0xf408017c

#define CLOCK_ENABLE				(volatile void*)0xf4080138
regdef (0x1, 16, XC_MISC)
regdef (0xff, 0, MMIO_SEL)

#define PHY_CONTROL					(volatile void*)0xf4023a20
regdef (0x1, 31, PHY_RESET)
regdef (0x1, 30, PHY_SIM_BYP)
regdef (0x1, 18, PHY1_AUTOMDIX)
regdef (0x1, 17, PHY1_FXMODE)
regdef (0x1, 16, PHY1_ENABLE)
regdef (0x1, 10, PHY0_AUTOMDIX)
regdef (0x1,  9, PHY0_FXMODE)
regdef (0x1,  8, PHY0_ENABLE)
regdef (0xf,  0, PHY_MODE)

#define INT_PHY_CTRL0				(volatile void*)0xf4023a90
#define INT_PHY_CTRL1				(volatile void*)0xf4023aa0
#define INT_PHY_CTRL2				(volatile void*)0xf4023ab0
#define INT_PHY_CTRL3				(volatile void*)0xf4023ac0
#define INT_PHY_CTRLx_SIZE			0x10

#define INT_PHY_CTRL_MIIMU(base)					(base+0x0)
#define INT_PHY_CTRL_MIIMU_SW(base)					(base+0x4)
regdef (0x1,  7, MDI_RO)
regdef (0x1,  6, MDOE)
regdef (0x1,  5, MDO)
regdef (0x1,  4, MDC)
regdef (0x1,  0, SW_MODE_ENABLE)

#define INT_PHY_CTRL_LED(base)						(base+0x8)
regdef (0xf, 12, LED_FLASH_INTERVAL)
regdef (0x3,  8, LED_MODE)
regdef (0x1,  7, LED1)
regdef (0x1,  6, LED0)
regdef (0x1,  5, SPEED100_RO)
regdef (0x1,  4, SPEED10_RO)
regdef (0x1,  3, LINK_RO)
regdef (0x1,  2, DUPLEX_RO)
regdef (0x1,  1, TX_ACTIVE_RO)
regdef (0x1,  0, RX_ACTIVE_RO)

#define INT_PHY_CTRL_ENHANCED_LINK_DETECTION(base)	(base+0xc)

struct platform_data {
	struct resource res;
	uint32_t intphy_id;
};

static struct platform_data netx4000_mdio_xc[] = {
	{
		.res = DEFINE_RES_MEM(INT_PHY_CTRL0, INT_PHY_CTRLx_SIZE),
		.intphy_id = INTPHY0_ID,
	},
	{
		.res = DEFINE_RES_MEM(INT_PHY_CTRL1, INT_PHY_CTRLx_SIZE),
		.intphy_id = INTPHY1_ID,
	},
	{
		.res = DEFINE_RES_MEM(INT_PHY_CTRL2, INT_PHY_CTRLx_SIZE),
		.intphy_id = -1, /* only external PHYs are supported */
	},
	{
		.res = DEFINE_RES_MEM(INT_PHY_CTRL3, INT_PHY_CTRLx_SIZE),
		.intphy_id = -1, /* only external PHYs are supported */
	},
};

static struct priv_data {
	struct device_d *dev;
	struct platform_data *data;
	struct mdiobb_ctrl ctrl;
	void __iomem *base;
	uint32_t intphy_id;
};

static void mdio_dir(struct mdiobb_ctrl *ctrl, int dir)
{
	struct priv_data *priv = container_of(ctrl, struct priv_data, ctrl);
	struct device_d *dev = priv->dev;
	uint32_t val32;

	dev_dbg(dev, "%s: +++ (*ctrl=%p, dir=%i)\n", __func__, ctrl, dir);

	val32 = ioread32(INT_PHY_CTRL_MIIMU_SW(priv->base));
	if (dir)
		iowrite32(val32 | sMDOE(1), INT_PHY_CTRL_MIIMU_SW(priv->base));
	else
		iowrite32(val32 & ~sMDOE(1), INT_PHY_CTRL_MIIMU_SW(priv->base));
}

static int mdio_get(struct mdiobb_ctrl *ctrl)
{
	struct priv_data *priv = container_of(ctrl, struct priv_data, ctrl);
	struct device_d *dev = priv->dev;
	uint32_t val32;

	dev_dbg(dev, "%s: +++ (*ctrl=%p)\n", __func__, ctrl);

	val32 = ioread32(INT_PHY_CTRL_MIIMU_SW(priv->base));

	return (val32 & sMDI_RO(1)) ? 1 : 0;
}

static void mdio_set(struct mdiobb_ctrl *ctrl, int what)
{
	struct priv_data *priv = container_of(ctrl, struct priv_data, ctrl);
	struct device_d *dev = priv->dev;
	uint32_t val32;

	dev_dbg(dev, "%s: +++ (*ctrl=%p, what=%i)\n", __func__, ctrl, what);

	val32 = ioread32(INT_PHY_CTRL_MIIMU_SW(priv->base));
	if (what)
		iowrite32(val32 | sMDO(1), INT_PHY_CTRL_MIIMU_SW(priv->base));
	else
		iowrite32(val32 & ~sMDO(1), INT_PHY_CTRL_MIIMU_SW(priv->base));
}

static void mdc_set(struct mdiobb_ctrl *ctrl, int what)
{
	struct priv_data *priv = container_of(ctrl, struct priv_data, ctrl);
	struct device_d *dev = priv->dev;
	uint32_t val32;

	dev_dbg(dev, "%s: +++ (*ctrl=%p, what=%i)\n", __func__, ctrl, what);

	val32 = ioread32(INT_PHY_CTRL_MIIMU_SW(priv->base));
	if (what)
		iowrite32(val32 | sMDC(1), INT_PHY_CTRL_MIIMU_SW(priv->base));
	else
		iowrite32(val32 & ~sMDC(1), INT_PHY_CTRL_MIIMU_SW(priv->base));
}

static struct mdiobb_ops netx4000_mdio_ops = {
	.set_mdc = mdc_set,
	.set_mdio_dir = mdio_dir,
	.set_mdio_data = mdio_set,
	.get_mdio_data = mdio_get,
};

static int netx4000_mdio_intphy_init(struct priv_data *priv)
{
	struct device_d *dev = priv->dev;
	struct device_node *phy_node;
	uint32_t keyval, val32, addr;
	char *p8 = NULL;

	dev_dbg(dev, "%s: +++ (*priv=%p)\n", __func__, priv);

	/* Check for phy node */
	phy_node = of_find_node_by_path_from(dev->device_node, "/phy");
	if (!phy_node) {
		dev_err(dev, "Invalid or missing '/phy' node in DT!\n");
		return -EINVAL;
	}

	/* Check if PHY is declared as internal PHY */
	of_property_read_string(phy_node, "phy-mode", &p8);
	if (!p8 || strcmp("internal", p8)) {
		dev_err(dev, "External PHYs currently not supported!\n");
		return -ENOTSUPP;
	}

	/* Check if PHY ID matched to MDIO interface */
	of_property_read_u32(phy_node, "reg", &addr);
	if (addr != priv->intphy_id) {
		dev_err(dev, "Invalid or missing PHY ID!\n");
		return -EINVAL;
	}

	if (addr == INTPHY0_ID) {
		val32 = ioread32(PHY_CONTROL);
		iowrite32(val32|sPHY0_ENABLE(1), PHY_CONTROL);
	}
	else if (addr == INTPHY1_ID) {
		val32 = ioread32(PHY_CONTROL);
		iowrite32(val32|sPHY1_ENABLE(1), PHY_CONTROL);
	}

	/* Enable software mode */
	val32 = ioread32(INT_PHY_CTRL_MIIMU_SW(priv->base));
	iowrite32(val32|sSW_MODE_ENABLE(1), INT_PHY_CTRL_MIIMU_SW(priv->base));

	/* Enable static LED mode */
	iowrite32(sLED_MODE(0x01 /* static */), INT_PHY_CTRL_LED(priv->base));

	/* Deassert PHY reset / Enable Auto-Negotiation */
	val32 = ioread32(PHY_CONTROL) & ~(sPHY_RESET(-1)|sPHY_MODE(-1));
	iowrite32(val32|sPHY_MODE(0x7 /* advertise all */), PHY_CONTROL);
	mdelay(100);

	return 0;
}

static int netx4000_mdio_probe(struct device_d *dev)
{
	struct priv_data *priv;
	struct mii_bus *bus;
	int rc;

	dev_dbg(dev, "%s: +++ (*dev=%p)\n", __func__, dev);

	priv = xzalloc(sizeof(*priv));
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	rc = dev_get_drvdata(dev, (const void **)&priv->data);
	if (priv->data) {
		/* Using platform data */
		struct resource *res;

		res = request_iomem_region(dev_name(dev), priv->data->res.start, priv->data->res.end);
		if (IS_ERR(res)) {
			rc = -EIO;
			goto err_out;
		}
		priv->base = IOMEM(res->start);

		priv->intphy_id = priv->data->intphy_id;
	}
	else {
		/* Using devicetree data */
		priv->base = dev_request_mem_region(dev, 0);
		if (IS_ERR(priv->base)){
			dev_err(dev, "Invalid or missing 'reg' node in DT!\n");
			rc = PTR_ERR(priv->base);
			goto err_out;
		}

		priv->intphy_id = -1;
		rc = of_property_read_u32(dev->device_node, "intphy_id", &priv->intphy_id);
		if (rc) {
			dev_dbg(dev, "Invalid or missing 'intphy_id' node in DT!\n");
		}
	}

	priv->ctrl.ops = &netx4000_mdio_ops;

	bus = alloc_mdio_bitbang(&priv->ctrl);
	bus->parent = dev;
	bus->dev.device_node = dev->device_node;

	dev->priv = bus;

	/* Enable clock for global XC logic */
	{
		uint32_t keyval, val32;

		val32 = ioread32(CLOCK_ENABLE);
		keyval = ioread32(ASIC_CTRL_ACCESS_KEY);
		iowrite32(keyval, ASIC_CTRL_ACCESS_KEY);
		iowrite32(val32|sXC_MISC(1), CLOCK_ENABLE);
	}

	/* Preparing for internal PHYs if configured in DT */
	rc = netx4000_mdio_intphy_init(priv);
	if (rc)
		goto err_out;

	rc = mdiobus_register(bus);
	if (rc) {
		dev_err(dev, "Initializing failed!\n");
		goto err_out;
	}
	dev_info(dev, "Initializing %s successed!\n", dev_name(&bus->dev));

	return 0;

err_out:
	free(bus);
	free(priv);

	return rc;
}

static const struct of_device_id netx4000_mdio_dt_ids[] = {
	{ .compatible = "hilscher,netx4000-mdio-xc", .data = NULL },
	{ .compatible = "hilscher,netx4000-mdio-xc0", .data = &netx4000_mdio_xc[0] },
	{ .compatible = "hilscher,netx4000-mdio-xc1", .data = &netx4000_mdio_xc[1] },
	{ .compatible = "hilscher,netx4000-mdio-xc2", .data = &netx4000_mdio_xc[2] },
	{ .compatible = "hilscher,netx4000-mdio-xc3", .data = &netx4000_mdio_xc[3] },
	{ /* sentinel */ }
};

static struct driver_d netx4000_mdio_driver = {
	.name = DRIVER_NAME,
	.probe = netx4000_mdio_probe,
	.of_compatible = DRV_OF_COMPAT(netx4000_mdio_dt_ids),
};

static int __init netx4000_mdio_init(void)
{
	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return platform_driver_register(&netx4000_mdio_driver);
}
device_initcall(netx4000_mdio_init);

/* --- Module information --- */

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
