/*
* USB EHCI driver for Hilscher netx4000 based platforms
*
* drivers/usb/host/ehci-netx4000.c
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#define DRIVER_DESC  "EHCI driver for Hilscher netx4000 based platforms"
#define DRIVER_NAME "ehci-netx4000"

#include <common.h>
#include <driver.h>
#include <init.h>
#include <usb/ehci.h>
#include <linux/clk.h>
#include "ehci.h"

#include <mach/hardware.h>
#include "ehci-netx4000.h"

struct netx4000_ehci_priv {
	struct device_d *dev;
	USB_HOST_AREA_T *regbase;
	struct clk *clk;

	unsigned int num_ports;

	unsigned int ahbpci_win1_ctr;
	unsigned int ahbpci_win2_ctr;
	unsigned int pciahb_win1_ctr;
	unsigned int pciahb_win2_ctr;

	unsigned int pciahb_win1_base_address;
	unsigned int pciahb_win2_base_address;
	unsigned int ahbpci_bridge_base_address;

	unsigned int ohci_base_address;
	unsigned int ehci_base_address;
};

static inline int32_t ioset32(uint32_t setmask, void *addr)
{
	uint32_t val;

	val = readl(addr);
	writel(val | setmask, addr);

	return 0;
}

static inline int32_t ioclear32(uint32_t clearmask, void *addr)
{
	uint32_t val;

	val = readl(addr);
	writel(val & ~clearmask, addr);

	return 0;
}

static int netx4000_ehci_chip_reset(struct netx4000_ehci_priv *priv)
{
	ioset32((0x3 << 10) | /* win1 size 2gb */
		MSK_USB_HOST_USBCTR_PLL_RST |
		MSK_USB_HOST_USBCTR_USBH_RST,
		(void*)&priv->regbase->ulUSB_HOST_USBCTR);
	udelay(5);
	ioclear32(MSK_USB_HOST_USBCTR_PLL_RST |
		MSK_USB_HOST_USBCTR_PCICLK_MASK |
		MSK_USB_HOST_USBCTR_USBH_RST,
		(void*)&priv->regbase->ulUSB_HOST_USBCTR);

	return 0;
}

#define USB2CFG  (NETX4000_SYSTEMCTRL_VIRT_BASE + 0x10)
#define netx4000_ehci_power_up()  ioclear32(0x1, (void*)USB2CFG)
#define netx4000_ehci_power_down()  ioset32(0x1, (void*)USB2CFG)
#define netx4000_ehci_h2mode_disable()  ioclear32(0x2, (void*)USB2CFG)
#define netx4000_ehci_h2mode_enable()  ioset32(0x2, (void*)USB2CFG)
static int netx4000_ehci_chip_init(struct netx4000_ehci_priv *priv)
{
	if (priv->num_ports > 1)
		netx4000_ehci_h2mode_enable();

	if(clk_enable(priv->clk)) {
		dev_err(priv->dev, "netx4000_periph_clock_enable() failed\n");
		return -EIO;
	}
	netx4000_ehci_power_up();

	netx4000_ehci_chip_reset(priv);

	/* AHB-PCI Bridge PCI Communiction Registers */
	writel(MSK_USB_HOST_AHB_BUS_CTR_SMODE_READY_CTR |
		MSK_USB_HOST_AHB_BUS_CTR_MMODE_HBUSREQ |
		MSK_USB_HOST_AHB_BUS_CTR_MMODE_WR_INCR |
		MSK_USB_HOST_AHB_BUS_CTR_MMODE_BYTE_BURST |
		MSK_USB_HOST_AHB_BUS_CTR_MMODE_HTRANS,
		&priv->regbase->ulUSB_HOST_AHB_BUS_CTR);

	writel((priv->pciahb_win1_ctr & MSK_USB_HOST_PCIAHB_WIN1_CTR_AHB_BASEADR) |
		(0 << SRT_USB_HOST_PCIAHB_WIN1_CTR_ENDIAN_CTR) |
		(3 << SRT_USB_HOST_PCIAHB_WIN1_CTR_PREFETCH),
		&priv->regbase->ulUSB_HOST_PCIAHB_WIN1_CTR); /* (1)-a */

	writel((priv->pciahb_win2_ctr & MSK_USB_HOST_PCIAHB_WIN2_CTR_AHB_BASEADR) |
		(0 << SRT_USB_HOST_PCIAHB_WIN2_CTR_ENDIAN_CTR) |
		(3 << SRT_USB_HOST_PCIAHB_WIN2_CTR_PREFETCH),
		&priv->regbase->ulUSB_HOST_PCIAHB_WIN2_CTR); /* (1)-b */

	writel((priv->ahbpci_win2_ctr & MSK_USB_HOST_AHBPCI_WIN2_CTR_PCIWIN2_BASEADR) |
		(0 << SRT_USB_HOST_AHBPCI_WIN2_CTR_BURST_EN) |
		(3 << SRT_USB_HOST_AHBPCI_WIN2_CTR_PCICMD),
		&priv->regbase->ulUSB_HOST_AHBPCI_WIN2_CTR); /* (2) */

	writel(0x000b103f,&priv->regbase->ulUSB_HOST_PCI_INT_ENABLE);

	writel(MSK_USB_HOST_PCI_ARBITER_CTR_PCIBP_MODE |
		MSK_USB_HOST_PCI_ARBITER_CTR_PCIREQ0 |
		MSK_USB_HOST_PCI_ARBITER_CTR_PCIREQ1,
		&priv->regbase->ulUSB_HOST_PCI_ARBITER_CTR);

	/* PCI Configuration Registers for AHB-PCI Bridge */

	writel((0x40000000 & MSK_USB_HOST_AHBPCI_WIN1_CTR_PCIWIN1_BASEADR) |
		(5 << SRT_USB_HOST_AHBPCI_WIN1_CTR_PCICMD),
		&priv->regbase->ulUSB_HOST_AHBPCI_WIN1_CTR); /* (3) */

	writel(MSK_USB_HOST_CMND_STS_OHCI_SERR_ENABLE |
		MSK_USB_HOST_CMND_STS_OHCI_PARITY_ERROR_RESPONSE |
		MSK_USB_HOST_CMND_STS_OHCI_BUS_MASTER |
		MSK_USB_HOST_CMND_STS_OHCI_MEMORY_SPACE,
		&priv->regbase->ulUSB_HOST_CMND_STS_OHCI);

	writel(priv->ahbpci_bridge_base_address, &priv->regbase->ulUSB_HOST_BASEAD_OHCI); /* (4) */
	writel(priv->pciahb_win1_base_address, &priv->regbase->ulUSB_HOST_WIN1_BASEAD); /* (5)-a */
	writel(priv->pciahb_win2_base_address, &priv->regbase->ulUSB_HOST_WIN2_BASEAD); /* (5)-b */

	/* PCI Configuration Registers for OHCI/EHCI */

	writel((0x80000000 & MSK_USB_HOST_AHBPCI_WIN1_CTR_PCIWIN1_BASEADR) |
		(5 << SRT_USB_HOST_AHBPCI_WIN1_CTR_PCICMD),
		&priv->regbase->ulUSB_HOST_AHBPCI_WIN1_CTR); /* (6) */

	writel(MSK_USB_HOST_CMND_STS_OHCI_SERR_ENABLE |
		MSK_USB_HOST_CMND_STS_OHCI_PARITY_ERROR_RESPONSE |
		MSK_USB_HOST_CMND_STS_OHCI_BUS_MASTER |
		MSK_USB_HOST_CMND_STS_OHCI_MEMORY_SPACE,
		&priv->regbase->ulUSB_HOST_CMND_STS_EHCI);

	writel(priv->ohci_base_address, &priv->regbase->ulUSB_HOST_BASEAD_OHCI); /* (7) */
	writel(priv->ehci_base_address, &priv->regbase->ulUSB_HOST_BASEAD_EHCI); /* (7) */

	/* UTMI */

//	writel(0x0, &priv->regbase->ulUSB_HOST_UTMICTRL_OHCI); // it's the same register as ...
	writel(0x0, &priv->regbase->ulUSB_HOST_UTMICTRL_EHCI);

	return 0;
}

static int netx4000_ehci_probe(struct device_d *dev)
{
	struct netx4000_ehci_priv *priv;
	struct ehci_data ehci;
	struct resource *res_mem;
	int rc, num_ports;

	if ((priv = xzalloc(sizeof(*priv))) == NULL)
		return -ENOMEM;

	/* Retrieve platform data */

	res_mem = dev_request_mem_resource(dev, 0);
	if (IS_ERR(res_mem))
		return PTR_ERR(res_mem);

	priv->clk = clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "Unable to get clock");
		return PTR_ERR(res_mem);
	}

	rc = of_property_read_u32(dev->device_node, "num-ports", &num_ports);
	if (rc) {
		dev_warn(dev, "num_ports not provided in DT => defaulting to one port!");
		num_ports = 1;
	}
	else if ((num_ports < 1) || (num_ports > 2)) {
		dev_err(dev, "num_ports out of range [1,2]");
		return -EINVAL;
	}

	/* */

	priv->dev = dev;
	priv->regbase = IOMEM(res_mem->start);
	priv->num_ports = num_ports;

	memset(&ehci,0,sizeof(ehci));
	ehci.flags = 0;
	ehci.hccr = (void*)priv->regbase + 0x1000;

	/* Initialize the USB chip */

//	priv->pciahb_win1_ctr = PHYS_OFFSET; /* DDR-RAM 0x40000000 */
	priv->pciahb_win1_ctr = 0x40000000;
	priv->pciahb_win2_ctr = 0x0; /* unused */
//	priv->pciahb_win1_base_address = PHYS_OFFSET; /* DDR-RAM 0x40000000 */
	priv->pciahb_win1_base_address = 0x40000000;
	priv->pciahb_win2_base_address = 0x0; /* unused */

	priv->ahbpci_win1_ctr = 0x0; /* xxx */
	priv->ahbpci_win2_ctr = (unsigned int)priv->regbase; /* 0xf9000000 */
	priv->ahbpci_bridge_base_address = priv->ahbpci_win2_ctr+0x10800; /* 0xf9010800 */

	priv->ohci_base_address = priv->ahbpci_win2_ctr;
	priv->ehci_base_address = priv->ohci_base_address+0x1000;

	priv->num_ports = num_ports;

	rc = netx4000_ehci_chip_init(priv);
	if (rc)
		goto err_out;

	/* Add USB device */

	rc = ehci_register(dev, &ehci);
	if (rc)
		goto err_out;

	dev_info(dev, "successfully initialized!\n");

	return 0;

err_out:
	return rc;
}

static const struct of_device_id netx4000_ehci_of_match[] = {
	{.compatible = "hilscher,netx4000-hcd",},
	{},
};

static struct driver_d netx4000_ehci_driver = {
	.name = DRIVER_NAME,
	.probe		= netx4000_ehci_probe,
	.of_compatible = DRV_OF_COMPAT(netx4000_ehci_of_match),
};

static int __init netx4000_ehci_init(void)
{
	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return platform_driver_register(&netx4000_ehci_driver);
}
device_initcall(netx4000_ehci_init);

/* --- Module information --- */

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
