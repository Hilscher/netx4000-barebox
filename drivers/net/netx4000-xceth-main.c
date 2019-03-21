/*
* XCeth driver for Hilscher netX4000 based platforms
*
* drivers/net/netx4000-xceth.c
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
*/

#define DRIVER_DESC "xc ethernet driver for Hilscher netX4000 based platforms"
#define DRIVER_NAME "netx4000-xceth"

#include <common.h>
#include <init.h>
#include <net.h>
#include <of_net.h>
#include <io.h>

#include <firmware.h>
#include <firmware/netx4000-xc.h>

#include <mach/regdef_netx4000.h>
#include "netx4000-xceth-hal.h"

#define FIRMWARE_HANDLER "netx4000-xc"

struct platform_data {
	char *firmware;
};

struct platform_data netx4000_xceth_pdata[] = {
	{ .firmware = "/boot/hethmac-xc0_6.2.0.2.bin", },
	{ .firmware = "/boot/hethmac-xc1_6.2.0.2.bin", },
	{ .firmware = "/boot/hethmac-xc2_6.2.0.2.bin", },
	{ .firmware = "/boot/hethmac-xc3_6.2.0.2.bin", },
};

struct firmware_data {
	struct firmware_mgr *mgr;
	char id[16];
	uint32_t xcinst;
};

struct priv_data {
	struct device_d *dev;
	struct eth_device edev;
	phy_interface_t interface;

	struct platform_data *pdata;
	struct firmware_data fw;

	ETHERNET_FRAME_T *ptFrame;
	void *hFrame;
};

/* --------------------------------------------------------------------------
 * Send and receive functions
 * -------------------------------------------------------------------------- */

#define TX_DESC_TIMEOUT  (0)
#define RX_DESC_TIMEOUT  (0) // (2*MSECOND)

static int netx4000_xceth_send(struct eth_device *edev, void *packet, int length)
{
	struct priv_data *priv = edev->priv;
	struct device_d *dev = priv->dev;
	uint32_t val32;
	int rc;

	dev_dbg(dev, "%s: +++ (*edev %p, *packet %p, length %i)\n", __func__, edev, packet, length);

	if (!priv->ptFrame) {
		rc = wait_on_timeout(TX_DESC_TIMEOUT, !netx4000_xceth_get_send_cnf(priv->fw.xcinst, &priv->ptFrame, &priv->hFrame, &val32, 0 /* low prio */));
		if (rc && (rc != -EIO)) {
			dev_err(dev, "TX timed out - no descriptor available!\n");
			return rc;
		}
	}

	memcpy(priv->ptFrame, packet, length);
	if (length < 60) {
		memset((char*)priv->ptFrame + length, 0, 60 - length);
		length = 60;
	}

	rc = netx4000_xceth_send_frame(priv->fw.xcinst, priv->hFrame, length, 0 /* LowPrio */);
	if (rc) {
		dev_err(dev, "netx4000_xceth_send_frame() failed!\n");
		return rc;
	}
	priv->ptFrame = NULL;

	return 0;
}

static int netx4000_xceth_recv(struct eth_device *edev)
{
	struct priv_data *priv = edev->priv;
	struct device_d *dev = priv->dev;
	ETHERNET_FRAME_T *ptFrame;
	void *hFrame;
	uint32_t length;
	uint32_t max_rx_count = 0;
	int rc;

	dev_dbg(dev, "%s: +++ (*edev %p)\n", __func__, edev);

	rc = wait_on_timeout(RX_DESC_TIMEOUT, !netx4000_xceth_recv_frame(priv->fw.xcinst, &ptFrame, &hFrame, &length, 0 /* LowPrio */));
	if (rc) {
		dev_dbg(dev, "RX timed out - no descriptor available!\n");
		return rc;
	}

	do {
		memcpy(NetRxPackets[0], ptFrame, length);
		net_receive(edev, NetRxPackets[0], length);
		netx4000_xceth_release_frame(priv->fw.xcinst, hFrame);
	} while (max_rx_count-- && !netx4000_xceth_recv_frame(priv->fw.xcinst, &ptFrame, &hFrame, &length, 0 /* LowPrio */));

	return 0;
}

/* --------------------------------------------------------------------------
 *
 * -------------------------------------------------------------------------- */

static void netx4000_xceth_update_linkspeed(struct eth_device *edev)
{
	struct priv_data *priv = edev->priv;
	struct device_d *dev = priv->dev;
	int rc;

	dev_dbg(dev, "%s: +++ (*edev=%p)\n", __func__, edev);

	rc = netx4000_xceth_set_link_mode(priv->fw.xcinst, edev->phydev->link, edev->phydev->speed, edev->phydev->duplex);
	if (rc)
		dev_err(dev, "%s: netx4000_xceth_set_link_mode() failed!\n", __func__);

	return;
}

static int netx4000_xceth_open(struct eth_device *edev)
{
	struct priv_data *priv = edev->priv;
	struct device_d *dev = priv->dev;
	struct ioctl_request req;
	int rc;

	dev_dbg(dev, "%s: +++ (*edev=%p)\n", __func__, edev);

	netx4000_xceth_initFifoUnit(priv->fw.xcinst);

	rc = netx4000_xceth_get_frame(priv->fw.xcinst, &priv->ptFrame, &priv->hFrame);
	if (rc) {
		dev_err(dev, "Reserving TX descriptor from fifo failed!\n");
		return rc;
	}

	/* Starting XC firmware */
	req.dev = priv->dev;
	req.id = priv->fw.id;
	rc = firmwaremgr_ioctl(priv->fw.mgr, FW_START, &req);
	if (rc) {
		dev_err(dev, "Starting XC firmware %s failed!\n", priv->pdata->firmware);
		return rc;
	}
	dev_dbg(dev, "Starting XC firmware %s successed\n", priv->pdata->firmware);

	rc = phy_device_connect(edev, NULL, -1, netx4000_xceth_update_linkspeed, 0, priv->interface);
	if (rc)
		return rc;

	return 0;
}

static int netx4000_xceth_get_ethaddr(struct eth_device *edev, u8 *addr)
{
	struct priv_data *priv = edev->priv;
	struct device_d *dev = priv->dev;
	int rc;

	dev_dbg(dev, "%s: +++ (*edev=%p, addr %p %02x:%02x:%02x:%02x:%02x:%02x)\n", __func__, edev, addr, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	rc = netx4000_xceth_get_mac_address(priv->fw.xcinst, ETH_MAC_ADDRESS_CHASSIS, (uint8_t (*)[6])addr);

	return rc;
}

static int netx4000_xceth_set_ethaddr(struct eth_device *edev, const unsigned char *addr)
{
	struct priv_data *priv = edev->priv;
	struct device_d *dev = priv->dev;
	int rc;

	dev_dbg(dev, "%s: +++ (*edev=%p, addr %p %02x:%02x:%02x:%02x:%02x:%02x)\n", __func__, edev, addr, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	rc = netx4000_xceth_set_mac_address(priv->fw.xcinst, ETH_MAC_ADDRESS_CHASSIS, (uint8_t *)addr);

	return rc;
}

/* --------------------------------------------------------------------------
 * Initialization functions for the driver and the chip
 * -------------------------------------------------------------------------- */

static int netx4000_xceth_xc_init(struct eth_device *edev)
{
	struct priv_data *priv = edev->priv;
	struct device_d *dev = priv->dev;
	struct ioctl_request req;
	int rc = -EINVAL;

	do {
		/* Searching for firmware manager */
		priv->fw.mgr = firmwaremgr_find(FIRMWARE_HANDLER);
		if (!priv->fw.mgr) {
			dev_err(dev, "XC firmware manager %s not found\n", FIRMWARE_HANDLER);
			break;
		}

		/* Uploading XC firmware only to firmware handler! */
		rc = firmwaremgr_load_file(priv->fw.mgr, priv->pdata->firmware);
		if (rc) {
			dev_err(dev, "Loading XC firmware %s failed!\n", priv->pdata->firmware);
			break;
		}
		dev_dbg(dev, "Loding XC firmware %s successed\n", priv->pdata->firmware);

		/* Store firmware id for further ioctl requests */
		strncpy(priv->fw.id, strrchr(priv->pdata->firmware, '/')+1, sizeof(priv->fw.id)-1);
		*strchr(priv->fw.id, '_') = 0;

		/* Initialize ioctl struct */
		req.dev = priv->dev;
		req.id = priv->fw.id;

		/* Requesting XC firmware */
		req.data = (void **)&priv->fw.xcinst;
		rc = firmwaremgr_ioctl(priv->fw.mgr, FW_REQUEST, &req);
		if (rc) {
			dev_err(dev, "Requesting XC firmware %s failed!\n", priv->pdata->firmware);
			break;
		}
		dev_dbg(dev, "Requesting XC firmware %s successed\n", priv->pdata->firmware);
		req.data = NULL;

		/* Check requested XC instance */
		switch (priv->fw.xcinst) {
			case 0x1: priv->fw.xcinst = 0; break;
			case 0x2: priv->fw.xcinst = 1; break;
			case 0x4: priv->fw.xcinst = 2; break;
			case 0x8: priv->fw.xcinst = 3; break;
			default: priv->fw.xcinst = -1;
		}
		if (priv->fw.xcinst == -1) {
			dev_err(dev, "Requested XC firmware %s returns invalid xcinst!\n", priv->pdata->firmware);
			break;
		}

		/* Resetting XC firmware */
		rc = firmwaremgr_ioctl(priv->fw.mgr, FW_RESET, &req);
		if (rc) {
			dev_err(dev, "Resetting XC firmware %s failed!\n", priv->pdata->firmware);
			break;
		}
		dev_dbg(dev, "Resetting XC firmware %s successed\n", priv->pdata->firmware);

		/* Uploading XC firmware to XC units! */
		rc = firmwaremgr_ioctl(priv->fw.mgr, FW_UPLOAD, &req);
		if (rc) {
			dev_err(dev, "Uploading XC firmware %s failed!\n", priv->pdata->firmware);
			break;
		}
		dev_dbg(dev, "Uploading XC firmware %s successed\n", priv->pdata->firmware);

		dev_dbg(dev, "Initializing XC firmware %s successed\n", priv->pdata->firmware);
		return 0;
	} while (0);

	dev_err(dev, "Initializing XC firmware %s failed!\n", priv->pdata->firmware);
	return rc;
}

static int netx4000_xceth_probe(struct device_d *dev)
{
	struct priv_data *priv;
	struct eth_device *edev;
	int rc;

	dev_dbg(dev, "%s: +++ (*dev=%p)\n", __func__, dev);

	priv = xzalloc(sizeof(*priv));
	if (!priv) {
		dev_err(dev, "%s: xzalloc() failed!\n", __func__);
		return -ENOMEM;
	}

	rc = dev_get_drvdata(dev, (const void **)&priv->pdata);
	if (rc) {
		priv->pdata = xzalloc(sizeof(*priv->pdata));
		rc = of_property_read_string(dev->device_node, "firmware", (const char **)&priv->pdata->firmware);
		if (rc) {
			dev_err(dev, "Invalid or missing 'firmware' node in DT!\n");
			return rc;
		}
	}

	priv->dev = dev;

	edev = &priv->edev;
	edev->priv = priv;
	edev->parent = dev;
	edev->open = netx4000_xceth_open;
	edev->send = netx4000_xceth_send;
	edev->recv = netx4000_xceth_recv;
	edev->get_ethaddr = netx4000_xceth_get_ethaddr;
	edev->set_ethaddr = netx4000_xceth_set_ethaddr;

	priv->interface = of_get_phy_mode(dev->device_node);

	/* Handel XC initialization */
	rc = netx4000_xceth_xc_init(edev);
	if (rc)
		goto err_out;

	/* Initialize pointer to xPEC DRAM */
	rc = netx4000_xceth_fifo_request(dev, priv->fw.xcinst);
	if (rc) {
		dev_err(dev, "netx4000_xceth_fifo_request() failed!\n");
		goto err_out;
	}

	/* Configure MAC address */
	if (of_get_mac_address(dev->device_node))
		netx4000_xceth_set_ethaddr(edev, of_get_mac_address(dev->device_node));

	if ((rc = eth_register(edev)) != 0) {
		goto err_out;
	}
	dev_info(dev, "Initializing eth%d successed!\n", edev->dev.id);

	return 0;

err_out:
	dev_err(dev, "Initializing failed\n");

	if (dev_get_drvdata(dev, (const void **)&priv->pdata)) {
		free(priv->pdata);
	}
	if (priv)
		free(priv);

	return rc;
}



static struct of_device_id netx4000_xceth_dt_ids[] = {
	{ .compatible = "hilscher,netx4000-xceth", .data = NULL },
	{ .compatible = "hilscher,netx4000-xceth0", .data = &netx4000_xceth_pdata[0] },
	{ .compatible = "hilscher,netx4000-xceth1", .data = &netx4000_xceth_pdata[1] },
	{ .compatible = "hilscher,netx4000-xceth2", .data = &netx4000_xceth_pdata[2] },
	{ .compatible = "hilscher,netx4000-xceth3", .data = &netx4000_xceth_pdata[3] },
	{ /* sentinel */ }
};

static struct driver_d netx4000_xceth_driver = {
	.name  = DRIVER_NAME,
	.probe = netx4000_xceth_probe,
	.of_compatible = DRV_OF_COMPAT(netx4000_xceth_dt_ids),
};

static int __init netx4000_xceth_init(void)
{
	/* FIXME: reset pointer FIFO borders */
	netx4000_pfifo_initial_reset();

	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return platform_driver_register(&netx4000_xceth_driver);
}
postenvironment_initcall(netx4000_xceth_init);

/* --- Module information --- */

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
