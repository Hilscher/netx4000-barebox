/*
* GMAC driver for Hilscher netX4000 based platforms
*
* drivers/net/netx4000-gmac.c
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

#define DRIVER_DESC "GMAC driver for Hilscher netX4000 based platforms"
#define DRIVER_NAME "gmac-netx4000"

#include <init.h>
#include <net.h>
#include <of_net.h>
#include <io.h>
#include <dma.h>
#include "asm/system.h" /* required by dmb() */

#include <digest.h>
#include <environment.h>

#include "gmac-netx4000.h"

struct netx4000_gmac_priv {
	void __iomem *base;

	struct dummy_desc *rx_chain;
	struct dummy_desc *tx_chain;
	uint8_t *txbuffer;
	uint8_t *rxbuffer;

	uint32_t tx_currdesc;
	uint32_t rx_currdesc;

	phy_interface_t interface;

	struct eth_device edev;
	struct mii_bus miibus;
};

/* --------------------------------------------------------------------------
 * Help functions
 * -------------------------------------------------------------------------- */

static inline int32_t netx4000_ioset32(uint32_t setmask, void *addr)
{
	uint32_t val;

	val = readl(addr);
	writel(val | setmask, addr);

	return 0;
}

static inline int32_t netx4000_ioclear32(uint32_t clearmask, void *addr)
{
	uint32_t val;

	val = readl(addr);
	writel(val & ~clearmask, addr);

	return 0;
}

static inline int32_t netx4000_iomod32(uint32_t clearmask, uint32_t setmask, void *addr)
{
	uint32_t val;

	val = readl(addr);
	val &= ~clearmask;
	val |= setmask;
	writel(val, addr);

	return 0;
}

/* --------------------------------------------------------------------------
 * Debug functions
 * -------------------------------------------------------------------------- */

#if defined(DEBUG_PKT_DUMP)
static int32_t netx4000_gmac_packet_dump(uint8_t *buf, uint32_t nbytes)
{
	uint32_t n = 0;

	pr_debug("\n/*********************************************/\n");
	pr_debug("Packet of %d Bytes\n", nbytes);
	pr_debug("\nDst MAC addr (6 bytes): ");
	for (n = 0; n < 6; n++)
		pr_debug("%02x%s", buf[n], (((n == 5) ? "" : ":")));
	pr_debug("\nSrc MAC addr (6 bytes): ");
	for (n = 6; n <= 11; n++)
		pr_debug("%02x%s", buf[n], (((n == 11) ? "" : ":")));
	pr_debug("\nType/Length (2 bytes):  %04x\n", (buf[12] << 8 | buf[13]));

	pr_debug("\nPay Load : %d bytes\n", (nbytes - 14));
	for (n = 14; n < nbytes; n++) {
		pr_debug("%02x%s", buf[n], (((n - 13) % 16) && (n < nbytes - 1)) ? ":" : "");
		if (((n - 13) % 16) == 0)
			pr_debug("\n");
	}
	pr_debug("\n/*********************************************/\n\n");

	return 0;
}
#endif

/* --------------------------------------------------------------------------
 * MDIO bus functions
 * -------------------------------------------------------------------------- */

static int netx4000_gmac_mdio_write(struct mii_bus *bus, int addr, int reg, u16 regval)
{
	struct netx4000_gmac_priv *priv = bus->priv;
	void __iomem *regbase = priv->base;
	uint32_t val;
	uint64_t start;

//	pr_debug("%s: addr: 0x%02x reg: 0x%02x val: 0x%04x\n",__func__, addr, reg, regval);

	start = get_time_ns();
	while (readl(regbase + MAC_MDIO_ADDR) & MAC_MDIO_ADDR_GB)
		if (is_timeout(start, 100 * MSECOND))
			return -EIO;

	val = regval << MAC_MDIO_DATA_GD_shift;
	writel(val, regbase + MAC_MDIO_DATA);
	val = (addr << MAC_MDIO_ADDR_PA_shift) | (reg << MAC_MDIO_ADDR_GR_shift) | (0x5 << MAC_MDIO_ADDR_CR_shift) | (0x1 << MAC_MDIO_ADDR_GOC_shift) | MAC_MDIO_ADDR_GB;
	writel(val, regbase + MAC_MDIO_ADDR);

	start = get_time_ns();
	while (readl(regbase + MAC_MDIO_ADDR) & MAC_MDIO_ADDR_GB)
		if (is_timeout(start, 100 * MSECOND))
			return -EIO;

	return 0;
}

static int netx4000_gmac_mdio_read(struct mii_bus *bus, int addr, int reg)
{
	struct netx4000_gmac_priv *priv = bus->priv;
	void __iomem *regbase = priv->base;
	uint32_t val, regval;
	uint64_t start;

	start = get_time_ns();
	while (readl(regbase + MAC_MDIO_ADDR) & MAC_MDIO_ADDR_GB)
		if (is_timeout(start, 100 * MSECOND))
			return -EIO;

	val = (addr << MAC_MDIO_ADDR_PA_shift) | (reg << MAC_MDIO_ADDR_GR_shift) | (0x5 << MAC_MDIO_ADDR_CR_shift) | (0x3 << MAC_MDIO_ADDR_GOC_shift) | MAC_MDIO_ADDR_GB;
	writel(val, regbase + MAC_MDIO_ADDR);

	start = get_time_ns();
	while (readl(regbase + MAC_MDIO_ADDR) & MAC_MDIO_ADDR_GB)
		if (is_timeout(start, 100 * MSECOND))
			return -EIO;

	regval = (readl(regbase + MAC_MDIO_DATA) & MAC_MDIO_DATA_GD_mask) >> MAC_MDIO_DATA_GD_shift;

//	pr_debug("%s: addr: 0x%02x reg: 0x%02x val: 0x%04x\n", __func__, addr, reg, regval);

	return regval;
}

/* --------------------------------------------------------------------------
 * Interrupt function
 * -------------------------------------------------------------------------- */

//static int32_t netx4000_gmac_isr(struct eth_device *edev)
//{
//	struct netx4000_gmac_priv *priv = edev->priv;
//	void  __iomem *regbase = priv->base;
//	uint32_t val;
//
//	while ((val = readl(regbase + MAC_IS)) != 0) {
//		if (val & MAC_IS_GPIIS)
//			dev_dbg(edev->parent, "Unsupported interrupt (GPIIS)\n");
//		if (val & (MAC_IS_RXSTSIS | MAC_IS_TXSTSIS)) {
//			val = readl(regbase + MAC_RXTX_STATUS);
//			dev_dbg(edev->parent, "Receive or transmit error occured (MAC_RxTx_STATUS: 0x%08x)\n", val);
//		}
//		if (val & MAC_IS_TSIS)
//			dev_dbg(edev->parent, "Unsupported interrupt (TSIS)\n");
//		if (val & MAC_IS_MMCRXIPIS)
//			dev_dbg(edev->parent, "Unsupported interrupt (MMCRXIPIS)\n");
//		if (val & MAC_IS_MMCTXIS)
//			dev_dbg(edev->parent, "Unsupported interrupt (MMCTXIS)\n");
//		if (val & MAC_IS_MMCRXIS)
//			dev_dbg(edev->parent, "Unsupported interrupt (MMCRXIS)\n");
//		if (val & MAC_IS_MMCIS)
//			dev_dbg(edev->parent, "Unsupported interrupt (MMCIS)\n");
//		if (val & MAC_IS_LPIIS)
//			dev_dbg(edev->parent, "Unsupported interrupt (LPIIS)\n");
//		if (val & MAC_IS_PMTIS)
//			dev_dbg(edev->parent, "Unsupported interrupt (PMTIS)\n");
//		if (val & MAC_IS_PHYIS)
//			dev_dbg(edev->parent, "Unsupported interrupt (PHYIS)\n");
//		if (val & MAC_IS_PCSANCIS)
//			dev_dbg(edev->parent, "Unsupported interrupt (PCSANCIS)\n");
//		if (val & MAC_IS_PCSLCHGIS)
//			dev_dbg(edev->parent, "Unsupported interrupt (PCSLCHGIS)\n");
//		if (val & MAC_IS_RGSMIIIS) {
//			val = readl(regbase + MAC_PHYIF_CS);
//			if (val & MAC_PHYIF_CS_LNKSTS) {
//				uint32_t speed = 0, duplex;
//
//				if ((val & MAC_PHYIF_CS_LNKSPEED_mask) == MAC_PHYIF_CS_LNKSPEED_1000)
//					speed = 1000;
//				else if ((val & MAC_PHYIF_CS_LNKSPEED_mask) == MAC_PHYIF_CS_LNKSPEED_100)
//					speed = 100;
//				else if ((val & MAC_PHYIF_CS_LNKSPEED_mask) == MAC_PHYIF_CS_LNKSPEED_10)
//					speed = 10;
//
//				duplex = (val & MAC_PHYIF_CS_LNKMOD) ? 1 : 0;
//
//				dev_dbg(edev->parent, "Link status changed: link up - %dMbps %s\n", speed, (duplex) ? "FD" : "HD");
//			} else
//				dev_dbg(edev->parent, "Link status changed: link down\n");
//		}
//	}
//
//	return 0;
//}

/* --------------------------------------------------------------------------
 * Send and receive functions
 * -------------------------------------------------------------------------- */

static int32_t netx4000_gmac_send (struct eth_device *edev, void *packet, int length)
{
	struct netx4000_gmac_priv *priv = edev->priv;
	void  __iomem *regbase = priv->base;
	struct dummy_desc *txdesc;
	uint8_t *txbuffer;
	uint64_t start;

	txdesc = priv->tx_chain + priv->tx_currdesc;
	txbuffer = priv->txbuffer + (priv->tx_currdesc * ETH_BUF_SZ);

	start = get_time_ns();
	while (txdesc->des3 & TDES3_OWN) {
		if (is_timeout(start, 100 * MSECOND)) {
			dev_err(edev->parent, "TX timed out - no descriptor available!\n");
			return -EIO;
		}
	}

	memcpy(txbuffer, packet, length);
	txdesc->des0 = cpu_to_le32(txbuffer);
	txdesc->des1 = cpu_to_le32(0);
	txdesc->des2 = cpu_to_le32(length << TDES2_B1L_shift);
	txdesc->des3 = cpu_to_le32(TDES3_OWN | TDES3_FD | TDES3_LD | (length << TDES3_FL_shift));

	/* inc and wrap */
	if (++priv->tx_currdesc >= TX_NUM_DESC)
		priv->tx_currdesc = 0;

	dmb();

#if defined(DEBUG_PKT_DUMP)
	netx4000_gmac_packet_dump((uint8_t*)txdesc->des0, length); // only for debug
#endif

	writel(priv->tx_chain + priv->tx_currdesc, regbase + DMA_CH0_TXDESC_TAIL_POINTER);

	start = get_time_ns();
	while (txdesc->des3 & TDES3_OWN) {
		if (is_timeout(start, 100 * MSECOND)) {
			dev_err(edev->parent, "TX timed out\n");
			return -EIO;
		}
	}

	return 0;
}

static int32_t netx4000_gmac_recv (struct eth_device *edev)
{
	int32_t rc = 0;
	struct netx4000_gmac_priv *priv = edev->priv;
	void  __iomem *regbase = priv->base;
	struct dummy_desc *rxdesc;
	uint8_t *rxbuffer;
	uint32_t length;
	uint64_t start;

	rxdesc = priv->rx_chain + priv->rx_currdesc;
	rxbuffer = priv->rxbuffer + (priv->rx_currdesc * ETH_BUF_SZ);

	start = get_time_ns();
	while (rxdesc->des3 & RDES3_OWN) {
		if (is_timeout(start, 100 * MSECOND)) {
			dev_dbg(edev->parent, "RX timed out - no descriptor available!\n");
			return -EIO;
		}
	}

	do {
		/* Check for errors */
		if ((rxdesc->des3 & RDES3_FD) == 0) {
			dev_err(edev->parent, "Error while receiving data - RX descriptor is not the first one => descriptor is dropping!\n");
			rc = -EIO;
			break;
		}

		if ((rxdesc->des3 & RDES3_LD) == 0) {
			dev_err(edev->parent, "Error while receiving data - RX descriptor is not the last one => descriptor is dropping!\n");
			rc = -EIO;
			break;
		}

		if (rxdesc->des3 & RDES3_ES) {
			dev_err(edev->parent, "Error while receiving data - Dribble, Receive, Overflow, Watchdog, Descriptor or CRC error occured!\n");
			rc = -EIO;
			break;
		}

		/* Process the descriptor */
		length = (rxdesc->des3 & RDES3_PL_mask) >> RDES3_PL_shift;

#if defined(DEBUG_PKT_DUMP)
		netx4000_gmac_packet_dump((uint8_t*)rxdesc->des0, length);
#endif

		net_receive(edev, rxbuffer, length);
	} while (0);

	if (rc)
		dev_err(edev->parent, "rxdesc=%p, rxdesc->des3=0x%08x\n", rxdesc, rxdesc->des3);

	/* Freeing the desciptor and resume a suspended DMA */
	rxdesc->des3 = RDES3_OWN | RDES3_BUF1V;
	writel(priv->rx_chain + RX_NUM_DESC, regbase + DMA_CH0_RXDESC_TAIL_POINTER);

	/* inc and wrap */
	if (++priv->rx_currdesc >= RX_NUM_DESC)
		priv->rx_currdesc = 0;

	return (rc != 0) ? rc : 0;
}

/* --------------------------------------------------------------------------
 *
 * -------------------------------------------------------------------------- */

static void netx4000_gmac_update_linkspeed(struct eth_device *edev)
{
	struct netx4000_gmac_priv *priv = edev->priv;
	void __iomem *regbase = priv->base;
	uint32_t val;

	val = readl(regbase + MAC_CFG);
	if (edev->phydev->duplex)
		val |= MAC_CFG_DM;
	else
		val &= ~MAC_CFG_DM;

	if (edev->phydev->speed == SPEED_1000)
		val &= ~MAC_CFG_PS;
	else
		val |= MAC_CFG_PS;

	if (edev->phydev->speed == SPEED_100)
		val |= MAC_CFG_FES;
	else
		val &= ~MAC_CFG_FES;

	writel(val, regbase + MAC_CFG);

	return;
}

static int32_t netx4000_gmac_open(struct eth_device *edev)
{
	struct netx4000_gmac_priv *priv = edev->priv;
	void __iomem *regbase = priv->base;
	int32_t rc;

	rc = phy_device_connect(edev, NULL /*&priv->miibus*/, -1 /*priv->phyid*/, netx4000_gmac_update_linkspeed, 0, priv->interface);
	if (rc)
		return rc;

	/* DMA: Enable TX and RX */
	netx4000_ioset32(DMA_CH0_TX_CONTROL_ST, regbase + DMA_CH0_TX_CONTROL);
	netx4000_ioset32(DMA_CH0_RX_CONTROL_SR, regbase + DMA_CH0_RX_CONTROL);

	/* MAC: Enable TX and RX*/
	netx4000_ioset32(MAC_CFG_TE | MAC_CFG_RE, regbase + MAC_CFG);

	return 0;
}

static void netx4000_gmac_halt (struct eth_device *edev)
{
	pr_warn("Entering unsupported function %s()\n", __func__);

	return;
}

static int netx4000_gmac_get_ethaddr(struct eth_device *edev, u8 *addr)
{
	struct netx4000_gmac_priv *priv = edev->priv;
	void __iomem *regbase = priv->base;
	uint32_t val;

	val = readl(regbase + MAC_ADDRESS_0_HIGH);
	addr[0] = (uint8_t)((val >> 8) & 0xff);
	addr[1] = (uint8_t)((val >> 0) & 0xff);

	val = readl(regbase + MAC_ADDRESS_0_LOW);
	addr[2] = (uint8_t)((val >> 24) & 0xff);
	addr[3] = (uint8_t)((val >> 16) & 0xff);
	addr[4] = (uint8_t)((val >> 8) & 0xff);
	addr[5] = (uint8_t)((val >> 0) & 0xff);

//	dev_dbg(edev->parent, "%s: %02x:%02x:%02x:%02x:%02x:%02x\n", __func__, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	return 0;
}

static int netx4000_gmac_set_ethaddr(struct eth_device *edev, const unsigned char *addr)
{
	struct netx4000_gmac_priv *priv = edev->priv;
	void __iomem *regbase = priv->base;

	netx4000_iomod32(0x0000ffff, (addr[0] << 8) | (addr[1]), regbase + MAC_ADDRESS_0_HIGH);
	netx4000_iomod32(0xffffffff, (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | (addr[5]), regbase + MAC_ADDRESS_0_LOW);

//	dev_dbg(edev->parent, "%s: %02x:%02x:%02x:%02x:%02x:%02x\n", __func__, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	return 0;
}

/* --------------------------------------------------------------------------
 * Initialization functions for the driver and the chip
 * -------------------------------------------------------------------------- */

static int32_t init_tx_desc(struct netx4000_gmac_priv *priv)
{
	void __iomem *regbase = priv->base;
	struct dummy_desc *txdesc = priv->tx_chain;
	uint32_t i;

	for (i = 0; i < TX_NUM_DESC; i++)
		memset(txdesc + i, 0, sizeof(*txdesc));
	dmb();

	writel(TX_NUM_DESC - 1, regbase + DMA_CH0_TXDESC_RING_LENGTH);
	writel(priv->tx_chain, regbase + DMA_CH0_TXDESC_LIST_ADDR);
	writel(priv->tx_chain, regbase + DMA_CH0_TXDESC_TAIL_POINTER);

	priv->tx_currdesc = 0;

	return 0;
}

static int32_t init_rx_desc(struct netx4000_gmac_priv *priv)
{
	void __iomem *regbase = priv->base;
	struct dummy_desc *rxdesc = priv->rx_chain;
	uint32_t i;

	for (i = 0; i < RX_NUM_DESC; i++) {
		memset(rxdesc + i, 0, sizeof(*rxdesc));
		(rxdesc + i)->des0 = (uint32_t)(priv->rxbuffer + (i * ETH_BUF_SZ));
		(rxdesc + i)->des3 = RDES3_OWN | RDES3_BUF1V;
	}
	dmb();

	writel(RX_NUM_DESC - 1, regbase + DMA_CH0_RXDESC_RING_LENGTH);
	writel(priv->rx_chain, regbase + DMA_CH0_RXDESC_LIST_ADDR);
	writel(priv->rx_chain + RX_NUM_DESC, regbase + DMA_CH0_RXDESC_TAIL_POINTER);

	priv->rx_currdesc = 0;

	return 0;
}

/* FIXME: Move it to a common platform code */
#define NOCPWRCTRL	0xf8000040
#define NOCPWRMASK	0xf8000044
#define	NOCPWRSTAT	0xf8000048
#define CLKCFG		0xf800004c
#define GMAC_CLOCK	(1 << 3)
static int32_t netx4000_gmac_clock_enable(void)
{
	uint64_t start;

	start = get_time_ns();
	while ((readl(NOCPWRSTAT) & GMAC_CLOCK) == 0) {
		if (is_timeout(start, 100 * MSECOND))
			return -1;
		netx4000_ioset32(GMAC_CLOCK, (void *)CLKCFG);
		netx4000_ioset32(GMAC_CLOCK, (void *)NOCPWRMASK);
		netx4000_ioset32(GMAC_CLOCK, (void *)NOCPWRCTRL);
	}
	return 0;
}

static int32_t netx4000_gmac_dma_init(struct netx4000_gmac_priv *priv)
{
	void __iomem *regbase = priv->base;
	uint64_t start;

	/* Provide a software reset. This resets all of the MAC internal registers and logic. */
	netx4000_ioset32(DMA_MODE_SWR, regbase + DMA_MODE);
	start = get_time_ns();
	while (readl(regbase + DMA_MODE) & DMA_MODE_SWR) {
		if (is_timeout(start, 100 * MSECOND)) {
			dev_err(priv->edev.parent, "DMA reset failed\n");
			return -EIO;
		}
	}

	netx4000_ioset32(DMA_CH0_CONTROL_PBLx8, regbase + DMA_CH0_CONTROL);

	/* Configure bus access */
	writel(DMA_SYSBUS_MODE_RD_OSR_LMT_mask /*| DMA_SYSBUS_MODE_WR_OSR_LMT_mask*/
//			| DMA_SYSBUS_MODE_BLEN128
//			| DMA_SYSBUS_MODE_BLEN64
//			| DMA_SYSBUS_MODE_BLEN32
			| DMA_SYSBUS_MODE_BLEN16 // *
			| DMA_SYSBUS_MODE_BLEN8
			| DMA_SYSBUS_MODE_BLEN4 // *
			| DMA_SYSBUS_MODE_FB, regbase + DMA_SYSBUS_MODE);

	/* Create and initialize a descriptor list for TX and RX */
	if (((priv->tx_chain = dma_alloc_coherent(TX_NUM_DESC * sizeof(*priv->tx_chain), DMA_ADDRESS_BROKEN)) == NULL) ||
			((priv->txbuffer = dma_alloc_coherent(TX_BUF_SZ, DMA_ADDRESS_BROKEN)) == NULL) ||
			((priv->rx_chain = dma_alloc_coherent(RX_NUM_DESC * sizeof(*priv->rx_chain), DMA_ADDRESS_BROKEN)) == NULL) ||
			((priv->rxbuffer = dma_alloc_coherent(RX_BUF_SZ, DMA_ADDRESS_BROKEN)) == NULL)) {

		return -ENOMEM;
	}

	init_tx_desc(priv);
	init_rx_desc(priv);

	/* TX Programmable Burst Length (4) */
	writel((4 << DMA_CH0_TX_CONTROL_TXPBL_shift), regbase + DMA_CH0_TX_CONTROL);

	/* RX Programmable Burst Length (4), Receive Buffer size (ETH_BUF_SZ) */
	writel((4 << DMA_CH0_RX_CONTROL_RXPBL_shift) | ((ETH_BUF_SZ << DMA_CH0_RX_CONTROL_RBSZ_shift) & DMA_CH0_RX_CONTROL_RBSZ_mask), regbase + DMA_CH0_RX_CONTROL);

	dmb();

	return 0;
}

static int32_t netx4000_gmac_mtl_init(struct netx4000_gmac_priv *priv)
{
	void __iomem *regbase = priv->base;

	/* Transmit Queue Size (2048), Transmit Store and Forward, Transmit Queue Enable */
	writel((0x7 << MTL_TXQ0_OM_TQS_shift) | MTL_TXQ0_OM_TSF | MTL_TXQ0_OM_TXQEN_en, regbase + MTL_TXQ0_OM);

	/* Receive Queue Size (2048), Receive Queue Store and Forward */
	writel((0x7 << MTL_RXQ0_OM_RQS_shift) | MTL_RXQ0_OM_RSF, regbase + MTL_RXQ0_OM);

	dmb();

	return 0;
}

static int32_t netx4000_gmac_mac_init(struct netx4000_gmac_priv *priv)
{
	void __iomem *regbase = priv->base;

	/* if the mac address is still invalid, use a autogenerated mac address related to the chip uuid */
	if (priv->edev.ethaddr[0] == 0xff) {
		uint32_t chipid[4], i;
		struct digest *digest;
		unsigned char *hash;

		chipid[0] = readl(0xf80000b0);
		chipid[1] = readl(0xf80000b4);
		chipid[2] = readl(0xf80000b8);
		chipid[3] = readl(0xf80000bc);
		dev_dbg(priv->edev.parent, "chipid: %08x-%08x-%08x-%08x\n", chipid[0], chipid[1], chipid[2], chipid[3]);

		digest = digest_alloc("sha256");
		if (!digest) {
			dev_err(priv->edev.parent, "digest_alloc() failed");
			return -ENODEV;
		}
		hash = xzalloc(digest_length(digest));

		if (!hash) {
			dev_err(priv->edev.parent, "xzalloc() failed");
			return -ENOMEM;
		}

		digest_digest(digest, &chipid[0], sizeof(chipid[0]) * 4, hash);
		dev_dbg(priv->edev.parent, "hash: ");
		for (i = 0; i < digest_length(digest); i++)
			pr_debug("%02x", hash[i]);
		pr_debug("\n");

		for (i=5; i > 0; i--) {
			if (i == 5)
				hash[i] += priv->edev.dev.id;
			else
				hash[i]++;
			if (hash[i])
				break;
		}
		hash[0] &= ~0x1; /* mark it as individual mac address */
		hash[0] |= 0x2;  /* mark it as local mac address */

		memcpy(priv->edev.ethaddr, hash, sizeof(priv->edev.ethaddr));

		dev_warn(priv->edev.parent, "using automatically generated mac address (%02x:%02x:%02x:%02x:%02x:%02x)\n"
					, priv->edev.ethaddr[0], priv->edev.ethaddr[1], priv->edev.ethaddr[2], priv->edev.ethaddr[3], priv->edev.ethaddr[4], priv->edev.ethaddr[5]);
	}

	netx4000_gmac_set_ethaddr(&priv->edev, priv->edev.ethaddr);

	/* Receive All */
	writel(MAC_PF_RA, regbase + MAC_PF);

	dmb();

	return 0;
}

/* called by eth_register() */
static int32_t netx4000_gmac_chip_init(struct eth_device *edev)
{
	struct netx4000_gmac_priv *priv = edev->priv;

	netx4000_gmac_dma_init(priv);
	netx4000_gmac_mtl_init(priv);
	netx4000_gmac_mac_init(priv);

	return 0;
}

static int32_t netx4000_gmac_probe(struct device_d *dev)
{
	int32_t rc = 0;
	struct netx4000_gmac_priv *priv;
	struct device_node *mdionode;

	if ((priv = xzalloc(sizeof(*priv))) == NULL)
		return -ENOMEM;

	priv->base = dev_request_mem_region(dev, 0);
	if (IS_ERR(priv->base)){
		rc = PTR_ERR(priv->base);
		goto err_out;
	}

	if (netx4000_gmac_clock_enable() < 0) {
		dev_err(dev, "GMAC clock setup failed\n");
		rc = -EIO;
		goto err_out;
	}

	mdionode = of_get_child_by_name(dev->device_node, "mdio");
	if(mdionode) {
		priv->miibus.priv = priv;
		priv->miibus.parent = dev;
		priv->miibus.dev.device_node = mdionode;
		priv->miibus.write = netx4000_gmac_mdio_write;
		priv->miibus.read = netx4000_gmac_mdio_read;

		mdiobus_register(&priv->miibus);
	}

	priv->edev.priv = priv;
	priv->edev.parent = dev;
	priv->edev.init = netx4000_gmac_chip_init;
	priv->edev.open = netx4000_gmac_open;
	priv->edev.send = netx4000_gmac_send;
	priv->edev.recv = netx4000_gmac_recv;
	priv->edev.halt = netx4000_gmac_halt;
	priv->edev.get_ethaddr = netx4000_gmac_get_ethaddr;
	priv->edev.set_ethaddr = netx4000_gmac_set_ethaddr;

	priv->interface = of_get_phy_mode(dev->device_node);

	/* read out the mac address from chip */
	netx4000_gmac_get_ethaddr(&priv->edev, (u8*)&priv->edev.ethaddr);

	/* if the mac address is invalid, use the mac address from bareboxenv instead */
	if (priv->edev.ethaddr[0] == 0xff) { /* bareboxenv */
		const char *ethaddr;

		if (!getenv("eth0.ethaddr"))
			ethaddr = getenv("global.eth0_ethaddr");
		else
			ethaddr = getenv("global.eth1_ethaddr");

		if (ethaddr) {
			string_to_ethaddr(ethaddr, priv->edev.ethaddr);
			dev_info(dev, "using mac address from bareboxenv (%02x:%02x:%02x:%02x:%02x:%02x)\n"
				, priv->edev.ethaddr[0], priv->edev.ethaddr[1], priv->edev.ethaddr[2], priv->edev.ethaddr[3], priv->edev.ethaddr[4], priv->edev.ethaddr[5]);
		}
	}

	/* if the mac address is still invalid, use the mac address from devicelabel instead */
	if (priv->edev.ethaddr[0] == 0xff) { /* device label */
		const char *env_ethaddr = NULL;
		char ethaddr[18] = "\0";

		if (!getenv("eth0.ethaddr"))
			env_ethaddr = getenv("devicelabel_eth0_ethaddr");
		else
			env_ethaddr = getenv("devicelabel_eth1_ethaddr");

		if (env_ethaddr) {
			if ((env_ethaddr[0] == '"') || (env_ethaddr[0] == '\''))
				env_ethaddr++;
			strncat(ethaddr, env_ethaddr, 17);
			string_to_ethaddr(ethaddr, priv->edev.ethaddr);
			dev_info(dev, "using mac address from devicelable (%02x:%02x:%02x:%02x:%02x:%02x)\n"
				, priv->edev.ethaddr[0], priv->edev.ethaddr[1], priv->edev.ethaddr[2], priv->edev.ethaddr[3], priv->edev.ethaddr[4], priv->edev.ethaddr[5]);
		}
	}

	/* if the mac address is still invalid, use the mac address from the fdt instead */
	if (priv->edev.ethaddr[0] == 0xff) { /* fdt */
		if (of_get_mac_address(dev->device_node)) {
			memcpy(priv->edev.ethaddr, of_get_mac_address(dev->device_node), sizeof(priv->edev.ethaddr));
			dev_info(dev, "using mac address from fdt (%02x:%02x:%02x:%02x:%02x:%02x)\n"
					, priv->edev.ethaddr[0], priv->edev.ethaddr[1], priv->edev.ethaddr[2], priv->edev.ethaddr[3], priv->edev.ethaddr[4], priv->edev.ethaddr[5]);
		}
	}

	if ((rc = eth_register(&priv->edev)) != 0) {
		dev_err(dev, "register eth%d failed\n", priv->edev.dev.id);
		goto err_out;
	}

	dev_info(dev, "eth%d successfully initialized!\n", priv->edev.dev.id);

	return 0;

err_out:
	if (priv != NULL)
		free(priv);

	return rc;
}

static __maybe_unused struct of_device_id netx4000_gmac_dt_ids[] = {
	{
		.compatible = "hilscher,netx4000-gmac",
	}, {
		/* sentinel */
	},
};

static struct driver_d netx4000_gmac_driver = {
	.name  = DRIVER_NAME,
	.probe = netx4000_gmac_probe,
	.of_compatible = DRV_OF_COMPAT(netx4000_gmac_dt_ids),
};

static int __init netx4000_gmac_init(void)
{
	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return platform_driver_register(&netx4000_gmac_driver);
}
postenvironment_initcall(netx4000_gmac_init);

/* --- Module information --- */

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
