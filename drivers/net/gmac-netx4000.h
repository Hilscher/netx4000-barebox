/*
* drivers/net/netx4000-gmac.h
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

#define TX_NUM_DESC			8
#define RX_NUM_DESC			16

#define ETH_BUF_SZ			2048
#define TX_BUF_SZ			(ETH_BUF_SZ * TX_NUM_DESC)
#define RX_BUF_SZ			(ETH_BUF_SZ * RX_NUM_DESC)



struct dummy_desc {
	uint32_t des0;
	uint32_t des1;
	uint32_t des2;
	uint32_t des3;
} __attribute__((packed, aligned(4)));

/* Transmit Normal Descriptor (Read Format) */

struct tdesc_rd {
	void	 *desc0;
	void	 *desc1;
	uint32_t desc2;
	uint32_t desc3;
} __attribute__((packed, aligned(4)));

#define TDES2_IC		(1 << 31)
#define TDES2_TTSE		(1 << 30)
#define TDES2_B2L_shift		16
#define TDES2_B2L_mask		(0x3fff << TDESC_B2L_shift)
#define TDES2_VTIR_shift	14
#define TDES2_VTIR_mask		(0x3 << TDES2_VTIR_shift)
#define TDES2_HL_shift		0
#define TDES2_HL_mask		(0x3fff << TDES2_HL_shift)
#define TDES2_B1L_shift		0
#define TDES2_B1L_mask		(0x3fff << TDES2_B1L_shift)

#define TDES3_OWN		(1 << 31)
#define TDES3_CTXT		(1 << 30)
#define TDES3_FD		(1 << 29)
#define TDES3_LD		(1 << 28)
#define TDES3_CPC_shift		26
#define TDES3_CPC_mask		(0x3 << TDES3_CPC_shift)
#define TDES3_SAIC_shift	23
#define TDES3_SAIC_mask		(0x7 << TDES3_SAIC_shift)
#define TDES3_SLOTNUM_shift	19
#define TDES3_SLOTNUM_mask	(0xf << TDES3_SLOTNUM_shift)
#define TDES3_TCPHDRLEN_shift	19
#define TDES3_TCPHDRLEN_mask	(0xf << TDES3_TCPHDRLEN_shift)
#define TDES3_TSE		(1 << 18)
#define TDES3_CIC_shift		16
#define TDES3_CIC_mask		(0x3 << TDES3_CIC_shift)
#define TDES3_TIPLH		(1 << 15)
#define TDES3_FL_shift		0
#define TDES3_FL_mask		(0x7fff << TDES3_FL_shift)

/* Transmit Normal Descriptor (Write-Back Format) */

struct tdesc_wb {
	uint32_t tsl;
	uint32_t tsh;
	uint32_t des2;
	uint32_t status;
} __attribute__((packed, aligned(4)));

#define STATUS_OWN		(1 << 31)
#define STATUS_CTXT		(1 << 30)
#define STATUS_FD		(1 << 29)
#define STATUS_LD		(1 << 28)
#define STATUS_TTSS		(1 << 17)
#define STATUS_ES		(1 << 15)
#define STATUS_JT		(1 << 14)
#define STATUS_FF		(1 << 13)
#define STATUS_PCE		(1 << 12)
#define STATUS_LOC		(1 << 11)
#define STATUS_NC		(1 << 10)
#define STATUS_LC		(1 << 9)
#define STATUS_EC		(1 << 8)
#define STATUS_CC_shift		4
#define STATUS_CC_mask		(0xf << STATUS_CC_shift)
#define STATUS_ED		(1 << 3)
#define STATUS_UF		(1 << 2)
#define STATUS_DB		(1 << 1)
#define STATUS_IHE		(1 << 0)
#define STATUS_Rsvd_mask	((0x7ff << 18) | (1 << 16)) /* reserved */

/* Receive Normal Descriptor (Read Format) */

struct rdesc_rd {
	void	 *decs0;
	uint32_t desc1;
	void	 *desc2;
	uint32_t desc3;
} __attribute__((packed, aligned(4)));

#define RDES3_OWN		(1 << 31)
#define RDES3_INTE		(1 << 30)
#define RDES3_BUF2V		(1 << 25)
#define RDES3_BUF1V		(1 << 24)
#define RDES3_Rsvd_mask		((0xf << 26) | (0xffffff << 0)) /* reserved */

/* Receive Normal Descriptor (Write-Back Format) */

struct rdesc_wb {
	uint32_t desc0;
	uint32_t desc1;
	uint32_t desc2;
	uint32_t desc3;
} __attribute__((packed, aligned(4)));

#define RDES3_OWN		(1 << 31)
#define RDES3_CTXT		(1 << 30)
#define RDES3_FD		(1 << 29)
#define RDES3_LD		(1 << 28)
#define RDES3_RS2V		(1 << 27)
#define RDES3_RS1V		(1 << 26)
#define RDES3_RS0V		(1 << 25)
#define RDES3_CE		(1 << 24)
#define RDES3_GP		(1 << 23)
#define RDES3_RWT		(1 << 22)
#define RDES3_OE		(1 << 21)
#define RDES3_RE		(1 << 20)
#define RDES3_DE		(1 << 19)
#define RDES3_LT_shift		18
#define RDES3_LT_mask		(0x7 << RDES3_LT_shift)
#define RDES3_ES		(1 << 15)
#define RDES3_PL_shift		0
#define RDES3_PL_mask		(0x7fff << RDES3_PL_shift)

/* MAC Register */

#define MAC_BASE		0x0000

#define MAC_CFG			0x0000	/* MAC_CONFIGURATION */
#define MAC_CFG_SARC_shift	28
#define MAC_CFG_SARC_mask	(0x7 << MAC_CFG_SARC_shift)
#define MAC_CFG_IPC		(1 << 27)
#define MAC_CFG_IPG_shift	24
#define MAC_CFG_IPG_mask	(0x7 << MAC_CFG_IPG_shift)
#define MAC_CFG_GPSLCE		(1 << 23)
#define MAC_CFG_S2KP		(1 << 22)
#define MAC_CFG_CST		(1 << 21)
#define MAC_CFG_ACS		(1 << 20)
#define MAC_CFG_WD		(1 << 19)
#define MAC_CFG_BE		(1 << 18)
#define MAC_CFG_JD		(1 << 17)
#define MAC_CFG_JE		(1 << 16)
#define MAC_CFG_PS		(1 << 15)
#define MAC_CFG_FES		(1 << 14)
#define MAC_CFG_DM		(1 << 13)
#define MAC_CFG_LM		(1 << 12)
#define MAC_CFG_ECRSFD		(1 << 11)
#define MAC_CFG_DO		(1 << 10)
#define MAC_CFG_DCRS		(1 << 9)
#define MAC_CFG_DR		(1 << 8)
#define MAC_CFG_BL_shift	5
#define MAC_CFG_BL_mask		(0x3 << MAC_CFG_BL_shift)
#define MAC_CFG_DC		(1 << 4)
#define MAC_CFG_PRELEN_shift	2
#define MAC_CFG_PRELEN_mask	(0x3 << MAC_CFG_PRELEN_shift)
#define MAC_CFG_TE		(1 << 1)
#define MAC_CFG_RE		(1 << 0)
#define MAC_CFG_Rsvd_mask	((0x1 << 31) | (0x1 << 7)) /* reserved */

#define MAC_PF			0x0008	/* MAC_PACKET_FILTER */
#define MAC_PF_RA		(1 << 31)
#define MAC_PF_DNTU		(1 << 21)
#define MAC_PF_IPFE		(1 << 20)
#define MAC_PF_VTFE		(1 << 16)
#define MAC_PF_HPF		(1 << 10)
#define MAC_PF_SAF		(1 << 9)
#define MAC_PF_SAIF		(1 << 8)
#define MAC_PF_PCF_shift	6
#define MAC_PF_PCF_mask		(0x3 << MAC_PF_PCF_shift)
#define MAC_PF_DBF		(1 << 5)
#define MAC_PF_PM		(1 << 4)
#define MAC_PF_DAIF		(1 << 3)
#define MAC_PF_HMC		(1 << 2)
#define MAC_PF_HUC		(1 << 1)
#define MAC_PF_PR		(1 << 0)
#define MAC_PF_Rsvd_mask	((0x1ff << 22) | (0x7 << 17) | (0x1f << 11)) /* reserved */

#define MAC_Q0TXFCR		0x0070	/* MAC_Q0_TX_FLOW_CTRL */
#define MAC_Q0TXFCR_PT_shift	16
#define MAC_Q0TXFCR_PT_mask	(0xff << MAC_Q0TXFCR_PT_shift)
#define MAC_Q0TXFCR_DZPQ	(1 << 7)
#define MAC_Q0TXFCR_PLT_shift	4
#define MAC_Q0TXFCR_PLT_mask	(0x3 << MAC_Q0TXFCR_PLT_shift)
#define MAC_Q0TXFCR_TFE		(1 << 1)
#define MAC_Q0TXFCR_FCB_BPA	(1 << 0)
#define MAC_Q0TXFCR_Rsvd_mask	((0xff << 8) | (0x3 << 2)) /* reserved */

#define MAC_IS			0x00b0 /* MAC_INTERRUPT_STATUS */
#define MAC_IS_GPIIS		(1 << 15)
#define MAC_IS_RXSTSIS		(1 << 14)
#define MAC_IS_TXSTSIS		(1 << 13)
#define MAC_IS_TSIS		(1 << 12)
#define MAC_IS_MMCRXIPIS	(1 << 11)
#define MAC_IS_MMCTXIS		(1 << 10)
#define MAC_IS_MMCRXIS		(1 << 9)
#define MAC_IS_MMCIS		(1 << 8)
#define MAC_IS_LPIIS		(1 << 5)
#define MAC_IS_PMTIS		(1 << 4)
#define MAC_IS_PHYIS		(1 << 3)
#define MAC_IS_PCSANCIS		(1 << 2)
#define MAC_IS_PCSLCHGIS	(1 << 1)
#define MAC_IS_RGSMIIIS		(1 << 0)
#define MAC_IS_Rsvd_mask	((0xff < 16) | (0x3 << 6)) /* reserved */

#define MAC_IE			0x00b4	/* MAC_INTERRUPT_ENABLE */
#define MAC_IE_RXSTSIE		(1 << 14)
#define MAC_IE_TXSTSIE		(1 << 13)
#define MAC_IE_TSIE		(1 << 12)
#define MAC_IE_LPIIE		(1 << 5)
#define MAC_IE_PMTIE		(1 << 4)
#define MAC_IE_PHYIE		(1 << 3)
#define MAC_IE_PCSANCIE		(1 << 2)
#define MAC_IE_PCSLCHGIE	(1 << 1)
#define MAC_IE_RGSMIIIE		(1 << 0)
#define MAC_IE_Rsvd_mask	((0x1ff < 15) | (0x3f << 6)) /* reserved */

#define MAC_RXTX_STATUS		0x00b8 /* MAC_Rx_Tx_Status */

#define MAC_PHYIF_CS			0x00f8 /* MAC_PHYIF_Control_Status */
#define MAC_PHYIF_CS_FALSCARDET		(1 << 21)
#define MAC_PHYIF_CS_JABTO		(1 << 20)
#define MAC_PHYIF_CS_LNKSTS		(1 << 19)
#define MAC_PHYIF_CS_LNKSPEED_shift	17
#define MAC_PHYIF_CS_LNKSPEED_mask	(0x3 << MAC_PHYIF_CS_LNKSPEED_shift)
#define MAC_PHYIF_CS_LNKSPEED_10	(0b00 << MAC_PHYIF_CS_LNKSPEED_shift)
#define MAC_PHYIF_CS_LNKSPEED_100	(0b01 << MAC_PHYIF_CS_LNKSPEED_shift)
#define MAC_PHYIF_CS_LNKSPEED_1000	(0b10 << MAC_PHYIF_CS_LNKSPEED_shift)
#define MAC_PHYIF_CS_LNKMOD		(1 << 16)
#define MAC_PHYIF_CS_SMIDRXS		(1 << 4)
#define MAC_PHYIF_CS_SFTERR		(1 << 2)
#define MAC_PHYIF_CS_LUD		(1 << 1)
#define MAC_PHYIF_CS_TC			(1 << 0)
#define MAC_PHYIF_CS_Rsvd_mask		((0x3ff << 22) | (0x7ff << 5) | ((1 << 3))) /* reserved */

#define MAC_VERSION		0x0110	/* This register identifies the version of the DWC_ether_qos core. */
#define MAC_DEBUG		0x0114	/* This register provides the debug status of various MAC blocks. */

#define MAC_HW_FEATURE_0	0x011C	/* This register indicates the presence of the optional features of the DWC_ether_qos core. */
#define MAC_HW_FEATURE_1	0x0120	/* This register indicates the presence of optional features of the DMA and the MTL. */
#define MAC_HW_FEATURE_2	0x0124	/* This register indicates the number of channels selected in the DMA and the number of queues selected in the MTL. */

#define MAC_ADDRESS_0_HIGH	0x0300	/* This register contains the higher 16 bits of the first MAC address. */
#define MAC_ADDRESS_0_LOW	0x0304	/* This register contains the lower 32 bits of the first MAC address. */


#define MAC_MDIO_ADDR		0x0200
#define MAC_MDIO_ADDR_PA_shift	21
#define MAC_MDIO_ADDR_PA_mask	(0x1f << MAC_MDIO_ADDR_PA_shift)
#define MAC_MDIO_ADDR_GR_shift	16
#define MAC_MDIO_ADDR_GR_mask	(0x1f << MAC_MDIO_ADDR_GR_shift)
#define MAC_MDIO_ADDR_CR_shift	8
#define MAC_MDIO_ADDR_CR_mask	(0xf << MAC_MDIO_ADDR_CR_shift)
#define MAC_MDIO_ADDR_SKAP	(1 << 4)
#define MAC_MDIO_ADDR_GOC_shift	2
#define MAC_MDIO_ADDR_GOC_mask	(0x3 << MAC_MDIO_ADDR_GOC_shift)
#define MAC_MDIO_ADDR_C45E	(1 << 1)
#define MAC_MDIO_ADDR_GB	(1 << 0)
#define MAC_MDIO_ADDR_Rsvd_mask	((0x3f << 26) | (0xf << 12) | (0x7 << 5)) /* reserved */

#define MAC_MDIO_DATA		0x0204
#define MAC_MDIO_DATA_RA_shift	16
#define MAC_MDIO_DATA_RA_mask	(0xffff << MAC_MDIO_DATA_RA_shift)
#define MAC_MDIO_DATA_GD_shift	0
#define MAC_MDIO_DATA_GD_mask	(0xffff << MAC_MDIO_DATA_GD_shift)

/* MLT Register */

#define MLT_BASE		0x0c00

#define MTL_TXQ0_OM		0x0d00	/* MTL_TxQ0_Operation_Mode */
#define MTL_TXQ0_OM_TQS_shift	16
#define MTL_TXQ0_OM_TQS_mask	(0x1ff << MTL_TXQ0_OM_TQS_shift)
#define MTL_TXQ0_OM_TTC_shift	4
#define MTL_TXQ0_OM_TTC_mask	(0x7 << MTL_TXQ0_OM_TTC_shift)
#define MTL_TXQ0_OM_TXQEN_shift	2
#define MTL_TXQ0_OM_TXQEN_mask	(0x3 << MTL_TXQ0_OM_TXQEN_shift)
#define MTL_TXQ0_OM_TXQEN_dis	(0x0 << MTL_TXQ0_OM_TXQEN_shift)
#define MTL_TXQ0_OM_TXQEN_en	(0x2 << MTL_TXQ0_OM_TXQEN_shift)
#define MTL_TXQ0_OM_TSF		(1 << 1)
#define MTL_TXQ0_OM_FTQ		(1 << 0)
#define MTL_TXQ0_OM_Rsvd_mask	((0x7f << 25) | (0x1ff << 7)) /* reserved */

#define MTL_RXQ0_OM		0x0d30	/* MTL_RxQ0_Operation_Mode */
#define MTL_RXQ0_OM_RQS_shift	20
#define MTL_RXQ0_OM_RQS_mask	(0x3ff << MTL_RXQ0_OM_RQS_shift)
#define MTL_RXQ0_OM_RFD_shift	13
#define MTL_RXQ0_OM_RFD_mask	(0x7 << MTL_RXQ0_OM_RFD_shift)
#define MTL_RXQ0_OM_RFA_shift	8
#define MTL_RXQ0_OM_RFA_mask	(0x7 << #define MTL_RXQ0_OM_RFA_shift)
#define MTL_RXQ0_OM_EHFC	(1 << 7)
#define MTL_RXQ0_OM_DIS_TCP_EF	(1 << 6)
#define MTL_RXQ0_OM_RSF		(1 << 5)
#define MTL_RXQ0_OM_FEP		(1 << 4)
#define MTL_RXQ0_OM_FUP		(1 << 3)
#define MTL_RXQ0_OM_RTC_shift	0
#define MTL_RXQ0_OM_RTC_mask	(0x3 << MTL_RXQ0_OM_RTC_shift)
#define MLT_RXQ0_OM_Rsvd_mask	((0x1 << 2) | (0x3 << 11) | (0xf << 16) | (0x3 << 30)) /* reserved */

/* DMA Register */

#define DMA_BASE		0x1000

#define DMA_MODE		0x1000
#define DMA_MODE_PR_shift	12
#define DMA_MODE_PR_mask	(0x7 << DMA_MODE_PR_shift)
#define DMA_MODE_TXPR		(1 << 11)
#define DMA_MODE_TAA_shift	2
#define DMA_MODE_TAA_mask	(0x7 << DMA_MODE_TAA_shift)
#define DMA_MODE_DA		(1 << 1)
#define DMA_MODE_SWR		(1 << 0)
#define DMA_MODE_Rsvd_mask	((0xffff << 15) | (0x3f << 5))

#define DMA_SYSBUS_MODE				0x1004
#define DMA_SYSBUS_MODE_EN_LPI			(1 << 31)
#define DMA_SYSBUS_MODE_LPI_XIT_PKT		(1 << 30)
#define DMA_SYSBUS_MODE_WR_OSR_LMT_shift	24
#define DMA_SYSBUS_MODE_WR_OSR_LMT_mask		(0xf << DMA_SYSBUS_MODE_WR_OSR_LMT_shift)
#define DMA_SYSBUS_MODE_RD_OSR_LMT_shift	16
#define DMA_SYSBUS_MODE_RD_OSR_LMT_mask		(0xf << DMA_SYSBUS_MODE_RD_OSR_LMT_shift)
#define DMA_SYSBUS_MODE_RB			(1 << 15
#define DMA_SYSBUS_MODE_MB			(1 << 14)
#define DMA_SYSBUS_MODE_ONEKBBE			(1 << 13)
#define DMA_SYSBUS_MODE_AAL			(1 << 12)
#define DMA_SYSBUS_MODE_BLEN256			(1 << 7)
#define DMA_SYSBUS_MODE_BLEN128			(1 << 6)
#define DMA_SYSBUS_MODE_BLEN64			(1 << 5)
#define DMA_SYSBUS_MODE_BLEN32			(1 << 4)
#define DMA_SYSBUS_MODE_BLEN16			(1 << 3)
#define DMA_SYSBUS_MODE_BLEN8			(1 << 2)
#define DMA_SYSBUS_MODE_BLEN4			(1 << 1)
#define DMA_SYSBUS_MODE_FB			(1 << 0)
#define DMA_SYSBUS_MODE_Rsvd_mask		((0x3 << 28) | (0xf << 20) | (0xf << 8))

#define DMA_CH0_CONTROL				0x1100
#define DMA_CH0_CONTROL_SPH			(1 << 24)
#define DMA_CH0_CONTROL_DSL_shift		18
#define DMA_CH0_CONTROL_DSL_mask		(0x7 << DMA_CH0_CONTROL_DSL_shift)
#define DMA_CH0_CONTROL_PBLx8			(1 << 16)
#define DMA_CH0_CONTROL_MSS_shift		0
#define DMA_CH0_CONTROL_MSS_mask		(0x3fff << DMA_CH0_CONTROL_MSS_shift)
#define DMA_CH0_CONTROL_Rsvd_mask 		((0x7f << 25) | (0x7 << 21) | (0x1 << 17) | (0X3 << 14))

#define DMA_CH0_TX_CONTROL			0x1104
#define DMA_CH0_TX_CONTROL_TXPBL_shift		16
#define DMA_CH0_TX_CONTROL_TXPBL_mask		(0x3f << DMA_CH0_TX_CONTROL_TxPBL_shift)
#define DMA_CH0_TX_CONTROL_IPBL			(1 << 15)
#define DMA_CH0_TX_CONTROL_TSE			(1 << 12)
#define DMA_CH0_TX_CONTROL_OSP			(1 << 4)
#define DMA_CH0_TX_CONTROL_TCW_shift		1
#define DMA_CH0_TX_CONTROL_TCW_mask		(0x7 << DMA_CH0_TX_CONTROL_TCW_shift)
#define DMA_CH0_TX_CONTROL_ST			(1 << 0)
#define DMA_CH0_TX_CONTROL_Rsvd_mask		((0x7ff << 22) | (0x3 << 13) | (0x7f << 5))

#define DMA_CH0_RX_CONTROL			0x1108
#define DMA_CH0_RX_CONTROL_MAMS			(1 << 27)
#define DMA_CH0_RX_CONTROL_RXPBL_shift		16
#define DMA_CH0_RX_CONTROL_RXPBL_mask		(0x3f << DMA_CH0_RX_CONTROL_RXPBL_shift)
#define DMA_CH0_RX_CONTROL_RBSZ_shift		1
#define DMA_CH0_RX_CONTROL_RBSZ_mask		(0x3fff << DMA_CH0_RX_CONTROL_RBSZ_shift)
#define DMA_CH0_RX_CONTROL_SR			(1 << 0)
#define DMA_CH0_RX_CONTROL_Rsvd_mask		((0xf << 28) | (0x1f << 22) | (0x1 << 15))

#define DMA_CH0_TXDESC_LIST_ADDR		0x1114
#define DMA_CH0_TXDESC_LIST_ADDR32_mask		0xfffffffe

#define DMA_CH0_RXDESC_LIST_ADDR		0x111c
#define DMA_CH0_RXDESC_LIST_ADDR32_mask		0xfffffffe

#define DMA_CH0_TXDESC_TAIL_POINTER		0x1120
#define DMA_CH0_TXDESC_TAIL_POINTER32_mask	0xfffffffe

#define DMA_CH0_RXDESC_TAIL_POINTER		0x1128
#define DMA_CH0_RXDESC_TAIL_POINTER32_mask	0xfffffffe

#define DMA_CH0_TXDESC_RING_LENGTH		0x112c
#define DMA_CH0_TXDESC_RING_LENGTH_shift	0
#define DMA_CH0_TXDESC_RING_LENGTH_mask		(0x3ff << DMA_CH0_TXDESC_RING_LENGTH_shift)

#define DMA_CH0_RXDESC_RING_LENGTH		0x1130
#define DMA_CH0_RXDESC_RING_LENGTH_shift	0
#define DMA_CH0_RXDESC_RING_LENGTH_mask		(0x3ff << DMA_CH0_RXDESC_RING_LENGTH_shift)

#define DMA_CH0_STATUS				0x1160
#define DMA_CH0_STATUS_REB_shift		19
#define DMA_CH0_STATUS_REB_mask			(0x7 << DMA_CH0_STATUS_REB_shift)
#define DMA_CH0_STATUS_TEB_shift		16
#define DMA_CH0_STATUS_TEB_mask			(0x7 << DMA_CH0_STATUS_TEB_shift)
#define DMA_CH0_STATUS_NIS			(1 << 15)
#define DMA_CH0_STATUS_AIS			(1 << 14)
#define DMA_CH0_STATUS_CDE			(1 << 13)
#define DMA_CH0_STATUS_FBE			(1 << 12)
#define DMA_CH0_STATUS_ERI			(1 << 11)
#define DMA_CH0_STATUS_ETI			(1 << 10)
#define DMA_CH0_STATUS_RWT			(1 << 9)
#define DMA_CH0_STATUS_RPS			(1 << 8)
#define DMA_CH0_STATUS_RBU			(1 << 7)
#define DMA_CH0_STATUS_RI			(1 << 6)
#define DMA_CH0_STATUS_TBU			(1 << 2)
#define DMA_CH0_STATUS_TPS			(1 << 1)
#define DMA_CH0_STATUS_TI			(1 << 0)
#define DMA_CH0_STATUS_Rsvd_mask 		((0x3ff << 22) | (0x7 << 3)) /* reserved */
