/*
* drivers/mci/mci-netx4000.h
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

#ifndef __NETX4000_SDIO_H__
#define __NETX4000_SDIO_H__

/* Register Definitions */
struct netx4000_sdio_reg {
	u32 sd_cmd;           /* command type and response type */
	u32 sd_portsel;       /* port selection of the sd-card  */
	u32 sd_arg0;          /* command argument */
	u32 sd_arg1;          /* command argument */
	u32 sd_stop;          /* en-/disable block counting in case of multiple block transfer */
	u32 sd_seccnt;        /* number of blocks to transfer */
	u32 sd_rsp10;         /* response of the sd-card */
	u32 sd_rsp1;          /*  */
	u32 sd_rsp32;
	u32 sd_rsp3;
	u32 sd_rsp54;
	u32 sd_rsp5;
	u32 sd_rsp76;
	u32 sd_rsp7;
	u32 sd_info1;         /* response info */
	u32 sd_info2;         /* response info */
	u32 sd_info1_mask;    /* en-/disables sd_info1 interrupt */
	u32 sd_info2_mask;    /* en-/disables sd_info2 interrupt */
	u32 sd_clk_ctrl;      /* SDCLK configuration */
	u32 sd_size;          /* transfer data length */
	u32 sd_option;        /* sd-card access control */
        u32 reserved0;
	u32 sd_err_sts1;      /* sd-card error status register */
	u32 sd_err_sts2;      /* sd-card error status register */
	u32 sd_buf0;          /* read/write buffer */
        u32 reserved1;
	u32 sdio_mode;        /* command mode configuration */
	u32 sdio_info1;       /* interrupt flag register */
	u32 sdio_info1_mask;  /* en-/disables sdio_info1 interrupt */
        u32 reserved2[79];
	u32 cc_ext_mode;      /* dma mode configuration */
        u32 reserved3[3];
	u32 soft_rst;         /* software reset register */
	u32 version;          /* version */
	u32 host_mode;        /* host interface mode */
	u32 sdif_mode;        /* sd interface mode */
        u32 reserved4[4];
	u32 ext_swap;         /* swap control register */
	u32 sd_status;
	u32 ext_sdio;         /* en-/disables sdio interrupt */
	u32 ext_wp;           /* write protected state of the extended port1 */
	u32 ext_cd;           /* card detect of port1 */
	u32 ext_cd_dat3;      /* extended card detect */
	u32 ext_cd_mask;      /* interrupt mask register */
	u32 ext_cd_dat3_mask; /* interrupt mask register */
};

/* NETX4000 - REGDEF */

/* --------------------------------------------------------------------- */
/* Register SDIO_SD_CLK_CTRL */
/* => SD clock control register */
/* => Mode: R/W */
/* --------------------------------------------------------------------- */

#define MSK_NX4000_SDIO_SD_CLK_CTRL_DIV                  0x000000ffU
#define SRT_NX4000_SDIO_SD_CLK_CTRL_DIV                  0
#define DFLT_VAL_NX4000_SDIO_SD_CLK_CTRL_DIV             0x00000020U
#define DFLT_BF_VAL_NX4000_SDIO_SD_CLK_CTRL_DIV          0x00000020U
#define MSK_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_EN            0x00000100U
#define SRT_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_EN            8
#define DFLT_VAL_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_EN       0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_EN    0x00000000U
#define MSK_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_OFFEN         0x00000200U
#define SRT_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_OFFEN         9
#define DFLT_VAL_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_OFFEN    0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_OFFEN 0x00000000U
#define MSK_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_SEL           0x00000400U
#define SRT_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_SEL           10
#define DFLT_VAL_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_SEL      0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_SEL   0x00000000U

/* --------------------------------------------------------------------- */
/* Register SDIO_SD_OPTION */
/* => SD card access control option register */
/* => Mode: R/W */
/* --------------------------------------------------------------------- */

#define MSK_NX4000_SDIO_SD_OPTION_CTOP          0x0000000fU
#define SRT_NX4000_SDIO_SD_OPTION_CTOP          0
#define DFLT_VAL_NX4000_SDIO_SD_OPTION_CTOP     0x0000000eU
#define DFLT_BF_VAL_NX4000_SDIO_SD_OPTION_CTOP  0x0000000eU
#define MSK_NX4000_SDIO_SD_OPTION_TOP           0x000000f0U
#define SRT_NX4000_SDIO_SD_OPTION_TOP           4
#define DFLT_VAL_NX4000_SDIO_SD_OPTION_TOP      0x000000e0U
#define DFLT_BF_VAL_NX4000_SDIO_SD_OPTION_TOP   0x0000000eU
#define MSK_NX4000_SDIO_SD_OPTION_WIDTH         0x00008000U
#define SRT_NX4000_SDIO_SD_OPTION_WIDTH         15
#define DFLT_VAL_NX4000_SDIO_SD_OPTION_WIDTH    0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_OPTION_WIDTH 0x00000000U

/* --------------------------------------------------------------------- */
/* Register SDIO_SD_CMD */
/* => Command type regsiter */
/* => Mode: R/W */
/* --------------------------------------------------------------------- */

#define MSK_NX4000_SDIO_SD_CMD_CF                 0x0000003fU
#define SRT_NX4000_SDIO_SD_CMD_CF                 0
#define DFLT_VAL_NX4000_SDIO_SD_CMD_CF            0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_CMD_CF         0x00000000U
#define MSK_NX4000_SDIO_SD_CMD_C                  0x000000c0U
#define SRT_NX4000_SDIO_SD_CMD_C                  6
#define DFLT_VAL_NX4000_SDIO_SD_CMD_C             0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_CMD_C          0x00000000U
#define MSK_NX4000_SDIO_SD_CMD_MD_RSP             0x00000700U
#define SRT_NX4000_SDIO_SD_CMD_MD_RSP             8
#define DFLT_VAL_NX4000_SDIO_SD_CMD_MD_RSP        0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_CMD_MD_RSP     0x00000000U
#define MSK_NX4000_SDIO_SD_CMD_MD3                0x00000800U
#define SRT_NX4000_SDIO_SD_CMD_MD3                11
#define DFLT_VAL_NX4000_SDIO_SD_CMD_MD3           0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_CMD_MD3        0x00000000U
#define MSK_NX4000_SDIO_SD_CMD_MD4                0x00001000U
#define SRT_NX4000_SDIO_SD_CMD_MD4                12
#define DFLT_VAL_NX4000_SDIO_SD_CMD_MD4           0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_CMD_MD4        0x00000000U
#define MSK_NX4000_SDIO_SD_CMD_MD5                0x00002000U
#define SRT_NX4000_SDIO_SD_CMD_MD5                13
#define DFLT_VAL_NX4000_SDIO_SD_CMD_MD5           0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_CMD_MD5        0x00000000U
#define MSK_NX4000_SDIO_SD_CMD_MD_MLT_BLK         0x0000c000U
#define SRT_NX4000_SDIO_SD_CMD_MD_MLT_BLK         14
#define DFLT_VAL_NX4000_SDIO_SD_CMD_MD_MLT_BLK    0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_CMD_MD_MLT_BLK 0x00000000U

/* --------------------------------------------------------------------- */
/* Register SDIO_SD_INFO1 */
/* => SD card interrupt flag register 1 */
/* => Mode: R/W */
/* --------------------------------------------------------------------- */

#define MSK_NX4000_SDIO_SD_INFO1_INFO0          0x00000001U
#define SRT_NX4000_SDIO_SD_INFO1_INFO0          0
#define DFLT_VAL_NX4000_SDIO_SD_INFO1_INFO0     0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO1_INFO0  0x00000000U
#define MSK_NX4000_SDIO_SD_INFO1_INFO2          0x00000004U
#define SRT_NX4000_SDIO_SD_INFO1_INFO2          2
#define DFLT_VAL_NX4000_SDIO_SD_INFO1_INFO2     0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO1_INFO2  0x00000000U
#define MSK_NX4000_SDIO_SD_INFO1_INFO3          0x00000008U
#define SRT_NX4000_SDIO_SD_INFO1_INFO3          3
#define DFLT_VAL_NX4000_SDIO_SD_INFO1_INFO3     0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO1_INFO3  0x00000000U
#define MSK_NX4000_SDIO_SD_INFO1_INFO4          0x00000010U
#define SRT_NX4000_SDIO_SD_INFO1_INFO4          4
#define DFLT_VAL_NX4000_SDIO_SD_INFO1_INFO4     0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO1_INFO4  0x00000000U
#define MSK_NX4000_SDIO_SD_INFO1_INFO5          0x00000020U
#define SRT_NX4000_SDIO_SD_INFO1_INFO5          5
#define DFLT_VAL_NX4000_SDIO_SD_INFO1_INFO5     0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO1_INFO5  0x00000000U
#define MSK_NX4000_SDIO_SD_INFO1_INFO7          0x00000080U
#define SRT_NX4000_SDIO_SD_INFO1_INFO7          7
#define DFLT_VAL_NX4000_SDIO_SD_INFO1_INFO7     0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO1_INFO7  0x00000000U
#define MSK_NX4000_SDIO_SD_INFO1_INFO8          0x00000100U
#define SRT_NX4000_SDIO_SD_INFO1_INFO8          8
#define DFLT_VAL_NX4000_SDIO_SD_INFO1_INFO8     0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO1_INFO8  0x00000000U
#define MSK_NX4000_SDIO_SD_INFO1_INFO9          0x00000200U
#define SRT_NX4000_SDIO_SD_INFO1_INFO9          9
#define DFLT_VAL_NX4000_SDIO_SD_INFO1_INFO9     0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO1_INFO9  0x00000000U
#define MSK_NX4000_SDIO_SD_INFO1_INFO10         0x00000400U
#define SRT_NX4000_SDIO_SD_INFO1_INFO10         10
#define DFLT_VAL_NX4000_SDIO_SD_INFO1_INFO10    0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO1_INFO10 0x00000000U

/* --------------------------------------------------------------------- */
/* Register SDIO_SD_INFO2 */
/* => SD card interrupt flag register 2 */
/* => Mode: R/W */
/* --------------------------------------------------------------------- */

#define MSK_NX4000_SDIO_SD_INFO2_ERR0              0x00000001U
#define SRT_NX4000_SDIO_SD_INFO2_ERR0              0
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_ERR0         0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_ERR0      0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_ERR1              0x00000002U
#define SRT_NX4000_SDIO_SD_INFO2_ERR1              1
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_ERR1         0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_ERR1      0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_ERR2              0x00000004U
#define SRT_NX4000_SDIO_SD_INFO2_ERR2              2
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_ERR2         0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_ERR2      0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_ERR3              0x00000008U
#define SRT_NX4000_SDIO_SD_INFO2_ERR3              3
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_ERR3         0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_ERR3      0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_ERR4              0x00000010U
#define SRT_NX4000_SDIO_SD_INFO2_ERR4              4
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_ERR4         0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_ERR4      0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_ERR5              0x00000020U
#define SRT_NX4000_SDIO_SD_INFO2_ERR5              5
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_ERR5         0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_ERR5      0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_ERR6              0x00000040U
#define SRT_NX4000_SDIO_SD_INFO2_ERR6              6
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_ERR6         0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_ERR6      0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_DAT0              0x00000080U
#define SRT_NX4000_SDIO_SD_INFO2_DAT0              7
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_DAT0         0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_DAT0      0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_BRE               0x00000100U
#define SRT_NX4000_SDIO_SD_INFO2_BRE               8
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_BRE          0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_BRE       0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_BWE               0x00000200U
#define SRT_NX4000_SDIO_SD_INFO2_BWE               9
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_BWE          0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_BWE       0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_BSYNC2            0x00000400U
#define SRT_NX4000_SDIO_SD_INFO2_BSYNC2            10
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_BSYNC2       0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_BSYNC2    0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_SCLKDIVEN         0x00002000U
#define SRT_NX4000_SDIO_SD_INFO2_SCLKDIVEN         13
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_SCLKDIVEN    0x00002000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_SCLKDIVEN 0x00000001U
#define MSK_NX4000_SDIO_SD_INFO2_CBSY              0x00004000U
#define SRT_NX4000_SDIO_SD_INFO2_CBSY              14
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_CBSY         0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_CBSY      0x00000000U
#define MSK_NX4000_SDIO_SD_INFO2_ILA               0x00008000U
#define SRT_NX4000_SDIO_SD_INFO2_ILA               15
#define DFLT_VAL_NX4000_SDIO_SD_INFO2_ILA          0x00000000U
#define DFLT_BF_VAL_NX4000_SDIO_SD_INFO2_ILA       0x00000000U


#endif /* __NETX4000_SDIO_H__ */
