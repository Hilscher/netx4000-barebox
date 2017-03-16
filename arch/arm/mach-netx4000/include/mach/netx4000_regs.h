/*
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#ifndef _NETX4000_REGS_H_
#define _NETX4000_REGS_H_

#define Adr_NX4000_DDR_CTRL_CTL_00	0xF8001000U

#define NETX4000_RAP_UART_BASE(n)	(0xF8036000 + (n) * 0x1000)

#define NETX4000_AXI_RAM_SPACE_START	(0x05000000)

#define NETX4000_DDR_ADDR_SPACE_START	(0x40000000)

#define NETX4000_DDRREGS_BASE		Adr_NX4000_DDR_CTRL_CTL_00
#define NETX4000_DDRPHY_BASE		(0xF8002000)

#define NETX4000_A9_PERIPHBASE_START (0xFAF00000)
#define NETX4000_A9_SCUREGS_BASE (NETX4000_A9_PERIPHBASE_START)
#define NETX4000_A9_IRQCTRL_BASE (NETX4000_A9_PERIPHBASE_START + 0x0100)
#define NETX4000_A9_GLOBALTIMER_BASE (NETX4000_A9_PERIPHBASE_START + 0x0200)
#define NETX4000_A9_PRIVATETIMER_BASE  (NETX4000_A9_PERIPHBASE_START + 0x0600)

#define NETX4000_A9_PERIPHBASE_START  (0xFAF00000)
#define NETX4000_A9_SCUREGS_BASE (NETX4000_A9_PERIPHBASE_START)
#define NETX4000_A9_IRQCTRL_BASE (NETX4000_A9_PERIPHBASE_START + 0x0100)
#define NETX4000_A9_GLOBALTIMER_BASE (NETX4000_A9_PERIPHBASE_START + 0x0200)
#define NETX4000_A9_PRIVATETIMER_BASE  (NETX4000_A9_PERIPHBASE_START + 0x0600)
#define NETX4000_A9_PL310_BASE (NETX4000_A9_PERIPHBASE_START + 0x10000)

#define NETX4000_USB2CFG		(0xF8000010)
/* USB interface Port setting signal 0 = Port1 Function, Port2 Host; */
/*                                   1 = Port1 Host, Port2 Host */
#define NETX4000_USB2CFG_H2MODE_MASK	(0x00000002)
/* Direct power down control */
#define NETX4000_USB2CFG_DIRPD_MASK	(0x00000001)

#endif /* _NETX4000_REGS_H_ */
