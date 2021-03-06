/*
 * (C) Copyright 2013
 *
 * Tijs Maas, Metatronics, tijsxdmaas.developer@gmail.com
 * 
 * Note: This version is based on the Hitex lpc1850 board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * Board specific code for the Keil MCB1800 board
 */

#include <common.h>


#include <asm/arch/lpc18xx_gpio.h>
#include <asm/arch/lpc18xx_scu.h>
#include <asm/arch/lpc18xx_creg.h>
#include <asm/arch/lpc18xx_ccu.h>

/*
 * some #define in it that can be included directly in this file.
 * See https://github.com/openxc/nxp-cdl/tree/master/LPC18xxLib/inc for more CMSIS headers.
 */
#include <asm/arch/lpc18xx_emc.h>

/* Load Mode Register command : Bank Row Column OR Row Bank Column */
#ifdef BRC_MODEREG
/*
 * MT48LC4M32B2 SDRAM: 32-bit, 4 banks, 12 row bits, 8 column bits.
 * 1 0 010 10:(0x74): 128 Mb (4Mx32), 4 banks, row length = 12, column length = 8
 * See table 364 "Address mapping" on page 417 in the LPC43xx User Manual.
 */
#define LPC18XX_EMC_AM		((EMC_SDRAM_WIDTH_32_BITS << 7) | (EMC_SDRAM_SIZE_128_MBITS << 9) |\
							(EMC_SDRAM_MODE_BANK_ROW_COLUMN << 12) |  (EMC_SDRAM_DATA_BUS_32_BITS << 14))
#else
#define LPC18XX_EMC_AM		((EMC_SDRAM_WIDTH_32_BITS << 7) | (EMC_SDRAM_SIZE_128_MBITS << 9) |\
							(EMC_SDRAM_MODE_ROW_BANK_COLUMN << 12) |  (EMC_SDRAM_DATA_BUS_32_BITS << 14))
#endif
/*
 * Timings for 166 MHz SDRAM clock and MT48LC4M32B2 memory chip
 * EMC_CLK = 90 Mhz ^= 11.11ns
 * Calculation:
 *  value = ([x]ns / 11.11ns); round up
 */
/* Active to read/write delay (RAS latency) */
#define SDRAM_RAS		2	/* tRCD = 18ns */
/* CAS latency (CL) */
#define SDRAM_CAS		3	/* CL = 3 (102Mhz is too high for CL 2) */
/* Command delayed strategy, using EMCCLKDELAY */
#define SDRAM_RDCFG_RD	        1
/* Precharge command period (tRP) */
#define SDRAM_T_RP		2	/* 18ns */
/* Active to precharge command period (tRAS) */
#define SDRAM_T_RAS		5	/* 42ns */
/* Self-refresh exit time (tSREX) */
#define SDRAM_T_SREX    8	/* We set this to the same as tXSR, NOT IN DATASHEET */
/* Last-data-out to active command time (tAPR) */
#define SDRAM_T_APR		1	/* NOT IN DATASHEET */
/* Data-in to active command (tDAL) */
#define SDRAM_T_DAL		4	/* 30ns (=5*6.02ns) */
/* Write recovery time (tWR) */
#define SDRAM_T_WR		2	/* (1 CLK + 6ns) */
/* Active to active command period (tRC) */
#define SDRAM_T_RC		7	/* 60ns */
/* Auto-refresh period and auto-refresh to active command period (tRFC) */
#define SDRAM_T_RFC		7	/* 60ns */
/* Exit self-refresh to active command time (tXSR) */
#define SDRAM_T_XSR		8	/* 70ns */
/* Active bank A to active bank B latency (tRRD) */
#define SDRAM_T_RRD		2	/* 12ns */
/* Load mode register to active command time (tMRD) */
#define SDRAM_T_MRD		2	/* 12ns (=2*6.02ns) */

/*
 * Refresh timer.
 * Indicates the multiple of 16 EMC_CCLKs between SDRAM refresh cycles.
 * 
 * see MT48LC4M32B2 datasheet
 * 	front page: 64ms, 4,096-cycle refresh (15.6μs/row)
 *
 * see UM10430 (cortex M3 user manual)
 * 	Table 349. Dynamic Memory Refresh Timer register
 *
 * if EMC clock = M4_CLK/2 = 90 Mhz:
 *	CCLK = 1[s]/0.090[Hz*10^-3] = 11.11ns per clocktick
 *
 * 64000000[64ms] / 4096[rows] / (11.11[ns] * 16); round down = 87
 */
#define SDRAM_REFRESH		99
/* Only for initialization */
#define SDRAM_REFRESH_FAST	1

/*
 * EMC registers
 */
/*
 * EMC Control register
 */
#define LPC_EMC_CTRL_EN_MSK		(1 << 0)

/*
 * Dynamic Memory Control register
 */
/* Dynamic memory clock enable (CE) */
#define LPC_EMC_DYCTRL_CE_MSK		(1 << 0)
/* Dynamic memory clock control (CS) */
#define LPC_EMC_DYCTRL_CS_MSK		(1 << 1)
/* SDRAM initialization (I) */
#define LPC_EMC_DYCTRL_I_BITS		7
#define LPC_EMC_DYCTRL_I_NORMAL		0
#define LPC_EMC_DYCTRL_I_MODE		1
#define LPC_EMC_DYCTRL_I_PALL		2	/* precharge all */
#define LPC_EMC_DYCTRL_I_NOP		3	/* no operation */

/*
 * Dynamic Memory Read Configuration register:
 *     Read data strategy (RD)
 */
#define LPC_EMC_DYRDCFG_RD_BITS	0

/*
 * The SDRAM chip (MT48LC4M32B2) mode register.
 * See MT48LC4M32B2 datasheet, page 13.
 */
#define SDRAM_MODEREG_BL_BITS		0	/* Burst Length */
#define SDRAM_MODEREG_CAS_BITS		4	/* CAS Latency */

/*
 * See MT48LC4M32B2 mode register (MT48LC4M32B2 datasheet, page 13).
 * CAS3, Burst Length = 4.
 */
#define SDRAM_MODEREG_BL		2	/* Burst Length code */
#define SDRAM_MODEREG_CAS		SDRAM_CAS	/* CAS Latency */

#define SDRAM_MODEREG_VALUE \
  ((SDRAM_MODEREG_BL << SDRAM_MODEREG_BL_BITS) |	\
   (SDRAM_MODEREG_CAS << SDRAM_MODEREG_CAS_BITS))

/*
 * SDRAM chip-specific options
 */
/*
 * Offset of the 12 least-significant bits of mode register (A0..A11)
 * in addresses on the AHB bus.
 *
 * In the high-performance mode the shift should be the following:
 * 11 = 8 (column bits) + 2 (bank select bits) + 1 (16 bits)
 *    1. MT48LC4M32B2 SDRAM has 256 columns, therefore 8 bits are used
 *         for the column number.
 *    2. Bank select field has 2 bits (4 banks).
 *    3. `2` is log2(32/8), because the SDRAM chip is 32-bit, and its
 *        internal addresses do not have 1 least-significant bit of
 *        the AHB bus addresses.
 *
 * In the low-power mode this shift will be different.
 */
#define LPC18XX_EMC_MODEREG_ADDR_SHIFT	12

/*
 * Dynamic Memory registers (per chip)
 */
/*
 * Dynamic Memory Configuration register
 */
/* Address mapping */
#define LPC_EMC_DYCFG_AM_BITS		7
/* Buffer enable */
#define LPC_EMC_DYCFG_B_MSK		(1 << 19)
/*
 * Dynamic Memory RAS & CAS Delay register
 */
/* RAS latency */
#define LPC_EMC_DYRASCAS_RAS_BITS	0
/* CAS latency */
#define LPC_EMC_DYRASCAS_CAS_BITS	8

/*
 * EMC per-chip registers for DRAM.
 *
 * This structure must be 0x20 bytes in size
 * (for `struct lpc_emc_regs` to be correct.)
 */
struct lpc_emc_dy_regs {
	u32 cfg;	/* Dynamic Memory Configuration register */
	u32 rascas;	/* Dynamic Memory RAS & CAS Delay registers */
	u32 rsv0[6];
};

/*
 * EMC controls for Static Memory CS. Each block occupies 0x20 bytes.
 */
struct lpc_emc_st_regs {
	u32 cfg;	/* Static Memory Configuration register */
	u32 we;		/* CS to WE delay register */
	u32 oe;		/* CS to OE delay register */
	u32 rd;		/* CS to Read delay register */
	u32 page;	/* async page mode access delay */
	u32 wr;		/* CS to Write delay register */
	u32 ta;		/* number of turnaround cycles */
	u32 rsv0[1];
};

/*
 * EMC (External Memory Controller) register map
 */
struct lpc_emc_regs {
	/* 0x000 */
	u32 emcctrl;	/* EMC Control register */
	u32 emcsts;	/* EMC Status register */
	u32 emccfg;	/* EMC Configuration register */
	u32 rsv0[5];

	/* 0x020 */
	u32 dy_ctrl;	/* Dynamic Memory Control register */
	u32 dy_rfsh;	/* Dynamic Memory Refresh Timer register */
	u32 dy_rdcfg;	/* Dynamic Memory Read Configuration register */
	u32 rsv1;

	/* 0x030 */
	u32 dy_trp;	/* Dynamic Memory Precharge Command Period register */
	u32 dy_tras;	/* Dynamic Memory Active to Precharge Command
				Period register */
	u32 dy_srex;	/* Dynamic Memory Self-refresh Exit Time register */
	u32 dy_apr;	/* Dynamic Memory Last Data Out to Active
				Time register */
	u32 dy_dal;	/* Dynamic Memory Data-in to Active Command
				Time register */
	u32 dy_wr;	/* Dynamic Memory Write Recovery Time register */
	u32 dy_rc;	/* Dynamic Memory Active to Active Command
				Period register */
	u32 dy_rfc;	/* Dynamic Memory Auto-refresh Period register */
	u32 dy_xsr;	/* Dynamic Memory Exit Self-refresh register */
	u32 dy_rrd;	/* Dynamic Memory Active Bank A to
				Active Bank B Time register */
	u32 dy_mrd;	/* Dynamic Memory Load Mode register to
				Active Command Time */
	/* 0x05C */
	u32 rsv2[41];

	/* 0x100 */
	struct lpc_emc_dy_regs dy[4];	/* 4 DRAM chips are possible */
	u32 rsv3[32];
	/* 0x200 */
	struct lpc_emc_st_regs st[4];	/* 4 static memory devices */
};

#define LPC18XX_EMC_BASE	0x40005000
#define LPC_EMC				((volatile struct lpc_emc_regs *) LPC18XX_EMC_BASE)

DECLARE_GLOBAL_DATA_PTR;

/*
 * Pin configuration table for MCB1800.
 *
 * This table does not list all MCU pins that will be configured. See also
 * the code in `iomux_init()`.
 */
static const struct lpc18xx_pin_config mcb1800_iomux[] = {
	/*
	 * Pin configuration for UART
	 */

	{{CONFIG_LPC18XX_UART_TX_IO_GROUP, CONFIG_LPC18XX_UART_TX_IO_PIN},
		LPC18XX_IOMUX_CONFIG(1, 0, 1, 0, 0, 0)},
	{{CONFIG_LPC18XX_UART_RX_IO_GROUP, CONFIG_LPC18XX_UART_RX_IO_PIN},
		LPC18XX_IOMUX_CONFIG(1, 0, 1, 0, 1, 0)},

#ifdef CONFIG_LPC18XX_ETH
	/*
	 * Pin configuration for Ethernet (MII + MDIO) (untested)
	 */

	/* PC.1 = ENET_MDC */
	{{0xC,  1}, LPC18XX_IOMUX_CONFIG(3, 0, 1, 0, 1, 1)},
	/* P1.17 = ENET_MDIO (high-drive pin) */
	{{0x1, 17}, LPC18XX_IOMUX_CONFIG(3, 0, 1, 0, 1, 1)},
	/* P1.18 = ENET_TXD0 */
	{{0x1, 18}, LPC18XX_IOMUX_CONFIG(3, 0, 1, 0, 1, 1)},
	/* P1.20 = ENET_TXD1 */
	{{0x1, 20}, LPC18XX_IOMUX_CONFIG(3, 0, 1, 0, 1, 1)},
	/* P0.1 = ENET_TX_EN */
	{{0x0,  1}, LPC18XX_IOMUX_CONFIG(6, 0, 1, 0, 1, 1)},
	/* P1.15 = ENET_RXD0 */
	{{0x1, 15}, LPC18XX_IOMUX_CONFIG(3, 0, 1, 0, 1, 1)},
	/* P0.0 = ENET_RXD1 */
	{{0x0,  0}, LPC18XX_IOMUX_CONFIG(2, 0, 1, 0, 1, 1)},
	/* P1.16 = ENET_RXDV */
	{{0x1, 16}, LPC18XX_IOMUX_CONFIG(7, 0, 1, 0, 1, 1)},
#endif /* CONFIG_LPC18XX_ETH */

#if defined(CONFIG_NR_DRAM_BANKS) || defined(CONFIG_SYS_FLASH_CS)
	/*
	 * EMC pins used for both the SDRAM and the NOR flash memory chips
	 */

	/* P1.6 = WE# - SDRAM,NOR */
	{{0x1, 6}, LPC18XX_IOMUX_EMC_CONFIG(3)},

	/* P2.11 = A2 - SDRAM,NOR */
	{{0x2, 11}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P2.12 = A3 - SDRAM,NOR */
	{{0x2, 12}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P2.13 = A4 - SDRAM,NOR */
	{{0x2, 13}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P1.0 = A5 - SDRAM,NOR */
	{{0x1, 0}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* P1.1 = A6 - SDRAM,NOR */
	{{0x1, 1}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* P1.2 = A7 - SDRAM,NOR */
	{{0x1, 2}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* P2.8 = A8 - SDRAM,NOR */
	{{0x2, 8}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P2.7 = A9 - SDRAM,NOR */
	{{0x2, 7}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P2.6 = A10 - SDRAM,NOR */
	{{0x2, 6}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* P2.2 = A11 - SDRAM,NOR */
	{{0x2, 2}, LPC18XX_IOMUX_EMC_CONFIG(2)},

	// BA0 is used for SDRAM/UART0.
#ifdef USE_UART0_PF
	/* P2.0 = A13/BA0 - SDRAM,NOR */
	{{0x2, 0}, LPC18XX_IOMUX_EMC_CONFIG(2)},
#else
#error Conflicting pin P2.0 used for UART and SDRAM/NOR!
#endif
	/* P6.8 = A14/BA1 - SDRAM,NOR */
	{{0x6, 8}, LPC18XX_IOMUX_EMC_CONFIG(1)},

	/* P1.7 = D0 - SDRAM,NOR */
	{{0x1, 7}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P1.8 = D1 - SDRAM,NOR */
	{{0x1, 8}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P1.9 = D2 - SDRAM,NOR */
	{{0x1, 9}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P1.10 = D3 - SDRAM,NOR */
	{{0x1, 10}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P1.11 = D4 - SDRAM,NOR */
	{{0x1, 11}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P1.12 = D5 - SDRAM,NOR */
	{{0x1, 12}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P1.13 = D6 - SDRAM,NOR */
	{{0x1, 13}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P1.14 = D7 - SDRAM,NOR */
	{{0x1, 14}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P5.4 = D8 - SDRAM,NOR */
	{{0x5, 4}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* P5.5 = D9 - SDRAM,NOR */
	{{0x5, 5}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* P5.6 = D10 - SDRAM,NOR */
	{{0x5, 6}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* P5.7 = D11 - SDRAM,NOR */
	{{0x5, 7}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* P5.0 = D12 - SDRAM,NOR */
	{{0x5, 0}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* P5.1 = D13 - SDRAM,NOR */
	{{0x5, 1}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* P5.2 = D14 - SDRAM,NOR */
	{{0x5, 2}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* P5.3 = D15 - SDRAM,NOR */
	{{0x5, 3}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PD.2 = D16 - SDRAM */
	{{0xD, 2}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PD.3 = D17 - SDRAM */
	{{0xD, 3}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PD.4 = D18 - SDRAM */
	{{0xD, 4}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PD.5 = D19 - SDRAM */
	{{0xD, 5}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PD.6 = D20 - SDRAM */
	{{0xD, 6}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PD.7 = D21 - SDRAM */
	{{0xD, 7}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PD.8 = D22 - SDRAM */
	{{0xD, 8}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PD.9 = D23 - SDRAM */
	{{0xD, 9}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PE.5 = D24 - SDRAM */
	{{0xE, 5}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PE.6 = D25 - SDRAM */
	{{0xE, 6}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PE.7 = D26 - SDRAM */
	{{0xE, 7}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PE.8 = D27 - SDRAM */
	{{0xE, 8}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PE.9 = D28 - SDRAM */
	{{0xE, 9}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PE.10 = D29 - SDRAM */
	{{0xE, 10}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PE.11 = D30 - SDRAM */
	{{0xE, 11}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PE.12 = D31 - SDRAM */
	{{0xE, 12}, LPC18XX_IOMUX_EMC_CONFIG(3)},
#endif /* CONFIG_NR_DRAM_BANKS || CONFIG_SYS_FLASH_CS */

#ifdef CONFIG_NR_DRAM_BANKS
	/*
	 * Configuration for EMC pins used only for SDRAM
	 */

	/*
	 * To use 16-bit wide and 32-bit wide SDRAM interfaces, select
	 * the EMC_CLK function and enable the input buffer (EZI = 1)
	 * in all four SFSCLKn registers in the SCU.
	 */
	/* Imaginary P-0x18.0 = CLK (CLK0) - SDRAM */
	{{0x18, 0}, LPC18XX_IOMUX_EMC_CONFIG(0)},
	/* Imaginary P-0x18.1 = CLK1 - SDRAM */
	{{0x18, 1}, LPC18XX_IOMUX_EMC_CONFIG(0)},
	/* Imaginary P-0x18.2 = CLK2 - SDRAM */
	{{0x18, 2}, LPC18XX_IOMUX_EMC_CONFIG(0)},
	/* Imaginary P-0x18.3 = CLK3 - SDRAM */
	{{0x18, 3}, LPC18XX_IOMUX_EMC_CONFIG(0)},

	/* P2.9 = A0 - SDRAM,NOR */
	{{0x2, 9}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P2.10 = A1 - SDRAM,NOR */
	{{0x2, 10}, LPC18XX_IOMUX_EMC_CONFIG(3)},

	/* P6.4 = CAS# - SDRAM */
	{{0x6, 4}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P6.5 = RAS# - SDRAM */
	{{0x6, 5}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P6.9 = CS# (DYCS0) - SDRAM */
	{{0x6, 9}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P6.11 = CKE0 - SDRAM */
	{{0x6, 11}, LPC18XX_IOMUX_EMC_CONFIG(3)},

	/* 32 bits bus SDRAM on Keil Board */
	/* P6.12 = DQM0 - SDRAM */
	{{0x6, 12}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P6.10 = DQM1 - SDRAM */
	{{0x6, 10}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PD.0 = DQM2 - SDRAM */
	{{0xD, 0}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PE.13 = DQM3 - SDRAM */
	{{0xE, 13}, LPC18XX_IOMUX_EMC_CONFIG(3)},


#endif /* CONFIG_NR_DRAM_BANKS */

#ifdef CONFIG_SYS_FLASH_CS
	/*
	 * Configuration for EMC pins used only for NOR flash
	 */

	/* P1.3 = OE# - NOR */
	{{0x1, 3}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P1.5 = CS0# - NOR */
	{{0x1, 5}, LPC18XX_IOMUX_EMC_CONFIG(3)},

	/* P1.4 = BLS0# - NOR */
	{{0x1, 4}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* P6.6 = BLS1# - NOR */
	{{0x6, 6}, LPC18XX_IOMUX_EMC_CONFIG(1)},
	/* PD.13 = BLS2# - NOR */
	{{0xD, 13}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PD.10 = BLS3# - NOR */
	{{0xD, 10}, LPC18XX_IOMUX_EMC_CONFIG(2)},

#ifdef USE_UART0_PF
	/* P2.1 = A12 - NOR */
	{{0x2, 1}, LPC18XX_IOMUX_EMC_CONFIG(2)},
#else
#error Conflicting pin P2.1 used for UART and NOR!
#endif
	/* P6.7 = A15 - NOR */
	{{0x6, 7}, LPC18XX_IOMUX_EMC_CONFIG(1)},
	/* PD.16 = A16 - NOR */
	{{0xD, 16}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PD.15 = A17 - NOR */
	{{0xD, 15}, LPC18XX_IOMUX_EMC_CONFIG(2)},
	/* PE.0 = A18 - NOR */
	{{0xE, 0}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PE.1 = A19 - NOR */
	{{0xE, 1}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PE.2 = A20 - NOR */
	{{0xE, 2}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PE.3 = A21 - NOR */
	{{0xE, 3}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PE.4 = A22 - NOR */
	{{0xE, 4}, LPC18XX_IOMUX_EMC_CONFIG(3)},
	/* PA.4 = A23 - NOR */
	{{0xA, 4}, LPC18XX_IOMUX_EMC_CONFIG(3)},
#endif /* CONFIG_SYS_FLASH_CS */

#ifdef CONFIG_LPC18XX_GPIO
	/*
	 * Configuration for GPIO LEDs
	 */

	/* GPIO 4[12] */
	{{0x9, 0}, LPC18XX_IOMUX_GPIO_IN(0)},
	/* GPIO 4[13] */
	{{0x9, 1}, LPC18XX_IOMUX_GPIO_IN(0)},
	/* GPIO 4[14] */
	{{0x9, 2}, LPC18XX_IOMUX_GPIO_IN(0)},
	/* GPIO 6[24] */
	{{0xD, 10}, LPC18XX_IOMUX_GPIO_IN(4)},
	/* GPIO 6[25] */
	{{0xD, 11}, LPC18XX_IOMUX_GPIO_IN(4)},
	/* GPIO 6[26] */
	{{0xD, 12}, LPC18XX_IOMUX_GPIO_IN(4)},
	/* GPIO 6[27] */
	{{0xD, 13}, LPC18XX_IOMUX_GPIO_IN(4)},
	/* GPIO 6[28] */
	{{0xD, 14}, LPC18XX_IOMUX_GPIO_IN(4)},
#endif /* CONFIG_LPC18XX_GPIO */
};


/*
 * Configure all necessary MCU pins
 */
static void iomux_init(void)
{
	/*
	 * Configure GPIO pins using the `mcb1800_iomux[]` table
	 */
	while(lpc18xx_pin_config_table(
		mcb1800_iomux, ARRAY_SIZE(mcb1800_iomux)) != 0);

	/*
	 * Turn off all leds
	 */
	int i;
	for(i = 12; i <= 14; i++) {
		struct lpc18xx_iomux_dsc pin = {4, i};
		lpc_gpio_dir(pin, 1);
		lpc_gpio_clear(pin);
	}
	for(i = 24; i <= 28; i++) {
		struct lpc18xx_iomux_dsc pin = {6, i};
		lpc_gpio_dir(pin, 1);
		lpc_gpio_clear(pin);
	}
}

#define DELAYCYCLES(ns) (ns / ((1.0 / __EMCHZ) * 1E9))
#define DELAYCYCLES_RAM(ns) (ns / ((1.0 / __EMCHZ_RAM) * 1E9))

/*
 * Early hardware init.
 */
int board_init(void)
{
	volatile struct lpc_emc_st_regs *st;

	/*
	 * Set SDRAM clock output delay to ~3.5ns (0x7777) // from lpc1850 u-boot, calculation is unknown.
	 * the SDRAM chip does not work otherwise.
	 * See LPC18xx user manual:
	 * 	Table 182. EMC clock delay register (EMCDELAYCLK, address 0x4008 6D00) bit description
	 */
	LPC18XX_SCU->emcdelayclk = ((CLK0_DELAY) |  (CLK0_DELAY << 4) |  (CLK0_DELAY << 8) |  (CLK0_DELAY << 12));


	/*
	 * Enable External memory controller (0x00000001)
	 */
	LPC_EMC->emcctrl = LPC_EMC_CTRL_EN_MSK;
	/*
	 * Little-endian mode
	 */
	LPC_EMC->emccfg = 0;
	/*
	 * Configure MCU pins
	 */
	iomux_init();

	/*
	 * Configure Flash
	*/
#ifdef CONFIG_SYS_FLASH_CS
	/* Set timing for flash */
	st = &LPC_EMC->st[CONFIG_SYS_FLASH_CS];
	st->cfg = CONFIG_SYS_FLASH_CFG;
	st->we = CONFIG_SYS_FLASH_WE;
	st->oe = CONFIG_SYS_FLASH_OE;
	st->rd = CONFIG_SYS_FLASH_RD;
	st->page  = CONFIG_SYS_FLASH_PAGE;
	st->wr = CONFIG_SYS_FLASH_WR;
	st->ta = CONFIG_SYS_FLASH_TA;
#endif

	return 0;
}

/*
 * Dump pertinent info to the console.
 */
int checkboard(void)
{
	printf("Board: Keil MCB1800 rev %s\n",
		CONFIG_SYS_BOARD_REV_STR);

	return 0;
}

/*
 * Configure board specific parts.
 */
#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
	return 0;
}
#endif /* CONFIG_MISC_INIT_R */

#define mdelay(n) ({unsigned long msec=(n); while (msec--) udelay(1000);})

/*
 * Setup external RAM.
 * See Initialization on MT48LC4M32B2 datasheet (page 11).
 */
int dram_init(void)
{
	volatile struct lpc_emc_dy_regs *dy;
	u32 tmp32;

#ifdef CONFIG_LPC18XX_EMC_HALFCPU
	/*
	 * EMC_CLK = 90 Mhz
	 */
	LPC18XX_CCU1->clk_m4_emcdiv_cfg |=
		LPC18XX_CCU1_CLK_RUN_MSK | LPC18XX_CCU1_CLK_EMCDIV_CFG_DIV2; // 65537
	LPC18XX_CREG->creg6 |= LPC18XX_CREG_CREG6_EMCCLKSEL_MSK; // 65536
	LPC18XX_CCU1->clk_m4_emc_cfg |= LPC18XX_CCU1_CLK_RUN_MSK; // 1
#else
#error EMC clock set to M4_CLK/1 is not supported
#endif

	dy = &LPC_EMC->dy[CONFIG_SYS_RAM_CS];

   /* Initialize EMC to interface with SDRAM */
	LPC_EMC->emcctrl = 0x00000001;   /* Enable the external memory controller */
	LPC_EMC->emccfg = 0;

	/*
	 * Address mapping
	 */
	dy->cfg = LPC18XX_EMC_AM;

	/*
	 * Configure DRAM timing
	 */
	dy->rascas =
		(SDRAM_RAS << LPC_EMC_DYRASCAS_RAS_BITS) |
		(SDRAM_CAS << LPC_EMC_DYRASCAS_CAS_BITS);
	LPC_EMC->dy_rdcfg =
		(SDRAM_RDCFG_RD << LPC_EMC_DYRDCFG_RD_BITS);

	LPC_EMC->dy_trp  = SDRAM_T_RP;
	LPC_EMC->dy_tras = SDRAM_T_RAS;
	LPC_EMC->dy_srex = SDRAM_T_SREX;
	LPC_EMC->dy_apr  = SDRAM_T_APR;
	LPC_EMC->dy_dal  = SDRAM_T_DAL;
	LPC_EMC->dy_wr   = SDRAM_T_WR;
	LPC_EMC->dy_rc   = SDRAM_T_RC;
	LPC_EMC->dy_rfc  = SDRAM_T_RFC;
	LPC_EMC->dy_xsr  = SDRAM_T_XSR;
	LPC_EMC->dy_rrd  = SDRAM_T_RRD;
	LPC_EMC->dy_mrd  = SDRAM_T_MRD;
	mdelay(100);

	/*
	 * Issue SDRAM NOP (no operation) command
	 */
	LPC_EMC->dy_ctrl =
		LPC_EMC_DYCTRL_CE_MSK | LPC_EMC_DYCTRL_CS_MSK |
		(LPC_EMC_DYCTRL_I_NOP << LPC_EMC_DYCTRL_I_BITS);
	mdelay(100);

	/*
	 * Pre-charge all with fast refresh
	 */
	LPC_EMC->dy_ctrl =
		LPC_EMC_DYCTRL_CE_MSK | LPC_EMC_DYCTRL_CS_MSK |
		(LPC_EMC_DYCTRL_I_PALL << LPC_EMC_DYCTRL_I_BITS);
	LPC_EMC->dy_rfsh = SDRAM_REFRESH_FAST;

	mdelay(1);

	/*
	 * Set refresh period
	 */
	LPC_EMC->dy_rfsh = SDRAM_REFRESH;

	mdelay(100);

	/*
	 * Load mode register
	 */
	LPC_EMC->dy_ctrl =
		LPC_EMC_DYCTRL_CE_MSK | LPC_EMC_DYCTRL_CS_MSK |
		(LPC_EMC_DYCTRL_I_MODE << LPC_EMC_DYCTRL_I_BITS);

	tmp32 = *((volatile u32 *)(CONFIG_SYS_RAM_BASE |
			(SDRAM_MODEREG_VALUE << LPC18XX_EMC_MODEREG_ADDR_SHIFT)));
	udelay(100);

	/*
	 * Normal mode
	 */
	LPC_EMC->dy_ctrl =
		(LPC_EMC_DYCTRL_I_NORMAL << LPC_EMC_DYCTRL_I_BITS);

	/*
	 * Enable DRAM buffer
	 */
	dy->cfg |= (LPC18XX_EMC_AM << LPC_EMC_DYCFG_AM_BITS) | LPC_EMC_DYCFG_B_MSK;

	/*
	 * Fill in global info with description of DRAM configuration
	 */
	gd->bd->bi_dram[0].start = CONFIG_SYS_RAM_BASE;
	gd->bd->bi_dram[0].size  = CONFIG_SYS_RAM_SIZE;

	return 0;
}


#ifdef CONFIG_LPC18XX_ETH
/*
* Register ethernet driver
*/
int board_eth_init(bd_t *bis)
{
        return lpc18xx_eth_driver_init(bis);
}
#endif

#ifdef CONFIG_FLASH_CFI_LEGACY
ulong board_flash_get_legacy (ulong base, int banknum, flash_info_t *info)
{
        if (banknum == 0) {        /* non-CFI flash */
                info->portwidth = FLASH_CFI_32BIT;
                info->chipwidth = FLASH_CFI_BY32;
                info->interface = FLASH_CFI_X8X16;
                return 1;
        } else
                return 0;
}
#endif
