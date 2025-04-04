#ifndef H3_H
#define H3_H

#include <stdint.h>
#include "h3_opi_one.h"

# define _CAST(x)	(x)

#define MEGABYTE				0x100000

#define H3_F_24M				24000000
#define H3_LOSC					32768

#define H3_MEM_DRAM_START		0x40000000
#define H3_MEM_COHERENT_REGION	(H3_MEM_DRAM_START + 0x400000) // Defined in linker script. 1M region = 1 Page
#define H3_MEM_COHERENT_SIZE	(1 * MEGABYTE)
#define H3_MEM_BROM_START		0xFFFF0000

# define H3_MEM_FB_SIZE		(32 * MEGABYTE)

#define H3_SRAM_A1_BASE			0x00000000
#define H3_SRAM_A1_SIZE	(64 * 1024)
#define H3_SRAM_A2_BASE			0x00044000
#define H3_SRAM_A2_SIZE	(32 * 1024)
#define H3_DE_BASE				0x01000000
#define H3_SYSTEM_BASE			0x01C00000
#define H3_DMA_BASE				0x01C02000
#define H3_LCD_BASE				0x01C0C000
#define H3_SD_MMC_BASE			0x01C0F000
#define H3_SID_BASE				0x01C14000
#define H3_SPINLOCK_BASE      	0x01C18000
#define H3_CCU_BASE				0x01C20000
#define H3_PIO_BASE				0x01C20800
#define H3_TIMER_BASE			0x01C20C00
#define H3_AC_BASE				0x01C22C00
#define H3_THS_BASE				0x01C25000
#define H3_UART_BASE			0x01C28000
#define H3_TWI_BASE				0x01C2AC00
#define H3_EMAC_BASE			0x01C30000
#define H3_GPU_BASE				0x01C40000
#define H3_HS_TIMER_BASE		0x01C60000
#define H3_SPI_BASE				0x01C68000
#define H3_GIC_BASE 			0x01C80000	///< GIC-400 v2
#define H3_HDMI_BASE			0x01EE0000
#define H3_RTC_BASE				0x01F00000
#define H3_PRCM_BASE			0x01F01400
#define H3_CPUCFG_BASE			0x01F01C00
#define H3_PIO_PORTL_BASE		0x01F02C00

#define H3_LCD0_BASE			(H3_LCD_BASE + (0 * 0x1000))
#define H3_LCD1_BASE			(H3_LCD_BASE + (1 * 0x1000))

#define H3_DMA_CHL0_BASE		(H3_DMA_BASE + 0x100 + (0 * 0x40))
#define H3_DMA_CHL1_BASE		(H3_DMA_BASE + 0x100 + (1 * 0x40))
#define H3_DMA_CHL2_BASE		(H3_DMA_BASE + 0x100 + (2 * 0x40))
#define H3_DMA_CHL3_BASE		(H3_DMA_BASE + 0x100 + (3 * 0x40))
#define H3_DMA_CHL4_BASE		(H3_DMA_BASE + 0x100 + (4 * 0x40))
#define H3_DMA_CHL5_BASE		(H3_DMA_BASE + 0x100 + (5 * 0x40))
#define H3_DMA_CHL6_BASE		(H3_DMA_BASE + 0x100 + (6 * 0x40))
#define H3_DMA_CHL7_BASE		(H3_DMA_BASE + 0x100 + (7 * 0x40))

#define H3_SD_MMC0_BASE			(H3_SD_MMC_BASE + (0 * 0x1000))
#define H3_SD_MMC1_BASE			(H3_SD_MMC_BASE + (1 * 0x1000))
#define H3_SD_MMC2_BASE			(H3_SD_MMC_BASE + (2 * 0x1000))

#define H3_PIO_PORTA_BASE		(H3_PIO_BASE + (0 * 0x24))
#define H3_PIO_PORTB_BASE		(H3_PIO_BASE + (1 * 0x24))
#define H3_PIO_PORTC_BASE		(H3_PIO_BASE + (2 * 0x24))
#define H3_PIO_PORTD_BASE		(H3_PIO_BASE + (3 * 0x24))
#define H3_PIO_PORTE_BASE		(H3_PIO_BASE + (4 * 0x24))
#define H3_PIO_PORTF_BASE		(H3_PIO_BASE + (5 * 0x24))
#define H3_PIO_PORTG_BASE		(H3_PIO_BASE + (6 * 0x24))
#define H3_PIO_PA_INT_BASE		(H3_PIO_BASE + 0x200 + (0 * 0x20))
#define H3_PIO_PG_INT_BASE		(H3_PIO_BASE + 0x200 + (1 * 0x20))

#define H3_UART0_BASE			(H3_UART_BASE + (0 * 0x400))
#define H3_UART1_BASE			(H3_UART_BASE + (1 * 0x400))
#define H3_UART2_BASE			(H3_UART_BASE + (2 * 0x400))
#define H3_UART3_BASE			(H3_UART_BASE + (3 * 0x400))

#define H3_TWI0_BASE			(H3_TWI_BASE + (0 * 0x400))
#define H3_TWI1_BASE			(H3_TWI_BASE + (1 * 0x400))
#define H3_TWI2_BASE			(H3_TWI_BASE + (2 * 0x400))

#define H3_SPI0_BASE			(H3_SPI_BASE + (0 * 0x1000))
#define H3_SPI1_BASE			(H3_SPI_BASE + (1 * 0x1000))

#define EXT_I2C_BASE            H3_TWI0_BASE
#define EXT_SPI_BASE            H3_SPI0_BASE

typedef enum T_H3_Pn {
    H3_GPIO_PORTA = 0,
    H3_GPIO_PORTB,
    H3_GPIO_PORTC,
    H3_GPIO_PORTD,
    H3_GPIO_PORTE,
    H3_GPIO_PORTF,
    H3_GPIO_PORTG,
} H3_Pn_TypeDef;

///< The H3 defines 157 interrupts (0-156)
#define H3_IRQn	157

typedef enum T_H3_IRQn {
    H3_PPI0_IRQn = 16,
    H3_PPI1_IRQn = 17,
    H3_PPI2_IRQn = 18,
    H3_PPI3_IRQn = 19,
    H3_PPI4_IRQn = 20,
    H3_PPI5_IRQn = 21,
    H3_PPI6_IRQn = 22,
    H3_PPI7_IRQn = 23,
    H3_PPI8_IRQn = 24,
    H3_PPI9_IRQn = 25,
    H3_PPI10_IRQn = 26,
    H3_PPI11_IRQn = 27, ///< Generic Timer - virtual timer
    H3_PPI12_IRQn = 28,
    H3_PPI13_IRQn = 29,	///< Generic Timer - Secure physical timer
    H3_PPI14_IRQn = 30,	///< Generic Timer - Non-secure physical timer
    H3_PPI15_IRQn = 31,
    H3_UART0_IRQn = 32,
    H3_UART1_IRQn = 33,
    H3_UART2_IRQn = 34,
    H3_UART3_IRQn = 35,
    H3_PA_EINT_IRQn = 43,
    H3_PG_EINT_IRQn = 49,
    H3_TIMER0_IRQn = 50,
    H3_TIMER1_IRQn = 51,
    H3_AUDIO_CODEC_IRQn = 61,
    H3_DMA_IRQn = 82,
    H3_HSTIMER_IRQn = 83
} H3_IRQn_TypeDef;

# define	__I		volatile		///< defines 'read only' permissions

#define		__O		volatile		///< defines 'write only' permissions
#define		__IO	volatile		///< defines 'read / write' permissions

typedef struct T_H3_SYSTEM {
    __I  uint32_t RES0[9];
    __I  uint32_t VER;				///< 0x24 Version Register
    __I  uint32_t RES1[2];
    __IO uint32_t EMAC_CLK;			///< 0x30 EMAC-EPHY Clock register
} H3_SYSTEM_TypeDef;

typedef struct T_H3_DMA {
    __IO uint32_t IRQ_EN0;			///< 0x00
    __IO uint32_t IRQ_EN1;			///< 0x04
    __I  uint32_t RES0[2];     		///< 0x08,0x0C
    __IO uint32_t IRQ_PEND0;		///< 0x10
    __IO uint32_t IRQ_PEND1;		///< 0x14
    __I  uint32_t RES1[2];     		///< 0x18,0x1C
    __IO uint32_t SEC;				///< 0x20
    __I  uint32_t RES2; 	   		///< 0x24
    __IO uint32_t AUTO_GATE;		///< 0x28
    __I  uint32_t RES3; 	   		///< 0x2C
    __IO uint32_t STA;				///< 0x30
} H3_DMA_TypeDef;

typedef struct T_H3_DMA_CHL {
    __IO uint32_t EN;				///< 0x00
    __IO uint32_t PAU;				///< 0x04
    __IO uint32_t DESC_ADDR;		///< 0x08
    __I  uint32_t CFG;				///< 0x0C
    __I  uint32_t CUR_SRC;			///< 0x10
    __I  uint32_t CUR_DST;			///< 0x14
    __I  uint32_t BCNT_LEFT;		///< 0x18
    __I  uint32_t PARA;				///< 0x1C
    __I  uint32_t RES0[3];     		///< 0x20,0x24,0x28
    __I  uint32_t FDESC_ADDR;		///< 0x2C
    __I  uint32_t PKG_NUM;			///< 0x30

} H3_DMA_CHL_TypeDef;

typedef struct T_H3_TCON {
    __IO uint32_t GCTL;				///< 0x00
    __IO uint32_t GINT0;			///< 0x04
    __IO uint32_t GINT1;			///< 0x08
    __I  uint32_t RES0[14];   		///< 0x0C-0x40
    __IO uint32_t TCON0_DCLK;		///< 0x44
    __I  uint32_t RES1[17];   		///< 0x48-0x88
    __IO uint32_t TCON0_IO_TRI;		///< 0x8C
    __IO uint32_t TCON1_CTL;		///< 0x90
    __IO uint32_t TCON1_BASIC0;		///< 0x94	Timing Source
    __IO uint32_t TCON1_BASIC1;		///< 0x98 	Timing Scale
    __IO uint32_t TCON1_BASIC2;		///< 0x9C	Timing Out
    __IO uint32_t TCON1_BASIC3;		///< 0xA0	Timing H
    __IO uint32_t TCON1_BASIC4;		///< 0xA4	Timing V
    __IO uint32_t TCON1_BASIC5;		///< 0xA8	Timing Sync
    __I  uint32_t RES2[17];   		///< 0xAC-0xDF
    __IO uint32_t TCON1_IO_POL;		///< 0xF0
    __IO uint32_t TCON1_IO_TRI;		///< 0xF4
} H3_TCON_TypeDef;

typedef struct T_H3_SD_MMC {
    __IO uint32_t GCTL;				///< 0x00
    __IO uint32_t CKC;				///< 0x04
    __IO uint32_t TMO;				///< 0x08
    __IO uint32_t BWD;				///< 0x0C
    __IO uint32_t BKS;				///< 0x10
    __IO uint32_t BYC;				///< 0x14
    __IO uint32_t CMD;				///< 0x18
    __IO uint32_t ARG;				///< 0x1C
    __IO uint32_t RESP0;			///< 0x20
    __IO uint32_t RESP1;			///< 0x24
    __IO uint32_t RESP2;			///< 0x28
    __IO uint32_t RESP3;			///< 0x2C
    __IO uint32_t IMK;				///< 0x30
    __IO uint32_t MIS;				///< 0x34
    __IO uint32_t RIS;				///< 0x38
    __I uint32_t STA;				///< 0x3C
    __IO uint32_t FWL;				///< 0x40
    __IO uint32_t FUNS;				///< 0x44
    __IO uint32_t RES0[4];          ///< 0x48,0x4C,0x50,0x54
    __IO uint32_t A12A;				///< 0x58
    __IO uint32_t ntsr;          	///< 0x5C
    __IO uint32_t RES1[6];       	///< 0x60,0x64,0x68,0x6C,0x70,0x74
    __IO uint32_t HWRST;         	///< 0x78
    __IO uint32_t RES2;          	///< 0x7C
    __IO uint32_t DMAC;          	///< 0x80
    __IO uint32_t DLBA;          	///< 0x84
    __IO uint32_t IDST;          	///< 0x88
    __IO uint32_t IDIE;          	///< 0x8C
    __IO uint32_t RES3[28];			///< 0x90-0xFC
    __IO uint32_t THLDC;			///< 0x100
    __IO uint32_t RES4[2];			///< 0x104,0x108
    __IO uint32_t DSBD;				///< 0x10C
    __IO uint32_t RES_CRC;			///< 0x110
    __IO uint32_t DATA7_CRC;		///< 0x114
    __IO uint32_t DATA6_CRC;		///< 0x118
    __IO uint32_t DATA5_CRC;		///< 0x11C
    __IO uint32_t DATA4_CRC;		///< 0x120
    __IO uint32_t DATA3_CRC;		///< 0x124
    __IO uint32_t DATA2_CRC;		///< 0x128
    __IO uint32_t DATA1_CRC;		///< 0x12C
    __IO uint32_t DATA0_CRC;		///< 0x130
    __IO uint32_t CRC_STA;			///< 0x134
    __IO uint32_t RES5[50];			///< 0x138-0x1FC
    __IO uint32_t FIFO;          	///< 0x200
} H3_SD_MMC_TypeDef;

typedef struct T_H3_SPINLOCK {
    __I  uint32_t SYS_STATUS;		///< 0x00
    __I  uint32_t RES1[3];			///< 0x04, 0x08, 0x0C
    __I  uint32_t STATUS;			///< 0x10
    __I  uint32_t RES2[59];			///< 0x14-0x0FC
    __IO uint32_t LOCK[32];         ///< 0x100
} H3_SPINLOCK_TypeDef;

typedef struct T_H3_SID {			///< http://linux-sunxi.org/SID_Register_Guide
    __I  uint32_t RES1[16];			///< 0x00-0x3C
    __IO uint32_t PRCTL;			///< 0x40 Control register
    __I  uint32_t RES2[3];			///< 0x44,0x48,0x4C
    __IO uint32_t PRKEY; 			///< 0x50 Program data
    __I  uint32_t RES33[3];			///< 0x54,0x58,0x5C
    __IO uint32_t RDKEY; 			///< 0x60 Read data
} H3_SID_TypeDef;

typedef struct T_H3_CCU {
    __IO uint32_t PLL_CPUX_CTRL;	///< 0x00
    __IO uint32_t RES0;				///< 0x04
    __IO uint32_t PLL_AUDIO_CTRL;	///< 0x08
    __IO uint32_t RES1;				///< 0x0C
    __IO uint32_t PLL_VIDEO_CTRL;	///< 0x10
    __IO uint32_t RES2;				///< 0x14
    __IO uint32_t PLL_VE_CTRL;		///< 0x18
    __IO uint32_t RES3;				///< 0x1C
    __IO uint32_t PLL_DDR_CTRL;		///< 0x20
    __IO uint32_t RES4;				///< 0x24
    __IO uint32_t PLL_PERIPH0_CTRL;	///< 0x28
    __IO uint32_t RES5[3];			///< 0x2C,0x30,0x34
    __IO uint32_t PLL_GPU_CTRL;		///< 0x38
    __IO uint32_t RES6[2];			///< 0x3C,0x40
    __IO uint32_t PLL_PERIPH1_CTRL;	///< 0x44
    __IO uint32_t PLL_DE_CTRL;		///< 0x48
    __IO uint32_t RES61;			///< 0x4C
    __IO uint32_t CPU_AXI_CFG;		///< 0x50
    __IO uint32_t AHB1_APB1_CFG;	///< 0x54
    __IO uint32_t APB2_CFG;			///< 0x58
    __IO uint32_t AHB2_CFG;			///< 0x5C
    __IO uint32_t BUS_CLK_GATING0;	///< 0x60
    __IO uint32_t BUS_CLK_GATING1;	///< 0x64
    __IO uint32_t BUS_CLK_GATING2;	///< 0x68
    __IO uint32_t BUS_CLK_GATING3;	///< 0x6C
    __IO uint32_t BUS_CLK_GATING4;	///< 0x70
    __IO uint32_t THS_CLK;			///< 0x74
    __IO uint32_t RES8[2];			///< 0x78,0x7C
    __IO uint32_t NAND_CLK;			///< 0x80
    __IO uint32_t RES9;				///< 0x84
    __IO uint32_t SDMMC0_CLK;		///< 0x88
    __IO uint32_t SDMMC1_CLK;		///< 0x8C
    __IO uint32_t SDMMC2_CLK;		///< 0x90
    __IO uint32_t RES10[2];			///< 0x94,0x98
    __IO uint32_t CE_CLK;			///< 0x9C
    __IO uint32_t SPI0_CLK;			///< 0xA0
    __IO uint32_t SPI1_CLK;			///< 0xA4
    __IO uint32_t RES11[2];			///< 0xA8,0xAC
    __IO uint32_t I2SPCM0_CLK;		///< 0xB0
    __IO uint32_t I2SPCM1_CLK;		///< 0xB4
    __IO uint32_t I2SPCM2_CLK;		///< 0xB8
    __IO uint32_t RES11A;			///< 0xBC
    __IO uint32_t OWA_CLK;			///< 0xC0
    __IO uint32_t RES12[2];			///< 0xC4,0xC8
    __IO uint32_t USBPHY_CFG;		///< 0xCC
    __IO uint32_t RES13[9];			///< 0xD0
    __IO uint32_t DRAM_CFG;			///< 0xF4
    __IO uint32_t RES131;			///< 0xF8
    __IO uint32_t MBUS_RESET;		///< 0xFC
    __IO uint32_t DRAM_CLK_GATING;	///< 0x100
    __IO uint32_t DE_CLK;			///< 0x104
    __IO uint32_t RES14[4];			///< 0x108
    __IO uint32_t TCON0_CLK;		///< 0x118
    __IO uint32_t TCON1_CLK;		///< 0x11C
    __IO uint32_t RES15[4];			///< 0x120
    __IO uint32_t mipi_csi_CLK_CFG;	///< 0x130 MIPI CSI module clock
    __IO uint32_t csi_CLK_CFG;		///< 0x134 CSI module clock
    __IO uint32_t RES16;			///< 0x138
    __IO uint32_t ve_CLK_CFG;		///< 0x13c VE module clock
    __IO uint32_t AC_DIG_CLK;		///< 0x140 AC Digital Clock Register
    __IO uint32_t AVS_CLK_CFG;		///< 0x144 AVS module clock
    __IO uint32_t RES18[2];			///< 0x148
    __IO uint32_t HDMI_CLK;			///< 0x150 HDMI module clock
    __IO uint32_t HDMI_SLOW_CLK;	///< 0x154 HDMI slow module clock
    __IO uint32_t RES19;			///< 0x158
    __IO uint32_t mbus_CLK_CFG;		///< 0x15c MBUS module clock
    __IO uint32_t RES20[2];			///< 0x160
    __IO uint32_t mipi_dsi_CLK_CFG;	///< 0x168 MIPI DSI clock control
    __IO uint32_t RES21[13];		///< 0x16c
    __IO uint32_t GPU_core_CLK_CFG;	///< 0x1a0 GPU core clock config
    __IO uint32_t GPU_mem_CLK_CFG;	///< 0x1a4 GPU memory clock config
    __IO uint32_t GPU_hyd_CLK_CFG;	///< 0x1a8 GPU HYD clock config
    __IO uint32_t RES22[21];		///< 0x1ac
    __IO uint32_t PLL_stable0;		///< 0x200 PLL stable time 0
    __IO uint32_t PLL_stable1;		///< 0x204 PLL stable time 1
    __IO uint32_t RES23;			///< 0x208
    __IO uint32_t PLL_stable_status;///< 0x20c PLL stable status register
    __IO uint32_t RES24[4];			///< 0x210
    __IO uint32_t PLL1_C0_BIAS_CFG;	///< 0x220 PLL1 C0cpu# Bias config
    __IO uint32_t PLL2_BIAS_CFG;	///< 0x224 PLL2 audio Bias config
    __IO uint32_t PLL3_BIAS_CFG;	///< 0x228 PLL3 video Bias config
    __IO uint32_t PLL4_BIAS_CFG;	///< 0x22c PLL4 ve Bias config
    __IO uint32_t PLL5_BIAS_CFG;	///< 0x230 PLL5 ddr Bias config
    __IO uint32_t PLL6_BIAS_CFG;	///< 0x234 PLL6 periph Bias config
    __IO uint32_t PLL1_C1_BIAS_CFG;	///< 0x238 PLL1 C1cpu# Bias config
    __IO uint32_t PLL8_BIAS_CFG;	///< 0x23c PLL7 Bias config
    __IO uint32_t RES25;			///< 0x240
    __IO uint32_t PLL9_BIAS_CFG;	///< 0x244 PLL9 hsic Bias config
    __IO uint32_t de_BIAS_CFG;		///< 0x248 display engine Bias config
    __IO uint32_t video1_BIAS_CFG;	///< 0x24c PLL video1 bias register
    __IO uint32_t C0_TUNING_CFG;	///< 0x250 PLL C0cpu# TUNING register
    __IO uint32_t C1_TUNING_CFG;	///< 0x254 PLL C1cpu# TUNING register
    __IO uint32_t RES26[11];		///< 0x258
    __IO uint32_t PLL_AUDIO_PAT_CTRL;	///< 0x284 PLL_AUDIO Pattern Control Register
    __IO uint32_t PLL3_pattern_CFG0;///< 0x288 PLL3 Pattern register 0
    __IO uint32_t RES27;			///< 0x28c
    __IO uint32_t PLL5_pattern_CFG0;///< 0x290 PLL5 Pattern register 0
    __IO uint32_t RES28[4];			///< 0x294
    __IO uint32_t PLL2_pattern_CFG1;///< 0x2a4 PLL2 Pattern register 1
    __IO uint32_t PLL3_pattern_CFG1;///< 0x2a8 PLL3 Pattern register 1
    __IO uint32_t RES29;			///< 0x2ac
    __IO uint32_t PLL5_pattern_CFG1;///< 0x2b0 PLL5 Pattern register 1
    __IO uint32_t RES30[3];			///< 0x2b4
    __IO uint32_t BUS_SOFT_RESET0;	///< 0x2C0
    __IO uint32_t BUS_SOFT_RESET1;	///< 0x2C4
    __IO uint32_t BUS_SOFT_RESET2;	///< 0x2C8
    __IO uint32_t RES31;			///< 0x2CC
    __IO uint32_t BUS_SOFT_RESET3;	///< 0x2D0
    __IO uint32_t RES32;			///< 0x2D4
    __IO uint32_t BUS_SOFT_RESET4;	///< 0x2D8
} H3_CCU_TypeDef;

typedef struct T_H3_PIO {
    __IO uint32_t CFG0;				///< Configure Register 0
    __IO uint32_t CFG1; 			///< Configure Register 1
    __IO uint32_t CFG2; 			///< Configure Register 2
    __IO uint32_t CFG3; 			///< Configure Register 3
    __IO uint32_t DAT;				///< Data Register
    __IO uint32_t DRV0; 			///< Multi-Driving Register 0
    __IO uint32_t DRV1; 			///< Multi-Driving Register 1
    __IO uint32_t PUL0;				///< Pull Register 0
    __IO uint32_t PUL1;				///< Pull Register 1
} H3_PIO_TypeDef;

typedef struct T_H3_PIO_INT {
    __IO uint32_t CFG0;				///< 0x00 Interrupt Configure Register 0
    __IO uint32_t CFG1; 			///< 0x04 Interrupt Configure Register 1
    __IO uint32_t CFG2; 			///< 0x08 Interrupt Configure Register 2
    __IO uint32_t CFG3; 			///< 0x0C Interrupt Configure Register 3
    __IO uint32_t CTL;				///< 0x10 Interrupt Control Register
    __IO uint32_t STA; 				///< 0x14 Interrupt Status Register
    __IO uint32_t DEB; 				///< 0x18 Interrupt Debounce Register
} H3_PIO_INT_TypeDef;

typedef struct T_H3_TIMER {
    __IO uint32_t IRQ_EN;			///< 0x00 IRQ Enable Register
    __IO uint32_t IRQ_STA;			///< 0x04 IRQ Status Register
    __IO uint32_t RES1[2];			///< 0x08, 0x0C
    __IO uint32_t TMR0_CTRL;		///< 0x10 Timer 0 Control Register
    __IO uint32_t TMR0_INTV;		///< 0x14 Timer 0 Interval Value Register
    __IO uint32_t TMR0_CUR;			///< 0x18 Timer 0 Current Value Register
    __IO uint32_t RES2;				///< 0x1C
    __IO uint32_t TMR1_CTRL;		///< 0x20 Timer 1 Control Register
    __IO uint32_t TMR1_INTV;		///< 0x24 Timer 1 Interval Value Register
    __IO uint32_t TMR1_CUR;			///< 0x28 Timer 1 Current Value Register
    __IO uint32_t RES3[21];			///< 0x2C-0x7C
    __IO uint32_t AVS_CTRL;			///< 0x80 AVS Control Register
    __IO uint32_t AVS_CNT0;			///< 0x84 AVS Counter 0 Register
    __IO uint32_t AVS_CNT1;			///< 0x88 AVS Counter 1 Register
    __IO uint32_t AVS_DIV;			///< 0x8C AVS Divisor Register
    __IO uint32_t RES4[4];			///< 0x90,0x94,0x98,0x9C,
    __IO uint32_t WDOG0_IRQ_EN;		///< 0xA0 Watchdog 0 IRQ Enable Register
    __IO uint32_t WDOG0_IRQ_STA;	///< 0xA4 Watchdog 0 IRQ Status Register
    __IO uint32_t RES5[2];			///< 0xA8, 0xAC
    __IO uint32_t WDOG0_CTRL;		///< 0xB0 Watchdog 0 Control Register
    __IO uint32_t WDOG0_CFG;		///< 0xB4 Watchdog 0 Configuration Register
    __IO uint32_t WDOG0_MODE;		///< 0xB8 Watchdog 0 Mode Register
} H3_TIMER_TypeDef;

typedef struct H3_AC {
    __IO uint32_t DAC_DPC;			///< 0x000 DAC Digital Part Control Register
    __IO uint32_t DAC_FIFOC;		///< 0x004 DAC FIFO Control Register
    __IO uint32_t DAC_FIFOS;		///< 0x008 DAC FIFO Status Register
    __I  uint32_t RES1;				///< 0x00C
    __IO uint32_t ADC_FIFOC;		///< 0x010 ADC FIFO Control Register
    __IO uint32_t ADC_FIFOS;		///< 0x014 ADC FIFO Status Register
    __I  uint32_t ADC_RXDATA;		///< 0x018 ADC RX Data Register
    __I  uint32_t RES2;				///< 0x01C
    __O  uint32_t DAC_TXDATA;		///< 0x020 DAC TX Data Register
    __I  uint32_t RES3[7];			///< 0x024
    __IO uint32_t DAC_CNT;			///< 0x040 DAC TX FIFO Counter Register
    __IO uint32_t ADC_CNT;			///< 0x044 ADC RX FIFO Counter Register
    __IO uint32_t DAC_DG;			///< 0x048 DAC Debug Register
    __IO uint32_t ADC_DG;			///< 0x04C ADC Debug Register
    __I  uint32_t RES4[4];			///< 0x050
    __IO uint32_t DAC_DAP_CTR;		///< 0x060 DAC DAP Control Register
    __I  uint32_t RES5[3];			///< 0x064
    __I  uint32_t ADC_DAP[36];		///< 0x070 ADC_DAP TODO
    __IO uint32_t DAC_DRC_HHPFC;	///< 0x100 DAC DRC High HPF Coef Register
    __IO uint32_t DAC_DRC_LHPFC;	///< 0x104 DAC DRC Low HPF Coef Register
    __IO uint32_t DAC_DRC_CTRL;		///< 0x108 DAC DRC Control Register
} H3_AC_TypeDef;

typedef struct T_H3_TWI {
    __IO uint32_t ADDR;				///< 0x00 TWI Slave Address
    __IO uint32_t XADDR;			///< 0x04 TWI Extended slave Address
    __IO uint32_t DATA;				///< 0x08
    __IO uint32_t CTL;				///< 0x0C
    __I  uint32_t STAT;				///< 0x10 TWI Status register
    __IO uint32_t CC;				///< 0x14 TWI Clock Control Register
    __IO uint32_t SRST;				///< 0x18 TWI Software reset
    __IO uint32_t EFR;				///< 0x1C TWI Enhance Feature Register
    __IO uint32_t LCR;				///< 0x20 TWI Line Control register
    __IO uint32_t DVFS;				///< 0x24
} H3_TWI_TypeDef;

typedef struct T_H3_SPI {
    __I  uint32_t RES1;				///< 0x00
    __IO uint32_t GC;				///< 0x04 SPI Global Control Register
    __IO uint32_t TC;				///< 0x08 SPI Transfer Control Register
    __I  uint32_t RES2;				///< 0x0C
    __IO uint32_t IE;				///< 0x10 SPI Interrupt Control Register
    __IO uint32_t IS;				///< 0x14 SPI Interrupt Status Register
    __IO uint32_t FC;				///< 0x18 SPI FIFO Control Register
    __IO uint32_t FS;				///< 0x1C SPI FIFO Status Register
    __IO uint32_t WC;				///< 0x20 SPI
    __IO uint32_t CC;				///< 0x24 SPI
    __I  uint32_t RES3[2];			///< 0x28,x2C
    __IO uint32_t MBC;				///< 0x30 SPI Burst Counter Register
    __IO uint32_t MTC;				///< 0x34 SPI Transmit Counter Register
    __IO uint32_t BCC;				///< 0x38 SPI Burst Control Register
    __I  uint32_t RES4[113];		///<
    union {							///< 0x200 SPI TX Data Register
        __O uint8_t byte;
        __O uint32_t word;
    } TX;
    __I uint32_t RES5[63];			///<
    union {							///< 0x300 SPI RX Data Register
        __I uint8_t byte;
        __I uint32_t word;
    } RX;
}H3_SPI_TypeDef;

/* http://linux-sunxi.org/PRCM */

#define EXT_I2C_NUMBER			((EXT_I2C_BASE - H3_TWI_BASE) / 0x400)
#define EXT_I2C					(_CAST(H3_TWI_TypeDef *)(EXT_I2C_BASE))
//#define EXT_I2C_SDA				GPIO_EXT_3
//#define EXT_I2C_SCL				GPIO_EXT_5

#endif // H3_H
