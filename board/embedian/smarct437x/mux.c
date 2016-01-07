/*
 * mux.c
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mux.h>
#include "board.h"

static struct module_pin_mux rmii1_pin_mux[] = {
	{OFFSET(mii1_txen), MODE(1)},			/* RMII1_TXEN */
	{OFFSET(mii1_txd1), MODE(1)},			/* RMII1_TD1 */
	{OFFSET(mii1_txd0), MODE(1)},			/* RMII1_TD0 */
	{OFFSET(mii1_rxd1), MODE(1) | RXACTIVE},	/* RMII1_RD1 */
	{OFFSET(mii1_rxd0), MODE(1) | RXACTIVE},	/* RMII1_RD0 */
	{OFFSET(mii1_rxdv), MODE(1) | RXACTIVE},	/* RMII1_RXDV */
	{OFFSET(mii1_crs), MODE(1) | RXACTIVE},		/* RMII1_CRS_DV */
	{OFFSET(mii1_rxerr), MODE(1) | RXACTIVE},	/* RMII1_RXERR */
	{OFFSET(rmii1_refclk), MODE(0) | RXACTIVE},	/* RMII1_refclk */
	{-1},
};

/* LAN1 */
static struct module_pin_mux rgmii1_pin_mux[] = {
	{OFFSET(mii1_txen), MODE(2)},			/* RGMII1_TCTL */
	{OFFSET(mii1_rxdv), MODE(2) | RXACTIVE},	/* RGMII1_RCTL */
	{OFFSET(mii1_txd3), MODE(2)},			/* RGMII1_TD3 */
	{OFFSET(mii1_txd2), MODE(2)},			/* RGMII1_TD2 */
	{OFFSET(mii1_txd1), MODE(2)},			/* RGMII1_TD1 */
	{OFFSET(mii1_txd0), MODE(2)},			/* RGMII1_TD0 */
	{OFFSET(mii1_txclk), MODE(2)},			/* RGMII1_TCLK */
	{OFFSET(mii1_rxclk), MODE(2) | RXACTIVE},	/* RGMII1_RCLK */
	{OFFSET(mii1_rxd3), MODE(2) | RXACTIVE},	/* RGMII1_RD3 */
	{OFFSET(mii1_rxd2), MODE(2) | RXACTIVE},	/* RGMII1_RD2 */
	{OFFSET(mii1_rxd1), MODE(2) | RXACTIVE},	/* RGMII1_RD1 */
	{OFFSET(mii1_rxd0), MODE(2) | RXACTIVE},	/* RGMII1_RD0 */
	{-1},
};

/* LAN2 */
static struct module_pin_mux rgmii2_pin_mux[] = {
        {OFFSET(gpmc_a0), MODE(2)},                  	/* RGMII2_TCTL */
        {OFFSET(gpmc_a1), MODE(2) | RXACTIVE},        	/* RGMII2_RCTL */
        {OFFSET(gpmc_a2), MODE(2)},                   	/* RGMII2_TD3 */
        {OFFSET(gpmc_a3), MODE(2)},                   	/* RGMII2_TD2 */
        {OFFSET(gpmc_a4), MODE(2)},                   	/* RGMII2_TD1 */
        {OFFSET(gpmc_a5), MODE(2)},                   	/* RGMII2_TD0 */
        {OFFSET(gpmc_a6), MODE(2)},                  	/* RGMII2_TCLK */
        {OFFSET(gpmc_a7), MODE(2) | RXACTIVE},       	/* RGMII2_RCLK */
        {OFFSET(gpmc_a8), MODE(2) | RXACTIVE},        	/* RGMII2_RD3 */
        {OFFSET(gpmc_a9), MODE(2) | RXACTIVE},        	/* RGMII2_RD2 */
        {OFFSET(gpmc_a10), MODE(2) | RXACTIVE},        	/* RGMII2_RD1 */
        {OFFSET(gpmc_a11), MODE(2) | RXACTIVE},        	/* RGMII2_RD0 */
        {-1},
};

static struct module_pin_mux mdio_pin_mux[] = {
	{OFFSET(mdio_data), MODE(0) | RXACTIVE | PULLUP_EN},	/* MDIO_DATA */
	{OFFSET(mdio_clk), MODE(0) | PULLUP_EN},		/* MDIO_CLK */
	{-1},
};

/* SER0 */
static struct module_pin_mux uart0_pin_mux[] = {
	{OFFSET(uart0_rxd), (MODE(0) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
	{OFFSET(uart0_txd), (MODE(0) | PULLUDDIS | PULLUP_EN | SLEWCTRL)},
        {OFFSET(uart0_ctsn), (MODE(0) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
        {OFFSET(uart0_rtsn), (MODE(0) | PULLUDDIS | PULLUP_EN | SLEWCTRL)},
	{-1},
};

/* SER2 */
static struct module_pin_mux uart2_pin_mux[] = {
        {OFFSET(cam1_data4), (MODE(2) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
        {OFFSET(cam1_data5), (MODE(2) | PULLUDDIS | PULLUP_EN | SLEWCTRL)},
        {OFFSET(cam1_data6), (MODE(2) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
        {OFFSET(cam1_data7), (MODE(2) | PULLUDDIS | PULLUP_EN | SLEWCTRL)},
        {-1},
};

/* SER1 */
static struct module_pin_mux uart3_pin_mux[] = {
        {OFFSET(uart3_rxd), (MODE(0) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
        {OFFSET(uart3_txd), (MODE(0) | PULLUDDIS | PULLUP_EN | SLEWCTRL)},
        {-1},
};

/* SER3 */
static struct module_pin_mux uart4_pin_mux[] = {
        {OFFSET(gpmc_wait0), (MODE(6) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
        {OFFSET(gpmc_wpn), (MODE(6) | PULLUDDIS | PULLUP_EN | SLEWCTRL)},
        {-1},
};

/* SD */
static struct module_pin_mux mmc0_pin_mux[] = {
	{OFFSET(mmc0_clk), (MODE(0) | PULLUDDIS | RXACTIVE)},	/* MMC0_CLK */
	{OFFSET(mmc0_cmd), (MODE(0) | PULLUP_EN | RXACTIVE)},  	/* MMC0_CMD */
	{OFFSET(mmc0_dat0), (MODE(0) | PULLUP_EN | RXACTIVE)}, 	/* MMC0_DAT0 */
	{OFFSET(mmc0_dat1), (MODE(0) | PULLUP_EN | RXACTIVE)}, 	/* MMC0_DAT1 */
	{OFFSET(mmc0_dat2), (MODE(0) | PULLUP_EN | RXACTIVE)}, 	/* MMC0_DAT2 */
	{OFFSET(mmc0_dat3), (MODE(0) | PULLUP_EN | RXACTIVE)}, 	/* MMC0_DAT3 */
	/*{OFFSET(rmii1_refclk), MODE(5) | PULLUP_EN},*/ 	/* SDIO_PWREN */
	{-1},
};

/* EMMC */
static struct module_pin_mux mmc1_pin_mux[] = {
        {OFFSET(gpmc_csn1), (MODE(2) | PULLUDDIS | RXACTIVE)}, 	/* MMC1_CLK */
        {OFFSET(gpmc_csn2), (MODE(2) | PULLUP_EN | RXACTIVE)}, 	/* MMC1_CMD */
        {OFFSET(gpmc_ad0), (MODE(1) | PULLUP_EN | RXACTIVE)}, 	/* MMC1_DAT0 */
        {OFFSET(gpmc_ad1), (MODE(1) | PULLUP_EN | RXACTIVE)}, 	/* MMC1_DAT1 */
        {OFFSET(gpmc_ad2), (MODE(1) | PULLUP_EN | RXACTIVE)}, 	/* MMC1_DAT2 */
        {OFFSET(gpmc_ad3), (MODE(1) | PULLUP_EN | RXACTIVE)}, 	/* MMC1_DAT3 */
        {OFFSET(gpmc_ad4), (MODE(1) | PULLUP_EN | RXACTIVE)}, 	/* MMC1_DAT4 */
        {OFFSET(gpmc_ad5), (MODE(1) | PULLUP_EN | RXACTIVE)}, 	/* MMC1_DAT5 */
        {OFFSET(gpmc_ad6), (MODE(1) | PULLUP_EN | RXACTIVE)}, 	/* MMC1_DAT6 */
        {OFFSET(gpmc_ad7), (MODE(1) | PULLUP_EN | RXACTIVE)}, 	/* MMC1_DAT7 */
        {-1},
};

/* SDMMC */
static struct module_pin_mux mmc2_pin_mux[] = {
        {OFFSET(gpmc_clk), (MODE(3) | PULLUDDIS | RXACTIVE)}, 	/* MMC2_CLK */
        {OFFSET(gpmc_csn3), (MODE(3) | PULLUP_EN | RXACTIVE)}, 	/* MMC2_CMD */
        {OFFSET(gpmc_ad12), (MODE(3) | PULLUP_EN | RXACTIVE)},	/* MMC2_DAT0 */
        {OFFSET(gpmc_ad13), (MODE(3) | PULLUP_EN | RXACTIVE)}, 	/* MMC2_DAT1 */
        {OFFSET(gpmc_ad14), (MODE(3) | PULLUP_EN | RXACTIVE)}, 	/* MMC2_DAT2 */
        {OFFSET(gpmc_ad15), (MODE(3) | PULLUP_EN | RXACTIVE)}, 	/* MMC2_DAT3 */
        {OFFSET(gpmc_ad8), (MODE(3) | PULLUP_EN | RXACTIVE)}, 	/* MMC2_DAT4 */
        {OFFSET(gpmc_ad9), (MODE(3) | PULLUP_EN | RXACTIVE)}, 	/* MMC2_DAT5 */
        {OFFSET(gpmc_ad10), (MODE(3) | PULLUP_EN | RXACTIVE)}, 	/* MMC2_DAT6 */
        {OFFSET(gpmc_ad11), (MODE(3) | PULLUP_EN | RXACTIVE)}, 	/* MMC2_DAT7 */
        {-1},
};

/* I2C_GP */
static struct module_pin_mux i2c0_pin_mux[] = {
	{OFFSET(i2c0_sda), (MODE(0) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
	{OFFSET(i2c0_scl), (MODE(0) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
	{-1},
};

/* I2C_PM */
static struct module_pin_mux i2c1_pin_mux[] = {
        {OFFSET(mii1_crs), (MODE(3) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
        {OFFSET(mii1_rxerr), (MODE(3) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
        {-1},
};

/* I2C_LCD */
static struct module_pin_mux i2c2_pin_mux[] = {
        {OFFSET(cam1_data0), (MODE(3) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
        {OFFSET(cam1_data1), (MODE(3) | PULLUP_EN | RXACTIVE | SLEWCTRL)},
        {-1},
};

/* GPIO */
static struct module_pin_mux smarc_gpio_pin_mux[] = {
        {OFFSET(mii1_col), (MODE(9) | PULLUP_EN | RXACTIVE)},  	/* USB0_OC#, mii1_col.gpio0_0*/
        {OFFSET(gpmc_be1n), (MODE(7) | PULLUP_EN | RXACTIVE)}, 	/* USB1_OC#, gmpc_be1n.gpio1.28 */
        {OFFSET(rmii1_refclk), (MODE(7) | PULLUP_EN)},  	/* SDIO_PWREN, rmii1_refclk.gpio0.29 */
/* By SMARC Spec. GPIO0-5 is recommended for use as outputs. */
        {OFFSET(spi2_cs0), (MODE(9) | PULLUP_EN)},             	/* GPIO0, spi2_cs0.gpio0_23 */
        {OFFSET(spi2_d0), (MODE(9) | PULLUP_EN)},          	/* GPIO1, spi2_d0_gpio0_20 */
        {OFFSET(spi2_d1), (MODE(9) | PULLUP_EN)},              	/* GPIO2, spi2_d1.gpio0_21 */
        {OFFSET(spi2_sclk), (MODE(9) | PULLUP_EN)},            	/* GPIO3, spi2_sclk.gpio0_22*/
        {OFFSET(cam0_data5), (MODE(7) | PULLUP_EN)},           	/* GPIO4, cam0_data5.gpio4_7 */
        {OFFSET(cam0_data7), (MODE(7) | PULLUP_EN)},           	/* GPIO5, cam0_data7.gpio4_29 */

/* By SMARC Spec. GPIO6-11 is recommended for use of inputs */
        {OFFSET(mcasp0_ahclkr), (MODE(7) | RXACTIVE)},  		/* GPIO6, mcasp0.ahclkr.gpio3_7 */
        {OFFSET(mcasp0_axr0), (MODE(7) | PULLUP_EN | RXACTIVE)},	/* GPIO7, mcasp0.axr0.gpio3_6 */
        {OFFSET(cam0_data2), (MODE(7) | PULLUP_EN | RXACTIVE)},        	/* GPIO8, cam0_data2.gpio4_24 */
        {OFFSET(cam0_data3), (MODE(7) | PULLUP_EN | RXACTIVE)},        	/* GPIO9, cam0_data3.gpio4_25 */
        {OFFSET(cam0_data4), (MODE(7) | PULLUP_EN | RXACTIVE)},       	/* GPIO10, cam0_data4_gpio4_26 */
        {OFFSET(cam0_data6), (MODE(7)  | PULLUP_EN | RXACTIVE)},       	/* GPIO11, cam0_data6_gpio4_28 */
        {-1},
};

/* DSS LCD */
static struct module_pin_mux dss_pin_mux[] = {
    	{OFFSET(lcd_data0), (MODE(0) | PULLUDDIS)},
	{OFFSET(lcd_data1), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data2), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data3), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data4), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data5), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data6), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data7), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data8), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data9), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data10), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data11), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data12), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data13), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data14), (MODE(0) | PULLUDDIS)},
    	{OFFSET(lcd_data15), (MODE(0) | PULLUDDIS)},
	/* DSS DATA16~23 */
    	{OFFSET(cam1_data9), (MODE(2) | PULLUDDIS)},
    	{OFFSET(cam0_data9), (MODE(2) | PULLUDDIS)},
    	{OFFSET(cam0_data8), (MODE(2) | PULLUDDIS)},
    	{OFFSET(cam0_pclk), (MODE(2) | PULLUDDIS)},
    	{OFFSET(cam0_wen), (MODE(2) | PULLUDDIS)},
    	{OFFSET(cam0_field), (MODE(2) | PULLUDDIS)},
    	{OFFSET(cam0_vd), (MODE(2) | PULLUDDIS)},
    	{OFFSET(cam0_hd), (MODE(2) | PULLUDDIS)},
    	{OFFSET(lcd_vsync), (MODE(0) | PULLUDEN)},
    	{OFFSET(lcd_hsync), (MODE(0) | PULLUDEN)},
    	{OFFSET(lcd_pclk), (MODE(0) | PULLUDEN)},
    	{OFFSET(lcd_ac_bias_en), (MODE(0) | PULLUDEN)},
    	{-1},
 };

static struct module_pin_mux gpio5_7_pin_mux[] = {
	{OFFSET(spi0_cs0), (MODE(7) | PULLUP_EN)},	/* GPIO5_7 */
	{-1},
};

#ifdef CONFIG_NAND
static struct module_pin_mux nand_pin_mux[] = {
	{OFFSET(gpmc_ad0),	(MODE(0) | PULLUDDIS | RXACTIVE)},	/* AD0 */
	{OFFSET(gpmc_ad1),	(MODE(0) | PULLUDDIS | RXACTIVE)},	/* AD1 */
	{OFFSET(gpmc_ad2),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD2 */
	{OFFSET(gpmc_ad3),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD3 */
	{OFFSET(gpmc_ad4),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD4 */
	{OFFSET(gpmc_ad5),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD5 */
	{OFFSET(gpmc_ad6),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD6 */
	{OFFSET(gpmc_ad7),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD7 */
#ifdef CONFIG_SYS_NAND_BUSWIDTH_16BIT
	{OFFSET(gpmc_ad8),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD8  */
	{OFFSET(gpmc_ad9),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD9  */
	{OFFSET(gpmc_ad10),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD10 */
	{OFFSET(gpmc_ad11),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD11 */
	{OFFSET(gpmc_ad12),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD12 */
	{OFFSET(gpmc_ad13),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD13 */
	{OFFSET(gpmc_ad14),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD14 */
	{OFFSET(gpmc_ad15),	(MODE(0) | PULLUDDIS | RXACTIVE)}, 	/* AD15 */
#endif
	{OFFSET(gpmc_wait0),	(MODE(0) | RXACTIVE | PULLUP_EN)}, 	/* Wait */
	{OFFSET(gpmc_wpn),	(MODE(7) | PULLUP_EN)},			/* Write Protect */
	{OFFSET(gpmc_csn0),	(MODE(0) | PULLUP_EN)},			/* Chip-Select */
	{OFFSET(gpmc_wen),	(MODE(0) | PULLDOWN_EN)}, 		/* Write Enable */
	{OFFSET(gpmc_oen_ren),	(MODE(0) | PULLDOWN_EN)}, 		/* Read Enable */
	{OFFSET(gpmc_advn_ale),	(MODE(0) | PULLDOWN_EN)}, 		/* Addr Latch Enable*/
	{OFFSET(gpmc_be0n_cle),	(MODE(0) | PULLDOWN_EN)}, 		/* Byte Enable */
	{-1},
};
#endif

static __maybe_unused struct module_pin_mux qspi_pin_mux[] = {
	{OFFSET(gpmc_csn0), (MODE(3) | PULLUP_EN | RXACTIVE)}, 		/* QSPI_CS0 */
	{OFFSET(gpmc_csn3), (MODE(2) | PULLUP_EN | RXACTIVE)}, 		/* QSPI_CLK */
	{OFFSET(gpmc_advn_ale), (MODE(3) | PULLUP_EN | RXACTIVE)}, 	/* QSPI_D0 */
	{OFFSET(gpmc_oen_ren), (MODE(3) | PULLUP_EN | RXACTIVE)}, 	/* QSPI_D1 */
	{OFFSET(gpmc_wen), (MODE(3) | PULLUP_EN | RXACTIVE)}, 		/* QSPI_D2 */
	{OFFSET(gpmc_be0n_cle), (MODE(3) | PULLUP_EN | RXACTIVE)}, 	/* QSPI_D3 */
	{-1},
};

/* SPI BOOT */
static struct module_pin_mux spi0_pin_mux[] = {
	{OFFSET(spi0_d0),   (MODE(0) | RXACTIVE | PULLUDEN)},
	{OFFSET(spi0_d1),   (MODE(0) | PULLUDEN)},
	{OFFSET(spi0_cs0),  (MODE(0) | PULLUDEN)},
	{OFFSET(spi0_sclk), (MODE(0) | RXACTIVE | PULLUDEN)},
	{-1},
};

/* SPI0 */
static struct module_pin_mux spi2_pin_mux[] = {
        {OFFSET(cam1_hd), (MODE(4) | PULLUP_EN | RXACTIVE)}, 		/* SPI0_CS0 */
        {OFFSET(cam1_field), (MODE(4) | PULLUP_EN | RXACTIVE)}, 	/* SPI0_CS1 */
        {OFFSET(cam1_pclk), (MODE(4) | PULLUP_EN | RXACTIVE)}, 		/* SPI0_CLK */
        {OFFSET(cam1_data8), (MODE(4) | PULLUP_EN | RXACTIVE)}, 	/* SPI0_D0 */
        {OFFSET(cam1_wen), (MODE(4) | PULLUP_EN | RXACTIVE)}, 		/* SPI0_D1 */
        {-1},
};

/* SPI1 */
static struct module_pin_mux spi4_pin_mux[] = {
        {OFFSET(spi4_cs0), (MODE(0) | PULLUP_EN | RXACTIVE)},	/* SPI0_CS0 */
        {OFFSET(uart3_ctsn), (MODE(2) | PULLUP_EN | RXACTIVE)},	/* SPI0_CS1 */
        {OFFSET(spi4_sclk), (MODE(0) | PULLUP_EN | RXACTIVE)}, 	/* SPI0_CLK */
        {OFFSET(spi4_d0), (MODE(0) | PULLUP_EN | RXACTIVE)}, 	/* SPI0_D0 */
        {OFFSET(spi4_d1), (MODE(0) | PULLUP_EN | RXACTIVE)}, 	/* SPI0_D1 */
        {-1},
};

/* BOOT_SEL */
static struct module_pin_mux boot_sel_pin_mux[] = {
        {OFFSET(gpio5_8), (MODE(7) | PULLUP_EN | RXACTIVE)},    /* BOOT_SEL0, mii1_col.gpio0_0*/
        {OFFSET(gpio5_9), (MODE(7) | PULLUP_EN | RXACTIVE)},    /* BOOT_SEL1, gmpc_be1n.gpio1.28 */
        {OFFSET(gpio5_10), (MODE(7) | PULLUP_EN | RXACTIVE)},   /* BOOT_SEL2, spi2_cs0.gpio0_23 */
        {-1},
};

void enable_uart0_pin_mux(void)
{
	configure_module_pin_mux(uart0_pin_mux);
}

void enable_uart2_pin_mux(void)
{
        configure_module_pin_mux(uart2_pin_mux);
}

void enable_uart3_pin_mux(void)
{
        configure_module_pin_mux(uart3_pin_mux);
}

void enable_uart4_pin_mux(void)
{
        configure_module_pin_mux(uart4_pin_mux);
}

void enable_board_pin_mux(void)
{
	configure_module_pin_mux(i2c1_pin_mux);
       	configure_module_pin_mux(spi0_pin_mux);
	if (board_is_gpevm()) {
		configure_module_pin_mux(gpio5_7_pin_mux);
		configure_module_pin_mux(rgmii1_pin_mux);
#if defined(CONFIG_NAND)
		configure_module_pin_mux(nand_pin_mux);
#endif
	} else if (board_is_sk() || board_is_idk()) {
		configure_module_pin_mux(rgmii1_pin_mux);
#if defined(CONFIG_NAND)
		printf("Error: NAND flash not present on this board\n");
#endif
		configure_module_pin_mux(qspi_pin_mux);
        } else if (board_is_smarc_t437x_800() || board_is_smarc_t437x_01g()) {
        	configure_module_pin_mux(mmc0_pin_mux);
        	configure_module_pin_mux(mmc1_pin_mux);
        	configure_module_pin_mux(i2c0_pin_mux);
        	configure_module_pin_mux(mdio_pin_mux);
        	configure_module_pin_mux(spi2_pin_mux);
                configure_module_pin_mux(spi4_pin_mux);
        	configure_module_pin_mux(rgmii1_pin_mux);
        	configure_module_pin_mux(rgmii2_pin_mux);
        	configure_module_pin_mux(mmc2_pin_mux);
        	configure_module_pin_mux(i2c2_pin_mux);
        	configure_module_pin_mux(smarc_gpio_pin_mux);
        	configure_module_pin_mux(dss_pin_mux);
        	configure_module_pin_mux(boot_sel_pin_mux);
	} else if (board_is_eposevm()) {
		configure_module_pin_mux(rmii1_pin_mux);
#if defined(CONFIG_NAND)
		configure_module_pin_mux(nand_pin_mux);
#else
		configure_module_pin_mux(qspi_pin_mux);
#endif
	}
}

void enable_i2c1_pin_mux(void)
{
	configure_module_pin_mux(i2c1_pin_mux);
}
