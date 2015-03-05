/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <phy.h>

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#ifdef CONFIG_SYS_I2C_MXC
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#endif
#include "smarcfimx6.h"
#ifdef CONFIG_CMD_SATA
#include <asm/imx-common/sata.h>
#endif
#ifdef CONFIG_FASTBOOT
#include <fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define EPDC_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define WEAK_PULLUP     (PAD_CTL_PUS_100K_UP |                  \
        PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
        PAD_CTL_SRE_SLOW)

#ifdef CONFIG_SYS_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/*I2C1 I2C_PM*/
struct i2c_pads_info i2c_pad_info1 = {
        .scl = {
                .i2c_mode = MX6_PAD_EIM_D21__I2C1_SCL | PC,
                .gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | PC,
                .gp = IMX_GPIO_NR(3, 21)
        },
        .sda = {
                .i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | PC,
                .gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | PC,
                .gp = IMX_GPIO_NR(3, 28)
        }
};

/* I2C2 HDMI */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

/* I2C3 TCA9546APWR */
struct i2c_pads_info i2c_pad_info3 = {
        .scl = {
                .i2c_mode = MX6_PAD_EIM_D17__I2C3_SCL | PC,
                .gpio_mode = MX6_PAD_EIM_D17__GPIO3_IO17 | PC,
                .gp = IMX_GPIO_NR(3, 17)
        },
        .sda = {
                .i2c_mode = MX6_PAD_EIM_D18__I2C3_SDA | PC,
                .gpio_mode = MX6_PAD_EIM_D18__GPIO3_IO18 | PC,
                .gp = IMX_GPIO_NR(3, 18)
        }
};
#endif

int dram_init(void)
{

	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

/* SER0/UART1 */
iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D20__UART1_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_EIM_D19__UART1_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* SER1/UART2 */
iomux_v3_cfg_t const uart2_pads[] = {
        MX6_PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* SER2/UART4 */
iomux_v3_cfg_t const uart4_pads[] = {
        MX6_PAD_CSI0_DAT12__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_CSI0_DAT13__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_CSI0_DAT16__UART4_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_CSI0_DAT17__UART4_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* SER3/UART5 Debug Port */
iomux_v3_cfg_t const uart5_pads[] = {
        MX6_PAD_CSI0_DAT14__UART5_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_CSI0_DAT15__UART5_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const wdt_pads[] = {
        MX6_PAD_EIM_D16__GPIO3_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	gpio_direction_input(IMX_GPIO_NR(4, 11));
}

/* SDIO */
iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_ENET_TX_EN__GPIO1_IO28	| MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25  | MUX_PAD_CTRL(NO_PAD_CTRL), /* WP */
        MX6_PAD_SD1_CMD__GPIO1_IO18  | MUX_PAD_CTRL(NO_PAD_CTRL), /* SDIO_PWR_EN */
};

/* SDMMC */
iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_RST__SD3_RESET | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

/* eMMC */
iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

#ifdef CONFIG_SYS_USE_SPINOR
/* SPI0 */
iomux_v3_cfg_t const ecspi2_pads[] = {
	MX6_PAD_CSI0_DAT8__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT9__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__GPIO5_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL),	/*SS0#*/
        MX6_PAD_EIM_D24__GPIO3_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),	/*SS2#*/
        MX6_PAD_EIM_D25__GPIO3_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),	/*SS3#*/
};

static void setup_spinor(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi2_pads, ARRAY_SIZE(ecspi2_pads));
	gpio_direction_output(IMX_GPIO_NR(5, 29), 0);
        gpio_direction_output(IMX_GPIO_NR(3, 24), 0);
        gpio_direction_output(IMX_GPIO_NR(3, 25), 0);
}
#endif

/* SPI1 */
	iomux_v3_cfg_t const ecspi1_pads[] = {
        MX6_PAD_KEY_COL0__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_KEY_COL1__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_KEY_ROW0__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_KEY_ROW1__GPIO4_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),	/*SS0#*/
        MX6_PAD_KEY_COL2__GPIO4_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),	/*SS1#*/
};

iomux_v3_cfg_t const pcie_pads[] = {
	MX6_PAD_SD1_DAT1__GPIO1_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* PCIe Present */
	MX6_PAD_SD1_CLK__GPIO1_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* RESET */
        MX6_PAD_SD1_DAT0__GPIO1_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* PCIe Clock Request */
};

static void setup_pcie(void)
{
	imx_iomux_v3_setup_multiple_pads(pcie_pads, ARRAY_SIZE(pcie_pads));
	gpio_direction_input(IMX_GPIO_NR(1, 16));
	gpio_direction_input(IMX_GPIO_NR(1, 17));
        gpio_direction_input(IMX_GPIO_NR(1, 19));
        gpio_direction_output(IMX_GPIO_NR(1, 20), 0);
}

/* CAN0/FLEXCAN1 */
iomux_v3_cfg_t const flexcan1_pads[] = {
        MX6_PAD_GPIO_7__FLEXCAN1_TX | MUX_PAD_CTRL(WEAK_PULLUP),
        MX6_PAD_GPIO_8__FLEXCAN1_RX | MUX_PAD_CTRL(WEAK_PULLUP),
};

/* CAN1/FLEXCAN2 */
iomux_v3_cfg_t const flexcan2_pads[] = {
        MX6_PAD_KEY_COL4__FLEXCAN2_TX | MUX_PAD_CTRL(WEAK_PULLUP),
        MX6_PAD_KEY_ROW4__FLEXCAN2_RX | MUX_PAD_CTRL(WEAK_PULLUP),
};

/* GPIOs */
iomux_v3_cfg_t const gpios_pads[] = {
        MX6_PAD_NANDF_CS0__GPIO6_IO11 | MUX_PAD_CTRL(WEAK_PULLUP), /* GPIO0 */
        MX6_PAD_NANDF_D2__GPIO2_IO02 | MUX_PAD_CTRL(WEAK_PULLUP), /* GPIO1 */
        MX6_PAD_NANDF_D6__GPIO2_IO06 | MUX_PAD_CTRL(WEAK_PULLUP), /* GPIO2 */
        MX6_PAD_NANDF_D3__GPIO2_IO03 | MUX_PAD_CTRL(WEAK_PULLUP), /* GPIO3 */
        MX6_PAD_NANDF_D7__GPIO2_IO07 | MUX_PAD_CTRL(WEAK_PULLUP), /* GPIO4 */
        MX6_PAD_NANDF_CS1__GPIO6_IO14 | MUX_PAD_CTRL(WEAK_PULLUP), /* GPIO6 */
        MX6_PAD_NANDF_CLE__GPIO6_IO07 | MUX_PAD_CTRL(WEAK_PULLUP), /* GPIO7 */
        MX6_PAD_NANDF_D4__GPIO2_IO04 | MUX_PAD_CTRL(WEAK_PULLUP), /* GPIO8 */
        MX6_PAD_NANDF_D0__GPIO2_IO00 | MUX_PAD_CTRL(WEAK_PULLUP), /* GPIO9 */
        MX6_PAD_NANDF_D5__GPIO2_IO05 | MUX_PAD_CTRL(WEAK_PULLUP), /* GPIO10 */
        MX6_PAD_NANDF_ALE__GPIO6_IO08 | MUX_PAD_CTRL(WEAK_PULLUP), /* GPIO11 */
};

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
static iomux_v3_cfg_t const epdc_enable_pads[] = {
	MX6_PAD_EIM_A16__EPDC_DATA00	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA10__EPDC_DATA01	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA12__EPDC_DATA02	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA11__EPDC_DATA03	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_LBA__EPDC_DATA04	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_EB2__EPDC_DATA05	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_CS0__EPDC_DATA06	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_RW__EPDC_DATA07	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_A21__EPDC_GDCLK	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_A22__EPDC_GDSP	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_A23__EPDC_GDOE	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_A24__EPDC_GDRL	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_D31__EPDC_SDCLK_P	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_D27__EPDC_SDOE	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA1__EPDC_SDLE	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_EB1__EPDC_SDSHR	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA2__EPDC_BDR0	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA4__EPDC_SDCE0	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA5__EPDC_SDCE1	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA6__EPDC_SDCE2	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
};

static iomux_v3_cfg_t const epdc_disable_pads[] = {
	MX6_PAD_EIM_A16__GPIO2_IO22,
	MX6_PAD_EIM_DA10__GPIO3_IO10,
	MX6_PAD_EIM_DA12__GPIO3_IO12,
	MX6_PAD_EIM_DA11__GPIO3_IO11,
	MX6_PAD_EIM_LBA__GPIO2_IO27,
	MX6_PAD_EIM_EB2__GPIO2_IO30,
	MX6_PAD_EIM_CS0__GPIO2_IO23,
	MX6_PAD_EIM_RW__GPIO2_IO26,
	MX6_PAD_EIM_A21__GPIO2_IO17,
	MX6_PAD_EIM_A22__GPIO2_IO16,
	MX6_PAD_EIM_A23__GPIO6_IO06,
	MX6_PAD_EIM_A24__GPIO5_IO04,
	MX6_PAD_EIM_D31__GPIO3_IO31,
	MX6_PAD_EIM_D27__GPIO3_IO27,
	MX6_PAD_EIM_DA1__GPIO3_IO01,
	MX6_PAD_EIM_EB1__GPIO2_IO29,
	MX6_PAD_EIM_DA2__GPIO3_IO02,
	MX6_PAD_EIM_DA4__GPIO3_IO04,
	MX6_PAD_EIM_DA5__GPIO3_IO05,
	MX6_PAD_EIM_DA6__GPIO3_IO06,
};
#endif

static void setup_iomux_uart1(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_iomux_uart2(void)
{
        imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
}

static void setup_iomux_uart4(void)
{
        imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

static void setup_iomux_uart5(void)
{
        imx_iomux_v3_setup_multiple_pads(uart5_pads, ARRAY_SIZE(uart5_pads));
}

static void setup_iomux_wdt(void)
{
        imx_iomux_v3_setup_multiple_pads(wdt_pads, ARRAY_SIZE(wdt_pads));

        /* Set HW_WDT as Output High */
        gpio_direction_output(IMX_GPIO_NR(3, 16) , 1);
}

static void setup_spi1(void)
{
        imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
        gpio_direction_output(IMX_GPIO_NR(4, 9), 0);
        gpio_direction_output(IMX_GPIO_NR(4, 10), 0);
}

static void setup_flexcan1(void)
{
        imx_iomux_v3_setup_multiple_pads(flexcan1_pads, ARRAY_SIZE(flexcan1_pads));
}

static void setup_flexcan2(void)
{
        imx_iomux_v3_setup_multiple_pads(flexcan2_pads, ARRAY_SIZE(flexcan2_pads));
}

static void setup_gpios(void)
{
        imx_iomux_v3_setup_multiple_pads(gpios_pads, ARRAY_SIZE(gpios_pads));
        gpio_direction_output(IMX_GPIO_NR(6, 11), 0);
        gpio_direction_output(IMX_GPIO_NR(2, 02), 0);
        gpio_direction_output(IMX_GPIO_NR(2, 06), 0);
        gpio_direction_output(IMX_GPIO_NR(2, 03), 0);
        gpio_direction_output(IMX_GPIO_NR(2, 07), 0);
        gpio_direction_input(IMX_GPIO_NR(6, 14));
        gpio_direction_input(IMX_GPIO_NR(6, 07));
        gpio_direction_input(IMX_GPIO_NR(2, 04));
        gpio_direction_input(IMX_GPIO_NR(2, 00));
        gpio_direction_input(IMX_GPIO_NR(2, 05));
        gpio_direction_input(IMX_GPIO_NR(6, 8));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int mmc_get_env_devno(void)
{
	u32 soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	u32 dev_no;
	u32 bootsel;

	bootsel = (soc_sbmr & 0x000000FF) >> 6 ;

	/* If not boot from sd/mmc, use default value */
	if (bootsel != 1)
		return CONFIG_SYS_MMC_ENV_DEV;

	/* BOOT_CFG2[3] and BOOT_CFG2[4] */
	dev_no = (soc_sbmr & 0x00001800) >> 11;

	/* need ubstract 1 to map to the mmc device id
	 * see the comments in board_mmc_init function
	 */

	dev_no--;

	return dev_no;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no + 1;
}

#define USDHC2_CD_GPIO	IMX_GPIO_NR(1, 28)
/*#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 0)*/

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		/*ret = !gpio_get_value(USDHC3_CD_GPIO);*/
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SDIO
	 * mmc1                    SDMMC
	 * mmc2                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
                        gpio_direction_output(IMX_GPIO_NR(1, 18), 1);
			gpio_direction_input(USDHC2_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			/*gpio_direction_input(USDHC3_CD_GPIO);*/
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		case 2:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
	}

	return status;
}

int check_mmc_autodetect(void)
{
	char *autodetect_str = getenv("mmcautodetect");

	if ((autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0)) {
		return 1;
	}

	return 0;
}

void board_late_mmc_env_init(void)
{
	char cmd[32];
	char mmcblk[32];
	u32 dev_no = mmc_get_env_devno();

	if (!check_mmc_autodetect())
		return;

	setenv_ulong("mmcdev", dev_no);

	/* Set mmcblk env */
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
		mmc_map_to_kernel_blk(dev_no));
	setenv("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}
#endif

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
#ifdef CONFIG_SPLASH_SCREEN
extern int mmc_get_env_devno(void);
int setup_splash_img(void)
{
#ifdef CONFIG_SPLASH_IS_IN_MMC
	int mmc_dev = mmc_get_env_devno();
	ulong offset = CONFIG_SPLASH_IMG_OFFSET;
	ulong size = CONFIG_SPLASH_IMG_SIZE;
	ulong addr = 0;
	char *s = NULL;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	s = getenv("splashimage");

	if (NULL == s) {
		puts("env splashimage not found!\n");
		return -1;
	}
	addr = simple_strtoul(s, NULL, 16);

	if (!mmc) {
		printf("MMC Device %d not found\n", mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
					blk_cnt, (u_char *)addr);
	flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);

	return (n == blk_cnt) ? 0 : -1;
#endif

	return 0;
}
#endif

vidinfo_t panel_info = {
	.vl_refresh = 85,
	.vl_col = 800,
	.vl_row = 600,
	.vl_pixclock = 26666667,
	.vl_left_margin = 8,
	.vl_right_margin = 100,
	.vl_upper_margin = 4,
	.vl_lower_margin = 8,
	.vl_hsync = 4,
	.vl_vsync = 1,
	.vl_sync = 0,
	.vl_mode = 0,
	.vl_flag = 0,
	.vl_bpix = 3,
	.cmap = 0,
};

struct epdc_timing_params panel_timings = {
	.vscan_holdoff = 4,
	.sdoed_width = 10,
	.sdoed_delay = 20,
	.sdoez_width = 10,
	.sdoez_delay = 20,
	.gdclk_hp_offs = 419,
	.gdsp_offs = 20,
	.gdoe_offs = 0,
	.gdclk_offs = 5,
	.num_ce = 1,
};

static void setup_epdc_power(void)
{
	/* Setup epdc voltage */

	/* EIM_A17 - GPIO2[21] for PWR_GOOD status */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_A17__GPIO2_IO21 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));
	/* Set as input */
	gpio_direction_input(IMX_GPIO_NR(2, 21));

	/* EIM_D17 - GPIO3[17] for VCOM control */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_D17__GPIO3_IO17 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/* Set as output */
	gpio_direction_output(IMX_GPIO_NR(3, 17), 1);

	/* EIM_D20 - GPIO3[20] for EPD PMIC WAKEUP */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_D20__GPIO3_IO20 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));
	/* Set as output */
	gpio_direction_output(IMX_GPIO_NR(3, 20), 1);

	/* EIM_A18 - GPIO2[20] for EPD PWR CTL0 */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_A18__GPIO2_IO20 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));
	/* Set as output */
	gpio_direction_output(IMX_GPIO_NR(2, 20), 1);
}

int setup_waveform_file(void)
{
#ifdef CONFIG_WAVEFORM_FILE_IN_MMC
	int mmc_dev = mmc_get_env_devno();
	ulong offset = CONFIG_WAVEFORM_FILE_OFFSET;
	ulong size = CONFIG_WAVEFORM_FILE_SIZE;
	ulong addr = CONFIG_WAVEFORM_BUF_ADDR;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	if (!mmc) {
		printf("MMC Device %d not found\n", mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
				      blk_cnt, (u_char *)addr);
	flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);

	return (n == blk_cnt) ? 0 : -1;
#else
	return -1;
#endif
}

static void epdc_enable_pins(void)
{
        /* epdc iomux settings */
        imx_iomux_v3_setup_multiple_pads(epdc_enable_pads,
                                ARRAY_SIZE(epdc_enable_pads));
}

static void epdc_disable_pins(void)
{
        /* Configure MUX settings for EPDC pins to GPIO */
        imx_iomux_v3_setup_multiple_pads(epdc_disable_pads,
                                ARRAY_SIZE(epdc_disable_pads));
}

static void setup_epdc(void)
{
        unsigned int reg;
        struct mxc_ccm_reg *ccm_regs = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

        /*** epdc Maxim PMIC settings ***/

        /* EPDC PWRSTAT - GPIO2[21] for PWR_GOOD status */
        imx_iomux_v3_setup_pad(MX6_PAD_EIM_A17__GPIO2_IO21 |
                                MUX_PAD_CTRL(EPDC_PAD_CTRL));

        /* EPDC VCOM0 - GPIO3[17] for VCOM control */
        imx_iomux_v3_setup_pad(MX6_PAD_EIM_D17__GPIO3_IO17 |
                                MUX_PAD_CTRL(EPDC_PAD_CTRL));

        /* UART4 TXD - GPIO3[20] for EPD PMIC WAKEUP */
        imx_iomux_v3_setup_pad(MX6_PAD_EIM_D20__GPIO3_IO20 |
                                MUX_PAD_CTRL(EPDC_PAD_CTRL));

        /* EIM_A18 - GPIO2[20] for EPD PWR CTL0 */
        imx_iomux_v3_setup_pad(MX6_PAD_EIM_A18__GPIO2_IO20 |
                                MUX_PAD_CTRL(EPDC_PAD_CTRL));

        /*** Set pixel clock rates for EPDC ***/

        /* EPDC AXI clk (IPU2_CLK) from PFD_400M, set to 396/2 = 198MHz */
        reg = readl(&ccm_regs->cscdr3);
        reg &= ~0x7C000;
        reg |= (1 << 16) | (1 << 14);
        writel(reg, &ccm_regs->cscdr3);

        /* EPDC AXI clk enable */
        reg = readl(&ccm_regs->CCGR3);
        reg |= 0x00C0;
        writel(reg, &ccm_regs->CCGR3);

        /* EPDC PIX clk (IPU2_DI1_CLK) from PLL5, set to 650/4/6 = ~27MHz */
        reg = readl(&ccm_regs->cscdr2);
        reg &= ~0x3FE00;
        reg |= (2 << 15) | (5 << 12);
        writel(reg, &ccm_regs->cscdr2);

        /* PLL5 enable (defaults to 650) */
        reg = readl(&ccm_regs->analog_pll_video);
        reg &= ~((1 << 16) | (1 << 12));
        reg |= (1 << 13);
        writel(reg, &ccm_regs->analog_pll_video);

        /* EPDC PIX clk enable */
        reg = readl(&ccm_regs->CCGR3);
        reg |= 0x0C00;
        writel(reg, &ccm_regs->CCGR3);

        panel_info.epdc_data.working_buf_addr = CONFIG_WORKING_BUF_ADDR;
        panel_info.epdc_data.waveform_buf_addr = CONFIG_WAVEFORM_BUF_ADDR;

        panel_info.epdc_data.wv_modes.mode_init = 0;
        panel_info.epdc_data.wv_modes.mode_du = 1;
        panel_info.epdc_data.wv_modes.mode_gc4 = 3;
        panel_info.epdc_data.wv_modes.mode_gc8 = 2;
        panel_info.epdc_data.wv_modes.mode_gc16 = 2;
        panel_info.epdc_data.wv_modes.mode_gc32 = 2;

        panel_info.epdc_data.epdc_timings = panel_timings;

        setup_epdc_power();

        /* Assign fb_base */
        gd->fb_base = CONFIG_FB_BASE;
}

void epdc_power_on(void)
{
        unsigned int reg;
        struct gpio_regs *gpio_regs = (struct gpio_regs *)GPIO2_BASE_ADDR;

        /* Set EPD_PWR_CTL0 to high - enable EINK_VDD (3.15) */
        gpio_set_value(IMX_GPIO_NR(2, 20), 1);
        udelay(1000);

        /* Enable epdc signal pin */
        epdc_enable_pins();

        /* Set PMIC Wakeup to high - enable Display power */
        gpio_set_value(IMX_GPIO_NR(3, 20), 1);

        /* Wait for PWRGOOD == 1 */
        while (1) {
                reg = readl(&gpio_regs->gpio_psr);
                if (!(reg & (1 << 21)))
                        break;

                udelay(100);
        }

        /* Enable VCOM */
        gpio_set_value(IMX_GPIO_NR(3, 17), 1);

        udelay(500);
}

void epdc_power_off(void)
{
        /* Set PMIC Wakeup to low - disable Display power */
        gpio_set_value(IMX_GPIO_NR(3, 20), 0);

        /* Disable VCOM */
        gpio_set_value(IMX_GPIO_NR(3, 17), 0);

        epdc_disable_pins();

        /* Set EPD_PWR_CTL0 to low - disable EINK_VDD (3.15) */
        gpio_set_value(IMX_GPIO_NR(2, 20), 0);
}
#endif

#if defined(CONFIG_VIDEO_IPUV3)

static iomux_v3_cfg_t const backlight_pads[] = {
        /* Backlight Enable for RGB: S127 */
        MX6_PAD_GPIO_0__GPIO1_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define BACKLIGHT_EN IMX_GPIO_NR(1, 00)
        /* PWM Backlight Control: S141 */
        MX6_PAD_GPIO_1__GPIO1_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define BACKLIGHT_PWM IMX_GPIO_NR(1, 01)
        /* Backlight Enable for LVDS: S127 */
        /*MX6_PAD_GPIO_0__GPIO1_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define LVDS_BACKLIGHT_EN IMX_GPIO_NR(1, 00)*/
        /* LCD VDD Enable: S133 */
        MX6_PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define LCD_VDD_EN IMX_GPIO_NR(1, 02)
};

static iomux_v3_cfg_t const rgb_pads[] = {
        MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
        MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15,
        MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,
        MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,
        MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15, /* DISP0_DRDY */
        MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00,
        MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01,
        MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02,
        MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03,
        MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04,
        MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05,
        MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06,
        MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07,
        MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08,
        MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09,
        MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10,
        MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11,
        MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12,
        MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13,
        MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14,
        MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15,
        MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16,
        MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17,
        MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18,
        MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19,
        MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20,
        MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21,
        MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22,
        MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23,
};

struct display_info_t {
        int     bus;
        int     addr;
        int     pixfmt;
        int     (*detect)(struct display_info_t const *dev);
        void    (*enable)(struct display_info_t const *dev);
        struct  fb_videomode mode;
};


static int detect_hdmi(struct display_info_t const *dev)
{
        struct hdmi_regs *hdmi  = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
        return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
        imx_enable_hdmi_phy();
}

static int detect_i2c(struct display_info_t const *dev)
{
        return ((0 == i2c_set_bus_num(dev->bus))
                &&
                (0 == i2c_probe(dev->addr)));
}

static void enable_lvds(struct display_info_t const *dev)
{
        struct iomuxc *iomux = (struct iomuxc *)
                                IOMUXC_BASE_ADDR;
        u32 reg = readl(&iomux->gpr[2]);
        reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
        writel(reg, &iomux->gpr[2]);
        gpio_direction_output(BACKLIGHT_EN, 1);
}

static void enable_rgb(struct display_info_t const *dev)
{
        imx_iomux_v3_setup_multiple_pads(
                rgb_pads,
                 ARRAY_SIZE(rgb_pads));
        gpio_direction_output(BACKLIGHT_EN, 1);
	gpio_direction_output(LCD_VDD_EN, 1);
	gpio_direction_output(BACKLIGHT_PWM, 1);
}

static struct display_info_t const displays[] = {{
        .bus    = -1,
        .addr   = 0,
        .pixfmt = IPU_PIX_FMT_RGB24,
        .detect = detect_hdmi,
        .enable = do_enable_hdmi,
        .mode   = {
                .name           = "HDMI",
                .refresh        = 60,
                .xres           = 1024,
                .yres           = 768,
                .pixclock       = 15385,
                .left_margin    = 220,
                .right_margin   = 40,
                .upper_margin   = 21,
                .lower_margin   = 7,
                .hsync_len      = 60,
                .vsync_len      = 10,
                .sync           = FB_SYNC_EXT,
                .vmode          = FB_VMODE_NONINTERLACED
} }, {
        .bus    = 3,
        .addr   = 0x4,
        .pixfmt = IPU_PIX_FMT_LVDS666,
        .detect = detect_i2c,
        .enable = enable_lvds,
        .mode   = {
                .name           = "Hannstar-XGA",
                .refresh        = 60,
                .xres           = 1024,
                .yres           = 768,
                .pixclock       = 15385,
                .left_margin    = 220,
                .right_margin   = 40,
                .upper_margin   = 21,
                .lower_margin   = 7,
                .hsync_len      = 60,
                .vsync_len      = 10,
                .sync           = FB_SYNC_EXT,
                .vmode          = FB_VMODE_NONINTERLACED
} }, {
        .bus    = 3,
        .addr   = 0x38,
        .pixfmt = IPU_PIX_FMT_LVDS666,
        .detect = detect_i2c,
        .enable = enable_lvds,
        .mode   = {
                .name           = "wsvga-lvds",
                .refresh        = 60,
                .xres           = 1024,
                .yres           = 600,
                .pixclock       = 15385,
                .left_margin    = 220,
                .right_margin   = 40,
                .upper_margin   = 21,
                .lower_margin   = 7,
                .hsync_len      = 60,
                .vsync_len      = 10,
                .sync           = FB_SYNC_EXT,
                .vmode          = FB_VMODE_NONINTERLACED
} }, {
        .bus    = 3,
        .addr   = 0x48,
        .pixfmt = IPU_PIX_FMT_RGB666,
        .detect = detect_i2c,
        .enable = enable_rgb,
        .mode   = {
                .name           = "wvga-rgb",
                .refresh        = 57,
                .xres           = 800,
                .yres           = 480,
                .pixclock       = 37037,
                .left_margin    = 40,
                .right_margin   = 60,
                .upper_margin   = 10,
                .lower_margin   = 10,
                .hsync_len      = 20,
                .vsync_len      = 10,
                .sync           = 0,
                .vmode          = FB_VMODE_NONINTERLACED
} } };
int board_video_skip(void)
{
        int i;
        int ret;
        char const *panel = getenv("panel");
        if (!panel) {
                for (i = 0; i < ARRAY_SIZE(displays); i++) {
                        struct display_info_t const *dev = displays+i;
                        if (dev->detect(dev)) {
                                panel = dev->mode.name;
                                printf("auto-detected panel %s\n", panel);
                                break;
                        }
                }
                if (!panel) {
                        panel = displays[0].mode.name;
                        printf("No panel detected: default to %s\n", panel);
                        i = 0;
                }
        } else {
                for (i = 0; i < ARRAY_SIZE(displays); i++) {
                        if (!strcmp(panel, displays[i].mode.name))
                                break;
                }
        }
        if (i < ARRAY_SIZE(displays)) {
                ret = ipuv3_fb_init(&displays[i].mode, 0,
                                    displays[i].pixfmt);
                if (!ret) {
                        displays[i].enable(displays+i);
                        printf("Display: %s (%ux%u)\n",
                               displays[i].mode.name,
                               displays[i].mode.xres,
                               displays[i].mode.yres);
                } else {
                        printf("LCD %s cannot be configured: %d\n",
                               displays[i].mode.name, ret);
                }
        } else {
                printf("unsupported panel %s\n", panel);
                ret = -EINVAL;
        }
        return (0 != ret);
}

static void setup_display(void)
{
        struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
        struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
        int reg;

        enable_ipu_clock();
        imx_setup_hdmi();
        /* Turn on LDB0,IPU,IPU DI0 clocks */
        reg = __raw_readl(&mxc_ccm->CCGR3);
        reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
        writel(reg, &mxc_ccm->CCGR3);

        /* set LDB0, LDB1 clk select to 011/011 */
        reg = readl(&mxc_ccm->cs2cdr);
        reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
                 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
        reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
              |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
        writel(reg, &mxc_ccm->cs2cdr);

        reg = readl(&mxc_ccm->cscmr2);
        reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
        writel(reg, &mxc_ccm->cscmr2);

        reg = readl(&mxc_ccm->chsccdr);
        reg |= (CHSCCDR_CLK_SEL_LDB_DI0
                <<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
        writel(reg, &mxc_ccm->chsccdr);

        reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
             |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
             |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
             |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
             |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
             |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
             |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
             |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
             |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
        writel(reg, &iomux->gpr[2]);

        reg = readl(&iomux->gpr[3]);
        reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
                        |IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
            | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
               <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
        writel(reg, &iomux->gpr[3]);

        /* backlights off until needed */
        /*imx_iomux_v3_setup_multiple_pads(backlight_pads,
                                         ARRAY_SIZE(backlight_pads));
        gpio_direction_input(BACKLIGHT_EN);*/
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
#if defined(CONFIG_MAC_ADDR_IN_EEPROM)

        uchar env_enetaddr[6];
        int enetaddr_found;

        enetaddr_found = eth_getenv_enetaddr("ethaddr", env_enetaddr);

        uint8_t enetaddr[8];
        int eeprom_mac_read;

        /* Read Ethernet MAC address from EEPROM */
        eeprom_mac_read = smarcfimx6_read_mac_address(enetaddr);

        /*
         * MAC address not present in the environment
         * try and read the MAC address from EEPROM flash
         * and set it.
         */
        if (!enetaddr_found) {
                if (eeprom_mac_read)
                        /* Set Ethernet MAC address from EEPROM */
                        smarcfimx6_sync_env_enetaddr(enetaddr);
        } else {
                /*
                 * MAC address present in environment compare it with
                 * the MAC address in EEPROM and warn on mismatch
                 */
                if (eeprom_mac_read && memcmp(enetaddr, env_enetaddr, 6))
                        printf("Warning: MAC address in EEPROM don't match "
                                        "with the MAC address in the environment\n");
                        printf("Default using MAC address from environment\n");
        }

#endif

	setup_iomux_enet();
	setup_pcie();

	return cpu_eth_init(bis);
}

int board_early_init_f(void)
{
        setup_iomux_wdt();
        setup_iomux_uart1();
        setup_iomux_uart2();
        setup_iomux_uart4();
        setup_iomux_uart5();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

#ifdef CONFIG_SYS_USE_SPINOR
	setup_spinor();
#endif

        setup_spi1();
	setup_flexcan1();
	setup_flexcan2();
	setup_gpios();

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
	setup_epdc();
#endif

#ifdef CONFIG_SYS_I2C_MXC
        setup_i2c(0, CONFIG_SYS_I2C_SPEED,
                        0x50, &i2c_pad_info1);
        setup_i2c(2, CONFIG_SYS_I2C_SPEED,
                        0x70, &i2c_pad_info3);

        /* Configure I2C switch (PCA9546) to enable channel 0. */
	i2c_set_bus_num(2);
	uint8_t i2cbuf;
        i2cbuf = CONFIG_SYS_I2C_PCA9546_ENABLE;
        if (i2c_write(CONFIG_SYS_I2C_PCA9546_ADDR, 0,
                      CONFIG_SYS_I2C_PCA9546_ADDR_LEN, &i2cbuf, 1)) {
                printf("Write to MUX @ 0x%02x failed\n", CONFIG_SYS_I2C_PCA9546_ADDR);
                return 1;
        }

#endif
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
       // Make sure we enable ECSPI2 clock
       int reg;
       struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
       reg = readl(&mxc_ccm->CCGR1);
       reg |=  MXC_CCM_CCGR1_ECSPI2S_MASK;
       writel(reg, &mxc_ccm->CCGR1);
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: SMARC-FiMX6 Rev.00A0\n");
	return 0;
}

#ifdef CONFIG_FASTBOOT

void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "sata");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "booti sata");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "booti mmc0");
	    break;
	case SD3_BOOT:
	case MMC3_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "booti mmc1");
	    break;
	case MMC4_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc2");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "booti mmc2");
	    break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("unsupported boot devices\n");
		break;
	}

}

#ifdef CONFIG_ANDROID_RECOVERY

#define GPIO_VOL_DN_KEY IMX_GPIO_NR(1, 5)
iomux_v3_cfg_t const recovery_key_pads[] = {
	(MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int check_recovery_cmd_file(void)
{
    int button_pressed = 0;
    int recovery_mode = 0;

    recovery_mode = recovery_check_and_clean_flag();

    /* Check Recovery Combo Button press or not. */
	imx_iomux_v3_setup_multiple_pads(recovery_key_pads,
			ARRAY_SIZE(recovery_key_pads));

    gpio_direction_input(GPIO_VOL_DN_KEY);

    if (gpio_get_value(GPIO_VOL_DN_KEY) == 0) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
    }

    return recovery_mode || button_pressed;
}

void board_recovery_setup(void)
{
	int bootdev = get_boot_device();

	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"booti sata recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"booti mmc0 recovery");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"booti mmc1 recovery");
		break;
	case MMC4_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"booti mmc2 recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("Unsupported bootup device for recovery: dev: %d\n",
			bootdev);
		return;
	}

	printf("setup env for recovery..\n");
	setenv("bootcmd", "run bootcmd_android_recovery");
}

#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FASTBOOT*/

#ifdef CONFIG_IMX_UDC
iomux_v3_cfg_t const otg_udc_pads[] = {
	(MX6_PAD_ENET_RX_ER__USB_OTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
};
void udc_pins_setting(void)
{
	imx_iomux_v3_setup_multiple_pads(otg_udc_pads,
			ARRAY_SIZE(otg_udc_pads));

	/*set daisy chain for otg_pin_id on 6q. for 6dl, this bit is reserved*/
    mxc_iomux_set_gpr_register(1, 13, 1, 0);
}
#endif /*CONFIG_IMX_UDC*/

#ifdef CONFIG_USB_EHCI_MX6
iomux_v3_cfg_t const usb_otg_pads[] = {
	MX6_PAD_EIM_D22__USB_OTG_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_ENET_RX_ER__USB_OTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const usb_hc1_pads[] = {
	MX6_PAD_ENET_TXD1__GPIO1_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int board_ehci_hcd_init(int port)
{
	switch (port) {
	case 0:
		imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
			ARRAY_SIZE(usb_otg_pads));

		/*set daisy chain for otg_pin_id on 6q. for 6dl, this bit is reserved*/
		mxc_iomux_set_gpr_register(1, 13, 1, 0);
		break;
	case 1:
		imx_iomux_v3_setup_multiple_pads(usb_hc1_pads,
			ARRAY_SIZE(usb_hc1_pads));
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}
	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		if (on){
			gpio_direction_output(IMX_GPIO_NR(1, 29), 1);
                        gpio_direction_input(IMX_GPIO_NR(1, 30));
			}
		else
			gpio_direction_output(IMX_GPIO_NR(1, 29), 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}
	return 0;
}
#endif
