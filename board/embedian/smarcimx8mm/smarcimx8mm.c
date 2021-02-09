/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <asm/mach-imx/dma.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include "../../freescale/common/tcpc.h"
#include <usb.h>
#include <sec_mipi_dsim.h>
#include <imx_mipi_dsi_bridge.h>
#include <mipi_dsi_panel.h>
#include <asm/mach-imx/video.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)
#define WEAK_PULLUP     (PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)
#define I2C_PAD_CTRL    (PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)

#ifdef CONFIG_CONSOLE_SER0
static iomux_v3_cfg_t const uart1_pads[] = {
        IMX8MM_PAD_SAI2_RXFS_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MM_PAD_SAI2_RXC_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MM_PAD_UART3_TXD_UART1_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MM_PAD_UART3_RXD_UART1_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

#ifdef CONFIG_CONSOLE_SER1
static iomux_v3_cfg_t const uart4_pads[] = {
        IMX8MM_PAD_UART4_RXD_UART4_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MM_PAD_UART4_TXD_UART4_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

#ifdef CONFIG_CONSOLE_SER2
static iomux_v3_cfg_t const uart3_pads[] = {
        IMX8MM_PAD_ECSPI1_SCLK_UART3_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MM_PAD_ECSPI1_MOSI_UART3_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MM_PAD_ECSPI1_SS0_UART3_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MM_PAD_ECSPI1_MISO_UART3_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

#ifdef CONFIG_CONSOLE_SER3
static iomux_v3_cfg_t const uart2_pads[] = {
        IMX8MM_PAD_SAI3_TXFS_UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MM_PAD_SAI3_TXC_UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

#ifdef CONFIG_FSL_FSPI
#define QSPI_PAD_CTRL	(PAD_CTL_DSE2 | PAD_CTL_HYS)
static iomux_v3_cfg_t const qspi_pads[] = {
	IMX8MM_PAD_NAND_ALE_QSPI_A_SCLK | MUX_PAD_CTRL(QSPI_PAD_CTRL | PAD_CTL_PE | PAD_CTL_PUE),
	IMX8MM_PAD_NAND_CE0_B_QSPI_A_SS0_B | MUX_PAD_CTRL(QSPI_PAD_CTRL),

	IMX8MM_PAD_NAND_DATA00_QSPI_A_DATA0 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA01_QSPI_A_DATA1 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA02_QSPI_A_DATA2 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA03_QSPI_A_DATA3 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
};

int board_qspi_init(void)
{
	imx_iomux_v3_setup_multiple_pads(qspi_pads, ARRAY_SIZE(qspi_pads));

	set_clk_qspi();

	return 0;
}
#endif

#ifdef CONFIG_MXC_SPI
#define SPI_PAD_CTRL	(PAD_CTL_DSE2 | PAD_CTL_HYS)

/* SPI0 */
static iomux_v3_cfg_t const ecspi2_pads[] = {
	IMX8MM_PAD_ECSPI2_SCLK_ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	IMX8MM_PAD_ECSPI2_MOSI_ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	IMX8MM_PAD_ECSPI2_MISO_ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	IMX8MM_PAD_ECSPI2_SS0_GPIO5_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* SS0 */
        IMX8MM_PAD_GPIO1_IO00_GPIO1_IO0 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* SS1 */
};

/* ESPI */
static iomux_v3_cfg_t const ecspi3_pads[] = {
        IMX8MM_PAD_UART1_RXD_ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
        IMX8MM_PAD_UART1_TXD_ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
        IMX8MM_PAD_UART2_RXD_ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
        IMX8MM_PAD_UART2_TXD_GPIO5_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* SS0 */
        IMX8MM_PAD_NAND_CE1_B_GPIO3_IO2 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* SS1 */
        IMX8MM_PAD_NAND_RE_B_GPIO3_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL),    /* SS2 */
        IMX8MM_PAD_NAND_WE_B_GPIO3_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL),    /* SS3 */
};

static void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
	imx_iomux_v3_setup_multiple_pads(ecspi2_pads, ARRAY_SIZE(ecspi2_pads));
	gpio_request(IMX_GPIO_NR(5, 13), "ECSPI2 CS0");
	gpio_request(IMX_GPIO_NR(1, 00), "ECSPI2 CS1");
        gpio_request(IMX_GPIO_NR(5, 25), "ECSPI3 CS0");
        gpio_request(IMX_GPIO_NR(3, 02), "ECSPI3 CS1");
        gpio_request(IMX_GPIO_NR(3, 15), "ECSPI3 CS2");
        gpio_request(IMX_GPIO_NR(3, 17), "ECSPI3 CS3");
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
        return (bus == 1 && cs == 0) ? (IMX_GPIO_NR(5, 13)) : -1;
        return (bus == 1 && cs == 1) ? (IMX_GPIO_NR(1, 00)) : -1;
        return (bus == 2 && cs == 0) ? (IMX_GPIO_NR(5, 25)) : -1;
        return (bus == 2 && cs == 1) ? (IMX_GPIO_NR(3, 02)) : -1;
        return (bus == 2 && cs == 2) ? (IMX_GPIO_NR(3, 15)) : -1;
        return (bus == 2 && cs == 3) ? (IMX_GPIO_NR(3, 17)) : -1;
}
#endif

#ifdef CONFIG_NAND_MXS
#define NAND_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL2 | PAD_CTL_HYS)
#define NAND_PAD_READY0_CTRL (PAD_CTL_DSE6 | PAD_CTL_FSEL2 | PAD_CTL_PUE)
static iomux_v3_cfg_t const gpmi_pads[] = {
	IMX8MM_PAD_NAND_ALE_RAWNAND_ALE | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_CE0_B_RAWNAND_CE0_B | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_CE1_B_RAWNAND_CE1_B | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_CLE_RAWNAND_CLE | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA00_RAWNAND_DATA00 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA01_RAWNAND_DATA01 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA02_RAWNAND_DATA02 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA03_RAWNAND_DATA03 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA04_RAWNAND_DATA04 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA05_RAWNAND_DATA05	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA06_RAWNAND_DATA06	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA07_RAWNAND_DATA07	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_RE_B_RAWNAND_RE_B | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_READY_B_RAWNAND_READY_B | MUX_PAD_CTRL(NAND_PAD_READY0_CTRL),
	IMX8MM_PAD_NAND_WE_B_RAWNAND_WE_B | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_WP_B_RAWNAND_WP_B | MUX_PAD_CTRL(NAND_PAD_CTRL),
};

static void setup_gpmi_nand(void)
{
	imx_iomux_v3_setup_multiple_pads(gpmi_pads, ARRAY_SIZE(gpmi_pads));
}
#endif

/* MISC PINs */
static iomux_v3_cfg_t const misc_pads[] = {
        IMX8MM_PAD_NAND_CLE_GPIO3_IO5 | MUX_PAD_CTRL(WEAK_PULLUP),             	/*S146, PCIE_WAKE*/
        IMX8MM_PAD_GPIO1_IO09_GPIO1_IO9 | MUX_PAD_CTRL(WEAK_PULLUP),           	/*S148, LID#*/
        IMX8MM_PAD_GPIO1_IO13_GPIO1_IO13 | MUX_PAD_CTRL(WEAK_PULLUP),          	/*S149, SLEEP#*/
        IMX8MM_PAD_GPIO1_IO01_GPIO1_IO1 | MUX_PAD_CTRL(WEAK_PULLUP),           	/*S151, CHARGING#*/
        IMX8MM_PAD_GPIO1_IO12_GPIO1_IO12 | MUX_PAD_CTRL(WEAK_PULLUP),          	/*S152, CHARGER_PRSNT#*/
        IMX8MM_PAD_SAI3_MCLK_GPIO5_IO2 | MUX_PAD_CTRL(WEAK_PULLUP),           	/*S153, CARRIER_STBY#*/
        IMX8MM_PAD_GPIO1_IO08_GPIO1_IO8 | MUX_PAD_CTRL(WEAK_PULLUP),           	/*S156, BATLOW#*/
        IMX8MM_PAD_NAND_WP_B_GPIO3_IO18 | MUX_PAD_CTRL(WEAK_PULLUP),           	/*CAN0_INT#*/
        IMX8MM_PAD_NAND_READY_B_GPIO3_IO16 | MUX_PAD_CTRL(WEAK_PULLUP),        	/*CAN1_INT#*/
        IMX8MM_PAD_NAND_DATA07_GPIO3_IO13 | MUX_PAD_CTRL(WEAK_PULLUP),       	/*LVDS_EN*/
	IMX8MM_PAD_GPIO1_IO05_GPIO1_IO5 | MUX_PAD_CTRL(WEAK_PULLUP),		/*BOOT_SEL0*/
	IMX8MM_PAD_GPIO1_IO06_GPIO1_IO6 | MUX_PAD_CTRL(WEAK_PULLUP),		/*BOOT_SEL1*/
	IMX8MM_PAD_GPIO1_IO07_GPIO1_IO7 | MUX_PAD_CTRL(WEAK_PULLUP),		/*BOOT_SEL2*/
};

static void setup_iomux_misc(void)
{
        imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));

        /* Set LID# as Input*/
        gpio_request(IMX_GPIO_NR(1, 9), "LID#");
        gpio_direction_input(IMX_GPIO_NR(1, 9));
        /* Set SLEEP# as Input*/
        gpio_request(IMX_GPIO_NR(1, 13), "SLEEP#");
        gpio_direction_input(IMX_GPIO_NR(1, 13));
        /* Set CHARGING# as Input*/
        gpio_request(IMX_GPIO_NR(1, 01), "CHARGING#");
        gpio_direction_input(IMX_GPIO_NR(1, 01));
        /* Set CARRIER_CHARGER_PRSNT# as Input*/
        gpio_request(IMX_GPIO_NR(1, 12), "CHARGER_PRSNT#");
        gpio_direction_input(IMX_GPIO_NR(1, 12));
        /* Set CARRIER_STBY# as Output High*/
        gpio_request(IMX_GPIO_NR(5, 02), "CARRIER_STBY#");
        gpio_direction_output(IMX_GPIO_NR(5, 02) , 1);
        /* Set BATLOW# as Input*/
        gpio_request(IMX_GPIO_NR(1, 8), "BATLOW#");
        gpio_direction_input(IMX_GPIO_NR(1, 8));
        /* Set PCIE_WAKE# as Input*/
        gpio_request(IMX_GPIO_NR(3, 5), "PCIE_WAKE#");
        gpio_direction_input(IMX_GPIO_NR(3, 5));
        /* Set CAN0_INT# as Input*/
        gpio_request(IMX_GPIO_NR(3, 18), "CAN0_INT#");
        gpio_direction_input(IMX_GPIO_NR(3, 18));
        /* Set CAN1_INT# as Input*/
        gpio_request(IMX_GPIO_NR(3, 16), "CAN1_INT#");
        gpio_direction_input(IMX_GPIO_NR(3, 16));
        /* Set LVDS_EN Output High*/
        gpio_request(IMX_GPIO_NR(3, 13), "LVDS_EN");
        gpio_direction_input(IMX_GPIO_NR(3, 13));
}

/* GPIO PINs, By SMARC specification, GPIO0~GPIO5 are recommended set as Output Low by default and GPIO6~GPIO11 are recommended set as Input*/
static iomux_v3_cfg_t const gpio_pads[] = {
        IMX8MM_PAD_SAI5_MCLK_GPIO3_IO25 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P108, GPIO0*/
        IMX8MM_PAD_SAI5_RXFS_GPIO3_IO19 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P109, GPIO1*/
        IMX8MM_PAD_SAI5_RXC_GPIO3_IO20 | MUX_PAD_CTRL(WEAK_PULLUP),            /*P110, GPIO2*/
        IMX8MM_PAD_SAI5_RXD0_GPIO3_IO21 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P111, GPIO3*/
        IMX8MM_PAD_SAI5_RXD1_GPIO3_IO22 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P112, GPIO4*/
        IMX8MM_PAD_SPDIF_TX_GPIO5_IO3 | MUX_PAD_CTRL(WEAK_PULLUP),             /*P113, GPIO5*/
        IMX8MM_PAD_SPDIF_RX_GPIO5_IO4 | MUX_PAD_CTRL(WEAK_PULLUP),             /*P114, GPIO6*/
        IMX8MM_PAD_SAI5_RXD2_GPIO3_IO23 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P115, GPIO7*/
        IMX8MM_PAD_SAI5_RXD3_GPIO3_IO24 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P116, GPIO8*/
        IMX8MM_PAD_SAI1_TXC_GPIO4_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),            /*P114, GPIO9*/
        IMX8MM_PAD_SAI1_TXFS_GPIO4_IO10 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P115, GPIO10*/
        IMX8MM_PAD_SAI1_MCLK_GPIO4_IO20 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P116, GPIO11*/
};

static void setup_iomux_gpio(void)
{
        imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));

        /* Set GPIO0 as Output Low*/
        gpio_request(IMX_GPIO_NR(3, 25), "GPIO0");
        gpio_direction_output(IMX_GPIO_NR(3, 25), 0);
        /* Set GPIO1 as Output Low*/
        gpio_request(IMX_GPIO_NR(3, 19), "GPIO1");
        gpio_direction_output(IMX_GPIO_NR(3, 19), 0);
        /* Set GPIO2 as Output Low*/
        gpio_request(IMX_GPIO_NR(3, 20), "GPIO2");
        gpio_direction_output(IMX_GPIO_NR(3, 20), 0);
        /* Set GPIO3 as Output Low*/
        gpio_request(IMX_GPIO_NR(3, 21), "GPIO3");
        gpio_direction_output(IMX_GPIO_NR(3, 21), 0);
        /* Set GPIO4 as Output Low*/
        gpio_request(IMX_GPIO_NR(3, 22), "GPIO4");
        gpio_direction_output(IMX_GPIO_NR(3, 22), 0);
        /* Set GPIO5 as Output Low*/
        gpio_request(IMX_GPIO_NR(5, 3), "GPIO5");
        gpio_direction_output(IMX_GPIO_NR(5, 3), 0);
        /* Set GPIO6 as Input*/
        gpio_request(IMX_GPIO_NR(5, 4), "GPIO6");
        gpio_direction_input(IMX_GPIO_NR(5, 4));
        /* Set GPIO7 as Input*/
        gpio_request(IMX_GPIO_NR(3, 23), "GPIO7");
        gpio_direction_input(IMX_GPIO_NR(3, 23));
        /* Set GPIO8 as Input*/
        gpio_request(IMX_GPIO_NR(3, 24), "GPIO8");
        gpio_direction_input(IMX_GPIO_NR(3, 24));
        /* Set GPIO9 as Input*/
        gpio_request(IMX_GPIO_NR(4, 11), "GPIO9");
        gpio_direction_input(IMX_GPIO_NR(4, 11));
        /* Set GPIO10 as Input*/
        gpio_request(IMX_GPIO_NR(4, 10), "GPIO10");
        gpio_direction_input(IMX_GPIO_NR(4, 10));
        /* Set GPIO11 as Input*/
        gpio_request(IMX_GPIO_NR(4, 20), "GPIO11");
        gpio_direction_input(IMX_GPIO_NR(4, 20));
}

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

#ifdef CONFIG_CONSOLE_SER0
        imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
#endif
#ifdef CONFIG_CONSOLE_SER1
        imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
#endif
#ifdef CONFIG_CONSOLE_SER2
        imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
#endif
#ifdef CONFIG_CONSOLE_SER3
        imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
#endif

#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand(); /* SPL will call the board_early_init_f */
#endif

	return 0;
}

#ifdef CONFIG_BOARD_POSTCLK_INIT
int board_postclk_init(void)
{
	/* TODO */
	return 0;
}
#endif

int dram_init(void)
{
	/* rom_pointer[1] contains the size of TEE occupies */
	if (rom_pointer[1])
		gd->ram_size = PHYS_SDRAM_SIZE - rom_pointer[1];
	else
		gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

#ifdef CONFIG_SYS_I2C
/*I2C2, I2C_CAM0 and I2C_LCD*/
struct i2c_pads_info i2c_pad_info2 = {
        .scl = {
                .i2c_mode = IMX8MM_PAD_I2C2_SCL_I2C2_SCL | I2C_PAD_CTRL,
                .gpio_mode = IMX8MM_PAD_I2C2_SCL_GPIO5_IO16 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 16),
        },
        .sda = {
                .i2c_mode = IMX8MM_PAD_I2C2_SDA_I2C2_SDA | I2C_PAD_CTRL,
                .gpio_mode = IMX8MM_PAD_I2C2_SDA_GPIO5_IO17 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 17),
        },
};

/*I2C3, I2C_GP*/
struct i2c_pads_info i2c_pad_info3 = {
        .scl = {
                .i2c_mode = IMX8MM_PAD_I2C3_SCL_I2C3_SCL | I2C_PAD_CTRL,
                .gpio_mode = IMX8MM_PAD_I2C3_SCL_GPIO5_IO18 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 18),
        },
        .sda = {
                .i2c_mode = IMX8MM_PAD_I2C3_SDA_I2C3_SDA | I2C_PAD_CTRL,
                .gpio_mode = IMX8MM_PAD_I2C3_SDA_GPIO5_IO19 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 19),
        },
};

/*I2C4, I2C_CAM1*/
struct i2c_pads_info i2c_pad_info4 = {
        .scl = {
                .i2c_mode = IMX8MM_PAD_I2C4_SCL_I2C4_SCL | I2C_PAD_CTRL,
                .gpio_mode = IMX8MM_PAD_I2C4_SCL_GPIO5_IO20 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 20),
        },
        .sda = {
                .i2c_mode = IMX8MM_PAD_I2C4_SDA_I2C4_SDA | I2C_PAD_CTRL,
                .gpio_mode = IMX8MM_PAD_I2C4_SDA_GPIO5_IO21 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 21),
        },
};
#endif

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	return 0;
}
#endif

/* Get the top of usable RAM */
ulong board_get_usable_ram_top(ulong total_size)
{

        //printf("board_get_usable_ram_top total_size is 0x%lx \n", total_size);

        if(gd->ram_top > 0x100000000)
           gd->ram_top = 0x100000000;

        return gd->ram_top;
}

#ifdef CONFIG_FEC_MXC
#define FEC_IRQ_PAD IMX_GPIO_NR(3, 04)
static iomux_v3_cfg_t const fec1_irq_pads[] = {
	IMX8MM_PAD_NAND_CE3_B_GPIO3_IO4 | MUX_PAD_CTRL(WEAK_PULLUP),
};

static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec1_irq_pads,
					 ARRAY_SIZE(fec1_irq_pads));

        gpio_request(IMX_GPIO_NR(3, 04), "fec1_irq");
        gpio_direction_input(IMX_GPIO_NR(3, 04));
}

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	setup_iomux_fec();

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
			IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_SHIFT, 0);
	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

/*USB Enable Over-Current and ID Pin Setting*/
static iomux_v3_cfg_t const usb_en_oc_pads[] = {
        IMX8MM_PAD_NAND_DATA04_GPIO3_IO10 | MUX_PAD_CTRL(WEAK_PULLUP),
        IMX8MM_PAD_NAND_DATA05_GPIO3_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),
        IMX8MM_PAD_GPIO1_IO10_GPIO1_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
        IMX8MM_PAD_GPIO1_IO11_GPIO1_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_usb_en_oc(void)
{
        imx_iomux_v3_setup_multiple_pads(usb_en_oc_pads,
                                         ARRAY_SIZE(usb_en_oc_pads));

        gpio_request(IMX_GPIO_NR(3, 10), "usb0_en_oc#");
        gpio_direction_input(IMX_GPIO_NR(3, 10));
        gpio_request(IMX_GPIO_NR(3, 11), "usb1_hub_en_oc#");
        gpio_direction_input(IMX_GPIO_NR(3, 11));
        gpio_request(IMX_GPIO_NR(1, 10), "usb0_id");
        gpio_direction_input(IMX_GPIO_NR(1, 10));
        gpio_request(IMX_GPIO_NR(1, 11), "usb1_id");
        gpio_direction_input(IMX_GPIO_NR(1, 11));
}

#ifdef CONFIG_USB_TCPC
struct tcpc_port port1;
struct tcpc_port port2;

static int setup_pd_switch(uint8_t i2c_bus, uint8_t addr)
{
	struct udevice *bus;
	struct udevice *i2c_dev = NULL;
	int ret;
	uint8_t valb;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return -EINVAL;
	}

	ret = dm_i2c_probe(bus, addr, 0, &i2c_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x\n",
			__func__, addr);
		return -ENODEV;
	}

	ret = dm_i2c_read(i2c_dev, 0xB, &valb, 1);
	if (ret) {
		printf("%s dm_i2c_read failed, err %d\n", __func__, ret);
		return -EIO;
	}
	valb |= 0x4; /* Set DB_EXIT to exit dead battery mode */
	ret = dm_i2c_write(i2c_dev, 0xB, (const uint8_t *)&valb, 1);
	if (ret) {
		printf("%s dm_i2c_write failed, err %d\n", __func__, ret);
		return -EIO;
	}

	/* Set OVP threshold to 23V */
	valb = 0x6;
	ret = dm_i2c_write(i2c_dev, 0x8, (const uint8_t *)&valb, 1);
	if (ret) {
		printf("%s dm_i2c_write failed, err %d\n", __func__, ret);
		return -EIO;
	}

	return 0;
}

int pd_switch_snk_enable(struct tcpc_port *port)
{
	if (port == &port1) {
		debug("Setup pd switch on port 1\n");
		return setup_pd_switch(1, 0x72);
	} else if (port == &port2) {
		debug("Setup pd switch on port 2\n");
		return setup_pd_switch(1, 0x73);
	} else
		return -EINVAL;
}

struct tcpc_port_config port1_config = {
	.i2c_bus = 1, /*i2c2*/
	.addr = 0x50,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 5000,
	.max_snk_ma = 3000,
	.max_snk_mw = 40000,
	.op_snk_mv = 9000,
	.switch_setup_func = &pd_switch_snk_enable,
};

struct tcpc_port_config port2_config = {
	.i2c_bus = 1, /*i2c2*/
	.addr = 0x52,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 5000,
	.max_snk_ma = 3000,
	.max_snk_mw = 40000,
	.op_snk_mv = 9000,
	.switch_setup_func = &pd_switch_snk_enable,
};

static int setup_typec(void)
{
	int ret;

	debug("tcpc_init port 2\n");
	ret = tcpc_init(&port2, port2_config, NULL);
	if (ret) {
		printf("%s: tcpc port2 init failed, err=%d\n",
		       __func__, ret);
	} else if (tcpc_pd_sink_check_charging(&port2)) {
		/* Disable PD for USB1, since USB2 has priority */
		port1_config.disable_pd = true;
		printf("Power supply on USB2\n");
	}

	debug("tcpc_init port 1\n");
	ret = tcpc_init(&port1, port1_config, NULL);
	if (ret) {
		printf("%s: tcpc port1 init failed, err=%d\n",
		       __func__, ret);
	} else {
		if (!port1_config.disable_pd)
			printf("Power supply on USB1\n");
		return ret;
	}

	return ret;
}

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;
	struct tcpc_port *port_ptr;

	debug("board_usb_init %d, type %d\n", index, init);

	if (index == 0)
		port_ptr = &port1;
	else
		port_ptr = &port2;

	imx8m_usb_power(index, true);

	if (init == USB_INIT_HOST)
		tcpc_setup_dfp_mode(port_ptr);
	else
		tcpc_setup_ufp_mode(port_ptr);

	return ret;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;

	debug("board_usb_cleanup %d, type %d\n", index, init);

	if (init == USB_INIT_HOST) {
		if (index == 0)
			ret = tcpc_disable_src_vbus(&port1);
		else
			ret = tcpc_disable_src_vbus(&port2);
	}

	imx8m_usb_power(index, false);
	return ret;
}

int board_ehci_usb_phy_mode(struct udevice *dev)
{
	int ret = 0;
	enum typec_cc_polarity pol;
	enum typec_cc_state state;
	struct tcpc_port *port_ptr;

	if (dev->seq == 0)
		port_ptr = &port1;
	else
		port_ptr = &port2;

	tcpc_setup_ufp_mode(port_ptr);

	ret = tcpc_get_cc_status(port_ptr, &pol, &state);
	if (!ret) {
		if (state == TYPEC_STATE_SRC_RD_RA || state == TYPEC_STATE_SRC_RD)
			return USB_INIT_HOST;
	}

	return USB_INIT_DEVICE;
}

#endif

int board_init(void)
{
#ifdef CONFIG_USB_TCPC
	setup_typec();
#endif

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#ifdef CONFIG_FSL_FSPI
	board_qspi_init();
#endif
        setup_iomux_usb_en_oc();
        setup_iomux_misc();
        setup_iomux_gpio();

	return 0;
}

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int board_late_init(void)
{
        /* Read Module Information from on module EEPROM and pass
         * mac address to kernel
        */
        struct udevice *dev;
        int ret;
        u8 name[8];
        u8 serial[12];
        u8 revision[4];
        u8 mac[6];

        ret = i2c_get_chip_for_busnum(2, 0x50, 2, &dev);
        if (ret) {
                debug("failed to get eeprom\n");
                return 0;
        }

        /* Board ID */
        ret = dm_i2c_read(dev, 0x4, name, 8);
        if (ret) {
                debug("failed to read board ID from EEPROM\n");
                return 0;
        }
        puts("---------Embedian SMARC-iMX8M------------\n");
        printf("  Board ID:             %c%c%c%c%c%c%c%c\n",
               name[0], name[1], name[2], name[3], name[4], name[5], name[6], name[7]);

        /* Board Hardware Revision */
        ret = dm_i2c_read(dev, 0xc, revision, 4);
        if (ret) {
                debug("failed to read hardware revison from EEPROM\n");
                return 0;
        }
        printf("  Hardware Revision:    %c%c%c%c\n",
               revision[0], revision[1], revision[2], revision[3]);

        /* Serial number */
        ret = dm_i2c_read(dev, 0x10, serial, 12);
        if (ret) {
                debug("failed to read srial number from EEPROM\n");
                return 0;
        }
        printf("  Serial Number#:       %c%c%c%c%c%c%c%c%c%c%c%c\n",
               serial[0], serial[1], serial[2], serial[3], serial[4], serial[5], serial[6], serial[7], serial[8], serial[9], serial[10], serial[11]);

        /*MAC address */
        ret = dm_i2c_read(dev, 0x3c, mac, 6);
        if (ret) {
                debug("failed to read eth0 mac address from EEPROM\n");
                return 0;
        }

        if (is_valid_ethaddr(mac))
        printf("  MAC Address:          %02x:%02x:%02x:%02x:%02x:%02x\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                eth_env_set_enetaddr("ethaddr", mac);
        puts("-----------------------------------------\n");

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
        env_set("board_name", "SMARC-iMX8MM");
        env_set("board_rev", "iMX8MM");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

/* SMARC BOOT_SEL*/
        gpio_request(IMX_GPIO_NR(1, 5), "BOOT_SEL_1");
	gpio_direction_input(IMX_GPIO_NR(1, 5));
        gpio_request(IMX_GPIO_NR(1, 6), "BOOT_SEL_2");
	gpio_direction_input(IMX_GPIO_NR(1, 6));
        gpio_request(IMX_GPIO_NR(1, 7), "BOOT_SEL_3");
	gpio_direction_input(IMX_GPIO_NR(1, 7));
        if ((gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 7)) == 0)) {
                puts("BOOT_SEL Detected: OFF OFF OFF, Boot from Carrier SATA is not supported...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 7)) == 1)) {
                puts("BOOT_SEL Detected: OFF OFF ON, Load Image from USB0...\n");
                env_set_ulong("usb dev", 1);
                env_set("bootcmd", "usb start; run loadusbbootenv; run importusbbootenv; run uenvcmd; loadusbimage; run usbboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 7)) == 0)) {
                puts("BOOT_SEL Detected: OFF ON OFF, Boot from Carrier eSPI is not supported...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 7)) == 0)) {
                puts("BOOT_SEL Detected: ON OFF OFF, Load Image from Carrier SD Card...\n");
                env_set_ulong("mmcdev", 1);
                env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadimage; run mmcboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 7)) == 1)) {
                puts("BOOT_SEL Detected: OFF ON ON, Load Image from Module eMMC Flash...\n");
                env_set_ulong("mmcdev", 0);
                env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadimage; run mmcboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 7)) == 1)) {
                puts("BOOT_SEL Detected: ON OFF ON, Load zImage from GBE...\n");
                env_set("bootcmd", "run netboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 7)) == 0)) {
                puts("Carrier SPI Boot is not supported...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 7)) == 1)) {
                puts("BOOT_SEL Detected: ON ON ON, Boot from Module SPI is not supported...\n");
                hang();
        } else {
                puts("unsupported boot devices\n");
                hang();
        }

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
#define LID_KEY IMX_GPIO_NR(1, 9)
#define LID_PAD_CTRL   (PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)

static iomux_v3_cfg_t const lid_pads[] = {
        IMX8MM_PAD_GPIO1_IO09_GPIO1_IO9 | MUX_PAD_CTRL(LID_PAD_CTRL),
};

int is_recovery_key_pressing(void)
{
        imx_iomux_v3_setup_multiple_pads(lid_pads, ARRAY_SIZE(lid_pads));
        gpio_request(LID_KEY, "LID");
        gpio_direction_input(LID_KEY);
        if (gpio_get_value(LID_KEY) == 0) { /* LID key is low assert */
                printf("Recovery key pressed\n");
                return 1;
        }
	return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/
