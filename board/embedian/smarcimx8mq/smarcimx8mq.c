// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 */

#include <common.h>
#include <env.h>
#include <hang.h>
#include <init.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../../freescale/common/tcpc.h"
#include "../../freescale/common/pfuze.h"
#include <usb.h>
#include <dwc3-uboot.h>

DECLARE_GLOBAL_DATA_PTR;

#define QSPI_PAD_CTRL   (PAD_CTL_DSE2 | PAD_CTL_HYS)

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)

#define WEAK_PULLUP     (PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)

#define I2C_PAD_CTRL    (PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MQ_PAD_GPIO1_IO02__WDOG1_WDOG_B | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

#ifdef CONFIG_CONSOLE_SER3
static iomux_v3_cfg_t const uart1_pads[] = {
        IMX8MQ_PAD_UART1_RXD__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MQ_PAD_UART1_TXD__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

#ifdef CONFIG_CONSOLE_SER2
static iomux_v3_cfg_t const uart2_pads[] = {
        IMX8MQ_PAD_UART2_RXD__UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MQ_PAD_UART2_TXD__UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_UART4_TXD__UART2_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_UART4_RXD__UART2_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),

};
#endif

#ifdef CONFIG_CONSOLE_SER1
static iomux_v3_cfg_t const uart3_pads[] = {
        IMX8MQ_PAD_UART3_RXD__UART3_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MQ_PAD_UART3_TXD__UART3_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

#ifdef CONFIG_CONSOLE_SER0
static iomux_v3_cfg_t const uart4_pads[] = {
	IMX8MQ_PAD_ECSPI2_SCLK__UART4_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MQ_PAD_ECSPI2_SS0__UART4_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
        IMX8MQ_PAD_ECSPI2_MISO__UART4_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_ECSPI2_MOSI__UART4_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));
	set_wdog_reset(wdog);

#ifdef CONFIG_CONSOLE_SER0
        imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
#endif
#ifdef CONFIG_CONSOLE_SER1
        imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
#endif
#ifdef CONFIG_CONSOLE_SER2
        imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
#endif
#ifdef CONFIG_CONSOLE_SER3
        imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
#endif

	return 0;
}

#ifdef CONFIG_FSL_QSPI
int board_qspi_init(void)
{
	set_clk_qspi();

	return 0;
}
#endif

/* SPI0*/
static iomux_v3_cfg_t const ecspi1_pads[] = {
        IMX8MQ_PAD_ECSPI1_MISO__ECSPI1_MISO | MUX_PAD_CTRL(QSPI_PAD_CTRL),
        IMX8MQ_PAD_ECSPI1_MOSI__ECSPI1_MOSI | MUX_PAD_CTRL(QSPI_PAD_CTRL),
        IMX8MQ_PAD_ECSPI1_SCLK__ECSPI1_SCLK | MUX_PAD_CTRL(QSPI_PAD_CTRL),

        IMX8MQ_PAD_ECSPI1_SS0__GPIO5_IO9 | MUX_PAD_CTRL(NO_PAD_CTRL),   /*SS0#*/
        IMX8MQ_PAD_GPIO1_IO00__GPIO1_IO0 | MUX_PAD_CTRL(NO_PAD_CTRL),   /*SS1#*/
        IMX8MQ_PAD_NAND_RE_B__GPIO3_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL),   /*SS2#*/
        IMX8MQ_PAD_NAND_WE_B__GPIO3_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL),   /*SS3#*/
};

/* MISC PINs */
static iomux_v3_cfg_t const misc_pads[] = {
        IMX8MQ_PAD_NAND_CLE__GPIO3_IO5 | MUX_PAD_CTRL(WEAK_PULLUP),             /*S146, PCIE_WAKE*/
        IMX8MQ_PAD_GPIO1_IO09__GPIO1_IO9 | MUX_PAD_CTRL(WEAK_PULLUP),           /*S148, LID#*/
        IMX8MQ_PAD_GPIO1_IO12__GPIO1_IO12 | MUX_PAD_CTRL(WEAK_PULLUP),          /*S149, SLEEP#*/
        IMX8MQ_PAD_NAND_DATA05__GPIO3_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),        
/*S151, CHARGING#*/
        IMX8MQ_PAD_SAI2_RXC__GPIO4_IO22 | MUX_PAD_CTRL(WEAK_PULLUP),            /*S152, CHARGER_PRSNT#*/
        IMX8MQ_PAD_SAI3_MCLK__GPIO5_IO2 | MUX_PAD_CTRL(WEAK_PULLUP),            /*S153, CARRIER_STBY#*/
        IMX8MQ_PAD_SAI2_RXFS__GPIO4_IO21 | MUX_PAD_CTRL(WEAK_PULLUP),           /*S156, BATLOW#*/
        IMX8MQ_PAD_NAND_WP_B__GPIO3_IO18 | MUX_PAD_CTRL(WEAK_PULLUP),           /*CAN0_INT#*/
        IMX8MQ_PAD_NAND_READY_B__GPIO3_IO16 | MUX_PAD_CTRL(WEAK_PULLUP),        /*CAN1_INT#*/
};

static void setup_iomux_misc(void)
{
        imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));

        /* Set CARRIER_LID# as Input*/
        gpio_request(IMX_GPIO_NR(1, 9), "LID#");
        gpio_direction_input(IMX_GPIO_NR(1, 9));
        /* Set CARRIER_SLEEP# as Input*/
        gpio_request(IMX_GPIO_NR(1, 12), "SLEEP#");
        gpio_direction_input(IMX_GPIO_NR(1, 12));
        /* Set CARRIER_CHARGING# as Input*/
        gpio_request(IMX_GPIO_NR(3, 11), "CHARGING#");
        gpio_direction_input(IMX_GPIO_NR(3, 11));
        /* Set CARRIER_CHARGER_PRSNT# as Input*/
        gpio_request(IMX_GPIO_NR(4, 22), "CHARGER_PRSNT#");
        gpio_direction_input(IMX_GPIO_NR(4, 22));
        /* Set CARRIER_STBY# as Output High*/
        gpio_request(IMX_GPIO_NR(5, 02), "CARRIER_STBY#");
        gpio_direction_output(IMX_GPIO_NR(5, 02) , 1);
        /* Set CARRIER_BATLOW# as Input*/
        gpio_request(IMX_GPIO_NR(4, 21), "BATLOW#");
        gpio_direction_input(IMX_GPIO_NR(4, 21));
        /* Set PCIE_WAKE# as Input*/
        gpio_request(IMX_GPIO_NR(3, 5), "PCIE_WAKE#");
        gpio_direction_input(IMX_GPIO_NR(3, 5));
        /* Set CAN0_INT# as Input*/
        gpio_request(IMX_GPIO_NR(3, 18), "CAN0_INT#");
        gpio_direction_input(IMX_GPIO_NR(3, 18));
        /* Set CAN1_INT# as Input*/
        gpio_request(IMX_GPIO_NR(3, 16), "CAN1_INT#");
        gpio_direction_input(IMX_GPIO_NR(3, 16));
}

/* GPIO PINs, By SMARC specification, GPIO0~GPIO5 are recommended set as Output Low by default and GPIO6~GPIO11 are recommended set as Input*/
static iomux_v3_cfg_t const gpio_pads[] = {
        IMX8MQ_PAD_SAI5_MCLK__GPIO3_IO25 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P108, GPIO0*/
        IMX8MQ_PAD_SAI5_RXFS__GPIO3_IO19 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P109, GPIO1*/
        IMX8MQ_PAD_SAI5_RXC__GPIO3_IO20 | MUX_PAD_CTRL(WEAK_PULLUP),            /*P110, GPIO2*/
        IMX8MQ_PAD_SAI5_RXD0__GPIO3_IO21 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P111, GPIO3*/
        IMX8MQ_PAD_SAI5_RXD1__GPIO3_IO22 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P112, GPIO4*/
        IMX8MQ_PAD_SPDIF_TX__GPIO5_IO3 | MUX_PAD_CTRL(WEAK_PULLUP),             /*P113, GPIO5*/
        IMX8MQ_PAD_SPDIF_RX__GPIO5_IO4 | MUX_PAD_CTRL(WEAK_PULLUP),             /*P114, GPIO6*/
        IMX8MQ_PAD_SAI5_RXD2__GPIO3_IO23 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P115, GPIO7*/
        IMX8MQ_PAD_SAI5_RXD3__GPIO3_IO24 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P116, GPIO8*/
        IMX8MQ_PAD_SAI1_TXC__GPIO4_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),            /*P114, GPIO9*/
        IMX8MQ_PAD_SAI1_TXFS__GPIO4_IO10 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P115, GPIO10*/
        IMX8MQ_PAD_SAI1_MCLK__GPIO4_IO20 | MUX_PAD_CTRL(WEAK_PULLUP),           /*P116, GPIO11*/
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

#ifdef CONFIG_SYS_I2C
/*I2C2, I2C_CAM0 and I2C_LCD*/
struct i2c_pads_info i2c_pad_info2 = {
        .scl = {
                .i2c_mode = IMX8MQ_PAD_I2C2_SCL__I2C2_SCL | I2C_PAD_CTRL,
                .gpio_mode = IMX8MQ_PAD_I2C2_SCL__GPIO5_IO16 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 16),
        },
        .sda = {
                .i2c_mode = IMX8MQ_PAD_I2C2_SDA__I2C2_SDA | I2C_PAD_CTRL,
                .gpio_mode = IMX8MQ_PAD_I2C2_SDA__GPIO5_IO17 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 17),
        },
};

/*I2C3, I2C_GP*/
struct i2c_pads_info i2c_pad_info3 = {
        .scl = {
                .i2c_mode = IMX8MQ_PAD_I2C3_SCL__I2C3_SCL | I2C_PAD_CTRL,
                .gpio_mode = IMX8MQ_PAD_I2C3_SCL__GPIO5_IO18 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 18),
        },
        .sda = {
                .i2c_mode = IMX8MQ_PAD_I2C3_SDA__I2C3_SDA | I2C_PAD_CTRL,
                .gpio_mode = IMX8MQ_PAD_I2C3_SDA__GPIO5_IO19 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 19),
        },
};

/*I2C4, I2C_CAM1*/
struct i2c_pads_info i2c_pad_info4 = {
        .scl = {
                .i2c_mode = IMX8MQ_PAD_I2C4_SCL__I2C4_SCL | I2C_PAD_CTRL,
                .gpio_mode = IMX8MQ_PAD_I2C4_SCL__GPIO5_IO20 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 20),
        },
        .sda = {
                .i2c_mode = IMX8MQ_PAD_I2C4_SDA__I2C4_SDA | I2C_PAD_CTRL,
                .gpio_mode = IMX8MQ_PAD_I2C4_SDA__GPIO5_IO21 | I2C_PAD_CTRL,
                .gp = IMX_GPIO_NR(5, 21),
        },
};
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
#define FEC_IRQ_PAD IMX_GPIO_NR(1, 11)

static iomux_v3_cfg_t const fec1_irq_pads[] = {
        IMX8MQ_PAD_GPIO1_IO11__GPIO1_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),
};

static void setup_iomux_fec(void)
{
        imx_iomux_v3_setup_multiple_pads(fec1_irq_pads,
                                         ARRAY_SIZE(fec1_irq_pads));

        gpio_request(IMX_GPIO_NR(1, 11), "fec1_irq");
        gpio_direction_input(IMX_GPIO_NR(1, 11));
}

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

        setup_iomux_fec();

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1],
		IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK, 0);
	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

static void setup_iomux_ecspi1(void)
{
        imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
                                         ARRAY_SIZE(ecspi1_pads));
        init_clk_ecspi(0);
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
        gpio_request(IMX_GPIO_NR(5, 9), "espi1_cs0");
        return (bus == 0 && cs == 0) ? (IMX_GPIO_NR(5, 9)) : -1;
        gpio_request(IMX_GPIO_NR(1, 0), "espi1_cs1");
        return (bus == 0 && cs == 1) ? (IMX_GPIO_NR(1, 0)) : -1;
        gpio_request(IMX_GPIO_NR(3, 15), "espi1_cs2");
        return (bus == 0 && cs == 2) ? (IMX_GPIO_NR(3, 15)) : -1;
        gpio_request(IMX_GPIO_NR(3, 17), "espi1_cs3");
        return (bus == 0 && cs == 3) ? (IMX_GPIO_NR(3, 17)) : -1;
}

#ifdef CONFIG_USB_DWC3

#define USB_PHY_CTRL0			0xF0040
#define USB_PHY_CTRL0_REF_SSP_EN	BIT(2)

#define USB_PHY_CTRL1			0xF0044
#define USB_PHY_CTRL1_RESET		BIT(0)
#define USB_PHY_CTRL1_COMMONONN		BIT(1)
#define USB_PHY_CTRL1_ATERESET		BIT(3)
#define USB_PHY_CTRL1_VDATSRCENB0	BIT(19)
#define USB_PHY_CTRL1_VDATDETENB0	BIT(20)

#define USB_PHY_CTRL2			0xF0048
#define USB_PHY_CTRL2_TXENABLEN0	BIT(8)

static struct dwc3_device dwc3_device_data = {
#ifdef CONFIG_SPL_BUILD
	.maximum_speed = USB_SPEED_HIGH,
#else
	.maximum_speed = USB_SPEED_SUPER,
#endif
	.base = USB1_BASE_ADDR,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
	.power_down_scale = 2,
};

int usb_gadget_handle_interrupts(void)
{
	dwc3_uboot_handle_interrupt(0);
	return 0;
}

static void dwc3_nxp_usb_phy_init(struct dwc3_device *dwc3)
{
	u32 RegData;

	RegData = readl(dwc3->base + USB_PHY_CTRL1);
	RegData &= ~(USB_PHY_CTRL1_VDATSRCENB0 | USB_PHY_CTRL1_VDATDETENB0 |
			USB_PHY_CTRL1_COMMONONN);
	RegData |= USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET;
	writel(RegData, dwc3->base + USB_PHY_CTRL1);

	RegData = readl(dwc3->base + USB_PHY_CTRL0);
	RegData |= USB_PHY_CTRL0_REF_SSP_EN;
	writel(RegData, dwc3->base + USB_PHY_CTRL0);

	RegData = readl(dwc3->base + USB_PHY_CTRL2);
	RegData |= USB_PHY_CTRL2_TXENABLEN0;
	writel(RegData, dwc3->base + USB_PHY_CTRL2);

	RegData = readl(dwc3->base + USB_PHY_CTRL1);
	RegData &= ~(USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET);
	writel(RegData, dwc3->base + USB_PHY_CTRL1);
}
#endif

/*USB Enable Over-Current Pin Setting*/
static iomux_v3_cfg_t const usb_en_oc_pads[] = {
        IMX8MQ_PAD_NAND_DATA04__GPIO3_IO10 | MUX_PAD_CTRL(WEAK_PULLUP),
        IMX8MQ_PAD_NAND_DATA06__GPIO3_IO12 | MUX_PAD_CTRL(WEAK_PULLUP),
        IMX8MQ_PAD_NAND_DATA07__GPIO3_IO13 | MUX_PAD_CTRL(WEAK_PULLUP),
};

static void setup_iomux_usb_en_oc(void)
{
        imx_iomux_v3_setup_multiple_pads(usb_en_oc_pads,
                                         ARRAY_SIZE(usb_en_oc_pads));

        gpio_request(IMX_GPIO_NR(3, 10), "usb0_en_oc#");
        gpio_direction_input(IMX_GPIO_NR(3, 10));
        gpio_request(IMX_GPIO_NR(3, 12), "usb2_en_oc#");
        gpio_direction_input(IMX_GPIO_NR(3, 12));
        gpio_request(IMX_GPIO_NR(3, 13), "usb3_en_oc#");
        gpio_direction_input(IMX_GPIO_NR(3, 13));
}

#ifdef CONFIG_USB_TCPC
struct tcpc_port port;
struct tcpc_port_config port_config = {
	.i2c_bus = 0,
	.addr = 0x50,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 20000,
	.max_snk_ma = 3000,
	.max_snk_mw = 15000,
	.op_snk_mv = 9000,
};

struct gpio_desc type_sel_desc;
static iomux_v3_cfg_t ss_mux_gpio[] = {
	IMX8MQ_PAD_NAND_RE_B__GPIO3_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void ss_mux_select(enum typec_cc_polarity pol)
{
	if (pol == TYPEC_POLARITY_CC1)
		dm_gpio_set_value(&type_sel_desc, 1);
	else
		dm_gpio_set_value(&type_sel_desc, 0);
}

static int setup_typec(void)
{
	int ret;

	imx_iomux_v3_setup_multiple_pads(ss_mux_gpio, ARRAY_SIZE(ss_mux_gpio));

	ret = dm_gpio_lookup_name("GPIO3_15", &type_sel_desc);
	if (ret) {
		printf("%s lookup GPIO3_15 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	ret = dm_gpio_request(&type_sel_desc, "typec_sel");
	if (ret) {
		printf("%s request typec_sel failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	dm_gpio_set_dir_flags(&type_sel_desc, GPIOD_IS_OUT);

	ret = tcpc_init(&port, port_config, &ss_mux_select);
	if (ret) {
		printf("%s: tcpc init failed, err=%d\n",
		       __func__, ret);
	}

	return ret;
}
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

	if (index == 0 && init == USB_INIT_DEVICE) {
		imx8m_usb_power(index, true);
#ifdef CONFIG_USB_TCPC
		ret = tcpc_setup_ufp_mode(&port);
#endif
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
	} else if (index == 0 && init == USB_INIT_HOST) {
#ifdef CONFIG_USB_TCPC
		ret = tcpc_setup_dfp_mode(&port);
#endif
		return ret;
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;
	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_uboot_exit(index);
		imx8m_usb_power(index, false);
	} else if (index == 0 && init == USB_INIT_HOST) {
#ifdef CONFIG_USB_TCPC
		ret = tcpc_disable_src_vbus(&port);
#endif
	}

	return ret;
}
#endif

int board_init(void)
{
        setup_iomux_usb_en_oc();
        setup_iomux_misc();
        setup_iomux_gpio();

#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
	init_usb_clk();
#endif

#ifdef CONFIG_USB_TCPC
	setup_typec();
#endif
	return 0;
}

int board_late_init(void)
{
        setup_iomux_ecspi1();

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
	env_set("board_name", "SMARC-iMX8M");
	env_set("board_rev", "iMX8MQ");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

/* SMARC BOOT_SEL*/
        gpio_request(IMX_GPIO_NR(1, 8), "BOOT_SEL_1");
        gpio_request(IMX_GPIO_NR(1, 5), "BOOT_SEL_2");
        gpio_request(IMX_GPIO_NR(1, 6), "BOOT_SEL_3");
        if ((gpio_get_value(IMX_GPIO_NR(1, 8)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)) {
                puts("BOOT_SEL Detected: OFF OFF OFF, Boot from Carrier SATA is not supported...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 8)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)) {
                puts("BOOT_SEL Detected: OFF OFF ON, Load Image from USB0...\n");
                env_set_ulong("usb dev", 1);
                env_set("bootcmd", "usb start; run loadusbbootenv; run importusbbootenv; run uenvcmd; loadusbimage; run usbboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 8)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)) {
                puts("BOOT_SEL Detected: OFF ON OFF, Boot from Carrier eSPI is not supported...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 8)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)) {
                puts("BOOT_SEL Detected: ON OFF OFF, Load Image from Carrier SD Card...\n");
                env_set_ulong("mmcdev", 1);
                env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadimage; run mmcboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 8)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)) {
                puts("BOOT_SEL Detected: OFF ON ON, Load Image from Module eMMC Flash...\n");
                env_set_ulong("mmcdev", 0);
                env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadimage; run mmcboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 8)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)) {
                puts("BOOT_SEL Detected: ON OFF ON, Load zImage from GBE...\n");
                env_set("bootcmd", "run netboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 8)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)) {
                puts("Carrier SPI Boot is not supported...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 8)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)) {
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
        IMX8MQ_PAD_GPIO1_IO09__GPIO1_IO9 | MUX_PAD_CTRL(LID_PAD_CTRL),
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
	return 0;
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif
