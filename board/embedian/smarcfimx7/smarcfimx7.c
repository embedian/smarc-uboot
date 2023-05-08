// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#include <init.h>
#include <net.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx7-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/io.h>
#include <hang.h>
#include <linux/delay.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <miiphy.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include "../../freescale/common/pfuze.h"
#include <i2c.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/crm_regs.h>
#if defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#include <asm/mach-imx/video.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_DSE_3P3V_49OHM | \
	PAD_CTL_PUS_PU100KOHM | PAD_CTL_HYS)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_PU100KOHM | \
	PAD_CTL_DSE_3P3V_49OHM)

#define NAND_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_SLOW | PAD_CTL_HYS)

#define SPI_PAD_CTRL \
	(PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define NAND_PAD_READY0_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUS_PU5KOHM)

#define EPDC_PAD_CTRL	0x0

#define WEAK_PULLUP (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM | \
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#ifdef CONFIG_MXC_SPI
/* SPI2 (SPINOR) */
static iomux_v3_cfg_t const ecspi2_pads[] = {
	MX7D_PAD_ECSPI2_SCLK__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX7D_PAD_ECSPI2_MOSI__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX7D_PAD_ECSPI2_MISO__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX7D_PAD_ECSPI2_SS0__GPIO4_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL),	/*SS0#*/
};

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	gpio_request(IMX_GPIO_NR(4, 23), "espi2_cs0");
	return (bus == 1 && cs == 0) ? (IMX_GPIO_NR(4, 23)) : -1;
}

static void setup_spinor(void)
{
         imx_iomux_v3_setup_multiple_pads(ecspi2_pads, ARRAY_SIZE(ecspi2_pads));
}
#endif

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

static iomux_v3_cfg_t const wdog_pads[] = {
	MX7D_PAD_GPIO1_IO00__WDOG1_WDOG_B | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/* SER0/UART6 */
#ifdef CONFIG_CONSOLE_SER0
static iomux_v3_cfg_t const uart6_pads[] = {
	MX7D_PAD_EPDC_DATA09__UART6_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_EPDC_DATA08__UART6_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_EPDC_DATA10__UART6_DTE_CTS | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_EPDC_DATA11__UART6_DTE_RTS | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

/* SER1/UART2 */
#ifdef CONFIG_CONSOLE_SER1
static iomux_v3_cfg_t const uart2_pads[] = {
	MX7D_PAD_UART2_TX_DATA__UART2_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_UART2_RX_DATA__UART2_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

/* SER2/UART7 */
#ifdef CONFIG_CONSOLE_SER2
static iomux_v3_cfg_t const uart7_pads[] = {
	MX7D_PAD_EPDC_DATA13__UART7_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_EPDC_DATA12__UART7_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_EPDC_DATA14__UART7_DTE_CTS | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_EPDC_DATA15__UART7_DTE_RTS | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

/* SER3/UART3 Debug Port */
#ifdef CONFIG_CONSOLE_SER3
static iomux_v3_cfg_t const uart3_pads[] = {
	MX7D_PAD_UART3_TX_DATA__UART3_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_UART3_RX_DATA__UART3_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

/* RESET_OUT# */
static iomux_v3_cfg_t const reset_out_pads[] = {
	MX7D_PAD_EPDC_BDR1__GPIO2_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/* SPI0 */
static iomux_v3_cfg_t const ecspi1_pads[] = {
	MX7D_PAD_ECSPI1_SCLK__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX7D_PAD_ECSPI1_MOSI__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX7D_PAD_ECSPI1_MISO__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX7D_PAD_ECSPI1_SS0__ECSPI1_SS0 | MUX_PAD_CTRL(NO_PAD_CTRL),	/*SS0#*/
	MX7D_PAD_UART1_RX_DATA__ECSPI1_SS1 | MUX_PAD_CTRL(NO_PAD_CTRL),	/*SS1#*/
};

/* ESPI */
static iomux_v3_cfg_t const ecspi3_pads[] = {
	MX7D_PAD_SAI2_RX_DATA__ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX7D_PAD_SAI2_TX_BCLK__ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX7D_PAD_SAI2_TX_SYNC__ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX7D_PAD_SAI2_TX_DATA__ECSPI3_SS0 | MUX_PAD_CTRL(NO_PAD_CTRL),  /*SS0#*/
	MX7D_PAD_SD2_CD_B__ECSPI3_SS2 | MUX_PAD_CTRL(NO_PAD_CTRL),      /*SS2#*/
};

/* CAN0/FLEXCAN1 */
static iomux_v3_cfg_t const flexcan1_pads[] = {
	MX7D_PAD_GPIO1_IO13__FLEXCAN1_TX | MUX_PAD_CTRL(WEAK_PULLUP),
	MX7D_PAD_GPIO1_IO12__FLEXCAN1_RX | MUX_PAD_CTRL(WEAK_PULLUP),
};

/* CAN1/FLEXCAN2 */
static iomux_v3_cfg_t const flexcan2_pads[] = {
	MX7D_PAD_GPIO1_IO15__FLEXCAN2_TX | MUX_PAD_CTRL(WEAK_PULLUP),
	MX7D_PAD_GPIO1_IO14__FLEXCAN2_RX | MUX_PAD_CTRL(WEAK_PULLUP),
};

/* GPIOs */
static iomux_v3_cfg_t const gpios_pads[] = {
	MX7D_PAD_EPDC_DATA00__GPIO2_IO0 | MUX_PAD_CTRL(WEAK_PULLUP),	/* GPIO0 */
	MX7D_PAD_EPDC_DATA01__GPIO2_IO1 | MUX_PAD_CTRL(WEAK_PULLUP),	/* GPIO1 */
	MX7D_PAD_EPDC_DATA02__GPIO2_IO2 | MUX_PAD_CTRL(WEAK_PULLUP),	/* GPIO2 */
	MX7D_PAD_EPDC_DATA03__GPIO2_IO3 | MUX_PAD_CTRL(WEAK_PULLUP),	/* GPIO3 */
	MX7D_PAD_EPDC_DATA04__GPIO2_IO4 | MUX_PAD_CTRL(WEAK_PULLUP),	/* GPIO4 */
	MX7D_PAD_EPDC_DATA05__GPIO2_IO5 | MUX_PAD_CTRL(WEAK_PULLUP),	/* GPIO6 */
	MX7D_PAD_EPDC_DATA07__GPIO2_IO7 | MUX_PAD_CTRL(WEAK_PULLUP),	/* GPIO7 */
	MX7D_PAD_EPDC_DATA06__GPIO2_IO6 | MUX_PAD_CTRL(WEAK_PULLUP),	/* GPIO8 */
	MX7D_PAD_UART1_TX_DATA__GPIO4_IO1 | MUX_PAD_CTRL(WEAK_PULLUP),	/* GPIO9 */
	MX7D_PAD_UART3_RTS_B__GPIO4_IO6 | MUX_PAD_CTRL(WEAK_PULLUP),	/* GPIO10 */
	MX7D_PAD_UART3_CTS_B__GPIO4_IO7 | MUX_PAD_CTRL(WEAK_PULLUP),	/* GPIO11 */
};

/* LVDS channel selection, set low as single channel LVDS and high as dual channel LVDS */
static iomux_v3_cfg_t const lvds_ch_sel_pads[] = {
        MX7D_PAD_SD2_CMD__GPIO5_IO13 | MUX_PAD_CTRL(WEAK_PULLUP),       /* LVDS_CH_SEL */
};

/* Misc. pins */
static iomux_v3_cfg_t const misc_pads[] = {
	MX7D_PAD_SD2_DATA0__GPIO5_IO14 | MUX_PAD_CTRL(WEAK_PULLUP),	/* SLEEP# */
	MX7D_PAD_GPIO1_IO09__GPIO1_IO9 | MUX_PAD_CTRL(WEAK_PULLUP),	/* CHARGER_PRSNT# */
	MX7D_PAD_GPIO1_IO08__GPIO1_IO8 | MUX_PAD_CTRL(WEAK_PULLUP),	/* CHARGING# */
	MX7D_PAD_SAI1_RX_SYNC__GPIO6_IO16 | MUX_PAD_CTRL(WEAK_PULLUP),	/* CARRIER_STBY# */
	MX7D_PAD_SD2_RESET_B__GPIO5_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),	/* BATLOW# */
	MX7D_PAD_EPDC_BDR0__GPIO2_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* PCIe_RST# */
	MX7D_PAD_EPDC_PWR_STAT__GPIO2_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* PCIe_WAKE# */
	MX7D_PAD_ENET1_CRS__GPIO7_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* WDT_TIME_OUT# */
};

#ifdef CONFIG_NAND_MXS
static iomux_v3_cfg_t const gpmi_pads[] = {
	MX7D_PAD_SD3_DATA0__NAND_DATA00 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA1__NAND_DATA01 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA2__NAND_DATA02 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA3__NAND_DATA03 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA4__NAND_DATA04 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA5__NAND_DATA05 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA6__NAND_DATA06 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA7__NAND_DATA07 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_CLK__NAND_CLE	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_CMD__NAND_ALE	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_STROBE__NAND_RE_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_RESET_B__NAND_WE_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_MCLK__NAND_WP_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_RX_BCLK__NAND_CE3_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_RX_SYNC__NAND_CE2_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_RX_DATA__NAND_CE1_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_TX_BCLK__NAND_CE0_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_TX_SYNC__NAND_DQS	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_TX_DATA__NAND_READY_B	| MUX_PAD_CTRL(NAND_PAD_READY0_CTRL),
};

static void setup_gpmi_nand(void)
{
	imx_iomux_v3_setup_multiple_pads(gpmi_pads, ARRAY_SIZE(gpmi_pads));

	/* NAND_USDHC_BUS_CLK is set in rom */
	set_clk_nand();
}
#endif

#ifdef CONFIG_DM_VIDEO
static iomux_v3_cfg_t const lcd_pads[] = {
	MX7D_PAD_LCD_RESET__GPIO3_IO4	| MUX_PAD_CTRL(LCD_PAD_CTRL),
};

static iomux_v3_cfg_t const backlight_pads[] = {
	/* Backlight Enable for RGB: S127 */
	MX7D_PAD_SAI1_RX_BCLK__GPIO6_IO17 | MUX_PAD_CTRL(WEAK_PULLUP),

	/* PWM Backlight Control: S141. Use GPIO for Brightness adjustment, duty cycle = period */
	MX7D_PAD_GPIO1_IO02__GPIO1_IO2 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static int setup_lcd(void)
{
	int ret;
	struct gpio_desc desc;

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	imx_iomux_v3_setup_multiple_pads(backlight_pads, ARRAY_SIZE(backlight_pads));

	/* Reset LCD */
	ret = dm_gpio_lookup_name("GPIO3_4", &desc);
	if (ret) {
		printf("%s lookup GPIO3_4 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	ret = dm_gpio_request(&desc, "lcd reset");
	if (ret) {
		printf("%s request lcd reset failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);
	udelay(500);
	dm_gpio_set_value(&desc, 1);

	/* Set Brightness to high */
	ret = dm_gpio_lookup_name("GPIO6_17", &desc);
	if (ret) {
		printf("%s lookup GPIO6_17 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	ret = dm_gpio_request(&desc, "backlight_enable");
	if (ret) {
		printf("%s request backlight_enable failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

        ret = dm_gpio_lookup_name("GPIO1_2", &desc);
        if (ret) {
                printf("%s lookup GPIO1_2 failed ret = %d\n", __func__, ret);
                return -ENODEV;
        }

        ret = dm_gpio_request(&desc, "backlight_pwm");
        if (ret) {
                printf("%s request backlight_pwm failed ret = %d\n", __func__, ret);
                return -ENODEV;
        }

        dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

	return 0;
}
#else
static inline int setup_lcd(void) { return 0; }
#endif

#ifdef CONFIG_CONSOLE_SER0
static void setup_iomux_uart6(void)
{
	imx_iomux_v3_setup_multiple_pads(uart6_pads, ARRAY_SIZE(uart6_pads));
}
#endif

#ifdef CONFIG_CONSOLE_SER1
static void setup_iomux_uart2(void)
{
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
}
#endif

#ifdef CONFIG_CONSOLE_SER2
static void setup_iomux_uart7(void)
{
	imx_iomux_v3_setup_multiple_pads(uart7_pads, ARRAY_SIZE(uart7_pads));
}
#endif

#ifdef CONFIG_CONSOLE_SER3
static void setup_iomux_uart3(void)
{
	imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
}
#endif

static void setup_iomux_reset_out(void)
{
	int ret;
	struct gpio_desc desc;

	imx_iomux_v3_setup_multiple_pads(reset_out_pads, ARRAY_SIZE(reset_out_pads));

	/* Set CPU RESET_OUT as Output */
	ret = dm_gpio_lookup_name("GPIO2_29", &desc);
	if (ret) {
		printf("%s lookup GPIO2_29 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "reset_out");
	if (ret) {
		printf("%s request reset_out failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);
}

static void setup_iomux_spi1(void)
{
	int ret;
	struct gpio_desc desc;

	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
	/* SPI0 CS0 */
	ret = dm_gpio_lookup_name("GPIO4_19", &desc);
	if (ret) {
		printf("%s lookup GPIO4_19 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "spi1_cs0");
	if (ret) {
		printf("%s request spi1_cs0 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* SPI0 CS1 */
	ret = dm_gpio_lookup_name("GPIO4_0", &desc);
	if (ret) {
		printf("%s lookup GPIO4_0 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "spi1_cs1");
	if (ret) {
		printf("%s request spi1_cs1 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);
}

static void setup_iomux_spi3(void)
{
	int ret;
	struct gpio_desc desc;

	imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
	/* ESPI CS0 */
        ret = dm_gpio_lookup_name("GPIO6_22", &desc);
        if (ret) {
                printf("%s lookup GPIO6_22 failed ret = %d\n", __func__, ret);
                return;
        }

	ret = dm_gpio_request(&desc, "spi3_cs0");
	if (ret) {
		printf("%s request spi3_cs0 failed ret = %d\n", __func__, ret);
		return;
	}

	/* ESPI CS2 */
	ret = dm_gpio_lookup_name("GPIO5_9", &desc);
	if (ret) {
		printf("%s lookup GPIO5_9 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "spi3_cs2");
	if (ret) {
		printf("%s request spi3_cs2 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);
}

static void setup_iomux_gpios(void)
{
	int ret;
	struct gpio_desc desc;

        imx_iomux_v3_setup_multiple_pads(gpios_pads, ARRAY_SIZE(gpios_pads));
	/* Set GPIO0 as Output */
	ret = dm_gpio_lookup_name("GPIO2_0", &desc);
	if (ret) {
		printf("%s lookup GPIO2_0 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio0");
	if (ret) {
		printf("%s request gpio0 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* Set GPIO1 as Output */
	ret = dm_gpio_lookup_name("GPIO2_1", &desc);
	if (ret) {
		printf("%s lookup GPIO2_1 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio1");
	if (ret) {
		printf("%s request gpio1 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* Set GPIO2 as Output */
	ret = dm_gpio_lookup_name("GPIO2_2", &desc);
	if (ret) {
		printf("%s lookup GPIO2_2 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio2");
	if (ret) {
		printf("%s request gpio2 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* Set GPIO3 as Output */
	ret = dm_gpio_lookup_name("GPIO2_3", &desc);
	if (ret) {
		printf("%s lookup GPIO2_3 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio3");
	if (ret) {
		printf("%s request gpio3 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* Set GPIO4 as Output */
	ret = dm_gpio_lookup_name("GPIO2_4", &desc);
	if (ret) {
		printf("%s lookup GPIO2_4 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio4");
	if (ret) {
		printf("%s request gpio4 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* Set GPIO6 as Input */
	ret = dm_gpio_lookup_name("GPIO2_5", &desc);
	if (ret) {
		printf("%s lookup GPIO2_5 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio6");
	if (ret) {
		printf("%s request gpio6 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set GPIO7 as Input */
	ret = dm_gpio_lookup_name("GPIO2_7", &desc);
	if (ret) {
		printf("%s lookup GPIO2_7 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio7");
	if (ret) {
		printf("%s request gpio7 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set GPIO8 as Input */
	ret = dm_gpio_lookup_name("GPIO2_6", &desc);
	if (ret) {
		printf("%s lookup GPIO2_6 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio8");
	if (ret) {
		printf("%s request gpio8 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set GPIO9 as Input */
	ret = dm_gpio_lookup_name("GPIO4_1", &desc);
	if (ret) {
		printf("%s lookup GPIO4_1 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio9");
	if (ret) {
		printf("%s request gpio9 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

        /* Set GPIO10 as Input */
        ret = dm_gpio_lookup_name("GPIO4_6", &desc);
        if (ret) {
                printf("%s lookup GPIO4_6 failed ret = %d\n", __func__, ret);
                return;
        }

        ret = dm_gpio_request(&desc, "gpio10");
        if (ret) {
                printf("%s request gpio10 failed ret = %d\n", __func__, ret);
                return;
        }

        dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

        /* Set GPIO11 as Input */
        ret = dm_gpio_lookup_name("GPIO4_7", &desc);
        if (ret) {
                printf("%s lookup GPIO4_7 failed ret = %d\n", __func__, ret);
                return;
        }

        ret = dm_gpio_request(&desc, "gpio11");
        if (ret) {
                printf("%s request gpio11 failed ret = %d\n", __func__, ret);
                return;
        }

        dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);
}

static void setup_iomux_lvds_ch_sel(void)
{
	int ret;
	struct gpio_desc desc;

	imx_iomux_v3_setup_multiple_pads(lvds_ch_sel_pads, ARRAY_SIZE(lvds_ch_sel_pads));
	/* Set LVDS_CH_SEL as Output Low */
	ret = dm_gpio_lookup_name("GPIO5_13", &desc);
	if (ret) {
		printf("%s lookup GPIO5_13 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "LVDS_CH_SEL");
	if (ret) {
		printf("%s request LVDS_CH_SEL failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);
}

static void setup_iomux_misc(void)
{
	int ret;
	struct gpio_desc desc;

	imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));

	/* Set SLEEP# as Input */
	ret = dm_gpio_lookup_name("GPIO5_14", &desc);
	if (ret) {
		printf("%s lookup GPIO5_14 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "SLEEP#");
	if (ret) {
		printf("%s request SLEEP# failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set CHARGING# as Input */
	ret = dm_gpio_lookup_name("GPIO1_8", &desc);
	if (ret) {
		printf("%s lookup GPIO1_8 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "CHARGING#");
	if (ret) {
		printf("%s request CHARGING# failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set CHARGER_PRSNT# as Input */
	ret = dm_gpio_lookup_name("GPIO1_9", &desc);
	if (ret) {
		printf("%s lookup GPIO1_9 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "CHARGER_PRSNT#");
	if (ret) {
		printf("%s request CHARGER_PRSNT# failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set BATLOW# as Input */
	ret = dm_gpio_lookup_name("GPIO5_11", &desc);
	if (ret) {
		printf("%s lookup GPIO5_11 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "BATLOW#");
	if (ret) {
		printf("%s request BATLOW# failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set CARRIER_STBY# as Output High*/
	ret = dm_gpio_lookup_name("GPIO6_16", &desc);
	if (ret) {
		printf("%s lookup GPIO6_16 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "CARRIER_STBY#");
	if (ret) {
		printf("%s request CARRIER_STBY# failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

	/* Reset PCIe*/
	ret = dm_gpio_lookup_name("GPIO2_28", &desc);
	if (ret) {
		printf("%s lookup GPIO2_28 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "PCIE_RST#");
	if (ret) {
		printf("%s request PCIE_RST# failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);
	udelay(500);
	dm_gpio_set_value(&desc, 1);

	/* Set PCIE_WAKE# as Input */
	ret = dm_gpio_lookup_name("GPIO2_31", &desc);
	if (ret) {
		printf("%s lookup GPIO2_31 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "PCIE_WAKE#");
	if (ret) {
		printf("%s request PCIE_WAKE# failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set WDT_TIME_OUT# as Output High */
	ret = dm_gpio_lookup_name("GPIO7_14", &desc);
	if (ret) {
		printf("%s lookup GPIO7_14 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "WDT_TIME_OUT#");
	if (ret) {
		printf("%s request WDT_TIME_OUT# failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);
}

static void setup_iomux_flexcan1(void)
{
	imx_iomux_v3_setup_multiple_pads(flexcan1_pads, ARRAY_SIZE(flexcan1_pads));
}

static void setup_iomux_flexcan2(void)
{
	imx_iomux_v3_setup_multiple_pads(flexcan2_pads, ARRAY_SIZE(flexcan2_pads));
}

#ifdef CONFIG_IMX_BOOTAUX
ulong board_get_usable_ram_top(ulong total_size)
{
	/* Reserve top 1M memory used by M core vring/buffer */
	return gd->ram_top - SZ_1M;
}
#endif

#ifdef CONFIG_FEC_MXC
static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, clear gpr1[13], gpr1[17]*/
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
		(IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK |
		 IOMUXC_GPR_GPR1_GPR_ENET1_CLK_DIR_MASK), 0);

	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
			(IOMUXC_GPR_GPR1_GPR_ENET2_TX_CLK_SEL_MASK |
			 IOMUXC_GPR_GPR1_GPR_ENET2_CLK_DIR_MASK), 0);

	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

#ifdef CONFIG_FSL_QSPI
int board_qspi_init(void)
{
	/* Set the clock */
	set_clk_qspi();

	return 0;
}
#endif

#ifdef CONFIG_MXC_EPDC
iomux_v3_cfg_t const epdc_en_pads[] = {
	MX7D_PAD_GPIO1_IO04__GPIO1_IO4 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const epdc_enable_pads[] = {
	MX7D_PAD_EPDC_DATA00__EPDC_DATA0	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_DATA01__EPDC_DATA1	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_DATA02__EPDC_DATA2	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_DATA03__EPDC_DATA3	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_DATA04__EPDC_DATA4	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_DATA05__EPDC_DATA5	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_DATA06__EPDC_DATA6	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_DATA07__EPDC_DATA7	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_SDCLK__EPDC_SDCLK		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_SDLE__EPDC_SDLE		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_SDOE__EPDC_SDOE		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_SDSHR__EPDC_SDSHR		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_SDCE0__EPDC_SDCE0		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_SDCE1__EPDC_SDCE1		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_GDCLK__EPDC_GDCLK		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_GDOE__EPDC_GDOE		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_GDRL__EPDC_GDRL		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_GDSP__EPDC_GDSP		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_BDR0__EPDC_BDR0		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX7D_PAD_EPDC_BDR1__EPDC_BDR1		| MUX_PAD_CTRL(EPDC_PAD_CTRL),
};

static iomux_v3_cfg_t const epdc_disable_pads[] = {
	MX7D_PAD_EPDC_DATA00__GPIO2_IO0,
	MX7D_PAD_EPDC_DATA01__GPIO2_IO1,
	MX7D_PAD_EPDC_DATA02__GPIO2_IO2,
	MX7D_PAD_EPDC_DATA03__GPIO2_IO3,
	MX7D_PAD_EPDC_DATA04__GPIO2_IO4,
	MX7D_PAD_EPDC_DATA05__GPIO2_IO5,
	MX7D_PAD_EPDC_DATA06__GPIO2_IO6,
	MX7D_PAD_EPDC_DATA07__GPIO2_IO7,
	MX7D_PAD_EPDC_SDCLK__GPIO2_IO16,
	MX7D_PAD_EPDC_SDLE__GPIO2_IO17,
	MX7D_PAD_EPDC_SDOE__GPIO2_IO18,
	MX7D_PAD_EPDC_SDSHR__GPIO2_IO19,
	MX7D_PAD_EPDC_SDCE0__GPIO2_IO20,
	MX7D_PAD_EPDC_SDCE1__GPIO2_IO21,
	MX7D_PAD_EPDC_GDCLK__GPIO2_IO24,
	MX7D_PAD_EPDC_GDOE__GPIO2_IO25,
	MX7D_PAD_EPDC_GDRL__GPIO2_IO26,
	MX7D_PAD_EPDC_GDSP__GPIO2_IO27,
	MX7D_PAD_EPDC_BDR0__GPIO2_IO28,
	MX7D_PAD_EPDC_BDR1__GPIO2_IO29,
};

vidinfo_t panel_info = {
	.vl_refresh = 85,
	.vl_col = 1024,
	.vl_row = 758,
	.vl_pixclock = 40000000,
	.vl_left_margin = 12,
	.vl_right_margin = 76,
	.vl_upper_margin = 4,
	.vl_lower_margin = 5,
	.vl_hsync = 12,
	.vl_vsync = 2,
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
	.gdclk_hp_offs = 524,
	.gdsp_offs = 327,
	.gdoe_offs = 0,
	.gdclk_offs = 19,
	.num_ce = 1,
};

struct gpio_desc epd_pwrstat_desc;
struct gpio_desc epd_vcom_desc;
struct gpio_desc epd_wakeup_desc;
struct gpio_desc epd_pwr_ctl0_desc;

static void setup_epdc_power(void)
{
	int ret;

	/* IOMUX_GPR1: bit30: Disable On-chip RAM EPDC Function */
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
		IOMUXC_GPR_GPR1_GPR_ENABLE_OCRAM_EPDC_MASK, 0);

	/* Setup epdc voltage */

	/* EPDC_PWRSTAT - GPIO2[31] for PWR_GOOD status */
	ret = dm_gpio_lookup_name("GPIO2_31", &epd_pwrstat_desc);
	if (ret) {
		printf("%s lookup GPIO2_31 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&epd_pwrstat_desc, "epdc_pwrstat");
	if (ret) {
		printf("%s request epdc_pwrstat failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&epd_pwrstat_desc, GPIOD_IS_IN);

	/* EPDC_VCOM0 - GPIO4[14] for VCOM control */
	/* Set as output */
	ret = dm_gpio_lookup_name("GPIO4_14", &epd_vcom_desc);
	if (ret) {
		printf("%s lookup GPIO4_14 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&epd_vcom_desc, "epdc_vcom");
	if (ret) {
		printf("%s request epdc_vcom failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&epd_vcom_desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);

	/* EPDC_PWRWAKEUP - GPIO2[23] for EPD PMIC WAKEUP */
	/* Set as output */
	ret = dm_gpio_lookup_name("GPIO2_23", &epd_wakeup_desc);
	if (ret) {
		printf("%s lookup GPIO2_23 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&epd_wakeup_desc, "epdc_pmic");
	if (ret) {
		printf("%s request epdc_pmic failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&epd_wakeup_desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);

	/* EPDC_PWRCTRL0 - GPIO2[30] for EPD PWR CTL0 */
	/* Set as output */
	ret = dm_gpio_lookup_name("GPIO2_30", &epd_pwr_ctl0_desc);
	if (ret) {
		printf("%s lookup GPIO2_30 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&epd_pwr_ctl0_desc, "epdc_pwr_ctl0");
	if (ret) {
		printf("%s request epdc_pwr_ctl0 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&epd_pwr_ctl0_desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
}

static void epdc_enable_pins(void)
{
	/* epdc iomux settings */
	imx_iomux_v3_setup_multiple_pads(epdc_enable_pads,
				ARRAY_SIZE(epdc_enable_pads));
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to GPIO  and drive to 0 */
	imx_iomux_v3_setup_multiple_pads(epdc_disable_pads,
				ARRAY_SIZE(epdc_disable_pads));
}

static void setup_epdc(void)
{
	/*** epdc Maxim PMIC settings ***/

	/* EPDC_PWRSTAT - GPIO2[31] for PWR_GOOD status */
	imx_iomux_v3_setup_pad(MX7D_PAD_EPDC_PWR_STAT__GPIO2_IO31 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/* EPDC_VCOM0 - GPIO4[14] for VCOM control */
	imx_iomux_v3_setup_pad(MX7D_PAD_I2C4_SCL__GPIO4_IO14 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/* EPDC_PWRWAKEUP - GPIO4[23] for EPD PMIC WAKEUP */
	imx_iomux_v3_setup_pad(MX7D_PAD_EPDC_SDCE3__GPIO2_IO23 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/* EPDC_PWRCTRL0 - GPIO4[20] for EPD PWR CTL0 */
	imx_iomux_v3_setup_pad(MX7D_PAD_EPDC_PWR_COM__GPIO2_IO30 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/* Set pixel clock rates for EPDC in clock.c */

	panel_info.epdc_data.wv_modes.mode_init = 0;
	panel_info.epdc_data.wv_modes.mode_du = 1;
	panel_info.epdc_data.wv_modes.mode_gc4 = 3;
	panel_info.epdc_data.wv_modes.mode_gc8 = 2;
	panel_info.epdc_data.wv_modes.mode_gc16 = 2;
	panel_info.epdc_data.wv_modes.mode_gc32 = 2;

	panel_info.epdc_data.epdc_timings = panel_timings;

	setup_epdc_power();
}

void epdc_power_on(void)
{
	unsigned int reg;
	struct gpio_regs *gpio_regs = (struct gpio_regs *)GPIO2_BASE_ADDR;

	/* Set EPD_PWR_CTL0 to high - enable EINK_VDD (3.15) */
	dm_gpio_set_value(&epd_pwr_ctl0_desc, 1);
	udelay(1000);

	/* Enable epdc signal pin */
	epdc_enable_pins();

	/* Set PMIC Wakeup to high - enable Display power */
	dm_gpio_set_value(&epd_wakeup_desc, 1);

	/* Wait for PWRGOOD == 1 */
	while (1) {
		reg = readl(&gpio_regs->gpio_psr);
		if (!(reg & (1 << 31)))
			break;

		udelay(100);
	}

	/* Enable VCOM */
	dm_gpio_set_value(&epd_vcom_desc, 1);

	udelay(500);
}

void epdc_power_off(void)
{
	/* Set PMIC Wakeup to low - disable Display power */
	dm_gpio_set_value(&epd_wakeup_desc, 0);

	/* Disable VCOM */
	dm_gpio_set_value(&epd_vcom_desc, 0);

	epdc_disable_pins();

	/* Set EPD_PWR_CTL0 to low - disable EINK_VDD (3.15) */
	dm_gpio_set_value(&epd_pwr_ctl0_desc, 0);
}
#endif

static void setup_usb(void)
{
	int ret;
	struct gpio_desc desc;

	/* OTG1 Over Current */
	ret = dm_gpio_lookup_name("GPIO1_4", &desc);
	if (ret) {
		printf("%s lookup GPIO1_4 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "OTG1_OC");
	if (ret) {
		printf("%s request OTG1_OC failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set OTG1_PWR as Output High */
	ret = dm_gpio_lookup_name("GPIO1_5", &desc);
	if (ret) {
		printf("%s lookup GPIO1_5 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "OTG1_PWR");
	if (ret) {
		printf("%s request OTG1_PWR failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

	/* OTG2 */
	/* OTG2 Over Current */
	ret = dm_gpio_lookup_name("GPIO1_6", &desc);
	if (ret) {
		printf("%s lookup GPIO1_6 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "OTG2_OC");
	if (ret) {
		printf("%s request OTG2_OC failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set OTG2_PWR as Output High */
	ret = dm_gpio_lookup_name("GPIO1_7", &desc);
	if (ret) {
		printf("%s lookup GPIO1_7 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "OTG2_PWR");
	if (ret) {
		printf("%s request OTG2_PWR failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);
}

int board_early_init_f(void)
{
#ifdef CONFIG_CONSOLE_SER0
	setup_iomux_uart6();
#endif

#ifdef CONFIG_CONSOLE_SER1
	setup_iomux_uart2();
#endif

#ifdef CONFIG_CONSOLE_SER2
	setup_iomux_uart7();
#endif

#ifdef CONFIG_CONSOLE_SER3
	setup_iomux_uart3();
#endif

#ifdef CONFIG_MXC_SPI
	setup_spinor();
#endif

	setup_iomux_flexcan1();
	setup_iomux_flexcan2();

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand();
#endif

#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif

#ifdef CONFIG_MXC_EPDC
	setup_epdc();
#endif

	return 0;
}


#ifdef CONFIG_DM_PMIC
int power_init_board(void)
{
	struct udevice *dev;
	int ret, dev_id, rev_id;
	u32 sw3mode;

	ret = pmic_get("pfuze3000@8", &dev);
	if (ret == -ENODEV)
		return 0;
	if (ret != 0)
		return ret;

	dev_id = pmic_reg_read(dev, PFUZE3000_DEVICEID);
	rev_id = pmic_reg_read(dev, PFUZE3000_REVID);
	printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", dev_id, rev_id);

	pmic_clrsetbits(dev, PFUZE3000_LDOGCTL, 0, 1);

	/* change sw3 mode to avoid DDR power off */
	sw3mode = pmic_reg_read(dev, PFUZE3000_SW3MODE);
	ret = pmic_reg_write(dev, PFUZE3000_SW3MODE, sw3mode | 0x20);
	if (ret < 0)
		printf("PMIC: PFUZE3000 change sw3 mode failed\n");

	return 0;
}
#endif

int board_late_init(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	env_set("tee", "no");
#ifdef CONFIG_IMX_OPTEE
	env_set("tee", "yes");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	setup_lcd();

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	/*
	 * Do not assert internal WDOG_RESET_B_DEB(controlled by bit 4),
	 * since we use PMIC_PWRON to reset the board.
	 */
	/*clrsetbits_le16(&wdog->wcr, 0, 0x10);*/

	puts("---------Embedian SMARC-FiMX7------------\n");
	/* Read Module Information from on module EEPROM and pass
	 * mac address to kernel
	 */
	struct udevice *dev;
	char bootmode = 0;
	int ret;
	struct gpio_desc desc;
	u8 name[8];
	u8 serial[12];
	u8 revision[4];
	u8 mac[6];
	u8 mac1[6];

	ret = i2c_get_chip_for_busnum(1, 0x50, 2, &dev);
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

	/*MAC address*/
	ret = dm_i2c_read(dev, 0x3c, mac, 6);
	if (ret) {
		debug("failed to read eth0 mac address from EEPROM\n");
		return 0;
	}

	if (is_valid_ethaddr(mac))
	printf("  MAC Address:          %02x:%02x:%02x:%02x:%02x:%02x\n",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		eth_env_set_enetaddr("ethaddr", mac);
#ifdef CONFIG_MX7D
	ret = dm_i2c_read(dev, 0x42, mac1, 6);
	if (ret) {
		debug("failed to read eth1 mac address from EEPROM\n");
		return 0;
	}

	if (is_valid_ethaddr(mac1))
	printf("  MAC1 Address:         %02x:%02x:%02x:%02x:%02x:%02x\n",
		mac1[0], mac1[1], mac1[2], mac1[3], mac1[4], mac1[5]);
		eth_env_set_enetaddr("eth1addr", mac1);
#endif
	puts("-----------------------------------------\n");

/* RESET_OUT Pin */
	setup_iomux_reset_out();
/* ECSPI1 CS Pins */
	setup_iomux_spi1();
/* ECSPI3 CS Pins */
	setup_iomux_spi3();
/* USB Power Control Pins */
	setup_usb();
/* GPIO Pins */
	setup_iomux_gpios();
/* LVDS Channel Selection Pin */
	setup_iomux_lvds_ch_sel();
/* MISC Pins */
	setup_iomux_misc();

/* eMMC Power Reset */
	ret = dm_gpio_lookup_name("GPIO6_11", &desc);
	if (ret) {
		printf("%s lookup GPIO6_11 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	ret = dm_gpio_request(&desc, "usdhc3_pwr");
	if (ret) {
		printf("%s request usdhc3_pwr failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);
	udelay(500);
	dm_gpio_set_value(&desc, 1);

/* SMARC BOOT_SEL*/
	/* BOOT_SEL1 */
	ret = dm_gpio_lookup_name("GPIO5_15", &desc);
	if (ret) {
		printf("%s lookup GPIO5_15 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	ret = dm_gpio_request(&desc, "BOOT_SEL1");
	if (ret) {
		printf("%s request BOOT_SEL1 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);
		bootmode |= (dm_gpio_get_value(&desc) ? 1 : 0) << 0;

	/* BOOT_SEL2 */
	ret = dm_gpio_lookup_name("GPIO5_16", &desc);
	if (ret) {
		printf("%s lookup GPIO5_16 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	ret = dm_gpio_request(&desc, "BOOT_SEL2");
	if (ret) {
		printf("%s request BOOT_SEL2 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);
		bootmode |= (dm_gpio_get_value(&desc) ? 1 : 0) << 1;

	/* BOOT_SEL3 */	
	ret = dm_gpio_lookup_name("GPIO5_17", &desc);
	if (ret) {
		printf("%s lookup GPIO5_17 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	ret = dm_gpio_request(&desc, "BOOT_SEL3");
	if (ret) {
		printf("%s request BOOT_SEL3 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);
		bootmode |= (dm_gpio_get_value(&desc) ? 1 : 0) << 2;

	if (bootmode == 0) {
		puts("BOOT_SEL Detected: OFF OFF OFF, unsupported boot up device: SATA...\n");
		hang();
	} else if (bootmode == 4) {
		puts("BOOT_SEL Detected: OFF OFF ON, unsupported boot up device: NAND...\n");
		hang();
	} else if (bootmode == 2) {
		puts("BOOT_SEL Detected: OFF ON OFF, unsupported boot up device: Carrier eSPI...\n");
		hang();
	} else if (bootmode == 1) {
		puts("BOOT_SEL Detected: ON OFF OFF, Load zImage from Carrier SD Card...\n");
		env_set_ulong("mmcdev", 0);
		env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
	} else if (bootmode == 6) {
		puts("BOOT_SEL Detected: OFF ON ON, Load zImage from Module eMMC Flash...\n");
		env_set_ulong("mmcdev", 1);
		env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
	} else if (bootmode == 5) {
		puts("BOOT_SEL Detected: ON OFF ON, Load zImage from GBE...\n");
		env_set("bootcmd", "run netboot;");
	} else if (bootmode == 2) {
		puts("BOOT_SEL Detected: OFF ON OFF, unsupported boot up device: Carrier SPI...\n");
		hang();
	} else if (bootmode == 7) {
		puts("BOOT_SEL Detected: ON ON ON, MOdule SPI Boot up is Default, Load zImage from Module eMMC...\n");
		env_set_ulong("mmcdev", 1);
		env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
	} else {
		puts("unsupported boot devices\n");
		hang();
	}

	return 0;
}
