/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx7-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <i2c.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
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

#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_DSE_3P3V_49OHM | \
	PAD_CTL_PUS_PU100KOHM | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM)
#define ENET_PAD_CTRL_MII  (PAD_CTL_DSE_3P3V_32OHM)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_PU100KOHM | \
	PAD_CTL_DSE_3P3V_49OHM)

#define QSPI_PAD_CTRL	\
	(PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUE | PAD_CTL_PUS_PU47KOHM)

#define NAND_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_SLOW | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define NAND_PAD_READY0_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUS_PU5KOHM)

#define BUTTON_PAD_CTRL    (PAD_CTL_PUS_PU5KOHM | PAD_CTL_DSE_3P3V_98OHM)

#define EPDC_PAD_CTRL	0x0

#define WEAK_PULLUP (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM | \
        PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#ifdef CONFIG_MXC_SPI
/* SPI2 (SPINOR) */
static iomux_v3_cfg_t const ecspi2_pads[] = {
        MX7D_PAD_ECSPI2_SCLK__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX7D_PAD_ECSPI2_MOSI__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX7D_PAD_ECSPI2_MISO__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX7D_PAD_ECSPI2_SS0__GPIO4_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL),    /*SS0#*/
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
static iomux_v3_cfg_t const uart6_pads[] = {
        MX7D_PAD_EPDC_DATA09__UART6_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX7D_PAD_EPDC_DATA08__UART6_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX7D_PAD_EPDC_DATA10__UART6_DTE_CTS | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX7D_PAD_EPDC_DATA11__UART6_DTE_RTS | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* SER1/UART2 */
static iomux_v3_cfg_t const uart2_pads[] = {
        MX7D_PAD_UART2_TX_DATA__UART2_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX7D_PAD_UART2_RX_DATA__UART2_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* SER2/UART7 */
static iomux_v3_cfg_t const uart7_pads[] = {
        MX7D_PAD_EPDC_DATA13__UART7_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX7D_PAD_EPDC_DATA12__UART7_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX7D_PAD_EPDC_DATA14__UART7_DTE_CTS | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX7D_PAD_EPDC_DATA15__UART7_DTE_RTS | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* SER3/UART3 Debug Port */
static iomux_v3_cfg_t const uart3_pads[] = {
        MX7D_PAD_UART3_TX_DATA__UART3_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX7D_PAD_UART3_RX_DATA__UART3_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* RESET_OUT# */
static iomux_v3_cfg_t const reset_out_pads[] = {
        MX7D_PAD_EPDC_BDR1__GPIO2_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/* SPI0 */
static iomux_v3_cfg_t const ecspi1_pads[] = {
        MX7D_PAD_ECSPI1_SCLK__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX7D_PAD_ECSPI1_MOSI__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX7D_PAD_ECSPI1_MISO__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX7D_PAD_ECSPI1_SS0__ECSPI1_SS0 | MUX_PAD_CTRL(NO_PAD_CTRL),    /*SS0#*/
        MX7D_PAD_UART1_RX_DATA__ECSPI1_SS1 | MUX_PAD_CTRL(NO_PAD_CTRL), /*SS1#*/
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
        MX7D_PAD_EPDC_DATA00__GPIO2_IO0 | MUX_PAD_CTRL(WEAK_PULLUP),    /* GPIO0 */
        MX7D_PAD_EPDC_DATA01__GPIO2_IO1 | MUX_PAD_CTRL(WEAK_PULLUP),    /* GPIO1 */
        MX7D_PAD_EPDC_DATA02__GPIO2_IO2 | MUX_PAD_CTRL(WEAK_PULLUP),    /* GPIO2 */
        MX7D_PAD_EPDC_DATA03__GPIO2_IO3 | MUX_PAD_CTRL(WEAK_PULLUP),    /* GPIO3 */
        MX7D_PAD_EPDC_DATA04__GPIO2_IO4 | MUX_PAD_CTRL(WEAK_PULLUP),    /* GPIO4 */
        MX7D_PAD_EPDC_DATA05__GPIO2_IO5 | MUX_PAD_CTRL(WEAK_PULLUP),    /* GPIO6 */
        MX7D_PAD_EPDC_DATA07__GPIO2_IO7 | MUX_PAD_CTRL(WEAK_PULLUP),    /* GPIO7 */
        MX7D_PAD_EPDC_DATA06__GPIO2_IO6 | MUX_PAD_CTRL(WEAK_PULLUP),    /* GPIO8 */
        MX7D_PAD_UART1_TX_DATA__GPIO4_IO1 | MUX_PAD_CTRL(WEAK_PULLUP),  /* GPIO9 */
        MX7D_PAD_UART3_RTS_B__GPIO4_IO6 | MUX_PAD_CTRL(WEAK_PULLUP),    /* GPIO10 */
        MX7D_PAD_UART3_CTS_B__GPIO4_IO7 | MUX_PAD_CTRL(WEAK_PULLUP),    /* GPIO11 */
};

/* LVDS channel selection, set low as single channel LVDS and high as dual channel LVDS */
static iomux_v3_cfg_t const lvds_ch_sel_pads[] = {
        MX7D_PAD_SD2_CMD__GPIO5_IO13 | MUX_PAD_CTRL(WEAK_PULLUP),       /* LVDS_CH_SEL */
};

/* Misc. pins */
static iomux_v3_cfg_t const misc_pads[] = {
        MX7D_PAD_SD2_DATA0__GPIO5_IO14 | MUX_PAD_CTRL(WEAK_PULLUP),     /* SLEEP# */
        MX7D_PAD_GPIO1_IO09__GPIO1_IO9 | MUX_PAD_CTRL(WEAK_PULLUP),     /* CHARGER_PRSNT# */
        MX7D_PAD_GPIO1_IO08__GPIO1_IO8 | MUX_PAD_CTRL(WEAK_PULLUP),     /* CHARGING# */
        MX7D_PAD_SAI1_RX_SYNC__GPIO6_IO16 | MUX_PAD_CTRL(WEAK_PULLUP),  /* CARRIER_STBY# */
        MX7D_PAD_SD2_RESET_B__GPIO5_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),   /* BATLOW# */
        MX7D_PAD_EPDC_BDR0__GPIO2_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),     /* PCIe_RST# */
        MX7D_PAD_EPDC_PWR_STAT__GPIO2_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL), /* PCIe_WAKE# */
        MX7D_PAD_ENET1_CRS__GPIO7_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL),     /* WDT_TIME_OUT# */
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

#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX7D_PAD_LCD_CLK__LCD_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_ENABLE__LCD_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_HSYNC__LCD_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_VSYNC__LCD_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA00__LCD_DATA0 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA01__LCD_DATA1 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA02__LCD_DATA2 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA03__LCD_DATA3 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA04__LCD_DATA4 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA05__LCD_DATA5 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA06__LCD_DATA6 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA07__LCD_DATA7 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA08__LCD_DATA8 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA09__LCD_DATA9 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA10__LCD_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA11__LCD_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA12__LCD_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA13__LCD_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA14__LCD_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA15__LCD_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA16__LCD_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA17__LCD_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA18__LCD_DATA18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA19__LCD_DATA19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA20__LCD_DATA20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA21__LCD_DATA21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA22__LCD_DATA22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA23__LCD_DATA23 | MUX_PAD_CTRL(LCD_PAD_CTRL),

	MX7D_PAD_LCD_RESET__GPIO3_IO4	| MUX_PAD_CTRL(LCD_PAD_CTRL),
};

static iomux_v3_cfg_t const backlight_pads[] = {
        /* Backlight Enable for RGB: S127 */
        MX7D_PAD_SAI1_RX_BCLK__GPIO6_IO17 | MUX_PAD_CTRL(WEAK_PULLUP),

        /* PWM Backlight Control: S141. Use GPIO for Brightness adjustment, duty cycle = period */
        MX7D_PAD_GPIO1_IO02__GPIO1_IO2 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void do_enable_parallel_lcd(struct display_info_t const *dev)
{
	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

        imx_iomux_v3_setup_multiple_pads(backlight_pads, ARRAY_SIZE(backlight_pads));

	/* Reset LCD */
	gpio_request(IMX_GPIO_NR(3, 4), "lcd reset");
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 0);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 1);

        /* Turn on Backlight */
        gpio_request(IMX_GPIO_NR(6, 17), "backlight_enable");
        gpio_direction_output(IMX_GPIO_NR(6, 17), 1);

        /* Set Brightness to high */
        gpio_request(IMX_GPIO_NR(1, 2), "backlight_pwm");
        gpio_direction_output(IMX_GPIO_NR(1, 2) , 1);
}

/* LVDS Panel for AUO G070VW01 V0 7-inch Color TFT 800x480 Panel Settings */
struct display_info_t const displays[] = {{
        .bus = ELCDIF1_IPS_BASE_ADDR,
        .addr = 0,
        .pixfmt = 24,
        .detect = NULL,
        .enable = do_enable_parallel_lcd,
        .mode   = {
                .name           = "G070VW01",
                .xres           = 800,
                .yres           = 480,
                .pixclock       = 31069,
                .left_margin    = 64,
                .right_margin   = 64,
                .upper_margin   = 12,
                .lower_margin   = 4,
                .hsync_len      = 128,
                .vsync_len      = 12,
                .sync           = 0,
                .vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);
#endif

static void setup_iomux_uart6(void)
{
        imx_iomux_v3_setup_multiple_pads(uart6_pads, ARRAY_SIZE(uart6_pads));
}

static void setup_iomux_uart2(void)
{
        imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
}

static void setup_iomux_uart7(void)
{
        imx_iomux_v3_setup_multiple_pads(uart7_pads, ARRAY_SIZE(uart7_pads));
}

static void setup_iomux_uart3(void)
{
        imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
}

static void setup_iomux_reset_out(void)
{
        imx_iomux_v3_setup_multiple_pads(reset_out_pads, ARRAY_SIZE(reset_out_pads));

        /* Set CPU RESET_OUT as Output */
        gpio_request(IMX_GPIO_NR(2, 29), "reset_out");
        gpio_direction_output(IMX_GPIO_NR(2, 29) , 0);
}

static void setup_iomux_spi1(void)
{
        imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
        gpio_request(IMX_GPIO_NR(4, 19), "spi1_cs0");
        gpio_direction_output(IMX_GPIO_NR(4, 19), 0);
        gpio_request(IMX_GPIO_NR(4, 0), "spi1_cs1");
        gpio_direction_output(IMX_GPIO_NR(4, 0), 0);
}

static void setup_iomux_spi3(void)
{
        imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
        gpio_request(IMX_GPIO_NR(6, 22), "spi3_cs0");
        gpio_direction_output(IMX_GPIO_NR(6, 22), 0);
        gpio_request(IMX_GPIO_NR(5, 9), "spi3_cs2");
        gpio_direction_output(IMX_GPIO_NR(5, 9), 0);
}

static void setup_iomux_gpios(void)
{
        imx_iomux_v3_setup_multiple_pads(gpios_pads, ARRAY_SIZE(gpios_pads));
        gpio_request(IMX_GPIO_NR(2, 0), "GPIO0");
        gpio_direction_output(IMX_GPIO_NR(2, 0), 0);
        gpio_request(IMX_GPIO_NR(2, 1), "GPIO1");
        gpio_direction_output(IMX_GPIO_NR(2, 1), 0);
        gpio_request(IMX_GPIO_NR(2, 2), "GPIO2");
        gpio_direction_output(IMX_GPIO_NR(2, 2), 0);
        gpio_request(IMX_GPIO_NR(2, 3), "GPIO3");
        gpio_direction_output(IMX_GPIO_NR(2, 3), 0);
        gpio_request(IMX_GPIO_NR(2, 4), "GPIO4");
        gpio_direction_output(IMX_GPIO_NR(2, 4), 0);
        gpio_request(IMX_GPIO_NR(2, 5), "GPIO6");
        gpio_direction_input(IMX_GPIO_NR(2, 5));
        gpio_request(IMX_GPIO_NR(2, 7), "GPIO7");
        gpio_direction_input(IMX_GPIO_NR(2, 7));
        gpio_request(IMX_GPIO_NR(2, 6), "GPIO8");
        gpio_direction_input(IMX_GPIO_NR(2, 6));
        gpio_request(IMX_GPIO_NR(4, 1), "GPIO9");
        gpio_direction_input(IMX_GPIO_NR(4, 1));
        gpio_request(IMX_GPIO_NR(4, 6), "GPIO10");
        gpio_direction_input(IMX_GPIO_NR(4, 6));
        gpio_request(IMX_GPIO_NR(4, 7), "GPIO11");
        gpio_direction_input(IMX_GPIO_NR(4, 7));
}

static void setup_iomux_lvds_ch_sel(void)
{
        imx_iomux_v3_setup_multiple_pads(lvds_ch_sel_pads, ARRAY_SIZE(lvds_ch_sel_pads));
        gpio_request(IMX_GPIO_NR(5, 13), "LVDS_CH_SEL");
        gpio_direction_output(IMX_GPIO_NR(5, 13), 0);
}

static void setup_iomux_misc(void)
{
        imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));
        gpio_request(IMX_GPIO_NR(5, 14), "SLEEP#");
        gpio_direction_input(IMX_GPIO_NR(5, 14));
        gpio_request(IMX_GPIO_NR(1, 8), "CHARGING#");
        gpio_direction_input(IMX_GPIO_NR(1, 8));
        gpio_request(IMX_GPIO_NR(1, 9), "CHARGER_PRSNT#");
        gpio_direction_input(IMX_GPIO_NR(1, 9));
        gpio_request(IMX_GPIO_NR(5, 11), "BATLOW#");
        gpio_direction_input(IMX_GPIO_NR(5, 11));
        gpio_request(IMX_GPIO_NR(6, 16), "CARRIER_STBY#");
        gpio_direction_output(IMX_GPIO_NR(6, 16), 1);
        gpio_request(IMX_GPIO_NR(2, 28), "PCIE_RST#");
        gpio_direction_output(IMX_GPIO_NR(2, 28), 0);
        udelay(500);
        gpio_direction_output(IMX_GPIO_NR(2, 28), 1);
        gpio_request(IMX_GPIO_NR(2, 31), "PCIE_WAKE#");
        gpio_direction_input(IMX_GPIO_NR(2, 31));
        /* Set WDT_TIME_OUT# as Output High */
        gpio_request(IMX_GPIO_NR(7, 14), "WDT_TIME_OUT#");
        gpio_direction_output(IMX_GPIO_NR(7, 14), 1);
}

static void setup_iomux_flexcan1(void)
{
        imx_iomux_v3_setup_multiple_pads(flexcan1_pads, ARRAY_SIZE(flexcan1_pads));
}

static void setup_iomux_flexcan2(void)
{
        imx_iomux_v3_setup_multiple_pads(flexcan2_pads, ARRAY_SIZE(flexcan2_pads));
}

int board_mmc_get_env_dev(int devno)
{
	if (devno == 2)
		devno--;

	return devno;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	if (dev_no == 1)
		dev_no++;

	return dev_no;
}

#ifdef CONFIG_FEC_MXC
static int setup_fec(int fec_id)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	if (0 == fec_id) {
		/* Use 125M anatop REF_CLK1 for ENET1, clear gpr1[13], gpr1[17]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
			(IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK |
			 IOMUXC_GPR_GPR1_GPR_ENET1_CLK_DIR_MASK), 0);
	} else {
		/* Use 125M anatop REF_CLK2 for ENET2, clear gpr1[14], gpr1[18]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
			(IOMUXC_GPR_GPR1_GPR_ENET2_TX_CLK_SEL_MASK |
			 IOMUXC_GPR_GPR1_GPR_ENET2_CLK_DIR_MASK), 0);
	}

	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	/*phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x21);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x7ea8);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x2f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x71b7);*/

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

#ifdef CONFIG_FSL_QSPI
#ifndef CONFIG_DM_SPI
static iomux_v3_cfg_t const quadspi_pads[] = {
	MX7D_PAD_EPDC_DATA00__QSPI_A_DATA0 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA01__QSPI_A_DATA1 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA02__QSPI_A_DATA2 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA03__QSPI_A_DATA3 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA05__QSPI_A_SCLK  | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA06__QSPI_A_SS0_B | MUX_PAD_CTRL(QSPI_PAD_CTRL),
};
#endif

int board_qspi_init(void)
{
#ifndef CONFIG_DM_SPI
	/* Set the iomux */
	imx_iomux_v3_setup_multiple_pads(quadspi_pads,
					 ARRAY_SIZE(quadspi_pads));
#endif

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

static void setup_epdc_power(void)
{
	/* IOMUX_GPR1: bit30: Disable On-chip RAM EPDC Function */
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
		IOMUXC_GPR_GPR1_GPR_ENABLE_OCRAM_EPDC_MASK, 0);

	/* Setup epdc voltage */

	/* EPDC_PWRSTAT - GPIO2[31] for PWR_GOOD status */
	imx_iomux_v3_setup_pad(MX7D_PAD_EPDC_PWR_STAT__GPIO2_IO31 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));
	gpio_request(IMX_GPIO_NR(2, 31), "epdc_pwrstat");
	gpio_direction_input(IMX_GPIO_NR(2, 31));

	/* EPDC_VCOM0 - GPIO4[14] for VCOM control */
	imx_iomux_v3_setup_pad(MX7D_PAD_I2C4_SCL__GPIO4_IO14 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/* Set as output */
	gpio_request(IMX_GPIO_NR(4, 14), "epdc_vcom");
	gpio_direction_output(IMX_GPIO_NR(4, 14), 1);

	/* EPDC_PWRWAKEUP - GPIO2[23] for EPD PMIC WAKEUP */
	imx_iomux_v3_setup_pad(MX7D_PAD_EPDC_SDCE3__GPIO2_IO23 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));
	/* Set as output */
	gpio_request(IMX_GPIO_NR(2, 23), "epdc_pmic");
	gpio_direction_output(IMX_GPIO_NR(2, 23), 1);

	/* EPDC_PWRCTRL0 - GPIO2[30] for EPD PWR CTL0 */
	imx_iomux_v3_setup_pad(MX7D_PAD_EPDC_PWR_COM__GPIO2_IO30 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));
	/* Set as output */
	gpio_request(IMX_GPIO_NR(2, 30), "epdc_pwr_ctl0");
	gpio_direction_output(IMX_GPIO_NR(2, 30), 1);
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
	gpio_set_value(IMX_GPIO_NR(2, 30), 1);
	udelay(1000);

	/* Enable epdc signal pin */
	epdc_enable_pins();

	/* Set PMIC Wakeup to high - enable Display power */
	gpio_set_value(IMX_GPIO_NR(2, 23), 1);

	/* Wait for PWRGOOD == 1 */
	while (1) {
		reg = readl(&gpio_regs->gpio_psr);
		if (!(reg & (1 << 31)))
			break;

		udelay(100);
	}

	/* Enable VCOM */
	gpio_set_value(IMX_GPIO_NR(4, 14), 1);

	udelay(500);
}

void epdc_power_off(void)
{
	/* Set PMIC Wakeup to low - disable Display power */
	gpio_set_value(IMX_GPIO_NR(2, 23), 0);

	/* Disable VCOM */
	gpio_set_value(IMX_GPIO_NR(4, 14), 0);

	epdc_disable_pins();

	/* Set EPD_PWR_CTL0 to low - disable EINK_VDD (3.15) */
	gpio_set_value(IMX_GPIO_NR(2, 30), 0);
}
#endif

static void setup_usb(void)
{
        /* OTG1 Over Current */
        gpio_request(IMX_GPIO_NR(1, 4), "OTG1_OC");
        gpio_direction_input(IMX_GPIO_NR(1, 4));
        gpio_request(IMX_GPIO_NR(1, 5), "OTG1_PWR");
        gpio_direction_output(IMX_GPIO_NR(1, 5), 1);
        /* OTG2 */
        gpio_request(IMX_GPIO_NR(1, 6), "OTG2_OC");
        gpio_direction_input(IMX_GPIO_NR(1, 6));
        gpio_request(IMX_GPIO_NR(1, 7), "OTG2_PWR");
        gpio_direction_output(IMX_GPIO_NR(1, 7), 1);
}

int board_early_init_f(void)
{
        setup_iomux_uart6();
        setup_iomux_uart2();
        setup_iomux_uart7();
        setup_iomux_uart3();

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
	setup_fec(CONFIG_FEC_ENET_DEV);
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

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd1", MAKE_CFGVAL(0x10, 0x10, 0x00, 0x00)},
	{"emmc", MAKE_CFGVAL(0x10, 0x2a, 0x00, 0x00)},
	/* TODO: Nand */
	{"qspi", MAKE_CFGVAL(0x00, 0x40, 0x00, 0x00)},
	{NULL,   0},
};
#endif

#ifdef CONFIG_DM_PMIC
int power_init_board(void)
{
	struct udevice *dev;
	int ret, dev_id, rev_id;
	u32 sw3mode;

	ret = pmic_get("pfuze3000", &dev);
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
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	env_set("tee", "no");
#ifdef CONFIG_IMX_OPTEE
	env_set("tee", "yes");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

        /*
         * Do not assert internal WDOG_RESET_B_DEB(controlled by bit 4),
         * since we use PMIC_PWRON to reset the board.
         */
        clrsetbits_le16(&wdog->wcr, 0, 0x10);

        puts("---------Embedian SMARC-FiMX7------------\n");
        /* Read Module Information from on module EEPROM and pass
         * mac address to kernel
        */
        struct udevice *dev;
        int ret;
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

/* SDCARD POWER Enable */
        gpio_request(IMX_GPIO_NR(5, 2), "SDIO_PWR_EN");
        gpio_direction_output(IMX_GPIO_NR(5, 2), 1);

/* eMMC Power Reset */
#define USDHC3_PWR_GPIO IMX_GPIO_NR(6, 11)
        gpio_request(USDHC3_PWR_GPIO, "usdhc3_pwr");
        gpio_direction_output(USDHC3_PWR_GPIO, 0);
        udelay(500);
        gpio_direction_output(USDHC3_PWR_GPIO, 1);

/* SMARC BOOT_SEL*/
        gpio_request(IMX_GPIO_NR(5, 15), "BOOT_SEL_1");
        gpio_request(IMX_GPIO_NR(5, 16), "BOOT_SEL_2");
        gpio_request(IMX_GPIO_NR(5, 17), "BOOT_SEL_3");

        if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 0)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 0)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 0)) {
                puts("BOOT_SEL Detected: OFF OFF OFF, unsupported boot up device: SATA...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 0)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 0)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 1)) {
                puts("BOOT_SEL Detected: OFF OFF ON, unsupported boot up device: NAND...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 0)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 0)) {
                puts("BOOT_SEL Detected: OFF ON OFF, unsupported boot up device: Carrier eSPI...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 0)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 0)) {
                puts("BOOT_SEL Detected: ON OFF OFF, Load zImage from Carrier SD Card...\n");
                env_set_ulong("mmcdev", 0);
                env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 0)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 1)) {
                puts("BOOT_SEL Detected: OFF ON ON, Load zImage from Module eMMC Flash...\n");
                env_set_ulong("mmcdev", 1);
                env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 0)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 1)) {
                puts("BOOT_SEL Detected: ON OFF ON, Load zImage from GBE...\n");
                env_set("bootcmd", "run netboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 0)) {
                puts("BOOT_SEL Detected: OFF ON OFF, unsupported boot up device: Carrier SPI...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 1)) {
                puts("BOOT_SEL Detected: ON ON ON, MOdule SPI Boot up is Default, Load zImage from Module eMMC...\n");
                env_set_ulong("mmcdev", 1);
                env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
        } else {
                puts("unsupported boot devices\n");
                hang();
        }

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY

/* Use LID# for recovery key */
#define GPIO_VOL_DN_KEY IMX_GPIO_NR(5, 10)
iomux_v3_cfg_t const recovery_key_pads[] = {
        (MX7D_PAD_SD2_WP__GPIO5_IO10 | MUX_PAD_CTRL(BUTTON_PAD_CTRL)),
};

int is_recovery_key_pressing(void)
{
	int button_pressed = 0;

	/* Check Recovery Combo Button press or not. */
	imx_iomux_v3_setup_multiple_pads(recovery_key_pads,
		ARRAY_SIZE(recovery_key_pads));

	gpio_request(GPIO_VOL_DN_KEY, "volume_dn_key");
	gpio_direction_input(GPIO_VOL_DN_KEY);

	if (gpio_get_value(GPIO_VOL_DN_KEY) == 0) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
	}

	return  button_pressed;
}

#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/
