/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx7-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include "pfuze.h"
#include <pwm.h>
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/arch/crm_regs.h>
#include <usb.h>
#include <usb/ehci-fsl.h>
#if defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#include <asm/imx-common/video.h>

#include "smarcfimx7.h"
#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_DSE_3P3V_49OHM | \
	PAD_CTL_PUS_PU100KOHM | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU47KOHM)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM)
#define ENET_PAD_CTRL_MII  (PAD_CTL_DSE_3P3V_32OHM)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM)

#define I2C_PAD_CTRL    (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU100KOHM)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_PU100KOHM | \
        PAD_CTL_DSE_3P3V_49OHM)

#define SPI_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define BUTTON_PAD_CTRL    (PAD_CTL_PUS_PU5KOHM | PAD_CTL_DSE_3P3V_98OHM)

#define NAND_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_SLOW | PAD_CTL_HYS)

#define NAND_PAD_READY0_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUS_PU5KOHM)

#define EPDC_PAD_CTRL	0x0

#define WEAK_PULLUP (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM | \
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#ifdef CONFIG_SYS_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/*
 * Read header information from EEPROM into global structure.
 */
static int read_eeprom(struct smarcfimx7_id *header)
{
        i2c_set_bus_num(1);
        /* Check if baseboard eeprom is available */
        if (i2c_probe(CONFIG_SYS_I2C_EEPROM_ADDR)) {
                puts("Could not probe the EEPROM; something fundamentally "
                        "wrong on the I2C bus.\n");
                return -ENODEV;
        }

        /* read the eeprom using i2c */
        if (i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0, 2, (uchar *)header,
                     sizeof(struct smarcfimx7_id))) {
                puts("Could not read the EEPROM; something fundamentally"
                        " wrong on the I2C bus.\n");
                return -EIO;
        }

        if (header->magic != 0xEE3355AA) {
                /*
                 * read the eeprom using i2c again,
                 * but use only a 1 byte address
                 */
                if (i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0, 1, (uchar *)header,
                             sizeof(struct smarcfimx7_id))) {
                        puts("Could not read the EEPROM; something "
                                "fundamentally wrong on the I2C bus.\n");
                        return -EIO;
                }

                if (header->magic != 0xEE3355AA) {
                        printf("Incorrect magic number (0x%x) in EEPROM\n",
                                        header->magic);
                        return -EINVAL;
                }
        }

        return 0;
}

/* I2C1 for PMIC (I2C_PM)*/
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C1_SCL__I2C1_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C1_SCL__GPIO4_IO8 | PC,
		.gp = IMX_GPIO_NR(4, 8),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C1_SDA__I2C1_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C1_SDA__GPIO4_IO9 | PC,
		.gp = IMX_GPIO_NR(4, 9),
	},
};

/* I2C2 for I2C_GP */
static struct i2c_pads_info i2c_pad_info2 = {
        .scl = {
                .i2c_mode = MX7D_PAD_I2C2_SCL__I2C2_SCL | PC,
                .gpio_mode = MX7D_PAD_I2C2_SCL__GPIO4_IO10 | PC,
                .gp = IMX_GPIO_NR(4, 10),
        },
        .sda = {
                .i2c_mode = MX7D_PAD_I2C2_SDA__I2C2_SDA | PC,
                .gpio_mode = MX7D_PAD_I2C2_SDA__GPIO4_IO11 | PC,
                .gp = IMX_GPIO_NR(4, 11),
        },
};

/* I2C3 for I2C_LCD */
static struct i2c_pads_info i2c_pad_info3 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C3_SCL__I2C3_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C3_SCL__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C3_SDA__I2C3_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C3_SDA__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13),
	},
};

/* I2C4 for I2C_CAM1 */
static struct i2c_pads_info i2c_pad_info4 = {
        .scl = {
                .i2c_mode = MX7D_PAD_I2C4_SCL__I2C4_SCL | PC,
                .gpio_mode = MX7D_PAD_I2C4_SCL__GPIO4_IO14 | PC,
                .gp = IMX_GPIO_NR(4, 14),
        },
        .sda = {
                .i2c_mode = MX7D_PAD_I2C4_SDA__I2C4_SDA | PC,
                .gpio_mode = MX7D_PAD_I2C4_SDA__GPIO4_IO15 | PC,
                .gp = IMX_GPIO_NR(4, 15),
        },
};
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

/* SD Card */
static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX7D_PAD_SD1_CLK__SD1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_CMD__SD1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA0__SD1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA1__SD1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA2__SD1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA3__SD1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	MX7D_PAD_SD1_CD_B__GPIO5_IO0 | MUX_PAD_CTRL(USDHC_PAD_CTRL), 	/*CD */
        MX7D_PAD_SD1_WP__GPIO5_IO1  | MUX_PAD_CTRL(NO_PAD_CTRL), 	/* WP */
	MX7D_PAD_SD1_RESET_B__GPIO5_IO2 | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* SDIO_PWR_EN */
};

/* eMMC */
static iomux_v3_cfg_t const usdhc3_emmc_pads[] = {
	MX7D_PAD_SD3_CLK__SD3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_CMD__SD3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_STROBE__SD3_STROBE	 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_RESET_B__GPIO6_IO11 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
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
        MX7D_PAD_GPIO1_IO09__GPIO1_IO9 | MUX_PAD_CTRL(WEAK_PULLUP), 	/* CHARGER_PRSNT# */
        MX7D_PAD_GPIO1_IO08__GPIO1_IO8 | MUX_PAD_CTRL(WEAK_PULLUP),     /* CHARGING# */
        MX7D_PAD_SAI1_RX_SYNC__GPIO6_IO16 | MUX_PAD_CTRL(WEAK_PULLUP),  /* CARRIER_STBY# */
        MX7D_PAD_SD2_RESET_B__GPIO5_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),   /* BATLOW# */
	MX7D_PAD_EPDC_BDR0__GPIO2_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),   	/* PCIe_RST# */
        MX7D_PAD_EPDC_PWR_STAT__GPIO2_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* PCIe_WAKE# */
        MX7D_PAD_ENET1_CRS__GPIO7_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL), 	/* WDT_TIME_OUT# */
};

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
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 1);

	/* Turn on Backlight */
        gpio_direction_output(IMX_GPIO_NR(6, 17), 1);

        /* Set Brightness to high */
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
                .name          	= "G070VW01",
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
        gpio_direction_output(IMX_GPIO_NR(2, 29) , 0);
}

static void setup_iomux_spi1(void)
{
        imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
        gpio_direction_output(IMX_GPIO_NR(4, 19), 0);
        gpio_direction_output(IMX_GPIO_NR(4, 0), 0);
}

static void setup_iomux_spi3(void)
{
        imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
        gpio_direction_output(IMX_GPIO_NR(6, 22), 0);
        gpio_direction_output(IMX_GPIO_NR(5, 9), 0);
}

static void setup_iomux_gpios(void)
{
        imx_iomux_v3_setup_multiple_pads(gpios_pads, ARRAY_SIZE(gpios_pads));
        gpio_direction_output(IMX_GPIO_NR(2, 0), 0);
        gpio_direction_output(IMX_GPIO_NR(2, 1), 0);
        gpio_direction_output(IMX_GPIO_NR(2, 2), 0);
        gpio_direction_output(IMX_GPIO_NR(2, 3), 0);
        gpio_direction_output(IMX_GPIO_NR(2, 4), 0);
        gpio_direction_input(IMX_GPIO_NR(2, 5));
        gpio_direction_input(IMX_GPIO_NR(2, 7));
        gpio_direction_input(IMX_GPIO_NR(2, 6));
        gpio_direction_input(IMX_GPIO_NR(4, 1));
        gpio_direction_input(IMX_GPIO_NR(4, 6));
        gpio_direction_input(IMX_GPIO_NR(4, 7));
}

static void setup_iomux_lvds_ch_sel(void)
{
        imx_iomux_v3_setup_multiple_pads(lvds_ch_sel_pads, ARRAY_SIZE(lvds_ch_sel_pads));
        gpio_direction_output(IMX_GPIO_NR(5, 13), 0);
}

static void setup_iomux_misc(void)
{
        imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));
        gpio_direction_input(IMX_GPIO_NR(5, 14));
        gpio_direction_input(IMX_GPIO_NR(1, 8));
        gpio_direction_input(IMX_GPIO_NR(1, 9));
        gpio_direction_input(IMX_GPIO_NR(5, 11));
        gpio_direction_output(IMX_GPIO_NR(6, 16), 0);
        gpio_direction_output(IMX_GPIO_NR(2, 28), 0);
       	udelay(500);
        gpio_direction_output(IMX_GPIO_NR(2, 28), 1);
        gpio_direction_input(IMX_GPIO_NR(2, 31));
        /* Set WDT_TIME_OUT# as Output High */
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

#ifdef CONFIG_FSL_ESDHC

#define USDHC1_CD_GPIO	IMX_GPIO_NR(5, 0)
#define USDHC1_PWR_GPIO	IMX_GPIO_NR(5, 2)
#define USDHC3_PWR_GPIO IMX_GPIO_NR(6, 11)

static struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC1_BASE_ADDR, 0, 4},
	{USDHC3_BASE_ADDR},
};

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

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = 1; /* Assume uSDHC3 emmc is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int i, ret;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc2                    USDHC3 (eMMC)
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			gpio_request(USDHC1_CD_GPIO, "usdhc1_cd");
			gpio_direction_input(USDHC1_CD_GPIO);
			gpio_request(USDHC1_PWR_GPIO, "usdhc1_pwr");
			gpio_direction_output(USDHC1_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC1_PWR_GPIO, 1);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_emmc_pads, ARRAY_SIZE(usdhc3_emmc_pads));
			gpio_request(USDHC3_PWR_GPIO, "usdhc3_pwr");
			gpio_direction_output(USDHC3_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC3_PWR_GPIO, 1);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
			}

			ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
			if (ret)
				return ret;
	}

	return 0;
}
#endif

#ifdef CONFIG_FEC_MXC
static iomux_v3_cfg_t const fec1_pads[] = {
	MX7D_PAD_ENET1_RGMII_RX_CTL__ENET1_RGMII_RX_CTL | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RD0__ENET1_RGMII_RD0 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RD1__ENET1_RGMII_RD1 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RD2__ENET1_RGMII_RD2 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RD3__ENET1_RGMII_RD3 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RXC__ENET1_RGMII_RXC | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TX_CTL__ENET1_RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TD0__ENET1_RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TD1__ENET1_RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TD2__ENET1_RGMII_TD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TD3__ENET1_RGMII_TD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TXC__ENET1_RGMII_TXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_GPIO1_IO10__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL_MII),
	MX7D_PAD_GPIO1_IO11__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL_MII),
};

static iomux_v3_cfg_t const fec2_pads[] = {
	MX7D_PAD_EPDC_SDCE0__ENET2_RGMII_RX_CTL | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_EPDC_SDCLK__ENET2_RGMII_RD0 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_EPDC_SDLE__ENET2_RGMII_RD1  | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_EPDC_SDOE__ENET2_RGMII_RD2  | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_EPDC_SDSHR__ENET2_RGMII_RD3 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_EPDC_SDCE1__ENET2_RGMII_RXC | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_EPDC_GDRL__ENET2_RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_EPDC_SDCE2__ENET2_RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_EPDC_SDCE3__ENET2_RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_EPDC_GDCLK__ENET2_RGMII_TD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_EPDC_GDOE__ENET2_RGMII_TD3  | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_EPDC_GDSP__ENET2_RGMII_TXC  | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_GPIO1_IO10__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL_MII),
	MX7D_PAD_GPIO1_IO11__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL_MII),
};

static void setup_iomux_fec(void)
{
	if (0 == CONFIG_FEC_ENET_DEV) {
		imx_iomux_v3_setup_multiple_pads(fec1_pads, ARRAY_SIZE(fec1_pads));
        	gpio_direction_input(IMX_GPIO_NR(7, 15));
	} else {
		imx_iomux_v3_setup_multiple_pads(fec2_pads, ARRAY_SIZE(fec2_pads));
                gpio_direction_input(IMX_GPIO_NR(7, 13));
	}
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
        eeprom_mac_read = smarcfimx7_read_mac_address(enetaddr);

        /*
         * MAC address not present in the environment
         * try and read the MAC address from EEPROM flash
         * and set it.
         */
        if (!enetaddr_found) {
                if (eeprom_mac_read)
                        /* Set Ethernet MAC address from EEPROM */
                        smarcfimx7_sync_env_enetaddr(enetaddr);
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

        uchar env_enet1addr[6];
        int enet1addr_found;

        enet1addr_found = eth_getenv_enetaddr("eth1addr", env_enet1addr);

        uint8_t enet1addr[8];
        int eeprom_mac1_read;

        /* Read Ethernet MAC address from EEPROM */
        eeprom_mac1_read = smarcfimx7_read_mac1_address(enet1addr);

        /*
         * MAC address not present in the environment
         * try and read the MAC address from EEPROM flash
         * and set it.
         */
        if (!enet1addr_found) {
                if (eeprom_mac1_read)
                        /* Set Ethernet MAC address from EEPROM */
                        smarcfimx7_sync_env_enet1addr(enet1addr);
        } else {
                /*
                 * MAC address present in environment compare it with
                 * the MAC address in EEPROM and warn on mismatch
                 */
                if (eeprom_mac_read && memcmp(enet1addr, env_enet1addr, 6))
                        printf("Warning: 2nd GBE MAC address in EEPROM don't match "
                                        "with the MAC address in the environment\n");
                        printf("Default using 2nd GBE MAC address from environment\n");
        }

#endif

	int ret;

	setup_iomux_fec();

	ret = fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
		CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
	if (ret)
		printf("FEC1 MXC: %s:failed\n", __func__);

	return ret;
}

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

        return set_clk_enet(ENET_125MHz);

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

#ifdef CONFIG_MXC_SPI
/* SPI2 (SPINOR) */
static iomux_v3_cfg_t const ecspi2_pads[] = {
        MX7D_PAD_ECSPI2_SCLK__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX7D_PAD_ECSPI2_MOSI__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX7D_PAD_ECSPI2_MISO__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX7D_PAD_ECSPI2_SS0__GPIO4_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL),	/*SS0#*/
};

static void setup_spinor(void)
{
        imx_iomux_v3_setup_multiple_pads(ecspi2_pads, ARRAY_SIZE(ecspi2_pads));
        gpio_direction_output(IMX_GPIO_NR(4, 23), 0);
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
        return (bus == 1 && cs == 0) ? (IMX_GPIO_NR(4, 23)) : -1;
}
#endif

#ifdef CONFIG_MXC_EPDC
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
	gpio_direction_input(IMX_GPIO_NR(2, 31));

	/* EPDC_VCOM0 - GPIO4[14] for VCOM control */
	imx_iomux_v3_setup_pad(MX7D_PAD_I2C4_SCL__GPIO4_IO14 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/* Set as output */
	gpio_direction_output(IMX_GPIO_NR(4, 14), 1);

	/* EPDC_PWRWAKEUP - GPIO2[23] for EPD PMIC WAKEUP */
	imx_iomux_v3_setup_pad(MX7D_PAD_EPDC_SDCE3__GPIO2_IO23 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));
	/* Set as output */
	gpio_direction_output(IMX_GPIO_NR(2, 23), 1);

	/* EPDC_PWRCTRL0 - GPIO2[30] for EPD PWR CTL0 */
	imx_iomux_v3_setup_pad(MX7D_PAD_EPDC_PWR_COM__GPIO2_IO30 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));
	/* Set as output */
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

#ifdef CONFIG_USB_EHCI_MX7
static iomux_v3_cfg_t const usb_otg1_pads[] = {
        MX7D_PAD_GPIO1_IO12__USB_OTG1_ID | MUX_PAD_CTRL(WEAK_PULLUP),
	/* OTG1 Power Enable */
        MX7D_PAD_GPIO1_IO05__USB_OTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* OTG1 Over Current */
        MX7D_PAD_GPIO1_IO04__GPIO1_IO4 | MUX_PAD_CTRL(WEAK_PULLUP),

};

static iomux_v3_cfg_t const usb_otg2_pads[] = {
        /* OTG2 Power Enable */
        MX7D_PAD_GPIO1_IO07__USB_OTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
        /* OTG2 Over Current */
        MX7D_PAD_GPIO1_IO06__GPIO1_IO6 | MUX_PAD_CTRL(WEAK_PULLUP),
};

static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg1_pads,
						 ARRAY_SIZE(usb_otg1_pads));
        gpio_direction_input(IMX_GPIO_NR(1, 4));

	imx_iomux_v3_setup_multiple_pads(usb_otg2_pads,
						 ARRAY_SIZE(usb_otg2_pads));
        gpio_direction_input(IMX_GPIO_NR(1, 6));
}

int board_usb_phy_mode(int port)
{
	if (port == 0)
		return usb_phy_mode(port);
	else
		return USB_INIT_HOST;
}
#endif

int board_early_init_f(void)
{
        setup_iomux_reset_out();
	setup_iomux_uart6();
        setup_iomux_uart2();
        setup_iomux_uart7();
        setup_iomux_uart3();

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
        setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);
        setup_i2c(3, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info4);
#endif

#ifdef CONFIG_MXC_SPI
        setup_spinor();
#endif

#ifdef CONFIG_USB_EHCI_MX7
	setup_usb();
#endif
        setup_iomux_spi1();
        setup_iomux_spi3();
        setup_iomux_flexcan1();
        setup_iomux_flexcan2();
        setup_iomux_gpios();
	setup_iomux_lvds_ch_sel();
        setup_iomux_misc();

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

#ifdef CONFIG_POWER
#define I2C_PMIC	0
int power_init_board(void)
{
	struct pmic *p;
	int ret;
	unsigned int reg, rev_id;

	ret = power_pfuze3000_init(I2C_PMIC);
	if (ret)
		return ret;

	p = pmic_get("PFUZE3000");
	ret = pmic_probe(p);
	if (ret)
		return ret;

	pmic_reg_read(p, PFUZE3000_DEVICEID, &reg);
	pmic_reg_read(p, PFUZE3000_REVID, &rev_id);
	printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", reg, rev_id);

	/* disable Low Power Mode during standby mode */
	pmic_reg_read(p, PFUZE3000_LDOGCTL, &reg);
	reg |= 0x1;
	pmic_reg_write(p, PFUZE3000_LDOGCTL, reg);

	/* SW1A/1B mode set to APS/APS */
	reg = 0x8;
	pmic_reg_write(p, PFUZE3000_SW1AMODE, reg);
	pmic_reg_write(p, PFUZE3000_SW1BMODE, reg);

	/* SW1A/1B standby voltage set to 0.975V */
	reg = 0xb;
	pmic_reg_write(p, PFUZE3000_SW1ASTBY, reg);
	pmic_reg_write(p, PFUZE3000_SW1BSTBY, reg);

	/* decrease SW1B normal voltage to 0.975V */
	pmic_reg_read(p, PFUZE3000_SW1BVOLT, &reg);
	reg &= ~0x1f;
	reg |= PFUZE3000_SW1AB_SETP(975);
	pmic_reg_write(p, PFUZE3000_SW1BVOLT, reg);

	return 0;
}
#endif

int board_late_init(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

/* Check Board Information */
        setup_i2c(1, CONFIG_SYS_I2C_SPEED,
                        0x50, &i2c_pad_info2);

        struct smarcfimx7_id header;

        if (read_eeprom(&header) < 0)
        puts("Could not get board ID.\n");

        puts("---------Embedian SMARC-FiMX7------------\n");
        printf("Board ID:               %.*s\n",
               sizeof(header.name), header.name);
        printf("Board Revision:         %.*s\n",
               sizeof(header.version), header.version);
        printf("Board Serial#:          %.*s\n",
               sizeof(header.serial), header.serial);
        puts("-----------------------------------------\n");

/* SMARC BOOT_SEL*/
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
                setenv_ulong("mmcdev", 0);
                setenv("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 0)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 1)) {
                puts("BOOT_SEL Detected: OFF ON ON, Load zImage from Module eMMC Flash...\n");
                setenv_ulong("mmcdev", 1);
                setenv("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 0)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 1)) {
                puts("BOOT_SEL Detected: ON OFF ON, Load zImage from GBE...\n");
                setenv("bootcmd", "run netboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 0)) {
                puts("BOOT_SEL Detected: OFF ON OFF, unsupported boot up device: Carrier SPI...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(5, 15)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 16)) == 1)&&(gpio_get_value(IMX_GPIO_NR(5, 17)) == 1)) {
                puts("BOOT_SEL Detected: ON ON ON, MOdule SPI Boot up is Default, Load zImage from Module eMMC...\n");
                setenv_ulong("mmcdev", 1);
                setenv("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
        } else {
                puts("unsupported boot devices\n");
                hang();
        }

        return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case MMC1_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc0");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc1");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("unsupported boot devices\n");
		break;
	}
}

#ifdef CONFIG_ANDROID_RECOVERY

/* Use LID# for recovery key */
#define GPIO_VOL_DN_KEY IMX_GPIO_NR(5, 10)
iomux_v3_cfg_t const recovery_key_pads[] = {
	(MX7D_PAD_SD2_WP__GPIO5_IO10 | MUX_PAD_CTRL(BUTTON_PAD_CTRL)),
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
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case MMC1_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "boota mmc0 recovery");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "boota mmc1 recovery");
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

#endif /*CONFIG_FSL_FASTBOOT*/

