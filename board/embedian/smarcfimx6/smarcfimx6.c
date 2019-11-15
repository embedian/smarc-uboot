/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/mach-imx/spi.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <pwm.h>
#include <i2c.h>
#include <input.h>
#include <usb.h>
#include <usb/ehci-ci.h>
#include <asm/arch/mx6-ddr.h>
#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#ifdef CONFIG_SATA
#include <asm/mach-imx/sata.h>
#endif
#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

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

#define WEAK_PULLUP (PAD_CTL_PUS_100K_UP |                      \
        PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
        PAD_CTL_SRE_SLOW)

#define I2C_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |               \
        PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
        PAD_CTL_DSE_40ohm | PAD_CTL_HYS |                       \
        PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define LCD_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_HIGH | \
                      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define EPDC_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)


#define WEAK_PULLUP (PAD_CTL_PUS_100K_UP |                      \
        PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
        PAD_CTL_SRE_SLOW)

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

#define DISP0_PWR_EN	IMX_GPIO_NR(1, 02)

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

/* SER0/UART1 */
iomux_v3_cfg_t const uart1_pads[] = {
        IOMUX_PADS(PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
        IOMUX_PADS(PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
        IOMUX_PADS(PAD_EIM_D20__UART1_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL)),
        IOMUX_PADS(PAD_EIM_D19__UART1_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

/* SER1/UART2 */
iomux_v3_cfg_t const uart2_pads[] = {
        IOMUX_PADS(PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
        IOMUX_PADS(PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

/* SER2/UART4 */
iomux_v3_cfg_t const uart4_pads[] = {
        IOMUX_PADS(PAD_CSI0_DAT12__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
        IOMUX_PADS(PAD_CSI0_DAT13__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
        IOMUX_PADS(PAD_CSI0_DAT16__UART4_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL)),
        IOMUX_PADS(PAD_CSI0_DAT17__UART4_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

/* SER3/UART5 Default Debug Port */
iomux_v3_cfg_t const uart5_pads[] = {
        IOMUX_PADS(PAD_CSI0_DAT14__UART5_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
        IOMUX_PADS(PAD_CSI0_DAT15__UART5_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

iomux_v3_cfg_t const wdt_pads[] = {
        IOMUX_PADS(PAD_EIM_D16__GPIO3_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

iomux_v3_cfg_t const reset_out_pads[] = {
        IOMUX_PADS(PAD_NANDF_CS3__GPIO6_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const enet_pads[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
};

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads);
        gpio_request(IMX_GPIO_NR(4, 11), "ETH0_IRQ");
        gpio_direction_input(IMX_GPIO_NR(4, 11));
}

/* SDIO */
static iomux_v3_cfg_t const usdhc2_pads[] = {
	IOMUX_PADS(PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_ENET_TX_EN__GPIO1_IO28  | MUX_PAD_CTRL(NO_PAD_CTRL)),    /* CD */
        IOMUX_PADS(PAD_ENET_CRS_DV__GPIO1_IO25  | MUX_PAD_CTRL(NO_PAD_CTRL)),   /* WP */
        IOMUX_PADS(PAD_SD1_CMD__GPIO1_IO18  | MUX_PAD_CTRL(NO_PAD_CTRL)),       /* SDIO_PWR_EN */
};

/* SDMMC */
iomux_v3_cfg_t const usdhc3_pads[] = {
        IOMUX_PADS(PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD3_RST__SD3_RESET | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

/* eMMC */
static iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

#ifdef CONFIG_MXC_SPI
/* SPI0 */
iomux_v3_cfg_t const ecspi2_pads[] = {
        IOMUX_PADS(PAD_CSI0_DAT8__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),
        IOMUX_PADS(PAD_CSI0_DAT10__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL)),
        IOMUX_PADS(PAD_CSI0_DAT9__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
        IOMUX_PADS(PAD_CSI0_DAT11__GPIO5_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL)),     /*SS0#*/
        IOMUX_PADS(PAD_EIM_D24__GPIO3_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL)),        /*SS2#*/
        IOMUX_PADS(PAD_EIM_D25__GPIO3_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL)),        /*SS3#*/
};

static void setup_spinor(void)
{
	SETUP_IOMUX_PADS(ecspi2_pads);
        gpio_request(IMX_GPIO_NR(5, 29), "ECSPI2 SS0");
        gpio_request(IMX_GPIO_NR(3, 24), "ECSPI2 SS2");
        gpio_request(IMX_GPIO_NR(3, 25), "ECSPI2 SS3");
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
        return (bus == 1 && cs == 0) ? (IMX_GPIO_NR(5, 29)) : -1;
        return (bus == 1 && cs == 2) ? (IMX_GPIO_NR(3, 24)) : -1;
        return (bus == 1 && cs == 3) ? (IMX_GPIO_NR(3, 25)) : -1;
}
#endif

/* SPI1 */
        iomux_v3_cfg_t const ecspi1_pads[] = {
        IOMUX_PADS(PAD_KEY_COL0__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),
        IOMUX_PADS(PAD_KEY_COL1__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL)),
        IOMUX_PADS(PAD_KEY_ROW0__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
        IOMUX_PADS(PAD_KEY_ROW1__GPIO4_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL)),       /*SS0#*/
        IOMUX_PADS(PAD_KEY_COL2__GPIO4_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL)),       /*SS1#*/
};

static iomux_v3_cfg_t const rgb_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
        IOMUX_PADS(PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(LCD_PAD_CTRL)), /* DISP0_DRDY */
	IOMUX_PADS(PAD_DISP0_DAT1__IPU1_DISP0_DATA01 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT2__IPU1_DISP0_DATA02 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT3__IPU1_DISP0_DATA03 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT4__IPU1_DISP0_DATA04 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT5__IPU1_DISP0_DATA05 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT6__IPU1_DISP0_DATA06 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT7__IPU1_DISP0_DATA07 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT8__IPU1_DISP0_DATA08 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT9__IPU1_DISP0_DATA09 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT10__IPU1_DISP0_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT11__IPU1_DISP0_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT12__IPU1_DISP0_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT13__IPU1_DISP0_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT14__IPU1_DISP0_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT15__IPU1_DISP0_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT16__IPU1_DISP0_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT17__IPU1_DISP0_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT18__IPU1_DISP0_DATA18 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT19__IPU1_DISP0_DATA19 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT20__IPU1_DISP0_DATA20 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT21__IPU1_DISP0_DATA21 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT22__IPU1_DISP0_DATA22 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT23__IPU1_DISP0_DATA23 | MUX_PAD_CTRL(LCD_PAD_CTRL)),
        /* LCD VDD Enable(for parallel LCD): S133 */
        IOMUX_PADS(PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const backlight_pads[] = {
        /* Backlight Enable for RGB: S127 */
#define BACKLIGHT_EN IMX_GPIO_NR(1, 00)
        IOMUX_PADS(PAD_GPIO_0__GPIO1_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL)),
        /* PWM Backlight Control: S141 */
        IOMUX_PADS(PAD_GPIO_1__PWM2_OUT | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void enable_backlight(void)
{
        SETUP_IOMUX_PADS(backlight_pads);
        gpio_request(DISP0_PWR_EN, "Display Power Enable");
        gpio_direction_output(DISP0_PWR_EN, 1);
        /* enable backlight PWM 2 */
        if (pwm_init(1, 0, 0))
                goto error;
        /* duty cycle 500ns, period: 3000ns */
        if (pwm_config(1, 1000, 3000))
                goto error;
        if (pwm_enable(1))
                goto error;
        return;

error:
        puts("error init pwm for backlight\n");
        return;
}

static void enable_rgb(struct display_info_t const *dev)
{
	SETUP_IOMUX_PADS(rgb_pads);

        gpio_request(DISP0_PWR_EN, "Display Power Enable");
        gpio_direction_output(DISP0_PWR_EN, 1);
        enable_backlight();
}

static void enable_lvds(struct display_info_t const *dev)
{
        struct iomuxc *iomux = (struct iomuxc *)
                                IOMUXC_BASE_ADDR;
        u32 reg = readl(&iomux->gpr[2]);
        reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
        writel(reg, &iomux->gpr[2]);

	enable_backlight();
}

#ifdef CONFIG_SYS_I2C
/*I2C1 I2C_PM*/
struct i2c_pads_info i2c_pad_info1 = {
        .scl = {
                .i2c_mode = MX6_PAD_EIM_D21__I2C1_SCL | I2C_PAD,
                .gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | I2C_PAD,
                .gp = IMX_GPIO_NR(3, 21)
        },
        .sda = {
                .i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | I2C_PAD,
                .gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | I2C_PAD,
                .gp = IMX_GPIO_NR(3, 28)
        }
};

/* I2C2 HDMI */
struct i2c_pads_info i2c_pad_info2 = {
        .scl = {
                .i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | I2C_PAD,
                .gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | I2C_PAD,
                .gp = IMX_GPIO_NR(4, 12)
        },
        .sda = {
                .i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | I2C_PAD,
                .gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | I2C_PAD,
                .gp = IMX_GPIO_NR(4, 13)
        }
};

/* I2C3 TCA9546APWR */
struct i2c_pads_info i2c_pad_info3 = {
        .scl = {
                .i2c_mode = MX6_PAD_EIM_D17__I2C3_SCL | I2C_PAD,
                .gpio_mode = MX6_PAD_EIM_D17__GPIO3_IO17 | I2C_PAD,
                .gp = IMX_GPIO_NR(3, 17)
        },
        .sda = {
                .i2c_mode = MX6_PAD_EIM_D18__I2C3_SDA | I2C_PAD,
                .gpio_mode = MX6_PAD_EIM_D18__GPIO3_IO18 | I2C_PAD,
                .gp = IMX_GPIO_NR(3, 18)
        }
};
#endif

#ifdef CONFIG_PCIE_IMX
iomux_v3_cfg_t const pcie_pads[] = {
        IOMUX_PADS(PAD_SD1_DAT1__GPIO1_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL)),       /* PCIe Present */
        IOMUX_PADS(PAD_SD1_DAT2__GPIO1_IO19 | MUX_PAD_CTRL(WEAK_PULLUP)),       /* PCIe_WAKE# */
        IOMUX_PADS(PAD_SD1_CLK__GPIO1_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL)),        /* RESET */
        IOMUX_PADS(PAD_SD1_DAT0__GPIO1_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL)),       /* PCIe Clock Request */
};

static void setup_pcie(void)
{
        SETUP_IOMUX_PADS(pcie_pads);
        gpio_request(IMX_GPIO_NR(1, 16), "PCIE_A_CLK_REQ");
        gpio_direction_input(IMX_GPIO_NR(1, 16));
        gpio_request(IMX_GPIO_NR(1, 17), "PCIE_A_PRSNT");
        gpio_direction_input(IMX_GPIO_NR(1, 17));
        gpio_request(IMX_GPIO_NR(1, 19), "PCIE_A_WAKE");
        gpio_direction_input(IMX_GPIO_NR(1, 19));
        gpio_request(IMX_GPIO_NR(1, 20), "PCIE_RST#");
        gpio_direction_output(IMX_GPIO_NR(1, 20), 0);
        udelay(500);
        gpio_direction_output(IMX_GPIO_NR(1, 20), 1);
}
#endif

iomux_v3_cfg_t const di0_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK),	/* DISP0_CLK */
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02),		/* DISP0_HSYNC */
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03),		/* DISP0_VSYNC */
};

/* CAN0/FLEXCAN1 */
iomux_v3_cfg_t const flexcan1_pads[] = {

        IOMUX_PADS(PAD_GPIO_7__FLEXCAN1_TX | MUX_PAD_CTRL(WEAK_PULLUP)),
        IOMUX_PADS(PAD_GPIO_8__FLEXCAN1_RX | MUX_PAD_CTRL(WEAK_PULLUP)),
};

/* CAN1/FLEXCAN2 */
iomux_v3_cfg_t const flexcan2_pads[] = {
        IOMUX_PADS(PAD_KEY_COL4__FLEXCAN2_TX | MUX_PAD_CTRL(WEAK_PULLUP)),
        IOMUX_PADS(PAD_KEY_ROW4__FLEXCAN2_RX | MUX_PAD_CTRL(WEAK_PULLUP)),
};

/* GPIOs */
iomux_v3_cfg_t const gpios_pads[] = {
        IOMUX_PADS(PAD_NANDF_CS0__GPIO6_IO11 | MUX_PAD_CTRL(WEAK_PULLUP)),      /* GPIO0 */
        IOMUX_PADS(PAD_NANDF_D2__GPIO2_IO02 | MUX_PAD_CTRL(WEAK_PULLUP)),       /* GPIO1 */
        IOMUX_PADS(PAD_NANDF_D6__GPIO2_IO06 | MUX_PAD_CTRL(WEAK_PULLUP)),       /* GPIO2 */
        IOMUX_PADS(PAD_NANDF_D3__GPIO2_IO03 | MUX_PAD_CTRL(WEAK_PULLUP)),       /* GPIO3 */
        IOMUX_PADS(PAD_NANDF_D7__GPIO2_IO07 | MUX_PAD_CTRL(WEAK_PULLUP)),       /* GPIO4 */
        IOMUX_PADS(PAD_NANDF_CS1__GPIO6_IO14 | MUX_PAD_CTRL(WEAK_PULLUP)),      /* GPIO6 */
        IOMUX_PADS(PAD_NANDF_CLE__GPIO6_IO07 | MUX_PAD_CTRL(WEAK_PULLUP)),      /* GPIO7 */
        IOMUX_PADS(PAD_NANDF_D4__GPIO2_IO04 | MUX_PAD_CTRL(WEAK_PULLUP)),       /* GPIO8 */
        IOMUX_PADS(PAD_NANDF_D0__GPIO2_IO00 | MUX_PAD_CTRL(WEAK_PULLUP)),       /* GPIO9 */
        IOMUX_PADS(PAD_NANDF_D5__GPIO2_IO05 | MUX_PAD_CTRL(WEAK_PULLUP)),       /* GPIO10 */
        IOMUX_PADS(PAD_NANDF_ALE__GPIO6_IO08 | MUX_PAD_CTRL(WEAK_PULLUP)),      /* GPIO11 */
};

/* Misc. pins */
static iomux_v3_cfg_t const misc_pads[] = {
        IOMUX_PADS(PAD_EIM_CS0__GPIO2_IO23 | MUX_PAD_CTRL(WEAK_PULLUP)),	/* SLEEP# */
        IOMUX_PADS(PAD_EIM_D22__GPIO3_IO22 | MUX_PAD_CTRL(WEAK_PULLUP)),      	/* CHARGER_PRSNT# */
        IOMUX_PADS(PAD_EIM_CS1__GPIO2_IO24 | MUX_PAD_CTRL(WEAK_PULLUP)),      	/* CHARGING# */
        IOMUX_PADS(PAD_GPIO_18__GPIO7_IO13 | MUX_PAD_CTRL(WEAK_PULLUP)),  	/* CARRIER_STBY# */
        IOMUX_PADS(PAD_EIM_D29__GPIO3_IO29 | MUX_PAD_CTRL(WEAK_PULLUP)),   	/* BATLOW# */
};

static void setup_iomux_uart1(void)
{
        SETUP_IOMUX_PADS(uart1_pads);
}

static void setup_iomux_uart2(void)
{
        SETUP_IOMUX_PADS(uart2_pads);
}

static void setup_iomux_uart4(void)
{
        SETUP_IOMUX_PADS(uart4_pads);
}

static void setup_iomux_uart5(void)
{
        SETUP_IOMUX_PADS(uart5_pads);
}

static void setup_iomux_wdt(void)
{
        SETUP_IOMUX_PADS(wdt_pads);
        /* Set HW_WDT as Output High*/
        gpio_request(IMX_GPIO_NR(3, 16), "WDT_ENABLE");
        gpio_direction_output(IMX_GPIO_NR(3, 16) , 1);
}

static void setup_iomux_reset_out(void)
{
        SETUP_IOMUX_PADS(reset_out_pads);
        /* Set CPU RESET_OUT as Output */
        gpio_request(IMX_GPIO_NR(6, 16), "CPU_RESET");
        gpio_direction_output(IMX_GPIO_NR(6, 16) , 0);
}

static void setup_spi1(void)
{
        SETUP_IOMUX_PADS(ecspi1_pads);
        gpio_request(IMX_GPIO_NR(4, 9), "EXSPI1_SS0");
        gpio_request(IMX_GPIO_NR(4, 10), "ECSPI1_SS1");
}

int board_spi1_cs_gpio(unsigned bus, unsigned cs)
{
        return (bus == 0 && cs == 0) ? (IMX_GPIO_NR(4, 9)) : -1;
        return (bus == 0 && cs == 1) ? (IMX_GPIO_NR(4, 10)) : -1;
}

static void setup_flexcan1(void)
{
        SETUP_IOMUX_PADS(flexcan1_pads);
}

static void setup_flexcan2(void)
{
        SETUP_IOMUX_PADS(flexcan2_pads);
}

static void setup_gpios(void)
{
        SETUP_IOMUX_PADS(gpios_pads);
        gpio_request(IMX_GPIO_NR(6, 11), "GPIO0");
        gpio_direction_output(IMX_GPIO_NR(6, 11), 0);
        gpio_request(IMX_GPIO_NR(2, 02), "GPIO1");
        gpio_direction_output(IMX_GPIO_NR(2, 02), 0);
        gpio_request(IMX_GPIO_NR(2, 06), "GPIO2");
        gpio_direction_output(IMX_GPIO_NR(2, 06), 0);
        gpio_request(IMX_GPIO_NR(2, 03), "GPIO3");
        gpio_direction_output(IMX_GPIO_NR(2, 03), 0);
        gpio_request(IMX_GPIO_NR(2, 07), "GPIO4");
        gpio_direction_output(IMX_GPIO_NR(2, 07), 0);
        gpio_request(IMX_GPIO_NR(6, 14), "GPIO6");
        gpio_direction_input(IMX_GPIO_NR(6, 14));
        gpio_request(IMX_GPIO_NR(6, 07), "GPIO7");
        gpio_direction_input(IMX_GPIO_NR(6, 07));
        gpio_request(IMX_GPIO_NR(2, 04), "GPIO8");
        gpio_direction_input(IMX_GPIO_NR(2, 04));
        gpio_request(IMX_GPIO_NR(2, 00), "GPIO9");
        gpio_direction_input(IMX_GPIO_NR(2, 00));
        gpio_request(IMX_GPIO_NR(2, 05), "GPIO10");
        gpio_direction_input(IMX_GPIO_NR(2, 05));
        gpio_request(IMX_GPIO_NR(6, 8), "GPIO11");
        gpio_direction_input(IMX_GPIO_NR(6, 8));
}

static void setup_misc(void)
{
        SETUP_IOMUX_PADS(misc_pads);
        gpio_request(IMX_GPIO_NR(2, 23), "SLEEP#");
        gpio_direction_input(IMX_GPIO_NR(2, 23));
        gpio_request(IMX_GPIO_NR(2, 24), "CHARGING#");
        gpio_direction_input(IMX_GPIO_NR(2, 24));
        gpio_request(IMX_GPIO_NR(3, 22), "CHARGER_PRSNT#");
        gpio_direction_input(IMX_GPIO_NR(3, 22));
        gpio_request(IMX_GPIO_NR(3, 29), "BATLOW#");
        gpio_direction_input(IMX_GPIO_NR(3, 29));
        gpio_request(IMX_GPIO_NR(7, 13), "CARRIER_STBY#");
        gpio_direction_output(IMX_GPIO_NR(7, 13), 1);
}

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
static iomux_v3_cfg_t const epdc_enable_pads[] = {
	IOMUX_PADS(PAD_EIM_A16__EPDC_DATA00	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA10__EPDC_DATA01	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA12__EPDC_DATA02	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA11__EPDC_DATA03	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_LBA__EPDC_DATA04	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_EB2__EPDC_DATA05	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_CS0__EPDC_DATA06	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_RW__EPDC_DATA07	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A21__EPDC_GDCLK	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A22__EPDC_GDSP	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A23__EPDC_GDOE	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A24__EPDC_GDRL	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D31__EPDC_SDCLK_P	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D27__EPDC_SDOE	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA1__EPDC_SDLE	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_EB1__EPDC_SDSHR	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA2__EPDC_BDR0	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA4__EPDC_SDCE0	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA5__EPDC_SDCE1	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA6__EPDC_SDCE2	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
};

static iomux_v3_cfg_t const epdc_disable_pads[] = {
	IOMUX_PADS(PAD_EIM_A16__GPIO2_IO22),
	IOMUX_PADS(PAD_EIM_DA10__GPIO3_IO10),
	IOMUX_PADS(PAD_EIM_DA12__GPIO3_IO12),
	IOMUX_PADS(PAD_EIM_DA11__GPIO3_IO11),
	IOMUX_PADS(PAD_EIM_LBA__GPIO2_IO27),
	IOMUX_PADS(PAD_EIM_EB2__GPIO2_IO30),
	IOMUX_PADS(PAD_EIM_CS0__GPIO2_IO23),
	IOMUX_PADS(PAD_EIM_RW__GPIO2_IO26),
	IOMUX_PADS(PAD_EIM_A21__GPIO2_IO17),
	IOMUX_PADS(PAD_EIM_A22__GPIO2_IO16),
	IOMUX_PADS(PAD_EIM_A23__GPIO6_IO06),
	IOMUX_PADS(PAD_EIM_A24__GPIO5_IO04),
	IOMUX_PADS(PAD_EIM_D31__GPIO3_IO31),
	IOMUX_PADS(PAD_EIM_D27__GPIO3_IO27),
	IOMUX_PADS(PAD_EIM_DA1__GPIO3_IO01),
	IOMUX_PADS(PAD_EIM_EB1__GPIO2_IO29),
	IOMUX_PADS(PAD_EIM_DA2__GPIO3_IO02),
	IOMUX_PADS(PAD_EIM_DA4__GPIO3_IO04),
	IOMUX_PADS(PAD_EIM_DA5__GPIO3_IO05),
	IOMUX_PADS(PAD_EIM_DA6__GPIO3_IO06),
};
#endif

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(1, 28)

int board_mmc_get_env_dev(int devno)
{
	return devno - 1;
}

int mmc_map_to_kernel_blk(int devno)
{
	return devno + 1;
}

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
#ifndef CONFIG_SPL_BUILD
	int ret;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    SDIO
	 * mmc1                    SDMMC
	 * mmc2                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			SETUP_IOMUX_PADS(usdhc2_pads);
			gpio_request(USDHC2_CD_GPIO, "USDHC2 CD");
			gpio_direction_input(USDHC2_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		case 1:
			SETUP_IOMUX_PADS(usdhc3_pads);
			/*gpio_request(USDHC3_CD_GPIO, "USDHC3 CD");
			gpio_direction_input(USDHC3_CD_GPIO);*/
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		case 2:
			SETUP_IOMUX_PADS(usdhc4_pads);
			usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
#else
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD1
	 * 0x2                  SD2
	 * 0x3                  SD4
	 */

	switch (reg & 0x3) {
	case 0x1:
		SETUP_IOMUX_PADS(usdhc2_pads);
		usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x2:
		SETUP_IOMUX_PADS(usdhc3_pads);
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x3:
		SETUP_IOMUX_PADS(usdhc4_pads);
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}
#endif

static int ar8031_phy_fixup(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	if (!is_mx6dqp()) {
		phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
		phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
		phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

		val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
		val &= 0xffe3;
		val |= 0x18;
		phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);
	}

	/* set the IO voltage to 1.8v */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	ar8031_phy_fixup(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
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

static iomux_v3_cfg_t const epdc_pwr_ctrl_pads[] = {
	IOMUX_PADS(PAD_EIM_A17__GPIO2_IO21	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D17__GPIO3_IO17	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D20__GPIO3_IO20	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A18__GPIO2_IO20	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
};

static void setup_epdc_power(void)
{
	SETUP_IOMUX_PADS(epdc_pwr_ctrl_pads);

	/* Setup epdc voltage */

	/* EIM_A17 - GPIO2[21] for PWR_GOOD status */
	/* Set as input */
	gpio_request(IMX_GPIO_NR(2, 21), "EPDC PWRSTAT");
	gpio_direction_input(IMX_GPIO_NR(2, 21));

	/* EIM_D17 - GPIO3[17] for VCOM control */
	/* Set as output */
	gpio_request(IMX_GPIO_NR(3, 17), "EPDC VCOM0");
	gpio_direction_output(IMX_GPIO_NR(3, 17), 1);

	/* EIM_D20 - GPIO3[20] for EPD PMIC WAKEUP */
	/* Set as output */
	gpio_request(IMX_GPIO_NR(3, 20), "EPDC PWR WAKEUP");
	gpio_direction_output(IMX_GPIO_NR(3, 20), 1);

	/* EIM_A18 - GPIO2[20] for EPD PWR CTL0 */
	/* Set as output */
	gpio_request(IMX_GPIO_NR(2, 20), "EPDC PWR CTRL0");
	gpio_direction_output(IMX_GPIO_NR(2, 20), 1);
}

static void epdc_enable_pins(void)
{
	/* epdc iomux settings */
	SETUP_IOMUX_PADS(epdc_enable_pads);
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to GPIO */
	SETUP_IOMUX_PADS(epdc_disable_pads);
}

static void setup_epdc(void)
{
	unsigned int reg;
	struct mxc_ccm_reg *ccm_regs = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

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
static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15384,
		.left_margin    = 160,
		.right_margin   = 24,
		.upper_margin   = 29,
		.lower_margin   = 3,
		.hsync_len      = 136,
		.vsync_len      = 6,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.pixclock       = 39721,
		.left_margin    = 48,
		.right_margin   = 16,
		.upper_margin   = 33,
		.lower_margin   = 10,
		.hsync_len      = 96,
		.vsync_len      = 2,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "SEIKO-WVGA",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 29850,
		.left_margin    = 89,
		.right_margin   = 164,
		.upper_margin   = 23,
		.lower_margin   = 10,
		.hsync_len      = 10,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	SETUP_IOMUX_PADS(di0_pads);

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
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

static void setup_fec(void)
{
	if (is_mx6dqp()) {
		int ret;

		/* select ENET MAC0 TX clock from PLL */
		imx_iomux_set_gpr_register(5, 9, 1, 1);
		ret = enable_fec_anatop_clock(0, ENET_125MHZ);
		if (ret)
		    printf("Error fec anatop clock settings!\n");
	}
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();
	return cpu_eth_init(bis);
}

#ifdef CONFIG_USB_EHCI_MX6
#ifdef CONFIG_DM_USB
int board_ehci_hcd_init(int port)
{
	switch (port) {
	case 0:
		/*
		  * Set daisy chain for otg_pin_id on 6q.
		 *  For 6dl, this bit is reserved.
		 */
		imx_iomux_set_gpr_register(1, 13, 1, 0);
		break;
	case 1:
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}
	return 0;
}
#else
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	/* OTG Over Current */
        IOMUX_PADS(PAD_ENET_TXD0__GPIO1_IO30   | MUX_PAD_CTRL(WEAK_PULLUP)),
	/* OTG ID */
        IOMUX_PADS(PAD_ENET_RX_ER__USB_OTG_ID  | MUX_PAD_CTRL(WEAK_PULLUP)),
        /* OTG Power enable */
        IOMUX_PADS(PAD_ENET_TXD1__GPIO1_IO29   | MUX_PAD_CTRL(OUTPUT_40OHM)),
};

static iomux_v3_cfg_t const usb_hc1_pads[] = {
	/* USB1 Over Current */
        IOMUX_PADS(PAD_ENET_RXD0__GPIO1_IO27   | MUX_PAD_CTRL(WEAK_PULLUP)),
        /* USB1 Power enable */
        IOMUX_PADS(MX6_PAD_ENET_RXD1__GPIO1_IO26   | MUX_PAD_CTRL(OUTPUT_40OHM)),
};

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	switch (port) {
	case 0:
		SETUP_IOMUX_PADS(usb_otg_pads);
        	gpio_request(IMX_GPIO_NR(1, 29), "USB OTG Power Enable")
		gpio_request(IMX_GPIO_NR(1, 30), "USB OTG Over Current")

		/*
		  * Set daisy chain for otg_pin_id on 6q.
		 *  For 6dl, this bit is reserved.
		 */
		imx_iomux_set_gpr_register(1, 13, 1, 0);

		usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

		setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);
		break;
	case 1:
		SETUP_IOMUX_PADS(usb_hc1_pads);
		gpio_request(IMX_GPIO_NR(1, 26), "USB HC1 Power Enable");
                gpio_request(IMX_GPIO_NR(1, 27), "USB HC1 Over Current");
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
                if (on)
                        gpio_direction_output(IMX_GPIO_NR(1, 29), 1);
			gpio_direction_input(IMX_GPIO_NR(1, 30));
                else
                        gpio_direction_output(IMX_GPIO_NR(1, 29), 0);
                break;
		break;
	case 1:
		if (on)
			gpio_direction_output(IMX_GPIO_NR(1, 26), 1);
                        gpio_direction_input(IMX_GPIO_NR(1, 27));
		else
			gpio_direction_output(IMX_GPIO_NR(1, 26), 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif
#endif

int board_early_init_f(void)
{
        setup_iomux_wdt();
        setup_iomux_reset_out();
        setup_iomux_uart1();
        setup_iomux_uart2();
        setup_iomux_uart4();
        setup_iomux_uart5();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
        setup_spi1();
        setup_flexcan1();
        setup_flexcan2();
        setup_gpios();
	setup_misc();

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
        // Make sure we enable ECSPI2 clock
        int reg;
        struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
        reg = readl(&mxc_ccm->CCGR1);
        reg |=  MXC_CCM_CCGR1_ECSPI2S_MASK;
        writel(reg, &mxc_ccm->CCGR1);

        gpio_request(IMX_GPIO_NR(4, 20), "SPI_LOCK_PIN");
        /*Unlock SPI Flash*/
        gpio_direction_output(IMX_GPIO_NR(4,20), 1);
        setup_spinor();
#endif

#ifdef CONFIG_SYS_I2C
        setup_i2c(2, CONFIG_SYS_I2C_SPEED,
                        0x70, &i2c_pad_info3);

        /* Configure I2C switch (PCA9546) to enable channel 0. */
        i2c_set_bus_num(2);
        uint8_t i2cbuf;
        i2cbuf = 0x07;  /* Enable channel 0, 1, 2. */
        if (i2c_write(0x70, 0,
                      0, &i2cbuf, 1)) {
                printf("Write to MUX @ 0x%02x failed\n", 0x70);
                return 1;
        }
#endif

#ifdef CONFIG_PCIE_IMX
	setup_pcie();
#endif

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
	setup_epdc();
#endif

#ifdef CONFIG_SATA
	setup_sata();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
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
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	env_set("tee", "no");
#ifdef CONFIG_IMX_OPTEE
	env_set("tee", "yes");
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	if (is_mx6dqp())
		env_set("board_rev", "MX6QP");
	else if (is_mx6dq())
		env_set("board_rev", "MX6Q");
	else if (is_mx6sdl())
		env_set("board_rev", "MX6DL");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

        puts("---------Embedian SMARC-FiMX6------------\n");
        /* Read Module Information from on module EEPROM and pass
         * mac address to kernel
        */
        struct udevice *dev;
        int ret;
        u8 name[8];
        u8 serial[12];
        u8 revision[4];
        u8 mac[6];

        ret = i2c_get_chip_for_busnum(0, 0x50, 2, &dev);
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
        puts("-----------------------------------------\n");

	/* Lock Up SPI NOR First to Free ECSPI2 Bus */
        gpio_direction_output(IMX_GPIO_NR(4,20), 0);
	/* SMARC BOOT_SEL*/
        gpio_request(IMX_GPIO_NR(1, 4), "BOOT_SEL_1");
        gpio_request(IMX_GPIO_NR(1, 5), "BOOT_SEL_2");
        gpio_request(IMX_GPIO_NR(1, 6), "BOOT_SEL_3");
        if ((gpio_get_value(IMX_GPIO_NR(1, 4)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)) {
                puts("BOOT_SEL Detected: OFF OFF OFF, Load zImage from Carrier SATA...\n");
                env_set("root", "/dev/sda1 rootwait rw ");
                env_set("bootcmd", "sata init; run loadsataenv; run importbootenv; run uenvcmd; run loadsatazimage; run loadsatafdt; run sataboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 4)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)) {
                puts("BOOT_SEL Detected: OFF OFF ON, USB Boot Up Not Defined...Carrier SPI Boot Not Supported...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 4)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)) {
                puts("BOOT_SEL Detected: OFF ON OFF, Load zImage from Carrier SDMMC...\n");
                env_set_ulong("mmcdev", 1);
                env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 4)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)) {
                puts("BOOT_SEL Detected: ON OFF OFF, Load zImage from Carrier SD Card...\n");
                env_set_ulong("mmcdev", 0);
                env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 4)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)) {
                puts("BOOT_SEL Detected: OFF ON ON, Load zImage from Module eMMC Flash...\n");
                env_set_ulong("mmcdev", 2);
                env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 4)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 0)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)) {
                puts("BOOT_SEL Detected: ON OFF ON, Load zImage from GBE...\n");
                env_set("bootcmd", "run netboot;");
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 4)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 0)) {
                puts("Carrier SPI Boot is not supported...\n");
                hang();
        } else if ((gpio_get_value(IMX_GPIO_NR(1, 4)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)&&(gpio_get_value(IMX_GPIO_NR(1, 6)) == 1)) {
                puts("BOOT_SEL Detected: ON ON ON, MOdule SPI Boot up is Default, Load zImage from Module eMMC...\n");
                env_set_ulong("mmcdev", 2);
                env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadzimage; run loadfdt; run mmcboot;");
        } else {
                puts("unsupported boot devices\n");
                hang();
        }

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY

#define GPIO_VOL_DN_KEY IMX_GPIO_NR(1, 5)
iomux_v3_cfg_t const recovery_key_pads[] = {
	IOMUX_PADS(PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int is_recovery_key_pressing(void)
{
	int button_pressed = 0;

	/* Check Recovery Combo Button press or not. */
	SETUP_IOMUX_PADS(recovery_key_pads);

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

#ifdef CONFIG_SPL_BUILD
#include <asm/arch/mx6-ddr.h>
#include <spl.h>
#include <linux/libfdt.h>

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	gpio_request(KEY_VOL_UP, "KEY Volume UP");
	gpio_direction_input(KEY_VOL_UP);

	/* Only enter in Falcon mode if KEY_VOL_UP is pressed */
	return gpio_get_value(KEY_VOL_UP);
}
#endif

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static int mx6q_dcd_table[] = {
	0x020e0798, 0x000C0000,
	0x020e0758, 0x00000000,
	0x020e0588, 0x00000030,
	0x020e0594, 0x00000030,
	0x020e056c, 0x00000030,
	0x020e0578, 0x00000030,
	0x020e074c, 0x00000030,
	0x020e057c, 0x00000030,
	0x020e058c, 0x00000000,
	0x020e059c, 0x00000030,
	0x020e05a0, 0x00000030,
	0x020e078c, 0x00000030,
	0x020e0750, 0x00020000,
	0x020e05a8, 0x00000030,
	0x020e05b0, 0x00000030,
	0x020e0524, 0x00000030,
	0x020e051c, 0x00000030,
	0x020e0518, 0x00000030,
	0x020e050c, 0x00000030,
	0x020e05b8, 0x00000030,
	0x020e05c0, 0x00000030,
	0x020e0774, 0x00020000,
	0x020e0784, 0x00000030,
	0x020e0788, 0x00000030,
	0x020e0794, 0x00000030,
	0x020e079c, 0x00000030,
	0x020e07a0, 0x00000030,
	0x020e07a4, 0x00000030,
	0x020e07a8, 0x00000030,
	0x020e0748, 0x00000030,
	0x020e05ac, 0x00000030,
	0x020e05b4, 0x00000030,
	0x020e0528, 0x00000030,
	0x020e0520, 0x00000030,
	0x020e0514, 0x00000030,
	0x020e0510, 0x00000030,
	0x020e05bc, 0x00000030,
	0x020e05c4, 0x00000030,
	0x021b0800, 0xa1390003,
	0x021b080c, 0x001F001F,
	0x021b0810, 0x001F001F,
	0x021b480c, 0x001F001F,
	0x021b4810, 0x001F001F,
	0x021b083c, 0x43270338,
	0x021b0840, 0x03200314,
	0x021b483c, 0x431A032F,
	0x021b4840, 0x03200263,
	0x021b0848, 0x4B434748,
	0x021b4848, 0x4445404C,
	0x021b0850, 0x38444542,
	0x021b4850, 0x4935493A,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x00020036,
	0x021b0008, 0x09444040,
	0x021b000c, 0x555A7975,
	0x021b0010, 0xFF538F64,
	0x021b0014, 0x01FF00DB,
	0x021b0018, 0x00001740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x005A1023,
	0x021b0040, 0x00000027,
	0x021b0000, 0x831A0000,
	0x021b001c, 0x04088032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x09408030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00005800,
	0x021b0818, 0x00011117,
	0x021b4818, 0x00011117,
	0x021b0004, 0x00025576,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,
};

static int mx6qp_dcd_table[] = {
	0x020e0798, 0x000c0000,
	0x020e0758, 0x00000000,
	0x020e0588, 0x00000030,
	0x020e0594, 0x00000030,
	0x020e056c, 0x00000030,
	0x020e0578, 0x00000030,
	0x020e074c, 0x00000030,
	0x020e057c, 0x00000030,
	0x020e058c, 0x00000000,
	0x020e059c, 0x00000030,
	0x020e05a0, 0x00000030,
	0x020e078c, 0x00000030,
	0x020e0750, 0x00020000,
	0x020e05a8, 0x00000030,
	0x020e05b0, 0x00000030,
	0x020e0524, 0x00000030,
	0x020e051c, 0x00000030,
	0x020e0518, 0x00000030,
	0x020e050c, 0x00000030,
	0x020e05b8, 0x00000030,
	0x020e05c0, 0x00000030,
	0x020e0774, 0x00020000,
	0x020e0784, 0x00000030,
	0x020e0788, 0x00000030,
	0x020e0794, 0x00000030,
	0x020e079c, 0x00000030,
	0x020e07a0, 0x00000030,
	0x020e07a4, 0x00000030,
	0x020e07a8, 0x00000030,
	0x020e0748, 0x00000030,
	0x020e05ac, 0x00000030,
	0x020e05b4, 0x00000030,
	0x020e0528, 0x00000030,
	0x020e0520, 0x00000030,
	0x020e0514, 0x00000030,
	0x020e0510, 0x00000030,
	0x020e05bc, 0x00000030,
	0x020e05c4, 0x00000030,
	0x021b0800, 0xa1390003,
	0x021b080c, 0x001b001e,
	0x021b0810, 0x002e0029,
	0x021b480c, 0x001b002a,
	0x021b4810, 0x0019002c,
	0x021b083c, 0x43240334,
	0x021b0840, 0x0324031a,
	0x021b483c, 0x43340344,
	0x021b4840, 0x03280276,
	0x021b0848, 0x44383A3E,
	0x021b4848, 0x3C3C3846,
	0x021b0850, 0x2e303230,
	0x021b4850, 0x38283E34,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08c0, 0x24912249,
	0x021b48c0, 0x24914289,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x00020036,
	0x021b0008, 0x24444040,
	0x021b000c, 0x555A7955,
	0x021b0010, 0xFF320F64,
	0x021b0014, 0x01ff00db,
	0x021b0018, 0x00001740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x005A1023,
	0x021b0040, 0x00000027,
	0x021b0400, 0x14420000,
	0x021b0000, 0x831A0000,
	0x021b0890, 0x00400C58,
	0x00bb0008, 0x00000000,
	0x00bb000c, 0x2891E41A,
	0x00bb0038, 0x00000564,
	0x00bb0014, 0x00000040,
	0x00bb0028, 0x00000020,
	0x00bb002c, 0x00000020,
	0x021b001c, 0x04088032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x09408030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00005800,
	0x021b0818, 0x00011117,
	0x021b4818, 0x00011117,
	0x021b0004, 0x00025576,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,
};

static int mx6dl_dcd_table[] = {
	0x020e0774, 0x000C0000,
	0x020e0754, 0x00000000,
	0x020e04ac, 0x00000030,
	0x020e04b0, 0x00000030,
	0x020e0464, 0x00000030,
	0x020e0490, 0x00000030,
	0x020e074c, 0x00000030,
	0x020e0494, 0x00000030,
	0x020e04a0, 0x00000000,
	0x020e04b4, 0x00000030,
	0x020e04b8, 0x00000030,
	0x020e076c, 0x00000030,
	0x020e0750, 0x00020000,
	0x020e04bc, 0x00000030,
	0x020e04c0, 0x00000030,
	0x020e04c4, 0x00000030,
	0x020e04c8, 0x00000030,
	0x020e04cc, 0x00000030,
	0x020e04d0, 0x00000030,
	0x020e04d4, 0x00000030,
	0x020e04d8, 0x00000030,
	0x020e0760, 0x00020000,
	0x020e0764, 0x00000030,
	0x020e0770, 0x00000030,
	0x020e0778, 0x00000030,
	0x020e077c, 0x00000030,
	0x020e0780, 0x00000030,
	0x020e0784, 0x00000030,
	0x020e078c, 0x00000030,
	0x020e0748, 0x00000030,
	0x020e0470, 0x00000030,
	0x020e0474, 0x00000030,
	0x020e0478, 0x00000030,
	0x020e047c, 0x00000030,
	0x020e0480, 0x00000030,
	0x020e0484, 0x00000030,
	0x020e0488, 0x00000030,
	0x020e048c, 0x00000030,
	0x021b0800, 0xa1390003,
	0x021b080c, 0x001F001F,
	0x021b0810, 0x001F001F,
	0x021b480c, 0x001F001F,
	0x021b4810, 0x001F001F,
	0x021b083c, 0x4220021F,
	0x021b0840, 0x0207017E,
	0x021b483c, 0x4201020C,
	0x021b4840, 0x01660172,
	0x021b0848, 0x4A4D4E4D,
	0x021b4848, 0x4A4F5049,
	0x021b0850, 0x3F3C3D31,
	0x021b4850, 0x3238372B,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x0002002D,
	0x021b0008, 0x00333030,
	0x021b000c, 0x3F435313,
	0x021b0010, 0xB66E8B63,
	0x021b0014, 0x01FF00DB,
	0x021b0018, 0x00001740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x00431023,
	0x021b0040, 0x00000027,
	0x021b0000, 0x831A0000,
	0x021b001c, 0x04008032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x05208030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00005800,
	0x021b0818, 0x00011117,
	0x021b4818, 0x00011117,
	0x021b0004, 0x0002556D,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,
};

static void ddr_init(int *table, int size)
{
	int i;

	for (i = 0; i < size / 2 ; i++)
		writel(table[2 * i + 1], table[2 * i]);
}

static void spl_dram_init(void)
{
	if (is_mx6dq())
		ddr_init(mx6q_dcd_table, ARRAY_SIZE(mx6q_dcd_table));
	else if (is_mx6dqp())
		ddr_init(mx6qp_dcd_table, ARRAY_SIZE(mx6qp_dcd_table));
	else if (is_mx6sdl())
		ddr_init(mx6dl_dcd_table, ARRAY_SIZE(mx6dl_dcd_table));
}

void board_init_f(ulong dummy)
{
	/* DDR initialization */
	spl_dram_init();

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
