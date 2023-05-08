// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 */

#include <common.h>
#include <cpu_func.h>
#include <env.h>
#include <hang.h>
#include <errno.h>
#include <i2c.h>
#include <init.h>
#include <asm/global_data.h>
#include <linux/libfdt.h>
#include <fdt_support.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/sci/sci.h>
#include <asm/arch/imx8-pins.h>
#include <asm/arch/snvs_security_sc.h>
#include <usb.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include "../../freescale/common/tcpc.h"
#include "command.h"

DECLARE_GLOBAL_DATA_PTR;

#define ENET_INPUT_PAD_CTRL	((SC_PAD_CONFIG_OD_IN << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_18V_10MA << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define ENET_NORMAL_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_18V_10MA << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))


#define GPIO_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))


#define UART_PAD_CTRL	((SC_PAD_CONFIG_OUT_IN << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

/* SER0 */
#ifdef CONFIG_CONSOLE_SER0
static iomux_cfg_t uart0_pads[] = {
	SC_P_UART0_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	SC_P_UART0_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

/* SER1 */
#ifdef CONFIG_CONSOLE_SER1
static iomux_cfg_t uart3_pads[] = {
	SC_P_M41_GPIO0_00 | MUX_MODE_ALT(2) | MUX_PAD_CTRL(UART_PAD_CTRL),	/* M41.GPIO0.IO00, M41.TPM0.CH0, DMA.UART3.RX, LSIO.GPIO0.IO12 */
	SC_P_M41_GPIO0_01 | MUX_MODE_ALT(2) | MUX_PAD_CTRL(UART_PAD_CTRL),	/* M41.GPIO0.IO01, M41.TPM0.CH1, DMA.UART3.TX, LSIO.GPIO0.IO13 */
};
#endif

/* SER2 */
#ifdef CONFIG_CONSOLE_SER2
static iomux_cfg_t uart1_pads[] = {
	SC_P_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	SC_P_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

/* SER3 */
#ifdef CONFIG_CONSOLE_SER3
static iomux_cfg_t uart4_pads[] = {
	SC_P_M40_GPIO0_00 | MUX_MODE_ALT(2) | MUX_PAD_CTRL(UART_PAD_CTRL),	/* M40.GPIO0.IO00, M40.TPM0.CH0, DMA.UART4.RX, LSIO.GPIO0.IO08 */
	SC_P_M40_GPIO0_01 | MUX_MODE_ALT(2) | MUX_PAD_CTRL(UART_PAD_CTRL),	/* M40.GPIO0.IO01, M40.TPM0.CH1, DMA.UART4.TX, LSIO.GPIO0.IO09 */
};
#endif

#ifdef CONFIG_CONSOLE_SER0
static void setup_iomux_uart0(void)
{
	imx8_iomux_setup_multiple_pads(uart0_pads, ARRAY_SIZE(uart0_pads));
}
#endif

#ifdef CONFIG_CONSOLE_SER1
static void setup_iomux_uart3(void)
{
	imx8_iomux_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
}
#endif

#ifdef CONFIG_CONSOLE_SER2
static void setup_iomux_uart1(void)
{
	imx8_iomux_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}
#endif

#ifdef CONFIG_CONSOLE_SER3
static void setup_iomux_uart4(void)
{
	imx8_iomux_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}
#endif

int board_early_init_f(void)
{
	sc_pm_clock_rate_t rate = SC_80MHZ;
	int ret;

#ifdef CONFIG_CONSOLE_SER0
	/* Set UART0 clock root to 80 MHz */
	ret = sc_pm_setup_uart(SC_R_UART_0, rate);
	if (ret)
		return ret;

	setup_iomux_uart0();
#endif

#ifdef CONFIG_CONSOLE_SER1
	/* Set UART3 clock root to 80 MHz */
	ret = sc_pm_setup_uart(SC_R_UART_3, rate);
	if (ret)
		return ret;

	setup_iomux_uart3();
#endif

#ifdef CONFIG_CONSOLE_SER2
	/* Set UART1 clock root to 80 MHz */
	ret = sc_pm_setup_uart(SC_R_UART_1, rate);
	if (ret)
		return ret;

	setup_iomux_uart1();
#endif

#ifdef CONFIG_CONSOLE_SER3
	/* Set UART4 clock root to 80 MHz */
	ret = sc_pm_setup_uart(SC_R_UART_4, rate);
	if (ret)
		return ret;

	setup_iomux_uart4();
#endif

	return 0;
}


#if IS_ENABLED(CONFIG_FEC_MXC)
#include <miiphy.h>

#ifndef CONFIG_DM_ETH
static iomux_cfg_t pad_enet1[] = {
	SC_P_ENET1_RGMII_RX_CTL | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_RXD0 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_RXD1 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_RXD2 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_RXD3 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_RXC | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_TX_CTL | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_RGMII_TXD0 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_RGMII_TXD1 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_RGMII_TXD2 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_RGMII_TXD3 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_RGMII_TXC | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),

	/* Shared MDIO */
	SC_P_ENET0_MDC | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_MDIO | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
};

static iomux_cfg_t pad_enet0[] = {
	SC_P_ENET0_RGMII_RX_CTL | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_RXD0 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_RXD1 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_RXD2 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_RXD3 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_RXC | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_TX_CTL | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_RGMII_TXD0 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_RGMII_TXD1 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_RGMII_TXD2 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_RGMII_TXD3 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_RGMII_TXC | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),

	/* Shared MDIO */
	SC_P_ENET0_MDC | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_MDIO | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
};

static void setup_iomux_fec(void)
{
	if (0 == CONFIG_FEC_ENET_DEV)
		imx8_iomux_setup_multiple_pads(pad_enet0, ARRAY_SIZE(pad_enet0));
	else
		imx8_iomux_setup_multiple_pads(pad_enet1, ARRAY_SIZE(pad_enet1));
}

int board_eth_init(bd_t *bis)
{
	int ret;
	struct power_domain pd;

	printf("[%s] %d\n", __func__, __LINE__);

	if (CONFIG_FEC_ENET_DEV) {
		if (!power_domain_lookup_name("conn_enet1", &pd))
			power_domain_on(&pd);
	} else {
		if (!power_domain_lookup_name("conn_enet0", &pd))
			power_domain_on(&pd);
	}

	setup_iomux_fec();

	ret = fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
		CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
	if (ret)
		printf("FEC1 MXC: %s:failed\n", __func__);

	return ret;
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	return 0;
}
#endif
#endif

static iomux_cfg_t smarc_gpio[] = {
	SC_P_MIPI_CSI0_GPIO0_01 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),	/* GPIO0 */
	SC_P_MIPI_CSI1_GPIO0_01 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),	/* GPIO1 */
	SC_P_MIPI_CSI0_GPIO0_00 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),	/* GPIO2 */
	SC_P_MIPI_CSI1_GPIO0_00 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),	/* GPIO3 */
	SC_P_GPT0_CLK | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),			/* GPIO4 */
	SC_P_GPT0_COMPARE | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* GPIO5 */
	SC_P_GPT0_CAPTURE | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* GPIO6 */
	SC_P_GPT1_COMPARE | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* GPIO7 */
	SC_P_GPT1_CAPTURE | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* GPIO8 */
	SC_P_GPT1_CLK | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),			/* GPIO9 */
	SC_P_FLEXCAN2_TX | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* GPIO10 */
	SC_P_FLEXCAN2_RX | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* GPIO11 */
	SC_P_SCU_GPIO0_03 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* LID# */
	SC_P_SCU_GPIO0_04 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* SLEEP# */
	SC_P_SCU_GPIO0_05 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* CHARGING# */
	SC_P_SCU_GPIO0_06 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* CHARGER_PRSNT# */
	SC_P_SCU_GPIO0_07 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* BATLOW# */
	SC_P_SCU_GPIO0_02 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* CARRIER_STBY# */
	SC_P_QSPI1A_DATA0 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* RGMII0_INT# */
	SC_P_QSPI1A_DATA1 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* RGMII1_INT# */
	SC_P_USB_SS3_TC0 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* USB_OTG1_PWR_EN */
	SC_P_USB_SS3_TC1 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* USB_OTG2_PWR_EN */
	SC_P_USB_SS3_TC2 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* USB_OTG1_OC */
	SC_P_USB_SS3_TC3 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* USB_OTG2_OC */
	SC_P_SIM0_GPIO0_00 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* HDMI_SEL */
	SC_P_MIPI_DSI1_GPIO0_00 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),	/* EDP0_EN */
	SC_P_MIPI_DSI0_GPIO0_00 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),	/* EDP1_EN */
	SC_P_MIPI_DSI1_GPIO0_01 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),	/* EDP0_IRQ */
	SC_P_MIPI_DSI0_GPIO0_01 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),	/* EDP1_IRQ */
	SC_P_LVDS0_GPIO01 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* LCD0_VDD_EN */
	SC_P_LVDS1_GPIO01 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* LCD1_VDD_EN */
	SC_P_LVDS0_I2C0_SCL | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* LCD0_BKLT_EN */
	SC_P_LVDS0_I2C0_SDA | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),		/* LCD1_BKLT_EN */
	SC_P_ADC_IN0 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),			/* BOOT_SEL0 */
	SC_P_ADC_IN1 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),			/* BOOT_SEL1 */
	SC_P_ADC_IN2 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),			/* BOOT_SEL2 */
	SC_P_SPDIF0_TX | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),			/* WDT_TIME_OUT# */
};

/* GPIO0 */
#define GPIO0 IMX_GPIO_NR(1, 28)		/* MIPI_CSI0.GPIO0.IO01, DMA.I2C0.SDA, MIPI_CSI1.I2C0.SDA, LSIO.GPIO1.IO28 */
/* GPIO1 */
#define GPIO1 IMX_GPIO_NR(1, 31)		/* MIPI_CSI1.GPIO0.IO01, DMA.UART4.TX, LSIO.GPIO1.IO31 */
/* GPIO2 */
#define GPIO2 IMX_GPIO_NR(1, 27)		/* MIPI_CSI0.GPIO0.IO00, DMA.I2C0.SCL, MIPI_CSI1.I2C0.SCL, LSIO.GPIO1.IO27 */
/* GPIO3 */
#define GPIO3 IMX_GPIO_NR(1, 30)		/* MIPI_CSI1.GPIO0.IO00, DMA.UART4.RX, LSIO.GPIO1.IO30 */
/* GPIO4 */
#define GPIO4 IMX_GPIO_NR(0, 14)		/* LSIO.GPT0.CLK, DMA.I2C1.SCL, LSIO.KPP0.COL4, LSIO.GPIO0.IO14 */
/* GPIO5 */
#define GPIO5 IMX_GPIO_NR(0, 16)		/* LSIO.GPT0.COMPARE, LSIO.PWM3.OUT, LSIO.KPP0.COL6, LSIO.GPIO0.IO16 */
/* GPIO6 */
#define GPIO6 IMX_GPIO_NR(0, 15)		/* LSIO.GPT0.CAPTURE, DMA.I2C1.SDA, LSIO.KPP0.COL5, LSIO.GPIO0.IO15 */
/* GPIO7 */
#define GPIO7 IMX_GPIO_NR(0, 19)		/* LSIO.GPT1.COMPARE, LSIO.PWM2.OUT, LSIO.KPP0.ROW5, LSIO.GPIO0.IO19 */
/* GPIO8 */
#define GPIO8 IMX_GPIO_NR(0, 18)		/* LSIO.GPT1.CAPTURE, DMA.I2C2.SDA, LSIO.KPP0.ROW4, LSIO.GPIO0.IO18 */
/* GPIO9 */
#define GPIO9 IMX_GPIO_NR(0, 17)		/* LSIO.GPT1.CLK, DMA.I2C2.SCL, LSIO.KPP0.COL7, LSIO.GPIO0.IO17 */
/* GPIO10 */
#define GPIO10 IMX_GPIO_NR(4, 02)		/* DMA.FLEXCAN2.TX, LSIO.GPIO4.IO02 */
/* GPIO11 */
#define GPIO11 IMX_GPIO_NR(4, 01)		/* DMA.FLEXCAN2.RX, LSIO.GPIO4.IO01 */
/* LID# */
#define LID IMX_GPIO_NR(0, 31)			/* SCU.GPIO0.IO03, SCU.GPIO0.IOXX_PMIC_GPU1_ON, LSIO.GPIO0.IO31 */
/* SLEEP# */
#define SLEEP IMX_GPIO_NR(1, 00)		/* SCU.GPIO0.IO04, SCU.GPIO0.IOXX_PMIC_A72_ON, LSIO.GPIO1.IO00 */
/* CHARGING# */
#define CHARGING IMX_GPIO_NR(1, 01)		/* SCU.GPIO0.IO05, SCU.GPIO0.IOXX_PMIC_A53_ON, LSIO.GPIO1.IO01 */
/* CHARGER_PRSNT# */
#define CHARGER_PRSNT IMX_GPIO_NR(1, 02)	/* SCU.GPIO0.IO06, SCU.TPM0.CH0, LSIO.GPIO1.IO02 */
/* BATLOW# */
#define BATLOW IMX_GPIO_NR(1, 03)		/* SCU.GPIO0.IO07, SCU.TPM0.CH1, SCU.DSC.RTC_CLOCK_OUTPUT_32K, LSIO.GPIO1.IO03 */
/* CARRIER_STBY# */
#define CARRIER_STBY IMX_GPIO_NR(0, 30)		/* SCU.GPIO0.IO02, SCU.GPIO0.IOXX_PMIC_GPU0_ON, LSIO.GPIO0.IO30 */
/* RGMII0_INT# */
#define RGMII0_INT IMX_GPIO_NR(4, 26)		/* LSIO.QSPI1A.DATA0, LSIO.GPIO4.IO26 */
/* RGMII1_INT# */
#define RGMII1_INT IMX_GPIO_NR(4, 25)		/* LSIO.QSPI1A.DATA1, DMA.I2C1.SDA, CONN.USB_OTG2.OC, LSIO.GPIO4.IO25 */
/* USB_OTG1_PWR_EN */
#define USB_OTG1_PWR_EN IMX_GPIO_NR(4, 03)	/* DMA.I2C1.SCL, CONN.USB_OTG1.PWR, LSIO.GPIO4.IO03 */
/* USB_OTG2_PWR_EN */
#define USB_OTG2_PWR_EN IMX_GPIO_NR(4, 04)	/* DMA.I2C1.SCL, CONN.USB_OTG2.PWR, LSIO.GPIO4.IO04 */
/* USB_OTG1_OC# */
#define USB_OTG1_OC IMX_GPIO_NR(4, 05)		/* DMA.I2C1.SDA, CONN.USB_OTG1.OC, LSIO.GPIO4.IO05 */
/* USB_OTG2_OC# */
#define USB_OTG2_OC IMX_GPIO_NR(4, 06)		/* DMA.I2C1.SDA, CONN.USB_OTG2.OC, LSIO.GPIO4.IO06 */
/* HDMI_SEL */
#define HDMI_SEL IMX_GPIO_NR(0, 05)		/* DMA.SIM0.POWER_EN, LSIO.GPIO0.IO05 */
/* EDP0_EN */
#define EDP0_EN IMX_GPIO_NR(1, 22)		/* MIPI_DSI1.GPIO0.IO00, MIPI_DSI1.PWM0.OUT, LSIO.GPIO1.IO22 */
/* EDP1_EN */
#define EDP1_EN IMX_GPIO_NR(1, 18)		/* MIPI_DSI0.GPIO0.IO00, MIPI_DSI0.PWM0.OUT, LSIO.GPIO1.IO18 */
/* EDP0_IRQ */
#define EDP0_IRQ IMX_GPIO_NR(1, 23)		/* MIPI_DSI1.GPIO0.IO01, LSIO.GPIO1.IO23 */
/* EDP1_IRQ */
#define EDP1_IRQ IMX_GPIO_NR(1, 19)		/* MIPI_DSI0.GPIO0.IO01, LSIO.GPIO1.IO19 */
/* LCD0_VDD_EN */
#define LCD0_VDD_EN IMX_GPIO_NR(1, 05)		/* LVDS0.GPIO0.IO01, LSIO.GPIO1.IO05 */
/* LCD1_VDD_EN */
#define LCD1_VDD_EN IMX_GPIO_NR(1, 11)		/* LVDS1.GPIO0.IO01, LSIO.GPIO1.IO11 */
/* LCD0_BKLT_EN */
#define LCD0_BKLT_EN IMX_GPIO_NR(1, 06)		/* LVDS0.I2C0.SCL, LVDS0.GPIO0.IO02, LSIO.GPIO1.IO06 */
/* LCD1_BKLT_EN */
#define LCD1_BKLT_EN IMX_GPIO_NR(1, 07)		/* LVDS0.I2C0.SDA, LVDS0.GPIO0.IO03, LSIO.GPIO1.IO07 */
/* WDT_TIME_OUT# */
#define WDT_TIME_OUT IMX_GPIO_NR(2, 15)		/* AUD.SPDIF0.TX, AUD.MQS.L, AUD.ACM.MCLK_OUT1, LSIO.GPIO2.IO15 */

static void board_gpio_init(void)
{
	int ret;
	struct gpio_desc desc;

	imx8_iomux_setup_multiple_pads(smarc_gpio, ARRAY_SIZE(smarc_gpio));
	/* By SMARC definition, GPIO0~GPIO5 are set as Outpin Low by default */
	/* GPIO0 */
	ret = dm_gpio_lookup_name("GPIO1_28", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_28 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio0");
	if (ret) {
	printf("%s request gpio0 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* GPIO1 */
	ret = dm_gpio_lookup_name("GPIO1_31", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_31 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio1");
	if (ret) {
		printf("%s request gpio1 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* GPIO2 */
	ret = dm_gpio_lookup_name("GPIO1_27", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_27 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio2");
	if (ret) {
		printf("%s request gpio2 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* GPIO3 */
	ret = dm_gpio_lookup_name("GPIO1_30", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_30 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio3");
	if (ret) {
		printf("%s request gpio3 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* GPIO4 */
	ret = dm_gpio_lookup_name("GPIO0_14", &desc);
	if (ret) {
		printf("%s lookup GPIO@0_14 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio4");
	if (ret) {
		printf("%s request gpio4 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* GPIO5 */
	ret = dm_gpio_lookup_name("GPIO0_16", &desc);
	if (ret) {
		printf("%s lookup GPIO@0_16 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio5");
	if (ret) {
		printf("%s request gpio5 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* GPIO6 */
	ret = dm_gpio_lookup_name("GPIO0_15", &desc);
	if (ret) {
		printf("%s lookup GPIO@0_15 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio6");
	if (ret) {
		printf("%s request gpio6 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* GPIO7 */
	ret = dm_gpio_lookup_name("GPIO0_19", &desc);
	if (ret) {
		printf("%s lookup GPIO@0_19 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio7");
	if (ret) {
		printf("%s request gpio7 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* GPIO8 */
	ret = dm_gpio_lookup_name("GPIO0_18", &desc);
	if (ret) {
		printf("%s lookup GPIO@0_18 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio8");
	if (ret) {
		printf("%s request gpio8 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* GPIO9 */
	ret = dm_gpio_lookup_name("GPIO0_17", &desc);
	if (ret) {
		printf("%s lookup GPIO@0_17 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio9");
	if (ret) {
		printf("%s request gpio9 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* GPIO10 */
	ret = dm_gpio_lookup_name("GPIO4_02", &desc);
	if (ret) {
		printf("%s lookup GPIO@4_02 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio10");
	if (ret) {
		printf("%s request gpio10 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* GPIO11 */
	ret = dm_gpio_lookup_name("GPIO4_01", &desc);
	if (ret) {
		printf("%s lookup GPIO@4_01 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "gpio11");
	if (ret) {
		printf("%s request gpio11 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set LID# pin as Input Pin */
	ret = dm_gpio_lookup_name("GPIO0_31", &desc);
	if (ret) {
		printf("%s lookup GPIO@0_31 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "lid");
	if (ret) {
		printf("%s request lid failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set SLEEP# pin as Input Pin */
	ret = dm_gpio_lookup_name("GPIO1_00", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_00 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "sleep");
	if (ret) {
		printf("%s request sleep failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set CHARGING# pin as Input Pin */
	ret = dm_gpio_lookup_name("GPIO1_01", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_01 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "charging");
	if (ret) {
		printf("%s request charging failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set CHARGER_PRSNT# pin as Input Pin */
	ret = dm_gpio_lookup_name("GPIO1_02", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_02 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "charger_prsnt");
	if (ret) {
		printf("%s request charger_prsnt failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set BATLOW# pin as Input Pin */
	ret = dm_gpio_lookup_name("GPIO1_03", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_03 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "batlow");
	if (ret) {
		printf("%s request batlow failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set CARRIER_STBY# pin as Output High */
	ret = dm_gpio_lookup_name("GPIO0_30", &desc);
	if (ret) {
		printf("%s lookup GPIO@0_30 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "carrier_stby");
	if (ret) {
		printf("%s request carrier_stby failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

	/* Set RGMII0_INT# pin as Input pin*/
	ret = dm_gpio_lookup_name("GPIO4_26", &desc);
	if (ret) {
		printf("%s lookup GPIO@4_26 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "rgmii0_int");
	if (ret) {
		printf("%s request rgmii0_int failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set RGMII1_INT# pin as Input pin*/
	ret = dm_gpio_lookup_name("GPIO4_25", &desc);
	if (ret) {
		printf("%s lookup GPIO@4_25 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "rgmii1_int");
	if (ret) {
		printf("%s request rgmii1_int failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Turn on USB0 Power */
	ret = dm_gpio_lookup_name("GPIO4_03", &desc);
	if (ret) {
		printf("%s lookup GPIO@4_03 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "usb_otg1_pwr_en");
	if (ret) {
		printf("%s request usb_otg1_pwr_en failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

	/* Turn on USB2514 Hub Power */
	ret = dm_gpio_lookup_name("GPIO4_04", &desc);
	if (ret) {
		printf("%s lookup GPIO@4_04 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "usb_otg2_pwr_en");
	if (ret) {
		printf("%s request usb_otg2_pwr_en failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

	/* Set USB0 OC# pin as Input pin*/
	ret = dm_gpio_lookup_name("GPIO4_05", &desc);
	if (ret) {
		printf("%s lookup GPIO@4_05 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "usb_otg1_oc");
	if (ret) {
		printf("%s request usb_otg1_oc failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set USB1 OC# pin as Input pin*/
	ret = dm_gpio_lookup_name("GPIO4_06", &desc);
	if (ret) {
		printf("%s lookup GPIO@4_06 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "usb_otg2_oc");
	if (ret) {
		printf("%s request usb_otg2_oc failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set HDMI_SEL pin as Input pin*/
	ret = dm_gpio_lookup_name("GPIO0_05", &desc);
	if (ret) {
		printf("%s lookup GPIO@0_05 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "hdmi_sel");
	if (ret) {
		printf("%s request usb_hdmi_sel failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Turn on eDP0 Power */
	ret = dm_gpio_lookup_name("GPIO1_22", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_22 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "edp0_en");
	if (ret) {
		printf("%s request edp0_en failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

	/* Turn off eDP1 Power */
	ret = dm_gpio_lookup_name("GPIO1_18", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_18 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "edp1_en");
	if (ret) {
		printf("%s request edp1_en failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);

	/* Set eDP0_IRQ pin as Input Pin */
	ret = dm_gpio_lookup_name("GPIO1_23", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_23 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "edp0_irq");
	if (ret) {
		printf("%s request edp0_irq failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Set eDP1_IRQ pin as Input Pin */
	ret = dm_gpio_lookup_name("GPIO1_19", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_19 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "edp1_irq");
	if (ret) {
		printf("%s request edp1_irq failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	/* Turn on LVDS0_VDD Power */
	ret = dm_gpio_lookup_name("GPIO1_05", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_05 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "lcd0_vdd_en");
	if (ret) {
		printf("%s request lcd0_vdd_en failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

	/* Turn on LVDS1_VDD Power */
	ret = dm_gpio_lookup_name("GPIO1_11", &desc);
	if (ret) {
		printf("%s lookup GPIO@1_11 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "lcd1_vdd_en");
	if (ret) {
		printf("%s request lcd1_vdd_en failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

	/* enable LVDS0 Backlight Power */
	ret = dm_gpio_lookup_name("GPIO1_06", &desc);
	if (ret) {
		printf("%s lookup GPIO1_06 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "lcd0_bklt_en");
	if (ret) {
		printf("%s request lcd0_bklt_en failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

	/* enable LVDS1 Backlight Power */
	ret = dm_gpio_lookup_name("GPIO1_07", &desc);
	if (ret) {
		printf("%s lookup GPIO1_07 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "lcd1_bklt_en");
	if (ret) {
		printf("%s request lcd1_bklt_en failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);

	/* Set WDT_TIME_OUT# pin as Output High */
	ret = dm_gpio_lookup_name("GPIO2_15", &desc);
	if (ret) {
		printf("%s lookup GPIO@2_15 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "wdt_time_out");
	if (ret) {
		printf("%s request wdt_time_out failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	dm_gpio_set_value(&desc, 1);
}

int checkboard(void)
{
	puts("Board: SMARC-iMX8QM CPU Module\n");

	print_bootinfo();

	return 0;
}

#ifdef CONFIG_USB

#ifdef CONFIG_USB_TCPC
struct gpio_desc type_sel_desc;

static iomux_cfg_t ss_mux_gpio[] = {
	SC_P_USB_SS3_TC3 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	SC_P_QSPI1A_SS0_B | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

struct tcpc_port port;
struct tcpc_port_config port_config = {
	.i2c_bus = 0,
	.addr = 0x51,
	.port_type = TYPEC_PORT_DFP,
};

void ss_mux_select(enum typec_cc_polarity pol)
{
	if (pol == TYPEC_POLARITY_CC1)
		dm_gpio_set_value(&type_sel_desc, 0);
	else
		dm_gpio_set_value(&type_sel_desc, 1);
}

static void setup_typec(void)
{
	int ret;
	struct gpio_desc typec_en_desc;

	imx8_iomux_setup_multiple_pads(ss_mux_gpio, ARRAY_SIZE(ss_mux_gpio));
	ret = dm_gpio_lookup_name("GPIO4_6", &type_sel_desc);
	if (ret) {
		printf("%s lookup GPIO4_6 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&type_sel_desc, "typec_sel");
	if (ret) {
		printf("%s request typec_sel failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&type_sel_desc, GPIOD_IS_OUT);

	ret = dm_gpio_lookup_name("GPIO4_19", &typec_en_desc);
	if (ret) {
		printf("%s lookup GPIO4_19 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&typec_en_desc, "typec_en");
	if (ret) {
		printf("%s request typec_en failed ret = %d\n", __func__, ret);
		return;
	}

	/* Enable SS MUX */
	dm_gpio_set_dir_flags(&typec_en_desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);

	ret = tcpc_init(&port, port_config, &ss_mux_select);
	if (ret) {
		printf("%s: tcpc init failed, err=%d\n", __func__, ret);
		return;
	}
}
#endif

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

	if (index == 1) {
		if (init == USB_INIT_HOST) {
#ifdef CONFIG_USB_TCPC
			ret = tcpc_setup_dfp_mode(&port);
#endif
#ifdef CONFIG_USB_CDNS3_GADGET
		} else {
#ifdef CONFIG_USB_TCPC
			ret = tcpc_setup_ufp_mode(&port);
			printf("%d setufp mode %d\n", index, ret);
#endif
#endif
		}
	}

	return ret;

}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;

	if (index == 1) {
		if (init == USB_INIT_HOST) {
#ifdef CONFIG_USB_TCPC
			ret = tcpc_disable_src_vbus(&port);
#endif
		}
	}

	return ret;
}
#endif

int board_init(void)
{
	board_gpio_init();


#if defined(CONFIG_USB) && defined(CONFIG_USB_TCPC)
	setup_typec();
#endif

#ifdef CONFIG_IMX_SNVS_SEC_SC_AUTO
	{
		int ret = snvs_security_sc_init();

		if (ret)
			return ret;
	}
#endif

	return 0;
}

/*
 * Board specific reset that is system reset.
 */
void reset_cpu(void)
{
	sc_pm_reboot(-1, SC_PM_RESET_TYPE_COLD);
	while(1);
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	return 0;
}
#endif

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

extern uint32_t _end_ofs;
int board_late_init(void)
{
	puts("---------Embedian SMARC-iMX8QM------------\n");
	/* Read Module Information from on module EEPROM and pass
	 * mac address to kernel
	*/
	struct gpio_desc desc;
	struct udevice *dev;
	char bootmode = 0;
	int ret;
	u8 name[8];
	u8 serial[12];
	u8 revision[4];
	u8 mac[6];
	u8 mac1[6];

	ret = i2c_get_chip_for_busnum(3, 0x50, 2, &dev);
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

	/*MAC1 address*/
	ret = dm_i2c_read(dev, 0x3c, mac, 6);
	if (ret) {
		debug("failed to read eth0 mac address from EEPROM\n");
		return 0;
	}

	if (is_valid_ethaddr(mac))
	printf("  MAC Address:          %02x:%02x:%02x:%02x:%02x:%02x\n",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
			eth_env_set_enetaddr("ethaddr", mac);

	/* MAC2 address */
	ret = dm_i2c_read(dev, 0x42, mac1, 6);
	if (ret) {
		debug("failed to read eth1 mac address from EEPROM\n");
		return 0;
	}

	if (is_valid_ethaddr(mac1))
	printf("  MAC1 Address:         %02x:%02x:%02x:%02x:%02x:%02x\n",
		mac1[0], mac1[1], mac1[2], mac1[3], mac1[4], mac1[5]);
			eth_env_set_enetaddr("eth1addr", mac1);
	puts("-----------------------------------------\n");

	build_info();

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "SMARC");
	env_set("board_rev", "iMX8QM");
#endif

	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

#if defined(CONFIG_IMX_LOAD_HDMI_FIMRWARE_RX) || defined(CONFIG_IMX_LOAD_HDMI_FIMRWARE_TX)
	char *end_of_uboot;
	char command[256];
	end_of_uboot = (char *)(ulong)(CONFIG_SYS_TEXT_BASE + _end_ofs + fdt_totalsize(gd->fdt_blob));
	end_of_uboot += 9;

	/* load hdmitxfw.bin and hdmirxfw.bin*/
	memcpy((void *)IMX_HDMI_FIRMWARE_LOAD_ADDR, end_of_uboot,
			IMX_HDMITX_FIRMWARE_SIZE + IMX_HDMIRX_FIRMWARE_SIZE);

#ifdef CONFIG_IMX_LOAD_HDMI_FIMRWARE_TX
	sprintf(command, "hdp load 0x%x", IMX_HDMI_FIRMWARE_LOAD_ADDR);
	run_command(command, 0);
#endif
#ifdef CONFIG_IMX_LOAD_HDMI_FIMRWARE_RX
	sprintf(command, "hdprx load 0x%x",
			IMX_HDMI_FIRMWARE_LOAD_ADDR + IMX_HDMITX_FIRMWARE_SIZE);
	run_command(command, 0);
#endif
#endif /* CONFIG_IMX_LOAD_HDMI_FIMRWARE_RX || CONFIG_IMX_LOAD_HDMI_FIMRWARE_TX */

	/* BOOT_SEL0 */
	ret = dm_gpio_lookup_name("GPIO3_18", &desc);
	if (ret) {
		printf("%s lookup GPIO@3_18 failed ret = %d\n", __func__, ret);
		return 0;
	}

	ret = dm_gpio_request(&desc, "bootsel0");
	if (ret) {
		printf("%s request bootsel0 failed ret = %d\n", __func__, ret);
		return 0;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);
		bootmode |= (dm_gpio_get_value(&desc) ? 1 : 0) << 0;

	/* BOOT_SEL1 */
	ret = dm_gpio_lookup_name("GPIO3_19", &desc);
	if (ret) {
		printf("%s lookup GPIO@3_19 failed ret = %d\n", __func__, ret);
		return 0;
	}

	ret = dm_gpio_request(&desc, "bootsel1");
	if (ret) {
		printf("%s request bootsel1 failed ret = %d\n", __func__, ret);
		return 0;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);
               bootmode |= (dm_gpio_get_value(&desc) ? 1 : 0) << 1;

	/* BOOT_SEL2 */
	ret = dm_gpio_lookup_name("GPIO3_20", &desc);
	if (ret) {
		printf("%s lookup GPIO@3_20 failed ret = %d\n", __func__, ret);
		return 0;
	}

	ret = dm_gpio_request(&desc, "bootsel2");
	if (ret) {
		printf("%s request bootsel2 failed ret = %d\n", __func__, ret);
		return 0;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);
		bootmode |= (dm_gpio_get_value(&desc) ? 1 : 0) << 2;

	if (bootmode == 0) {
		puts("BOOT_SEL Detected: OFF OFF OFF, Boot from Carrier SATA is not supported...\n");
		hang();
	} else if (bootmode == 4) {
		puts("BOOT_SEL Detected: OFF OFF ON, Load Image from USB0...\n");
		env_set_ulong("usb dev", 1);
		env_set("bootcmd", "usb start; run loadusbbootenv; run importusbbootenv; loadusbimage; run usbboot;");
	} else if (bootmode == 2) {
		puts("BOOT_SEL Detected: OFF ON OFF, Boot from Carrier eSPI is not supported...\n");
		hang();
	} else if (bootmode == 1) {
		puts("BOOT_SEL Detected: ON OFF OFF, Load Image from Carrier SD Card...\n");
		env_set_ulong("mmcdev", 1);
		env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run loadimage; run mmcboot;");
	} else if (bootmode == 6) {
		puts("BOOT_SEL Detected: OFF ON ON, Load Image from Module eMMC Flash...\n");
		env_set_ulong("mmcdev", 0);
		env_set("bootcmd", "mmc rescan; run loadbootenv; run importbootenv; run loadimage; run mmcboot;");
	} else if (bootmode == 5) {
		puts("BOOT_SEL Detected: ON OFF ON, Load zImage from GBE...\n");
		env_set("bootcmd", "run netboot;");
	} else if (bootmode == 3) {
		puts("BOOT_SEL Detected: ON ON OFF, Boot from Carrier SPI is not supported...\n");
		hang();
	} else if (bootmode == 7) {
		puts("BOOT_SEL Detected: ON ON ON, Boot from Module SPI is not supported...\n");
		hang();
	} else {
		puts("unsupported boot devices\n");
		hang();
	}

	return 0;
}

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	sc_bool_t status = SC_FALSE;

	sc_misc_get_button_status(-1, &status);
	return (bool)status;
}
#endif

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /* TODO */
}
#endif /* CONFIG_ANDROID_RECOVERY */
#endif /* CONFIG_FSL_FASTBOOT */
