// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019 NXP
 */

#include <common.h>
#include <efi_loader.h>
#include <env.h>
#include <errno.h>
#include <hang.h>
#include <init.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/delay.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mp_pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <spl.h>
#include <asm/mach-imx/dma.h>
#include <power/pmic.h>
#include "../../freescale/common/tcpc.h"
#include <usb.h>
#include <dwc3-uboot.h>
#include <imx_sip.h>
#include <linux/arm-smccc.h>
#include <mmc.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)
#define WEAK_PULLUP	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart4_pads[] = {
	MX8MP_PAD_UART4_RXD__UART4_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX8MP_PAD_UART4_TXD__UART4_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	MX8MP_PAD_GPIO1_IO02__WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

/* MISC PINs */
static iomux_v3_cfg_t const misc_pads[] = {
	MX8MP_PAD_SAI2_MCLK__GPIO4_IO27 | MUX_PAD_CTRL(WEAK_PULLUP),	/* Watchdog Enable */
	MX8MP_PAD_GPIO1_IO11__GPIO1_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),	/* TS_RST# */
	MX8MP_PAD_SAI1_RXD0__GPIO4_IO02 | MUX_PAD_CTRL(WEAK_PULLUP),	/* WDT_TIME_OUT# */
	MX8MP_PAD_NAND_DQS__GPIO3_IO14 | MUX_PAD_CTRL(WEAK_PULLUP),	/* USB2_HUB_EN */
	MX8MP_PAD_I2C4_SCL__PCIE_CLKREQ_B | MUX_PAD_CTRL(WEAK_PULLUP),	/* PCIE_A_CKREQ# */
	MX8MP_PAD_GPIO1_IO05__GPIO1_IO05 | MUX_PAD_CTRL(WEAK_PULLUP),	/* BOOT_SEL0# */
	MX8MP_PAD_I2C4_SDA__GPIO5_IO21 | MUX_PAD_CTRL(WEAK_PULLUP),	/* USER_LED */
	MX8MP_PAD_NAND_DATA00__GPIO3_IO06 | MUX_PAD_CTRL(WEAK_PULLUP),	/* CAM1_PWR# */
	MX8MP_PAD_NAND_DATA01__GPIO3_IO07 | MUX_PAD_CTRL(WEAK_PULLUP),	/* CAM1_RST# */
	MX8MP_PAD_SAI3_RXC__GPIO4_IO29 | MUX_PAD_CTRL(WEAK_PULLUP),	/* CAM1_VSYNC */
	MX8MP_PAD_SAI2_RXFS__GPIO4_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL),	/*ENET_INT# */
	MX8MP_PAD_SAI1_RXD1__GPIO4_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL),	/*ENET1_INT# */
	MX8MP_PAD_SAI5_RXD0__GPIO3_IO21 | MUX_PAD_CTRL(WEAK_PULLUP),	/*LCD0_BKLT_PWM */
	MX8MP_PAD_SAI1_TXD6__GPIO4_IO18 | MUX_PAD_CTRL(WEAK_PULLUP),	/*LCD0_BKLT Enable */
	MX8MP_PAD_SAI1_RXC__GPIO4_IO01 | MUX_PAD_CTRL(WEAK_PULLUP),	/*LCD0_VDD_EN */
};

static void setup_iomux_misc(void)
{
	imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));

	/* Set M.2_IRQ# as Input */
	gpio_request(IMX_GPIO_NR(2, 20), "M2_IRQ#");
	gpio_direction_input(IMX_GPIO_NR(2, 20));
	/* Set TPM_IRQ# as Input */
	gpio_request(IMX_GPIO_NR(4, 19), "TPM_IRQ#");
	gpio_direction_input(IMX_GPIO_NR(4, 19));
	/* Set TPM_PP as Input */
	gpio_request(IMX_GPIO_NR(5, 11), "TPM_PP");
	gpio_direction_input(IMX_GPIO_NR(5, 11));
	/* Set TPM_LP as Input */
	gpio_request(IMX_GPIO_NR(5, 10), "TPM_LP");
	gpio_direction_input(IMX_GPIO_NR(5, 10));
	/* Set TCA6408_IO_Expander_INT# (M.2) as Input */
	gpio_request(IMX_GPIO_NR(4, 22), "TCA6408_M2_INT#");
	gpio_direction_input(IMX_GPIO_NR(4, 22));
	/* Set TCA6408_IO_Expander_INT# (MPCIe) as Input */
	gpio_request(IMX_GPIO_NR(5, 00), "TCA6408_MPCIE_INT#");
	gpio_direction_input(IMX_GPIO_NR(5, 00));
	/* Set AUD_INT# as Input */
	gpio_request(IMX_GPIO_NR(4, 28), "AUD_INT#");
	gpio_direction_input(IMX_GPIO_NR(4, 28));
	/* Set TS_INT# as Input */
	gpio_request(IMX_GPIO_NR(1, 15), "TS_INT#");
	gpio_direction_input(IMX_GPIO_NR(1, 15));
	/* Reset TS_RST# */
	gpio_request(IMX_GPIO_NR(1, 11), "TS_RST#");
	gpio_direction_output(IMX_GPIO_NR(1, 11), 0);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(1, 11), 1);
	/* Set Watchdog Enable as Output Low */
	gpio_request(IMX_GPIO_NR(4, 27), "WDT_EN");
	gpio_direction_output(IMX_GPIO_NR(4, 27), 0);
	/* Set WDT_TIME_OUT# as Output High */
	gpio_request(IMX_GPIO_NR(4, 2), "WDT_TIME_OUT#");
	gpio_direction_output(IMX_GPIO_NR(4, 2), 1);
	/* Set USB2 HUB Enable as Output High */
	gpio_request(IMX_GPIO_NR(3, 14), "USB2_HUB_EN");
	gpio_direction_output(IMX_GPIO_NR(3, 14), 1);
	/* Set BOOT_SEL0# as Input */
	gpio_request(IMX_GPIO_NR(1, 5), "BOOT_SEL0#");
	gpio_direction_input(IMX_GPIO_NR(1, 5));
	/* Set USER_LED as Output Low */
	gpio_request(IMX_GPIO_NR(5, 21), "USER_LED");
	gpio_direction_output(IMX_GPIO_NR(5, 21), 0);
	/* Set CAM1_PWR# as Output Low */
	gpio_request(IMX_GPIO_NR(3, 6), "CAM1_PWR#");
	gpio_direction_output(IMX_GPIO_NR(3, 6), 0);
	/* Reset CAM1_RST# */
	gpio_request(IMX_GPIO_NR(3, 7), "CAM1_RST#");
	gpio_direction_output(IMX_GPIO_NR(3, 7), 1);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(3, 7), 0);
	/* Set CAM1_VSYNC as Input */
	gpio_request(IMX_GPIO_NR(4, 29), "CAM1_VSYNC");
	gpio_direction_input(IMX_GPIO_NR(4, 29));
	/* Set CAM2_PWR# as Output Low */
	gpio_request(IMX_GPIO_NR(3, 8), "CAM2_PWR#");
	gpio_direction_output(IMX_GPIO_NR(3, 8), 0);
	/* Reset CAM2_RST# */
	gpio_request(IMX_GPIO_NR(3, 9), "CAM2_RST#");
	gpio_direction_output(IMX_GPIO_NR(3, 9), 1);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(3, 9), 0);
	/* Set CAM2_VSYNC as Input */
	gpio_request(IMX_GPIO_NR(4, 0), "CAM2_VSYNC");
	gpio_direction_input(IMX_GPIO_NR(4, 0));
	/* Turn on External IO Power */
	gpio_request(IMX_GPIO_NR(5, 05), "EIO_PWR_EN");
	gpio_direction_output(IMX_GPIO_NR(5, 05), 1);
	/* Set ENET_INT# as Input */
	gpio_request(IMX_GPIO_NR(4, 21), "ENET_INT#");
	gpio_direction_input(IMX_GPIO_NR(4, 21));
	/* Set ENET1_INT# as Input */
	gpio_request(IMX_GPIO_NR(4, 3), "ENET1_INT#");
	gpio_direction_input(IMX_GPIO_NR(4, 3));
	/* Set LCD0_BKLT_PWM Output High */
	gpio_request(IMX_GPIO_NR(3, 21), "LCD0_BKLT_PWM");
	gpio_direction_output(IMX_GPIO_NR(3, 21),1);
	/* Set LCD0_BKLT_EN Output High */
	gpio_request(IMX_GPIO_NR(4, 18), "LCD0_BKLT_EN");
	gpio_direction_output(IMX_GPIO_NR(1, 0),1);
	/* Set LCD0_VDD_EN Output High */
	gpio_request(IMX_GPIO_NR(4, 1), "LCD0_VDD_EN");
	gpio_direction_output(IMX_GPIO_NR(1, 1),1);
}

static iomux_v3_cfg_t const gpio_pads[] = {
	MX8MP_PAD_GPIO1_IO00__GPIO1_IO00 | MUX_PAD_CTRL(WEAK_PULLUP),	/* EIO_GPIO1 */
	MX8MP_PAD_GPIO1_IO01__GPIO1_IO01 | MUX_PAD_CTRL(WEAK_PULLUP),	/* EIO_GPIO2 */
	MX8MP_PAD_GPIO1_IO06__GPIO1_IO06 | MUX_PAD_CTRL(WEAK_PULLUP),	/* EIO_GPIO3 */
	MX8MP_PAD_GPIO1_IO07__GPIO1_IO07 | MUX_PAD_CTRL(WEAK_PULLUP),	/* EIO_GPIO4 */
	MX8MP_PAD_GPIO1_IO08__GPIO1_IO08 | MUX_PAD_CTRL(WEAK_PULLUP),	/* EIO_GPIO5 */
	MX8MP_PAD_GPIO1_IO09__GPIO1_IO09 | MUX_PAD_CTRL(WEAK_PULLUP),	/* EIO_GPIO6 */
};

static void setup_iomux_gpio(void)
{
	imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));

	/* Set EIO_GPIO1 as Output Low*/
	gpio_request(IMX_GPIO_NR(1, 0), "GPIO0");
	gpio_direction_input(IMX_GPIO_NR(1, 0));
	/* Set EIO_GPIO2 as Output Low*/
	gpio_request(IMX_GPIO_NR(1, 1), "GPIO1");
	gpio_direction_input(IMX_GPIO_NR(1, 1));
	/* Set EIO_GPIO3 as Output Low*/
	gpio_request(IMX_GPIO_NR(1, 6), "GPIO2");
	gpio_direction_input(IMX_GPIO_NR(1, 6));
	/* Set EIO_GPIO4 as Output Low*/
	gpio_request(IMX_GPIO_NR(1, 7), "GPIO3");
	gpio_direction_output(IMX_GPIO_NR(1, 7), 0);
	/* Set EIO_GPIO5 as Output Low*/
	gpio_request(IMX_GPIO_NR(1, 8), "GPIO4");
	gpio_direction_output(IMX_GPIO_NR(1, 8), 0);
	/* Set EIO_GPIO6 as Output Low*/
	gpio_request(IMX_GPIO_NR(1, 9), "GPIO5");
	gpio_direction_output(IMX_GPIO_NR(1, 9), 0);
}

/* RESET_OUT */
static iomux_v3_cfg_t const reset_out_pads[] = {
	MX8MP_PAD_SAI3_RXFS__GPIO4_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_reset_out(void)
{
	imx_iomux_v3_setup_multiple_pads(reset_out_pads, ARRAY_SIZE(reset_out_pads));

	/* Set CPU RESET_OUT as Output */
	gpio_request(IMX_GPIO_NR(4, 28), "CPU_RESET");
	gpio_direction_output(IMX_GPIO_NR(4, 28) , 0);
}

#ifdef CONFIG_NAND_MXS

static void setup_gpmi_nand(void)
{
	init_nand_clk();
}
#endif

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"IMX8MP-SMARC-RAW",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 2=flash-bin raw 0 0x2000 mmcpart 1",
	.images = fw_images,
};

u8 num_image_type_guids = ARRAY_SIZE(fw_images);
#endif /* EFI_HAVE_CAPSULE_SUPPORT */

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	setup_iomux_reset_out();

	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
	init_uart_clk(3);

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
#ifdef CONFIG_IMX8M_DRAM_INLINE_ECC
#ifdef CONFIG_TARGET_IMX8MP_DDR4_EVK
	int rc;
	phys_addr_t ecc_start = 0x120000000;
	size_t ecc_size = 0x20000000;

	rc = add_res_mem_dt_node(blob, "ecc", ecc_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc reserved-memory node.\n");
		return rc;
	}
#else
	int rc;
	phys_addr_t ecc0_start = 0xb0000000;
	phys_addr_t ecc1_start = 0x130000000;
	phys_addr_t ecc2_start = 0x1b0000000;
	size_t ecc_size = 0x10000000;

	rc = add_res_mem_dt_node(blob, "ecc", ecc0_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc0 reserved-memory node.\n");
		return rc;
	}

	rc = add_res_mem_dt_node(blob, "ecc", ecc1_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc1 reserved-memory node.\n");
		return rc;
	}

	rc = add_res_mem_dt_node(blob, "ecc", ecc2_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc2 reserved-memory node.\n");
		return rc;
	}
#endif
#endif

	return 0;
}
#endif

/*USB Enable Over-Current Pin Setting*/
static iomux_v3_cfg_t const usb_en_oc_pads[] = {
	MX8MP_PAD_GPIO1_IO12__GPIO1_IO12 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX8MP_PAD_GPIO1_IO13__GPIO1_IO13 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX8MP_PAD_GPIO1_IO10__GPIO1_IO10 | MUX_PAD_CTRL(WEAK_PULLUP),	/* USB1_ID */
};

static void setup_iomux_usb_en_oc(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_en_oc_pads,
					ARRAY_SIZE(usb_en_oc_pads));

	gpio_request(IMX_GPIO_NR(1, 12), "usb0_en");
	gpio_direction_output(IMX_GPIO_NR(1, 12), 1);
	gpio_request(IMX_GPIO_NR(1, 13), "usb0_oc#");
	gpio_direction_input(IMX_GPIO_NR(1, 13));
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
	} else
		return -EINVAL;
}

/* Port2 is the power supply, port 1 does not support power */
struct tcpc_port_config port1_config = {
	.i2c_bus = 1, /*i2c2*/
	.addr = 0x50,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 20000,
	.max_snk_ma = 3000,
	.max_snk_mw = 45000,
	.op_snk_mv = 15000,
	.switch_setup_func = &pd_switch_snk_enable,
	.disable_pd = true,
};

struct tcpc_port_config port2_config = {
	.i2c_bus = 2, /*i2c3*/
	.addr = 0x50,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 20000,
	.max_snk_ma = 3000,
	.max_snk_mw = 45000,
	.op_snk_mv = 15000,
};

#define USB_TYPEC_SEL IMX_GPIO_NR(4, 20)
#define USB_TYPEC_EN IMX_GPIO_NR(2, 20)

static iomux_v3_cfg_t ss_mux_gpio[] = {
	MX8MP_PAD_SAI1_MCLK__GPIO4_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX8MP_PAD_SD2_WP__GPIO2_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void ss_mux_select(enum typec_cc_polarity pol)
{
	if (pol == TYPEC_POLARITY_CC1)
		gpio_direction_output(USB_TYPEC_SEL, 0);
	else
		gpio_direction_output(USB_TYPEC_SEL, 1);
}

static int setup_typec(void)
{
	int ret;
	struct gpio_desc per_12v_desc;

	debug("tcpc_init port 2\n");
	ret = tcpc_init(&port2, port2_config, NULL);
	if (ret) {
		printf("%s: tcpc port2 init failed, err=%d\n",
		       __func__, ret);
	} else if (tcpc_pd_sink_check_charging(&port2)) {
		printf("Power supply on USB2\n");

		/* Enable PER 12V, any check before it? */
		ret = dm_gpio_lookup_name("gpio@20_1", &per_12v_desc);
		if (ret) {
			printf("%s lookup gpio@20_1 failed ret = %d\n", __func__, ret);
			return -ENODEV;
		}

		ret = dm_gpio_request(&per_12v_desc, "per_12v_en");
		if (ret) {
			printf("%s request per_12v failed ret = %d\n", __func__, ret);
			return -EIO;
		}

		/* Enable PER 12V regulator */
		dm_gpio_set_dir_flags(&per_12v_desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	}

	debug("tcpc_init port 1\n");
	imx_iomux_v3_setup_multiple_pads(ss_mux_gpio, ARRAY_SIZE(ss_mux_gpio));
	gpio_request(USB_TYPEC_SEL, "typec_sel");
	gpio_request(USB_TYPEC_EN, "typec_en");
	gpio_direction_output(USB_TYPEC_EN, 0);

	ret = tcpc_init(&port1, port1_config, &ss_mux_select);
	if (ret) {
		printf("%s: tcpc port1 init failed, err=%d\n",
		       __func__, ret);
	} else {
		return ret;
	}

	return ret;
}
#endif

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

#define USB_PHY_CTRL6			0xF0058

#define HSIO_GPR_BASE                               (0x32F10000U)
#define HSIO_GPR_REG_0                              (HSIO_GPR_BASE)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT    (1)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN          (0x1U << HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT)


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

int usb_gadget_handle_interrupts(int index)
{
	dwc3_uboot_handle_interrupt(index);
	return 0;
}

static void dwc3_nxp_usb_phy_init(struct dwc3_device *dwc3)
{
	u32 RegData;

	/* enable usb clock via hsio gpr */
	RegData = readl(HSIO_GPR_REG_0);
	RegData |= HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN;
	writel(RegData, HSIO_GPR_REG_0);

	/* USB3.0 PHY signal fsel for 100M ref */
	RegData = readl(dwc3->base + USB_PHY_CTRL0);
	RegData = (RegData & 0xfffff81f) | (0x2a<<5);
	writel(RegData, dwc3->base + USB_PHY_CTRL0);

	RegData = readl(dwc3->base + USB_PHY_CTRL6);
	RegData &=~0x1;
	writel(RegData, dwc3->base + USB_PHY_CTRL6);

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

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
#define USB2_PWR_EN IMX_GPIO_NR(1, 14)
int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;
	imx8m_usb_power(index, true);

	if (index == 0 && init == USB_INIT_DEVICE) {
#ifdef CONFIG_USB_TCPC
		ret = tcpc_setup_ufp_mode(&port1);
		if (ret)
			return ret;
#endif
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
	} else if (index == 0 && init == USB_INIT_HOST) {
#ifdef CONFIG_USB_TCPC
		ret = tcpc_setup_dfp_mode(&port1);
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
	} else if (index == 0 && init == USB_INIT_HOST) {
#ifdef CONFIG_USB_TCPC
		ret = tcpc_disable_src_vbus(&port1);
#endif
	}

	imx8m_usb_power(index, false);

	return ret;
}

#ifdef CONFIG_USB_TCPC
/* Not used so far */
int board_typec_get_mode(int index)
{
	int ret = 0;
	enum typec_cc_polarity pol;
	enum typec_cc_state state;

	if (index == 0) {
		tcpc_setup_ufp_mode(&port1);

		ret = tcpc_get_cc_status(&port1, &pol, &state);
		if (!ret) {
			if (state == TYPEC_STATE_SRC_RD_RA || state == TYPEC_STATE_SRC_RD)
				return USB_INIT_HOST;
		}

		return USB_INIT_DEVICE;
	} else {
		return USB_INIT_HOST;
	}
}
#endif
#endif

static void setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	/* Enable RGMII TX clk output */
	setbits_le32(&gpr->gpr[1], BIT(22));
}

static int setup_eqos(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	/* set INTF as RGMII, enable RGMII TXC clock */
	clrsetbits_le32(&gpr->gpr[1],
			IOMUXC_GPR_GPR1_GPR_ENET_QOS_INTF_SEL_MASK, BIT(16));
	setbits_le32(&gpr->gpr[1], BIT(19) | BIT(21));

	return set_clk_eqos(ENET_125MHZ);
}

#if CONFIG_IS_ENABLED(NET)
int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

#define DISPMIX				13
#define MIPI				15

int board_init(void)
{
	struct arm_smccc_res res;
	setup_iomux_misc();
	setup_iomux_gpio();
	setup_iomux_usb_en_oc();

#ifdef CONFIG_USB_TCPC
	setup_typec();

	/* Enable USB power default */
	imx8m_usb_power(0, true);
	imx8m_usb_power(1, true);
#endif

	if (CONFIG_IS_ENABLED(FEC_MXC)) {
		setup_fec();
	}

	if (CONFIG_IS_ENABLED(DWC_ETH_QOS)) {
		setup_eqos();
	}

#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand();
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
	init_usb_clk();
#endif

	/* enable the dispmix & mipi phy power domain */
	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN,
		      DISPMIX, true, 0, 0, 0, 0, &res);
	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN,
		      MIPI, true, 0, 0, 0, 0, &res);

	return 0;
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
	u8 mac1[6];

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
	puts("---------Embedian pITX-iMX8M-PLUS---------\n");
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

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "pITX");
	env_set("board_rev", "iMX8MP");
#endif

	if ((gpio_get_value(IMX_GPIO_NR(1, 5)) == 1)) {
		puts("BOOT_SEL Detected: SD Card, Load Image from SD Card...\n");
		env_set_ulong("mmcdev", 1);
		env_set("bootcmd", "i2c dev 0; i2c mw 0x25 0x0a 0x3; mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadimage; run mmcboot;");
	} else {
		puts("BOOT_SEL Detected: eMMC, Load Image from eMMC Flash...\n");
		env_set_ulong("mmcdev", 2);
		env_set("bootcmd", "i2c dev 0; i2c mw 0x25 0x0a 0x3; mmc rescan; run loadbootenv; run importbootenv; run uenvcmd; run loadimage; run mmcboot;");
	}

	return 0;
}

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif

#ifdef CONFIG_SPL_MMC
#define UBOOT_RAW_SECTOR_OFFSET 0x40
unsigned long spl_mmc_get_uboot_raw_sector(struct mmc *mmc)
{
	u32 boot_dev = spl_boot_device();
	switch (boot_dev) {
		case BOOT_DEVICE_MMC2:
			return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR - UBOOT_RAW_SECTOR_OFFSET;
		default:
			return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR;
	}
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
