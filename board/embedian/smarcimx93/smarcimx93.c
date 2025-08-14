// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#include <common.h>
#include <env.h>
#include <hang.h>
#include <efi_loader.h>
#include <init.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/global_data.h>
#include <asm/arch-imx9/ccm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch-imx9/imx93_pins.h>
#include <asm/arch/clock.h>
#include <power/pmic.h>
#include "../../freescale/common/tcpc.h"
#include <dm/device.h>
#include <dm/uclass.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <asm/gpio.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE(6) | PAD_CTL_FSEL2)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE(6) | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

/* SER0 */
#ifdef CONFIG_CONSOLE_SER0
static iomux_v3_cfg_t const uart6_pads[] = {
	MX93_PAD_GPIO_IO05__LPUART6_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX93_PAD_GPIO_IO04__LPUART6_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

/* SER1 */
#ifdef CONFIG_CONSOLE_SER1
static iomux_v3_cfg_t const uart2_pads[] = {
	MX93_PAD_UART2_RXD__LPUART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX93_PAD_UART2_TXD__LPUART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

/* SER2 */
#ifdef CONFIG_CONSOLE_SER2
static iomux_v3_cfg_t const uart8_pads[] = {
	MX93_PAD_GPIO_IO13__LPUART8_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX93_PAD_GPIO_IO12__LPUART8_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

/* SER3 */
#ifdef CONFIG_CONSOLE_SER3
static iomux_v3_cfg_t const uart1_pads[] = {
	MX93_PAD_UART1_RXD__LPUART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX93_PAD_UART1_TXD__LPUART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
#define IMX_BOOT_IMAGE_GUID \
	EFI_GUID(0xbc550d86, 0xda26, 0x4b70, 0xac, 0x05, \
		 0x2a, 0x44, 0x8e, 0xda, 0x6f, 0x21)

struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"SMARCIMX93-RAW",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 0=flash-bin raw 0 0x2000 mmcpart 1",
	.num_images = ARRAY_SIZE(fw_images),
	.images = fw_images,
};

#endif /* EFI_HAVE_CAPSULE_SUPPORT */

int board_early_init_f(void)
{
#ifdef CONFIG_CONSOLE_SER0
	imx_iomux_v3_setup_multiple_pads(uart6_pads, ARRAY_SIZE(uart6_pads));

#endif
#ifdef CONFIG_CONSOLE_SER1
        imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));

#endif
#ifdef CONFIG_CONSOLE_SER2
        imx_iomux_v3_setup_multiple_pads(uart8_pads, ARRAY_SIZE(uart8_pads));

#endif
#ifdef CONFIG_CONSOLE_SER3
        imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));

#endif

	return 0;
}

#ifdef CONFIG_USB_TCPC
struct tcpc_port port1;
struct tcpc_port port2;
struct tcpc_port portpd;

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
		return setup_pd_switch(2, 0x71);
	} else if (port == &port2) {
		debug("Setup pd switch on port 2\n");
		return setup_pd_switch(2, 0x73);
	} else
		return -EINVAL;
}

struct tcpc_port_config portpd_config = {
	.i2c_bus = 2, /*i2c3*/
	.addr = 0x52,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 20000,
	.max_snk_ma = 3000,
	.max_snk_mw = 15000,
	.op_snk_mv = 9000,
};

struct tcpc_port_config port1_config = {
	.i2c_bus = 2, /*i2c3*/
	.addr = 0x50,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 5000,
	.max_snk_ma = 3000,
	.max_snk_mw = 40000,
	.op_snk_mv = 9000,
	.switch_setup_func = &pd_switch_snk_enable,
	.disable_pd = true,
};

struct tcpc_port_config port2_config = {
	.i2c_bus = 2, /*i2c3*/
	.addr = 0x51,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 9000,
	.max_snk_ma = 3000,
	.max_snk_mw = 40000,
	.op_snk_mv = 9000,
	.switch_setup_func = &pd_switch_snk_enable,
	.disable_pd = true,
};

static int setup_typec(void)
{
	int ret;

	debug("tcpc_init port pd\n");
	ret = tcpc_init(&portpd, portpd_config, NULL);
	if (ret) {
		printf("%s: tcpc portpd init failed, err=%d\n",
		       __func__, ret);
	}

	debug("tcpc_init port 2\n");
	ret = tcpc_init(&port2, port2_config, NULL);
	if (ret) {
		printf("%s: tcpc port2 init failed, err=%d\n",
		       __func__, ret);
	}

	debug("tcpc_init port 1\n");
	ret = tcpc_init(&port1, port1_config, NULL);
	if (ret) {
		printf("%s: tcpc port1 init failed, err=%d\n",
		       __func__, ret);
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

	return ret;
}

int board_ehci_usb_phy_mode(struct udevice *dev)
{
	int ret = 0;
	enum typec_cc_polarity pol;
	enum typec_cc_state state;
	struct tcpc_port *port_ptr;

	debug("%s %d\n", __func__, dev_seq(dev));

	if (dev_seq(dev) == 0)
		port_ptr = &port1;
	else
		port_ptr = &port2;

	tcpc_setup_ufp_mode(port_ptr);

	ret = tcpc_get_cc_status(port_ptr, &pol, &state);

	tcpc_print_log(port_ptr);
	if (!ret) {
		if (state == TYPEC_STATE_SRC_RD_RA || state == TYPEC_STATE_SRC_RD)
			return USB_INIT_HOST;
	}

	return USB_INIT_DEVICE;
}
#endif

static int setup_fec(void)
{
	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_init(void)
{
#ifdef CONFIG_USB_TCPC
	setup_typec();
#endif

	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();

	return 0;
}

int board_late_init(void)
{
	puts("---------Embedian SMARC-iMX93------------\n");
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

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "SMARCIMX93");
	env_set("board_rev", "iMX93");
#endif

        /* BOOT_SEL0 */
        ret = dm_gpio_lookup_name("gpio@23_21", &desc);
        if (ret) {
                printf("%s lookup gpio@23_21 failed ret = %d\n", __func__, ret);
                return 0;
        }

        ret = dm_gpio_request(&desc, "BOOT_SEL_0");
        if (ret) {
                printf("%s request BOOT_SEL_0 failed ret = %d\n", __func__, ret);
                return 0;
        }

        dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);
                bootmode |= (dm_gpio_get_value(&desc) ? 1 : 0) << 0;

        /* BOOT_SEL1 */
        ret = dm_gpio_lookup_name("gpio@23_22", &desc);
        if (ret) {
                printf("%s lookup gpio@23_22 failed ret = %d\n", __func__, ret);
                return 0;
        }

        ret = dm_gpio_request(&desc, "BOOT_SEL_1");
        if (ret) {
                printf("%s request BOOT_SEL_1 failed ret = %d\n", __func__, ret);
                return 0;
        }

        dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);
               bootmode |= (dm_gpio_get_value(&desc) ? 1 : 0) << 1;

        /* BOOT_SEL2 */
        ret = dm_gpio_lookup_name("gpio@23_23", &desc);
        if (ret) {
                printf("%s lookup gpio@23_23 failed ret = %d\n", __func__, ret);
                return 0;
        }

        ret = dm_gpio_request(&desc, "BOOT_SEL_2");
        if (ret) {
                printf("%s request BOOT_SEL_2 failed ret = %d\n", __func__, ret);
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

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0;
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/
