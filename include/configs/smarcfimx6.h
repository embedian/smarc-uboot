/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 */

#ifndef __SMARCFIMX6_CONFIG_H
#define __SMARCFIMX6_CONFIG_H

#ifdef CONFIG_CONSOLE_SER0
#define CFG_MXC_UART_BASE	UART1_BASE
#define CONSOLE_DEV		"ttymxc0"
#endif
#ifdef CONFIG_CONSOLE_SER1
#define CFG_MXC_UART_BASE	UART2_BASE
#define CONSOLE_DEV		"ttymxc1"
#endif
#ifdef CONFIG_CONSOLE_SER2
#define CFG_MXC_UART_BASE	UART4_BASE
#define CONSOLE_DEV		"ttymxc3"
#endif
#ifdef CONFIG_CONSOLE_SER3
#define CFG_MXC_UART_BASE	UART5_BASE
#define CONSOLE_DEV		"ttymxc4"
#endif

#define CONFIG_MMCROOT			"/dev/mmcblk2p2"	/* SDHC3 */

#if defined(CONFIG_MX6Q) || defined(CONFIG_MX6QP)
#define PHYS_SDRAM_SIZE		(1u * 1024 * 1024 * 1024)
#elif defined(CONFIG_MX6DL)
#define PHYS_SDRAM_SIZE		(1u * 1024 * 1024 * 1024)
#elif defined(CONFIG_MX6S)
#define PHYS_SDRAM_SIZE		(512u * 1024 * 1024)
#endif

#include "smarcfimx6_common.h"

/* Falcon Mode */

/* Falcon Mode - MMC support: args@1MB kernel@2MB */

#define CFG_SYS_FSL_USDHC_NUM	3

/* PWM Configs */
#define CONFIG_PWM_IMX
#define CFG_IMX6_PWM_PER_CLK	66000000

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CFG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CFG_MXC_USB_FLAGS		0
#endif

#endif                         /* __SMARCFIMX6_CONFIG_H */
