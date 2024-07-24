/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX7.
 */

#ifndef __MX7SMARC_COMMON_H
#define __MX7SMARC_COMMON_H

#include <linux/sizes.h>
#include <linux/stringify.h>
#include <asm/arch/imx-regs.h>
#include <asm/mach-imx/gpio.h>

/* Timer settings */
#define CFG_SC_TIMER_CLK 8000000 /* 8Mhz */

/* UART */
#if defined(CONFIG_CONSOLE_SER0)
#define CFG_MXC_UART_BASE		UART6_IPS_BASE_ADDR
#define CONSOLE_DEV			"ttymxc5"
#endif
#if defined(CONFIG_COMSOLE_SER1)
#define CFG_MXC_UART_BASE		UART2_IPS_BASE_ADDR
#define CONSOLE_DEV			"ttymxc1"
#endif
#if defined(CONFIG_CONSOLE_SER2)
#define CFG_MXC_UART_BASE		UART7_IPS_BASE_ADDR
#define CONSOLE_DEV			"ttymxc6"
#endif
#if defined(CONFIG_CONSOLE_SER3)
#define CFG_MXC_UART_BASE		UART3_IPS_BASE_ADDR
#define CONSOLE_DEV			"ttymxc2"
#endif

/* NET PHY */
#define PHY_ANEG_TIMEOUT 20000

#ifdef CONFIG_IMX_OPTEE
#define TEE_ENV "tee=yes\0"
#else
#define TEE_ENV "tee=no\0"
#endif

#endif
