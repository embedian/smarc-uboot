/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2018 NXP
 */

#ifndef __SMARCIMX8MQ_H
#define __SMARCIMX8MQ_H

#include <linux/sizes.h>
#include <linux/stringify.h>
#include <asm/arch/imx-regs.h>
#include "imx_env.h"

#define CONFIG_SYS_BOOTM_LEN		(64 * SZ_1M)

#define CONFIG_SPL_MAX_SIZE		(152 * 1024)
#define CONFIG_SYS_MONITOR_LEN		(512 * 1024)

#ifdef CONFIG_SPL_BUILD
/*#define CONFIG_ENABLE_DDR_TRAINING_DEBUG*/
#define CONFIG_SPL_STACK		0x187FF0
#define CONFIG_SPL_BSS_START_ADDR      0x00180000
#define CONFIG_SPL_BSS_MAX_SIZE        0x2000	/* 8 KB */
#define CONFIG_SYS_SPL_MALLOC_START    0x42200000
#define CONFIG_SYS_SPL_MALLOC_SIZE    0x80000	/* 512 KB */
#define CONFIG_SYS_SPL_PTE_RAM_BASE    0x41580000

/* malloc f used before GD_FLG_FULL_MALLOC_INIT set */
#define CONFIG_MALLOC_F_ADDR		0x182000
/* For RAW image gives a error info not panic */
#define CONFIG_SPL_ABORT_ON_RAW_IMAGE

#undef CONFIG_DM_MMC

#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR 0x08
#endif

/* ENET Config */
/* ENET1 */
#if defined(CONFIG_FEC_MXC)
#define CONFIG_ETHPRIME                 "FEC"
#define PHY_ANEG_TIMEOUT 20000

#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_FEC_MXC_PHYADDR          1

#define IMX_FEC_BASE			0x30BE0000
#endif

#ifdef CONFIG_DISTRO_DEFAULTS
#define BOOT_TARGET_DEVICES(func) \
	func(USB, usb, 0) \
	func(MMC, mmc, 1) \
	func(MMC, mmc, 0)

#include <config_distro_bootcmd.h>
#else
#define BOOTENV
#endif

/*
 * Another approach is add the clocks for inmates into clks_init_on
 * in clk-imx8mq.c, then clk_ingore_unused could be removed.
 */
#define JAILHOUSE_ENV \
	"jh_clk= \0 " \
	"jh_mmcboot=setenv fdt_file imx8mq-smarc.dtb; " \
		"setenv jh_clk clk_ignore_unused mem=1872M; " \
			   "if run loadimage; then " \
				   "run mmcboot; " \
			   "else run jh_netboot; fi; \0" \
	"jh_netboot=setenv fdt_file imx8mq-smarc.dtb; setenv jh_clk clk_ignore_unused mem=1872MB; run netboot; \0 "

#define CONFIG_MFG_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS_DEFAULT \
	"initrd_addr=0x43800000\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"emmc_dev=0\0"\
	"sd_dev=1\0" \

/* Initial environment variables */
#define CONFIG_EXTRA_ENV_SETTINGS		\
	CONFIG_MFG_ENV_SETTINGS \
	BOOTENV \
	JAILHOUSE_ENV \
	"prepare_mcore=setenv mcore_clk clk-imx8mq.mcore_booted;\0" \
	"scriptaddr=0x43500000\0" \
	"kernel_addr_r=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
	"bsp_script=boot.scr\0" \
	"image=Image\0" \
	"splashimage=0x50000000\0" \
	"m4_bin=hello_world.bin\0" \
	"use_m4=no\0" \
	"console=ttymxc0,115200\0" \
	"fdt_addr_r=0x43000000\0"			\
	"fdt_addr=0x43000000\0"			\
	"m4_addr=0x7e0000\0" \
	"m4_addr_tmp=0x48000000\0"      \
	"fdt_high=0xffffffffffffffff\0"		\
	"boot_fdt=try\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"bootm_size=0x10000000\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=1\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"usbroot=/dev/sda2 rootwait ro\0" \
		"mmcrootfstype=ext4 rootwait\0" \
		"loadbootenv=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} uEnv.txt\0" \
		"loadusbbootenv=fatload usb 0:1 ${loadaddr} uEnv.txt\0" \
	"mmcautodetect=yes\0" \
		"importbootenv=echo Importing environment from mmc (uEnv.txt)...; " \
			"env import -t $loadaddr $filesize\0" \
		"importusbbootenv=echo Importing environment from USB (uEnv.txt)...; " \
			"env import -t $loadaddr $filesize\0" \
	"mmcargs=setenv bootargs ${jh_clk} ${mcore_clk} console=${console} ${optargs} " \
	"rootfstype=${mmcrootfstype} root=${mmcroot}\0 " \
	"usbargs=setenv bootargs ${jh_clk} console=${console} ${optargs} " \
	"rootfsusbtype=${usbrootfstype} root=${usbroot}\0 " \
	"loadbootscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bsp_script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadm4bin=load mmc ${mmcdev}:${mmcpart} ${m4_addr_tmp} ${m4_bin}\0" \
	"loadusbimage=fatload usb 0:1 ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} /dtbs/${fdt_file}\0" \
	"loadusbfdt=fatload usb 0:1 ${fdt_addr} /dtbs/${fdt_file}\0" \
	"cpm4mem=cp.b ${m4_addr_tmp} ${m4_addr} 20000\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"booti ${loadaddr} - ${fdt_addr_r}; " \
			"else " \
				"echo WARN: Cannot load the DT; " \
			"fi; " \
		"else " \
			"echo wait for boot; " \
		"fi;\0" \
	"m4boot=" \
		"if test ${m4_addr} = 0x7e0000; then " \
			"echo Booting M4 from TCM; " \
		"else " \
			"echo Booting M4 from DRAM; " \
			"dcache flush; " \
		"fi; " \
	"bootaux ${m4_addr};\0" \
	"usbboot=echo Booting from USB ...; " \
		"run usbargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadusbfdt; then " \
				"booti ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"echo WARN: Cannot load the DT; " \
			"fi; " \
		"else " \
			"echo wait for boot; " \
		"fi;\0" \
	"netargs=setenv bootargs ${jh_clk} ${mcore_clk} console=${console} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs;  " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${loadaddr} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr_r} ${fdt_file}; then " \
				"booti ${loadaddr} - ${fdt_addr_r}; " \
			"else " \
				"echo WARN: Cannot load the DT; " \
			"fi; " \
		"else " \
			"booti; " \
		"fi;\0" \
	"bsp_bootcmd=echo Running BSP bootcmd ...; " \
			"mmc dev ${mmcdev}; if mmc rescan; then " \
				"if test ${use_m4} = yes && run loadm4bin; then " \
					"run cpm4mem; " \
					"run m4boot; " \
				"fi; " \
			"echo Checking for: uEnv.txt ...; " \
			"if test -e mmc ${bootpart} /uEnv.txt; then " \
				"if run loadbootenv; then " \
					"echo Loaded environment from uEnv.txt;" \
					"run importbootenv;" \
				"fi;" \
				"echo Checking if uenvcmd is set ...;" \
				"if test -n ${uenvcmd}; then " \
					"echo Running uenvcmd ...;" \
					"run uenvcmd;" \
				"fi;" \
			"fi; " \
			"if run loadimage; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \

/* Link Definitions */

#define CONFIG_SYS_INIT_RAM_ADDR        0x40000000
#define CONFIG_SYS_INIT_RAM_SIZE        0x80000
#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_MMCROOT			"/dev/mmcblk1p2"  /* USDHC2 */

#define CONFIG_SYS_SDRAM_BASE           0x40000000
#define PHYS_SDRAM                      0x40000000
#ifdef CONFIG_2GB_LPDDR4
#define PHYS_SDRAM_SIZE			0x80000000	/* 2GB DDR */
#else
#define PHYS_SDRAM_SIZE			0x100000000	/* 4GB DDR */
#endif
#define CONFIG_BAUDRATE			115200

#ifdef CONFIG_CONSOLE_SER0
#define CONFIG_MXC_UART_BASE		UART4_BASE_ADDR
#define CONSOLE_DEV			"ttymxc3"
#endif

#ifdef CONFIG_CONSOLE_SER1
#define CONFIG_MXC_UART_BASE		UART3_BASE_ADDR
#define CONSOLE_DEV			"ttymxc2"
#endif

#ifdef CONFIG_CONSOLE_SER2
#define CONFIG_MXC_UART_BASE		UART2_BASE_ADDR
#define CONSOLE_DEV			"ttymxc1"
#endif

#ifdef CONFIG_CONSOLE_SER3
#define CONFIG_MXC_UART_BASE		UART1_BASE_ADDR
#define CONSOLE_DEV			"ttymxc0"
#endif

/* Monitor Command Prompt */
#define CONFIG_SYS_CBSIZE		1024
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR       0

#define CONFIG_CMD_READ
#define CONFIG_SERIAL_TAG
#define CONFIG_FASTBOOT_USB_DEV 0

#define CONFIG_USB_MAX_CONTROLLER_COUNT         2

#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_VBUS_DRAW 2

#ifdef CONFIG_ANDROID_SUPPORT
#include "smarcimx8mq_android.h"
#endif

#endif
