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

#ifdef CONFIG_SPL_BUILD
/*#define CONFIG_ENABLE_DDR_TRAINING_DEBUG*/

/* malloc f used before GD_FLG_FULL_MALLOC_INIT set */
#define CFG_MALLOC_F_ADDR		0x182000
/* For RAW image gives a error info not panic */

#define CFG_POWER_PFUZE100_I2C_ADDR 0x08
#endif

/* ENET Config */
/* ENET1 */
#if defined(CONFIG_FEC_MXC)
#define PHY_ANEG_TIMEOUT 20000
#define CFG_FEC_MXC_PHYADDR		1
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
		"setenv jh_clk kvm.enable_virt_at_load=false clk_ignore_unused mem=1872M; " \
			   "if run loadimage; then " \
				   "run mmcboot; " \
			   "else run jh_netboot; fi; \0" \
	"jh_netboot=setenv fdtfile imx8mq-evk-root.dtb; setenv jh_clk kvm.enable_virt_at_load=false clk_ignore_unused mem=1872MB; run netboot; \0 "

#define SR_IR_V2_COMMAND \
	"nodes=/soc@0/caam-sm@100000 /soc@0/bus@30000000/caam_secvio /soc@0/bus@30000000/caam-snvs@30370000 /soc@0/bus@32c00000/hdmi@32c00000 /soc@0/bus@32c00000/display-controller@32e00000 /soc@0/vpu@38300000 /soc@0/vpu_v4l2 /gpu3d@38000000 /audio-codec-bt-sco /audio-codec /sound-bt-sco /sound-wm8524 /sound-spdif /sound-hdmi-arc /binman \0" \
	"sr_ir_v2_cmd=cp.b ${fdtcontroladdr} ${fdt_addr_r} 0x10000;"\
	"fdt addr ${fdt_addr_r};"\
	"fdt set /soc@0/usb@38100000 compatible snps,dwc3;" \
	"fdt set /soc@0/usb@38200000 compatible snps,dwc3;" \
	"for i in ${nodes}; do fdt rm ${i}; done \0"

#define CFG_MFG_ENV_SETTINGS \
	CFG_MFG_ENV_SETTINGS_DEFAULT \
	"initrd_addr=0x43800000\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"emmc_dev=0\0"\
	"sd_dev=1\0" \

/* Initial environment variables */
#define CFG_EXTRA_ENV_SETTINGS		\
	CFG_MFG_ENV_SETTINGS \
	BOOTENV \
	SR_IR_V2_COMMAND \
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
		"loadbootenv=load mmc ${mmcdev}:${mmcpart} ${loadaddr} uEnv.txt\0" \
		"loadusbbootenv=load usb 0:1 ${loadaddr} uEnv.txt\0" \
	"mmcautodetect=yes\0" \
		"importbootenv=echo Importing environment from mmc (uEnv.txt)...; " \
			"env import -t $loadaddr $filesize\0" \
		"importusbbootenv=echo Importing environment from USB (uEnv.txt)...; " \
			"env import -t $loadaddr $filesize\0" \
	"mmcargs=setenv bootargs ${jh_clk} ${mcore_clk} console=${console} ${optargs} " \
	"rootfstype=${mmcrootfstype} root=${mmcroot}\0 " \
	"usbargs=setenv bootargs ${jh_clk} console=${console} ${optargs} " \
	"rootfsusbtype=${usbrootfstype} root=${usbroot}\0 " \
	"loadbootscript=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bsp_script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadm4bin=load mmc ${mmcdev}:${mmcpart} ${m4_addr_tmp} ${m4_bin}\0" \
	"loadusbimage=load usb 0:1 ${loadaddr} ${image}\0" \
	"loadfdt=load mmc ${mmcdev}:${mmcpart} ${fdt_addr} /dtbs/${fdt_file}\0" \
	"loadusbfdt=load usb 0:1 ${fdt_addr} /dtbs/${fdt_file}\0" \
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

#define CONFIG_MMCROOT			"/dev/mmcblk1p2"  /* USDHC2 */

#define CFG_SYS_INIT_RAM_ADDR		0x40000000
#define CFG_SYS_INIT_RAM_SIZE		0x80000

#define CFG_SYS_SDRAM_BASE		0x40000000
#define PHYS_SDRAM                      0x40000000
#ifdef CONFIG_2GB_LPDDR4
#define PHYS_SDRAM_SIZE			0x80000000	/* 2GB DDR */
#else
#define PHYS_SDRAM_SIZE			0xC0000000	/* 3GB DDR */
#define PHYS_SDRAM_2			0x100000000
#define PHYS_SDRAM_2_SIZE		0x40000000	/* 1GB */
#endif
#define CONFIG_BAUDRATE			115200

#ifdef CONFIG_CONSOLE_SER0
#define CFG_MXC_UART_BASE		UART_BASE_ADDR(4)
#define CONSOLE_DEV			"ttymxc3"
#endif

#ifdef CONFIG_CONSOLE_SER1
#define CFG_MXC_UART_BASE		UART_BASE_ADDR(3)
#define CONSOLE_DEV			"ttymxc2"
#endif

#ifdef CONFIG_CONSOLE_SER2
#define CFG_MXC_UART_BASE		UART_BASE_ADDR(2)
#define CONSOLE_DEV			"ttymxc1"
#endif

#ifdef CONFIG_CONSOLE_SER3
#define CFG_MXC_UART_BASE		UART_BASE_ADDR(1)
#define CONSOLE_DEV			"ttymxc0"
#endif

#define CFG_SYS_FSL_USDHC_NUM		2
#define CFG_SYS_FSL_ESDHC_ADDR		0

#ifdef CONFIG_ANDROID_SUPPORT
#include "smarcimx8mq_android.h"
#endif

#endif
