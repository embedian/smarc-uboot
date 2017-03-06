#
# (C) Copyright 2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# SPDX-License-Identifier:	GPL-2.0+
#

ifndef CONFIG_SPL_BUILD
obj-$(CONFIG_ID_EEPROM)	+= sys_eeprom.o
endif

ifdef CONFIG_ARM
obj-$(CONFIG_DEEP_SLEEP)		+= arm_sleep.o
endif

obj-y				+= mmc.o
ifdef CONFIG_FSL_FASTBOOT
obj-${CONFIG_ANDROID_RECOVERY} += recovery.o
endif

ifdef CONFIG_SECURE_BOOT
obj-$(CONFIG_CMD_ESBC_VALIDATE) += fsl_validate.o cmd_esbc_validate.o
endif
obj-$(CONFIG_CHAIN_OF_TRUST) += fsl_chain_of_trust.o
