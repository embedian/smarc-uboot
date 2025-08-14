#
# Copyright 2022 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += smarcimx93.o ../../freescale/common/ 

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_TARGET_SMARCIMX93) += lpddr4x_timing.o
endif
