#
# Copyright 2017 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += smarcimx8mq.o ../../freescale/common/mmc.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
endif
