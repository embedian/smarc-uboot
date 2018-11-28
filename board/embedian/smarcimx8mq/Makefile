#
# Copyright 2016 Freescale Semiconductor
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += smarcimx8mq.o

obj-$(CONFIG_POWER_PFUZE100) += ../../freescale/common/pfuze.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-y += ddr/ddr_init.o ddr/ddrphy_train.o ddr/helper.o
endif
