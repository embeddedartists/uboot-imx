#
# Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
#
# (C) Copyright 2011 Freescale Semiconductor, Inc.
#
# SPDX-License-Identifier:	GPL-2.0+
#


obj-y  += ea_eeprom.o
obj-y  += ea_mac.o
obj-y  += ea_gpio_expander.o
obj-y  += mmc.o
obj-y  += board.o

# TODO: common spl code??
ifndef CONFIG_TARGET_MX8MQEA_COM
obj-y  += spl.o
endif

obj-$(CONFIG_POWER_PFUZE100)    += pfuze.o
ifdef CONFIG_FSL_FASTBOOT
obj-${CONFIG_ANDROID_RECOVERY} += recovery.o
endif

