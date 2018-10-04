#
# Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
#
# (C) Copyright 2011 Freescale Semiconductor, Inc.
#
# SPDX-License-Identifier:	GPL-2.0+
#


obj-y  += mx6ea_eeprom.o
obj-y  += mx6ea-com_spl.o
obj-$(CONFIG_POWER_PFUZE100)    += pfuze.o
ifdef CONFIG_FSL_FASTBOOT
obj-${CONFIG_ANDROID_RECOVERY} += recovery.o
endif

