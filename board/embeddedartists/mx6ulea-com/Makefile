# (C) Copyright 2015 Embedded Artists AB.
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := mx6ulea-com.o

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
