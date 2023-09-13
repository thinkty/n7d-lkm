# See https://www.kernel.org/doc/Documentation/kbuild/modules.txt
# Shared Makefile format

ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
MODULE := n7d
OBJECTS := main.o
C_FLAGS := -Wall -Werror

obj-m += $(MODULE).o
$(MODULE)-y := $(OBJECTS)
ccflags-y += $(C_FLAGS)

else
# normal makefile
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
MODULE_DIR := $(shell pwd)

default:
	$(MAKE) -C $(KERNEL_DIR) M=$(MODULE_DIR) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(MODULE_DIR) clean

endif
