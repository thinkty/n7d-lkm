# See https://www.kernel.org/doc/Documentation/kbuild/modules.txt

KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
MODULE_DIR := $(shell pwd)

default:
	$(MAKE) -C $(KERNEL_DIR) M=$(MODULE_DIR) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(MODULE_DIR) clean

.PHONY: default clean