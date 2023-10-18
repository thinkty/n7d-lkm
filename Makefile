# See https://www.kernel.org/doc/Documentation/kbuild/modules.txt

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
MODULE_SRC := $(shell pwd)

default:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODULE_SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODULE_SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODULE_SRC) clean

.PHONY: default clean
