# See https://www.kernel.org/doc/Documentation/kbuild/modules.txt

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
MODULE_SRC := $(shell pwd)

DT_OVERLAY := n7d_overlay

default: dt
	$(MAKE) -C $(KERNEL_SRC) M=$(MODULE_SRC) modules

dt: $(DT_OVERLAY).dts
	dtc -@ -I dts -O dtb -o $(DT_OVERLAY).dtbo $(DT_OVERLAY).dts

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODULE_SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODULE_SRC) clean
	rm -f $(DT_OVERLAY).dtbo

.PHONY: default clean
