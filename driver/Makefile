obj-m := supercamera_simple.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build

all: driver

driver:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install
	/sbin/depmod -a

-.PHONY: all driver clean install
