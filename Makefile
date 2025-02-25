obj-m := supercamera_simple.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build

all: driver test

driver:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

test: simple-test.c
	gcc -o simple-test simple-test.c

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
	rm -f simple-test

install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install
	/sbin/depmod -a

.PHONY: all driver clean install
