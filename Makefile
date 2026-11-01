obj-m := supercamera_simple.o

KERNEL_VERSION := $(shell uname -r)
MODULE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
KERNELDIR ?= $(if $(wildcard /lib/modules/$(KERNEL_VERSION)/build/Makefile),/lib/modules/$(KERNEL_VERSION)/build,/usr/src/linux-headers-$(KERNEL_VERSION))
MOK_KEY ?= /var/lib/shim-signed/mok/MOK.priv
MOK_CERT ?= /var/lib/shim-signed/mok/MOK.der
INSTALLED_MODULE := /lib/modules/$(KERNEL_VERSION)/updates/supercamera_simple.ko

all: driver test

driver:
	$(MAKE) -C $(KERNELDIR) M=$(MODULE_DIR) modules

test: simple-test.c
	gcc -o simple-test simple-test.c

clean:
	$(MAKE) -C $(KERNELDIR) M=$(MODULE_DIR) clean
	rm -f simple-test

install:
	$(MAKE) -C $(KERNELDIR) M=$(MODULE_DIR) modules_install
	@test -r "$(MOK_KEY)" || { echo "Missing Secure Boot signing key: $(MOK_KEY)"; exit 1; }
	@test -r "$(MOK_CERT)" || { echo "Missing Secure Boot certificate: $(MOK_CERT)"; exit 1; }
	@test -x "$(KERNELDIR)/scripts/sign-file" || { echo "Missing kernel sign-file utility"; exit 1; }
	$(KERNELDIR)/scripts/sign-file sha256 "$(MOK_KEY)" "$(MOK_CERT)" "$(INSTALLED_MODULE)"
	/sbin/depmod -a $(KERNEL_VERSION)

.PHONY: all driver clean install
