obj-m := supercamera_simple.o

KERNEL_VERSION := $(shell uname -r)
MODULE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
KERNELDIR ?= $(if $(wildcard /lib/modules/$(KERNEL_VERSION)/build/Makefile),/lib/modules/$(KERNEL_VERSION)/build,/usr/src/linux-headers-$(KERNEL_VERSION))
MOK_KEY ?= /var/lib/shim-signed/mok/MOK.priv
MOK_CERT ?= /var/lib/shim-signed/mok/MOK.der
TARGET_KERNEL = $(shell cat $(KERNELDIR)/include/config/kernel.release)
INSTALL_MOD_DIR ?= updates
INSTALLED_MODULE = $(INSTALL_MOD_PATH)/lib/modules/$(TARGET_KERNEL)/$(INSTALL_MOD_DIR)/supercamera_simple.ko

all: driver test

driver:
	$(MAKE) -C $(KERNELDIR) M=$(MODULE_DIR) modules

test: simple-test.c
	gcc -o simple-test simple-test.c

clean:
	$(MAKE) -C $(KERNELDIR) M=$(MODULE_DIR) clean
	rm -f simple-test

install: driver
	@test -r "$(MOK_KEY)" || { echo "Missing Secure Boot signing key: $(MOK_KEY)"; exit 1; }
	@test -r "$(MOK_CERT)" || { echo "Missing Secure Boot certificate: $(MOK_CERT)"; exit 1; }
	@test -x "$(KERNELDIR)/scripts/sign-file" || { echo "Missing kernel sign-file utility"; exit 1; }
	@set -e; signed=$$(mktemp); trap 'rm -f "$$signed"' EXIT; \
		cp "$(MODULE_DIR)/supercamera_simple.ko" "$$signed"; \
		"$(KERNELDIR)/scripts/sign-file" sha256 "$(MOK_KEY)" "$(MOK_CERT)" "$$signed"; \
		install -D -m 644 "$$signed" "$(INSTALLED_MODULE)"
	/sbin/depmod $(if $(INSTALL_MOD_PATH),-b "$(INSTALL_MOD_PATH)") -a "$(TARGET_KERNEL)"

.PHONY: all driver clean install
