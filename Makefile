BUILDROOT_BRANCH := 2026.02.x
BUILDROOT_SHA    := 679b9ead7620bbf193620d1ebf56f53c1764d37a
# Ubuntu 26.04's uutils install breaks Buildroot; use GNU install for the build only.
SHIMS := $(CURDIR)/.shims
export PATH := $(SHIMS):$(PATH)

# output-min: the image that ships (bt3dd only). output: the debug image
# (shell, Python, BlueZ tools, the bring-up scripts, bt3dd run by hand).
MIN_O   := $(CURDIR)/output-min
DEBUG_O := $(CURDIR)/output
BR = $(MAKE) -C buildroot BR2_EXTERNAL=$(CURDIR) O=$(1)

all: $(SHIMS)/install buildroot $(MIN_O)/.config
	$(call BR,$(MIN_O))

debug: $(SHIMS)/install buildroot $(DEBUG_O)/.config
	$(call BR,$(DEBUG_O))

buildroot:
	git clone --depth 1 --branch $(BUILDROOT_BRANCH) https://gitlab.com/buildroot.org/buildroot.git buildroot
	test "$$(git -C buildroot rev-parse HEAD)" = "$(BUILDROOT_SHA)"

$(MIN_O)/.config: configs/pi_bt3dsp_defconfig | buildroot
	$(call BR,$(MIN_O)) pi_bt3dsp_defconfig

$(DEBUG_O)/.config: configs/pi_bt3dsp_debug_defconfig | buildroot
	$(call BR,$(DEBUG_O)) pi_bt3dsp_debug_defconfig

$(SHIMS)/install:
	mkdir -p $(SHIMS)
	if [ -x /usr/bin/gnuinstall ]; then ln -sf /usr/bin/gnuinstall $@; \
	elif install --version 2>/dev/null | grep -q GNU; then ln -sf $$(command -v install) $@; \
	else echo "GNU install is needed (Ubuntu 26.04: apt install coreutils-from-gnu)" >&2; exit 1; fi

# Host test of the timing core (sync.h).
test:
	$(CC) -std=gnu11 -O2 -Wall -Wextra -o $(CURDIR)/.test_sync bt3dd/test_sync.c -lm
	$(CURDIR)/.test_sync

# Boot the Pi plugged into this machine (inner "USB" port, no SD card). No sudo:
# Ubuntu's rpiboot package installs a udev rule (70-rpiboot.rules) that gives
# the logged-in user the Pi's boot ROM (0a5c:2763) and second stage (2764).
boot:
	rpiboot -d $(MIN_O)/images/boot

boot-debug:
	rpiboot -d $(DEBUG_O)/images/boot

# The boot files as one archive (what the add-on bundles), and the sources of
# everything in the image (GPL), for a release.
VERSION := $(shell sed -n 's/^#define VERSION "\(.*\)"/\1/p' bt3dd/bt3dd.c)
dist: all
	mkdir -p dist
	tar -C $(MIN_O)/images -czf dist/pi-bt3dsp-$(VERSION).tar.gz boot
	cd dist && sha256sum pi-bt3dsp-$(VERSION).tar.gz > pi-bt3dsp-$(VERSION).tar.gz.sha256

dist-debug: debug
	mkdir -p dist
	tar -C $(DEBUG_O)/images -czf dist/pi-bt3dsp-$(VERSION)-debug.tar.gz boot
	cd dist && sha256sum pi-bt3dsp-$(VERSION)-debug.tar.gz > pi-bt3dsp-$(VERSION)-debug.tar.gz.sha256

legal-info: all
	$(call BR,$(MIN_O)) legal-info
	mkdir -p dist
	tar -C $(MIN_O) -czf dist/pi-bt3dsp-$(VERSION)-sources.tar.gz legal-info

version:
	@echo $(VERSION)

.PHONY: all debug test boot boot-debug dist dist-debug legal-info version
