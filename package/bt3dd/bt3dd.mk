BT3DD_VERSION = local
BT3DD_SITE = $(BR2_EXTERNAL_PIBT3DSP_PATH)/bt3dd
BT3DD_SITE_METHOD = local
BT3DD_LICENSE = PolyForm-Noncommercial-1.0.0
BT3DD_LICENSE_FILES = LICENSE
# which build of this repository: the host compares it with the image it holds
BT3DD_BUILD_ID := $(shell git -C $(BR2_EXTERNAL_PIBT3DSP_PATH) describe --always --dirty --abbrev=7 2>/dev/null || echo unknown)

define BT3DD_BUILD_CMDS
	$(TARGET_CC) $(TARGET_CFLAGS) -std=gnu11 -Os -Wall -Wextra \
		-DBT3DD_BUILD='"$(BT3DD_BUILD_ID)"' \
		-o $(@D)/bt3dd $(@D)/bt3dd.c $(TARGET_LDFLAGS) -lm
endef

define BT3DD_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/bt3dd $(TARGET_DIR)/sbin/bt3dd
	echo "$(BT3DD_BUILD_ID)" > $(TARGET_DIR)/etc/bt3dd-build
endef

$(eval $(generic-package))
