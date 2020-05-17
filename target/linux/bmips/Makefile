include $(TOPDIR)/rules.mk

ARCH:=mips
CPU_TYPE:=mips32
BOARD:=bmips
BOARDNAME:=Broadcom BMIPS
SUBTARGETS:=generic nand
FEATURES:=pci squashfs usb
KERNEL_PATCHVER:=5.4

define Target/Description
	Build firmware images for BCM33xx cable modem chips,
	BCM63xx DSL chips and BCM7xxx set-top box chips.
endef

include $(INCLUDE_DIR)/target.mk

DEFAULT_PACKAGES += swconfig kmod-gpio-button-hotplug

$(eval $(call BuildTarget))
