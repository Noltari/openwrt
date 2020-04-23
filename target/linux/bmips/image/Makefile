include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

KERNEL_LOADADDR = 0x80010000		# RAM start + 64K
KERNEL_ENTRY = $(KERNEL_LOADADDR)	# Kernel Entry at Load Address
LOADER_ENTRY = 0x80a00000		# RAM start + 10M, for relocate
RAMSIZE = 0x02000000			# 32MB
LZMA_TEXT_START = 0x81800000		# 32MB - 8MB

LOADER_MAKEOPTS = \
	KDIR=$(KDIR) \
	LOADADDR=$(KERNEL_LOADADDR) \
	KERNEL_ENTRY=$(KERNEL_ENTRY) \
	RAMSIZE=$(RAMSIZE) \
	LZMA_TEXT_START=$(LZMA_TEXT_START)

RELOCATE_MAKEOPTS = \
	CACHELINE_SIZE=16 \
	KERNEL_ADDR=$(KERNEL_ENTRY) \
	CROSS_COMPILE=$(TARGET_CROSS) \
	LZMA_TEXT_START=$(LOADER_ENTRY)

define Build/Compile
	rm -rf $(KDIR)/relocate
	$(CP) ../../generic/image/relocate $(KDIR)
	$(MAKE) -C $(KDIR)/relocate $(RELOCATE_MAKEOPTS)
endef

### Kernel scripts ###
define Build/loader-lzma
	@rm -rf $@.src
	$(MAKE) -C lzma-loader \
		$(LOADER_MAKEOPTS) \
		PKG_BUILD_DIR="$@.src" \
		TARGET_DIR="$(dir $@)" \
		LOADER_DATA="$@" \
		LOADER_NAME="$(notdir $@)" \
		compile loader.$(1)
	@mv "$@.$(1)" "$@"
	@rm -rf $@.src
endef

define Build/lzma-cfe
	# CFE is a LZMA nazi! It took me hours to find out the parameters!
	# Also I think lzma has a bug cause it generates different output depending on
	# if you use stdin / stdout or not. Use files instead of stdio here, cause
	# otherwise CFE will complain and not boot the image.
	$(call Build/lzma-no-dict,-d22 -fb64 -a1 $(1))
	# Strip out the length, CFE doesn't like this
	dd if=$@ of=$@.new bs=5 count=1
	dd if=$@ of=$@.new ibs=13 obs=5 skip=1 seek=1 conv=notrunc
	@mv $@.new $@
endef

define Build/relocate-kernel
	# CFE only allows ~4 MiB for the uncompressed kernels, but uncompressed
	# kernel might get larger than that, so let CFE unpack and load at a
	# higher address and make the kernel relocate itself to the expected
	# location.
	( \
		dd if=$(KDIR)/relocate/loader.bin bs=32 conv=sync && \
		perl -e '@s = stat("$@"); print pack("N", @s[7])' && \
		cat $@ \
	) > $@.relocate
	@mv $@.relocate $@
endef

### Image scripts ###
define rootfspad/jffs2-128k
--align-rootfs
endef
define rootfspad/jffs2-64k
--align-rootfs
endef
define rootfspad/squashfs
endef

define Image/LimitName16
$(shell expr substr "$(1)" 1 16)
endef

define Image/FileSystemStrip
$(firstword $(subst +,$(space),$(subst root.,,$(notdir $(1)))))
endef

define Build/cfe-bin
	$(STAGING_DIR_HOST)/bin/imagetag -i $(IMAGE_KERNEL) -f $(IMAGE_ROOTFS) \
		--output $@ --boardid $(CFE_BOARD_ID) --chipid $(CHIP_ID) \
		--entry $(LOADER_ENTRY) --load-addr $(LOADER_ENTRY) \
		--info1 "$(call Image/LimitName16,$(DEVICE_NAME))" \
		--info2 "$(call Image/FileSystemStrip,$(IMAGE_ROOTFS))" \
		$(call rootfspad/$(call Image/FileSystemStrip,$(IMAGE_ROOTFS))) \
		$(CFE_EXTRAS) $(1)
endef

### Device scripts ###
define Device/Default
  PROFILES = Default $$(DEVICE_NAME)
  KERNEL_DEPENDS = $$(wildcard ../dts/$$(DEVICE_DTS).dts)
  DEVICE_DTS_DIR := ../dts
  SOC = bcm$$(CHIP_ID)
  DEVICE_DTS = $$(SOC)-$(subst _,-,$(1))
endef
DEVICE_VARS += CHIP_ID

BRCMSMAC_PACKAGES := kmod-brcmsmac wpad-mini
USB1_PACKAGES := kmod-usb-ohci kmod-ledtrig-usbdev
USB2_PACKAGES := $(USB1_PACKAGES) kmod-usb2

include bcm63xx.mk

$(eval $(call BuildImage))
