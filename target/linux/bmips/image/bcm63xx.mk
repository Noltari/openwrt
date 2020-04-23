#
# BCM63xx Profiles
#

DEVICE_VARS += CFE_BOARD_ID CFE_EXTRAS FLASH_MB IMAGE_OFFSET

define Device/bcm63xx-cfe
  FILESYSTEMS := squashfs jffs2-64k jffs2-128k
  KERNEL := kernel-bin | append-dtb | relocate-kernel | lzma
  KERNEL_INITRAMFS := kernel-bin | append-dtb | lzma | loader-lzma elf
  KERNEL_INITRAMFS_SUFFIX := .elf
  IMAGES := cfe.bin sysupgrade.bin
  IMAGE/cfe.bin := cfe-bin --pad $$$$(shell expr $$$$(FLASH_MB) / 2)
  IMAGE/sysupgrade.bin := cfe-bin
  BLOCKSIZE := 0x10000
  IMAGE_OFFSET :=
  FLASH_MB :=
  CHIP_ID :=
  CFE_BOARD_ID :=
  CFE_EXTRAS := --block-size $$(BLOCKSIZE) --image-offset $$(if $$(IMAGE_OFFSET),$$(IMAGE_OFFSET),$$(BLOCKSIZE))
endef

# Legacy CFEs with specific LZMA parameters and no length
define Device/bcm63xx-cfe-legacy
  $(Device/bcm63xx-cfe)
  KERNEL := kernel-bin | append-dtb | relocate-kernel | lzma-cfe
endef

### Comtrend ###
define Device/comtrend_ar-5387un
  $(Device/bcm63xx-cfe)
  DEVICE_VENDOR := Comtrend
  DEVICE_MODEL := AR-5387un
  CHIP_ID := 6328
  CFE_BOARD_ID := 96328A-1441N1
  FLASH_MB := 16
  DEVICE_PACKAGES += $(USB2_PACKAGES) $(BRCMSMAC_PACKAGES)
endef
TARGET_DEVICES += comtrend_ar-5387un
