include $(TOPDIR)/rules.mk

PKG_NAME:=r8126
PKG_RELEASE:=1

PKG_LICENSE:=GPLv2
PKG_LICENSE_FILES:=

PKG_SOURCE_URL:=https://github.com/Noltari/rtl8126.git
PKG_SOURCE_PROTO:=git
PKG_SOURCE_VERSION:=7262bb22bd3a20dfb47124c76d6b11587b3c5e78
PKG_VERSION:=v10.013.00
PKG_MIRROR_HASH:=4c1e071f9d27e78bc0c14bbf209b6b055918c3b2c6fe0aa257caaa3aaad558ec

PKG_MAINTAINER:=Alvaro Fernandez Rojas <noltari@gmail.com>
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define KernelPackage/r8126
  SUBMENU:=Network Devices
  TITLE:=Realtek 8126 driver
  FILES:= $(PKG_BUILD_DIR)/r8126.ko
  AUTOLOAD:=$(call AutoProbe,r8126)
endef

define Build/Compile
	+$(KERNEL_MAKE) $(PKG_JOBS) \
		M="$(PKG_BUILD_DIR)" \
		modules
endef

$(eval $(call KernelPackage,r8126))
