include $(TOPDIR)/rules.mk

PKG_NAME:=r8125
PKG_RELEASE:=1

PKG_LICENSE:=GPLv2
PKG_LICENSE_FILES:=

PKG_SOURCE_URL:=https://github.com/Noltari/rtl8125.git
PKG_SOURCE_PROTO:=git
PKG_SOURCE_VERSION:=74a4b8259789ef342e2c2701b18a629b16aea4d9
PKG_VERSION:=v9.013.02
PKG_MIRROR_HASH:=e4eee47763e0471d16149a264e14dee4bf5835516f1422a64d09c3f2adbafbd3

PKG_MAINTAINER:=Alvaro Fernandez Rojas <noltari@gmail.com>
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define KernelPackage/r8125
  SUBMENU:=Network Devices
  TITLE:=Realtek 8125 driver
  FILES:= $(PKG_BUILD_DIR)/r8125.ko
  AUTOLOAD:=$(call AutoProbe,r8125)
endef

define Build/Compile
	+$(KERNEL_MAKE) $(PKG_JOBS) \
		M="$(PKG_BUILD_DIR)" \
		modules
endef

$(eval $(call KernelPackage,r8125))
