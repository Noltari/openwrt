include $(TOPDIR)/rules.mk

PKG_NAME:=r8168
PKG_RELEASE:=1

PKG_LICENSE:=GPLv2
PKG_LICENSE_FILES:=

PKG_SOURCE_URL:=https://github.com/Noltari/rtl8168.git
PKG_SOURCE_PROTO:=git
PKG_SOURCE_VERSION:=b332d56f81d203b45371d549227f94eb54593576
PKG_VERSION:=v8.053.00
PKG_MIRROR_HASH:=c5db8c49aa320277cc0a8cadc4e0e0494816e622d49053c8891315ecb6b483b1

PKG_MAINTAINER:=Alvaro Fernandez Rojas <noltari@gmail.com>
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define KernelPackage/r8168
  SUBMENU:=Network Devices
  TITLE:=Realtek 8168 driver
  FILES:= $(PKG_BUILD_DIR)/r8168.ko
  AUTOLOAD:=$(call AutoProbe,r8168)
endef

define Build/Compile
	+$(KERNEL_MAKE) $(PKG_JOBS) \
		M="$(PKG_BUILD_DIR)" \
		modules
endef

$(eval $(call KernelPackage,r8168))
