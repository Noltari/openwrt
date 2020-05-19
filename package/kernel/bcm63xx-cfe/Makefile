include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=bcm63xx-cfe
PKG_RELEASE:=1

PKG_SOURCE_URL:=https://github.com/openwrt/bcm63xx-cfe.git
PKG_SOURCE_PROTO:=git
PKG_SOURCE_DATE:=2020-05-20
PKG_SOURCE_VERSION:=424b57c2433b62094d2941fc9be796a29fd1085f
PKG_MIRROR_HASH:=e46555e93066dd0a199460a0d4b91c05e1904a2f22bfc5fa8deee6b07e80cfb8

PKG_FLAGS:=nonshared

include $(INCLUDE_DIR)/package.mk

define Package/bcm63xx-cfe
  SECTION:=boot
  CATEGORY:=Boot Loaders
  DEPENDS:=@TARGET_bcm63xx
  TITLE:=bcm63xx-cfe
  DEFAULT:=y if TARGET_bcm63xx
endef

define Package/bcm63xx-cfe/description
  CFE RAM binaries for bcm63xx.
endef

define Build/Compile
	true
endef

define Package/bcm63xx-cfe/install
	true
endef

define Build/InstallDev
	rm -rf $(KERNEL_BUILD_DIR)/$(PKG_NAME)
	mkdir -p $(KERNEL_BUILD_DIR)/$(PKG_NAME)
	$(CP) -r $(PKG_BUILD_DIR)/* $(KERNEL_BUILD_DIR)/$(PKG_NAME)
endef

$(eval $(call BuildPackage,bcm63xx-cfe))
