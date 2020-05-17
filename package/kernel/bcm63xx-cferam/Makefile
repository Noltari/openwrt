include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=bcm63xx-cferam
PKG_VERSION:=2020-05-10
PKG_RELEASE:=2

PKG_SOURCE_URL:=https://github.com/Noltari/bcm63xx-cferam.git
PKG_SOURCE_PROTO:=git
PKG_SOURCE_VERSION:=bc48a2657ec5a4a3fec88bb6d3bedca5de6c062f
PKG_MIRROR_HASH:=d584cbfb15acacb8e64cbc93e55663e43aa769955580f364d57a1135402143b7

PKG_BUILD_DIR:=$(KERNEL_BUILD_DIR)/$(PKG_NAME)

PKG_FLAGS:=nonshared

include $(INCLUDE_DIR)/package.mk

TAR_OPTIONS:=--strip-components 1 $(TAR_OPTIONS)
TAR_CMD=$(HOST_TAR) -C $(1) $(TAR_OPTIONS)

define Package/bcm63xx-cferam
  SECTION:=boot
  CATEGORY:=Boot Loaders
  DEPENDS:=@(TARGET_bcm63xx||TARGET_bmips)
  TITLE:=bcm63xx-cferam
  DEFAULT:=y if (TARGET_bcm63xx||TARGET_bmips)
endef

define Package/bcm63xx-cferam/description
  CFE RAM binaries for bcm63xx.
endef

define Build/Compile
	true
endef

define Package/bcm63xx-cferam/install
	true
endef

$(eval $(call BuildPackage,bcm63xx-cferam))
