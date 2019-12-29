include $(TOPDIR)/rules.mk

PKG_NAME:=mpv
PKG_VERSION:=0.31.0
PKG_RELEASE:=1

PKG_SOURCE:=v$(PKG_VERSION).tar.gz
PKG_SOURCE_URL:=https://github.com/mpv-player/mpv/archive/
PKG_HASH:=805a3ac8cf51bfdea6087a6480c18835101da0355c8e469b6d488a1e290585a5

PKG_MAINTAINER:=Álvaro Fernández Rojas <noltari@gmail.com>

PKG_LICENSE:=GPL-2.0-or-later
PKG_LICENSE_FILES:=LICENSE.GPL

include $(INCLUDE_DIR)/package.mk

#TAR_OPTIONS:=--strip-components 1 $(TAR_OPTIONS)
#TAR_CMD=$(HOST_TAR) -C $(1) $(TAR_OPTIONS)

WAF_FILE:=waf-2.0.9

define Download/waf
  FILE:=$(WAF_FILE)
  URL:=https://waf.io/
  URL_FILE:=$(WAF_FILE)
  HASH:=2a8e0816f023995e557f79ea8940d322bec18f286917c8f9a6fa2dc3875dfa48
endef
$(eval $(call Download,waf))

CONFIGURE_CMD=./waf

CONFIGURE_ARGS:=$(filter-out \
	--target=% \
	--host=% \
	--build=% \
	--program-prefix=% \
	--program-suffix=% \
	--disable-nls \
	--disable-ipv6 \
	--sbindir=% \
	--libexecdir=% \
	--localstatedir=% \
	--infodir=% \
	, $(CONFIGURE_ARGS))

CONFIGURE_ARGS += \
	--disable-libass \
	--disable-build-date \
	--enable-sdl2

CONFIGURE_ARGS:=configure $(CONFIGURE_ARGS)

define Package/mpv
  SECTION:=multimedia
  CATEGORY:=Multimedia
  TITLE:=MPV Player
  URL:=https://mpv.io/
  DEPENDS:=+libffmpeg-rpi +libsdl2 +liblua
endef

define Package/mpv/description
  mpv is a free (as in freedom) media player for the command line.
  It supports a wide variety of media file formats, audio and video codecs
  and subtitle types.
endef

define Package/mpv/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/bin/mpv $(1)/usr/bin
	$(INSTALL_DIR) $(1)/etc/mpv
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/etc/mpv/encoding-profiles.conf $(1)/etc/mpv
	$(INSTALL_DIR) $(1)/usr/share/doc/mpv
	$(CP) $(PKG_INSTALL_DIR)/usr/share/doc/mpv* $(1)/usr/share/doc/mpv
	$(INSTALL_DIR) $(1)/usr/share/icons
	$(CP) $(PKG_INSTALL_DIR)/usr/share/icons/* $(1)/usr/share/icons
endef

define Build/Prepare
	$(Build/Prepare/Default)
	$(INSTALL_BIN) $(DL_DIR)/$(WAF_FILE) $(PKG_BUILD_DIR)/waf
endef

define Build/Compile
	(cd $(PKG_BUILD_DIR); \
		./waf \
		--jobs=$(shell nproc) \
		--destdir="$(PKG_INSTALL_DIR)" \
	)
endef

define Build/Install
	(cd $(PKG_BUILD_DIR); \
		./waf install \
		--jobs=$(shell nproc) \
		--destdir="$(PKG_INSTALL_DIR)" \
	)
endef

$(eval $(call BuildPackage,mpv))
