#
# Copyright (C) 2020 Álvaro Fernández Rojas <noltari@gmail.com>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=SDL2
PKG_VERSION:=2.0.10
PKG_RELEASE:=1

PKG_SOURCE:=SDL2-$(PKG_VERSION).tar.gz
PKG_SOURCE_URL:=https://www.libsdl.org/release
PKG_HASH:=b4656c13a1f0d0023ae2f4a9cf08ec92fffb464e0f24238337784159b8b91d57

PKG_MAINTAINER:=Álvaro Fernández Rojas <noltari@gmail.com>

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/cmake.mk

CMAKE_BINARY_SUBDIR:=build

define Package/libsdl2
  TITLE:=Simple DirectMedia Layer
  SECTION:=multimedia
  CATEGORY:=Multimedia
  DEPENDS:=+TARGET_brcm2708:brcm2708-userland
endef

define Package/libsdl2/description
  Simple DirectMedia Layer is a cross-platform development library designed to
  provide low level access to audio, keyboard, mouse, joystick, and graphics
  hardware via OpenGL and Direct3D. It is used by video playback software,
  emulators, and popular games including Valve's award winning catalog and
  many Humble Bundle games.
endef

define Package/libsdl2/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/libSDL2*.so* $(1)/usr/lib
	$(INSTALL_DIR) $(1)/usr/lib/cmake/SDL2
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/cmake/SDL2/SDL2*.cmake $(1)/usr/lib/cmake/SDL2
	$(INSTALL_DIR) $(1)/usr/lib/pkgconfig
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/pkgconfig/sdl2.pc $(1)/usr/lib/pkgconfig
	$(INSTALL_DIR) $(1)/usr/share/aclocal
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/share/aclocal/sdl2.m4 $(1)/usr/share/aclocal
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/bin/sdl2-config $(1)/usr/bin
	$(INSTALL_DIR) $(1)/usr/include/SDL2
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/include/SDL2/*.h $(1)/usr/include/SDL2
	$(INSTALL_DIR) $(1)/usr/lib
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/libSDL2*.{a,so}* $(1)/usr/lib
	$(INSTALL_DIR) $(1)/usr/lib/cmake/SDL2
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/cmake/SDL2/SDL2*.cmake $(1)/usr/lib/cmake/SDL2
	$(INSTALL_DIR) $(1)/usr/lib/pkgconfig
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/pkgconfig/sdl2.pc $(1)/usr/lib/pkgconfig
endef

$(eval $(call BuildPackage,libsdl2))
