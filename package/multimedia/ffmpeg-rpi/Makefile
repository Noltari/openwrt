#
# Copyright (C) 2017-2019 Ian Leonard <antonlacon@gmail.com>
# Copyright (C) 2018 Ted Hess <thess@kitschensync.net>
# Copyright (C) 2020 Álvaro Fernández Rojas <noltari@gmail.com>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=ffmpeg-rpi
PKG_VERSION:=4.2.2
PKG_RELEASE:=1

PKG_SOURCE:=ffmpeg-$(PKG_VERSION).tar.xz
PKG_SOURCE_URL:=https://ffmpeg.org/releases/
PKG_HASH:=cb754255ab0ee2ea5f66f8850e1bd6ad5cac1cd855d0a2f4990fb8c668b0d29c

PKG_LICENSE:=LGPL-2.1-or-later GPL-2.0-or-later LGPL-3.0-or-later
PKG_LICENSE_FILES:=COPYING.GPLv2 COPYING.GPLv3 COPYING.LGPLv2.1 COPYING.LGPLv3

PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)

include $(INCLUDE_DIR)/package.mk

TAR_OPTIONS:=--strip-components 1 $(TAR_OPTIONS)
TAR_CMD=$(HOST_TAR) -C $(1) $(TAR_OPTIONS)

FFMPEG_CONFIGURE := \
	CFLAGS="$(TARGET_CFLAGS) $(TARGET_CPPFLAGS) $(FPIC)" \
	LDFLAGS="$(TARGET_LDFLAGS)" \
	./configure \
	--enable-cross-compile \
	--cross-prefix="$(TARGET_CROSS)" \
	--arch="$(ARCH)" \
	$(if $(REAL_CPU_TYPE),--cpu=$(call qstrip,$(REAL_CPU_TYPE)),) \
	--target-os=linux \
	--prefix=$(CONFIGURE_PREFIX) \
	--pkg-config="pkg-config" \
	--disable-armv5te \
	--disable-autodetect \
	--disable-debug \
	--disable-runtime-cpudetect \
	--enable-bzlib \
	--enable-hardcoded-tables \
	--enable-decoder=ac3 \
	--enable-decoder=dca \
	--enable-decoder=eac3 \
	--enable-decoder=flac \
	--enable-decoder=h263 \
	--enable-decoder=h264 \
	--enable-decoder=mlp \
	--enable-decoder=mjpeg \
	--enable-decoder=mjpegb \
	--enable-decoder=mpegvideo \
	--enable-decoder=mpeg1video \
	--enable-decoder=mpeg2video \
	--enable-decoder=mpeg4 \
	--enable-decoder=opus \
	--enable-decoder=theora \
	--enable-decoder=truehd \
	--enable-decoder=vc1 \
	--enable-decoder=vp3 \
	--enable-decoder=vp6 \
	--enable-decoder=vp6f \
	--enable-decoder=vp8 \
	--enable-decoder=wmv3 \
	--enable-mmal \
	--enable-sdl2 \
	--enable-shared \
	--enable-static \
	--enable-zlib

define Package/ffmpeg-rpi
  TITLE:=FFmpeg RPi binaries
  URL:=https://ffmpeg.org/
  SECTION:=multimedia
  CATEGORY:=Multimedia
  DEPENDS:=+TARGET_brcm2708:brcm2708-userland +libpthread +zlib +libbz2 +libffmpeg-rpi
endef

define Package/ffmpeg-rpi/description
  FFmpeg is a a software package that can record, convert and stream digital
  audio and video in numerous formats.

  FFmpeg licensing / patent issues are complex. It is the reponsibility of the
  user to understand any requirements in this regard with its usage. See:
  https://ffmpeg.org/legal.html for further information.
endef

define Package/ffmpeg-rpi/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/bin/{ffmpeg,ffplay,ffprobe} $(1)/usr/bin
endef

define Package/libffmpeg-rpi
  TITLE:=FFmpeg RPi libraries
  URL:=https://ffmpeg.org/
  SECTION:=libs
  CATEGORY:=Libraries
  DEPENDS:=+TARGET_brcm2708:brcm2708-userland +libpthread +zlib +libbz2 +libsdl2
endef

define Package/libffmpeg-rpi/description
  FFmpeg is a a software package that can record, convert and stream digital
  audio and video in numerous formats.

  FFmpeg licensing / patent issues are complex. It is the reponsibility of the
  user to understand any requirements in this regard with its usage. See:
  https://ffmpeg.org/legal.html for further information.
endef

define Package/libffmpeg-rpi/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/lib{avdevice,avcodec,avfilter,avformat,avutil,swresample,swscale}.so.* $(1)/usr/lib
endef

define Build/Configure
	( cd $(PKG_BUILD_DIR); $(FFMPEG_CONFIGURE) )
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		DESTDIR="$(PKG_INSTALL_DIR)" \
		all install
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include
	$(CP) $(PKG_INSTALL_DIR)/usr/include/lib{avdevice,avcodec,avfilter,avformat,avutil,swresample,swscale} $(1)/usr/include
	$(INSTALL_DIR) $(1)/usr/lib
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/lib{avdevice,avcodec,avfilter,avformat,avutil,swresample,swscale}.so* $(1)/usr/lib
	$(INSTALL_DIR) $(1)/usr/lib/pkgconfig
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/pkgconfig/lib{avdevice,avcodec,avfilter,avformat,avutil,swresample,swscale}.pc $(1)/usr/lib/pkgconfig
endef

$(eval $(call BuildPackage,ffmpeg-rpi))
$(eval $(call BuildPackage,libffmpeg-rpi))
