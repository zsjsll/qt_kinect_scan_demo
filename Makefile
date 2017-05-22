#############################################################################
# Makefile for building: KinectV2InQt
# Generated by qmake (3.0) (Qt 5.6.1)
# Project:  KinectV2InQt.pro
# Template: app
# Command: E:\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\bin\qmake.exe -spec win32-msvc2013 "CONFIG+=debug" "CONFIG+=qml_debug" -o Makefile KinectV2InQt.pro
#############################################################################

MAKEFILE      = Makefile

first: debug
install: debug-install
uninstall: debug-uninstall
QMAKE         = E:\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\bin\qmake.exe
DEL_FILE      = del
CHK_DIR_EXISTS= if not exist
MKDIR         = mkdir
COPY          = copy /y
COPY_FILE     = copy /y
COPY_DIR      = xcopy /s /q /y /i
INSTALL_FILE  = copy /y
INSTALL_PROGRAM = copy /y
INSTALL_DIR   = xcopy /s /q /y /i
DEL_FILE      = del
SYMLINK       = $(QMAKE) -install ln -f -s
DEL_DIR       = rmdir
MOVE          = move
SUBTARGETS    =  \
		debug \
		release


debug: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Debug
debug-make_first: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Debug 
debug-all: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Debug all
debug-clean: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Debug clean
debug-distclean: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Debug distclean
debug-install: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Debug install
debug-uninstall: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Debug uninstall
release: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Release
release-make_first: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Release 
release-all: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Release all
release-clean: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Release clean
release-distclean: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Release distclean
release-install: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Release install
release-uninstall: FORCE
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Release uninstall

Makefile: KinectV2InQt.pro ..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\win32-msvc2013\qmake.conf ..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\spec_pre.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\common\angle.conf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\common\msvc-base.conf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\common\msvc-desktop.conf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\qconfig.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dcore.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dcore_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dinput.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dinput_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dlogic.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dlogic_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquick.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquick_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquickinput.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquickinput_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquickrender.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquickrender_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3drender.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3drender_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axbase.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axbase_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axcontainer.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axcontainer_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axserver.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axserver_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_bluetooth.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_bluetooth_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_bootstrap_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_clucene_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_concurrent.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_concurrent_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_core.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_core_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_dbus.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_dbus_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_designer.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_designer_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_designercomponents_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_gui.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_gui_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_help.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_help_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_labscontrols_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_labstemplates_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_location.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_location_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_multimedia.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_multimedia_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_multimediawidgets.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_multimediawidgets_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_network.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_network_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_nfc.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_nfc_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_opengl.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_opengl_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_openglextensions.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_openglextensions_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_platformsupport_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_positioning.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_positioning_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_printsupport.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_printsupport_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qml.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qml_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qmldevtools_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qmltest.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qmltest_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qtmultimediaquicktools_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_quick.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_quick_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_quickparticles_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_quickwidgets.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_quickwidgets_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_script.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_script_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_scripttools.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_scripttools_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_sensors.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_sensors_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_serialbus.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_serialbus_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_serialport.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_serialport_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_sql.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_sql_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_svg.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_svg_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_testlib.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_testlib_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_uiplugin.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_uitools.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_uitools_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webchannel.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webchannel_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webengine.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webengine_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webenginecore.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webenginecore_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webenginecoreheaders_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webenginewidgets.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webenginewidgets_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_websockets.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_websockets_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webview.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webview_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_widgets.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_widgets_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_winextras.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_winextras_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_xml.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_xml_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_xmlpatterns.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_xmlpatterns_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_zlib_private.pri \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\qt_functions.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\qt_config.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\win32\qt_config.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\win32-msvc2013\qmake.conf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\spec_post.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\exclusive_builds.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\default_pre.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\win32\default_pre.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\resolve_config.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\exclusive_builds_post.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\default_post.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\qml_debug.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\win32\rtti.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\precompile_header.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\warn_on.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\qt.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\resources.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\moc.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\win32\opengl.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\uic.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\file_copies.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\win32\windows.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\testcase_targets.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\exceptions.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\yacc.prf \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\lex.prf \
		KinectV2InQt.pro \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\lib\qtmaind.prl \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\lib\Qt5Widgets.prl \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\lib\Qt5Gui.prl \
		..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\lib\Qt5Core.prl
	$(QMAKE) -spec win32-msvc2013 "CONFIG+=debug" "CONFIG+=qml_debug" -o Makefile KinectV2InQt.pro
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\spec_pre.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\common\angle.conf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\common\msvc-base.conf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\common\msvc-desktop.conf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\qconfig.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dcore.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dcore_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dinput.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dinput_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dlogic.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dlogic_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquick.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquick_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquickinput.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquickinput_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquickrender.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3dquickrender_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3drender.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_3drender_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axbase.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axbase_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axcontainer.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axcontainer_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axserver.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_axserver_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_bluetooth.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_bluetooth_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_bootstrap_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_clucene_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_concurrent.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_concurrent_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_core.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_core_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_dbus.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_dbus_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_designer.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_designer_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_designercomponents_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_gui.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_gui_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_help.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_help_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_labscontrols_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_labstemplates_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_location.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_location_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_multimedia.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_multimedia_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_multimediawidgets.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_multimediawidgets_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_network.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_network_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_nfc.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_nfc_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_opengl.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_opengl_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_openglextensions.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_openglextensions_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_platformsupport_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_positioning.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_positioning_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_printsupport.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_printsupport_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qml.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qml_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qmldevtools_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qmltest.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qmltest_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_qtmultimediaquicktools_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_quick.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_quick_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_quickparticles_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_quickwidgets.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_quickwidgets_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_script.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_script_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_scripttools.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_scripttools_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_sensors.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_sensors_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_serialbus.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_serialbus_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_serialport.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_serialport_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_sql.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_sql_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_svg.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_svg_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_testlib.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_testlib_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_uiplugin.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_uitools.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_uitools_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webchannel.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webchannel_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webengine.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webengine_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webenginecore.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webenginecore_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webenginecoreheaders_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webenginewidgets.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webenginewidgets_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_websockets.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_websockets_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webview.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_webview_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_widgets.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_widgets_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_winextras.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_winextras_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_xml.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_xml_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_xmlpatterns.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_xmlpatterns_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\modules\qt_lib_zlib_private.pri:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\qt_functions.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\qt_config.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\win32\qt_config.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\win32-msvc2013\qmake.conf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\spec_post.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\exclusive_builds.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\default_pre.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\win32\default_pre.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\resolve_config.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\exclusive_builds_post.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\default_post.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\qml_debug.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\win32\rtti.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\precompile_header.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\warn_on.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\qt.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\resources.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\moc.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\win32\opengl.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\uic.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\file_copies.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\win32\windows.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\testcase_targets.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\exceptions.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\yacc.prf:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\mkspecs\features\lex.prf:
KinectV2InQt.pro:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\lib\qtmaind.prl:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\lib\Qt5Widgets.prl:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\lib\Qt5Gui.prl:
..\..\..\..\DevSoftware\Qt\Qt5.6.1-2013\5.6\msvc2013\lib\Qt5Core.prl:
qmake: FORCE
	@$(QMAKE) -spec win32-msvc2013 "CONFIG+=debug" "CONFIG+=qml_debug" -o Makefile KinectV2InQt.pro

qmake_all: FORCE

make_first: debug-make_first release-make_first  FORCE
all: debug-all release-all  FORCE
clean: debug-clean release-clean  FORCE
	-$(DEL_FILE) KinectV2InQt.exp
	-$(DEL_FILE) KinectV2InQt.ilk
	-$(DEL_FILE) KinectV2InQt.idb
distclean: debug-distclean release-distclean  FORCE
	-$(DEL_FILE) Makefile
	-$(DEL_FILE) KinectV2InQt.lib KinectV2InQt.pdb

debug-mocclean:
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Debug mocclean
release-mocclean:
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Release mocclean
mocclean: debug-mocclean release-mocclean

debug-mocables:
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Debug mocables
release-mocables:
	@set MAKEFLAGS=$(MAKEFLAGS)
	$(MAKE) -f $(MAKEFILE).Release mocables
mocables: debug-mocables release-mocables

check: first

benchmark: first
FORCE:

$(MAKEFILE).Debug: Makefile
$(MAKEFILE).Release: Makefile
