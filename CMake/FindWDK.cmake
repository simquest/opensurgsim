# - Try to find the Windows Driver Kit (WDK, formerly WinDDK).
#
# Once done this will define
#  WDK_FOUND - true if the system has GLUT
#  WDK_INCLUDE_DIR - the GLUT include directory (not including the GL/)
#  WDK_LIBRARIES - the GLUT libraries

# This file is a part of the OpenSurgSim project.
# Copyright 2013, SimQuest Solutions Inc.


# Cache settings and WinDDK environment variables take precedence,
# or we try to fall back to the default search.

find_path(WDK_INCLUDE_DIR
	NAMES hidsdi.h
	PATHS "$ENV{WINDDK_PATH}" "$ENV{WINDDK_ROOT}" "$ENV{WDK_PATH}"
	PATH_SUFFIXES inc/api
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)
find_path(WDK_INCLUDE_DIR
	NAMES hidsdi.h
	PATH_SUFFIXES inc/api
)
mark_as_advanced(WDK_INCLUDE_DIR)

# magical WinDDK system subdirectory names
set(WDK_os_winXP "wxp")
set(WDK_os_server2003 "wnet")
set(WDK_os_winVista "wlh")
set(WDK_os_server2008 "wlh")
set(WDK_os_win7 "win7")

set(LIB_ARCH "unknown")
if(CMAKE_CL_64)
	set(LIB_ARCH "amd64")
else()
	set(LIB_ARCH "i386")
endif()

if(WDK_INCLUDE_DIR)
	get_filename_component(WDK_DIR_UP1
		${WDK_INCLUDE_DIR} PATH)
	get_filename_component(WDK_ROOT_DIR
		${WDK_DIR_UP1} PATH)
endif(WDK_INCLUDE_DIR)


macro(winddk_find_library LIB_NAME)
	# find the library in the oldest available OS version directory
	find_library(WDK_${LIB_NAME}_LIBRARY
		NAMES ${LIB_NAME}
		HINTS "${WDK_ROOT_DIR}"
		PATH_SUFFIXES "lib/${WDK_os_winXP}/${LIB_ARCH}"
			"lib/${WDK_os_server2003}/${LIB_ARCH}"
			"lib/${WDK_os_winVista}/${LIB_ARCH}"
			"lib/${WDK_os_server2008}/${LIB_ARCH}"
			"lib/${WDK_os_win7}/${LIB_ARCH}"
		NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
	)
	find_library(WDK_${LIB_NAME}_LIBRARY
		NAMES ${LIB_NAME}
		HINTS "${WDK_ROOT_DIR}"
		PATH_SUFFIXES "lib/${WDK_os_winXP}/${LIB_ARCH}"
			"lib/${WDK_os_server2003}/${LIB_ARCH}"
			"lib/${WDK_os_winVista}/${LIB_ARCH}"
			"lib/${WDK_os_server2008}/${LIB_ARCH}"
			"lib/${WDK_os_win7}/${LIB_ARCH}"
	)
	mark_as_advanced(WDK_${LIB_NAME}_LIBRARY)
endmacro()

winddk_find_library(hid)
winddk_find_library(setupapi)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(WDK
	DEFAULT_MSG WDK_INCLUDE_DIR WDK_hid_LIBRARY WDK_setupapi_LIBRARY)

if(WDK_FOUND)
	set(WDK_LIBRARIES
		${WDK_hid_LIBRARY} ${WDK_setupapi_LIBRARY})
endif(WDK_FOUND)
mark_as_advanced(WDK_LIBRARIES)
