# - Try to find the Windows Driver Development Kit (WinDDK).
#
# Once done this will define
#  WINDDK_FOUND - true if the system has GLUT
#  WINDDK_INCLUDE_DIR - the GLUT include directory (not including the GL/)
#  WINDDK_LIBRARIES - the GLUT libraries

# This file is a part of the OpenSurgSim project.
# Copyright 2013, SimQuest Solutions Inc.


# Cache settings and WinDDK environment variables take precedence,
# or we try to fall back to the default search.

find_path(WINDDK_INCLUDE_DIR
	NAMES hidsdi.h
	PATHS "$ENV{WINDDK_PATH}" "$ENV{WINDDK_ROOT}"
	PATH_SUFFIXES inc/api
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)
find_path(WINDDK_INCLUDE_DIR
	NAMES hidsdi.h
	PATH_SUFFIXES inc/api
)
mark_as_advanced(WINDDK_INCLUDE_DIR)

# magical WinDDK system subdirectory names
set(WINDDK_os_winXP "wxp")
set(WINDDK_os_server2003 "wnet")
set(WINDDK_os_winVista "wlh")
set(WINDDK_os_server2008 "wlh")
set(WINDDK_os_win7 "win7")

set(LIB_ARCH "unknown")
if(CMAKE_CL_64)
	set(LIB_ARCH "amd64")
else()
	set(LIB_ARCH "i386")
endif()

if(WINDDK_INCLUDE_DIR)
	get_filename_component(WINDDK_DIR_UP1
		${WINDDK_INCLUDE_DIR} PATH)
	get_filename_component(WINDDK_ROOT_DIR
		${WINDDK_DIR_UP1} PATH)
endif(WINDDK_INCLUDE_DIR)


macro(winddk_find_library LIB_NAME)
	# find the library in the oldest available OS version directory
	find_library(WINDDK_${LIB_NAME}_LIBRARY
		NAMES ${LIB_NAME}
		HINTS "${WINDDK_ROOT_DIR}"
		PATH_SUFFIXES "lib/${WINDDK_os_winXP}/${LIB_ARCH}"
			"lib/${WINDDK_os_server2003}/${LIB_ARCH}"
			"lib/${WINDDK_os_winVista}/${LIB_ARCH}"
			"lib/${WINDDK_os_server2008}/${LIB_ARCH}"
			"lib/${WINDDK_os_win7}/${LIB_ARCH}"
		NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
	)
	find_library(WINDDK_${LIB_NAME}_LIBRARY
		NAMES ${LIB_NAME}
		HINTS "${WINDDK_ROOT_DIR}"
		PATH_SUFFIXES "lib/${WINDDK_os_winXP}/${LIB_ARCH}"
			"lib/${WINDDK_os_server2003}/${LIB_ARCH}"
			"lib/${WINDDK_os_winVista}/${LIB_ARCH}"
			"lib/${WINDDK_os_server2008}/${LIB_ARCH}"
			"lib/${WINDDK_os_win7}/${LIB_ARCH}"
	)
	mark_as_advanced(WINDDK_${LIB_NAME}_LIBRARY)
endmacro()

winddk_find_library(hid)
winddk_find_library(setupapi)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(WinDDK
	DEFAULT_MSG WINDDK_INCLUDE_DIR WINDDK_hid_LIBRARY WINDDK_setupapi_LIBRARY)

if(WINDDK_FOUND)
	set(WINDDK_LIBRARIES
		${WINDDK_hid_LIBRARY} ${WINDDK_setupapi_LIBRARY})
endif(WINDDK_FOUND)
mark_as_advanced(WINDDK_LIBRARIES)
