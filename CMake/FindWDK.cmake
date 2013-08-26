# - Try to find the Windows Driver Kit (WDK, formerly WinDDK).
#
# Once done this will define
#  WDK_FOUND - true if the system has GLUT
#  WDK_INCLUDE_DIR - the GLUT include directory (not including the GL/)
#  WDK_LIBRARIES - the GLUT libraries

# This file is a part of the OpenSurgSim project.
# Copyright 2013, SimQuest Solutions Inc.


# We try finding the necessary files in the compiler's default include
# and library paths.  If not found, then cache settings and WinDDK
# environment variables take precedence, or we try to fall back to the
# default search.

if(NOT DEFINED WDK_CAN_BUILD_DIRECTLY)
	# Note: check_include_files will NOT find hidsdi.h, even if it's
	#   present in the compiler's default search path!  Sigh.
	try_compile(WDK_CAN_BUILD_DIRECTLY
		${CMAKE_BINARY_DIR}/CMakeFiles/tryCompileWdk
		${CMAKE_SOURCE_DIR}/CMake/tryCompileWdk  tryCompileWdk
		OUTPUT_VARIABLE dbg_tryCompileWdk
	)
	#message("OUT:${dbg_tryCompileWdk}")
endif(NOT DEFINED WDK_CAN_BUILD_DIRECTLY)

if(WDK_CAN_BUILD_DIRECTLY)
	# No include directory is needed.  Set it to . for convenience in other files.
	set(WDK_INCLUDE_DIR . CACHE PATH "Include directory for the WDK.")
	set(WDK_location provided_by_compiler)
else()
	# Find the include directory.
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
	set(WDK_location ${WDK_INCLUDE_DIR})
	if(WDK_INCLUDE_DIR)
		get_filename_component(WDK_DIR_UP1
			${WDK_INCLUDE_DIR} PATH)
		get_filename_component(WDK_ROOT_DIR
			${WDK_DIR_UP1} PATH)
	endif(WDK_INCLUDE_DIR)
endif()
mark_as_advanced(WDK_INCLUDE_DIR)

# magical WinDDK system subdirectory names
set(WDK710_os_winXP "wxp")
set(WDK710_os_server2003 "wnet")
set(WDK710_os_winVista "wlh")
set(WDK710_os_server2008 "wlh")
set(WDK710_os_win7 "win7")

set(WDK710_LIB_ARCH "unknown")
if(CMAKE_CL_64)
	set(WDK710_LIB_ARCH "amd64")
else()
	set(WDK710_LIB_ARCH "i386")
endif()



macro(winddk_find_library LIB_NAME)
	if(WDK_CAN_BUILD_DIRECTLY)
		# just assume we can get at this library.
		# Note: find_library and check_library_exists will NOT find the
		#   WDK 8 libraries that are present in the compiler's default
		#   search path!  Sigh.
		set(WDK_${LIB_NAME}_LIBRARY ${LIB_NAME}
			CACHE FILEPATH "Library for the WDK.")
	else()
		# find the library in the oldest available OS version directory
		find_library(WDK_${LIB_NAME}_LIBRARY
			NAMES ${LIB_NAME}
			HINTS "${WDK_ROOT_DIR}"
			PATH_SUFFIXES "lib/${WDK710_os_winXP}/${WDK710_LIB_ARCH}"
				"lib/${WDK710_os_server2003}/${WDK710_LIB_ARCH}"
				"lib/${WDK710_os_winVista}/${WDK710_LIB_ARCH}"
				"lib/${WDK710_os_server2008}/${WDK710_LIB_ARCH}"
				"lib/${WDK710_os_win7}/${WDK710_LIB_ARCH}"
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
		)
		find_library(WDK_${LIB_NAME}_LIBRARY
			NAMES ${LIB_NAME}
			HINTS "${WDK_ROOT_DIR}"
			PATH_SUFFIXES "lib/${WDK710_os_winXP}/${WDK710_LIB_ARCH}"
				"lib/${WDK710_os_server2003}/${WDK710_LIB_ARCH}"
				"lib/${WDK710_os_winVista}/${WDK710_LIB_ARCH}"
				"lib/${WDK710_os_server2008}/${WDK710_LIB_ARCH}"
				"lib/${WDK710_os_win7}/${WDK710_LIB_ARCH}"
		)
	endif()
	mark_as_advanced(WDK_${LIB_NAME}_LIBRARY)
endmacro()

winddk_find_library(hid)
winddk_find_library(setupapi)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(WDK
	DEFAULT_MSG WDK_location
	WDK_INCLUDE_DIR WDK_hid_LIBRARY WDK_setupapi_LIBRARY)

if(WDK_FOUND)
	set(WDK_LIBRARIES
		${WDK_hid_LIBRARY} ${WDK_setupapi_LIBRARY})
endif(WDK_FOUND)
mark_as_advanced(WDK_LIBRARIES)
