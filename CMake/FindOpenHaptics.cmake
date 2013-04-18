# - Try to find the Sensable OpenHaptics (specifically, the HDAPI portion).
#
# Once done this will define
#  OPENHAPTICS_FOUND - system has the OpenHaptics directory
#  OPENHAPTICS_INCLUDE_DIR - the OpenHaptics include directory
#  OPENHAPTICS_LIBRARIES - the OpenHaptics HDAPI libraries

# This file is a part of the OpenSurgSim project.
# Copyright 2013, SimQuest Solutions Inc.


# Cache settings and OpenHaptics environment variables take
# precedence, or we try to fall back to the default search.

find_path(OPENHAPTICS_INCLUDE_DIR
	NAMES HD/hd.h
	PATHS "$ENV{3DTOUCH_BASE}" "$ENV{OH_SDK_BASE}"
	PATH_SUFFIXES "include"
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)
find_path(OPENHAPTICS_INCLUDE_DIR
	NAMES include/HD/hd.h
	PATH_SUFFIXES "include"
)


set(LIB_ARCH "unknown")
if(WIN32 AND MSVC)
	if(NOT CMAKE_CL_64)
		set(LIB_ARCH "win32")
	else()
		set(LIB_ARCH "x64")
	endif()
else(WIN32 AND MSVC)
	if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
		# On Linux, the installation puts things into /usr/lib64 or
		# /usr/lib as appropriate.
		#    http://www.sensable.com/documents/documents/HW_userguide_Linux.pdf
		# If you only have one library flavor installed, things should
		# work out of the box.  (If you have both, things get messy fast,
		# and we don't have a Linux version of OpenHaptics to test against,
		# so this is likely broken for now.)
		set(LIB_ARCH "")
	else(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
		message(STATUS "Could not determine the OpenHaptics architecture!")
	endif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
endif(WIN32 AND MSVC)

if(OPENHAPTICS_INCLUDE_DIR)
	get_filename_component(OPENHAPTICS_ROOT_DIR
		${OPENHAPTICS_INCLUDE_DIR} PATH)
endif(OPENHAPTICS_INCLUDE_DIR)


find_library(OPENHAPTICS_HD_LIBRARY
	NAMES HD  # note: case doesn't matter on Windows, uppercase needed for Linux
	HINTS "${OPENHAPTICS_ROOT_DIR}"
	PATH_SUFFIXES "lib/${LIB_ARCH}" "lib"
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)
find_library(OPENHAPTICS_HD_LIBRARY
	NAMES HD
	HINTS "${OPENHAPTICS_ROOT_DIR}"
	PATH_SUFFIXES "lib/${LIB_ARCH}" "lib"
)


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenHaptics
	DEFAULT_MSG OPENHAPTICS_ROOT_DIR OPENHAPTICS_INCLUDE_DIR
	OPENHAPTICS_HD_LIBRARY)
mark_as_advanced(OPENHAPTICS_INCLUDE_DIR)
mark_as_advanced(OPENHAPTICS_HD_LIBRARY)


# Big assumption: will be using the HDAPI, without the HDU utility library.
if(OPENHAPTICS_FOUND)
	set(OPENHAPTICS_LIBRARIES ${OPENHAPTICS_HD_LIBRARY})
endif(OPENHAPTICS_FOUND)
mark_as_advanced(OPENHAPTICS_LIBRARIES)
