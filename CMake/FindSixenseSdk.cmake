# - Try to find the Sixense SDK, used by the Razer Hydra gaming controller.
#
# Once done this will define
#  SIXENSE_SDK_FOUND - system has the Sixense SDK directory
#  SIXENSE_SDK_INCLUDE_DIR - the Sixense SDK include directory
#  SIXENSE_SDK_LIBRARIES - the Sixense SDK libraries

# This file is a part of the OpenSurgSim project.
# Copyright 2013, SimQuest Solutions Inc.


# Cache settings and Sixense SDK environment variables take
# precedence, or we try to fall back to the default search.

find_path(SIXENSE_SDK_INCLUDE_DIR
	NAMES sixense.h
	PATHS "$ENV{SIXENSE_SDK_PATH}" "$ENV{SIXENSE_ROOT}"
	PATH_SUFFIXES "include"
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)
if(WIN32)
	# Shake down some usual suspects under Windows.
	#
	# This is far from exhaustive-- Steam may not be installed in
	# C:\Program Files, and even if it is, you can install apps into
	# alternate locations.  But this does covers the most common case.
	#
	# Alternately, we could look at examining the registry; on my
	# machine, there's some interesting stuff in
	# HKLM\SOFTWARE\Wow6432Node\Microsoft\Windows\CurrentVersion\Uninstall\Steam App 42300.
	find_path(SIXENSE_SDK_INCLUDE_DIR
		NAMES sixense.h
		PATHS
			"C:/Program Files (x86)/Steam/steamapps/common/sixense sdk/SixenseSDK"
			"C:/Program Files/Steam/steamapps/common/sixense sdk/SixenseSDK"
		PATH_SUFFIXES "include"
		NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
	)
endif(WIN32)
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	# Shake down some usual suspects under Linux.
	# This is largely hypothetical.
	find_path(SIXENSE_SDK_INCLUDE_DIR
		NAMES sixense.h
		PATHS
			"$ENV{HOME}/.local/share/Steam/SteamApps/common/sixense sdk/SixenseSDK"
			"/usr/local"
		PATH_SUFFIXES "include"
		NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
	)
endif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
find_path(SIXENSE_SDK_INCLUDE_DIR
	NAMES sixense.h
	PATH_SUFFIXES "include"
)


# Look for shared libraries.  Static linking is not currently supported.
set(LIB_SUFFIX "")
#set(LIB_SUFFIX "_s")

set(LIB_ARCH "unknown")
set(LIB_SYSDIR "lib_unknown")
if(WIN32 AND MSVC)
	if(CMAKE_CL_64)
		set(LIB_ARCH "x64")
		set(LIB_SUFFIX "${LIB_SUFFIX}_x64")
	else()
		set(LIB_ARCH "win32")
	endif()
endif(WIN32 AND MSVC)
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8") # 64 bit
		set(LIB_ARCH "linux_x64")
		set(LIB_SYSDIR "lib64")
		set(LIB_SUFFIX "${LIB_SUFFIX}_x64")
	else("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8") # 32 bit
		set(LIB_ARCH "linux")
		set(LIB_SYSDIR "lib")
	endif("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
endif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

if(SIXENSE_SDK_INCLUDE_DIR)
	get_filename_component(SIXENSE_SDK_ROOT_DIR
		${SIXENSE_SDK_INCLUDE_DIR} PATH)
endif(SIXENSE_SDK_INCLUDE_DIR)


macro(sixense_shared_from_link OUTPUT)
	set(SHARED_LIST)
	foreach(FILE ${ARGN})
		if(WIN32)
			string(REPLACE "/lib/" "/bin/" SHARED "${FILE}")
			string(REPLACE ".lib" ".dll" SHARED "${SHARED}")
		else()
			set(SHARED "${FILE}")
		endif()
		if(EXISTS "${SHARED}")
			list(APPEND SHARED_LIST "${SHARED}")
		else()
			message(SEND_ERROR "Could not find dynamic library for ${FILE}")
		endif()
	endforeach(FILE ${ARGN})
	set(${OUTPUT} ${SHARED_LIST}
		CACHE STRING "DLLs/SOs from the Sixense SDK.")
	mark_as_advanced(${OUTPUT})
endmacro()

macro(sixense_find_library LIB_NAME)
	find_library(SIXENSE_SDK_${LIB_NAME}_LIBRARY_RELEASE
		NAMES ${LIB_NAME}${LIB_SUFFIX}
		HINTS "${SIXENSE_SDK_ROOT_DIR}"
		PATH_SUFFIXES "lib/${LIB_ARCH}/release_dll" "lib/${LIB_ARCH}/release"
		NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
	)
	find_library(SIXENSE_SDK_${LIB_NAME}_LIBRARY_RELEASE
		NAMES ${LIB_NAME}${LIB_SUFFIX}
		PATH_SUFFIXES "lib/${LIB_ARCH}/release_dll" "lib/${LIB_ARCH}/release"
			"${LIB_SYSDIR}"
	)

	find_library(SIXENSE_SDK_${LIB_NAME}_LIBRARY_DEBUG
		NAMES ${LIB_NAME}d${LIB_SUFFIX}
		HINTS "${SIXENSE_SDK_ROOT_DIR}"
		PATH_SUFFIXES "lib/${LIB_ARCH}/debug_dll" "lib/${LIB_ARCH}/debug"
		NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
	)
	find_library(SIXENSE_SDK_${LIB_NAME}_LIBRARY_DEBUG
		NAMES ${LIB_NAME}d${LIB_SUFFIX}
		PATH_SUFFIXES "lib/${LIB_ARCH}/debug_dll" "lib/${LIB_ARCH}/debug"
			"${LIB_SYSDIR}"
	)

	if(SIXENSE_SDK_${LIB_NAME}_LIBRARY_RELEASE AND
			SIXENSE_SDK_${LIB_NAME}_LIBRARY_DEBUG AND
			NOT SIXENSE_SDK_${LIB_NAME}_LIBRARY)
		set(SIXENSE_SDK_${LIB_NAME}_LIBRARY
			optimized ${SIXENSE_SDK_${LIB_NAME}_LIBRARY_RELEASE}
			debug     ${SIXENSE_SDK_${LIB_NAME}_LIBRARY_DEBUG}
			CACHE STRING "The ${LIB_NAME} library from the Sixense SDK.")
		mark_as_advanced(SIXENSE_SDK_${LIB_NAME}_LIBRARY)
	endif()

	if(SIXENSE_SDK_${LIB_NAME}_LIBRARY_RELEASE AND
			NOT SIXENSE_SDK_${LIB_NAME}_SHARED_RELEASE)
		sixense_shared_from_link(SIXENSE_SDK_${LIB_NAME}_SHARED_RELEASE
			"${SIXENSE_SDK_${LIB_NAME}_LIBRARY_RELEASE}")
	endif()
	if(SIXENSE_SDK_${LIB_NAME}_LIBRARY_DEBUG AND
			NOT SIXENSE_SDK_${LIB_NAME}_SHARED_DEBUG)
		sixense_shared_from_link(SIXENSE_SDK_${LIB_NAME}_SHARED_DEBUG
			"${SIXENSE_SDK_${LIB_NAME}_LIBRARY_DEBUG}")
	endif()
endmacro()

sixense_find_library(sixense)
sixense_find_library(sixense_utils)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SixenseSdk
	DEFAULT_MSG SIXENSE_SDK_ROOT_DIR SIXENSE_SDK_INCLUDE_DIR
	SIXENSE_SDK_sixense_LIBRARY SIXENSE_SDK_sixense_utils_LIBRARY)

if(SIXENSE_SDK_FOUND)
	set(SIXENSE_SDK_LIBRARIES
		${SIXENSE_SDK_sixense_LIBRARY} ${SIXENSE_SDK_sixense_utils_LIBRARY})
endif(SIXENSE_SDK_FOUND)
mark_as_advanced(SIXENSE_SDK_LIBRARIES)

if(SIXENSE_SDK_FOUND)
	# HACK: for debug, we may also need the mysterious DeviceDLL.dll on Windows.
	if(WIN32)
		find_file(SIXENSE_SDK_DeviceDLL_SHARED_DEBUG
			NAMES DeviceDLL.dll
			HINTS "${SIXENSE_SDK_ROOT_DIR}"
			PATH_SUFFIXES "samples/${LIB_ARCH}/sixense_simple3d"
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
		)
		if(NOT SIXENSE_SDK_DeviceDLL_SHARED_DEBUG)
			# if not found, clear it and hope for the best...
			message("Warning: DeviceDLL.dll (from Sixense) not found; continuing.")
			set(SIXENSE_SDK_DeviceDLL_SHARED_DEBUG
				CACHE PATH "Path to DeviceDLL, if any." FORCE)
		endif(NOT SIXENSE_SDK_DeviceDLL_SHARED_DEBUG)
	else()
		set(SIXENSE_SDK_DeviceDLL_SHARED_DEBUG
			CACHE PATH "Path to DeviceDLL, if any.")
		mark_as_advanced(SIXENSE_SDK_DeviceDLL_SHARED_DEBUG)
	endif()
endif(SIXENSE_SDK_FOUND)
