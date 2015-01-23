# - Try to find the Novint HDAL SDK, used by the Novint Falcon haptic devices.
#
# Once done this will define
#  NOVINT_HDAL_SDK_FOUND - system has the Novint HDAL SDK directory
#  NOVINT_HDAL_SDK_INCLUDE_DIR - the Novint HDAL SDK include directory
#  NOVINT_HDAL_SDK_LIBRARIES - the Novint HDAL SDK libraries

# This file is a part of the OpenSurgSim project.
# Copyright 2013, SimQuest Solutions Inc.


# Cache settings and Novint HDAL SDK environment variables take
# precedence, or we try to fall back to the default search.

find_path(NOVINT_HDAL_SDK_INCLUDE_DIR
	NAMES hdl/hdl.h
	PATHS "$ENV{NOVINT_DEVICE_SUPPORT}"
	PATH_SUFFIXES "include"
		# hack for getting the HDAL from the SimQuest-only SutureSim source tree
		"SurgTool2003/DeviceDrivers/hdal/include"
	DOC "Path in which the file hdl/hdl.h is located. File is part of HDAL SDK."
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)

# TODO(advornik): Shake down some usual suspects under Windows/Linux?
find_path(NOVINT_HDAL_SDK_INCLUDE_DIR
	NAMES hdl/hdl.h
	PATH_SUFFIXES "include"
)

if(NOVINT_HDAL_SDK_INCLUDE_DIR)
	get_filename_component(PARENT_DIR "${NOVINT_HDAL_SDK_INCLUDE_DIR}" PATH)
	set(NOVINT_HDAL_SDK_ROOT_DIRS "${PARENT_DIR}")

	# include all parent directories, too
	get_filename_component(PARENT_DIR "${PARENT_DIR}" PATH)
	get_filename_component(NEXT_PARENT_DIR "${PARENT_DIR}" PATH)
	while(NOT "${NEXT_PARENT_DIR}" STREQUAL "${PARENT_DIR}")
		# use PARENT_DIR, nor NEXT_PARENT_DIR, to stay out of the root directory
		list(APPEND NOVINT_HDAL_SDK_ROOT_DIRS "${PARENT_DIR}")
		set(PARENT_DIR "${NEXT_PARENT_DIR}")
		get_filename_component(NEXT_PARENT_DIR "${PARENT_DIR}" PATH)
	endwhile()
endif(NOVINT_HDAL_SDK_INCLUDE_DIR)

# Look for (shared) libraries.
set(LIB_SUFFIX "")
if(WIN32 AND MSVC)
	if(CMAKE_CL_64)
		set(LIB_SUFFIX "${LIB_SUFFIX}_64")
	endif()
endif(WIN32 AND MSVC)
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8") # 64 bit
		set(LIB_SUFFIX "${LIB_SUFFIX}_64")
	endif()
endif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")


macro(novint_shared_from_link OUTPUT)
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
		CACHE STRING "DLLs/SOs from the Novint HDAL SDK.")
	mark_as_advanced(${OUTPUT})
endmacro()

macro(novint_find_library LIB_NAME)
	find_library(NOVINT_HDAL_SDK_${LIB_NAME}_LIBRARY
		NAMES "${LIB_NAME}${LIB_SUFFIX}"
		HINTS ${NOVINT_HDAL_SDK_ROOT_DIRS}
		PATH_SUFFIXES "lib"
		NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
	)
	find_library(NOVINT_HDAL_SDK_${LIB_NAME}_LIBRARY
		NAMES "${LIB_NAME}${LIB_SUFFIX}"
		PATH_SUFFIXES "lib"
	)
	mark_as_advanced(NOVINT_HDAL_SDK_${LIB_NAME}_LIBRARY)

	if(NOVINT_HDAL_SDK_${LIB_NAME}_LIBRARY AND
			NOT NOVINT_HDAL_SDK_${LIB_NAME}_SHARED)
		novint_shared_from_link(NOVINT_HDAL_SDK_${LIB_NAME}_SHARED
			"${NOVINT_HDAL_SDK_${LIB_NAME}_LIBRARY}")
	endif()
endmacro()

novint_find_library(hdl)

# In order to use a hardware device, the binary needs the HDAL shared
# library (hdl.dll on Windows).  The HDAL will in turn need to load
# the hdal.ini configuration file, a number of auxiliary DLLs, .bin
# files for the hardware type, etc.  All of those will be found
# automatically based on the NOVINT_DEVICE_SUPPORT environment
# variable, so the HDAL library is the only thing we need to copy.

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NovintHdalSdk
	DEFAULT_MSG NOVINT_HDAL_SDK_INCLUDE_DIR NOVINT_HDAL_SDK_hdl_LIBRARY)

if(NOVINT_HDAL_SDK_FOUND)
	set(NOVINT_HDAL_SDK_LIBRARIES ${NOVINT_HDAL_SDK_hdl_LIBRARY})
endif(NOVINT_HDAL_SDK_FOUND)
mark_as_advanced(NOVINT_HDAL_SDK_LIBRARIES)
