# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.


# - Try to find OptiTrack
#
# Once done this will define
#  OPTITRACK_FOUND - system has OptiTrack lib with correct version
#  OPTITRACK_INCLUDE_DIR - the OptiTrack include directory
#
# Rewritten from scratch to keep things simple and flexible.

# Attempt to define OPTITRACK_INCLUDE_DIR if undefined
find_path(OPTITRACK_INCLUDE_DIR
	NAMES cameralibrary.h linuxtrack.h
	PATHS "$ENV{NP_CAMERASDK}" "/opt/linuxtrack" "/usr/local"
	PATH_SUFFIXES "include"
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)

if(OPTITRACK_INCLUDE_DIR)
	get_filename_component(OPTITRACK_ROOT_DIR
		${OPTITRACK_INCLUDE_DIR} PATH)
endif(OPTITRACK_INCLUDE_DIR)

macro(optitrack_shared_from_link OUTPUT)
	set(SHARED_LIST)
	foreach(FILE ${ARGN})
		if(WIN32)
			string(REPLACE ".lib" ".dll" SHARED "${FILE}")
		else()
			string(REPLACE ".a" ".so" SHARED "${FILE}")
		endif()

		if(EXISTS "${SHARED}")
			list(APPEND SHARED_LIST "${SHARED}")
		else()
			message(SEND_ERROR "Could not find dynamic library for ${FILE}")
		endif()
	endforeach(FILE ${ARGN})
 
	set(${OUTPUT} ${SHARED_LIST}
		CACHE STRING "DLLs/SOs from the OptiTrack SDK.")
	mark_as_advanced(${OUTPUT})
endmacro()

macro(optiTrack_find_library LIB_NAME)
	if(WIN32)
		if(CMAKE_CL_64)
			set(LIB_SUFFIX "x64")
		endif()
		find_library(OPTITRACK_LIBRARY_RELEASE
			NAMES "${LIB_NAME}2010${LIB_SUFFIX}S"
			HINTS ${OPTITRACK_ROOT_DIR}
			PATH_SUFFIXES "lib"
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
		)
		find_library(OPTITRACK_LIBRARY_DEBUG
			NAMES "${LIB_NAME}2010${LIB_SUFFIX}D"
			HINTS ${OPTITRACK_ROOT_DIR}
			PATH_SUFFIXES "lib"
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
		)
	else()
		find_library(OPTITRACK_LIBRARY_RELEASE
			NAMES "${LIB_NAME}.a"
			HINTS ${OPTITRACK_ROOT_DIR}
			PATH_SUFFIXES "lib/linuxtrack"
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
		)
		find_library(OPTITRACK_LIBRARY_DEBUG
			NAMES "${LIB_NAME}.a"
			HINTS ${OPTITRACK_ROOT_DIR}
			PATH_SUFFIXES "lib/linuxtrack"
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
		)
	endif()
	
	if(OPTITRACK_LIBRARY_RELEASE AND
	   OPTITRACK_LIBRARY_DEBUG AND
	   NOT OPTITRACK_LIBRARY)
			set(OPTITRACK_LIBRARY
				optimized ${OPTITRACK_LIBRARY_RELEASE}
				debug     ${OPTITRACK_LIBRARY_DEBUG}
				CACHE STRING "The ${LIB_NAME} library from the OptiTrack SDK.")
			mark_as_advanced(OPTITRACK_LIBRARY)
	endif()

	if(OPTITRACK_LIBRARY_RELEASE AND
			NOT OPTITRACK_SHARED_RELEASE)
		optitrack_shared_from_link(OPTITRACK_SHARED_RELEASE
			"${OPTITRACK_LIBRARY_RELEASE}")
	endif()

	if(OPTITRACK_LIBRARY_DEBUG AND
			NOT OPTITRACK_SHARED_DEBUG)
		optitrack_shared_from_link(OPTITRACK_SHARED_DEBUG
			"${OPTITRACK_LIBRARY_DEBUG}")
	endif()
endmacro()

if(WIN32)
	optiTrack_find_library(cameralibrary)
else()
	optiTrack_find_library(liblinuxtrack)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OptiTrack
	DEFAULT_MSG OPTITRACK_ROOT_DIR OPTITRACK_INCLUDE_DIR
	OPTITRACK_LIBRARY)

if(OPTITRACK_FOUND)
	set(OPTITRACK_LIBRARIES ${OPTITRACK_LIBRARY})
endif(OPTITRACK_FOUND)
