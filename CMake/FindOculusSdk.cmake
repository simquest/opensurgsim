# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.


# - Try to find Oculus SDK
#
# Once done this will define
#  OCULUSSDK_FOUND - system has OculusSDK lib with correct version
#  OCULUSSDK_INCLUDE_DIR - the OculusSDK include directory

# Attempt to define OCULUSSDK_INCLUDE_DIR if undefined
find_path(OCULUSSDK_INCLUDE_DIR
	NAMES OVR.h
	PATHS "$ENV{OCULUSSDK_DIR}/LibOVR" "/usr/local/LibOVR"
	PATH_SUFFIXES "Include"
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)

if(OCULUSSDK_INCLUDE_DIR)
	get_filename_component(OCULUSSDK_ROOT_DIR
		${OCULUSSDK_INCLUDE_DIR} PATH)
endif(OCULUSSDK_INCLUDE_DIR)

macro(oculussdk_find_library LIB_NAME)
	if(WIN32)
		if(CMAKE_CL_64)
			set(SEARCH_PATH ${OCULUSSDK_ROOT_DIR}/lib/x64)
			set(LIBNAME "${LIB_NAME}64")
		else()
			set(SEARCH_PATH ${OCULUSSDK_ROOT_DIR}/lib/Win32)
			set(LIBNAME "${LIB_NAME}")
		endif()

		if (MSVC_VERSION EQUAL 1600)
			set(SEARCH_PATH ${SEARCH_PATH}/VS2010)
		else (MSVC_VERSION EQUAL 1700)
			set(SEARCH_PATH ${SEARCH_PATH}/VS2012)
		else (MSVC_VERSION EQUAL 1800)
			set(SEARCH_PATH ${SEARCH_PATH}/VS2013)
		endif()

		find_library(OCULUSSDK_LIBRARY_RELEASE
			NAMES "${LIBNAME}"
			HINTS ${SEARCH_PATH}
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
			)
		find_library(OCULUSSDK_LIBRARY_DEBUG
			NAMES "${LIBNAME}d"
			HINTS ${SEARCH_PATH}
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
			)
	else()
		find_library(OCULUSSDK_LIBRARY_RELEASE
			NAMES "${LIB_NAME}.a"
			HINTS ${OCULUSSDK_ROOT_DIR}/Lib/Linux/Release/x86_64
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
			)
			
		find_library(OCULUSSDK_LIBRARY_DEBUG
			NAMES "${LIB_NAME}.a"
			HINTS ${OCULUSSDK_ROOT_DIR}/Lib/Linux/Debug/x86_64
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
			)
	endif()
	
	if(NOT OCULUSSDK_LIBRARY)
		if(OCULUSSDK_LIBRARY_RELEASE AND OCULUSSDK_LIBRARY_DEBUG)
			set(OCULUSSDK_LIBRARY
				optimized ${OCULUSSDK_LIBRARY_RELEASE}
				debug     ${OCULUSSDK_LIBRARY_DEBUG}
				CACHE STRING "The ${LIB_NAME} library from Oculus SDK.")
		endif()
		
		if(OCULUSSDK_LIBRARY_RELEASE AND NOT OCULUSSDK_LIBRARY_DEBUG)
			set(OCULUSSDK_LIBRARY
				${OCULUSSDK_LIBRARY_RELEASE}
				CACHE STRING "The release version of ${LIB_NAME} library from Oculus SDK.")
		endif()
		
		if(NOT OCULUSSDK_LIBRARY_RELEASE AND OCULUSSDK_LIBRARY_DEBUG)
			set(OCULUSSDK_LIBRARY
				${OCULUSSDK_LIBRARY_DEBUG}
				CACHE STRING "The debug version ${LIB_NAME} library from Oculus SDK.")
		endif()
		
		mark_as_advanced(OCULUSSDK_LIBRARY)
	endif()
endmacro()

oculussdk_find_library(libovr)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OculusSdk
	DEFAULT_MSG OCULUSSDK_ROOT_DIR OCULUSSDK_INCLUDE_DIR
	OCULUSSDK_LIBRARY)

if(OCULUSSDK_FOUND)
	#Feb-9-2015, this is a work around for a bug in 0.4.4-beta OculusSDK on Windows and Linux.
	if(WIN32)
		set(OCULUSSDK_LIBRARIES ${OCULUSSDK_LIBRARY} winmm.lib; ws2_32.lib)
	else() #Work around for Linux
		set(OCULUSSDK_LIBRARIES ${OCULUSSDK_LIBRARY} X11 GL Xrandr)
	endif()
endif(OCULUSSDK_FOUND)
