# This file is a part of the OpenSurgSim project.
# Copyright 2015, SimQuest Solutions Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#  Find Oculus Rift DK2 SDK
#  Supported version(s):  0.5.0.1 beta
#  https://developer.oculus.com/
#
#  Script will define:
#  OCULUSSDK_FOUND       - TRUE/FALSE if Oculus SDK directory was found
#  OCULUSSDK_DIR         - Oculus SDK root directory
#  OCULUSSDK_INCLUDE_DIR - the Oculus SDK include directory
#  OCULUSSDK_LIBRARY     - the Oculus SDK library
#
#  Default OCULUSSDK_DIR is under /usr/local/. Set OCULUSSDK_DIR
#  environmental variable to override this default.

# Step 1. Search for Oculus SDK Include directory which contains OVR.h
find_path(OCULUSSDK_INCLUDE_DIR
	NAMES OVR.h
	PATH_SUFFIXES Include
	HINTS $ENV{OCULUSSDK_DIR} /usr/local
)
mark_as_advanced(OCULUSSDK_INCLUDE_DIR)

# Step 2. Define Oculus Root directory then set Cached OCULUSSDK_DIR variable
if($ENV{OCULUSSDK_DIR})
	set(OCULUSSDK_ROOT_DIR $ENV{OCULUSSDK_DIR})
else()
	get_filename_component(OCULUSSDK_ROOT_DIR ${OCULUSSDK_INCLUDE_DIR} PATH)
endif()

set(OCULUSSDK_DIR ${OCULUSSDK_ROOT_DIR} CACHE PATH "The OCULUS SDK directory.")
mark_as_advanced(OCULUSSDK_DIR)

# Step 3. Define Macro to search for Oculus Libraries
macro(oculussdk_find_library LIB_NAME)
	if(WIN32)
		if(CMAKE_CL_64)
			set(SEARCH_PATH ${OCULUSSDK_ROOT_DIR}/Lib/Windows/x64)
		else()
			set(SEARCH_PATH ${OCULUSSDK_ROOT_DIR}/Lib/Windows/Win32)
		endif()
		
		set(DEBUG_LIB_PATH ${SEARCH_PATH}/Debug)
		set(RELEASE_LIB_PATH ${SEARCH_PATH}/Release)
		
		if     (MSVC_VERSION EQUAL 1600)
			set(DEBUG_LIB_PATH ${DEBUG_LIB_PATH}/VS2010)
			set(RELEASE_LIB_PATH ${RELEASE_LIB_PATH}/VS2010)
		elseif (MSVC_VERSION EQUAL 1700)
			set(DEBUG_LIB_PATH ${DEBUG_LIB_PATH}/VS2012)
			set(RELEASE_LIB_PATH ${RELEASE_LIB_PATH}/VS2012)
		elseif (MSVC_VERSION EQUAL 1800)
			set(DEBUG_LIB_PATH ${DEBUG_LIB_PATH}/VS2013)
			set(RELEASE_LIB_PATH ${RELEASE_LIB_PATH}/VS2013)
		endif()

		find_library(OCULUSSDK_LIBRARY_RELEASE
			NAMES ${LIB_NAME}
			HINTS ${RELEASE_LIB_PATH}
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
			)
		find_library(OCULUSSDK_LIBRARY_DEBUG
			NAMES ${LIB_NAME}
			HINTS ${DEBUG_LIB_PATH}/usr/local
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
			)
	else()
		find_library(OCULUSSDK_LIBRARY_RELEASE
			NAMES "${LIB_NAME}.a"
			HINTS
				${OCULUSSDK_ROOT_DIR}/lib
				${OCULUSSDK_ROOT_DIR}/Lib/Linux/x86_64/Release
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
			)

		find_library(OCULUSSDK_LIBRARY_DEBUG
			NAMES "${LIB_NAME}.a"
			HINTS
				${OCULUSSDK_ROOT_DIR}/lib
				${OCULUSSDK_ROOT_DIR}/Lib/Linux/x86_64/Debug
			NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
			)
	endif()

	if(NOT OCULUSSDK_LIBRARY)
		if(OCULUSSDK_LIBRARY_RELEASE AND OCULUSSDK_LIBRARY_DEBUG)
			set(OCULUSSDK_LIBRARY
				optimized ${OCULUSSDK_LIBRARY_RELEASE}
				debug     ${OCULUSSDK_LIBRARY_DEBUG}
				CACHE STRING "Oculus Release and Debug libraries were FOUND.")
		endif()

		if(OCULUSSDK_LIBRARY_RELEASE AND NOT OCULUSSDK_LIBRARY_DEBUG)
			set(OCULUSSDK_LIBRARY
				${OCULUSSDK_LIBRARY_RELEASE}
				CACHE STRING "ONLY the Release version of Oculus library was found.")
		endif()

		if(NOT OCULUSSDK_LIBRARY_RELEASE AND OCULUSSDK_LIBRARY_DEBUG)
			set(OCULUSSDK_LIBRARY
				${OCULUSSDK_LIBRARY_DEBUG}
				CACHE STRING "ONLY the Debug version of the Oculus library was found.")
		endif()

		mark_as_advanced(OCULUSSDK_LIBRARY)
	endif()
endmacro()

# Step 4. Search for Oculus libraries
oculussdk_find_library("libOVR")

# Step 5. Set OCULUSSDK_FOUND variable
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OculusSdk DEFAULT_MSG
	OCULUSSDK_DIR OCULUSSDK_INCLUDE_DIR OCULUSSDK_LIBRARY)

# Step 6. Work around for library dependencies in OculusSDK 0.5+ on Windows and Linux. Feb-9-2015
if(OCULUSSDK_LIBRARY)
	if(WIN32)
		set(OCULUSSDK_LIBRARIES ${OCULUSSDK_LIBRARY} winmm.lib; ws2_32.lib)
	else()
		set(OCULUSSDK_LIBRARIES ${OCULUSSDK_LIBRARY} X11 GL Xrandr dl) # Linux
	endif()
endif(OCULUSSDK_LIBRARY)
