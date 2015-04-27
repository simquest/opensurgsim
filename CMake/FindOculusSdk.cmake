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
set(OCULUSSDK_DIR CACHE PATH "The directory containing the Oculus SDK.")
mark_as_advanced(OCULUSSDK_DIR)

find_path(OCULUSSDK_INCLUDE_DIR
	NAMES OVR.h
	PATH_SUFFIXES LibOVR/Include
	HINTS
		${OCULUSSDK_DIR}
		$ENV{OCULUSSDK_DIR}
	PATHS /usr /usr/local
)
mark_as_advanced(OCULUSSDK_INCLUDE_DIR)

set(OCULUSSDK_VERSION_H "${OCULUSSDK_INCLUDE_DIR}/OVR_Version.h")
if(EXISTS "${OCULUSSDK_VERSION_H}")
	file(READ "${OCULUSSDK_VERSION_H}" _oculussdk_version_header)

	string(REGEX MATCH "define[ \t]+OVR_PRODUCT_VERSION[ \t]+([0-9]+)" _oculussdk_product_version_match "${_oculussdk_version_header}")
	set(OVR_PRODUCT_VERSION "${CMAKE_MATCH_1}")

	string(REGEX MATCH "define[ \t]+OVR_MAJOR_VERSION[ \t]+([0-9]+)" _oculussdk_major_version_match "${_oculussdk_version_header}")
	set(OVR_MAJOR_VERSION "${CMAKE_MATCH_1}")

	string(REGEX MATCH "define[ \t]+OVR_MINOR_VERSION[ \t]+([0-9]+)" _oculussdk_minor_version_match "${_oculussdk_version_header}")
	set(OVR_MINOR_VERSION "${CMAKE_MATCH_1}")
	
	string(REGEX MATCH "define[ \t]+OVR_PATCH_VERSION[ \t]+([0-9]+)" _oculussdk_patch_version_match "${_oculussdk_version_header}")
	set(OVR_PATCH_VERSION "${CMAKE_MATCH_1}")

	set(OCULUSSDK_VERSION ${OVR_PRODUCT_VERSION}.${OVR_MAJOR_VERSION}.${OVR_MINOR_VERSION}.${OVR_PATCH_VERSION})
endif()

if(WIN32)
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(ARCH x64)
	else()
		set(ARCH Win32)
	endif()

	set(WIN_DEBUG   Lib/Windows/${ARCH}/Debug/VS2012)
	set(WIN_RELEASE Lib/Windows/${ARCH}/Release/VS2012)
else()
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(ARCH x86_64)
	else()
		set(ARCH i386)
	endif()
	
	set(LINUX_DEBUG Lib/Linux/${ARCH}/Debug)
	set(LINUX_RELEASE Lib/Linux/${ARCH}/Release)
endif()

find_library(OCULUSSDK_LIBRARY_RELEASE
	NAMES libOVR libOVR.a
	HINTS
		${OCULUSSDK_DIR}/LibOVR/${LINUX_RELEASE}
		${OCULUSSDK_DIR}/LibOVR/${WIN_RELEASE}
		$ENV{OCULUSSDK_DIR}/LibOVR/${LINUX_RELEASE}
		$ENV{OCULUSSDK_DIR}/LibOVR/${WIN_RELEASE}
	PATHS /usr /usr/local
)

find_library(OCULUSSDK_LIBRARY_DEBUG
	NAMES libOVR libOVR.a
	HINTS
		${OCULUSSDK_DIR}/LibOVR/${LINUX_DEBUG}
		${OCULUSSDK_DIR}/LibOVR/${WIN_DEBUG}
		$ENV{OCULUSSDK_DIR}/LibOVR/${LINUX_DEBUG}
		$ENV{OCULUSSDK_DIR}/LibOVR/${WIN_DEBUG}
)

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
			CACHE STRING "Oculus Release library version was found.")
	endif()

	if(NOT OCULUSSDK_LIBRARY_RELEASE AND OCULUSSDK_LIBRARY_DEBUG)
		set(OCULUSSDK_LIBRARY
			${OCULUSSDK_LIBRARY_DEBUG}
			CACHE STRING "Oculus Debug library version was found.")
	endif()

	mark_as_advanced(OCULUSSDK_LIBRARY)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OculusSdk 
	FAIL_MESSAGE "OculusSDK was not found, this could be due to a missing \
environment variable or a change in library structure.\
 Please make sure version 0.5.0.1 beta is installed." 
	REQUIRED_VARS OCULUSSDK_INCLUDE_DIR OCULUSSDK_VERSION_H OCULUSSDK_LIBRARY
	VERSION_VAR OCULUSSDK_VERSION
)

#Workaround, appending missing 3rd party libraries
if(OCULUSSDK_LIBRARY)
	if(WIN32)
		set(OCULUSSDK_LIBRARY ${OCULUSSDK_LIBRARY} winmm.lib; ws2_32.lib)
	else()
		set(OCULUSSDK_LIBRARY ${OCULUSSDK_LIBRARY} X11 GL Xrandr dl)
	endif()
endif(OCULUSSDK_LIBRARY)
