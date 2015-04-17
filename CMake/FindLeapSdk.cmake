# This file is a part of the OpenSurgSim project.
# Copyright 2014, SimQuest Solutions Inc.
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


# - Try to find Leap SDK
#
# Once done this will define
#  LEAPSDK_FOUND
#  LEAPSDK_INCLUDE_DIR 
#  LEAPSDK_LIBRARY

set(LEAPSDK_DIR CACHE PATH "The directory containing the Leap Motion SDK.")
mark_as_advanced(LEAPSDK_DIR)

find_path(LEAPSDK_INCLUDE_DIR Leap.h
	HINTS
		ENV LEAPSDK_DIR
		${LEAPSDK_DIR}
	PATH_SUFFIXES include
	PATHS /usr /usr/local
)
mark_as_advanced(LEAPSDK_INCLUDE_DIR)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	set(LIB_SUFFIX x64)
else()
	set(LIB_SUFFIX x86)
endif()

find_library(LEAPSDK_LIBRARY
	NAMES libLeap Leap
	HINTS ENV LEAPSDK_DIR ${LEAPSDK_DIR}
	PATH_SUFFIXES lib/${LIB_SUFFIX}
	PATHS /usr /usr/local
)
mark_as_advanced(LEAPSDK_LIBRARY)


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LeapSdk DEFAULT_MSG LEAPSDK_LIBRARY LEAPSDK_INCLUDE_DIR)
