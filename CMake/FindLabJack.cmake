# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.


# - Try to find LabJack
#
# Once done this will define
#  LABJACK_FOUND - system has LabJack lib with correct version
#  LABJACK_INCLUDE_DIR - the LabJack include directory

# Attempt to define LABJACK_INCLUDE_DIR if undefined
find_path(LABJACK_INCLUDE_DIR
	NAMES LabJackUD.h labjackusb.h
	PATHS "$ENV{LABJACK_SDK}" "/usr/local" "/usr"
	PATH_SUFFIXES "include"
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)

if(LABJACK_INCLUDE_DIR)
	set(LABJACK_ROOT_DIR ${LABJACK_INCLUDE_DIR})
endif(LABJACK_INCLUDE_DIR)

if(WIN32 AND CMAKE_CL_64)
	 find_library(LABJACK_LIBRARY
		NAMES LabJackUD
		PATHS ${LABJACK_ROOT_DIR}/64bit
		NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
	)
endif(WIN32 AND CMAKE_CL_64)

find_library(LABJACK_LIBRARY
	NAMES LabJackUD labjackusb
	HINTS ${LABJACK_ROOT_DIR} "/usr/local/lib" "/usr/lib"
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)
mark_as_advanced(LABJACK_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LabJack
	DEFAULT_MSG LABJACK_ROOT_DIR LABJACK_INCLUDE_DIR
	LABJACK_LIBRARY)

if(LABJACK_FOUND)
	set(LABJACK_LIBRARIES ${LABJACK_LIBRARY})
endif(LABJACK_FOUND)
