# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.


# - Try to find Ascension Technology Corporation's 3D Guidance SDK.
#
# Once done this will define
#  ATC3DG_FOUND - system has ATC 3DG lib with correct version
#  ATC3DG_INCLUDE_DIR - the ATC 3DG include directory

# Attempt to define ATC3DG_INCLUDE_DIR if undefined
find_path(ATC3DG_INCLUDE_DIR
	NAMES ATC3DG.h
	PATHS "$ENV{ATC3DG_SDK}"
	PATH_SUFFIXES ""
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)

if(CMAKE_CL_64)
	 find_library(ATC3DG_LIBRARY
		NAMES ATC3DG64
		PATHS ${ATC3DG_INCLUDE_DIR}
		NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
	)
endif(CMAKE_CL_64)

find_library(ATC3DG_LIBRARY
	NAMES ATC3DG
	HINTS ${ATC3DG_INCLUDE_DIR}
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)
mark_as_advanced(ATC3DG_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ATC3DG
	DEFAULT_MSG ATC3DG_INCLUDE_DIR ATC3DG_LIBRARY)

if(ATC3DG_FOUND)
	set(ATC3DG_LIBRARIES ${ATC3DG_LIBRARY})
endif(ATC3DG_FOUND)
