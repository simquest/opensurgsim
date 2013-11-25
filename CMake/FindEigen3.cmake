# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.


# - Try to find Eigen 3.x
#
# Once done this will define
#  EIGEN3_FOUND - system has eigen lib with correct version
#  EIGEN3_INCLUDE_DIR - the eigen include directory
#
# Rewritten from scratch to keep things simple and flexible.

# Attempt to define EIGEN3_INCLUDE_DIR if undefined
if(NOT EIGEN3_INCLUDE_DIR)
	if(DEFINED ENV{EIGEN_DIR})
		find_path(EIGEN3_INCLUDE_DIR
			NAMES signature_of_eigen3_matrix_library
			PATHS "$ENV{EIGEN_DIR}"
		)
	else(DEFINED ENV{EIGEN_DIR})
		find_path(EIGEN3_INCLUDE_DIR
			NAMES signature_of_eigen3_matrix_library
			PATHS ${KDE4_INCLUDE_DIR} /usr/include
			PATH_SUFFIXES eigen3 eigen
		)
	endif(DEFINED ENV{EIGEN_DIR})
endif(NOT EIGEN3_INCLUDE_DIR)

set(EIGEN3_MACROS_H "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h")

if(EXISTS "${EIGEN3_MACROS_H}")
	set(EIGEN3_MACROS_H_FOUND TRUE)
else(EXISTS "${EIGEN3_MACROS_H}")
	set(EIGEN3_MACROS_H_FOUND FALSE)
endif(EXISTS "${EIGEN3_MACROS_H}")

if(EIGEN3_MACROS_H_FOUND)
	file(READ "${EIGEN3_MACROS_H}" _eigen3_version_header)

	string(REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _eigen3_world_version_match "${_eigen3_version_header}")
	set(EIGEN3_WORLD_VERSION "${CMAKE_MATCH_1}")

	string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen3_major_version_match "${_eigen3_version_header}")
	set(EIGEN3_MAJOR_VERSION "${CMAKE_MATCH_1}")

	string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen3_minor_version_match "${_eigen3_version_header}")
	set(EIGEN3_MINOR_VERSION "${CMAKE_MATCH_1}")

	set(EIGEN3_VERSION ${EIGEN3_WORLD_VERSION}.${EIGEN3_MAJOR_VERSION}.${EIGEN3_MINOR_VERSION})
endif(EIGEN3_MACROS_H_FOUND)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3
	REQUIRED_VARS EIGEN3_MACROS_H EIGEN3_MACROS_H_FOUND
	VERSION_VAR EIGEN3_VERSION
)

mark_as_advanced(EIGEN3_INCLUDE_DIR)
