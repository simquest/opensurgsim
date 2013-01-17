# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest LLC.


# - Try to find Eigen 3.x
#
# Once done this will define
#  EIGEN3_FOUND - system has eigen lib with correct version
#  EIGEN3_INCLUDE_DIR - the eigen include directory
#
# Rewritten from scratch to keep things simple and flexible.

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

if(EIGEN3_INCLUDE_DIR)
  set(EIGEN3_VERSION_OK TRUE)
else(EIGEN3_INCLUDE_DIR)
  set(EIGEN3_VERSION_OK FALSE)
endif(EIGEN3_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3
	DEFAULT_MSG EIGEN3_INCLUDE_DIR EIGEN3_VERSION_OK)
mark_as_advanced(EIGEN3_INCLUDE_DIR)
