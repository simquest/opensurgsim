# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.


# - Try to find the Google Mock Source directory
#
# Once done this will define
#  GOOGLEMOCK_FOUND
#  GOOGLEMOCK_DIR
#

if(NOT GOOGLEMOCK_DIR)
	find_path(GOOGLEMOCK_DIR
		NAMES src/gmock.cc
		PATHS "$ENV{GOOGLMOCK_DIR}" "/usr/src/gmock/"
	)
endif(NOT GOOGLEMOCK_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GoogleMock DEFAULT_MSG GOOGLEMOCK_DIR)

mark_as_advanced(GOOGLEMOCK_DIR)
