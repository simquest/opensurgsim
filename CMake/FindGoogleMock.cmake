# This file is a part of the OpenSurgSim project.
# Copyright 2014-2016, SimQuest Solutions Inc.
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


# - Try to find the Google Mock Source directory
#
# Once done this will define
#  GOOGLEMOCK_FOUND
#  GOOGLEMOCK_DIR
#

if(NOT GOOGLEMOCK_DIR)
	find_path(GOOGLEMOCK_DIR
		NAMES src/gmock.cc
		PATHS "$ENV{GOOGLEMOCK_DIR}" "/usr/src/googletest/googlemock" "/usr/src/gmock/"
	)
endif(NOT GOOGLEMOCK_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GoogleMock DEFAULT_MSG GOOGLEMOCK_DIR)

mark_as_advanced(GOOGLEMOCK_DIR)
