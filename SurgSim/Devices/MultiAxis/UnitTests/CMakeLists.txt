# This file is a part of the OpenSurgSim project.
# Copyright 2013, SimQuest Solutions Inc.
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

include_directories(
)

set(UNIT_TEST_SOURCES
	MultiAxisDeviceTest.cpp
	RawMultiAxisDeviceTest.cpp
	RawMultiAxisScaffoldTest.cpp
)

set(UNIT_TEST_HEADERS
)

set(LIBS 
	MultiAxisDevice
)

if(WDK_FOUND)
	list(APPEND LIBS ${WDK_LIBRARIES})
endif()

# The unit tests will be built whenever the library is built, but
# running them at build time requires hardware and is disabled by
# default.
option(SURGSIM_TESTS_RUN_DEVICE_MULTIAXIS
	"Run the MultiAxis device tests as part of unit tests.  (Requires a multi-axis controller device, e.g. Logitech's 3DConnexion SpaceNavigator.)"
	OFF)
surgsim_add_unit_tests(MultiAxisDeviceTest
	RUN ${SURGSIM_TESTS_RUN_DEVICE_MULTIAXIS})

set_target_properties(MultiAxisDeviceTest PROPERTIES FOLDER "Devices")