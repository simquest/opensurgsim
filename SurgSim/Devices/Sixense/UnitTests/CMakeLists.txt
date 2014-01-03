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
	SixenseDeviceTest.cpp
	SixenseScaffoldTest.cpp
)

set(UNIT_TEST_HEADERS
)

set(LIBS 
	SixenseDevice
	SurgSimInput
	SurgSimFramework
	SurgSimDataStructures
	${Boost_LIBRARIES}
	${SIXENSE_SDK_LIBRARIES}
)

set(UNIT_TEST_SHARED_RELEASE_LIBS
	${SIXENSE_SDK_sixense_SHARED_RELEASE}
	${SIXENSE_SDK_sixense_utils_SHARED_RELEASE}
)
set(UNIT_TEST_SHARED_DEBUG_LIBS
	${SIXENSE_SDK_sixense_SHARED_DEBUG}
	${SIXENSE_SDK_sixense_utils_SHARED_DEBUG}
	${SIXENSE_SDK_DeviceDLL_SHARED_DEBUG}  # crazy debug-only dependency
)

# The unit tests will be built whenever the library is built, but
# running them at build time requires hardware and is disabled by
# default.
option(SURGSIM_TESTS_RUN_DEVICE_SIXENSE
	"Run the Sixense tests as part of unit tests.  (Requires Razer Hydra hardware and drivers.)"
	OFF)
surgsim_add_unit_tests(SixenseDeviceTest
	RUN ${SURGSIM_TESTS_RUN_DEVICE_SIXENSE})

set_target_properties(SixenseDeviceTest PROPERTIES FOLDER "Devices")