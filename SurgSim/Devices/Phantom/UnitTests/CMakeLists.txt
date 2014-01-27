# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.
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
	${gtest_SOURCE_DIR}/include
)

set(UNIT_TEST_SOURCES
	PhantomDeviceTest.cpp
	PhantomScaffoldTest.cpp
)

set(LIBS
	PhantomDevice
	SurgSimInput
	SurgSimFramework
	SurgSimDataStructures
	${Boost_LIBRARIES}
	${OPENHAPTICS_LIBRARIES}
)

# The unit tests will be built whenever the library is built, but
# running them at build time requires hardware and is disabled by
# default.
option(SURGSIM_TESTS_RUN_DEVICE_PHANTOM
	"Run the Phantom tests as part of unit tests.  (Requires hardware, OpenHaptics 3.0 and device license!)"
	OFF)
surgsim_add_unit_tests(PhantomDeviceTest
	RUN ${SURGSIM_TESTS_RUN_DEVICE_PHANTOM})

set_target_properties(PhantomDeviceTest PROPERTIES FOLDER "Devices")
