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
	${gtest_SOURCE_DIR}
)

set(UNIT_TEST_SOURCES
	TrackIRDeviceTest.cpp
	TrackIRScaffoldTest.cpp
)

set(UNIT_TEST_HEADERS
)

set(LIBS
	TrackIRDevice 
	SurgSimInput 
	SurgSimFramework 
	SurgSimDataStructures
	gmock
	gtest
	${Boost_LIBRARIES} 
	${OPTITRACK_LIBRARY}
)

# The unit tests will be built whenever the library is built, but
# running them at build time requires hardware and is disabled by
# default.
option(SURGSIM_TESTS_RUN_DEVICE_TRACKIR
	"Run the TrackIR tests as part of unit tests.  (Requires hardware and OptiTrack!)"
	OFF)
surgsim_add_unit_tests(TrackIRDeviceTest
	RUN ${SURGSIM_TESTS_RUN_DEVICE_TRACKIR})

# Copy the device DLLs into the build directory
surgsim_copy_to_target_directory_for_release(TrackIRDeviceTest
	${OPTITRACK_SHARED_RELEASE}
)
surgsim_copy_to_target_directory_for_debug(TrackIRDeviceTest
	${OPTITRACK_SHARED_DEBUG}
)

# Put TrackIRDeviceTest into folder "Devices"
set_target_properties(TrackIRDeviceTest PROPERTIES FOLDER "Devices")