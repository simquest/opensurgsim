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
	${gtest_SOURCE_DIR}/include
)

set(UNIT_TEST_SOURCES
	SixenseDeviceTest.cpp
	SixenseScaffoldTest.cpp
)

set(LIBS 
	SixenseDevice
	SurgSimInput
	SurgSimFramework
	SurgSimDataStructures
	${Boost_LIBRARIES}
	${SIXENSESDK_LIBRARIES}
)

set(UNIT_TEST_SHARED_RELEASE_LIBS
	${SIXENSESDK_sixense_SHARED_RELEASE}
	${SIXENSESDK_sixense_utils_SHARED_RELEASE}
)
set(UNIT_TEST_SHARED_DEBUG_LIBS
	${SIXENSESDK_sixense_SHARED_DEBUG}
	${SIXENSESDK_sixense_utils_SHARED_DEBUG}
	${SIXENSESDK_DeviceDLL_SHARED_DEBUG}  # crazy debug-only dependency
)

surgsim_add_unit_tests(SixenseDeviceTest)

set_target_properties(SixenseDeviceTest PROPERTIES FOLDER "Devices")
