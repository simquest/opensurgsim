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
	CommonDeviceTests.cpp
	InputComponentTest.cpp
	InputManagerTest.cpp
	TestDevice.cpp
)

set(UNIT_TEST_HEADERS
	TestDevice.h
)

include_directories(
)

set(LIBS
	SurgSimInput
	gmock
	gtest
)

surgsim_add_unit_tests(SurgSimInputTest)

# Put SurgSimInputTest into folder "Input"
set_target_properties(SurgSimInputTest PROPERTIES FOLDER "Input")
