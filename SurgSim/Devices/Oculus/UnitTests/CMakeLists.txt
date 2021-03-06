# This file is a part of the OpenSurgSim project.
# Copyright 2015, SimQuest Solutions Inc.
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
	OculusDeviceTest.cpp
	OculusDisplaySettingsTests.cpp
	OculusViewTests.cpp
)

set(UNIT_TEST_HEADERS
)

set(LIBS
	SurgSimDataStructures
	SurgSimInput
	SurgSimTesting
	OculusDevice
)

surgsim_add_unit_tests(OculusDeviceTest)

target_link_libraries(OculusDeviceTest ${LIBS})

# Put OculusDeviceTest into folder "Devices"
set_target_properties(OculusDeviceTest PROPERTIES FOLDER "Devices")
