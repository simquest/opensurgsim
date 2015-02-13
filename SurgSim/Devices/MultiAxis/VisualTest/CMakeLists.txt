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


set(EXAMPLE_SOURCES
	main.cpp
)

set(EXAMPLE_HEADERS
)

set(RAW_EXAMPLE_SOURCES
	raw_test_main.cpp
)

set(RAW_EXAMPLE_HEADERS
)

set(LIBS
	IdentityPoseDevice
	MultiAxisDevice
	SurgSimInput
	VisualTestCommon
)

surgsim_add_executable(MultiAxisVisualTest
	"${EXAMPLE_SOURCES}" "${EXAMPLE_HEADERS}")
target_link_libraries(MultiAxisVisualTest ${LIBS})

surgsim_add_executable(RawMultiAxisVisualTest
	"${RAW_EXAMPLE_SOURCES}" "${RAW_EXAMPLE_HEADERS}")
target_link_libraries(RawMultiAxisVisualTest ${LIBS})

# Put all projects into folder "Devices"
set_target_properties(RawMultiAxisVisualTest PROPERTIES FOLDER "Devices")
set_target_properties(MultiAxisVisualTest PROPERTIES FOLDER "Devices")
