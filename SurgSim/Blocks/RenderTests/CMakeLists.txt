# This file is a part of the OpenSurgSim project.
# Copyright 2012-2016, SimQuest Solutions Inc.
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
	KnotIdentificationRenderTests.cpp
)

set(UNIT_TEST_HEADERS
)

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

set(LIBS
	SurgSimBlocks
	SurgSimFramework
	SurgSimGraphics
)

file(COPY Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

surgsim_add_unit_tests(SurgSimBlocksRenderTest)

set_target_properties(SurgSimBlocksRenderTest PROPERTIES FOLDER "Blocks")
