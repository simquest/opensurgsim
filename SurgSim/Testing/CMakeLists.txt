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
	${EIGEN3_INCLUDE_DIR}
	${gmock_SOURCE_DIR}/include
	${gtest_SOURCE_DIR}/include
)

set(SURGSIM_TESTING_SOURCES
	DevicesUtilities.cpp
	MathUtilities.cpp
	TestCube.cpp
	TestingMain.cpp
)

set(SURGSIM_TESTING_HEADERS
	DevicesUtilities.h
	MathUtilities.h
	MockPhysicsManager.h
	TestCube.h
)

surgsim_add_library(
	SurgSimTesting
	"${SURGSIM_TESTING_SOURCES}"
	"${SURGSIM_TESTING_HEADERS}"
	"SurgSim/Testing"
)

add_subdirectory(MlcpIO)

if(GLUT_FOUND)
	add_subdirectory(VisualTestCommon)
endif(GLUT_FOUND)

file(COPY Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set_target_properties(SurgSimTesting PROPERTIES FOLDER "Testing")
