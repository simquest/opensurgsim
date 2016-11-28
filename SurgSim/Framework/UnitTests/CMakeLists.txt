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

include_directories (
	${gtest_SOURCE_DIR}/include
	${gmock_SOURCE_DIR}/include
)

set(UNIT_TEST_SOURCES
	AccessibleTests.cpp
	AccessibleTypeTests.cpp
	ApplicationDataTest.cpp
	AssertTest.cpp
	AssetTests.cpp
	BarrierTest.cpp
	BasicSceneElementTests.cpp
	BasicThreadTests.cpp
	BehaviorManagerTest.cpp
	ComponentManagerTests.cpp
	ComponentTest.cpp
	LockedContainerTest.cpp
	LoggerManagerTest.cpp
	LoggerTest.cpp
	MessengerTest.cpp
	MockObjects.cpp
	ObjectFactoryTests.cpp
	ReuseFactoryTest.cpp
	RuntimeTest.cpp
	SamplingMetricBaseTest.cpp
	SceneElementTest.cpp
	SceneTest.cpp
	SharedInstanceTest.cpp
	ThreadPoolTest.cpp
	TimerTest.cpp
	TransferPropertiesBehaviorTests.cpp
)

set(UNIT_TEST_HEADERS
	MockObjects.h
)

file(COPY Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Configure the path for the data files
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

set(LIBS
	SurgSimFramework
	SurgSimDataStructures
)

surgsim_add_unit_tests(SurgSimFrameworkTest)

set_target_properties(SurgSimFrameworkTest PROPERTIES FOLDER "Framework")
