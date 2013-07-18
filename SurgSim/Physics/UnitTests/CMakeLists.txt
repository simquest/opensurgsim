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
	CollisionPairTests.cpp
	ConstraintTests.cpp
	ContactCalculationTests.cpp
	ContactConstraintDataTests.cpp
	ContactConstraintGenerationTests.cpp
	ComputationTests.cpp
	DcdCollisionTests.cpp
	FemRepresentationParametersTest.cpp
	FixedRepresentationContactTests.cpp
	FixedRepresentationLocalizationTest.cpp
	FixedRepresentationTest.cpp
	FreeMotionTests.cpp
	MeshShapeTest.cpp
	PhysicsManagerTests.cpp
	PostUpdateTests.cpp
	PreUpdateTests.cpp
	RepresentationTest.cpp
	RepresentationUtilities.cpp
	RigidRepresentationTest.cpp
	RigidRepresentationBaseStateTest.cpp
	RigidRepresentationContactTests.cpp
	RigidRepresentationLocalizationTest.cpp
	RigidRepresentationParametersTest.cpp
	RigidRepresentationStateTest.cpp
	RigidRepresentationTest.cpp
	RigidShapeTest.cpp
	SolveMlcpTests.cpp
	VtcRigidParametersTest.cpp
	VtcRigidRepresentationTest.cpp
)

set(UNIT_TEST_HEADERS
	MockObjects.h
	RepresentationUtilities.h
)

include_directories(
)

set(LIBS SurgSimFramework SurgSimDataStructures SurgSimPhysics MlcpTestIO gtest ${Boost_LIBRARIES})

file(COPY ${CMAKE_SOURCE_DIR}/SurgSim/Testing/MlcpIO/MlcpTestData DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

surgsim_add_unit_tests(SurgSimPhysicsTest)
