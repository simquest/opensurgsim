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
	BuildMlcpTests.cpp
	ComputationTests.cpp
	ConstraintComponentTests.cpp
	ConstraintTests.cpp
	ContactConstraintDataTests.cpp
	ContactConstraintGenerationTests.cpp
	DcdCollisionTests.cpp
	DeformableRepresentationStateTest.cpp
	DeformableRepresentationTest.cpp
	EigenGtestAsserts.cpp
	Fem1DRepresentationTests.cpp
	Fem2DRepresentationTests.cpp
	Fem3DRepresentationBilateral3DTests.cpp
	Fem3DRepresentationContactTests.cpp
	Fem3DRepresentationReaderTests.cpp
	Fem3DRepresentationTests.cpp
	FemElement1DBeamTests.cpp
	FemElement2DTriangleTests.cpp
	FemElement3DCubeTests.cpp
	FemElement3DTetrahedronTests.cpp
	FemElementTests.cpp
	FemRepresentationLocalizationTest.cpp
	FemRepresentationParametersTest.cpp
	FemRepresentationTests.cpp
	FixedRepresentationBilateral3DTests.cpp
	FixedRepresentationContactTests.cpp
	FixedRepresentationLocalizationTest.cpp
	FixedRepresentationTest.cpp
	FreeMotionTests.cpp
	LinearSpringTest.cpp
	MassSpringRepresentationContactTest.cpp
	MassSpringRepresentationLocalizationTest.cpp
	MassSpringRepresentationTests.cpp
	MassTest.cpp
	PhysicsManagerStateTests.cpp
	PhysicsManagerTests.cpp
	PostUpdateTests.cpp
	PreUpdateTests.cpp
	PushResultsTests.cpp
	RepresentationTest.cpp
	RigidRepresentationBaseStateTest.cpp
	RigidRepresentationBilateral3DTests.cpp
	RigidRepresentationContactTests.cpp
	RigidRepresentationLocalizationTest.cpp
	RigidRepresentationParametersTest.cpp
	RigidRepresentationStateTest.cpp
	RigidRepresentationTest.cpp
	SolveMlcpTests.cpp
	VirtualToolCouplerTest.cpp
)

set(UNIT_TEST_HEADERS
	CommonTests.h
	EigenGtestAsserts.h
	MockObjects.h
)

set(LIBS
	SurgSimBlocks
	SurgSimPhysics
	SurgSimMath
	SurgSimInput
	MlcpTestIO
	IdentityPoseDevice
)

file(COPY ${CMAKE_SOURCE_DIR}/SurgSim/Math/UnitTests/MlcpTestData DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

surgsim_add_unit_tests(SurgSimPhysicsTest)

set_target_properties(SurgSimPhysicsTest PROPERTIES FOLDER "Physics")
