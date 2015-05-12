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
	ConstraintImplementationFactoryTests.cpp
	ConstraintImplementationTests.cpp
	ConstraintTests.cpp
	ContactConstraintDataTests.cpp
	ContactConstraintGenerationTests.cpp
	DcdCollisionTests.cpp
	DeformableCollisionRepresentationTest.cpp
	DeformableRepresentationTest.cpp
	EigenGtestAsserts.cpp
	Fem1DElementBeamTests.cpp
	Fem1DMechanicalValidationTests.cpp
	Fem1DPlyReaderDelegateTests.cpp
	Fem1DRepresentationLocalizationTest.cpp
	Fem1DRepresentationTests.cpp
	Fem2DElementTriangleTests.cpp
	Fem2DMechanicalValidationTests.cpp
	Fem2DPlyReaderDelegateTests.cpp
	Fem2DRepresentationLocalizationTest.cpp
	Fem2DRepresentationTests.cpp
	Fem3DElementCorotationalTetrahedronTests.cpp
	Fem3DElementCubeTests.cpp
	Fem3DElementTetrahedronTests.cpp
	Fem3DPlyReaderDelegateTests.cpp
	Fem3DRepresentationBilateral3DTests.cpp
	Fem3DRepresentationContactTests.cpp
	Fem3DRepresentationLocalizationTest.cpp
	Fem3DRepresentationTests.cpp
	FemElementTests.cpp
	FemRepresentationTests.cpp
	FixedRepresentationBilateral3DTests.cpp
	FixedRepresentationContactTests.cpp
	FixedRepresentationTest.cpp
	FreeMotionTests.cpp
	LinearSpringTest.cpp
	MassSpringMechanicalValidationTests.cpp
	MassSpringRepresentationContactTest.cpp
	MassSpringRepresentationLocalizationTest.cpp
	MassSpringRepresentationTests.cpp
	MassTest.cpp
	MockObjects.cpp
	PhysicsManagerStateTests.cpp
	PhysicsManagerTests.cpp
	PostUpdateTests.cpp
	PreUpdateTests.cpp
	PushResultsTests.cpp
	RepresentationTest.cpp
	RigidCollisionRepresentationTest.cpp
	RigidRepresentationBaseLocalizationTest.cpp
	RigidRepresentationBilateral3DTests.cpp
	RigidRepresentationContactTests.cpp
	RigidRepresentationStateTest.cpp
	RigidRepresentationTest.cpp
	SolveMlcpTests.cpp
	UpdateCollisionRepresentationsTest.cpp
	VirtualToolCouplerTest.cpp
)

set(UNIT_TEST_HEADERS
	CommonTests.h
	EigenGtestAsserts.h
	MockObjects.h
)

set(LIBS
	SurgSimBlocks
	SurgSimGraphics
	SurgSimPhysics
	SurgSimMath
	SurgSimInput
	MlcpTestIO
	IdentityPoseDevice
)


file(COPY ${SURGSIM_SOURCE_DIR}/SurgSim/Math/UnitTests/MlcpTestData DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

surgsim_add_unit_tests(SurgSimPhysicsTest)

set_target_properties(SurgSimPhysicsTest PROPERTIES FOLDER "Physics")
