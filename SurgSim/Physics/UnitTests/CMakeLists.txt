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
	${gmock_SOURCE_DIR}/include
)

set(UNIT_TEST_SOURCES
	BuildMlcpTests.cpp
	CcdCollisionLoopTest.cpp
	ComputationGroupTest.cpp
	ComputationTests.cpp
	ConstraintComponentTests.cpp
	ConstraintImplementationFactoryTests.cpp
	ConstraintImplementationTests.cpp
	ConstraintTests.cpp
	ContactConstraintDataTests.cpp
	ContactConstraintGenerationTests.cpp
	ContactFilteringTest.cpp
	DcdCollisionTests.cpp
	DeformableCollisionRepresentationTest.cpp
	DeformableRepresentationTest.cpp
	EigenGtestAsserts.cpp
	Fem1DConstraintFixedPointTests.cpp
	Fem1DConstraintFixedRotationVectorTests.cpp
	Fem1DConstraintFrictionalSlidingTests.cpp
	Fem1DConstraintFrictionlessContactTests.cpp
	Fem1DConstraintFrictionlessSlidingTests.cpp
	Fem1DElementBeamTests.cpp
	Fem1DLocalizationTest.cpp
	Fem1DMechanicalValidationTests.cpp
	Fem1DPlyReaderDelegateTests.cpp
	Fem1DRepresentationTests.cpp
	Fem2DConstraintFixedPointTests.cpp
	Fem2DConstraintFrictionalSlidingTests.cpp
	Fem2DConstraintFrictionlessContactTests.cpp
	Fem2DConstraintFrictionlessSlidingTests.cpp
	Fem2DElementTriangleTests.cpp
	Fem2DLocalizationTest.cpp
	Fem2DMechanicalValidationTests.cpp
	Fem2DPlyReaderDelegateTests.cpp
	Fem2DRepresentationTests.cpp
	Fem3DConstraintFixedPointTests.cpp
	Fem3DConstraintFrictionalSlidingTests.cpp
	Fem3DConstraintFrictionlessContactTests.cpp
	Fem3DConstraintFrictionlessSlidingTests.cpp
	Fem3DCorotationalTetrahedronRepresentationTests.cpp
	Fem3DElementCorotationalTetrahedronTests.cpp
	Fem3DElementCubeTests.cpp
	Fem3DElementTetrahedronTests.cpp
	Fem3DLocalizationTest.cpp
	Fem3DPlyReaderDelegateTests.cpp
	Fem3DRepresentationTests.cpp
	FemElementTests.cpp
	FemLocalizationTest.cpp
	FemRepresentationTests.cpp
	FixedConstraintFixedPointTests.cpp
	FixedConstraintFixedRotationVectorTests.cpp
	FixedConstraintFrictionlessContactTests.cpp
	FixedRepresentationTest.cpp
	FreeMotionTests.cpp
	LinearSpringTest.cpp
	MassSpringConstraintFixedPointTest.cpp
	MassSpringConstraintFrictionlessContactTest.cpp
	MassSpringLocalizationTest.cpp
	MassSpringMechanicalValidationTests.cpp
	MassSpringRepresentationTests.cpp
	MassTest.cpp
	MockObjects.cpp
	ParticleCollisionResponseTests.cpp
	PhysicsManagerStateTests.cpp
	PhysicsManagerTests.cpp
	PostUpdateTests.cpp
	PrepareCollisionPairsTests.cpp
	PreUpdateTests.cpp
	PushResultsTests.cpp
	RepresentationTest.cpp
	RigidCollisionRepresentationTest.cpp
	RigidConstraintFixedPointTests.cpp
	RigidConstraintFixedRotationVectorTests.cpp
	RigidConstraintFrictionlessContactTests.cpp
	RigidLocalizationTest.cpp
	RigidRepresentationTest.cpp
	RigidStateTest.cpp
	RotationVectorConstraintDataTests.cpp
	SlidingConstraintDataTests.cpp
	SolveMlcpTests.cpp
	UpdateCollisionRepresentationsTest.cpp
	VirtualToolCouplerTest.cpp
)

set(UNIT_TEST_HEADERS
	CommonTests.h
	DeformableTestsUtility.h
	DeformableTestsUtility-inl.h
	EigenGtestAsserts.h
	MockObjects.h
)

set(LIBS
	SurgSimBlocks
	SurgSimGraphics
	SurgSimInput
	SurgSimMath
	SurgSimParticles
	SurgSimPhysics
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
