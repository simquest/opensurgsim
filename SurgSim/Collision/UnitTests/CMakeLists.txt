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
	BoxCapsuleContactCalculationTests.cpp
	BoxDoubleSidedPlaneContactCalculationTests.cpp
	BoxPlaneContactCalculationTests.cpp
	BoxSphereContactCalculationTests.cpp
	CapsuleSphereContactCalculationTests.cpp
	CollisionPairTests.cpp
	CompoundShapeContactCalculationTests.cpp
	ContactCalculationTests.cpp
	ContactCalculationTestsCommon.cpp
	DefaultContactCalculationTests.cpp
	OctreeContactCalculationTests.cpp
	RepresentationTest.cpp
	RepresentationUtilities.cpp
	SegmentMeshTriangleMeshContactCalculationTests.cpp
	SegmentSegmentCcdIntervalCheckTests.cpp
	SegmentSegmentCcdMovingContactTests.cpp
	SegmentSegmentCcdStaticContactTests.cpp
	SegmentSelfContactTests.cpp
	ShapeCollisionRepresentationTest.cpp
	SphereDoubleSidedPlaneContactCalculationTests.cpp
	SpherePlaneContactCalculationTests.cpp
	SphereSphereContactCalculationTests.cpp
	TriangleMeshParticlesContactCalculationTests.cpp
	TriangleMeshPlaneContactCalculationTests.cpp
	TriangleMeshSurfaceMeshContactCalculationTests.cpp
	TriangleMeshTriangleMeshContactCalculationTests.cpp
)

set(UNIT_TEST_HEADERS
	ContactCalculationTestsCommon.h
	RepresentationUtilities.h
)

set(LIBS
	SurgSimCollision
	SurgSimDataStructures
	SurgSimMath
	SurgSimPhysics
	SurgSimInput
)

# Configure the path for the data files
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

surgsim_add_unit_tests(SurgSimCollisionTest)

set_target_properties(SurgSimCollisionTest PROPERTIES FOLDER "Collision") 
