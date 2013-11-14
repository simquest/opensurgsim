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
	BoxDoubleSidedPlaneContactCalculationTests.cpp
	BoxPlaneContactCalculationTests.cpp
	BoxSphereContactCalculationTests.cpp
	CapsuleSphereContactCalculationTests.cpp
	CollisionPairTests.cpp
	ContactCalculationTests.cpp
	ContactCalculationTestsCommon.cpp
	DefaultContactCalculationTests.cpp
	MockCollisionRepresentation.cpp
	RepresentationUtilities.cpp
	SphereDoubleSidedPlaneContactCalculationTests.cpp
	SpherePlaneContactCalculationTests.cpp
	SphereSphereContactCalculationTests.cpp
	TriangleMeshPlaneContactCalculationTests.cpp
)

set(UNIT_TEST_HEADERS
	ContactCalculationTestsCommon.h
	MockCollisionRepresentation.h
	RepresentationUtilities.h
)

include_directories(
)

set(LIBS
	SurgSimCollision
	gtest
)

surgsim_add_unit_tests(SurgSimCollisionTest)

# Put SurgSimCollisionTest into folder "Collision"
set_target_properties(SurgSimCollisionTest PROPERTIES FOLDER "Collision") 