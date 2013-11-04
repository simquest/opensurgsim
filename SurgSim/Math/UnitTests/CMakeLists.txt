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
	${CMAKE_CURRENT_SOURCE_DIR}
)

set(UNIT_TEST_SOURCES
	GeometryTests.cpp
	MakeRigidTransformTests.cpp
	MeshShapeTests.cpp
	MlcpGaussSeidelSolverTests.cpp
	ShapeTests.cpp
	ValidTests.cpp
)

set(UNIT_TEST_HEADERS
)

set(UNIT_TEST_EIGEN_TYPE_SOURCES
	MatrixTests.cpp
	QuaternionTests.cpp
	RigidTransformTests.cpp
	VectorTests.cpp
)

set(UNIT_TEST_EIGEN_TYPE_HEADERS
)

option(SURGSIM_TESTS_BUILD_EIGEN
	"Include basic tests for Eigen math class typedefs, which build very slowly."
	OFF)
if(SURGSIM_TESTS_BUILD_EIGEN)
	set(UNIT_TEST_SOURCES ${UNIT_TEST_SOURCES} ${UNIT_TEST_EIGEN_TYPE_SOURCES})
	set(UNIT_TEST_HEADERS ${UNIT_TEST_HEADERS} ${UNIT_TEST_EIGEN_TYPE_HEADERS})

	# Fix a problem with 64-bit builds running out of sections.
	if(MSVC AND CMAKE_CL_64)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
	endif(MSVC AND CMAKE_CL_64)
endif(SURGSIM_TESTS_BUILD_EIGEN)

set(LIBS SurgSimMath SurgSimFramework MlcpTestIO ${Boost_LIBRARIES} gtest)

file(COPY ${CMAKE_SOURCE_DIR}/SurgSim/Math/UnitTests/MlcpTestData DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

surgsim_add_unit_tests(SurgSimMathTest)
