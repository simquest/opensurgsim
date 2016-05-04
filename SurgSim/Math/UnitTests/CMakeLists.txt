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
	AabbTests.cpp
	CardinalSplinesTests.cpp
	CompoundShapeTests.cpp
	CubicSolverTests.cpp
	GeometryTests.cpp
	IntervalArithmeticTests.cpp
	KalmanFilterTests.cpp
	LinearMotionArithmeticTests.cpp
	MakeRigidTransformTests.cpp
	MeshShapeTests.cpp
	MinMaxTests.cpp
	MlcpGaussSeidelSolverTests.cpp
	OdeEquationTests.cpp
	OdeSolverEulerExplicitModifiedTests.cpp
	OdeSolverEulerExplicitTests.cpp
	OdeSolverEulerImplicitTests.cpp
	OdeSolverRungeKutta4Tests.cpp
	OdeSolverStaticTests.cpp
	OdeSolverTests.cpp
	OdeStateTests.cpp
	ParticlesShapeTests.cpp
	PolynomialRootTests.cpp
	PolynomialTests.cpp
	PolynomialValuesTests.cpp
	ScalarTests.cpp
	SegmentMeshShapeTests.cpp
	ShapeTests.cpp
	SurfaceMeshShapeTests.cpp
	TriangleCapsuleContactCalculationTests.cpp
	TriangleTriangleContactCalculationTests.cpp
	TriangleTriangleIntersectionTests.cpp
	ValidTests.cpp
)

set(UNIT_TEST_HEADERS
	MockCapsule.h
	MockObject.h
	MockTriangle.h
	TriangleTriangleTestParameters.h
)

set(UNIT_TEST_EIGEN_TYPE_SOURCES
	AngleAxisTests.cpp
	LinearSolveAndInverseTests.cpp
	LinearSparseSolveAndInverseTests.cpp
	MatrixTests.cpp
	QuaternionTests.cpp
	RigidTransformTests.cpp
	SparseMatrixTests.cpp
	VectorTests.cpp
)

set(UNIT_TEST_EIGEN_TYPE_HEADERS
)

option(BUILD_TESTING_EIGEN
	"Include basic tests for Eigen math class typedefs, which build very slowly."
	OFF)
if(BUILD_TESTING_EIGEN)
	set(UNIT_TEST_SOURCES ${UNIT_TEST_SOURCES} ${UNIT_TEST_EIGEN_TYPE_SOURCES})
	set(UNIT_TEST_HEADERS ${UNIT_TEST_HEADERS} ${UNIT_TEST_EIGEN_TYPE_HEADERS})
	# Fix a problem with 64-bit builds running out of sections.
	if(MSVC AND CMAKE_CL_64)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
	endif(MSVC AND CMAKE_CL_64)
endif(BUILD_TESTING_EIGEN)

set(LIBS 
	SurgSimMath
	MlcpTestIO
)

file(COPY ${SURGSIM_SOURCE_DIR}/SurgSim/Math/UnitTests/MlcpTestData DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Configure the path for the data files
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

surgsim_add_unit_tests(SurgSimMathTest)

set_target_properties(SurgSimMathTest PROPERTIES FOLDER "Math")
