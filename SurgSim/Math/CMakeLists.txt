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



set(SURGSIM_MATH_SOURCES
	BoxShape.cpp
	CapsuleShape.cpp
	CylinderShape.cpp
	DoubleSidedPlaneShape.cpp
	GaussLegendreQuadrature.cpp
	LinearSolveAndInverse.cpp
	LinearSparseSolveAndInverse.cpp
	MathConvert.cpp
	MeshShape.cpp
	MlcpGaussSeidelSolver.cpp
	MlcpProblem.cpp
	OctreeShape.cpp
	OdeEquation.cpp
	OdeSolver.cpp
	OdeSolverEulerExplicit.cpp
	OdeSolverEulerExplicitModified.cpp
	OdeSolverEulerImplicit.cpp
	OdeSolverLinearEulerExplicit.cpp
	OdeSolverLinearEulerExplicitModified.cpp
	OdeSolverLinearEulerImplicit.cpp
	OdeSolverLinearRungeKutta4.cpp
	OdeSolverLinearStatic.cpp
	OdeSolverRungeKutta4.cpp
	OdeSolverStatic.cpp
	OdeState.cpp
	ParticlesShape.cpp
	PlaneShape.cpp
	SegmentMeshShape.cpp
	Shape.cpp
	SphereShape.cpp
	SurfaceMeshShape.cpp
)

set(SURGSIM_MATH_HEADERS
	Aabb.h
	BoxShape.h
	CapsuleShape.h
	CylinderShape.h
	DoubleSidedPlaneShape.h
	GaussLegendreQuadrature.h
	Geometry.h
	IntervalArithmetic.h
	IntervalArithmetic-inl.h
	KalmanFilter.h
	KalmanFilter-inl.h
	LinearMotionArithmetic.h
	LinearMotionArithmetic-inl.h
	LinearSolveAndInverse.h
	LinearSolveAndInverse-inl.h
	LinearSparseSolveAndInverse.h
	MathConvert.h
	MathConvert-inl.h
	Matrix.h
	MeshShape.h
	MeshShape-inl.h
	MinMax.h
	MinMax-inl.h
	MlcpConstraintType.h
	MlcpConstraintTypeName.h
	MlcpGaussSeidelSolver.h
	MlcpProblem.h
	MlcpSolution.h
	MlcpSolver.h
	OctreeShape.h
	OctreeShape-inl.h
	OdeEquation.h
	OdeSolver.h
	OdeSolverEulerExplicit.h
	OdeSolverEulerExplicitModified.h
	OdeSolverEulerImplicit.h
	OdeSolverLinearEulerExplicit.h
	OdeSolverLinearEulerExplicitModified.h
	OdeSolverLinearEulerImplicit.h
	OdeSolverLinearRungeKutta4.h
	OdeSolverLinearStatic.h
	OdeSolverRungeKutta4.h
	OdeSolverStatic.h
	OdeState.h
	ParticlesShape.h
	ParticlesShape-inl.h
	PlaneShape.h
	Polynomial.h
	Polynomial-inl.h
	PolynomialRoots.h
	PolynomialRoots-inl.h
	PolynomialValues.h
	PolynomialValues-inl.h
	Quaternion.h
	RigidTransform.h
	SegmentMeshShape.h
	SegmentMeshShape-inl.h
	Shape.h
	Shapes.h
	SparseMatrix.h
	SphereShape.h
	SurfaceMeshShape.h
	SurfaceMeshShape-inl.h
	TriangleCapsuleContactCalculation-inl.h
	TriangleTriangleContactCalculation-inl.h
	TriangleTriangleIntersection-inl.h
	Valid.h
	Valid-inl.h
	Vector.h
)
surgsim_create_library_header(Math.h "${SURGSIM_MATH_HEADERS}")

surgsim_add_library(
	SurgSimMath
	"${SURGSIM_MATH_SOURCES}"
	"${SURGSIM_MATH_HEADERS}"
)

set(LIBS 
	SurgSimDataStructures
	SurgSimFramework 
	${YAML_CPP_LIBRARIES}
)

target_link_libraries(SurgSimMath ${LIBS})

if(BUILD_TESTING)
	add_subdirectory(UnitTests)
endif()

# Put SurgSimMath into folder "Math"
set_target_properties(SurgSimMath PROPERTIES FOLDER "Math")
