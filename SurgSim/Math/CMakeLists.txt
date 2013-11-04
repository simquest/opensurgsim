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
	${EIGEN3_INCLUDE_DIR}
)

set(SURGSIM_MATH_SOURCES
	BoxShape.cpp
	CapsuleShape.cpp
	CylinderShape.cpp
	DoubleSidedPlaneShape.cpp
	LinearSolveAndInverse.cpp
	Matrix.cpp
	MlcpGaussSeidelSolver.cpp
	PlaneShape.cpp
	SphereShape.cpp
)

set(SURGSIM_MATH_HEADERS
	BoxShape.h
	CapsuleShape.h
	CylinderShape.h
	DoubleSidedPlaneShape.h
	Geometry.h
	LinearSolveAndInverse.h
	Matrix.h
	MeshShape.h
	MeshShape-inl.h
	MlcpConstraintType.h
	MlcpConstraintTypeName.h
	MlcpGaussSeidelSolver.h
	MlcpProblem.h
	MlcpSolution.h
	MlcpSolver.h
	OdeEquation.h
	OdeEquation-inl.h
	OdeSolver.h
	OdeSolver-inl.h
	OdeSolverEulerExplicit.h
	OdeSolverEulerExplicit-inl.h
	OdeSolverEulerExplicitModified.h
	OdeSolverEulerExplicitModified-inl.h
	OdeSolverEulerImplicit.h
	OdeSolverEulerImplicit-inl.h
	PlaneShape.h
	Quaternion.h
	RigidTransform.h
	Shape.h
	Shapes.h
	SphereShape.h
	Valid.h
	Valid-inl.h
	Vector.h
)

surgsim_add_library(
	SurgSimMath
	"${SURGSIM_MATH_SOURCES}"
	"${SURGSIM_MATH_HEADERS}"
	"SurgSim/Math"
)

if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
endif()
