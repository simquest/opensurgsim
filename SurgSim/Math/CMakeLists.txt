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
	${YAML_CPP_INCLUDE_DIR}
)

set(SURGSIM_MATH_SOURCES
	BoxShape.cpp
	CapsuleShape.cpp
	CylinderShape.cpp
	DoubleSidedPlaneShape.cpp
	GaussLegendreQuadrature.cpp
	LinearSolveAndInverse.cpp
	MathConvert.cpp
	Matrix.cpp
	MeshShape.cpp
	MlcpGaussSeidelSolver.cpp
	MlcpProblem.cpp
	OctreeShape.cpp
	PlaneShape.cpp
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
	LinearSolveAndInverse.h
	MathConvert.h
	MathConvert-inl.h
	Matrix.h
	MeshShape.h
	MeshShape-inl.h
	MlcpConstraintType.h
	MlcpConstraintTypeName.h
	MlcpGaussSeidelSolver.h
	MlcpProblem.h
	MlcpSolution.h
	MlcpSolver.h
	OctreeShape.h
	OctreeShape-inl.h
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
	OdeSolverLinearEulerExplicit.h
	OdeSolverLinearEulerExplicit-inl.h
	OdeSolverLinearEulerExplicitModified.h
	OdeSolverLinearEulerExplicitModified-inl.h
	OdeSolverLinearEulerImplicit.h
	OdeSolverLinearEulerImplicit-inl.h
	OdeSolverLinearStatic.h
	OdeSolverLinearStatic-inl.h
	OdeSolverStatic.h
	OdeSolverStatic-inl.h
	PlaneShape.h
	Quaternion.h
	RigidTransform.h
	Shape.h
	Shapes.h
	SphereShape.h
	SurfaceMeshShape.h
	SurfaceMeshShape-inl.h
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

set(LIBS 
	SurgSimDataStructures
	SurgSimFramework 
	SurgSimTesting
	${YAML_CPP_LIBRARIES}
)

target_link_libraries(SurgSimMath ${LIBS}
)


if(BUILD_TESTING)
	add_subdirectory(UnitTests)
endif()

# Put SurgSimMath into folder "Math"
set_target_properties(SurgSimMath PROPERTIES FOLDER "Math")
