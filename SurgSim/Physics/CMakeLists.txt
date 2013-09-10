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

set(SURGSIM_PHYSICS_SOURCES
	BuildMlcp.cpp
	Computation.cpp
	Constraint.cpp
	ConstraintData.cpp
	ConstraintImplementation.cpp
	ConstraintImplementationFactory.cpp
	ContactConstraintGeneration.cpp
	DcdCollision.cpp
	DeformableRepresentation.cpp
	DeformableRepresentationState.cpp
	FemRepresentationParameters.cpp
	FixedRepresentationContact.cpp
	FreeMotion.cpp
	Localization.cpp
	PhysicsManager.cpp
	PhysicsManagerState.cpp
	PostUpdate.cpp
	PreUpdate.cpp
	PushResults.cpp
	Representation.cpp
	RigidRepresentation.cpp
	RigidRepresentationBase.cpp
	RigidRepresentationContact.cpp
	RigidRepresentationLocalization.cpp
	SolveMlcp.cpp
	VtcRigidRepresentation.cpp
)

set(SURGSIM_PHYSICS_HEADERS
	BoxShape.h
	BuildMlcp.h
	CapsuleShape.h
	Computation.h
	Constraint.h
	ConstraintData.h
	ConstraintImplementation.h
	ConstraintImplementationFactory.h
	ContactConstraintData.h
	ContactConstraintGeneration.h
	CylinderShape.h
	DcdCollision.h
	DeformableRepresentation.h
	DeformableRepresentationState.h
	DoubleSidedPlaneShape.h
	FemRepresentationParameters.h
	FixedRepresentation.h
	FixedRepresentationContact.h
	FixedRepresentationLocalization.h
	FreeMotion.h
	Localization.h
	MeshShape.h
	MlcpMapping.h
	MlcpPhysicsProblem.h
	MlcpPhysicsSolution.h
	PhysicsManager.h
	PhysicsManagerState.h
	PlaneShape.h
	PostUpdate.h
	PreUpdate.h
	PushResults.h
	Representation.h
	RigidRepresentation.h
	RigidRepresentationBase.h
	RigidRepresentationBase-inl.h
	RigidRepresentationBaseState.h
	RigidRepresentationContact.h
	RigidRepresentationLocalization.h
	RigidRepresentationParameters.h
	RigidRepresentationState.h
	RigidShape.h
	Shapes.h
	SolveMlcp.h
	SphereShape.h
	VtcRigidParameters.h
	VtcRigidRepresentation.h
)

surgsim_add_library(
	SurgSimPhysics
	"${SURGSIM_PHYSICS_SOURCES}"
	"${SURGSIM_PHYSICS_HEADERS}"
	"SurgSim/Physics"
)
target_link_libraries(SurgSimPhysics SurgSimCollision SurgSimFramework SurgSimMath)

if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
endif()
