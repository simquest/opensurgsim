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

set(SURGSIM_PHYSICS_SOURCES
	BuildMlcp.cpp
	Computation.cpp
	Constraint.cpp
	ConstraintData.cpp
	ConstraintImplementation.cpp
	ConstraintImplementationFactory.cpp
	ContactConstraintGeneration.cpp
	DcdCollision.cpp
	DeformableRepresentationState.cpp
	Fem3DRepresentation.cpp
	Fem3DRepresentationLocalization.cpp
	FemElement.cpp
	FemElement3DCube.cpp
	FemElement3DTetrahedron.cpp
	FemRepresentationCoordinate.cpp
	FemRepresentationParameters.cpp
	FixedRepresentationContact.cpp
	FreeMotion.cpp
	LinearSpring.cpp
	Localization.cpp
	MassSpringRepresentation.cpp
	MassSpringRepresentationContact.cpp
	MassSpringRepresentationLocalization.cpp
	MlcpPhysicsProblem.cpp
	PhysicsManager.cpp
	PhysicsManagerState.cpp
	PostUpdate.cpp
	PreUpdate.cpp
	PushResults.cpp
	Representation.cpp
	RigidCollisionRepresentation.cpp
	RigidRepresentation.cpp
	RigidRepresentationBase.cpp
	RigidRepresentationContact.cpp
	RigidRepresentationLocalization.cpp
	SolveMlcp.cpp
	VirtualToolCoupler.cpp
)

set(SURGSIM_PHYSICS_HEADERS
	BuildMlcp.h
	Computation.h
	Constraint.h
	ConstraintData.h
	ConstraintImplementation.h
	ConstraintImplementationFactory.h
	ContactConstraintData.h
	ContactConstraintGeneration.h
	DcdCollision.h
	DeformableRepresentation.h
	DeformableRepresentation-inl.h
	DeformableRepresentationState.h
	Fem3DRepresentation.h
	Fem3DRepresentationLocalization.h
	FemElement.h
	FemElement3DCube.h
	FemElement3DTetrahedron.h
	FemRepresentation.h
	FemRepresentation-inl.h
	FemRepresentationCoordinate.h
	FemRepresentationParameters.h
	FixedRepresentation.h
	FixedRepresentationContact.h
	FixedRepresentationLocalization.h
	FreeMotion.h
	LinearSpring.h
	Localization.h
	Mass.h
	MassSpringRepresentation.h
	MassSpringRepresentationContact.h
	MassSpringRepresentationLocalization.h
	MlcpMapping.h
	MlcpPhysicsProblem.h
	MlcpPhysicsSolution.h
	PhysicsManager.h
	PhysicsManagerState.h
	PostUpdate.h
	PreUpdate.h
	PushResults.h
	Representation.h
	RigidCollisionRepresentation.h
	RigidRepresentation.h
	RigidRepresentationBase.h
	RigidRepresentationBase-inl.h
	RigidRepresentationBaseState.h
	RigidRepresentationContact.h
	RigidRepresentationLocalization.h
	RigidRepresentationParameters.h
	RigidRepresentationState.h
	SolveMlcp.h
	Spring.h
	VirtualToolCoupler.h
)

surgsim_add_library(
	SurgSimPhysics
	"${SURGSIM_PHYSICS_SOURCES}"
	"${SURGSIM_PHYSICS_HEADERS}"
	"SurgSim/Physics"
)

SET(LIBS
	SurgSimCollision
)

target_link_libraries(SurgSimPhysics ${LIBS})

if(BUILD_TESTING)
	add_subdirectory(UnitTests)

	if(BUILD_RENDER_TESTING)
		add_subdirectory(RenderTests)
	endif()
endif()

set_target_properties(SurgSimPhysics PROPERTIES FOLDER "Physics")
