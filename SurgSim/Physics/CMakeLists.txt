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


set(SURGSIM_PHYSICS_SOURCES
	BuildMlcp.cpp
	Computation.cpp
	Constraint.cpp
	ConstraintComponent.cpp
	ConstraintData.cpp
	ConstraintImplementation.cpp
	ContactConstraintGeneration.cpp
	DcdCollision.cpp
	DeformableCollisionRepresentation.cpp
	DeformableRepresentation.cpp
	Fem1DElementBeam.cpp
	Fem1DPlyReaderDelegate.cpp
	Fem1DRepresentation.cpp
	Fem1DRepresentationLocalization.cpp
	Fem2DElementTriangle.cpp
	Fem2DPlyReaderDelegate.cpp
	Fem2DRepresentation.cpp
	Fem2DRepresentationLocalization.cpp
	Fem3DElementCorotationalTetrahedron.cpp
	Fem3DElementCube.cpp
	Fem3DElementTetrahedron.cpp
	Fem3DPlyReaderDelegate.cpp
	Fem3DRepresentation.cpp
	Fem3DRepresentationBilateral3D.cpp
	Fem3DRepresentationContact.cpp
	Fem3DRepresentationLocalization.cpp
	FemElement.cpp
	FemPlyReaderDelegate.cpp
	FemRepresentation.cpp
	FixedRepresentation.cpp
	FixedRepresentationBilateral3D.cpp
	FixedRepresentationContact.cpp
	FreeMotion.cpp
	LinearSpring.cpp
	Localization.cpp
	MassSpringRepresentation.cpp
	MassSpringRepresentationContact.cpp
	MassSpringRepresentationLocalization.cpp
	MlcpPhysicsProblem.cpp
	PhysicsConvert.cpp
	PhysicsManager.cpp
	PhysicsManagerState.cpp
	PostUpdate.cpp
	PreUpdate.cpp
	PushResults.cpp
	Representation.cpp
	RigidCollisionRepresentation.cpp
	RigidRepresentation.cpp
	RigidRepresentationBase.cpp
	RigidRepresentationBaseLocalization.cpp
	RigidRepresentationBilateral3D.cpp
	RigidRepresentationContact.cpp
	RigidRepresentationState.cpp
	SolveMlcp.cpp
	Spring.cpp
	UpdateCollisionRepresentations.cpp
	VirtualToolCoupler.cpp
)

set(SURGSIM_PHYSICS_HEADERS
	BuildMlcp.h
	Computation.h
	Constraint.h
	ConstraintComponent.h
	ConstraintData.h
	ConstraintImplementation.h
	ContactConstraintData.h
	ContactConstraintGeneration.h
	DcdCollision.h
	DeformableCollisionRepresentation.h
	DeformableRepresentation.h
	Fem1DElementBeam.h
	Fem1DPlyReaderDelegate.h
	Fem1DRepresentation.h
	Fem1DRepresentationLocalization.h
	Fem2DElementTriangle.h
	Fem2DPlyReaderDelegate.h
	Fem2DRepresentation.h
	Fem2DRepresentationLocalization.h
	Fem3DElementCorotationalTetrahedron.h
	Fem3DElementCube.h
	Fem3DElementTetrahedron.h
	Fem3DPlyReaderDelegate.h
	Fem3DRepresentation.h
	Fem3DRepresentationBilateral3D.h
	Fem3DRepresentationContact.h
	Fem3DRepresentationLocalization.h
	FemElement.h
	FemPlyReaderDelegate.h
	FemRepresentation.h
	FixedRepresentation.h
	FixedRepresentationBilateral3D.h
	FixedRepresentationContact.h
	FreeMotion.h
	LinearSpring.h
	Localization.h
	Mass.h
	MassSpringRepresentation.h
	MassSpringRepresentationContact.h
	MassSpringRepresentationLocalization.h
	MlcpMapping.h
	MlcpPhysicsProblem.h
	MlcpPhysicsProblem-inl.h
	MlcpPhysicsSolution.h
	PhysicsConvert.h
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
	RigidRepresentationBaseLocalization.h
	RigidRepresentationBilateral3D.h
	RigidRepresentationContact.h
	RigidRepresentationState.h
	SolveMlcp.h
	Spring.h
	UpdateCollisionRepresentations.h
	VirtualToolCoupler.h
)
surgsim_create_library_header(Physics.h "${SURGSIM_PHYSICS_HEADERS}")

surgsim_add_library(
	SurgSimPhysics
	"${SURGSIM_PHYSICS_SOURCES}"
	"${SURGSIM_PHYSICS_HEADERS}"
)

SET(LIBS
	SurgSimCollision
	SurgSimInput
)

target_link_libraries(SurgSimPhysics ${LIBS})

if(BUILD_TESTING)
	add_subdirectory(UnitTests)

	if(BUILD_RENDER_TESTING)
		add_subdirectory(RenderTests)
	endif()

	if(BUILD_PERFORMANCE_TESTING)
		add_subdirectory(PerformanceTests)
	endif()
endif()

set_target_properties(SurgSimPhysics PROPERTIES FOLDER "Physics")
