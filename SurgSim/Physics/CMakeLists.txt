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
	ConstraintImplementationFactory.cpp
	ContactConstraintGeneration.cpp
	DcdCollision.cpp
	DeformableCollisionRepresentation.cpp
	DeformableRepresentation.cpp
	Fem1DElementBeam.cpp
	Fem1DLocalization.cpp
	Fem1DPlyReaderDelegate.cpp
	Fem1DRepresentation.cpp
	Fem2DElementTriangle.cpp
	Fem2DLocalization.cpp
	Fem2DPlyReaderDelegate.cpp
	Fem2DRepresentation.cpp
	Fem3DConstraintBilateral3D.cpp
	Fem3DConstraintContact.cpp
	Fem3DElementCorotationalTetrahedron.cpp
	Fem3DElementCube.cpp
	Fem3DElementTetrahedron.cpp
	Fem3DLocalization.cpp
	Fem3DPlyReaderDelegate.cpp
	Fem3DRepresentation.cpp
	FemElement.cpp
	FemPlyReaderDelegate.cpp
	FemRepresentation.cpp
	FixedConstraintBilateral3D.cpp
	FixedConstraintContact.cpp
	FixedRepresentation.cpp
	FreeMotion.cpp
	LinearSpring.cpp
	Localization.cpp
	MassSpringConstraintContact.cpp
	MassSpringLocalization.cpp
	MassSpringRepresentation.cpp
	MlcpPhysicsProblem.cpp
	PhysicsConvert.cpp
	PhysicsManager.cpp
	PhysicsManagerState.cpp
	PostUpdate.cpp
	PreUpdate.cpp
	PushResults.cpp
	Representation.cpp
	RigidCollisionRepresentation.cpp
	RigidConstraintBilateral3D.cpp
	RigidConstraintContact.cpp
	RigidLocalization.cpp
	RigidRepresentation.cpp
	RigidRepresentationBase.cpp
	RigidState.cpp
	SolveMlcp.cpp
	Spring.cpp
	UpdateCollisions.cpp
	VirtualToolCoupler.cpp
)

set(SURGSIM_PHYSICS_HEADERS
	BuildMlcp.h
	Computation.h
	Constraint.h
	ConstraintComponent.h
	ConstraintData.h
	ConstraintImplementation.h
	ConstraintImplementationFactory.h
	ContactConstraintData.h
	ContactConstraintGeneration.h
	DcdCollision.h
	DeformableCollisionRepresentation.h
	DeformableRepresentation.h
	Fem1DElementBeam.h
	Fem1DLocalization.h
	Fem1DPlyReaderDelegate.h
	Fem1DRepresentation.h
	Fem2DElementTriangle.h
	Fem2DLocalization.h
	Fem2DPlyReaderDelegate.h
	Fem2DRepresentation.h
	Fem3DConstraintBilateral3D.h
	Fem3DConstraintContact.h
	Fem3DElementCorotationalTetrahedron.h
	Fem3DElementCube.h
	Fem3DElementTetrahedron.h
	Fem3DLocalization.h
	Fem3DPlyReaderDelegate.h
	Fem3DRepresentation.h
	FemElement.h
	FemPlyReaderDelegate.h
	FemRepresentation.h
	FixedConstraintBilateral3D.h
	FixedConstraintContact.h
	FixedRepresentation.h
	FreeMotion.h
	LinearSpring.h
	Localization.h
	Mass.h
	MassSpringConstraintContact.h
	MassSpringLocalization.h
	MassSpringRepresentation.h
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
	RigidConstraintBilateral3D.h
	RigidConstraintContact.h
	RigidLocalization.h
	RigidRepresentation.h
	RigidRepresentationBase.h
	RigidRepresentationBase-inl.h
	RigidState.h
	SolveMlcp.h
	Spring.h
	UpdateCollisions.h
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
	SurgSimDataStructures
	SurgSimFramework
	SurgSimInput
	SurgSimMath
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
