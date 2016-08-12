# This file is a part of the OpenSurgSim project.
# Copyright 2012-2016, SimQuest Solutions Inc.
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
	CcdCollision.cpp
	CcdCollisionLoop.cpp
	Computation.cpp
	ComputationGroup.cpp
	Constraint.cpp
	ConstraintComponent.cpp
	ConstraintData.cpp
	ConstraintImplementation.cpp
	ConstraintImplementationFactory.cpp
	ContactConstraintGeneration.cpp
	ContactFiltering.cpp
	DcdCollision.cpp
	DeformableCollisionRepresentation.cpp
	DeformableRepresentation.cpp
	Fem1D.cpp
	Fem1DElementBeam.cpp
	Fem1DLocalization.cpp
	Fem1DPlyReaderDelegate.cpp
	Fem1DRepresentation.cpp
	Fem2D.cpp
	Fem2DElementTriangle.cpp
	Fem2DLocalization.cpp
	Fem2DPlyReaderDelegate.cpp
	Fem2DRepresentation.cpp
	Fem3D.cpp
	Fem3DElementCorotationalTetrahedron.cpp
	Fem3DElementCube.cpp
	Fem3DElementTetrahedron.cpp
	Fem3DLocalization.cpp
	Fem3DPlyReaderDelegate.cpp
	Fem3DRepresentation.cpp
	FemConstraintFixedPoint.cpp
	FemConstraintFixedRotationVector.cpp
	FemConstraintFrictionalSliding.cpp
	FemConstraintFrictionlessContact.cpp
	FemConstraintFrictionlessSliding.cpp
	FemElement.cpp
	FemLocalization.cpp
	FemPlyReaderDelegate.cpp
	FemRepresentation.cpp
	FixedConstraintFixedPoint.cpp
	FixedConstraintFixedRotationVector.cpp
	FixedConstraintFrictionlessContact.cpp
	FixedRepresentation.cpp
	FreeMotion.cpp
	LinearSpring.cpp
	Localization.cpp
	MassSpringConstraintFixedPoint.cpp
	MassSpringConstraintFrictionlessContact.cpp
	MassSpringLocalization.cpp
	MassSpringRepresentation.cpp
	MlcpPhysicsProblem.cpp
	ParticleCollisionResponse.cpp
	PhysicsConvert.cpp
	PhysicsManager.cpp
	PhysicsManagerState.cpp
	PostUpdate.cpp
	PrepareCollisionPairs.cpp
	PreUpdate.cpp
	PushResults.cpp
	Representation.cpp
	RigidCollisionRepresentation.cpp
	RigidConstraintFixedPoint.cpp
	RigidConstraintFixedRotationVector.cpp
	RigidConstraintFrictionlessContact.cpp
	RigidLocalization.cpp
	RigidRepresentation.cpp
	RigidRepresentationBase.cpp
	RigidState.cpp
	RotationVectorConstraint.cpp
	SlidingConstraint.cpp
	SlidingConstraintData.cpp
	SolveMlcp.cpp
	Spring.cpp
	UpdateCcdData.cpp
	UpdateCollisionData.cpp
	UpdateCollisionRepresentations.cpp
	UpdateDcdData.cpp
	VirtualToolCoupler.cpp
)

set(SURGSIM_PHYSICS_HEADERS
	BuildMlcp.h
	CcdCollision.h
	CcdCollisionLoop.h
	Computation.h
	ComputationGroup.h
	Constraint.h
	ConstraintComponent.h
	ConstraintData.h
	ConstraintImplementation.h
	ConstraintImplementationFactory.h
	ConstraintType.h
	ContactConstraintData.h
	ContactConstraintGeneration.h
	ContactFiltering.h
	DcdCollision.h
	DeformableCollisionRepresentation.h
	DeformableRepresentation.h
	Fem.h
	Fem-inl.h
	Fem1D.h
	Fem1DElementBeam.h
	Fem1DLocalization.h
	Fem1DPlyReaderDelegate.h
	Fem1DRepresentation.h
	Fem2D.h
	Fem2DElementTriangle.h
	Fem2DLocalization.h
	Fem2DPlyReaderDelegate.h
	Fem2DRepresentation.h
	Fem3D.h
	Fem3DElementCorotationalTetrahedron.h
	Fem3DElementCube.h
	Fem3DElementTetrahedron.h
	Fem3DLocalization.h
	Fem3DPlyReaderDelegate.h
	Fem3DRepresentation.h
	FemConstraintFixedPoint.h
	FemConstraintFixedRotationVector.h
	FemConstraintFrictionalSliding.h
	FemConstraintFrictionlessContact.h
	FemConstraintFrictionlessSliding.h
	FemElement.h
	FemElement-inl.h
	FemElementStructs.h
	FemLocalization.h
	FemPlyReaderDelegate.h
	FemRepresentation.h
	FixedConstraintFixedPoint.h
	FixedConstraintFixedRotationVector.h
	FixedConstraintFrictionlessContact.h
	FixedRepresentation.h
	FreeMotion.h
	LinearSpring.h
	Localization.h
	Mass.h
	MassSpringConstraintFixedPoint.h
	MassSpringConstraintFrictionlessContact.h
	MassSpringLocalization.h
	MassSpringRepresentation.h
	MlcpMapping.h
	MlcpPhysicsProblem.h
	MlcpPhysicsSolution.h
	ParticleCollisionResponse.h
	PhysicsConvert.h
	PhysicsManager.h
	PhysicsManagerState.h
	PostUpdate.h
	PrepareCollisionPairs.h
	PreUpdate.h
	PushResults.h
	Representation.h
	RigidCollisionRepresentation.h
	RigidConstraintFixedPoint.h
	RigidConstraintFixedRotationVector.h
	RigidConstraintFrictionlessContact.h
	RigidLocalization.h
	RigidRepresentation.h
	RigidRepresentationBase.h
	RigidRepresentationBase-inl.h
	RigidState.h
	RotationVectorConstraint.h
	RotationVectorConstraintData.h
	SlidingConstraint.h
	SlidingConstraintData.h
	SolveMlcp.h
	Spring.h
	UpdateCcdData.h
	UpdateCollisionData.h
	UpdateCollisionRepresentations.h
	UpdateDcdData.h
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
	SurgSimParticles
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
